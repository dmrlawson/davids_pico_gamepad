// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
//
// Bluetooth HID to USB XInput adapter firmware for Raspberry Pi Pico 2 W.
// Receives input from an 8BitDo N64 Mod Kit controller and presents it to
// the host PC as a USB Xbox 360 (XInput) controller.
//
// Architecture: dual-core.
//   Core 0 — BTstack Bluetooth HID host (runs the BTstack event loop)
//   Core 1 — TinyUSB XInput device     (runs the USB task loop)

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/uart.h"

#include "btstack.h"
#include "classic/hid_host.h"
#include "pico/btstack_run_loop_async_context.h"
#include "pico/btstack_cyw43.h"
#include "tusb.h"

#include "btstack_config.h"
#include "driver.h"
#include "8bitdo_n64_mk.h"

//--------------------------------------------------------------------
// Configuration
//--------------------------------------------------------------------
#ifdef NDEBUG
#define DEBUG_ENABLED false
#else
#define DEBUG_ENABLED true
#endif

#define DEBUG_LOG(...) if (DEBUG_ENABLED) printf(__VA_ARGS__)

//--------------------------------------------------------------------
// Cross-core shared state
//
// Core 0 (Bluetooth) writes usb_report and sets usb_report_dirty.
// Core 1 (USB) reads usb_report and clears usb_report_dirty.
//
// On Cortex-M33, byte/halfword/word accesses are naturally atomic.
// `volatile` prevents the compiler from caching values in registers.
// The memcpy of the report struct is a best-effort snapshot — occasional
// tearing is harmless since the USB host polls at 1ms intervals.
//--------------------------------------------------------------------
static XInputReport usb_report;
static volatile bool usb_report_dirty = false;

static volatile uint8_t rumble_low   = 0;
static volatile uint8_t rumble_high  = 0;
static volatile bool    rumble_dirty = false;

//--------------------------------------------------------------------
// Bluetooth state (Core 0 only)
//--------------------------------------------------------------------
static bd_addr_t remote_addr;
static uint16_t hid_host_cid = 0;
static uint16_t remote_vid   = 0;
static uint16_t remote_pid   = 0;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_timer_source_t ui_timer;

static uint8_t hid_descriptor_storage[300];

volatile app_state_t app_state = STATE_IDLE;

//--------------------------------------------------------------------
// Fallback driver
//
// Selected for any unrecognised Bluetooth HID controller. Maps the
// first two axes of the raw HID report to the left stick.
//--------------------------------------------------------------------
static bool generic_process_report(const uint8_t * report, uint16_t len, XInputReport * out) {
    if (len < 2) return false;
    *out = XInputReport{};
    out->report_size = 0x14;
    out->thumb_lx = static_cast<int16_t>((static_cast<int>(report[0]) - 128) * 256);
    out->thumb_ly = static_cast<int16_t>((127 - static_cast<int>(report[1])) * 256);
    return true;
}

static gamepad_driver_t generic_driver   = { 0, 0, "Generic", NULL, generic_process_report, NULL };
static gamepad_driver_t * current_driver = &generic_driver;

//--------------------------------------------------------------------
// Core 0: Bluetooth
//--------------------------------------------------------------------
static void handle_sdp_client_query_result(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(packet_type); UNUSED(channel); UNUSED(size);
    if (hci_event_packet_get_type(packet) == SDP_EVENT_QUERY_ATTRIBUTE_VALUE) {
        uint16_t attribute_id = sdp_event_query_attribute_value_get_attribute_id(packet);
        const uint8_t * value = sdp_event_query_attribute_value_get_attribute_value(packet);
        if (value[0] == 0x09) {
            uint16_t val = (value[1] << 8) | value[2];
            if (attribute_id == 0x0201) remote_vid = val;
            if (attribute_id == 0x0202) remote_pid = val;
        }
    } else if (hci_event_packet_get_type(packet) == SDP_EVENT_QUERY_COMPLETE) {
        // VID 0x057E = Nintendo; 0x0000 = SDP returned no vendor ID (treat as n64mk)
        if (remote_vid == 0x057E || remote_vid == 0x0000) current_driver = &n64mk_driver;
        else current_driver = &generic_driver;
        DEBUG_LOG("[C0] Driver: %s (VID=%04x PID=%04x)\n", current_driver->name, remote_vid, remote_pid);
        if (current_driver->init) current_driver->init(hid_host_cid);
        else app_state = STATE_CONNECTED;
    }
}

static void start_scan(void) {
    DEBUG_LOG("[C0] Inquiry...\n");
    app_state = STATE_SCANNING;
    gap_inquiry_start(8);
}

static void ui_timer_handler(btstack_timer_source_t * ts) {
    static bool led_on = false;
    static uint32_t hb = 0;
    static uint32_t blink_tick = 0;
    if (++hb >= 1000) { hb = 0; DEBUG_LOG("[C0] State: %d\n", (int)app_state); }

    if (app_state == STATE_CONNECTED) {
        led_on = true;
        blink_tick = 0;
    } else if (app_state >= STATE_SCANNING && app_state <= STATE_HANDSHAKE_2) {
        if (++blink_tick >= 50) { blink_tick = 0; led_on = !led_on; } // 50 × 5ms = 250ms
    } else {
        led_on = false;
        blink_tick = 0;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    if (rumble_dirty && app_state == STATE_CONNECTED && hid_host_cid != 0) {
        if (current_driver->send_rumble_packet) {
            current_driver->send_rumble_packet(hid_host_cid, (uint8_t)rumble_low, (uint8_t)rumble_high);
            DEBUG_LOG("[C0] Rumble Sent\n");
            rumble_dirty = false;
        }
    }

    btstack_run_loop_set_timer(ts, 5); // 5ms UI/rumble poll interval
    btstack_run_loop_add_timer(ts);
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t event = hci_event_packet_get_type(packet);
    bd_addr_t addr;
    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) start_scan();
            break;
        case HCI_EVENT_LINK_KEY_REQUEST:
            DEBUG_LOG("[C0] Link Key Req\n");
            hci_event_link_key_request_get_bd_addr(packet, addr);
            hci_send_cmd(&hci_link_key_request_negative_reply, addr);
            break;
        case HCI_EVENT_PIN_CODE_REQUEST:
            DEBUG_LOG("[C0] PIN Req\n");
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            hci_send_cmd(&hci_pin_code_request_negative_reply, addr);
            break;
        case GAP_EVENT_INQUIRY_RESULT:
            gap_event_inquiry_result_get_bd_addr(packet, addr);
            if ((gap_event_inquiry_result_get_class_of_device(packet) & 0x1F00) == 0x0500) {
                if (app_state == STATE_SCANNING) {
                    app_state = STATE_WAIT_INQUIRY_STOP;
                    bd_addr_copy(remote_addr, addr);
                    gap_inquiry_stop();
                }
            }
            break;
        case GAP_EVENT_INQUIRY_COMPLETE:
            if (app_state == STATE_WAIT_INQUIRY_STOP) {
                app_state = STATE_CONNECTING;
                hid_host_connect(remote_addr, HID_PROTOCOL_MODE_REPORT, &hid_host_cid);
            } else if (app_state == STATE_SCANNING) {
                start_scan();
            }
            break;
        case HCI_EVENT_HID_META: {
            switch (hci_event_hid_meta_get_subevent_code(packet)) {
                case HID_SUBEVENT_CONNECTION_OPENED:
                    if (hid_subevent_connection_opened_get_status(packet) == 0) {
                        hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
                        DEBUG_LOG("[C0] Connected\n");
                        app_state = STATE_IDENTIFYING;
                        sdp_client_query_uuid16(&handle_sdp_client_query_result, remote_addr, 0x1200);
                    } else {
                        start_scan();
                    }
                    break;
                case HID_SUBEVENT_CONNECTION_CLOSED:
                    app_state = STATE_IDLE;
                    hid_host_cid = 0;
                    start_scan();
                    break;
                case HID_SUBEVENT_REPORT: {
                    XInputReport report_out;
                    if (current_driver->process_report &&
                        current_driver->process_report(
                            hid_subevent_report_get_report(packet),
                            hid_subevent_report_get_report_len(packet),
                            &report_out)) {
                        memcpy((void*)&usb_report, &report_out, sizeof(XInputReport));
                        usb_report_dirty = true;
                    }
                    break;
                }
            }
            break;
        }
    }
}

//--------------------------------------------------------------------
// Core 1: USB
//--------------------------------------------------------------------
void core1_main() {
    multicore_lockout_victim_init();
    flash_safe_execute_core_init();
    tusb_init();

    uint32_t last_log = to_ms_since_boot(get_absolute_time());
    uint32_t packet_count = 0;

    while (true) {
        tud_task();

        // Read rumble commands from the host (XInput output report: 0x00 0x08 ... low high)
        if (tud_vendor_available()) {
            uint8_t buf[64];
            uint32_t count = tud_vendor_read(buf, sizeof(buf));
            if (count >= 5 && buf[0] == 0x00 && buf[1] == 0x08) {
                DEBUG_LOG("[C1] Rumble RX: %02x %02x\n", buf[3], buf[4]);
                rumble_low   = buf[3];
                rumble_high  = buf[4];
                rumble_dirty = true;
            }
        }

        // Forward the latest gamepad state to the host
        if (tud_ready() && usb_report_dirty) {
            if (tud_vendor_write_available() >= sizeof(XInputReport)) {
                XInputReport report_copy;
                memcpy(&report_copy, (void*)&usb_report, sizeof(XInputReport));
                if (tud_vendor_write(&report_copy, sizeof(XInputReport))) {
                    tud_vendor_flush();
                    usb_report_dirty = false;
                    packet_count++;
                }
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_log > 5000) {
            DEBUG_LOG("[C1] USB Packets: %u\n", (unsigned int)packet_count);
            last_log = now;
        }
    }
}

void tud_mount_cb(void)            { DEBUG_LOG("[C1] USB Mounted\n"); }
void tud_vendor_rx_cb(uint8_t itf) { (void)itf; }

//--------------------------------------------------------------------
// Main (Core 0 entry point)
//--------------------------------------------------------------------
int main() {
#if DEBUG_ENABLED
    stdio_init_all();
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    sleep_ms(500); // Brief settle time for UART
    DEBUG_LOG("\n\n--- Pico 2 W Gateway Boot ---\n");
#endif

    if (cyw43_arch_init()) return -1;
    sleep_ms(500);

    l2cap_init();
    sdp_client_init();
    hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
    hid_host_register_packet_handler(packet_handler);
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    btstack_run_loop_set_timer_handler(&ui_timer, ui_timer_handler);
    btstack_run_loop_set_timer(&ui_timer, 100);
    btstack_run_loop_add_timer(&ui_timer);

    multicore_launch_core1(core1_main);
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute(); // Does not return
    return 0;
}
