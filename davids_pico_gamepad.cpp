// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
//
// Bluetooth HID to USB XInput adapter firmware for Raspberry Pi Pico 2 W.
// Receives input from an 8BitDo N64 Bluetooth controller (NSO N64 protocol)
// and presents it to the host as a USB Xbox 360 (XInput) controller.
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
#include "xinput_reports.h"

//--------------------------------------------------------------------
// Configuration
//--------------------------------------------------------------------
#ifdef NDEBUG
#define DEBUG_ENABLED false
#else
#define DEBUG_ENABLED true
#endif

#define DEBUG_LOG(...) if (DEBUG_ENABLED) printf(__VA_ARGS__)

// true  — NSO full input mode (0x30): 100Hz polling, calibrated stick data
// false — NSO simple HID mode (0x3F): 60Hz polling, lower power
#define USE_NSO_FULL_MODE true

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
static btstack_timer_source_t handshake_timer;

static uint8_t hid_descriptor_storage[300];

typedef enum {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_WAIT_INQUIRY_STOP,
    STATE_CONNECTING,
    STATE_IDENTIFYING,
    STATE_HANDSHAKE_1,
    STATE_HANDSHAKE_2,
    STATE_CONNECTED
} app_state_t;

static volatile app_state_t app_state = STATE_IDLE;

//--------------------------------------------------------------------
// Driver interface
//--------------------------------------------------------------------
typedef struct {
    uint16_t vid;
    uint16_t pid;
    const char * name;
    void (*init)(uint16_t cid);
    void (*process_report)(const uint8_t * report, uint16_t len);
    void (*send_rumble_packet)(uint16_t cid, uint8_t low, uint8_t high);
} gamepad_driver_t;

//--------------------------------------------------------------------
// NSO N64 driver
//
// The Nintendo Switch Online N64 controller uses a proprietary protocol
// over Bluetooth Classic HID. After connection, an initialisation
// handshake enables vibration (subcommand 0x48) and sets the input
// report mode (subcommand 0x03). Subcommands are sent as HID output
// report 0x01; input arrives as either:
//   0x3F — simple HID  (60Hz, basic buttons + hat + one stick)
//   0x30 — full mode   (100Hz, all buttons + calibrated stick data)
//
// Rumble is sent via HID output report 0x10 using the HD Rumble format:
// a 4-byte payload encoding two frequency+amplitude pairs, mirrored to
// fill an 8-byte field.
//--------------------------------------------------------------------
static uint8_t switch_counter = 0;

// HD Rumble frequency byte values (empirically chosen for good feel)
static constexpr uint8_t HD_RUMBLE_HF_FREQ = 0x28; // ~320 Hz high-frequency component
static constexpr uint8_t HD_RUMBLE_LF_FREQ = 0x80; // ~160 Hz low-frequency component

static void nso_send_subcommand(uint16_t cid, uint8_t subcmd, const uint8_t * data, uint16_t len) {
    uint8_t msg[64];
    memset(msg, 0, sizeof(msg));
    msg[0] = switch_counter++ & 0x0F;
    msg[1] = 0x00; msg[2] = 0x01; msg[3] = 0x40; msg[4] = 0x40; // rumble off
    msg[5] = 0x00; msg[6] = 0x01; msg[7] = 0x40; msg[8] = 0x40; // rumble off (right)
    msg[9] = subcmd;
    if (len > 0 && data != NULL) memcpy(&msg[10], data, len);
    hid_host_send_report(cid, 0x01, msg, 10 + len);
}

static void nso_n64_send_rumble_packet(uint16_t cid, uint8_t low, uint8_t high) {
    uint8_t msg[9];
    msg[0] = switch_counter++ & 0x0F;
    if (low == 0 && high == 0) {
        uint8_t off[] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
        memcpy(&msg[1], off, 8);
    } else {
        // Map XInput (0–255) to NSO amplitude (0–127)
        uint8_t hfa = high >> 1; // High-frequency amp (small motor)
        uint8_t lfa = low  >> 1; // Low-frequency amp  (large motor)

        // Encode 4-byte HD Rumble sample
        msg[1] = HD_RUMBLE_HF_FREQ;
        msg[2] = 0x80 | (hfa & 0x7F); // HF amp; top bit marks the frequency range
        msg[3] = HD_RUMBLE_LF_FREQ;
        msg[4] = lfa & 0x7F;           // LF amp

        // Protocol expects two motor slots (8 bytes total); mirror the sample
        memcpy(&msg[5], &msg[1], 4);
    }
    hid_host_send_report(cid, 0x10, msg, sizeof(msg));
}

static void handshake_timer_handler(btstack_timer_source_t * ts) {
    (void)ts;
    if (app_state == STATE_HANDSHAKE_1) {
        uint8_t enable = 0x01;
        nso_send_subcommand(hid_host_cid, 0x48, &enable, 1); // Enable vibration
        app_state = STATE_HANDSHAKE_2;
        btstack_run_loop_set_timer(&handshake_timer, 200);
        btstack_run_loop_add_timer(&handshake_timer);
    } else if (app_state == STATE_HANDSHAKE_2) {
        uint8_t mode = USE_NSO_FULL_MODE ? 0x30 : 0x3F;
        nso_send_subcommand(hid_host_cid, 0x03, &mode, 1);   // Set input report mode
        app_state = STATE_CONNECTED;
        DEBUG_LOG("[C0] NSO Ready (%s)\n", USE_NSO_FULL_MODE ? "Full" : "Simple");
    }
}

static void nso_n64_init(uint16_t cid) {
    uint8_t enable = 0x01;
    nso_send_subcommand(cid, 0x40, &enable, 1); // Enable HID (required before other commands)
    app_state = STATE_HANDSHAKE_1;
    btstack_run_loop_set_timer_handler(&handshake_timer, handshake_timer_handler);
    btstack_run_loop_set_timer(&handshake_timer, 200);
    btstack_run_loop_add_timer(&handshake_timer);
}

// Parse a simple HID report (mode 0x3F): basic buttons, hat switch, one analogue stick.
// Report layout (starting at report[2]): buttons1, buttons2, hat, ?, stickX, ?, stickY
static void nso_n64_process_simple_report(const uint8_t * report, uint16_t len, XInputReport & out) {
    if (len < 10) return;
    const uint8_t b1  = report[2];
    const uint8_t b2  = report[3];
    const uint8_t hat = report[4];

    if (b1 & 0x02) out.buttons |= XInputButton::A;
    if (b1 & 0x01) out.buttons |= XInputButton::B;
    if (b2 & 0x02) out.buttons |= XInputButton::Start;
    if (b1 & 0x10) out.buttons |= XInputButton::LeftShoulder;
    if (b1 & 0x20) out.buttons |= XInputButton::RightShoulder;
    if (b1 & 0x40) out.left_trigger = 255;
    if (b1 & 0x04) out.thumb_ry =  32767; // C-up
    if (b1 & 0x80) out.thumb_ry = -32768; // C-down
    if (b1 & 0x08) out.thumb_rx = -32768; // C-left
    if (b2 & 0x01) out.thumb_rx =  32767; // C-right

    if (hat <= 7) {
        if (hat == 7 || hat == 0 || hat == 1) out.buttons |= XInputButton::DpadUp;
        if (hat == 3 || hat == 4 || hat == 5) out.buttons |= XInputButton::DpadDown;
        if (hat == 5 || hat == 6 || hat == 7) out.buttons |= XInputButton::DpadLeft;
        if (hat == 1 || hat == 2 || hat == 3) out.buttons |= XInputButton::DpadRight;
    }

    out.thumb_lx = static_cast<int16_t>((static_cast<int>(report[6]) - 128) * 256);
    out.thumb_ly = static_cast<int16_t>((127 - static_cast<int>(report[8])) * 256);
}

// Parse a full input report (mode 0x30): all buttons, calibrated 12-bit stick data.
// Byte layout: [0]=timer [1]=conn_info [2]=? [3]=? [4]=r_btns [5]=m_btns [6]=l_btns [7-9]=stick
static void nso_n64_process_full_report(const uint8_t * report, uint16_t len, XInputReport & out) {
    if (len < 13) return;
    const uint8_t r_btns = report[4]; // Right-side buttons (A, B, R, ZR)
    const uint8_t m_btns = report[5]; // Middle buttons (+, -, Home, Capture)
    const uint8_t l_btns = report[6]; // Left-side buttons (L, ZL, D-pad)

    if (r_btns & 0x08) out.buttons |= XInputButton::A;
    if (r_btns & 0x04) out.buttons |= XInputButton::B;
    if (m_btns & 0x02) out.buttons |= XInputButton::Start;

    if (l_btns & 0x40) out.buttons |= XInputButton::LeftShoulder;
    if (r_btns & 0x40) out.buttons |= XInputButton::RightShoulder;
    if (l_btns & 0x80) out.left_trigger = 255;

    if (r_btns & 0x01) out.thumb_ry =  32767; // C-up
    if (r_btns & 0x80) out.thumb_ry = -32768; // C-down
    if (r_btns & 0x02) out.thumb_rx = -32768; // C-left
    if (m_btns & 0x01) out.thumb_rx =  32767; // C-right

    if (m_btns & 0x10) out.buttons |= XInputButton::Start;
    if (m_btns & 0x20) out.buttons |= XInputButton::Back;

    if (l_btns & 0x02) out.buttons |= XInputButton::DpadUp;
    if (l_btns & 0x01) out.buttons |= XInputButton::DpadDown;
    if (l_btns & 0x08) out.buttons |= XInputButton::DpadLeft;
    if (l_btns & 0x04) out.buttons |= XInputButton::DpadRight;

    // Stick axes are packed as two 12-bit values across 3 bytes
    uint16_t lx = report[7] | ((report[8] & 0x0F) << 8);
    uint16_t ly = (report[8] >> 4) | (report[9] << 4);
    out.thumb_lx = static_cast<int16_t>((static_cast<int>(lx) - 2048) * 16);
    out.thumb_ly = static_cast<int16_t>((static_cast<int>(ly) - 2048) * 16);
}

static void nso_n64_process_report(const uint8_t * report, uint16_t len) {
    const uint8_t report_id = report[1];

    XInputReport next_report = {};
    next_report.report_size = 0x14;

    if (report_id == 0x3F) {
        nso_n64_process_simple_report(report, len, next_report);
    } else if (report_id == 0x30) {
        nso_n64_process_full_report(report, len, next_report);
    } else {
        return;
    }

    memcpy((void*)&usb_report, &next_report, sizeof(XInputReport));
    usb_report_dirty = true;
}

static gamepad_driver_t nso_n64_driver = {
    0x057E, 0x2019, "NSO N64",
    nso_n64_init, nso_n64_process_report, nso_n64_send_rumble_packet
};

// Fallback driver for unrecognised controllers: maps the first two axes
// of the raw HID report to the left stick.
static void generic_process_report(const uint8_t * report, uint16_t len) {
    if (len < 6) return;
    XInputReport next_report = {};
    next_report.report_size = 0x14;
    next_report.thumb_lx = static_cast<int16_t>((static_cast<int>(report[0]) - 128) * 256);
    next_report.thumb_ly = static_cast<int16_t>((127 - static_cast<int>(report[1])) * 256);
    memcpy((void*)&usb_report, &next_report, sizeof(XInputReport));
    usb_report_dirty = true;
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
        if (remote_vid == 0x057E || remote_vid == 0x0000) current_driver = &nso_n64_driver;
        else current_driver = &generic_driver;
        DEBUG_LOG("[C0] Driver: %s\n", current_driver->name);
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
    if (++hb >= 1000) { hb = 0; DEBUG_LOG("[C0] State: %d\n", (int)app_state); }

    if (app_state == STATE_CONNECTED)
        led_on = true;
    else if (app_state >= STATE_SCANNING && app_state <= STATE_HANDSHAKE_2)
        led_on = !led_on;
    else
        led_on = false;
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
                case HID_SUBEVENT_REPORT:
                    if (current_driver->process_report)
                        current_driver->process_report(
                            hid_subevent_report_get_report(packet),
                            hid_subevent_report_get_report_len(packet));
                    break;
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
    stdio_init_all();
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    sleep_ms(2000);
    DEBUG_LOG("\n\n--- Pico 2 W Gateway Boot ---\n");

    if (cyw43_arch_init()) return -1;
    sleep_ms(1000);

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
