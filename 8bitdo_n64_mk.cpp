// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
//
// Driver for the 8BitDo N64 Mod Kit controller.
// This controller emulates the Nintendo Switch Online N64 protocol over
// Bluetooth Classic HID (VID 0x057E, PID 0x2019).
//
// After connection, an initialisation handshake enables vibration
// (subcommand 0x48) and sets the input report mode (subcommand 0x03).
// Subcommands are sent as HID output report 0x01; input arrives as:
//   0x3F — simple HID  (60Hz, basic buttons + hat + one stick)
//   0x30 — full mode   (100Hz, all buttons + calibrated stick data)
//
// Rumble is sent via HID output report 0x10 using the HD Rumble format:
// a 4-byte payload encoding two frequency+amplitude pairs, mirrored to
// fill an 8-byte field.

#include "8bitdo_n64_mk.h"

#include <string.h>
#include "btstack.h"
#include "classic/hid_host.h"

// true  — NSO full input mode (0x30): 100Hz polling, calibrated stick data
// false — NSO simple HID mode (0x3F): 60Hz polling, lower power
#define USE_NSO_FULL_MODE true

// HD Rumble HF frequency byte (bits 7:0 of the 9-bit freq field; bit 8 is in msg[2] bit 0)
// 0x28 encodes ~320 Hz, a good default for the small motor.
static constexpr uint8_t HD_RUMBLE_HF_FREQ = 0x28;

static uint8_t  s_counter = 0;
static uint16_t s_cid     = 0;
static btstack_timer_source_t s_handshake_timer;

static void n64mk_send_subcommand(uint8_t subcmd, const uint8_t * data, uint16_t len) {
    uint8_t msg[64];
    memset(msg, 0, sizeof(msg));
    msg[0] = s_counter++ & 0x0F;
    msg[1] = 0x00; msg[2] = 0x01; msg[3] = 0x40; msg[4] = 0x40; // rumble off
    msg[5] = 0x00; msg[6] = 0x01; msg[7] = 0x40; msg[8] = 0x40; // rumble off (right)
    msg[9] = subcmd;
    if (len > 0 && data != NULL) memcpy(&msg[10], data, len);
    hid_host_send_report(s_cid, 0x01, msg, 10 + len);
}

static void n64mk_send_rumble_packet(uint16_t cid, uint8_t low, uint8_t high) {
    uint8_t msg[9];
    msg[0] = s_counter++ & 0x0F;
    if (low == 0 && high == 0) {
        uint8_t off[] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
        memcpy(&msg[1], off, 8);
    } else {
        // Map XInput (0–255) to NSO amplitude (0–127)
        uint8_t hfa = high >> 1; // High-frequency amp (small motor)
        uint8_t lfa = low  >> 1; // Low-frequency amp  (large motor)

        // Encode 4-byte HD Rumble sample (layout from BlueRetro sw.h analysis):
        //   msg[1] = HF freq[7:0]
        //   msg[2] = HF amp[6:0] in bits[7:1], HF freq[8] in bit[0]
        //   msg[3] = LF amp[0]   in bit[7],    LF freq[6:0] in bits[6:0]
        //   msg[4] = tbd1 (must=1) in bit[6],  LF amp[6:1] in bits[5:0]
        msg[1] = HD_RUMBLE_HF_FREQ;                     // HF freq[7:0] = 0x28 (~320 Hz)
        msg[2] = (hfa & 0x7F) << 1;                     // HF amp[6:0]; freq[8]=0
        msg[3] = 0x70 | ((lfa & 0x01) << 7);            // LF freq = 0x70 (~160 Hz); LF amp[0]
        msg[4] = ((lfa >> 1) & 0x3F) | 0x40;            // LF amp[6:1]; tbd1=1

        // Protocol expects two motor slots (8 bytes total); mirror the sample
        memcpy(&msg[5], &msg[1], 4);
    }
    hid_host_send_report(cid, 0x10, msg, sizeof(msg));
}

static void handshake_timer_handler(btstack_timer_source_t * ts) {
    (void)ts;
    if (app_state == STATE_HANDSHAKE_1) {
        uint8_t enable = 0x01;
        n64mk_send_subcommand(0x48, &enable, 1); // Enable vibration
        app_state = STATE_HANDSHAKE_2;
        btstack_run_loop_set_timer(&s_handshake_timer, 100);
        btstack_run_loop_add_timer(&s_handshake_timer);
    } else if (app_state == STATE_HANDSHAKE_2) {
        uint8_t mode = USE_NSO_FULL_MODE ? 0x30 : 0x3F;
        n64mk_send_subcommand(0x03, &mode, 1);   // Set input report mode
        app_state = STATE_CONNECTED;
    }
}

static void n64mk_init(uint16_t cid) {
    s_cid = cid;
    uint8_t enable = 0x01;
    n64mk_send_subcommand(0x40, &enable, 1); // Enable HID (required before other commands)
    app_state = STATE_HANDSHAKE_1;
    btstack_run_loop_set_timer_handler(&s_handshake_timer, handshake_timer_handler);
    btstack_run_loop_set_timer(&s_handshake_timer, 100);
    btstack_run_loop_add_timer(&s_handshake_timer);
}

// Parse a simple HID report (mode 0x3F): basic buttons, hat switch, one analogue stick.
// Report layout (starting at report[2]): buttons1, buttons2, hat, ?, stickX, ?, stickY
static void process_simple_report(const uint8_t * report, uint16_t len, XInputReport & out) {
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
static void process_full_report(const uint8_t * report, uint16_t len, XInputReport & out) {
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

static bool n64mk_process_report(const uint8_t * report, uint16_t len, XInputReport * out) {
    const uint8_t report_id = report[1];

    *out = XInputReport{};
    out->report_size = 0x14;

    if (report_id == 0x3F) {
        process_simple_report(report, len, *out);
    } else if (report_id == 0x30) {
        process_full_report(report, len, *out);
    } else {
        return false;
    }
    return true;
}

gamepad_driver_t n64mk_driver = {
    0x057E, 0x2019, "8BitDo N64 Mod Kit",
    n64mk_init, n64mk_process_report, n64mk_send_rumble_packet
};
