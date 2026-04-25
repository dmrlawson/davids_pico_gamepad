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
// a tid byte followed by two independent 4-byte motor samples (left, right),
// each encoding an HF freq+amp pair and an LF freq+amp pair.

#include "8bitdo_n64_mk.h"

#include <string.h>
#include "btstack.h"
#include "classic/hid_host.h"

// true  — NSO full input mode (0x30): 100Hz polling, calibrated 12-bit sticks
// false — NSO simple HID mode (0x3F): 60Hz polling, 8-bit sticks, less MCU work
//
// BlueRetro leaves the controller in simple mode (SW_SET_NATIVE_EN is disabled
// in their build), which matches real N64 joystick resolution anyway (~128
// steps) and leaves the controller MCU with more headroom to service rumble.
#define USE_NSO_FULL_MODE false

// HD Rumble frequencies. Values match BlueRetro's proven-working encoding for
// the 8BitDo N64 Mod Kit (main/bluetooth/hidp/sw.h in darthcloud/BlueRetro).
// HF freq is a 9-bit field; LF freq is 7 bits. Left and right motors use
// different HF frequencies by design — both motor slots need to be driven with
// distinct signals for the controller to respond correctly.
static constexpr uint16_t HD_RUMBLE_L_HF_FREQ = 0x060;
static constexpr uint16_t HD_RUMBLE_R_HF_FREQ = 0x028;
static constexpr uint8_t  HD_RUMBLE_LF_FREQ   = 0x70;

static uint8_t  s_counter = 0;
static uint16_t s_cid     = 0;
static btstack_timer_source_t s_handshake_timer;

static void n64mk_send_subcommand(uint8_t subcmd, const uint8_t * data, uint16_t len) {
    static uint8_t msg[64];
    memset(msg, 0, sizeof(msg));
    msg[0] = s_counter++ & 0x0F;
    msg[1] = 0x00; msg[2] = 0x01; msg[3] = 0x40; msg[4] = 0x40; // rumble off
    msg[5] = 0x00; msg[6] = 0x01; msg[7] = 0x40; msg[8] = 0x40; // rumble off (right)
    msg[9] = subcmd;
    if (len > 0 && data != NULL) memcpy(&msg[10], data, len);
    hid_host_send_report(s_cid, 0x01, msg, 10 + len);
}

// Map XInput amplitude (0–255) to the HD Rumble encoded amp (0–0x64).
// Linear with a clamp: the encoded amp field's valid range is 0–100, so the
// old `x >> 1` (range 0–127) overshot at the top and the top half of the
// XInput range collapsed into "out of spec". A sqrt curve was tried but
// compressed the perceivable dynamic range too far. Straight linear scaling
// into the valid range keeps the widest amp spread the controller will accept.
static uint8_t xinput_to_hd_amp(uint8_t x) {
    return (uint8_t)(((uint16_t)x * 100 + 127) / 255);
}

// Encode one 4-byte HD Rumble motor sample in-place at `out`.
// Layout (two 16-bit little-endian fields):
//   byte 0: HF freq[7:0]
//   byte 1: HF freq[8] in bit 0 | HF amp[6:0] in bits 7:1
//   byte 2: LF freq[6:0] in bits 6:0 | LF amp[0] in bit 7
//   byte 3: LF amp[6:1] in bits 5:0 | tbd1=1 in bit 6 | tbd2=0 in bit 7
// Both HF amp and LF amp are set to the same value — the 8BitDo N64 Mod Kit
// needs both populated to reliably activate its single motor. This mirrors
// BlueRetro's sw_fb_from_generic encoding.
static void encode_hd_rumble_motor(uint8_t * out, uint16_t hf_freq, uint8_t amp) {
    out[0] = hf_freq & 0xFF;
    out[1] = ((hf_freq >> 8) & 0x01) | ((amp & 0x7F) << 1);
    out[2] = HD_RUMBLE_LF_FREQ | ((amp & 0x01) << 7);
    out[3] = ((amp >> 1) & 0x3F) | 0x40;
}

static void n64mk_send_rumble_packet(uint16_t cid, uint8_t low, uint8_t high) {
    static uint8_t msg[9];
    msg[0] = s_counter++ & 0x0F;
    if (low == 0 && high == 0) {
        uint8_t off[] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
        memcpy(&msg[1], off, 8);
    } else {
        // XInput low (large/low-frequency motor) → left slot.
        // XInput high (small/high-frequency motor) → right slot.
        encode_hd_rumble_motor(&msg[1], HD_RUMBLE_L_HF_FREQ, xinput_to_hd_amp(low));
        encode_hd_rumble_motor(&msg[5], HD_RUMBLE_R_HF_FREQ, xinput_to_hd_amp(high));
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
    // Don't enable the IMU (subcmd 0x40). Previous code mislabelled this as
    // "enable HID" but 0x40 is ENABLE_IMU in the NSO protocol, which runs the
    // controller's 6-axis motion sensor at ~800Hz and bundles data into every
    // input report. In simple mode (0x3F) we never see the IMU data anyway,
    // but the controller's MCU keeps sampling and processing it — continuous
    // load we don't need. BlueRetro explicitly omits this step.
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
