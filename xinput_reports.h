// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
#pragma once

#include <cstdint>

// Standard 20-byte XInput gamepad input report.
// report_id and report_size are fixed; the host reads buttons, triggers,
// and stick axes. Reserved bytes must be zero.
struct XInputReport {
    uint8_t  report_id;      // Always 0x00
    uint8_t  report_size;    // Always 0x14 (20 bytes)
    uint16_t buttons;
    uint8_t  left_trigger;
    uint8_t  right_trigger;
    int16_t  thumb_lx;
    int16_t  thumb_ly;
    int16_t  thumb_rx;
    int16_t  thumb_ry;
    uint8_t  reserved[6];
} __attribute__((packed));

static_assert(sizeof(XInputReport) == 20, "XInputReport must be exactly 20 bytes");

// XInput digital button bit masks
namespace XInputButton {
    constexpr uint16_t DpadUp        = 0x0001;
    constexpr uint16_t DpadDown      = 0x0002;
    constexpr uint16_t DpadLeft      = 0x0004;
    constexpr uint16_t DpadRight     = 0x0008;
    constexpr uint16_t Start         = 0x0010;
    constexpr uint16_t Back          = 0x0020;
    constexpr uint16_t LeftThumb     = 0x0040;
    constexpr uint16_t RightThumb    = 0x0080;
    constexpr uint16_t LeftShoulder  = 0x0100;
    constexpr uint16_t RightShoulder = 0x0200;
    constexpr uint16_t A             = 0x1000;
    constexpr uint16_t B             = 0x2000;
    constexpr uint16_t X             = 0x4000;
    constexpr uint16_t Y             = 0x8000;
}
