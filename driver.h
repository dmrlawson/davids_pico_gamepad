// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
#pragma once

#include <cstdint>
#include "xinput_reports.h"

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

// Defined in davids_pico_gamepad.cpp; read/written by drivers during handshake
extern volatile app_state_t app_state;

// A gamepad driver translates raw Bluetooth HID reports into XInput reports
// and optionally sends rumble packets back to the controller.
typedef struct {
    uint16_t vid;
    uint16_t pid;
    const char * name;
    void (*init)(uint16_t cid);
    // Returns true and fills `out` if the report contains usable input data
    bool (*process_report)(const uint8_t * report, uint16_t len, XInputReport * out);
    void (*send_rumble_packet)(uint16_t cid, uint8_t low, uint8_t high);
} gamepad_driver_t;
