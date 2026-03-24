# Pico 2 W Gamepad Adapter

Firmware for the Raspberry Pi Pico 2 W that acts as a Bluetooth-to-USB gamepad adapter. It connects to an **8BitDo N64 Bluetooth controller** (Nintendo Switch Online N64 mod kit) and presents it to the host PC as a **USB Xbox 360 (XInput) controller**. Rumble commands from the PC are forwarded to the controller via HD Rumble.

## Hardware

- Raspberry Pi Pico 2 W
- 8BitDo N64 Bluetooth controller (NSO N64, VID `057E` PID `2019`) — the mode switch on the controller must be set to **S** (Switch)

No additional components are required. UART output on GP0/GP1 (115200 baud) provides debug logging in debug builds.

## How it works

- **Core 0** runs the BTstack Bluetooth Classic HID host. On boot it scans for a nearby HID device with a peripheral class of device, connects, identifies the controller via SDP, and runs the NSO N64 handshake to enable full input mode (100Hz) and vibration.
- **Core 1** runs the TinyUSB device stack. It presents a USB XInput interface to the host PC. Incoming gamepad state is forwarded as XInput reports; incoming rumble commands are forwarded over to Core 0 to send to the controller via HD Rumble.

The LED indicates connection state: off = idle, blinking = scanning/connecting, solid = connected.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) v2.2.0 or later and the VS Code Pico extension (or a manually configured CMake environment).

```bash
mkdir build && cd build
cmake -DPICO_BOARD=pico2_w ..
make -j$(nproc)
```

Flash `davids_pico_gamepad.uf2` to the Pico in BOOTSEL mode.

## Configuration

In `davids_pico_gamepad.cpp`:

```cpp
// true  — NSO full input mode (0x30): 100Hz polling, calibrated stick data
// false — NSO simple HID mode (0x3F): 60Hz polling, lower power
#define USE_NSO_FULL_MODE true
```

## License

Copyright (C) 2026 David Lawson
Licensed under the [GNU General Public License v3.0](LICENSE) or later.
