# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a Raspberry Pi Pico 2 W that presents a Bluetooth HID gamepad to a host PC as a USB XInput (Xbox 360) controller, including rumble. Primary target controller is the 8BitDo N64 Mod Kit (VID `057E` PID `2019`, mode switch on `S`).

There is no test suite. Verification is done by flashing and testing against real hosts (Linux `fftest`, Windows, web Gamepad API).

## Build

```bash
mkdir build && cd build
cmake -DPICO_BOARD=pico2_w ..      # or pico_w; Release builds also work
make -j$(nproc)
```

Flash `build/davids_pico_gamepad.uf2` to the Pico in BOOTSEL mode. Requires Pico SDK ≥ 2.2.0 (`PICO_SDK_PATH` or the Pico VS Code extension's auto-config).

Debug vs release: `DEBUG_LOG` is a no-op when `NDEBUG` is defined (release). UART stdio on GP0/GP1 at 115200 baud carries the debug logs in debug builds only.

The `rumble_test/` directory is a stray CMake build output tree, not a test suite — ignore it.

Do not edit the "DO NOT EDIT" block at the top of `CMakeLists.txt`; it's maintained by the Pico VS Code extension.

## Architecture

**Dual-core, not dual-thread.** Both cores run independent event loops with no OS and no shared locking. They communicate only through a handful of `volatile` cross-core variables.

### Core 0 — Bluetooth (BTstack)
Entry: `main()` → `btstack_run_loop_execute()` (never returns).

- Drives the BTstack event loop and HCI/L2CAP/SDP/HID-Host stack over the CYW43 radio.
- On boot: inquiry → connect to first device advertising a peripheral CoD → SDP query for VID/PID → dispatch to a driver.
- `ui_timer_handler` is a 5ms BTstack timer that drives (a) the on-board LED state indicator, and (b) the rumble send pump that forwards `rumble_low`/`rumble_high` over BT to the controller at ~67Hz.
- HID input reports arriving from the controller are translated by the active driver's `process_report()` into an `XInputReport` written into shared state.

### Core 1 — USB device (TinyUSB)
Entry: `core1_main()` (launched via `multicore_launch_core1`). Tight `while(true)` loop: `tud_task()`, read OUT endpoint (rumble), write IN endpoint (gamepad state).

- Presents as an Xbox 360 wired controller: `idVendor=0x045E idProduct=0x028E`, Microsoft XUSB20 MS-OS compatible ID so Windows binds `xusb22.sys` automatically.
- Incoming XInput OUT packets are parsed by type/size (byte 0 = type, byte 1 = packet size) into `rumble_low`/`rumble_high`.

### Shared state (between cores)

All declared in `davids_pico_gamepad.cpp`:
- `usb_report` + `usb_report_dirty`: Core 0 writes, Core 1 reads and flushes to USB.
- `rumble_low`, `rumble_high`: Core 1 writes as rumble commands arrive, Core 0 reads in the timer and sends at ~67Hz.
- `app_state` (`volatile app_state_t`): connection state machine, read across files.

Atomicity relies on naturally atomic byte/halfword/word accesses on Cortex-M33 + `volatile`. No memory barriers. Transient tearing on multi-byte reads is tolerated because it self-corrects on the next tick.

### Driver abstraction (`driver.h`)

Each supported controller implements a `gamepad_driver_t` with `init()`, `process_report()`, `send_rumble_packet()`. Core 0's SDP handler picks the driver by VID/PID (`057E`/`2019` or VID `0000` falls through to `n64mk_driver`; anything else uses `generic_driver` which is a fallback stub). To add a controller: create `<name>.{cpp,h}`, export a `gamepad_driver_t`, and add both to `CMakeLists.txt` and the VID/PID dispatch in `handle_sdp_client_query_result`.

### 8BitDo N64 Mod Kit protocol (`8bitdo_n64_mk.cpp`)

The controller emulates the Nintendo Switch NSO N64 protocol. Initialisation is a sequenced handshake driven by a BTstack timer: enable HID (subcommand `0x40`) → enable vibration (`0x48`) → set report mode (`0x03`). Input reports are `0x3F` (simple, 60Hz) or `0x30` (full, 100Hz, calibrated 12-bit sticks) depending on `USE_NSO_FULL_MODE`.

Rumble is HID output report `0x10` carrying a 4-byte HD Rumble encoding mirrored into two motor slots. `n64mk_send_rumble_packet`'s `msg[9]` buffer is `static` on purpose — BTstack's `hid_host_send_report` stores a pointer rather than copying, and the actual send happens later when `L2CAP_EVENT_CAN_SEND_NOW` fires. Making it a local would leave a dangling pointer.

### Rumble pipeline gotchas (learned the hard way)

- **Sustain required.** The controller does not latch HD Rumble state; Core 0 must re-send the current rumble values at ~67Hz or the motor stops. Matches the Switch, which sends rumble bundled with input polling at ~60Hz.
- **EP OUT `wMaxPacketSize` must exceed any rumble packet size.** TinyUSB arms the OUT endpoint for a multi-packet transfer that completes only on a full buffer or a short packet. With `wMaxPacketSize = 8`, an 8-byte rumble packet is full-size and does not terminate the transfer — packets accumulate in the hardware buffer and never reach the FIFO. Set to 32 (matches real Xbox 360 hardware) so any XInput OUT command is a short packet.
- **Iterate over concatenated OUT packets.** The TinyUSB RX FIFO is a raw byte stream; multiple OUT transfers may appear in one `tud_vendor_read`. Walk with `buf[i+1]` (size byte) and advance by that size — parsing only the first packet silently drops stops queued behind LED or intensity-change packets.
- **RX FIFO sizing.** `CFG_TUD_VENDOR_RX_BUFSIZE` in `tusb_config.h` sets the FIFO. If you allocate a local buffer of that size in `core1_main`, it will overflow Core 1's default 2KB stack — use `static`.

### Linux xpad magic handshake

`tud_vendor_control_xfer_cb` in `usb_descriptors.c` answers a specific vendor IN request (`bRequest=0x01`, `wIndex=0x0000`) with 20 zero bytes. Without this, Linux times out during enumeration and input reporting can fail to start.
