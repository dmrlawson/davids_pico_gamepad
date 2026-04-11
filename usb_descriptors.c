// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 David Lawson
//
// USB descriptors that make the Pico appear as an Xbox 360 controller.
// The device presents a vendor-specific (XInput) interface using the
// Microsoft-assigned VID/PID. Windows loads the built-in xusb22.sys
// driver via the XUSB20 compatible ID in the MS OS descriptor.

#include "tusb.h"

#define STR_MANUFACTURER "Microsoft"
#define STR_PRODUCT      "Xbox 360 Controller"
#define STR_SERIAL       "0123456789ABCDEF"

//--------------------------------------------------------------------
// Device Descriptor
//--------------------------------------------------------------------
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF,
    .bDeviceSubClass    = 0xFF,
    .bDeviceProtocol    = 0xFF,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT_SIZE_MAX,

    .idVendor           = 0x045E,
    .idProduct          = 0x028E,
    .bcdDevice          = 0x0114,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------
// Configuration Descriptor
//--------------------------------------------------------------------
#define CONFIG_TOTAL_LEN  (9 + 9 + 7 + 7)

uint8_t const desc_configuration[] =
{
  // Configuration Descriptor (9 bytes)
  // bLength, bDescriptorType, wTotalLength, bNumInterfaces, bConfigurationValue,
  // iConfiguration, bmAttributes (bus-powered + remote wakeup), bMaxPower (500mA)
  0x09, TUSB_DESC_CONFIGURATION, U16_TO_U8S_LE(CONFIG_TOTAL_LEN), 0x01, 0x01, 0x00, 0xA0, 0xFA,

  // Interface Descriptor (9 bytes)
  // bLength, bDescriptorType, bInterfaceNumber, bAlternateSetting, bNumEndpoints,
  // bInterfaceClass (vendor), bInterfaceSubClass (XInput), bInterfaceProtocol (XInput), iInterface
  0x09, TUSB_DESC_INTERFACE, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,

  // Endpoint IN 0x81 — gamepad state reports to host (7 bytes)
  // bLength, bDescriptorType, bEndpointAddress, bmAttributes (interrupt), wMaxPacketSize, bInterval (1ms)
  // wMaxPacketSize must be > 20 (our report size): Linux xpad submits 64-byte URBs and relies on
  // receiving a short packet (actual_length < wMaxPacketSize) to know the transfer is done.
  // With wMaxPacketSize=20, our 20-byte report is a full packet and the URB never completes.
  0x07, TUSB_DESC_ENDPOINT, 0x81, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 0x01,

  // Endpoint OUT 0x01 — rumble commands from host (7 bytes)
  // bLength, bDescriptorType, bEndpointAddress, bmAttributes (interrupt), wMaxPacketSize, bInterval (1ms)
  // wMaxPacketSize must be > the largest rumble packet (8 bytes): TinyUSB arms the endpoint
  // for a multi-packet transfer that only completes on either a full buffer or a short
  // packet (< wMaxPacketSize). With wMaxPacketSize=8, an 8-byte rumble packet is full-size
  // (not short), so packets accumulate in the hardware buffer and never reach the FIFO
  // until a gap or burst brings the total to 64 bytes. 32 matches real Xbox 360 hardware.
  // bInterval 1ms (real Xbox 360 uses 8ms) — shorter host-to-device rumble delivery latency.
  0x07, TUSB_DESC_ENDPOINT, 0x01, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(32), 0x01
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

//--------------------------------------------------------------------
// String Descriptors
//--------------------------------------------------------------------
char const* string_desc_arr [] =
{
  (const char[]) { 0x09, 0x04 },
  STR_MANUFACTURER,
  STR_PRODUCT,
  STR_SERIAL,
};

static uint16_t _desc_str[32];

uint8_t const desc_ms_os_string[] = {
  0x12,
  TUSB_DESC_STRING,
  'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00,
  0x01, // bMS_VendorCode
  0x00
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  uint8_t chr_count;

  if (index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else if (index == 0xEE)
  {
    return (uint16_t const*) (uintptr_t) desc_ms_os_string;
  }
  else
  {
    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;
    const char* str = string_desc_arr[index];
    chr_count = (uint8_t) strlen(str);
    if ( chr_count > 31 ) chr_count = 31;
    for(uint8_t i=0; i<chr_count; i++) _desc_str[1+i] = str[i];
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8 ) | (2*chr_count + 2));
  return _desc_str;
}

//--------------------------------------------------------------------
// Microsoft OS Feature Descriptors
//--------------------------------------------------------------------
uint8_t const desc_ms_os_compat_id[] =
{
    0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 'X', 'U', 'S', 'B', '2', '0', 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t const desc_ms_os_properties[] =
{
    0x8E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x00, 0x01, 0x00,
    0x84, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x28, 0x00,
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00,
    'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00,
    'D', 0x00, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00,
    '{', 0x00, 'F', 0x00, '1', 0x00, '4', 0x00, '5', 0x00, 'A', 0x00,
    '6', 0x00, 'E', 0x00, '7', 0x00, '-', 0x00, '0', 0x00, 'C', 0x00,
    '3', 0x00, '0', 0x00, '-', 0x00, '4', 0x00, '2', 0x00, '4', 0x00,
    '7', 0x00, '-', 0x00, 'A', 0x00, 'E', 0x00, '7', 0x00, '7', 0x00,
    '-', 0x00, '7', 0x00, '3', 0x00, '4', 0x00, '9', 0x00, '8', 0x00,
    '5', 0x00, 'B', 0x00, '5', 0x00, '4', 0x00, 'E', 0x00, '5', 0x00,
    'C', 0x00, '}', 0x00, 0x00, 0x00
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  if (stage != CONTROL_STAGE_SETUP) return true;

  if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == 0x01)
  {
    if (request->wIndex == 0x0004)
    {
      return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_compat_id, sizeof(desc_ms_os_compat_id));
    }
    if (request->wIndex == 0x0005)
    {
      return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_properties, sizeof(desc_ms_os_properties));
    }
    // Linux xpad driver sends a vendor IN request (bRequest=0x01, wIndex=0x00) as a
    // "magic message" during init and expects 20 bytes back. Responding with zeros lets
    // it succeed cleanly. Without this, tud_control_status() skips the data stage and
    // the host times out with ETIMEDOUT, which may prevent input processing from starting.
    if (request->bmRequestType_bit.direction == TUSB_DIR_IN && request->wIndex == 0x0000)
    {
      static uint8_t dummy[20] = {};
      return tud_control_xfer(rhport, request, dummy, sizeof(dummy));
    }
  }
  return false;
}
