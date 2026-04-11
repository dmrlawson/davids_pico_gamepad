#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
  #define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
  #define CFG_TUSB_OS OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
  #define CFG_TUSB_DEBUG 0
#endif

// Device mode
#ifndef CFG_TUD_ENABLED
#define CFG_TUD_ENABLED 1
#endif

// Port mode
#ifndef CFG_TUSB_RHPORT0_MODE
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT_SIZE_MAX
  #define CFG_TUD_ENDPOINT_SIZE_MAX 64
#endif

// Enable Vendor Class
#define CFG_TUD_VENDOR          1

// Vendor FIFO size of TX and RX (must be power of 2, and >= 64 — TinyUSB's stream
// implementation uses TUSB_EPSIZE_BULK_FS=64 as its internal minimum packet size
// regardless of the endpoint's wMaxPacketSize in the USB descriptor)
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#define CFG_TUD_VENDOR_TX_BUFSIZE 64

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
