#include "xc.h"
#include <stdbool.h>
#include "test.h"

#ifndef DEVICE_H
#define	DEVICE_H

//------------------------------------------------------------------------------
#define __DEVICE_DIN    100100102    // HWVAMP1K (Paolo)
#define __DEVICE_VER    001017       // ver.rel
#define __DEVICE_USB_SR 0310         // Usb serial
#define __DEVICE_NET_ID 0xCCCC       // 802. mac-id
#define __DEVICE_PAN_ID 0            // 802. pan-id

#define MRF24J40_DONGLE_ADDRESS 0x8888
//------------------------------------------------------------------------------
#include "dev_hardware.h"
#include "dev_config.h"
//------------------------------------------------------------------------------

//#define Device_IsUsbConnected() USB_Status
#define Device_IsWireLinked() USB_Status

typedef enum { // 
    STARTUP, // Switch-on state
    SAMPLING, // Start sampling cycle
    EXCHANGE, // Initiate exchange mode
    WAITING
} devicestate_t;

typedef struct {
    config_t cnf; // (RW) configuration ( 94 Bytes )
    status_t sts; // (RO) status
    unsigned long SYS_CLOCK;
    //runlevel_t rlevel; // Use: device.st.llevel
} device_t;

#endif