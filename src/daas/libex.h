/******************************************************************************
 * L I B E X c h a n g e 
 * 
 * Rel.0.0.17 - 15/11/21 - NO MEASUREMENT !
 * Rel.0.0.16 - 14/11/21 - MRF24ME OK !
 * Rel 0.0.15 - 03/09/21
 ******************************************************************************
 * 
 */

#ifndef LIBEX_H
#define LIBEX_H

#define LIBEX_VER 0017

typedef enum {
    CNL_NONE = 0x00,
    CNL_WIRED,          // Serial AT commands / USB OTG Device / others
    CNL_WIRELESS        // Radio Transceiver SPI/I2C/Serial
} channel_t;


typedef enum {
    RTCMD_NONE = 0, // Exit
    RTCMD_RESET = 10, // Reset
    RTCMD_SYNC_DATETIME = 20,
    RTCMD_START_MEASUREMENT = 30,
    RTCMD_TEST = 99,
} RealTimeCommandType;


typedef enum {
    MODE_SLEEP = 0x01,
    MODE_REALTIME,
    MODE_RESET,
} ExchangeModeType;


#define EX_OK          0 // - Connected
#define EX_ERR_CHANNEL 1 // - channel unavailable 
#define EX_ERR_PARAMS  2//  - parameters unavailable
#define EX_ERR_SKEY    3 // - identity unknow (skey)


/* Function
 * -------------------------------------------------------------------------- */
void Exchange_Initialize(uint32_t DIN, uint32_t lpeer, uint16_t hwver);
uint8_t Exchange_Connect(channel_t cnl, uint32_t rpeer, uint32_t skey, uint8_t exmode);
void Exchange_Disconnect(); // channel_t cnl);
inline bool Exchange_isConnected();

uint8_t Exchange_commandsHandler(RealTimeCommandType *rtCommand);
bool Exchange_SendResponse(uint16_t dataSize);

/* Runtime configuration
 * -------------------------------------------------------------------------- */
void Exchange_SetDriverPtr(channel_t ch, void * exdrv, uint16_t hwver);

/* Call-backs
 * -------------------------------------------------------------------------- */

// Syncronize date-time-timezone
bool __attribute__((weak)) cb_GetDateTime(uint32_t *ltime, uint16_t *tzone);
void __attribute__((weak)) cb_SetDateTime(uint8_t *rxData); // set current time and timezone ( size fix 6 Byte )

// Get device hardware status & resets
uint8_t __attribute__((weak)) cb_GetDeviceState(uint8_t *dobj);
void __attribute__((weak)) cb_PerformReset(uint8_t rtype); // Reset / Reboot !!!!! 

// Manage device configuration ( Align )
bool __attribute__((weak)) cb_GetDeviceConfigCRC16(uint16_t *crc16);
void __attribute__((weak)) cb_SetDeviceConfig(uint8_t *dobj);
uint8_t __attribute__((weak)) cb_GetDeviceConfig(uint8_t *dobj);

// Generic Transfer
// Measurements Transfer
bool __attribute__((weak)) cb_GetMeasurementCounter(uint16_t *nobj);
bool __attribute__((weak)) cb_GetMeasurement(uint16_t index, uint32_t * dtime, uint16_t * tset, uint16_t * ns, uint16_t * nss);
void __attribute__((weak)) cb_GetMeasurementBlock(uint8_t *pbuf, uint16_t offset, uint16_t size);
void __attribute__((weak)) cb_DeleteMeasurement(uint16_t index);

// Lock/ Unlock !!!!!
void __attribute__((weak)) cb_SetSecurityKey(uint32_t old_skey, uint32_t new_skey);
//bool __attribute__((weak)) cb_CheckSecurityKey(uint32_t skey);


#endif // EXCHANGE_H
