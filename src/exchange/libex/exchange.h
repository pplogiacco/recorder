/******************************************************************************
 * E X C H A N G E ( based on library "protocol.a" ver 1.0.0 )
*******************************************************************************/

/******************************************************************************
 PROTOCOL RESOURCES:

    bool openChannel(bool isUsb);
    void closeChannel();

    void sendAck();
    void sendNack();
    bool waitAck(uint16_t timeOut);

    uint8_t *ptrSendData();
    uint8_t *ptrReceiveData();

    bool protocolSend(uint8_t command, uint16_t dataSize, uint16_t timeOut);
    bool protocolReceive(uint8_t *command, uint16_t timeOut);
    void protocolFlush();
  
*******************************************************************************/

/******************************************************************************
 COMMANDS & SERVICES:
 
    // Basic network services
    CMD_NONE = 0x00,    // send
    CMD_HANDSHAKE = 0x01, // send Handshake ( DIN,timestamp,version...
    CMD_KEEP_ALIVE = 0xF1,  

    // Device managment services
    CMD_INIT_DEVICE = 0x90, // receive ( force new device's config ) 
    CMD_SET_DATE_TIME = 0xA1, // Update time/timezone   
    CMD_SET_DEVICE_CONFIG = 0xA2,
    CMD_GET_DEVICE_CONFIG = 0xA3  // force default config if CRC16 not valid
    CMD_GET_HW_STATE = 0xA4,
    CMD_SWITCH_MODE = 0x91, // ???
    CMD_REALTIME_COMMAND = 0x92,

    // Measurements exchange
    CMD_MEASUREMENT_HEADER = 0x81,
    CMD_MEASUREMENT_BLOCK = 0x82,
    CMD_MEASUREMENT_AVAILABLE = 0xB0,
    CMD_MEASUREMENT_RETRIEVE_NEXT = 0xB1,
  
*******************************************************************************/

#include "libex/libex.h"

#ifndef EXCHANGE_H
#define EXCHANGE_H

#define EX_DEVICE       // Enable device managment services (require device.h)
#define EX_MEASUREMENT  // Enable measurements exchange services (measurement.h)
#undef __DONGLE_PASSTHRU

/* 
typedef struct { // Exchange Process:
    uint32_t SKEY; // locked on security-key
    uint16_t vms_mode; // (0) Interrupt mode, (1) Polling mode
    uint16_t attempt_mode; // (0) Try every cycle, (>0) Scheduled every <n> seconds
    uint16_t handshake_timeout; // Timeout on handshake phase
    uint16_t interchar_timeout; // Timeout on data transfer
    uint16_t app_timeout; // Timeout on waiting application commands
    uint16_t retrieve_mode; // (0) Retrieve all datas and delete, (1) Retrieve last meas...
} cnf_exchange_t; // 16 Bytes
#define EXCHANGE_ATTEMPTMODE_EVERYCYCLE  0x00  // Try every sampling cycle
*/
//----------------------------------------------------------------------------//

typedef enum {
    RTCMD_NONE = 0,
    RTCMD_RESET = 10,
    RTCMD_SYNC_DATETIME = 20,
    RTCMD_START_MEASUREMENT = 30,
    RTCMD_TEST = 99,
} RealTimeCommandType;

typedef enum {
    EXCH_OPEN,
    EXCH_START_DISCOVERY,
    EXCH_WAIT_COMMAND,
    EXCH_SEND_COMMAND_RESPONSE,
    EXCH_EXIT,
} exchangestate_t;

//bool Exchange_connect(bool useUsb);
//bool Exchange_disconnect();

#define Exchange_connect(x) openChannel(x)
#define Exchange_disconnect() closeChannel()

bool Exchange_sendHandshake(uint32_t din, uint32_t ltime, uint16_t hfver, uint16_t ccrc);

void Exchange_commandsHandler(RealTimeCommandType *rtCommand);

bool Exchange_commandSendResponse(uint16_t dataSize);

//void Exchange_passthruMainLoop(void);
// uint8_t *ptrSendData();





/* -------------------------------------------------------------------------- */
//devicestate_t exchangeHandler();

#endif // EXCHANGE_H
