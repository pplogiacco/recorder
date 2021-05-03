/**
 File: exchange.h
 */

#include "protocol.h"

#ifndef EXCHANGE_H
#define EXCHANGE_H

typedef enum {
    MODE_SLEEP = 0x01,
    MODE_REALTIME,
    MODE_RESET,
} ExchangeModeType;

typedef enum {
    CMD_NONE = CMD_NONE_0x00,

    //send
    CMD_HANDSHAKE = CMD_HANDSHAKE_0x01, // Handshake
    // Send: DIN,timestamp,version
    // Ret: ?

    CMD_MEASUREMENT_HEADER = 0x81,
    CMD_MEASUREMENT_BLOCK = 0x82,

    //receive
    CMD_INIT_DEVICE = 0x90, // ??? (  forza il salvataggio della configurazione )
    CMD_SWITCH_MODE = 0x91, // ???
    CMD_REALTIME_COMMAND = 0x92,

    // To do:
    // CMD_DEVICE_RESET     //  restart, Reset alarm counter,
    // CMD_SET_LOCK_SKEY    // Lock/unlock device
    // Rcv: old_skey, new_skey

    CMD_SET_DATE_TIME = 0xA1, // Aggiorna ora time/timezone
    CMD_SET_DEVICE_CONFIG = 0xA2,
    CMD_GET_DEVICE_CONFIG = 0xA3, // Se il CRC restituito non corrisponde forza la configurazione di default

    CMD_GET_HW_STATE = 0xA4,

    CMD_MEASUREMENT_AVAILABLE = 0xB0,
    CMD_MEASUREMENT_RETRIEVE_NEXT = 0xB1,

    CMD_KEEP_ALIVE = 0xF1,
} ExchangeCommandType;

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


/* -------------------------------------------------------------------------- */
#ifndef __DONGLE_PASSTHRU
devicestate_t exchangeHandler();
#endif

#define Exchange_Connect(x) openChannel()
#define Exchange_Disconnect() closeChannel()

void passthruMainLoop(void);

bool Exchange_sendHandshake(void);

bool Exchange_commandSendResponse(uint16_t dataSize);

void Exchange_commandsHandler(RealTimeCommandType *rtCommand);

#endif // EXCHANGE_H
