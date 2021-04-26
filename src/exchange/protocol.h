#include <stdbool.h>
#include "../device.h"

#ifndef PROTOCOL_H
#define PROTOCOL_H

#define STX_SIZE 1
#define NRPCK_SIZE 2
#define DIN_SIZE 4
#define LEN_SIZE 2
#define CMD_SIZE 1
#define LRC_SIZE 1
//#define DATA_OFFSET STX_SIZE + NRPCK_SIZE + DIN_SIZE + LEN_SIZE + CMD_SIZE
#define DATA_OFFSET STX_SIZE + NRPCK_SIZE + LEN_SIZE + CMD_SIZE

#ifdef __VAMP1K_TEST
#define PROT_BUF_SIZE 1      // printf !!!!
#else
#define PROT_BUF_SIZE 128
#endif

//#define BLOCK_MAXSAMPLES 60
// #define BLOCK_MAXSAMPLES 32
#define BLOCK_MAXSAMPLES 48 // Datablock NVM 96 Byte

// #define BLOCK_MAXSAMPLES 36  multiplo di 3 per il recupero da nvm

typedef enum {
    CNL_NONE = 0x00,
    CNL_USBD,
    CNL_RF,
} ExchangeChannelType;

typedef enum {
    MODE_SLEEP = 0x01,
    MODE_REALTIME,
    MODE_RESET,
} ExchangeModeType;

typedef enum {
    CMD_NONE = 0x00,

    //send
    CMD_HANDSHAKE = 0x01, // Handshake
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

bool Exchange_openChannel();

void Exchange_closeChannel();

bool Exchange_sendHandshake(void);

void Exchange_passthruMainLoop(void);

uint8_t *Exchange_ptrSendData();

bool Exchange_commandSendResponse(uint16_t dataSize);

void Exchange_commandsHandler(RealTimeCommandType *rtCommand);

#endif // PROTOCOL_H
