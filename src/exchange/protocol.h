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
// #define DATA_OFFSET STX_SIZE + NRPCK_SIZE + DIN_SIZE + LEN_SIZE + CMD_SIZE
#define DATA_OFFSET STX_SIZE + NRPCK_SIZE + LEN_SIZE + CMD_SIZE

#ifdef __VAMP1K_TEST
#define PROT_BUF_SIZE 1      // printf !!!!
#else
#define PROT_BUF_SIZE 128
#endif

typedef enum {
    CNL_NONE = 0x00,
    CNL_USBD,
    CNL_RF,
} ExchangeChannelType;

//#define BLOCK_MAXSAMPLES 60
// #define BLOCK_MAXSAMPLES 32
#define BLOCK_MAXSAMPLES 48 // Datablock NVM 96 Byte

// #define BLOCK_MAXSAMPLES 36  multiplo di 3 per il recupero da nvm

#define  CMD_NONE_0x00  0x00
#define  CMD_HANDSHAKE_0x01   0x01// Handshake


uint16_t getShort(const uint8_t *buff);
void setShort(uint16_t val, uint8_t *buff);
void setLong(uint32_t val, uint8_t *buff);



bool openChannel();

void closeChannel();

uint8_t *ptrSendData();
uint8_t *ptrReceiveData(); 

bool protocolSend(uint8_t command, uint16_t dataSize, uint16_t timeOut);

bool protocolReceive(uint8_t *command, uint16_t timeOut);
//void passthruMainLoop(void);
//
//bool Exchange_sendHandshake(void);
//
//bool Exchange_commandSendResponse(uint16_t dataSize);
//
//void Exchange_commandsHandler(RealTimeCommandType *rtCommand);

#endif // PROTOCOL_H
