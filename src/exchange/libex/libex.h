#include <xc.h>
#include <stdbool.h>

#ifndef PROTOCOL_H
#define PROTOCOL_H

#define STX_SIZE 1
#define NRPCK_SIZE 2
#define DIN_SIZE 4
#define LEN_SIZE 2
#define CMD_SIZE 1
#define LRC_SIZE 1

#define DATA_OFFSET STX_SIZE + NRPCK_SIZE + LEN_SIZE + CMD_SIZE

#define __ACK_RETRY 5
#define __ACK_TIMEOUT_DEFAULT 3000
#define __APP_TIMEOUT_DEFAULT 10000
#define __INCHAR_TIMEOUT_DEFAULT 1000
#define __HANDSHAKE_TIMEOUT_DEFAULT 3000


#undef __DONGLE_PASSTHRU

#ifdef __TEST
#define PROT_BUF_SIZE 1      // printf !!!!
#else
#define PROT_BUF_SIZE 128
#endif

//#define BLOCK_MAXSAMPLES 32
#define BLOCK_MAXSAMPLES 48 // Datablock NVM 96 Byte

uint16_t getShort(const uint8_t *buff);
void setShort(uint16_t val, uint8_t *buff);
void setLong(uint32_t val, uint8_t *buff) ;


/* locals
uint16_t USBD_RxBuffer(uint8_t *buff, uint16_t maxSize);
bool USBD_TxBuffer(uint8_t *buff, uint16_t size);
bool RF_TxBuffer(uint8_t *buff, uint16_t size);
bool RF_IsRxReady(void);
uint16_t RF_RxBuffer(uint8_t *buff, uint16_t maxSize);
uint16_t _RxBuffer(uint8_t *buff, uint16_t maxSize);
bool _IsRxReady(void);
bool _TxBuffer(uint8_t *buff, uint16_t size);
uint8_t calculateLRC(uint8_t *data, uint16_t length);
*/

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


#endif // PROTOCOL_H
