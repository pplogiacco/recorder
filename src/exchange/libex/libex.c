
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// #include "../RTCC.h"   // Timeout

// #define  __delay(x) {Nop(); Nop(); Nop(); Nop(); Nop(); }

#include "libex.h"
//#include "exchange.h" // CMD_HANDSHAKE

// channel's drivers
#include "drivers/uart2.h"
#include "drivers/MRF24J40.h"


// To be checked 
 uint16_t rfDestinationAddr;


enum {
    STX = 0x02,
    ETX = 0x03,
    EOT = 0x04, // end of transmission
    ACK = 0x06, // acknowledge
    NAK = 0x15 //  negative acknowledge
};

typedef enum {
    WAIT_STX = 1,
    WAIT_NR_PACKET, //2byte
    //WAIT_DIN, //4byte
    WAIT_LEN, //1byte
    WAIT_CMD,
    WAIT_DATA,
    WAIT_LRC,
    WAIT_END
} rcv_state_t;

typedef enum {
    CNL_NONE = 0x00,
    CNL_USBD,
    CNL_RF,
} channel_t;


static channel_t channelType;
static uint8_t rxDataOffset;
static uint8_t txrx_buffer[PROT_BUF_SIZE];
uint16_t g_nrPacket = 0;

uint16_t getShort(const uint8_t *buff) {
    return ((buff[0] << 8) + buff[1]);
}

void setShort(uint16_t val, uint8_t *buff) {
    buff[0] = (uint8_t) ((val & 0xFF00) >> 8);
    buff[1] = (uint8_t) (val & 0x00FF);
}

void setLong(uint32_t val, uint8_t *buff) {
    buff[0] = (uint8_t) ((val & 0xFF000000) >> 24);
    buff[1] = (uint8_t) ((val & 0x00FF0000) >> 16);
    buff[2] = (uint8_t) ((val & 0x0000FF00) >> 8);
    buff[3] = (uint8_t) ((val & 0x000000FF));
}

/* -------------------------------------------------------------------------- */


bool openChannel(bool isUsb) {

    if (isUsb) {
        UART2_Enable();
        channelType = CNL_USBD;

    } else { // Try by RF

        MRF24J40_Enable(); // Enable ??
        MRF24J40_setAddress(MRF24J40_DEVICE_ADDRESS);
        rfDestinationAddr = MRF24J40_DONGLE_ADDRESS;
        channelType = CNL_RF;
    }

    return (true);
}

/* -------------------------------------------------------------------------- */
void closeChannel() {
    if (channelType == CNL_USBD) {
        UART2_Disable();
    } else if (channelType == CNL_RF) {
        MRF24J40_Disable(); // Disable ??
    }
    channelType = CNL_NONE;
};

/* -------------------------------------------------------------------------- */
uint16_t USBD_RxBuffer(uint8_t *buff, uint16_t maxSize) {
    uint8_t rxByte = 0;
    uint16_t rxSize = 0;
    uint16_t lTimeOut = 250;

    //    setTimeout(0, 250);
    do {
        if (UART2_IsRxReady()) {
            rxByte = UART2_Read();
            buff[rxSize++] = rxByte;
            if (rxSize == maxSize) {
                break;
            }
        }
        __delay(1);
    } while (lTimeOut--);
    //   } while (!isTimeout());
    return rxSize;
}

/* -------------------------------------------------------------------------- */
bool USBD_TxBuffer(uint8_t *buff, uint16_t size) {
    int i;
    for (i = 0; i < size; i++) {
        UART2_Write(buff[i]);
    }
    while (!UART2_IsTxDone());
    return true;
}

/* -------------------------------------------------------------------------- */
bool RF_TxBuffer(uint8_t *buff, uint16_t size) {
    bool result = false;
    uint16_t lTimeOut = 250;

    MRF24J40_TxBuffer(rfDestinationAddr, buff, size, false);
    do {
        if (MRF24J40_transmissionDone()) {
            result = true;
            break;
        }
        __delay(1);
    } while (lTimeOut--);
    return result;
}

#define USBD_IsRxReady UART2_IsRxReady

/* -------------------------------------------------------------------------- */
bool RF_IsRxReady(void) {
    if (MRF24J40_IsRxReady()) {
        return true;
    } else {
        return MRF24J40_receivePacket();
    }
    return false;
}

/* -------------------------------------------------------------------------- */
uint16_t RF_RxBuffer(uint8_t *buff, uint16_t maxSize) {
    return MRF24J40_RxBuffer(buff, maxSize);
}

/* -------------------------------------------------------------------------- */
uint16_t _RxBuffer(uint8_t *buff, uint16_t maxSize) {
    if (channelType == CNL_USBD) {
        return USBD_RxBuffer(buff, maxSize);
    } else if (channelType == CNL_RF) {
        return RF_RxBuffer(buff, maxSize);
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
bool _IsRxReady(void) {
    if (channelType == CNL_USBD) {
        return USBD_IsRxReady();
    } else if (channelType == CNL_RF) {
        return RF_IsRxReady();
    }
    return false;
}

/* -------------------------------------------------------------------------- */
bool _TxBuffer(uint8_t *buff, uint16_t size) {
    if (channelType == CNL_USBD) {
        return USBD_TxBuffer(buff, size);
    } else if (channelType == CNL_RF) {
        return RF_TxBuffer(buff, size);
    }
    return false;
}

/* -------------------------------------------------------------------------- */
uint8_t calculateLRC(uint8_t *data, uint16_t length) {
    uint8_t LRC = 0;
    uint16_t i = 0;
    for (i = 0; i < length; i++) {
        LRC ^= data[i];
    }
    return LRC;
}

/* -------------------------------------------------------------------------- */
void protocolFlush() {
    uint8_t rxByte;
    uint16_t lTimeOut = 1;

    if (channelType == CNL_RF) {
        MRF24J40_rxFlush();
    } else {
        do {
            if (_IsRxReady()) {
                if (_RxBuffer(&rxByte, 1) > 0) {
                    lTimeOut = 1;
                }
            }
            __delay(1);

        } while (lTimeOut--);
    }
}

/* -------------------------------------------------------------------------- */
void sendAck() {
    uint8_t value = ACK;
    protocolFlush();
    _TxBuffer(&value, 1);
}

/* -------------------------------------------------------------------------- */
void sendNack() {
    uint8_t value = NAK;
    protocolFlush();
    _TxBuffer(&value, 1);
}

/* -------------------------------------------------------------------------- */
bool waitAck(uint16_t timeOut) {
    uint8_t value = 0x00;
    uint16_t lTimeOut = (timeOut == 0) ? __ACK_TIMEOUT_DEFAULT : timeOut;

    do {
        if (_IsRxReady()) {
            uint16_t len = _RxBuffer(&value, 1);
            if (len == 1 && value == ACK) {
                return true;
            }
        }
        __delay(1);
    } while (lTimeOut--);
    return false;
}

/* -------------------------------------------------------------------------- */
uint8_t *ptrSendData() {
    memset(txrx_buffer, 0, PROT_BUF_SIZE);
    return &txrx_buffer[DATA_OFFSET];
}

/* -------------------------------------------------------------------------- */
uint8_t *ptrReceiveData() {
    return &txrx_buffer[rxDataOffset];
}

/* -------------------------------------------------------------------------- */
bool protocolSend(uint8_t command, uint16_t dataSize, uint16_t timeOut) {
    bool ret = false;
    uint8_t retry;
    int sendLen = 0;

    txrx_buffer[sendLen] = STX;
    sendLen += STX_SIZE;

    g_nrPacket++;
    setShort(g_nrPacket, &txrx_buffer[sendLen]);
    sendLen += NRPCK_SIZE;

    //setLong(g_status.DIN, &txrx_buffer[sendLen]);
    //sendLen += DIN_SIZE;

    setShort((dataSize + 2), &txrx_buffer[sendLen]);
    sendLen += LEN_SIZE;

    txrx_buffer[sendLen] = command;
    sendLen += CMD_SIZE;

    if (dataSize > 0) {
        sendLen += dataSize;
    }

    txrx_buffer[sendLen] = calculateLRC(txrx_buffer, sendLen);
    sendLen += LRC_SIZE;

    for (retry = 0; retry < __ACK_RETRY; ++retry) {
        protocolFlush();
        _TxBuffer(txrx_buffer, sendLen);
        if (waitAck(timeOut) == true) {
            ret = true;
            break;
        }
        if (command == 0x01) { //    exchange.h: CMD_HANDSHAKE = 0x01, // Handshake
            break;
        }
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
bool protocolReceive(uint8_t *command, uint16_t timeOut) {
    bool result = false;
    uint16_t length = 0;
    uint16_t lTimeOut = timeOut;
    uint16_t rxSize = 0;
    uint16_t rxMaxSize = STX_SIZE;
    uint16_t dataLen = 0;
    uint16_t stxOffset = 0;
    rcv_state_t rxState = WAIT_STX;
    uint16_t commandLen = 0;
    uint8_t LRC = 0;
    uint16_t offset = 0;

    uint16_t icTimeout = __INCHAR_TIMEOUT_DEFAULT;

    rxDataOffset = 0;
    rxState = WAIT_STX;
    rxMaxSize = STX_SIZE;
    memset(txrx_buffer, 0, sizeof (txrx_buffer));
    do {
        result = false;
        if (_IsRxReady()) {
            rxSize = 0;
            rxSize = _RxBuffer(&txrx_buffer[length], rxMaxSize);
            if (rxSize > 0) {
                lTimeOut = icTimeout;

                length += rxSize;
                switch (rxState) {
                    case WAIT_STX:
                        if (txrx_buffer[offset] == STX) {
                            stxOffset = offset;
                            dataLen = 0;
                            //nrPacket = 0;
                            rxMaxSize = NRPCK_SIZE;
                            rxState = WAIT_NR_PACKET;
                        }
                        offset++;
                        break;
                    case WAIT_NR_PACKET:
                        dataLen += rxSize;
                        if (dataLen == NRPCK_SIZE) {
                            //ToDo nrPacket = ((txrx_buffer[offset] << 8) + txrx_buffer[offset+1]);
                            dataLen = 0;
                            rxMaxSize = LEN_SIZE;
                            rxState = WAIT_LEN;
                            offset += LEN_SIZE;
                        } else {
                            rxMaxSize -= rxSize;
                        }
                        break;
                    case WAIT_LEN:
                        dataLen += rxSize;
                        if (dataLen == LEN_SIZE) {
                            commandLen = (((txrx_buffer[offset] << 8) + txrx_buffer[offset + 1]) - 2);
                            if (commandLen > (PROT_BUF_SIZE - DATA_OFFSET)) {
                                length = 0;
                                commandLen = 0;
                                lTimeOut = timeOut;
                                rxMaxSize = STX_SIZE;

                                offset = 0;
                                rxState = WAIT_STX;
                                memset(txrx_buffer, 0, sizeof (txrx_buffer));
                                sendNack();
                            } else {
                                dataLen = 0;
                                rxMaxSize = CMD_SIZE;
                                rxState = WAIT_CMD;
                                offset += CMD_SIZE;
                            }
                        } else {
                            rxMaxSize -= rxSize;
                        }
                        break;
                    case WAIT_CMD:
                        dataLen = 0;
                        offset++;
                        *command = txrx_buffer[offset];
                        if (commandLen == 0) {
                            rxMaxSize = LRC_SIZE;
                            rxState = WAIT_LRC;
                        } else {
                            rxMaxSize = commandLen;
                            rxState = WAIT_DATA;
                        }
                        offset += CMD_SIZE;
                        break;
                    case WAIT_DATA:
                        dataLen += rxSize;
                        if (dataLen == commandLen) {
                            rxMaxSize = LRC_SIZE;
                            offset += commandLen;
                            rxState = WAIT_LRC;
                        } else {
                            rxMaxSize -= rxSize;
                        }
                        break;
                    case WAIT_LRC:
                        offset++;
                        LRC = calculateLRC(&txrx_buffer[stxOffset], offset - stxOffset);
                        if (LRC == txrx_buffer[offset]) {
                            rxDataOffset = stxOffset + DATA_OFFSET;
                            sendAck();
                            rxState = WAIT_END;
                            result = true;
                        } else {
                            length = 0;
                            commandLen = 0;
                            lTimeOut = timeOut;
                            rxMaxSize = STX_SIZE;

                            offset = 0;
                            rxState = WAIT_STX;
                            memset(txrx_buffer, 0, sizeof (txrx_buffer));
                            sendNack();
                        }
                    case WAIT_END:
                        break;
                }
            }
        }
        if (rxState == WAIT_END) {
            break;
        }
        __delay(1);
    } while (lTimeOut--);

    return result;
}

