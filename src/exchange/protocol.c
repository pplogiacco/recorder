
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../utils.h"
#include "protocol.h"
// #include "exchange.h"

#include "../modules/RTCC.h"
#include "../sampling/measurement.h"

#include "uart2.h"

#include "MRF24J40.h"

#ifndef __DONGLE_PASSTHRU

#if 0 // only debug
extern sample_t SSBUF[];
extern measurement_t g_measurement; // working measurement's buffer
#endif

extern device_t g_dev;
uint16_t rfDestinationAddr;
ExchangeChannelType channelType;

uint16_t g_nrPacket = 0;
static uint8_t rxDataOffset;
static uint8_t txrx_buffer[PROT_BUF_SIZE];

#define __ACK_RETRY 5
#define __ACK_TIMEOUT_DEFAULT 3000
#define __APP_TIMEOUT_DEFAULT 10000
#define __INCHAR_TIMEOUT_DEFAULT 1000
#define __HANDSHAKE_TIMEOUT_DEFAULT 3000

enum {
    STX = 0x02,
    ETX = 0x03,
    EOT = 0x04, // end of transmission
    ACK = 0x06, // acknowledge
    NAK = 0x15 //  negative acknowledge
};

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
bool Exchange_openChannel() {

    if (Device_IsUsbConnected()) {
        UART2_Enable();
        channelType = CNL_USBD;

    } else {    // Try by RF

        MRF24J40_Enable(); // Enable ??
        
        MRF24J40_setAddress(MRF24J40_DEVICE_ADDRESS);
        rfDestinationAddr = MRF24J40_DONGLE_ADDRESS;
        channelType = CNL_RF;
    }

    return (true);
}

/* -------------------------------------------------------------------------- */
void Exchange_closeChannel() {
    if (channelType == CNL_USBD) {
        UART2_Disable();
    } else if (channelType == CNL_RF) {
        MRF24J40_Disable();   // Disable ??
    }
    channelType = CNL_NONE;
};

/* -------------------------------------------------------------------------- */
uint16_t USBD_RxBuffer(uint8_t *buff, uint16_t maxSize) {
    uint8_t rxByte = 0;
    uint16_t rxSize = 0;

    Timeout_Set(0, 250);
    do {
        if (UART2_IsRxReady()) {
            rxByte = UART2_Read();
            buff[rxSize++] = rxByte;
            if (rxSize == maxSize) {
                break;
            }
        }
    } while (!isTimeout());
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
uint8_t *Exchange_ptrSendData() {
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
        if (command == CMD_HANDSHAKE) {
            break;
        }
    }
    return ret;
}

typedef enum {
    WAIT_STX = 1,
    WAIT_NR_PACKET, //2byte
    //WAIT_DIN, //4byte
    WAIT_LEN, //1byte
    WAIT_CMD,
    WAIT_DATA,
    WAIT_LRC,
    WAIT_END
} ReceiveState;

/* -------------------------------------------------------------------------- */
bool protocolReceive(uint8_t *command, uint16_t timeOut) {
    bool result = false;
    uint16_t length = 0;
    uint16_t lTimeOut = timeOut;
    uint16_t rxSize = 0;
    uint16_t rxMaxSize = STX_SIZE;
    uint16_t dataLen = 0;
    uint16_t stxOffset = 0;
    ReceiveState rxState = WAIT_STX;
    uint16_t commandLen = 0;
    uint8_t LRC = 0;
    uint16_t offset = 0;

    uint16_t icTimeout = g_dev.cnf.exchange.interchar_timeout < __INCHAR_TIMEOUT_DEFAULT ?
            __INCHAR_TIMEOUT_DEFAULT : g_dev.cnf.exchange.interchar_timeout;

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

/* -------------------------------------------------------------------------- */
bool Exchange_sendHandshake(void) {
    uint8_t command;
    uint8_t offset = 0;
    uint8_t *buffer = Exchange_ptrSendData();
    uint32_t timestamp;
    uint16_t timeout;

    timestamp = RTCC_GetTimeL();
    timeout = g_dev.cnf.exchange.handshake_timeout < __HANDSHAKE_TIMEOUT_DEFAULT ?
            __HANDSHAKE_TIMEOUT_DEFAULT : g_dev.cnf.exchange.handshake_timeout;

    memcpy(&buffer[offset], &g_dev.st.DIN, 4);
    offset += 4;
    memcpy(&buffer[offset], &timestamp, 4);
    offset += 4;
    memcpy(&buffer[offset], &g_dev.st.version, 2);
    offset += 2;
    //g_dev.cnf.CRC16 = 331; // Test
    memcpy(&buffer[offset], &g_dev.cnf.CRC16, 2);
    offset += 2;
    buffer[offset++] = Device_IsUsbConnected();

    if (protocolSend(CMD_HANDSHAKE, offset, timeout)) {
        if (protocolReceive(&command, timeout)) {
            //uint8_t *rxData = ptrReceiveData();
            //(void) rxData;
            return true;
        }
    }
    return false;
}

/* -------------------------------------------------------------------------- */
bool sendAvailableMeasurement(uint16_t counter) {
    uint8_t *buffer = Exchange_ptrSendData();
    memcpy(buffer, &counter, 2);
    return protocolSend(CMD_MEASUREMENT_AVAILABLE, 2, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool sendMeasurement(uint16_t index) {
    bool result = false;
    measurement_t ms;

    uint16_t x = 0;
    uint8_t offset = 0;
    uint8_t *buffer = Exchange_ptrSendData();

    uint16_t totSamplesBlocks = 0;
    uint16_t spareSamples = 0;
    uint8_t blockSize = 0;

#if 0 //only debug
    uint16_t xSS = 0;
    ms = &g_measurement;
    ms->dtime = RTCC_GetTimeL();
    ms->ss = SSBUF;
    ms->typeset = _SIG0;
    // Single samples
    ms->ns = 2;
    ms->nss = 512;

    xSS = 0;
    ms->ss[xSS++] = 888; // Temperature
    ms->ss[xSS++] = 555; // WindSpeed
    for (xSS = 0; xSS < ms->nss; xSS++) {
        ms->ss[xSS + 2] = 1000 + (xSS + 1);
    }
    if (1) {
#else
    if (measurementLoad(index, &ms) > 0) {
#endif
        totSamplesBlocks = (uint16_t) ((uint16_t) (ms.nss + ms.ns) / BLOCK_MAXSAMPLES);
        spareSamples = (uint16_t) ((uint16_t) (ms.nss + ms.ns) % BLOCK_MAXSAMPLES);

        if (spareSamples > 0) {
            totSamplesBlocks++;
        }

        offset = 0;
        memcpy(&buffer[offset], &totSamplesBlocks, 2);
        offset += 2;
        memcpy(&buffer[offset], &ms.dtime, 4);
        offset += 4;
        buffer[offset++] = ms.typeset;
        memcpy(&buffer[offset], &ms.ns, 2);
        offset += 2;
        memcpy(&buffer[offset], &ms.nss, 2);
        offset += 2;

        if (protocolSend(CMD_MEASUREMENT_HEADER, offset, __ACK_TIMEOUT_DEFAULT)) {
            result = true;
            for (x = 0; x < totSamplesBlocks; x++) {
                blockSize = BLOCK_MAXSAMPLES;
                if ((spareSamples > 0) && (x == (totSamplesBlocks - 1))) {
                    blockSize = spareSamples;
                }

                offset = 0;
                buffer[offset++] = (uint8_t) (x + 1);
                
                //memcpy(&buffer[offset], &ms.ss[x * BLOCK_MAXSAMPLES], (blockSize * 2));
                getMeasurementBlock(&buffer[offset], x * BLOCK_MAXSAMPLES,  blockSize * 2  ); // blocksize multiplo di 3 in bytes 
                             
                offset += (blockSize * 2);
                if (!protocolSend(CMD_MEASUREMENT_BLOCK, offset, __ACK_TIMEOUT_DEFAULT)) {
                    result = false;
                    break;
                }
            }
        }
    } else { //send error
        result = true;
        offset = 2 + 4 + 1 + 2 + 2;
        totSamplesBlocks = 0;
        memset(buffer, 0, offset);
        protocolSend(CMD_MEASUREMENT_HEADER, offset, __ACK_TIMEOUT_DEFAULT);
    }
    return result;
}

/* -------------------------------------------------------------------------- */
bool sendDeviceConfig() {
    uint8_t offset = 0;
    uint8_t *buffer = Exchange_ptrSendData();

    setShort(sizeof (config_t), &buffer[offset]); //2 byte
    offset += 2;
    memcpy(&buffer[offset], &g_dev.cnf, sizeof (config_t));
    offset += sizeof (config_t);

    return protocolSend(CMD_GET_DEVICE_CONFIG, offset, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool setDeviceConfig(uint8_t *rxData) {
    return Device_ConfigWrite(rxData);
}

/* -------------------------------------------------------------------------- */
void setDateTime(uint8_t *rxData) {
    uint8_t offset = 0;
    timestamp_t ts;
    unsigned char weekday;

    ts.year = rxData[offset++];
    ts.month = rxData[offset++];
    ts.day = rxData[offset++];
    ts.hour = rxData[offset++];
    ts.min = rxData[offset++];
    ts.sec = rxData[offset++];
    weekday = rxData[offset++];

    RTCC_SetTime(&ts, weekday);
}

/* -------------------------------------------------------------------------- */
bool sendDeviceState() {
    uint8_t offset = 0;
    uint8_t *buffer = Exchange_ptrSendData();
    //status_t status;

    //Device_GetStatus(&status);
    setShort(sizeof (status_t), &buffer[offset]); //2 byte
    offset += 2;
    //memcpy(&buffer[offset], &status, sizeof (status_t));
    memcpy(&buffer[offset], &g_dev.st, sizeof (status_t));
    offset += sizeof (status_t);

    return protocolSend(CMD_GET_HW_STATE, offset, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool sendKeepAlive() {
    return protocolSend(CMD_KEEP_ALIVE, 10, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool Exchange_commandSendResponse(uint16_t dataSize){
    return protocolSend(CMD_REALTIME_COMMAND, dataSize, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
void Exchange_commandsHandler(RealTimeCommandType *rtCommand) {
    bool exit = false;
    uint8_t command;
    uint16_t indexMeasurement = 0;
    uint16_t timeout = g_dev.cnf.exchange.app_timeout < __APP_TIMEOUT_DEFAULT ?
            __APP_TIMEOUT_DEFAULT : g_dev.cnf.exchange.app_timeout;

    uint8_t *rxData;

    do {
        command = CMD_NONE;
        *rtCommand = RTCMD_NONE;
        if (protocolReceive(&command, timeout)) {
            rxData = ptrReceiveData();
            switch (command) {
                    //----------------------------------------------------------
                case CMD_INIT_DEVICE:
                    break;
                    //----------------------------------------------------------
                case CMD_SWITCH_MODE:
                    if (rxData[0] == MODE_REALTIME) {
                        // realtime
                    } else if (rxData[0] == MODE_SLEEP) {
                        exit = true;
                    } else if (rxData[0] == MODE_RESET) {
                        //ToDo jmp 0
                    }
                    break;
                    //----------------------------------------------------------
                case CMD_SET_DATE_TIME:
                    setDateTime(rxData);
                    break;
                    //----------------------------------------------------------
                case CMD_SET_DEVICE_CONFIG:
                    setDeviceConfig(rxData);
                    break;
                    //----------------------------------------------------------
                case CMD_GET_DEVICE_CONFIG:
                    sendDeviceConfig();
                    break;
                    //----------------------------------------------------------
                case CMD_GET_HW_STATE:
                    sendDeviceState();
                    break;
                    //----------------------------------------------------------
                case CMD_MEASUREMENT_AVAILABLE:
                    indexMeasurement = 1;
                    sendAvailableMeasurement(measurementCounter());
                    break;
                    //----------------------------------------------------------
                case CMD_MEASUREMENT_RETRIEVE_NEXT:
                    if (sendMeasurement(indexMeasurement)) {
                        measurementDelete(indexMeasurement);
                        indexMeasurement++;
                    }
                    break;
                    //----------------------------------------------------------
                case CMD_KEEP_ALIVE:
                    sendKeepAlive();
                    break;
                    //----------------------------------------------------------
                case CMD_REALTIME_COMMAND:
                    *rtCommand = (RealTimeCommandType)rxData[0];
                    exit = true;
                    break;
                    //----------------------------------------------------------
                default:
                    exit = true;
                    break;
            }
        } else {
            exit = true;
        }
        __clearWDT();
    } while (!exit);
}
#endif

#ifdef __DONGLE_PASSTHRU

/* -------------------------------------------------------------------------- */
bool RF_TxBuffer(uint8_t *buff, uint16_t size) {
    bool result = false;
    uint16_t lTimeOut = 250;

    MRF24J40_TxBuffer(MRF24J40_DEVICE_ADDRESS, buff, size, false);
    do {
        if (MRF24J40_transmissionDone()) {
            result = true;
            break;
        }
        __delay_ms(1);
    } while (lTimeOut--);
    return result;
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
int USBD_RxBuffer(uint8_t *buff, uint16_t maxSize) {
    uint8_t rxByte = 0;
    int rxSize = 0;
    uint16_t lTimeOut = 50;
    do {
        if (UART2_IsRxReady()) {
            lTimeOut = 50;
            rxByte = UART2_Read();
            buff[rxSize++] = rxByte;
            if (rxSize == maxSize) {
                break;
            }
        }
        __delay_ms(1);
    } while (lTimeOut--);
    return rxSize;
}

#define USBD_IsRxReady UART2_IsRxReady

#define INACTIVITY_TIME 3000

void passthruMainLoop(void) {

    int rxSize = 0;
    uint8_t buffer[256];

    UART2_Initialize();

    SPI1_Initialize();

    MRF24J40_Enable();

    MRF24J40_setAddress(MRF24J40_DONGLE_ADDRESS);

    uint16_t lTimeOut = INACTIVITY_TIME;
    while (1) {
        //test send
        //RF_TxBuffer("\x31\x32\x33\x34", 4);
        //__delay_ms(1000);
        //
        
        if (MRF24J40_receivePacket()) {
            IO_LED1_On();
            lTimeOut = INACTIVITY_TIME;

            memset(buffer, 0, sizeof (buffer));
            rxSize = MRF24J40_RxBuffer(buffer, sizeof (buffer));
            if (rxSize > 0) {
                USBD_TxBuffer(buffer, rxSize);
            }
        }

        if (USBD_IsRxReady()) {
            IO_LED1_On();
            lTimeOut = INACTIVITY_TIME;

            memset(buffer, 0, sizeof (buffer));
            rxSize = USBD_RxBuffer(buffer, sizeof (buffer));
            if (rxSize > 0) {
                RF_TxBuffer(buffer, rxSize);
            }

        }

        lTimeOut--;
        if (lTimeOut == 0) {
            lTimeOut = INACTIVITY_TIME;
            IO_LED1_Off();
        }
        __delay_ms(1);
    }
}
#endif



