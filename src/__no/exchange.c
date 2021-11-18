
#include "xc.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "../utils.h"   // <stdio.h> printf
#include "../device.h" // isUsbConnected()

//#include "../modules/UART2.h"
//#include "MRF_spi1.h"
//#include "MRF24J40.h"

#include "../modules/RTCC.h"
#include "../sampling/measurement.h"
#include "protocol.h"
#include "exchange.h"


#ifndef __DONGLE_PASSTHRU

#define __ACK_RETRY 5
#define __ACK_TIMEOUT_DEFAULT 3000
#define __APP_TIMEOUT_DEFAULT 10000
#define __INCHAR_TIMEOUT_DEFAULT 1000
#define __HANDSHAKE_TIMEOUT_DEFAULT 3000

// const uint16_t BLOCK_MAXSAMPLES = 48;
#define BLOCK_MAXSAMPLES_IN_BYTE 48

extern device_t device;
extern uint16_t rfDestinationAddr;

/* -------------------------------------------------------------------------- */
devicestate_t exchangeHandler()
{
    RealTimeCommandType rtCommand;
    devicestate_t devicemode = TOSLEEP;

    exchangestate_t state = EXCH_OPEN;

    do {
        switch (state) {
        case EXCH_OPEN:
            if (!openChannel()) {
                state = EXCH_EXIT;
            }
            else {
                state = EXCH_START_DISCOVERY;
            }
            break;
        case EXCH_START_DISCOVERY:
            if (Exchange_sendHandshake()) {
                state = EXCH_WAIT_COMMAND;
            }
            else {
                state = EXCH_EXIT;
            }
            break;
        case EXCH_WAIT_COMMAND:
            Exchange_commandsHandler(&rtCommand);
            state = EXCH_EXIT;
            break;
        case EXCH_EXIT:
            break;
        default:
            break;
        }
    }
    while (state != EXCH_EXIT);

    closeChannel();
    return devicemode;
}

#endif //#ifndef __DONGLE_PASSTHRU



#ifndef __DONGLE_PASSTHRU

/* -------------------------------------------------------------------------- */
bool Exchange_sendHandshake(void)
{
    uint8_t command;
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();
    uint32_t timestamp;
    uint16_t timeout;

    timestamp = RTCC_GetTimeL();
    timeout = device.cnf.exchange.handshake_timeout < __HANDSHAKE_TIMEOUT_DEFAULT ?
            __HANDSHAKE_TIMEOUT_DEFAULT : device.cnf.exchange.handshake_timeout;

    memcpy(&buffer[offset], &device.sts.DIN, 4);
    offset += 4;
    memcpy(&buffer[offset], &timestamp, 4); // g_dev.st.timestamp
    offset += 4;
    memcpy(&buffer[offset], &device.sts.version, 2);
    offset += 2;
    memcpy(&buffer[offset], &device.cnf.CRC16, 2);
    offset += 2;
    buffer[offset++] = Device_IsUsbConnected();
    // g_dev.st.link_status USB 0 not connected, <1000 usb connected, RSSI: 100..200 RSSI
    // Is locked.... 

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
bool sendAvailableMeasurement(uint16_t counter)
{
    uint8_t *buffer = ptrSendData();
    memcpy(buffer, &counter, 2);
    return protocolSend(CMD_MEASUREMENT_AVAILABLE, 2, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool sendMeasurement(uint16_t index)
{
    bool result = false;
    measurement_t ms;

    uint16_t x = 0;
    //uint8_t offset = 0;
    uint16_t offset = 0;
    uint8_t *buffer = ptrSendData();

    uint16_t totSamplesBlocks = 0;
    uint16_t spareSamples = 0;
    //uint8_t blockSize = 0;
    uint16_t blockSize = 0;

#if 0 //only debug
    uint16_t xSS = 0;
    ms = &g_measurement;
    ms->dtime = RTCC_GetTimeL();
    ms->ss = SSBUF;
    ms->tset = _SIG0;
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
        totSamplesBlocks = (uint16_t) ((uint16_t) (ms.nss + ms.ns) / BLOCK_MAXSAMPLES_IN_BYTE);
        spareSamples = (uint16_t) ((uint16_t) (ms.nss + ms.ns) % BLOCK_MAXSAMPLES_IN_BYTE);

        if (spareSamples > 0) {
            totSamplesBlocks++;
        }

        offset = 0;
        memcpy(&buffer[offset], &totSamplesBlocks, 2);
        offset += 2;
        memcpy(&buffer[offset], &ms.dtime, 4);
        offset += 4;
        buffer[offset++] = ms.tset;
        memcpy(&buffer[offset], &ms.ns, 2);
        offset += 2;
        memcpy(&buffer[offset], &ms.nss, 2);
        offset += 2;

        #define SAMPLE_SIZE_IN_BYTE 2

        if (protocolSend(CMD_MEASUREMENT_HEADER, offset, __ACK_TIMEOUT_DEFAULT)) {
            result = true;

            for (x = 0; x < totSamplesBlocks; x++) {
                blockSize = BLOCK_MAXSAMPLES_IN_BYTE;
                if ((spareSamples > 0) && (x == (totSamplesBlocks - 1))) {
                    blockSize = spareSamples;
                }

                offset = 0;
                buffer[offset++] = (uint8_t) (x + 1);

                //memcpy(&buffer[offset], &ms.ss[x * BLOCK_MAXSAMPLES], (blockSize * 2));
                measurementLoadSamples(&buffer[offset], x * BLOCK_MAXSAMPLES_IN_BYTE, blockSize * SAMPLE_SIZE_IN_BYTE); // blocksize multiplo di 3 in bytes 

                offset += (blockSize * SAMPLE_SIZE_IN_BYTE);
                result = protocolSend(CMD_MEASUREMENT_BLOCK, offset, __ACK_TIMEOUT_DEFAULT);

                //                if (!protocolSend(CMD_MEASUREMENT_BLOCK, offset, __ACK_TIMEOUT_DEFAULT)) {
                //                    result = false;
                //                    break;
                //                }
            }

        }
    }
    else { //send error
        result = true;
        offset = 2 + 4 + 1 + 2 + 2;
        totSamplesBlocks = 0;
        memset(buffer, 0, offset);
        protocolSend(CMD_MEASUREMENT_HEADER, offset, __ACK_TIMEOUT_DEFAULT);
    }
    return result;
}

/* -------------------------------------------------------------------------- */
bool sendDeviceConfig()
{
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();

    setShort(sizeof (config_t), &buffer[offset]); //2 byte
    offset += 2;
    memcpy(&buffer[offset], &device.cnf, sizeof (config_t));
    offset += sizeof (config_t);

    return protocolSend(CMD_GET_DEVICE_CONFIG, offset, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool setDeviceConfig(uint8_t *rxData)
{
    return Device_ConfigWrite(rxData);
}

/* -------------------------------------------------------------------------- */
void setDateTime(uint8_t *rxData)
{
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
bool sendDeviceState()
{
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();
    //status_t status;

    //Device_GetStatus(&status);
    setShort(sizeof (status_t), &buffer[offset]); //2 byte
    offset += 2;
    //memcpy(&buffer[offset], &status, sizeof (status_t));
    memcpy(&buffer[offset], &device.sts, sizeof (status_t));
    offset += sizeof (status_t);

    return protocolSend(CMD_GET_HW_STATE, offset, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool sendKeepAlive()
{
    return protocolSend(CMD_KEEP_ALIVE, 10, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool Exchange_commandSendResponse(uint16_t dataSize)
{
    return protocolSend(CMD_REALTIME_COMMAND, dataSize, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
void Exchange_commandsHandler(RealTimeCommandType *rtCommand)
{
    bool exit = false;
    uint8_t command;
    uint16_t indexMeasurement = 0;
    uint16_t timeout = device.cnf.exchange.app_timeout < __APP_TIMEOUT_DEFAULT ?
            __APP_TIMEOUT_DEFAULT : device.cnf.exchange.app_timeout;

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
                }
                else if (rxData[0] == MODE_SLEEP) {
                    exit = true;
                }
                else if (rxData[0] == MODE_RESET) {
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
                sendAvailableMeasurement(measurementCounterGet());
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
                *rtCommand = (RealTimeCommandType) rxData[0];
                exit = true;
                break;
                //----------------------------------------------------------
            default:
                exit = true;
                break;
            }
        }
        else {
            exit = true;
        }
        __clearWDT();
    }
    while (!exit);
}
#endif
