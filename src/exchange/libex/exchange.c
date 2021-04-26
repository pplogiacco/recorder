#include "xc.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

//#include "../utils.h"   // <stdio.h> printf
#include "../device.h" // isUsbConnected()  ??????????????????????
#include "../modules/RTCC.h"   // Datetime & Timeout 

#include "exchange.h"
#include "libex/libex.h"


#ifdef EX_DEVICE
#include "../device.h"
extern device_t g_dev;
/*  g_dev.st.DIN
    timestamp
    g_dev.st.version
    g_dev.cnf.CRC16
    g_dev.st.link_status */
#endif 


#ifdef EX_MEASUREMENT
#include "../sampling/sampling.h"
#if 0 // only debug
extern sample_t SSBUF[];
extern measurement_t g_measurement; // working measurement's buffer
#endif
#endif

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


/* -------------------------------------------------------------------------- */
//bool Exchange_connect(bool useUsb) {
//   return openChannel(useUsb);
//}
//bool Exchange_disconnect(){
//return closeChannel();
//}
/* -------------------------------------------------------------------------- */
bool sendKeepAlive() {
    return protocolSend(CMD_KEEP_ALIVE, 10, __ACK_TIMEOUT_DEFAULT);
}
/* -------------------------------------------------------------------------- */
bool Exchange_commandSendResponse(uint16_t dataSize) {
    return protocolSend(CMD_REALTIME_COMMAND, dataSize, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool Exchange_sendHandshake(uint32_t din, uint32_t ltime, uint16_t hfver, uint16_t ccrc) {
    uint8_t command;
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();
    uint32_t timestamp;
    //    uint16_t timeout = g_dev.cnf.exchange.handshake_timeout < __HANDSHAKE_TIMEOUT_DEFAULT ?
    //            __HANDSHAKE_TIMEOUT_DEFAULT : g_dev.cnf.exchange.handshake_timeout;
    uint16_t timeout = __HANDSHAKE_TIMEOUT_DEFAULT;
    
//    memcpy(&buffer[offset], &din, 4);
//    offset += 4;
//    memcpy(&buffer[offset], &ltime, 4);
//    offset += 4;
//    memcpy(&buffer[offset], &hfver, 2);
//    offset += 2;
//    memcpy(&buffer[offset], &ccrc, 2);
//    offset += 2;
//    buffer[offset++] = Device_IsUsbConnected(); // &g_dev.st.link_status ??? protocolChannelInfo()

    timestamp = RTCC_CurrentTimeL();
        
    memcpy(&buffer[offset], &g_dev.st.DIN, 4);
    offset += 4;
    memcpy(&buffer[offset], &timestamp, 4);
    offset += 4;
    memcpy(&buffer[offset], &g_dev.st.version, 2);
    offset += 2;
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



#ifdef EX_MEASUREMENT

/* -------------------------------------------------------------------------- */
bool sendAvailableMeasurement(uint16_t counter) {
    uint8_t *buffer = ptrSendData();
    memcpy(buffer, &counter, 2);
    return protocolSend(CMD_MEASUREMENT_AVAILABLE, 2, __ACK_TIMEOUT_DEFAULT);
}

/* -------------------------------------------------------------------------- */
bool sendMeasurement(uint16_t index) {
    bool result = false;

    measurement_t ms;

    uint16_t x = 0;
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();

    uint16_t totSamplesBlocks = 0;
    uint16_t spareSamples = 0;
    uint8_t blockSize = 0;

#if 0 //only debug
    uint16_t xSS = 0;
    ms = &g_measurement;
    ms->dtime = RTCC_CurrentTimeL();
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
                getMeasurementBlock(&buffer[offset], x * BLOCK_MAXSAMPLES, blockSize * 2); // blocksize multiplo di 3 in bytes 

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

#endif


#ifdef EX_DEVICE    

/* -------------------------------------------------------------------------- */
bool sendDeviceConfig() {
    uint8_t offset = 0;
    uint8_t *buffer = ptrSendData();

    setShort(sizeof (config_t), &buffer[offset]); //2 byte
    offset += 2;
    memcpy(&buffer[offset], &g_dev.cnf, sizeof (config_t));
    offset += sizeof (config_t);

    return protocolSend(CMD_GET_DEVICE_CONFIG, offset, __ACK_TIMEOUT_DEFAULT);
}

// Call -backs

/* -------------------------------------------------------------------------- */
bool setDeviceConfig(uint8_t *rxData) {
    return Device_WriteConfig(rxData);
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

       RTCC_TimeSet(&ts, weekday); 
}

/* -------------------------------------------------------------------------- */
bool sendDeviceState() {

    uint8_t offset = 0;

    uint8_t *buffer = ptrSendData();
    //status_t status;

    //Device_GetStatus(&status);
    setShort(sizeof (status_t), &buffer[offset]); //2 byte
    offset += 2;
    
    //memcpy(&buffer[offset], &status, sizeof (status_t));
    memcpy(&buffer[offset], &g_dev.st, sizeof (status_t));
    offset += sizeof (status_t);

    return protocolSend(CMD_GET_HW_STATE, offset, __ACK_TIMEOUT_DEFAULT);
}
#endif

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
void Exchange_commandsHandler(RealTimeCommandType *rtCommand) {
    bool exit = false;
    uint8_t command;
    uint8_t *rxData;

#ifdef EX_DEVICE
    uint16_t timeout = g_dev.cnf.exchange.app_timeout < __APP_TIMEOUT_DEFAULT ?
            __APP_TIMEOUT_DEFAULT : g_dev.cnf.exchange.app_timeout;
#else
    uint16_t timeout = __APP_TIMEOUT_DEFAULT;
#endif

#ifdef EX_MEASUREMENT
    uint16_t indexMeasurement = 0;
#endif
    
    do {
        command = CMD_NONE;
        *rtCommand = RTCMD_NONE;
        if (protocolReceive(&command, timeout)) {
            rxData = ptrReceiveData();
            switch (command) {

                    //----------------------------------------------------------
#ifdef EX_DEVICE

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
#endif

#ifdef EX_MEASUREMENT

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

#endif
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
        } else {
            exit = true;
        }
        __clearWDT();
    } while (!exit);
}








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



/*
// -------------------------------------------------------------------------- //
devicestate_t exchangeHandler() {
    RealTimeCommandType rtCommand;
    devicestate_t devicemode = TOSLEEP;

    ExchangeStateType state = EXCH_OPEN;

    do {
        switch (state) {
            case EXCH_OPEN:
                if (!Exchange_openChannel()) {
                    state = EXCH_EXIT;
                } else {
                    state = EXCH_START_DISCOVERY;
                }
                break;
            case EXCH_START_DISCOVERY:
                if (Exchange_sendHandshake()) {
                    state = EXCH_WAIT_COMMAND;
                } else {
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
    } while (state != EXCH_EXIT);

    Exchange_closeChannel();
    return devicemode;
}

 */

