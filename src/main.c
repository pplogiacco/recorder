/******************************************************************************\
|                                                                              |
| V A M P -  R E C O R D E R                                                   |
| Ver. 0.0.27                                                  (@)2021 DTeam ! |
|                                                                              |
\******************************************************************************/

//------------------------------------------------------------------------------
#include "bits.h"                   // PIC Settings
#include "test.h"
#include "modules/RTCC.h"           // hal           
#include "modules/UART2.h"          // hal
#include "device.h"                 // DaaS-Ex Services: config, status, sync  
#include "utils.h"
#include "sampling/measurement.h"
#include "daas/libex.h"

//------------------------------------------------------------------------------ 
device_t device;
sample_t SSBUF[SS_BUF_SIZE]; // global buffer
measurement_t lmeas;
//------------------------------------------------------------------------------


#ifdef __VAMP1K_TEST 
#include "test.c"  
#else  

int main(void) {

    runlevel_t rlevel; // Use: device.st.llevel
    RealTimeCommandType exRTCmd;
    timestamp_t stime;

    devicestate_t state, lstate; // Main cycle controls
    state = STARTUP; // System startup
    lstate = state;

    while (1) {

        switch (state) {
                //------------------------------------------------------------------
            case STARTUP: // Executed only after Power-On/Reset
                Device_SwitchSys(SYS_BOOT); // Initialize Hardware & Services
                if (!Device_ConfigRead()) { // Read persistent config
                    // if error... Factory default
                    depotDefaultSet(); // Memory   
                }
                Device_StatusRead();
                state = EXCHANGE; // After start-up always try to connect
                break;

                //------------------------------------------------------------------
            case SAMPLING:

                // if ( g_dev.cnf.samplingmode != CNF_SAMPLING_OFF )
                //!! RTCC_TimeGet(&stime);
                //!! if ((stime.lstamp > g_config.general.startdate) && (stime.lstamp < g_config.general.stopdate)) {

                measurementAcquire(); // Uses Device_SwitchSys()
                Device_SwitchSys(SYS_EXCHANGE);
                measurementSave();
                Device_SwitchSys(SYS_DEFAULT);
                //                //!!};
                //                if (lstate == EXCHANGE_RT) {
                //                    lstate = state;
                //                    state = EXCHANGE_RT;
                //                } else {
                //                    lstate = state;
                //                    // state = (g_dev.cnf.exchange.attempt_mode == EXCHANGE_ATTEMPTMODE_EVERYCYCLE) ? EXCHANGE : TOSLEEP;
                //                    state = EXCHANGE;
                //                }
                lstate = state;
                state = EXCHANGE;
                break;

                //------------------------------------------------------------------
            case EXCHANGE:

                lstate = state;
                rlevel = Device_SwitchSys(SYS_EXCHANGE);
                if (!Exchange_isConnected()) {

                    Exchange_Initialize(__DEVICE_DIN, __DEVICE_NET_ID, __DEVICE_VER); // DIN, lpeer, hwver.. 802.15.4 PanID ????
                    // while (1) { printf("Ready !\n"); } // Test UART

                    if (Device_IsWiredLinked()) {
                        Exchange_Connect(CNL_WIRED, 0, 0, 0); //ch,rpeer,skey,mode 0-RLY 1-RT
                    } else {
                        Exchange_Connect(CNL_WIRELESS, MRF24J40_DONGLE_ADDRESS, 0, 0); //ch,rpeer,skey,mode 0-RLY 1-RT
                    }
                    exRTCmd = RTCMD_NONE;
                }

                while (Exchange_isConnected()) {
                    //dprint("Ex Cycle!\n");

                    switch (Exchange_commandsHandler(&exRTCmd)) {

                        case 0: // Exit (normal)
                            Exchange_Disconnect(); // Exit
                            state = WAITING;
                            break;

                        case 1: // Start Exchange-RT
                            break;

                        case 2: // Exit req.exec(exRTCmd)

                            //                rtDataSize = 0;
                            //                switch (exRTCmd) {
                            //                    case RTCMD_TEST:
                            //                        rtDataBuffer[rtDataSize++] = g_dev.cnf.general.typeset && 0xFF; // Ret typeset
                            //                        rtDataBuffer[rtDataSize++] = (g_dev.cnf.general.typeset >> 8) && 0xFF;
                            //                        //                        rtDataBuffer[rtDataSize++] = 'H';
                            //                        //                        rtDataBuffer[rtDataSize++] = 'E';
                            //                        //                        rtDataBuffer[rtDataSize++] = 'L';
                            //                        //                        rtDataBuffer[rtDataSize++] = 'L';
                            //                        //                        rtDataBuffer[rtDataSize++] = 'O';
                            //                        break;
                            //                    case RTCMD_START_MEASUREMENT:
                            //                        rtDataBuffer[rtDataSize++] = 0;
                            //                        lstate = state;
                            //                        state = SAMPLING;
                            //                        break;
                            //                    default:
                            //                        break;
                            //                }
                            // bool Exchange_SendResponse(uint16_t dataSize);
                            // No Disconnect !!

                            break;

                        case 6: // Exit req. reset
                            Exchange_Disconnect();
                            // HARDWARE RESET 
                            state = STARTUP;
                            break;

                        case 10: // Exit req.sleep
                            Exchange_Disconnect();
                            state = WAITING;
                            break;

                        case 11: // Exit req.initialize
                            // DEFAULT SETTINGS
                            // HARDWARE RESET 
                            state = STARTUP;
                            break;
                    }
                }; // Exchange_isConnected()
                Device_SwitchSys(rlevel);

                if (device.cnf.exchange.attempt_mode != CNF_ATTEMPTMODE_EVERYCYCLE) {
                    // Schedule wake-up time to attempts exchange 
                }
                break;

            case WAITING:
                //lstate = EXCHANGE_RT;
                lstate = state;
                state = SAMPLING;
                //state = EXCHANGE;

                if (Device_IsWiredLinked()) {
                    __delay(5000); // wait 5 secs and restart cycle
                } else { //  

                    RTCC_GetTime(&stime);
                    stime.sec += device.cnf.general.delaytime % 60; // secs
                    stime.min += device.cnf.general.delaytime / 60; // minutes
                    if (stime.min > 59) {
                        stime.min = stime.min % 60;
                    }
                    RTCC_AlarmSet(&stime); // Enable RTCC Wake-Up
                    Device_SwitchSys(SYS_SLEEP); //  ...wait event (INT0/RTCC)
                    RTCC_AlarmUnset();
                    Device_SwitchSys(SYS_DEFAULT);
                }
                break;
        }
        __clearWDT();
        Device_StatusRefresh();
    } // End VAMP1K

    return (0);
} // End Main


#include <string.h>


/* -------------------------------------------------------------------------- */
/* Service Syncronize                                                         */

/* -------------------------------------------------------------------------- */
bool cb_GetDateTime(uint32_t *ltime, uint16_t * tzone) {

    *ltime = RTCC_GetTimeL();
    *tzone = 1;
    return true;
};

void cb_SetDateTime(uint8_t * rxData) {
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

};


/* -------------------------------------------------------------------------- */
/* Service Availability                                                       */
/* -------------------------------------------------------------------------- */
uint8_t cb_GetDeviceState(uint8_t *dobj) {
    uint8_t nbyte = 0;
    Device_StatusRefresh();
    *dobj = (int) &device.sts;
    nbyte = sizeof (device.sts);
    return (nbyte);
};

/* -------------------------------------------------------------------------- */
/* Service Alignment                                                          */
/* -------------------------------------------------------------------------- */

bool cb_GetDeviceConfigCRC16(uint16_t * crc16) {
    *crc16 = device.cnf.CRC16;
    return (true);
};

void cb_SetDeviceConfig(uint8_t *dobj) {
    Device_ConfigWrite(dobj);
};

uint8_t cb_GetDeviceConfig(uint8_t *dobj) {
    uint8_t nbyte = 0;
    if (Device_ConfigRead(&device.cnf)) {
        *dobj = (int) &device.cnf;
        nbyte = sizeof (device.cnf);
    }
    return (nbyte);
};

/* -------------------------------------------------------------------------- */
/* Service Transfer                                                           */
/* -------------------------------------------------------------------------- */
bool cb_GetMeasurementCounter(uint16_t * nobj) {
    *nobj = 1; //measurementCounterGet(); // measurementsCounter();
    return (true);
};

bool cb_GetMeasurement(uint16_t index, uint32_t * dtime, uint16_t * tset, uint16_t * ns, uint16_t * nss) { // ** pointer !!!!!!!!!!!!!!!!!!!!!!!!!
    //uint16_t Depot_ReadPart(uint16_t index, uint16_t offset, uint8_t* pbuf, uint16_t bsize); // 0 error, >0 Readed bytes
    if (measurementLoad(index) == index) {
        //        *dtime = measurementGetPTR()->tset;
        //        *tset = (uint16_t) measurementGetPTR()->tset;
        //        *ns = measurementGetPTR()->ns;
        //        *nss = measurementGetPTR()->nss;
        *dtime = lmeas.dtime;
        *tset = (uint16_t) lmeas.tset;
        *ns = lmeas.ns;
        *nss = lmeas.nss;
        //memcpy(meas, measurementGetPTR(), sizeof(measurement_t) ); // Sample size 2Byte
        return (true);
    };
    return (false);
};

void cb_GetMeasurementBlock(uint8_t *pbuf, uint16_t offset, uint16_t size) {
    memcpy(pbuf, &(lmeas.ss[offset]), size); // Sample size 2Byte
    //uint16_t Depot_ReadPart(uint16_t index, uint16_t offset, uint8_t* pbuf, uint16_t bsize); // 0 error, >0 Readed bytes
};

void cb_DeleteMeasurement(uint16_t index) {
    //  depot_delete(index);
};

/* -------------------------------------------------------------------------- */


#endif