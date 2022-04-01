/******************************************************************************\
|                                                                              |
|                         V A M P -  R E C O R D E R                           |
|                           ver. 0.1.17 - 03/14/22                             |
|                                                                              |
|                                                             (@)2021 DTeam !  |
\******************************************************************************/

//------------------------------------------------------------------------------
#include "bits.h"                   // PIC Settings
#include "test.h"
#include "modules/RTCC.h"           // hal           
#include "modules/UART2.h"          // hal
#include "device.h"                 // system 
#include "utils.h"                  // system
#include "sampling/measurement.h"
#include "daas/libex.h"             // DaaS
//------------------------------------------------------------------------------ 
device_t device;
sample_t SSBUF[SS_BUF_SIZE]; // global buffer
measurement_t lmeas;
uint32_t lstime = 0, DSTIME; // last sampling 
//------------------------------------------------------------------------------

#ifdef __VAMP1K_TEST 
#include "test.c"  
#else  

int main(void) {

    runlevel_t rlevel; // Use: device.st.llevel
    RealTimeCommandType exRTCmd;
    devicestate_t state, lstate; // Main cycle controls
    state = STARTUP; // System startup

    while (1) {

        switch (state) {
                //------------------------------------------------------------------
            case STARTUP: // Executed only after Power-On/Reset
                Device_SwitchSys(SYS_BOOT); // initialize hardware
                while (!Device_ConfigRead()) // persistent config
                {
                    Device_ConfigDefaultSet(); // Factory default....
                    Device_StatusDefaultSet();
                    depotDefaultSet(); // initialize memory
                }
                Device_StatusRead(); // set status object
                DSTIME = device.cnf.general.delaytime; // Compute delaytime in long format
                state = EXCHANGE; // first try to connect
                //__delay(1000); // !!!! usb stabilizes
                break;

                //------------------------------------------------------------------
            case SAMPLING:

                // if ( g_dev.cnf.samplingmode != CNF_SAMPLING_OFF )
                //!! RTCC_TimeGet(&stime);

                // if ((stime.lstamp > g_config.general.startdate) && (stime.lstamp < g_config.general.stopdate)) {

                //   if ((device.sts.timestamp - lstime) > DSTIME) { // Check if delaytime is elapsed 
                Device_StatusRefresh(); // ??????????????
                
                measurementAcquire(device.sts.timestamp, device.cnf.general.typeset); // Uses Device_SwitchSys()
                measurementSave();
                lstime = device.sts.timestamp;

                //   }

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
                state = WAITING;
                rlevel = Device_SwitchSys(SYS_EXCHANGE);
                if (!Exchange_isConnected()) {

                    Exchange_Initialize(__DEVICE_DIN, __DEVICE_NET_ID, __DEVICE_VER); // DIN, lpeer, hwver.. 802.15.4 PanID ????
                    // while (1) { printf("Ready !\n"); } // Test UART

                    if (Device_IsWireLinked()) {
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
                lstate = state;
                state = SAMPLING; // Smpling on wake-up
                //state = EXCHANGE;

                if (Device_IsWireLinked()) {
                    __delay(5000); // wait 5 secs and restart cycle
                } else { //  

                    ////                    RTCC_GetTime(&stime);
                    ////                    stime.sec += device.cnf.general.delaytime; // secs
                    ////                    if (stime.sec > 59) {
                    ////                        stime.min += (stime.sec) / 60; // minutes
                    ////                        stime.sec = (device.cnf.general.delaytime) % 60; // secs
                    ////                        if (stime.min > 59) {
                    ////                            stime.min = stime.min % 60;
                    ////                        }
                    ////                    };
                    ////                    //__builtin_disable_interrupts();
                    ////                    RTCC_AlarmSet(&stime); // Enable RTCC event to Wake-Up on time elapsed

                    RTCC_SetWakeup(device.cnf.general.delaytime);

                    IFS0bits.INT0IF = 0; // Enable INT0 event to Wake-Up on wire connect
                    IEC0bits.INT0IE = 1;
                    //
                    RCONbits.RETEN = 1;
                    RCONbits.VREGS = 1; // 
                    // Device_SwitchSys(SYS_SLEEP); //  switch off all 

                    Device_Power_Save();

                    Nop();
                    Sleep(); // enter in sleep mode
                    Nop();

                    Device_Power_Default();
                    Device_SwitchSys(SYS_DEFAULT);
                    //
                    IEC0bits.INT0IE = 0; // Disable INT0 (No change detection) 
                    RTCC_AlarmUnset(); // Disable Alarm
                    //__builtin_enable_interrupts();
                }
                break;
        }
        __clearWDT(); // Reset WDT
        Device_StatusRefresh();
    } // End VAMP1K

    return (0);
} // End Main



#include <string.h>

/***************************************************************************** 
 *                              D a a S - Lib-Ex                             *  
 *****************************************************************************/


/******************************************************************************
 * Mapping                                                                    *
 *****************************************************************************/

/******************************************************************************
 * Syncronize                                                                 *
 *****************************************************************************/
bool cb_GetDateTime(uint32_t *ltime, uint16_t * tzone) {
    *ltime = RTCC_GetTimeL();
    *tzone = device.cnf.general.timezone;
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
    // device.cnf.general.timezone = ???
    device.sts.timestamp = RTCC_GetTimeL(); // Update status object
};

/******************************************************************************
 * Avability                                                                  *
 *****************************************************************************/
uint8_t cb_GetDeviceState(uint8_t *dobj) {
    uint8_t nbyte = 0;
    Device_StatusRefresh();
    *dobj = (int) &device.sts;
    nbyte = sizeof (device.sts);
    return (nbyte);
};

/******************************************************************************
 * Alignment                                                                  *
 *****************************************************************************/
bool cb_GetDeviceConfigCRC16(uint16_t * crc16) {
    *crc16 = device.cnf.CRC16;
    return (true);
};

void cb_SetDeviceConfig(uint8_t *dobj) {
    if (Device_ConfigWrite(dobj)) {
        //Device_ConfigDefaultSet(); // Factory default....
        Device_StatusDefaultSet();
        depotDefaultSet(); // FORCE !!!!   Initialize memory
        Device_ConfigRead();
        Device_StatusRead();
        DSTIME = device.cnf.general.delaytime; // Compute delaytime in long format

    }
};

uint8_t cb_GetDeviceConfigPtr(uint8_t **dobj) {
    uint8_t nbyte = sizeof (config_t);
    *dobj = (uint8_t*) (&device.cnf);
    return (nbyte);
};

/******************************************************************************
 * Transfer                                                                   *
 *****************************************************************************/
bool cb_GetMeasurementCounter(uint16_t * nobj) {
    *nobj = device.sts.meas_counter;
    return (true);
};

bool cb_GetMeasurement(uint16_t index, uint32_t * dtime, uint16_t * tset, uint16_t * ns, uint16_t * nss) {
    __clearWDT(); // !!!!!!!!!!!!!!!
    if (measurementLoad(index)) { // == index
        //*dtime = 171717 + (index * 10); // TEST !!!!!!!!!
        *dtime = lmeas.dtime;
        *tset = (uint16_t) lmeas.tset;
        *ns = lmeas.ns;
        *nss = lmeas.nss;
        return (true);
    };
    return (false);
};

uint16_t cb_GetMeasurementBlock(uint8_t *pbuf, uint16_t offset, uint16_t size) {
    size = size * SAMPLE_SIZE_IN_BYTE;
    memcpy(pbuf, &(lmeas.ss[offset]), size); // Sample size 2Byte
    return (size);
};

void cb_DeleteMeasurement(uint16_t index) {
    measurementDelete(index);
};



/******************************************************************************
 * Security                                                                   *
 *****************************************************************************/

#endif