/******************************************************************************\
| V A M P -  R E C O R D E R                                                   |
| Ver. 0.0.19                                                  (@)2021 DTeam ! |
\******************************************************************************/

//------------------------------------------------------------------------------
#include "bits.h"  // PIC Settings
#include "device.h"
//
#include "utils.h"
#include "modules/RTCC.h"
#include "modules/UART2.h"
//
#include "exchange/exchange.h"
#include "sampling/measurement.h"
#include "memory/storage.h"
#include "memory/SST26VF064B.h"

//------------------------------------------------------------------------------
// Global Device 
device_t g_dev;
//timestamp_t stime; // Use: g_dev.st.lasttime

// Measurement
measurement_t g_measurement;

// Exchange 
exchangestate_t exchState = EXCH_OPEN; // Use: g_dev.st.exchangestatus
long attempt_last_time;
//------------------------------------------------------------------------------

int waittime = 0;

#ifdef __VAMP1K_TEST
#include "test.c"
#else

int main(void) {

    uint16_t rtDataSize = 0;
    uint8_t *rtDataBuffer = ptrSendData();
    RealTimeCommandType rtCommand;

    devicestate_t state, lstate; // Main cycle controls
    timestamp_t stime;
    
    Device_SwitchSys(SYS_BOOT); // Eval RCON, rtcc, pins, read config

    state = STARTUP; // System startup
    lstate = state;

    while (1) {

        switch (state) {
                //------------------------------------------------------------------
            case STARTUP: // Executed only after Power-On/Reset
                Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
                //g_dev.cnf.general.delaytime = 50; /// Override nvm check 
                //Device_GetStatus(&g_dev.st);
                //g_dev.st.alarm_counter = Device_CheckReset();
                state = EXCHANGE; // After start-up try to connect
                break;
                //------------------------------------------------------------------
            case SAMPLING:
                //lstate = state;
                //state = SAMPLING;
                // if ( g_dev.cnf.samplingmode != CNF_SAMPLING_OFF )
                //!! RTCC_TimeGet(&stime);
                //!! if ((stime.lstamp > g_config.general.startdate) && (stime.lstamp < g_config.general.stopdate)) {
                measurementAcquire(&g_measurement);
                measurementSave(&g_measurement);
                //!!};
                if (lstate == EXCHANGE_RT) {
                    lstate = state;
                    state = EXCHANGE_RT;
                } else {
                    lstate = state;
                    // state = (g_dev.cnf.exchange.attempt_mode == EXCHANGE_ATTEMPTMODE_EVERYCYCLE) ? EXCHANGE : TOSLEEP;
                    state = EXCHANGE;
                }

                break;
                //------------------------------------------------------------------

            case EXCHANGE:
                lstate = state;
                Device_StatusGet(&g_dev.st);
                Device_SwitchSys(SYS_ON_EXCHANGE);

                do {
                    switch (exchState) {
                        case EXCH_OPEN:
                            if (!Exchange_Connect( Device_IsUsbConnected() )) {
                                exchState = EXCH_EXIT;
                            } else {
                                exchState = EXCH_START_DISCOVERY;
                            }
                            break;

                        case EXCH_START_DISCOVERY:
                            // if (Exchange_sendHandshake( g_dev.st.DIN, g_dev.st.timestamp, g_dev.st.version, g_dev.cnf.CRC16 )) {
                            if (Exchange_sendHandshake()) {
                                exchState = EXCH_WAIT_COMMAND;
                            } else {
                                exchState = EXCH_EXIT; // Non collegato
                            }
                            break;

                        case EXCH_WAIT_COMMAND:
                            Exchange_commandsHandler(&rtCommand);

                            if (rtCommand == RTCMD_NONE) { // RTCMD_TERMINATE
                                state = TOSLEEP;
                                exchState = EXCH_EXIT;
                                // not until disconnect request...
                            } else {
                                rtDataSize = 0;
                                state = EXCHANGE_RT;
                                exchState = EXCH_SEND_COMMAND_RESPONSE;
                            }
                            break;

                        case EXCH_SEND_COMMAND_RESPONSE:
                            if (Exchange_commandSendResponse(rtDataSize)) {
                                exchState = EXCH_WAIT_COMMAND;
                            } else {
                                exchState = EXCH_EXIT;
                            }
                            break;

                        default:
                            break;
                    }
                    if (exchState == EXCH_SEND_COMMAND_RESPONSE) {
                        break;
                    }
                    __clearWDT();
                } while (exchState != EXCH_EXIT);

                if (exchState != EXCH_SEND_COMMAND_RESPONSE) {
                    exchState = EXCH_OPEN;
                    Exchange_Disconnect();
                    Device_SwitchSys(SYS_DEFAULT);
                }

                if (g_dev.cnf.exchange.attempt_mode != CNF_ATTEMPTMODE_EVERYCYCLE) {
                    // Scheduled
                    // Set wake-up time to attempt exchange 
                }

                break;
                //------------------------------------------------------------------
            case EXCHANGE_RT:
                lstate = state;
                state = EXCHANGE_RT;

                rtDataSize = 0;
                switch (rtCommand) {
                    case RTCMD_TEST:
                        rtDataBuffer[rtDataSize++] = g_dev.cnf.general.typeset && 0xFF;
                        rtDataBuffer[rtDataSize++] = (g_dev.cnf.general.typeset >> 8) && 0xFF;
                        //                        rtDataBuffer[rtDataSize++] = 'H';
                        //                        rtDataBuffer[rtDataSize++] = 'E';
                        //                        rtDataBuffer[rtDataSize++] = 'L';
                        //                        rtDataBuffer[rtDataSize++] = 'L';
                        //                        rtDataBuffer[rtDataSize++] = 'O';
                        break;
                    case RTCMD_START_MEASUREMENT:
                        rtDataBuffer[rtDataSize++] = 0;
                        lstate = state;
                        state = SAMPLING;
                        break;
                    default:
                        break;
                }
                break;
                //------------------------------------------------------------------

            case TOSLEEP:
                //lstate = EXCHANGE_RT;
                lstate = state;
                state = SAMPLING;
                //state = EXCHANGE;

                if ( Device_IsUsbConnected() ) {
                    //                    if (g_dev.cnf.general.delaytime > 60) {
                    //                        waittime = RTCC_GetMinutes();
                    //                        while ((RTCC_GetMinutes() - waittime) < (g_dev.cnf.general.delaytime / 60)) {
                    //                            __delay(1000);
                    //                            __clearWDT();
                    //                        }
                    //                    } else {
                    //                        __delay(5000);
                    //                    }
                    //state = SAMPLING;
                    __delay(10000);
                    
                } else { //  
                    // Enable USB Wake-Up
                    // Enable RTCC Alarm Wake-Up
                    RTCC_GetTime(&stime);
                    stime.sec += g_dev.cnf.general.delaytime % 60;
                    if (stime.sec > 59) {
                        stime.sec -= 59;
                        stime.min++;
                    }
                    stime.min += g_dev.cnf.general.delaytime / 60;
                    if (stime.min > 59) {
                        stime.min -= 59;
                    }
                    RTCC_AlarmSet(&stime);  // RTTC_SetWakeUp ( delay_s );
                    
                    Device_SwitchSys(SYS_SLEEP);
                    //  ... wait wake-up event
                    Device_SwitchSys(SYS_DEFAULT);
                } 
                break;
        }
        __clearWDT();
    } // End VAMP1K

    return (0);
} // End Main

#endif