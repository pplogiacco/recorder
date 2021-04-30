/******************************************************************************\
| V A M P -  D O N G L E                                                       |
| Ver. 0.1.23                                                  (@)2021 DTeam ! |
\******************************************************************************/

//------------------------------------------------------------------------------
#include "bits.h"  // PIC Settings
#include <xc.h>
#include "device.h"
//
#include "utils.h"
#include "modules/RTCC.h"
#include "exchange/uart2.h"
//
#include "exchange/exchange.h"
#include "sampling/measurement.h"
#include "memory/flash302.h"

//------------------------------------------------------------------------------
// Global Device 
device_t g_dev;
timestamp_t stime; // Use: g_dev.st.lasttime

// Measurement
measurement_t g_measurement;

// Exchange 
exchangestate_t exchState = EXCH_OPEN; // Use: g_dev.st.exchangestatus
long attempt_last_time;
//------------------------------------------------------------------------------

int waittime = 0;

/*
volatile int tmr3trig = 0;
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
    tmr3trig = 1;
    IFS0bits.T3IF = 0; // ClearInt Flag
}
 */

int main(void) {

    uint16_t rtDataSize = 0;
    uint8_t *rtDataBuffer = Exchange_ptrSendData();

    RealTimeCommandType rtCommand;

    devicestate_t state, lstate; // Main cycle controls
    // g_dev.cnf.CRC16 = 0xFFFF; // Load default config

    Device_SwitchSys(SYS_BOOT); // Eval RCON, rtcc, pins, read config

    UART2_Initialize(); // Configure UART 

    RTCC_Init();

    state = STARTUP; // System startup
    lstate = state;

    attempt_last_time = 0;
    _TRISB2 = 0; // Out


#ifdef __DONGLE_PASSTHRU
    passthruMainLoop();
#endif


#ifdef __VAMP1K
    while (1) {

        switch (state) {
                //------------------------------------------------------------------
            case STARTUP: // Executed only after Power-On/Reset
                Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
                // g_dev.cnf.general.delaytime = 50; /// Override nvm check 
                //Device_GetStatus(&g_dev.st);
                //g_dev.st.alarm_counter = Device_CheckReset();
                state = EXCHANGE; // After start-up try to connect
                break;
                //------------------------------------------------------------------
            case SAMPLING:
                lstate = state;
                //!! RTCC_TimeGet(&stime);
                //!! if ((stime.lstamp > g_config.general.startdate) && (stime.lstamp < g_config.general.stopdate)) {
                measurementAcquire(&g_measurement); // Device_SwitchMode() 
                measurementSave(&g_measurement);
                //!!};
                if (lstate == EXCHANGE_RT) {
                    state = EXCHANGE_RT;
                } else {
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
                            if (!Exchange_Connect(Device_IsUsbConnected())) {
                                exchState = EXCH_EXIT;
                            } else {
                                exchState = EXCH_START_DISCOVERY;
                            }
                            break;

                        case EXCH_START_DISCOVERY:
                            //                            if (Exchange_sendHandshake( g_dev.st.DIN,g_dev.st.timestamp, g_dev.st.version, g_dev.cnf.CRC16 )) {
                            if (Exchange_sendHandshake()) {
                                exchState = EXCH_WAIT_COMMAND;
                            } else {
                                exchState = EXCH_EXIT; // Non collegato
                            }
                            break;

                        case EXCH_WAIT_COMMAND:
                            Exchange_commandsHandler(&rtCommand);
                            if (rtCommand == RTCMD_NONE) {
                                state = TOSLEEP;
                                exchState = EXCH_EXIT;
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
                } while (exchState != EXCH_EXIT);

                if (exchState != EXCH_SEND_COMMAND_RESPONSE) {
                    exchState = EXCH_OPEN;
                    Exchange_Disconnect();
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
                        rtDataBuffer[rtDataSize++] = (g_dev.cnf.general.typeset>>8) && 0xFF; 
//                        rtDataBuffer[rtDataSize++] = 'H';
//                        rtDataBuffer[rtDataSize++] = 'E';
//                        rtDataBuffer[rtDataSize++] = 'L';
//                        rtDataBuffer[rtDataSize++] = 'L';
//                        rtDataBuffer[rtDataSize++] = 'O';
//                        rtDataBuffer[rtDataSize++] = ' ';
//                        rtDataBuffer[rtDataSize++] = 'W';
//                        rtDataBuffer[rtDataSize++] = 'O';
//                        rtDataBuffer[rtDataSize++] = 'R';
//                        rtDataBuffer[rtDataSize++] = 'L';
//                        rtDataBuffer[rtDataSize++] = 'D';
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
                lstate = EXCHANGE_RT;

                // Device_Hybernate();
                // Device_SwitchPower(PW_SLEEP);
                // Device_SwitchPower(PW_ON_DEFAULT);
                if (g_dev.cnf.general.delaytime > 60) {
                    waittime = RTCC_GetMinutes();
                    while ((RTCC_GetMinutes() - waittime) < (g_dev.cnf.general.delaytime / 60)) {
                        __delay(1000);
                        __clearWDT();
                    }
                } else {
                    __delay(5000);
                }
                state = SAMPLING;
                /*
                if (g_dev.st.locked) {
                    state = SAMPLING;
                } else {
                    state = EXCHANGE;
                }
                 */
                break;
        }
        __clearWDT();
    } // End VAMP1K
#endif



    /*----------------------------------------------------------------------------*
     *  TEST
     * 
     *  
     *----------------------------------------------------------------------------*/

#ifdef __VAMP1K_TEST

    _TRISB2 = 0; // Out

    int i;
    //UART2_Initialize();
    UART2_Enable();

    RTCC_GetTime(&stime);
    printf("[%u:%u:%u]\n#:", stime.hour, stime.min, stime.sec);

#ifdef __VAMP1K_TEST_USB
    while (! Device_IsUsbConnected()) {
                printf("Waiting USB... \n");
        __delay(1000);
    }
    printf("USB Ready ! \n");
#endif
    
    
#ifdef __VAMP1K_TEST_CONFIG
    while (1) {
        Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
        __delay(5000);
    }
#else    
    Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
    //    Device_ConfigDefaultSet(&g_dev.cnf);
    //    Device_ConfigWrite((uint8_t*) & g_dev.cnf); // Write EEprom/Flash
#endif


#ifdef __VAMP1K_TEST_ADG
    while (1) {
        printf("ADG all on \n");
        Device_SwitchADG(0xFF); //_bs8(PW_WST) | _bs8(PW_ADA));
        __delay(3000);
        printf("ADG all off \n");
        Device_SwitchADG(0b0); //_bs8(PW_WST) | _bs8(PW_ADA));
                __delay(3000);
    }
#else
    Device_SwitchADG(0xFF); //_bs8(PW_WST) | _bs8(PW_ADA));
#endif      


#ifdef  __VAMP1K_TEST_RESET    

    if (RCONbits.WDTO) { // WDT Overflow Reset
        printf("WDT\n");
        RCONbits.WDTO = 0;
        // Increment "alarm_counter"
    } else if (RCONbits.BOR) { // Brown-out Reset
        printf("Bor\n");
        RCONbits.BOR = 0;
        // Increment "alarm_counter"
    } else if (RCONbits.DPSLP) { // Resume from Deep-sleep / DS Retention
        printf("D-S\n");
        RCONbits.DPSLP = 0;
        if (RCONbits.RETEN) {
            printf("RETEN\n");
            // RCONbits.RETEN = 0;
        }
    } else if (RCONbits.POR) { // Power Reset
        printf("POR\n");
        RCONbits.POR = 0;
    } else if (RCONbits.SWR) { // Power Reset
        printf("SwR\n");
        RCONbits.SWR = 0;
    }
    __delay(2000);
#endif 


    /*
        /// TEST RTCC
        stime.year = 21;
        stime.month = 02;
        stime.day = 06;
        stime.hour = 12;
        stime.min = 25;
        stime.sec = 1;
        // RTCC_TimeSet(&stime, 1);
     
     */

    /*
    /// _______________TEST TMR1 TIMEOUT
    setTimeout(2, 0);
    printf("Timeout: 2sec \n");
    while (!isTimeout()) { // Loop until cycle-time or full filled buffer
        __delay(10);
        printf(".");
    }
    printf("Tmr1 ok\n");

    /// _______________TEST TMR3 COUNTER
    AV_SYN_SetDigital();
    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)


    T3CONbits.TON = 1;
    //T3CON = 0x00; // Timer 3 Control Register
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CONbits.TCS = 1; // Clock Source 
    //1 = External clock from pin, TyCK (on the rising edge)
    //0 = Internal clock (FOSC/2)
    T3CONbits.TECS = 0b01; // bit 9-8: Timery Extended Clock Source (when TCS = 1)
    //11 = Generic timer (TxCK) external input
    //10 = LPRC Oscillator
    //01 = T3CK external clock input
    //00 = SOSC
    T3CONbits.TCKPS = 0b00; // Input Clock Prescale (00 = 1:1)
    TMR3 = 0x00; //TMR3 Timer3 Counter Register  
    PR3 = 10; // Count 
    IFS0bits.T3IF = 0; // Reset Int vector
    IEC0bits.T3IE = 1; // Int call-back 
    //T3CONbits.TON = 1;

    tmr3trig= false;
    while (1) {
        if (tmr3trig) {
            printf("!");
            tmr3trig = 0;
        }
        // printf("°");
    }

     */



    while (1) {
        switch (state) {
            case STARTUP: // Executed only after Power-On/Reset

                //g_dev.cnf.CRC16 = 0; // Set Default config
                Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
                //Device_GetStatus(&g_dev.st);
                //g_dev.st.alarm_counter = Device_CheckReset();
                //g_dev.cnf.general.typeset = _AV00;
                //printf("Delay %u\n", g_dev.cnf.general.delaytime);
                //printf("Cycle %u\n", g_dev.cnf.general.cycletime);
                state = SAMPLING; // After start-up try to connect
                break;

            case SAMPLING:
                lstate = state;
                // RTCC_TimeGet(&stime);
                //!! if ((stime.lstamp > g_config.general.startdate) && (stime.lstamp < g_config.general.stopdate)) {
                measurementAcquire(&g_measurement);
                //measurementSave(&g_measurement);
                //!!};
                state = (g_dev.cnf.exchange.attempt_mode == CNF_ATTEMPTMODE_EVERYCYCLE) ? EXCHANGE : TOSLEEP;
                break;

            case EXCHANGE:
                lstate = state;
                Device_StatusGet(&g_dev.st);
                //Device_SwitchPower(PW_ON_EXCHANGE);
                //mode = TOSLEEP;
                //mode = exchangeHandler();


#ifdef __VAMP1K_TEST_measurement_printf
                printf("EXCHANGE...\n");
                printf("---------------\n");
                printf("Typeset:%u\n", g_measurement.typeset);
                printf("Timestamp:%lu ", g_measurement.dtime);
                stime.lstamp = g_measurement.dtime;
                Timestamp2Time(&stime);
                printf("%u/%u/%u - %u:%u:%u \n", stime.day, stime.month, stime.year, stime.hour, stime.min, stime.sec);

                printf("ns:%u \n", g_measurement.ns);
                for (i = 0; i < g_measurement.ns; i++) {
                    printf("%u,", *(g_measurement.ss + i));
                };
                printf("\n");
                printf("nss:%u\n", g_measurement.nss);
                for (i = 0; i < (g_measurement.nss >> 1); i++) {
                    printf("%u,%u\n", *(g_measurement.ss + (i * 2) + g_measurement.ns), *(g_measurement.ss + ((i * 2) + 1) + g_measurement.ns));
                };
                printf("---------------\n\n");

#elif defined( __VAMP1K_TEST_measurement_DATAVIS )
                for (i = 4; i < (g_measurement.nss >> 1); i++) {
                    // printf("%u,%u\n", *(g_measurement.ss + (i * 2) + g_measurement.ns), *(g_measurement.ss + ((i * 2) + 1) + g_measurement.ns));
                    UART2_Write(0x5F);
                    UART2_Write(*(g_measurement.ss + (i * 2) + g_measurement.ns) & 0xFF);
                    UART2_Write(*(g_measurement.ss + (i * 2) + g_measurement.ns) >> 8);
                    UART2_Write(*(g_measurement.ss + ((i * 2) + 1) + g_measurement.ns) & 0xFF);
                    UART2_Write(*(g_measurement.ss + ((i * 2) + 1) + g_measurement.ns) >> 8);
                    UART2_Write(0xA0);
                    __delay(1);
                };
#endif
                state = TOSLEEP;
                //state = SAMPLING;
                break;

            case EXCHANGE_RT:
                lstate = state;
                lstate = EXCHANGE_RT;
                break;

            case TOSLEEP:
                lstate = state;
                // RTCC_WakeupSet(uint16_t period);
                // Device_SwitchPower(PW_SLEEP);
                // Device_SwitchPower(PW_ON_DEFAULT);
                state = SAMPLING; // EXCHANGE ??? 

                //waittime = RTCC_GetMinutes();
                //while ((RTCC_GetMinutes() - waittime) < (g_dev.cnf.general.delaytime / 60)) {
                //                 printf("%u - %u > %u\n",RTCC_GetMinutes(),waittime, (g_dev.cnf.general.delaytime / 60));
                //  __delay(5000);
                __delay(2000);
                __clearWDT();
                // }

                break;
        }
        __clearWDT();
    } // End VAMP1K

#endif

#ifdef __DONGLE_VAMP1K

    int elapsed = 0;
    //int waittime = 0;
    bool senddata = false;
    bool autosend = false;
    const int pushtime = 20; // 2 sec
    int delaytime = 2;

    IO_LED1_Off(); // Flash Leds
    IO_LED2_On();
    while (elapsed++ < pushtime) {
        IO_LED1_Toggle();
        IO_LED2_Toggle();
        __delay(50);
    }
    Device_ConfigRead(&g_dev.cnf); // Read from EEprom/Flash
    Device_StatusGet(&g_dev.st);
    elapsed = 0;
#ifdef __DONGLE_VAMP1K_AUTOSEND    
    IO_LED1_On(); // Start in AUTOSEND  !
    autosend = true;
#endif 
    IO_LED2_Off();

    //workaround prima misura non ha vibration
    g_dev.cnf.general.cycletime = 10;
    //

    while (1) {

        delaytime = (uint16_t) (g_dev.cnf.general.delaytime / 60); //
        //RTCC_TimeGet(&ctime);
        //printf("%u/%u/%u - %u:%u:%u \n", ctime.day, ctime.month, ctime.year, ctime.hour, ctime.min, ctime.sec);

        while (IO_SWC1_Value() && (elapsed++ <pushtime)) {
            __delay(100);
        }
        if (elapsed > pushtime) {
            IO_LED1_Toggle();
            autosend ^= 1;
            waittime = RTCC_GetMinutes();
            elapsed = 0;
            __delay(500);
        }

        if (elapsed > 0) {
            senddata = true;
            elapsed = 0;
        }

        if (autosend) {
            if ((RTCC_GetMinutes() - waittime) >= delaytime) {
                waittime = RTCC_GetMinutes();
                senddata = true;
            }
        }

        if (senddata) {
            IO_LED2_On();
            measurementAcquire(&g_measurement);
            measurementSave(&g_measurement);

            exchangeHandler();
            IO_LED2_Off();
            senddata = false;
        }
        __clearWDT();
    } // End

#endif

    return (0);
} // End Main

// ========================================================================== //

//while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur
// Wait for PLL to lock
// while (OSCCONbits.LOCK != 1);

// Unlock IO Pins
// __builtin_write_OSCCONL(OSCCON & _OSCCON_IOLOCK_MASK);

// the fastest way to represent a 8-bit
// value on a 16-bit device, such as Microchip's dsPIC line of products, is to use
// int_fast8_t from stdint.h.

// =============================================================================