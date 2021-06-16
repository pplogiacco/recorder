
/*----------------------------------------------------------------------------*
 *  TEST !!!!!!!                                                              *              
 *----------------------------------------------------------------------------*/
#ifdef __VAMP1K_TEST
#include "utils.h"
#include "modules/RTCC.h"


// Test HW
//#define __VAMP1K_TEST_HW                      // Perform hardware test
//#define __VAMP1K_TEST_RESET
#define __VAMP1K_TEST_TIMERS
//#define __VAMP1K_TEST_CONFIG 
// #define __VAMP1K_TEST_ADG
// #define __VAMP1K_TEST_USB
//#define __VAMP1K_TEST_BATTERY
//#define __VAMP1K_TEST_SST26
// #define __VAMP1K_TEST_SLEEP
// #define __VAMP1K_TEST_RTCC

#define __VAMP1K_TEST_measurement_printf

//#define __AV0NVM     // Save samples in flash



//volatile int int0trig = 0;
//void __attribute__((weak)) EX_INT0_CallBack(void) {
//}



// TEST TMR3
volatile int tmr3trig = 0;
//void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
//    tmr3trig = 1;
//    IFS0bits.T3IF = 0; // ClearInt Flag
//}
//
// TEST TMR2
volatile int tmr2trig = 0;
//
//void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
//    tmr2trig = 1;
//    IFS0bits.T2IF = 0; // ClearInt Flag
//}

// TEST RTTC
volatile int alarm_event = 0;
timestamp_t stime;

void Alarm(void) {
    printf("Alarm !\n");
    stime.sec += 5;
    RTCC_AlarmSet(&stime);
    //printf("Alarm set: %u:%u:%u \n", stime.hour, stime.min, stime.sec);
}


// WINDSPEED on TMR2
//volatile int tmr2_wsready = 0;
//volatile int tmr2_icycle = 0;
//volatile int tmr2_wsptime[5];
//void tmr2_wscapture(void) {
//    if (tmr2_wsready) { // Stop
//        tmr2_wsptime[tmr2_icycle] = TMR2;
//        tmr2_wsready = 0; // re-Start
//        tmr2_icycle++;
//    } else { // Start
//        tmr2_wsready = 1;
//        TMR2 = 0;
//    }
//}

int main(void) {

    uint16_t rtDataSize = 0;
    uint8_t *rtDataBuffer = ptrSendData();
    RealTimeCommandType rtCommand;

    devicestate_t state, lstate; // Main cycle controls

    Device_SwitchSys(SYS_BOOT); // Eval RCON, rtcc, pins, read config

    state = STARTUP; // System startup
    lstate = state;

    //    _TRISB2 = 0; // Out


    int i;

    UART2_Enable(); // printf()...)

    RTCC_GetTime(&stime);
    printf("[%u:%u:%u]\n#:", stime.hour, stime.min, stime.sec);

    
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
    
#ifdef __VAMP1K_TEST_USB
    while (1) {
        if (!Device_IsUsbConnected()) {
            printf("USB Waiting (RB7=%d)... \n",Device_IsUsbConnected());
        } else {
            printf("USB Connected (RB7=%d) !  \n",Device_IsUsbConnected());
        }
        __delay(1000);
    }
#endif


#ifdef __VAMP1K_TEST_ADG
    Device_SwitchSys(SYS_DEFAULT);

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


#ifdef __VAMP1K_TEST_TIMERS

    /// _______________TEST TMR1 TIMEOUT
    Timeout_Set(2, 0);
    printf("TMR1: Timeout = 2sec \n");
    while (!isTimeout()) { // Loop until cycle-time or full filled buffer
        __delay(10);
        printf(".");
    }
    printf("Tmr1 ok\n");


    /// _______________TEST TMR2 COUNTER  
    printf("TMR2: Tsrc=T2CK pin \n");
    WS_IN_SetDigitalInputLow();
    IEC0bits.T2IE = 0; // Disable Int
    T2CON = 0x00; // Reset TMR2, 16 Bit, No Prescaler
    T2CONbits.TCS = 1; //  Extended Clock Source (TECS))
    T2CONbits.TECS = 1; // T2CK pin 
    TMR2 = 0x00; // TMR2 Counter Register
    PR2 = 0x10; // TMR2 Counter
    IFS0bits.T2IF = 0; // Reset iflag
    IEC0bits.T2IE = 1; // Enable Int
    T2CONbits.TON = 1;
    
    tmr2trig = false;
    while (!tmr2trig) {
        if (tmr2trig) {
            printf("Tmr2 ok\n");
          //  tmr2trig = 0;
        }
        // printf("°");
    }


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

    /*
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>

    WS_IN_SetDigitalInputLow(); // Input RB2 (6) 

    IEC0bits.T2IE = 0; // Disable Int
    T2CON = 0x00; // Reset TMR2, 16 Bit, No Prescaler
    T2CONbits.TCS = 1; //  Extended Clock Source (TECS))
    T2CONbits.TECS = 1; // T2CK pin 
    TMR2 = 0x00; // TMR2 Counter Register
    PR2 = 0xFFFF; // TMR4 Single Event

    Timeout_SetCallBack(tmr2_wscapture);

    memset(&tmr2_wsptime, 0, sizeof (tmr2_wsptime));
    tmr2_icycle = 0;
    tmr2_wsready = 0;
    T2CONbits.TON = 1;

    while (1) {

        Timeout_SetCallBack(tmr2_wscapture);
        Timeout_Set(1, 0); // TMR1 1Second Start !
        while ((tmr2_icycle < 5)) {
            Nop();
        }
        Timeout_Unset();

        // T2CONbits.TON = 0;
        printf("---------------- \n");
        for (i = 0; i < 5; i++) {
            printf("cycles=%d, counter=%d \n", i, tmr2_wsptime[i]);
        }
        printf("\n\n");
        tmr2_icycle = 0;
    }
     */

    
#endif
    
    
    
    
    
    
    
    
    
    
    
#ifdef __VAMP1K_TEST_SST26

    Device_SwitchSys(SYS_ON_EXCHANGE); // SPI

    unsigned long sst_addr = 0;
    uint8_t datas[32];

    SST26_Enable();
    //SST26_Chip_Erase();
    //SST26_Switch_Power();
    //SST26_ResetEn();
    //SST26_Reset();
    SST26_Global_Block_Protection_Unlock();
    SST26_Wait_Busy();
    SST26_WREN();
    SST26_Block_Erase(sst_addr); // Set 4K in 0xFF state
    SST26_Wait_Busy();


    int man, typ, id;

    while (1) {
        printf("______SST26VF064B  Test: \n");
        SST26_Jedec_ID_Read(&man, &typ, &id);
        printf("manufacturer=%u  \n", man);
        printf("device_type=%u  \n", typ);
        printf("identifier=%u  \n", id);
        printf("config_reg=%u  \n", SST26_Read_Configuration());
        printf("status_reg=%u  \n", SST26_Read_Status());

        //        SST26_WREN();
        sst_addr = 0;
        //
        //        SST26_Wait_Busy();
        for (i = 0; i < 16; i++) {
            datas[i] = i;
        }
        //SST26_Page_Program(sst_addr, &datas[0]);
        SST26_WREN();
        SST26_Write_Bytes(sst_addr, datas, 16);
        printf("status=%u  \n", SST26_Read_Status());
        SST26_Wait_Busy();
        printf("Write page ok !\n");
        SST26_WRDI();

        printf("status=%u  \n", SST26_Read_Status());

        // RTCC_GetTimeL();
        //SST26_Read_Cont(sst_addr, 16, (unsigned int*) datas);
        SST26_Read_Bytes(sst_addr, 16, datas);
        for (i = 0; i < 16; i++) {
            //            printf("Read[%u]=%u \n", i, SST26_Read(sst_addr++));
            printf("Read[%u]=%u \n", i, datas[i]);
        }
        __delay(5000);
    }
    SST26_Disable();
    Device_SwitchSys(SYS_DEFAULT); // SPI
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


#ifdef __VAMP1K_TEST_BATTERY
    Device_SwitchSys(SYS_DEFAULT);

    while (1) {
        printf("Read battery level... \n");
        printf("level=%d  \n", Device_GetBatteryLevel());
        __delay(1000);
        // RTCC_GetTimeL();
    }

#endif

#ifdef __VAMP1K_TEST_SLEEP

    // RTCC Settings
    g_dev.cnf.general.delaytime = 20;
    if (1) {
        stime.day = 14;
        stime.month = 06;
        stime.year = 21;

        stime.hour = 12;
        stime.min = 25;
        stime.sec = 1;
        RTCC_SetTime(&stime, 1);
    }
    // Enable USB Wake-Up
    // Device_IsUsbConnected()

    while (1) {

        // Enable RTC Alarm Wake-Up
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
        RTCC_AlarmSet(&stime);
        printf("Wakeup set: %u:%u:%u \n", stime.hour, stime.min, stime.sec);

        printf("Put into sleep mode. (RTTC/INT0 Wake)... \n");
        __delay(100); // Wait to complete printf

        Device_SwitchSys(SYS_SLEEP);
        //  wait wake-up event...        
        // Reset USB Wake-up flag
        // 
        Device_SwitchSys(SYS_DEFAULT);
        UART2_Enable();

        //printf("Hello, wake-up ! \n");
        int i;
        int z;
        for (i = 0; i < 500; i++) {
            for (z = i; z < 500; z++) {
            }
        }
        RTCC_GetTime(&stime);
        printf("wake-up: %u/%u/%u - %u:%u:%u \n", stime.day, stime.month, stime.year, stime.hour, stime.min, stime.sec);
    }
#endif


#ifdef __VAMP1K_TEST_RTCC
    if (1) {
        stime.day = 14;
        stime.month = 06;
        stime.year = 21;

        stime.hour = 12;
        stime.min = 25;
        stime.sec = 1;
        RTCC_SetTime(&stime, 1);
    }

    printf("%u/%u/%u - %u:%u:%u \n", stime.day, stime.month, stime.year, stime.hour, stime.min, stime.sec);
    //    __delay(500);

    RTCC_SetCallBack(Alarm);
    g_dev.cnf.general.delaytime = 70;

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

    RTCC_AlarmSet(&stime);
    printf("Alarm set: %u:%u:%u \n", stime.hour, stime.min, stime.sec);

    while (1) {
        RTCC_GetTime(&stime);
        printf("%u/%u/%u - %u:%u:%u \n", stime.day, stime.month, stime.year, stime.hour, stime.min, stime.sec);
        //        if (alarm_event) {
        //            printf("Alarm...\n");
        //            alarm_event = false;
        //        }

        //printf("L1=%d \n",RTCC_GetTimeL(&stime));
        //printf("L2=%d \n",RTCC_GetTimeL2(&stime));
        //printf("waiting...\n");
        __delay(1000);
    }

#endif


   


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


    return (0);
} // End Main


#endif


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