
#include "utils.h"

/*******************************************************************************
T I M E O U T  
TMR1  
 *******************************************************************************/

volatile bool _timed = false;

void (*Timeout_CallBack)(void) = NULL; // Interrupt Handler Default CallBack

void Timeout_SetCallBack(void (* NewCallBack)(void)) { // Call after Enable
    Timeout_CallBack = NewCallBack;
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
    if (Timeout_CallBack) { // Execute call-back 
        Timeout_CallBack();
    } else {
        _timed = true;
    }
}

void Timeout_Set(uint16_t nsec, uint16_t nms) {
    if ((nsec + nms) > 0) { // set Timeout
        _timed = false;

#if defined(__PIC24FJ256GA702__)

        T1CON = 0;
        T1CONbits.TCS = 1; // clock source is selected by T1ECS
        T1CONbits.TECS = 0b10; // uses the LPRC (31.25 kHz) as the clock source
        TMR1 = 0; // Clear timer register 
        if (nsec > 0) { // Scount sec and ignore ms
            T1CONbits.TCKPS = 0b01; // Select 1:8 Prescaler   
            PR1 = ( nsec * 0xF22); // Tick = 1s
        } else { // count ms
            T1CONbits.TCKPS = 0b00; // Select 1:1 (LPRC 1/31Khz * 0x1E = )
            PR1 = (nms * 0x1E); // ( 0x1E = 125 = 1ms period )
        }
        IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
        IEC0bits.T1IE = 1; // Enable interrupt
        T1CONbits.TON = 1; // Enable Timer

#else
        T1CON = 0;
        T1CONbits.TCS = 1; // clock source is selected by T1ECS
        T1CONbits.T1ECS = 0b10; // uses the LPRC (31.25 kHz) as the clock source
        TMR1 = 0; // Clear timer register 
        if (nsec > 0) { // Scount sec and ignore ms
            T1CONbits.TCKPS = 0b11; // Select 1:256 Prescaler (LPRC/256=125Hz)  
            PR1 = nsec * 125; // Tick = 8ms
        } else { // count ms
            T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler (LPRC/64=500Hz)  (00 = 1:1)
            PR1 = (nms >> 1); // tick = 2ms
        }
        IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
        IEC0bits.T1IE = 1; // Enable interrupt
        T1CONbits.TON = 1; // Enable Timer
#endif        
    } else {
        _timed = true;
    }

}
// startTimeout()

bool isTimeout(void) { // 2ms x unit
    if (_timed) {
        T1CONbits.TON = 0; // Disable Timer    
    }
    return (_timed);
}

// stopTimeout()

void Timeout_Unset(void) {
    IEC0bits.T1IE = 0; // Disable interrupt
    T1CON = 0; // Disable Timer   
    Timeout_CallBack = NULL;
}

void RTCC_SetWakeup(uint16_t period) {
    // Get current time
    // if ( attemptmode>0 ) {   // Wake-up to exchange
    // if ( samplingmode==0 ) {   // Wake-up to sampling
    // Power is ok ?

    // set alarm to current time + delaycycle   

}

/*

 void RTCC_Initialize(void)
{
    // Set the RTCWREN bit
    __builtin_write_RTCWEN();

    RCFGCALbits.RTCEN = 0;

    if(RCON2bits.VBAT & RCONbits.POR)
    {
        if(!RTCCTimeInitialized())
        {
            // set RTCC time 2016-12-08 09-22-03
            RCFGCALbits.RTCPTR = 3; // start the sequence
            RTCVAL = 0x16; // YEAR
            RTCVAL = 0x1208; // MONTH-1/DAY-1
            RTCVAL = 0x409; // WEEKDAY/HOURS
            RTCVAL = 0x2203; // MINUTES/SECONDS
        }
    }

    RCON2bits.VBAT = 0;
    RCONbits.POR = 0;

    // RTCOUT Alarm Pulse; PWSPRE disabled; RTCLK SOSC; PWCPRE disabled; PWCEN disabled; PWCPOL disabled;
    RTCPWC = 0x0000;

    // Enable RTCC, clear RTCWREN
    RCFGCALbits.RTCEN = 1;
    RCFGCALbits.RTCWREN = 0;

    return;
}

 */