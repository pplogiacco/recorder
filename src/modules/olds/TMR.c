
#include <stdio.h>
#include "TMR.h"
#include "xc.h"


void (* TMR3_CallBack)(void) = NULL; // Interrupt Handler Default CallBack

void __attribute__((interrupt, no_auto_psv, weak)) _T3Interrupt() {
    if (TMR3_CallBack) { // Execute call-back to process buffer ( Slow process)
        TMR3_CallBack();
    }
    IFS0bits.T3IF = 0; // Reset vector
}

void TMR3_SetCallBack(void (* NewCallBack)(void)) { // Call after Enable
    T3CONbits.TON = 0; // Stop
    TMR3_CallBack = NewCallBack;
    IFS0bits.T3IF = 0; // Reset vector
    IEC0bits.T3IE = 1; // Enable Int call-back
}

void TMR3_Enable(uint16_t period) {

    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CON = 0x00; //Timer 3 Control Register
    TMR3_CallBack = NULL;
    T3CONbits.TCS = 0b00; // Internal clock (Fosc/2)
    T3CONbits.TCKPS = 0b01; // 1:8 Prescale value ( Fosc/2/8 = 500Khz )
    TMR3 = 0x00; //TMR3 Timer3 Counter Register
    PR3 = period; //Period = 1ms,500Khz ->PR3=499;
    T3CONbits.TGATE = 0; // Gated time accumulation is disabled

    IFS0bits.T3IF = 0; // Reset Int vector
    IEC0bits.T3IE = 1; // Enable Int call-back
    //T3CONbits.TON = 1; // Starts 16-bit Timery
}

void TMR3_EnableADC(uint16_t period) {
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CON = 0x00; // TIMER3 Control Register
    TMR3_CallBack = NULL;
    T3CONbits.TCS = 0b00; // Internal clock (FOSC/2)
    T3CONbits.TCKPS = 0b01; // 1:8 Prescale value  ( Fosc/8 = 500Khz )
    TMR3 = 0x00; // TMR3 Timer3 Counter Register
    PR3 = period; // 0x1F3;            // Period = 0.001 s; 500Khz/PR3(499);

    T3CONbits.TGATE = 0; // Gated time accumulation is disabled
    IEC0bits.T3IE = 0; // Doesn't set Int/call-back, event is handled by the ADC
}

void TMR3_Disable(void) {
    T3CONbits.TON = 0; // Stop 16-bit Timery
    // disable module
}

inline void TMR3_Start(void) {
    T3CONbits.TON = 1;
};

inline void TMR3_Stop(void) {
    T3CONbits.TON = 0;
};

inline uint16_t TMR3_Counter(void) {
    return (TMR3);
};

inline void TMR3_Reset(void) {
    T3CONbits.TON = 0;
    TMR3 = 0;
    T3CONbits.TON = 1;
};

/* TMR1 ------------------------------------------------------------------------
void TMR1_Init_2Hz(void) {
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; // Select internal instruction cycle clock
    T1CONbits.TGATE = 0; // Disable Gated Timer mode
    T1CONbits.TCKPS = 0b10; // 1:64 Fcy
    //0b011; // Select 1:8 Prescaler
    TMR1 = 0x00; // Clear timer register
    PR1 = 9; // Load the period value
    IPC0bits.T1IP = 0x07; // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer1 interrupt
    T1CONbits.TON = 1; // Start Timer
}

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
   // Interrupt Service Routine code goes here
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
}
-------------------------------------------------------------------------------*/

