
#include <stdio.h>
#include "CMP.h"


static uint16_t _IC2Mode = 0x0; //  Input capture module is turned off

void (* IC2_CallBack)(void) = NULL; // Interrupt Handler Default CallBack

void __attribute__((interrupt, no_auto_psv)) _ISR _IC2Interrupt(void) {
    if (IFS0bits.IC2IF) {
        IFS0bits.IC2IF = 0; // Reset interrupt flag (immediatelly)
        if (IC2_CallBack) { // Execute call-back routine
            IC2_CallBack();
        }
    }
}

void IC2_SetCallBack(void (*NewCallBack)(void)) { // Call after Enable
    IC2CON1bits.ICM = false; // Turn off module (clear ICM)
    IC2_CallBack = NewCallBack;
    IFS0bits.IC2IF = false; // Reset vector flag
    IEC0bits.IC2IE = true; // Enable Interrupt
}

void IC2_Init(void) {
    uint16_t x;

    IC2CON1 = 0x00; // Turn off the capture module (clear ICM)
    IC2CON2 = 0x0; //
    IEC0bits.IC2IE = false;
    IC2_CallBack = NULL;

    // Make sure that any previous data has been removed
    while (IC2CON1bits.ICBNE) {
        x = IC2BUF;
    }

    IC2CON1bits.ICSIDL = 0; // Input capture module halts in CPU Idle mode
    IC2CON1bits.IC2TSEL = 0; //Clock source for ICx module (TMR3)
    //! if clock source is running set ICTSELx before enable ICx

    // ____________________________________IC2(RB9) Digital IN
    _TRISB9 = 1; // digital IN
    //! disable all peripherals multiplexed with input pin
    IC2CON2bits.SYNCSEL = 0b10101; // Input Capture Src: (RB9,IC2)
    IC2CON1bits.ICI = 0b011; // Interrupt on every 4th capture event

    //! trigger mode: set ICTRIG and clear the TRIGSTAT bit
    IC2CON2bits.ICTRIG = 0; // Triggers ICx from SYNCSELx source event
    // 1 = Triggers ICx from SYNCSELx source event (src module's event!!)
    // 0 = Synchronizes ICx with source designated by the SYNCSELx bits

    IC2CON2bits.TRIGSTAT = 0; // Timer Trigger Status bit (force trig)
    // 1 = Timer source has been triggered and is running (set in hardware, can be set in software)
    // 0 = Timer source has not been triggered and is being held clear
    _IC2Mode = 0b100;
    // 100 = Prescaler Capture mode: Capture on every 4th rising edge
    // 101 = Prescaler Capture mode: Capture on every 16th rising edge

    //  IPC1bits.IC2IP = 3; // Set IC1 interrupt priority
    IFS0bits.IC2IF = false;
    IEC0bits.IC2IE = true;

}

void IC2_EnableT3RB2() {
    // TMR3_Enable();
    // TMR3_Start();
    IC2_Init();
}

inline void IC2_Start(void) {
    // IC2CON2bits.ICTRIG = 1;
    IC2CON1bits.ICM = 0b101; // _IC2Mode;
}

inline void IC2_Stop(void) {
    IC2CON1bits.ICM = 0; // Reset overflow, capture FIFO, prescale count
}

inline bool IC2_Running(void) {
    return (IC2CON1bits.ICM > 0);
}

inline uint16_t IC2_Read(void) {
    return (IC2BUF);
}

void IC2_Disable(void) {
    IC2CON1bits.ICM = 0;
    // TMR3_Disable();
}

/*
void IC1_ManualTriggerSet(void) {
    IC1CON2bits.TRIGSTAT = true;
}

bool IC1_TriggerStatusGet(void) {
    return ( IC1CON2bits.TRIGSTAT);
}

void IC1_TriggerStatusClear(void) {
    // Clears the trigger status
    IC1CON2bits.TRIGSTAT = 0;
}

bool IC1_HasCaptureBufferOverflowed(void) {
    return ( IC1CON1bits.ICOV);
}

bool IC1_IsCaptureBufferEmpty(void) {
    return ( !IC1CON1bits.ICBNE);
}
 */