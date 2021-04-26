
#include <stdio.h>
#include "ADC.h"

static unsigned long volatile ADC1_NSample = 0; // 32 bit samples counter
void (*ADC1_CallBack)(void) = NULL; // Interrupt Handler Default CallBack

void __attribute__((__interrupt__, auto_psv, weak)) _ADC1Interrupt(void) {
    if (ADC1_CallBack) { // Execute call-back to process buffer (slowly)
        ADC1_CallBack();
    }
    ADC1_NSample--;
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
}

void __attribute__((__interrupt__, auto_psv, weak)) _INT0Interrupt(void) {
    IFS0bits.INT0IF = 0;
}

/*
void __attribute__((__interrupt__, auto_psv, weak)) _AltT1ADC1Interrupt(void) {

    // ********** Begin Fast Processing routine P-P / FFT ************
    //  Alternate Interrupt Handler !!!
    // ************ End Fast Processing routine P-P / FFT ************
    ADC1_NSample--;
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
}
 */


#define ADC_CH1     0x01
#define ADC_CH_LM35 0x02

void ADC1_SetCallBack(void (* NewCallBack)(void)) { // Call after Enable
    AD1CON1bits.ADON = 0; // Converter off
    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
    ADC1_CallBack = NewCallBack;
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
}

void ADC1_Init(uint8_t channel) { // Configure ADC1 PIC24FV32KA304 FAMILY

    AD1CON1bits.ADON = 0; // Converter off
    ADC1_CallBack = NULL;
    AD1CON2 = 0; // Inputs are not scanned
    AD1CHS = 0;
    AD1CSSL = 0; // No inputs are scanned. //ADC1MD bit in the PMD1

    switch (channel) {
        case ADC_CH1:

            // ____________________________________AD analog input pins
            ANSAbits.ANSA0 = 1; //  RA0 AN0 (2 DIP20) VRef+
            ANSAbits.ANSA1 = 1; //  RA1 AN1 (3 DIP20) VRef-
            TRISAbits.TRISA0 = 0;
            TRISAbits.TRISA1 = 0;

            // ____________________________________AD voltage references
            // Differential mode (Vref+ != Vcc)
            AD1CON2bits.PVCFG = 0; // 01 = External ( Vdd/Vref+ Pin AN0 )
            AD1CON2bits.NVCFG = 0; // 00 = Internal ( Vss/Vref- Pin AN1)

            // ____________________________________AD input channels
            AD1CHSbits.CH0SA = 1; // S/H+ input
            // 010 = AN1
            // 001 = AN0
            // 000 = AVSS
            AD1CHSbits.CH0NA = 2; // S/H- input
            // 00000 = AN0
            // 00010 = AN1
            // 11101 = AVSS (29)
            break;
    }

    // ____________________________________AD clock and conversion mode
    AD1CON1 = 0x2200; // No operation in Idle mode (ADSIDL=1)

    AD1CON1bits.FORM = 1; // 00 = Absolute decimal result, unsigned, right-justified
    AD1CON1bits.ASAM = 1; // Auto-Convert ON ( end sampling and start conversion )
    AD1CON1bits.SSRC = 0b0111; // Internal counter (AUTO-CONVERT)
    // AD1CON1bits.SSRC = 2;    // Timer3 event ends sampling and starts conversion
    // AD1CON1bits.SSRC = 1;    // INT0 event
    // AD1CON1bits.SSRC = 0;    // Software mode ( set SAMP=1 )

    // ____________________________________AD buffering mode
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 1; // Starts filling the buffer at address ADCBUF0
    //AD1CON2bits.SMPI = 0b10000; // Interrupt on 16th sample/conversion cycle
    AD1CON2bits.SMPI = 7; // 0b11111; // Interrupts on 32th samples
    //
    // ____________________________________AD conversion timing
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
    // Fcy 4Mhz -> TCY = 250nS
    // 1+3 TAD extended SAMPLE-TIME + 12TAD(10b) CONVERSION-TIME
    // ADC-SEQUENCE-TIME = 16TAD * 4TCY * 250nS = 5uS
    //
    AD1CON3bits.EXTSAM = 1; // Extended Sampling Time bit
    // 1=A/D is still sampling after conversion (SAMP=0)
    //
    AD1CON3bits.SAMC = 31; // Auto-Sample Time TAD ( extended SAMPLE-TIME ?????, SAMPLE-TIME/CONVERSION-TIME ????? )
    // 11111 = 31 TAD
    // 00001 = 1 TAD...
    //
    AD1CON3bits.ADCS = 4; // A/D Conversion Clock Select bits ( 4Tcy=1TAD 250nS*4 = 1uS)
    // 00111111 = 64톂cy = TAD (0x3F )
    // 00000001 = 2톂cy = TAD
    // 00000000 = Tcy = TAD

    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
    // Configure A/D interrupt priority bits (AD1IP<2:0>) here, if required.
    // AD1CON1bits.ADON = 1;   // Converter On
}

void ADC1_EnableSR(uint8_t channel, uint16_t t) { // Uses TMR3
    TMR3_EnableADC(t); // Use Timer3 to perform AD conversion t=period(1Khz)
    if (channel > 0) { // Set ADC channel
        ADC1_Init(channel);
    }
    AD1CON1bits.SSRC = 0x2; // Timer 3 trig cycle ( sample and auto-conversion )
    TMR3_Start();
}

void ADC1_EnableINT0(uint8_t channel) {
    ADC1_Init(channel);
    //IEC0bits.INT0IE = 1;
    AD1CON1bits.SSRC = 0x1; // INT0 ( sample and auto-conversion )
    // IPC0bits.INT0IP = 1;  //  INT0I: INT0 - External Interrupt 0 ( priority: 1 )
}

inline void ADC1_Start(unsigned long ns) {
    ADC1_NSample = ns;
    AD1CON1bits.ADON = 1;
}

inline void ADC1_Stop() {
    AD1CON1bits.ADON = 0;
} // Free resosurces (TMR3)

inline bool ADC1_Running() {
    return (ADC1_NSample > 0);
}

void ADC1_Disable() {
    AD1CON1bits.ADON = 0;
    TMR3_Disable();
}





/*
void ADC_Init() {  // Configure ADC1 PIC24FV32KA304 FAMILY

  // Configure analog inputs and references (ANS<12:10>, ANS<5:0>).
  // Select voltage reference source (AD1CON2<15:13>).
  // Select clock to and data rate (AD1CON3<7:0>).
  // Select sample/conversion sequence (AD1CON1<7:4> and AD1CON3<12:8>).
  // Select how results are presented in the buffer (AD1CON1<9:8>).
  // Select the interrupt rate (AD1CON2<6:2>).

   // Set A0 A1 Analog Input Pins
    // ADC Pins:
    //  RA0 AN0 (2 DIP20) VRef+
    //  RA1 AN1 (3 DIP20) VRef-
    //
    TRISAbits.TRISA0 = 0; // Set Input
    TRISAbits.TRISA1 = 0;
    ANSAbits.ANSA0 = 1; // Set Analog
    ANSAbits.ANSA1 = 1;


    AD1CON1bits.ADON = 0;    // Converter off

    AD1CON1 = 0x2200;   // Configure sample clock source and conversion trigger mode.
                        // Unsigned Fraction format (FORM<1:0>=10),
                        // Manual conversion trigger (SSRC<3:0>=0000),
                        // Manual start of sampling (ASAM=0),
                        // No operation in Idle mode (ADSIDL=1)
                        // S/H in Sample (SAMP = 1)


    AD1CON1bits.FORM = 0; // 00 = Absolute decimal result, unsigned, right-justified

    AD1CON1bits.ASAM = 1;       // Auto conversion
    AD1CON1bits.SSRC = 0b0111;  // Internal counter ends sampling and starts conversion (AUTO-CONVERT)
    // AD1CON1bits.SSRC = 2;    // Timer3 event ends sampling and starts conversion;
    // AD1CON1bits.SSRC = 1;    // INT0

    // AD1CON1bits.ASAM = 0; // Auto conversion
    // AD1CON1bits.SSRC = 7; // Manual triggering (set SAMP=1 to start)



    AD1CON2 = 0;          // Configure A/D voltage reference and buffer fill modes
                          // Inputs are not scanned

    AD1CON2bits.PVCFG=1;  // 01 = External ( Vdd/Vref+ Pin AN0 )
    AD1CON2bits.NVCFG=1;  // 00 = Internal ( Vss/Vref- Pin AN1)

    AD1CON2bits.BUFREGEN =0;    // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 1;       // Starts filling the buffer at address ADCBUF0
    //AD1CON2bits.SMPI = 0b10000; // Interrupt on every 16th sample/conversion cycle
    AD1CON2bits.SMPI = 0b11111; // Interrupts at the completion of the conversion for each 32nd sample


    AD1CON3 = 0;            // A/D conversion timing
                            // ---------------------
    AD1CON3bits.ADRC = 0;   // Clock is derived from the system clock (TCY=1/FCY)
                            // FCY 4Mhz -> TCY = 250nS
                            // Sampling rate 1Khz --> T = 1ms = 4000 TCY !!!!!!!!!

                            // 10b-> 12TAD+31TAD extended -->
                            // ADC-SEQUENCE-TIME = 43TAD * 64TCY * 250nS = 688uS
                            // Sampling rate 1.453KHz Decimated 16 90.8 Hz

    AD1CON3bits.EXTSAM= 1;  // Extended Sampling Time bit
                            // 1=A/D is still sampling after SAMP=0

    AD1CON3bits.SAMC =  31; // Auto-Sample Time TAD
                            // 11111 = 31 TAD
                            // 00001 = 1 TAD...

    AD1CON3bits.ADCS =0x3F; // A/D Conversion Clock Select bits
                            // 00111111 = 64톂CY = TAD (0x3F )
                            // 00000001 = 2톂CY = TAD
                            // 00000000 = TCY = TAD



    AD1CHS = 0;             // Configure input channels
    AD1CHSbits.CH0SA = 1;   // S/H+ input
                            // 010 = AN1
                            // 001 = AN0
                            // 000 = AVSS

    AD1CHSbits.CH0NA = 2;   // S/H- input
                            // 00000 = AN0
                            // 00010 = AN1
                            // 11101 = AVSS

    AD1CSSL = 0;            // No inputs are scanned. //ADC1MD bit in the PMD1

    IFS0bits.AD1IF = 0;     // Clear A/D conversion interrupt.
    IEC0bits.AD1IE = 1;     // Enable A/D conversion interrupt
                            // Configure A/D interrupt priority bits (AD1IP<2:0>) here, if required.

   //  AD1CON1bits.ADON = 1;   // Converter On
}
 */



