#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>
#include "sampling.h"

#include "../utils.h"
#include "../modules/RTCC.h"





/*----------------------------------------------------------------------------*
 * L V D T  ( A E O L I A N  V I B R A T I O N )                              *   
 * acquireAV_INIT(adc_period)  --> tick_time                                  *
 *----------------------------------------------------------------------------*/

volatile uint8_t _adcReady;
volatile bool _cycletime;

void cycletimer(void) {
    _cycletime = false;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void) { // , weak
    IFS0bits.AD1IF = 0; // Clear Int flag immediatelly
    // _LATB2 ^= 1; // IO_LED2_Toggle() 
    _adcReady++;
}// End ADC

void acquireAV_INIT(uint16_t av_period) {

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // =============== BEGIN:INIT    
    // TMR3 as pulse counter on T3CK pin (SYNCO)
    AV_SYN_SetDigital();
    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CON = 0x00; // Timer 3 Control Register
    T3CONbits.TCS = 1; // clock source T3CK pin
    T3CONbits.TCKPS = 0; // Timer Input Clock Prescale 
    TMR3 = 0x00; //TMR3 Timer3 Counter Register  
    PR3 = (1U << (av_period + 3)) - 1; // 
    IFS0bits.T3IF = 0; // Reset Int vector
    IEC0bits.T3IE = 0; // Int call-back disabled (Tmr3 Trigs ADC)

#ifndef __HWDEVICE_V2_302 // Old sensor board

    // ____________________________________Input Analog pins
    /*TRISAbits.TRISA0 = 1; // Inputs
    TRISAbits.TRISA1 = 1;
    ANSAbits.ANSA0 = 1; //  RA0 AN0 (2 DIP20) VRef+
    ANSAbits.ANSA1 = 1; //  RA1 AN1 (3 DIP20) VRef-*/
    // ____________________________________A/D Converter Setup
    AD1CON1bits.ADON = 0; // Converter off
    //AD1CSSL = 0; // No scan //ADC1MD bit in the PMD1
    // ____________________________________Clock and Conversion Mode
    AD1CON1 = 0x2200; // No operation in Idle mode (ADSIDL=1)
    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
    AD1CON1bits.SSRC = 0x2; // Timer 3 trig ADC
    AD1CON1bits.ASAM = 1; // Auto-Convert ON ( end sampling and start conversion )
    AD1CON1bits.FORM = 0b01; // Samples Format 
    // ____________________________________Buffering Mode
    AD1CON2 = 0; // Inputs are not scanned
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
    AD1CON2bits.SMPI = 0; // Interrupts every samples
    // ____________________________________Conversion Timing
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0; // Clock from system clock (Tcy= 1/Fcy)
    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit
    AD1CON3bits.SAMC = 14; // Auto-Sample Time (TAD=n-bits+2)
    AD1CON3bits.ADCS = 0b010; // ADC Clock (0b10 -> 4Tcy=1TAD 250nS*4 = 1uS)
    // 00111111 = 64톂CY = TAD
    // 00000001 = 2톂CY = TAD
    // ____________________________________Input channels & References
    AD1CON5bits.BGREQ = 1; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)

    AD1CON2bits.PVCFG = 0b10; // ADC Positive Reference
    // 11 = Internal VRH2
    // 10 = Internal VRH1 ( BGP*2 = 2.4volt )
    // 01 = External VREF+ ( Pin AN0 )
    // 00 = AVDD

    AD1CON2bits.NVCFG = 0; // ADC Negative Reference 
    // 1 = External VREF- ( Pin AN1 )
    // 0 = AVSS

    AD1CHSbits.CH0NA = 0b10; // S/H- Input A
    // 010 = AN1
    // 001 = AN0
    // 000 = AVSS   

    AD1CHSbits.CH0SA = 0b0; // S/H+ Input A 
    // 11110 = AVDD
    // 11101 = AVSS
    // 11100 = Upper guardband rail (0.785 * VDD)
    // 11011 = Lower guardband rail (0.215 * VDD)
    // 11010 = Internal Band Gap Reference (VBG)
    // 00001 = AN1
    // 00000 = AN0

#else  // New sensor board PIC 302

    // ____________________________________Input Analog pins
    AV_INP_SetAnalog(); // RA0 AN0 (2 DIP20) VRef+
    AV_INP_SetAnalogInput();
    AV_INN_SetAnalog(); //  RA1 AN1 (3 DIP20) VRef-
    AV_INN_SetAnalogInput();
    // ____________________________________A/D Converter Setup
    AD1CON1bits.ADON = 0; // Converter off
    AD1CON2 = 0; // Inputs are not scanned
    AD1CSSL = 0; // No Scan //ADC1MD bit in the PMD1
    AD1CHS = 0; // Channel   
    // ____________________________________Clock and Conversion Mode
    AD1CON1 = 0; // No operation in Idle mode (ADSIDL=1)
    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
    AD1CON1bits.SSRC = 2; // Timer 3 trig ADC
    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
    AD1CON1bits.FORM = 0b01; // Format (Decimal result, signed, right-justified)
    // 11 = Fractional result, signed, left-justified
    // 10 = Absolute fractional result, unsigned, left-justified
    // 01 = Decimal result, signed, right-justified
    // 00 = Absolute decimal result, unsigned, right-justified
    // ____________________________________Buffering Mode
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
    AD1CON2bits.SMPI = 0; // Interrupts every samples
    //0b1 = Interrupts at the completion of 2th sample
    // ____________________________________Conversion Timing
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit

    AD1CON3bits.SAMC = 14; // 16 Auto-Sample Time TAD
    AD1CON3bits.ADCS = 0b010; // ADC Clock ( 1TAD = ? TCY )
    // 00111111 = 64톂CY = TAD
    // 00000001 = 2톂CY = TAD

    // ____________________________________Input channels & References
    AD1CON5bits.BGREQ = 1; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)

    AD1CON2bits.PVCFG = 0b10; // ADC Positive Reference
    // 11 = Internal VRH2
    // 10 = Internal VRH1 ( BGP*2 = 2.4volt )
    // 01 = External VREF+ ( Pin AN0 )
    // 00 = AVDD

    AD1CON2bits.NVCFG = 0; // ADC Negative Reference 
    // 1 = External VREF- ( Pin AN1 )
    // 0 = AVSS

    AD1CHSbits.CH0NA = 0b1; // S/H- Input A
    // 010 = AN1
    // 001 = AN0
    // 000 = AVSS   

    AD1CHSbits.CH0SA = 0b1; // S/H+ Input A 
    // 11110 = AVDD
    // 11101 = AVSS
    // 11100 = Upper guardband rail (0.785 * VDD)
    // 11011 = Lower guardband rail (0.215 * VDD)
    // 11010 = Internal Band Gap Reference (VBG)
    // 00001 = AN1
    // 00000 = AN0

#endif  // Old sensor board

    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IPC3bits.AD1IP = 2; // High Interrupt Priority
    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1
    _adcReady = false;
    // =============== END:INIT   

#elif defined( __PIC24FJ256GA702__ ) // New sensor board & PIC 702

    // =============== BEGIN:INIT    
    ANCFG |= 0x100; // A/D Band Gap Enable (1ms to tune-up)  
    IEC0bits.AD1IE = 0; // Enable A/D conversion interrupt
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IPC3bits.AD1IP = 1; // High Interrupt Priority

    AV_SYN_SetDigital();
    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)

    // TMR3 as pulse counter on T3CK pin (SYNCO)
    IEC0bits.T3IE = 0; // TMR3 Int call-back disabled (Trig ADC)
    T3CONbits.TON = 1;
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CONbits.TECS = 1; // Timery Extended Clock Source (when TCS = 1)
    //11 = Generic timer (TxCK) external input
    //10 = LPRC Oscillator
    //01 = T3CK external clock input
    //00 = SOSC
    T3CONbits.TCS = 1; // Clock Source 
    //1 = External clock from pin, TyCK (on the rising edge)
    //0 = Internal clock (FOSC/2)
    T3CONbits.TCKPS = 0; // Timer Input Clock (38.4KHz:8 = 4.8Khz)
    //11 = 1:256
    //10 = 1:64
    //01 = 1:8
    //00 = 1:1
    TMR3 = 0x00;
    /*        
switch (av_period) {
    case 3: // 2^3 -1 = 63
        PR3 = 63; // Synco 38.4 Khz : 8 =  4.8Khz, T=0,000208s
        break;
    case 4:
        PR3 = 127; // Synco 38.4 Khz :16 = 2.4Khz, T=0,000416s 
        break;
    case 5:
        PR3 = 255; // Synco 38.4 Khz :32 = 1.2Khz, T=0.000833s 
        break;
    default: // 2^6
        PR3 = 511; // Synco 38.4 Khz :64 = 600Hz, T=0.0017s (1,7 ms)
        break;
}
     */

    av_period--; // tuning !!!!! 
    PR3 = (1U << (av_period)) - 1; // FRequency divider

    IFS0bits.T3IF = 0; // Reset int flag

    // ____________________________________Input Analog pins
    AV_INP_SetAnalog(); // RA0 AN0 (2 DIP20) VRef+
    AV_INP_SetAnalogInput();
    AV_INN_SetAnalog(); //  RA1 AN1 (3 DIP20) VRef-
    AV_INN_SetAnalogInput();

    // ____________________________________A/D Converter Setup
    // AD1CON1bits.ADON = 0; // Converter off
    AD1CON2 = 0; // Inputs are not scanned
    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1
    AD1CHS = 0; // No channels
    // ____________________________________Clock and Conversion Mode
    AD1CON1 = 0; // No operation in Idle mode (ADSIDL=1)
    AD1CON1bits.DMABM = 0; // bit 12 : Extended DMA Buffer Mode Select bit(1)
    // 1 = Extended Buffer mode: Buffer address is defined by the DMADSTn register
    // 0 = PIA mode: Buffer addresses are defined by the DMA Controller and AD1CON4<2:0>
    AD1CON1bits.DMAEN = 0; // bit 11 : Extended DMA/Buffer Enable bit
    // 1 = Extended DMA and buffer features are enabled
    // 0 = Extended features are disabled
    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
    AD1CON1bits.FORM = 0b00; // Format (Decimal result, signed, right-justified)
    // 11 = Fractional result, signed, left-justified
    // 10 = Absolute fractional result, unsigned, left-justified
    // 01 = Decimal result, signed, right-justified
    // 00 = Absolute decimal result, unsigned, right-justified
    AD1CON1bits.SSRC = 2; // Timer 3
    // 0000 = SAMP is cleared by software
    // 0001 = INT0
    // 0010 = Timer3
    // 0100 = CTMU trigger
    // 0101 = Timer1 (will not trigger during Sleep mode)
    // 0110 = Timer1 (may trigger during Sleep mode)
    // 0111 = Auto-Convert mode
    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)

    // ____________________________________Buffering & References
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
    //AD1CON2bits.SMPI = 0b1111; // Interrupt Sample/DMA Increment Rate Select bits
    AD1CON2bits.SMPI = 0; // Interrupt Sample/DMA Increment Rate Select bits
    //11111 = Interrupts at the completion of the conversion for each 32nd sample
    //11110 = Interrupts at the completion of the conversion for each 31st sample
    //???
    //00001 = Interrupts at the completion of the conversion for every other sample
    //00000 = Interrupts at the completion of the conversion for each sample


    // ____________________________________Conversion Timing   
    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit
    AD1CON3bits.SAMC = 14; //14; // 16 Auto-Sample Time TAD
    AD1CON3bits.ADCS = 3; // 0x7; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
    // 00111111 = 64톂CY = TAD
    // 00000001 = 2톂CY = TAD
    AD1CON3bits.PUMPEN = 0; // If AVDD is < 2.7V enable the Charge Pump 

    // ____________________________________Input channels 

#if (0)  // No single ended    
    AD1CON5bits.BGREQ = 1; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
    //1 = Band gap is enabled when the A/D is enabled and active
    //0 = Band gap is not enabled by the A/D
    AD1CON5bits.CTMREQ = 0; // CTMU Request bit
    //1 = CTMU is enabled when the A/D is enabled and active
    //0 = CTMU is not enabled by the A/D

    AD1CON2bits.PVCFG = 1; // ADC Positive Reference
    // 1 = External VREF+ ( Pin AN0 )
    // 0 = AVDD
    AD1CON2bits.NVCFG0 = 0; // ADC Negative Reference 
    // 1 = External VREF- ( Pin AN1 )
    // 0 = AVSS
    AD1CHSbits.CH0NA = 0; // S/H- Input A
    // 000 = AVSS (NVCFG0) !!!!!!!!!!!!!  

    _ANSB3 = 1; // AN5
    _TRISB3 = 1; // Analog Input  

    AD1CHSbits.CH0SA = 0b00101; // 1; // S/H+ Input A 
    //11110 = AVDD(1)
    //11101 = AVSS(1)
    //11100 = Band Gap Reference (VBG)(1)
    //10000-11011 = Reserved
    //01111 = No external channels connected (used for CTMU)
    //01110 = No external channels connected (used for CTMU temperature sensor)
    //01101 = AN13
    //01100 = AN12
    //01011 = AN11
    //01010 = AN10
    //01001 = AN9
    //01000 = AN8
    //00111 = AN7
    //00110 = AN6
    //00101 = AN5
    //00100 = AN4
    //00011 = AN3
    //00010 = AN2
    //00001 = AN1
    //00000 = AN0
#else
    AD1CON5bits.BGREQ = 0; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
    AD1CON5bits.CTMREQ = 0; // CTMU Request bit
    AD1CON2bits.PVCFG = 0; // ADC Positive Reference
    AD1CON2bits.NVCFG0 = 0; // ADC Negative Reference 
    AD1CHSbits.CH0NA = 0; // S/H- Input A
    AD1CHSbits.CH0SA = 0; // 1; // S/H+ Input A 
#endif

    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1
    _adcReady = 0;
    // =============== END:INIT   

#endif // __PIC24FJ256GA702__
}

void acquireAV_START(uint16_t nsec) {
    // =============== BEGIN:START  
    _cycletime = true;
    Timeout_Set(nsec, 0);
    Timeout_SetCallBack(&cycletimer);
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    T3CONbits.TON = 1;
#endif
    AD1CON1bits.ADON = 1; // Start ADC
    ADA2200_Synco(0b011); // Enable SYNC 
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
    // =============== END:START
}

void acquireAV_STOP() {
    // =============== BEGIN:STOP   
    AD1CON1bits.ADON = 0; // Converter Off
    T3CONbits.TON = 0;
    ADA2200_Disable();
    Timeout_Unset();
    // =============== END:STOP  
}


uint16_t acquireAV(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period, uint16_t av_filter) {

#ifdef __VAMP1K_TEST
    printf("AV\n");
#elif !defined(__SENSOR_BOARD)
    return (0); // Hardware not supported
#endif       

    sample_t *pSSBUF = dbuf;
    uint16_t ppPoints;
    uint16_t pIndex;
    sample_t Tc, Tcp;
    //


    if ((av_period < 2) || (av_period > 6)) { // 2,3,4,5,6
        av_period = 6; // Default: SYNCO_FREQUENCY / 2^6 = 600Hz
    }
    acquireAV_INIT(av_period); // Compute PR3

    ppPoints = 1; // Use first to store <period> and <scale>
    *pSSBUF = SYNCO_FREQUENCY>>av_period;
    pSSBUF++;
#if defined( __PIC24FJ256GA702__ )
    *pSSBUF = 4200; // Max ADC value
#else
    *pSSBUF = SCALE_TOUNSIGNED << 1; // Max ADC value
#endif    
    pSSBUF++;


#if defined(__VAMP1K_TEST_adc_printf)

    acquireAV_START(nsec);
    maxpoints--; // Reserve one for last sample
    int i, diff;
    while (_cycletime) { // Loop until cycle-time 

        if (_adcReady) { // New data available
            _adcReady--;
            // =============== BEGIN:READ

            diff = (SCALE_TOUNSIGNED - ADC1BUF0); //  & 0xFFE;
            printf("%d (%d) \n", diff, ADC1BUF0);
            
#if (0)
            if (AD1CON2bits.SMPI) {
                //diff = (ADC1BUF0 + ADC1BUF1)>>1; //  & 0xFFE;
                diff = 0;
                for (i = 0; i < AD1CON2bits.SMPI; i++) {
                    diff += *(&ADC1BUF0 + i);
                }
                diff /= AD1CON2bits.SMPI;
                printf("%u ", diff);
                /*
            printf("%d ", ADC1BUF0);
            printf("%d (%u) ", diff,AD1CON2bits.SMPI+1);
            for (i = 16; i > 0; i--) {
                printf("%u", ((diff >> i) & 1U));
            }
            printf(" scaled: %u \n", (sample_t) (SCALE_TOUNSIGNED - diff));
                 */

            } else {
                printf("%d scaled %u ", ADC1BUF0, (sample_t) (SCALE_TOUNSIGNED - ADC1BUF0));
            }
#endif

            // =============== END:READ
        }// _adcReady

    }
    acquireAV_STOP();
    printf("Restart acquire...\n");
    

#elif defined( __VAMP1K_TEST_adc_DATAVIS)

    uint16_t adc12bit;
    acquireAV_START(nsec);
    while (1) { //   Loop until cycle-time or full filled buffer

        if (_adcReady) { // New data available
            // =============== BEGIN:READ
            _adcReady--;
            adc12bit = (ADC1BUF0 + ADC1BUF1) >> 1;
            UART2_Write(0x5F);
            UART2_Write(adc12bit & 0xFF);
            UART2_Write(adc12bit >> 8);
            UART2_Write(_adcReady);
            UART2_Write(0xA0);

            //for (txCounter = 0; txCounter < 6; txCounter++) {
            //  UART2_Write(*(prtTx + txCounter));
            //}
            Tc++; // T = fSynco/16
            // =============== END:READ
        }// _adcReady
    }
    acquireAV_STOP();

  

#elif defined( __AV00_PP )

    point_t points[3]; // SAMPLING_AV_PBUFFER
    int pm01, pm12, lpm;

    Tc = 0;
    Tcp = 0;
    pIndex = 0;
    lpm = -1;

    maxpoints--; // Reserve one for last sample
    acquireAV_START(nsec);
    while ((ppPoints < maxpoints) && !isTimeout()) { // Loop until cycle-time or full filled buffer

        if (_adcReady) { // New data available

            // =============== BEGIN:READ

            // ---------------- get samples
            _adcReady--;
            points[pIndex].A = ADC1BUF0; //(SCALE_TOUNSIGNED - ADC1BUF0); // Positive point
            points[pIndex].T = Tc;
            if (pIndex > 0) { // !!! Inizializzare a 2 volte SCALE_... ed elimina IF nel ciclo
                if (abs(points[pIndex].A - points[pIndex - 1].A) < g_dev.cnf.calibration.av_filter) {
                    points[pIndex - 1] = points[pIndex];
                } else {
                    pIndex++;
                }
            } else { // save first sample T=0
                *pSSBUF = points[0].T;
                pSSBUF++;
                *pSSBUF = points[0].A;
                pSSBUF++;
                ppPoints++;
                pIndex++;
            }
            Tc++; // T = fSynco/16
            // ---------------- get samples

            if ((pIndex == 3)) { // min 3 points to mach PP

                // PPmatch = false;
                //pm01 = (points[0].A > (points[1].A + g_dev.cnf.calibration.av_filter)); // _snr = 30;
                //pm12 = (points[1].A > (points[2].A + g_dev.cnf.calibration.av_filter));
                // pm01 = (points[0].A > points[1].A); // On filtered points
                // pm12 = (points[1].A > points[2].A);
                // if ((!pm01) != (!pm12)) { // Save PP point


                pm01 = (points[0].A < points[1].A); // _snr = 30;
                pm12 = (points[1].A < points[2].A);
                if (pm01 != pm12) { // Save PP point

                    if (pm01 == lpm) { // Over the last one ?
                        *(pSSBUF - 2) += (points[1].T - Tcp); // Time
                        Tcp = points[1].T;
                        *(pSSBUF - 1) = points[1].A; // Amplitude
                    } else {
                        *pSSBUF = (points[1].T - Tcp); // Time
                        Tcp = points[1].T;
                        pSSBUF++;
                        *pSSBUF = points[1].A; // Amplitude
                        pSSBUF++;
                        ppPoints++;
                    }
                    points[0] = points[2];
                    pIndex = 1;
                    lpm = pm01;

                } else {
                    points[0] = points[1];
                    points[1] = points[2];
                    pIndex = 2;
                }
            }
            // =============== END:READ
        }// _adcReady
    }

    acquireAV_STOP();

    *pSSBUF = points[pIndex - 1].T - Tcp; // Last Sample Tn  
    pSSBUF++;
    *pSSBUF = points[pIndex - 1].A; // Amplitude
    //ppPoints++;

#elif defined( __AV00_RAW )

    Tc = 0;
    Tcp = 0;
#if defined( __VAMP1K_TEST )    
    _TRISB2 = 0;
    _ANSB2 = 0;
    _LATB2 = 0;
#endif

    acquireAV_START(nsec);
    // _adcReady=0;
    while ((ppPoints <= maxpoints) && !isTimeout()) { // Loop until cycle-time or full filled buffer

        if (_adcReady) { // New data available

            // =============== BEGIN:READ
            _adcReady--;

#if defined( __VAMP1K_TEST )    
            _LATB2 ^= 1; // IO_LED2_Toggle() 
#endif
#if defined( __VAMP1K_TEST_measurement_DATAVIS )
            *pSSBUF = Tc;
#else
            *pSSBUF = Tc - Tcp; //-Tcp;
#endif

            Tcp = Tc;
            pSSBUF++;
            *pSSBUF = ADC1BUF0; //(SCALE_TOUNSIGNED - ADC1BUF0); // Positive point
            pSSBUF++;
            ppPoints++;
            pIndex++;
            Tc++; // T = fSynco/16
            // =============== END:READ
        }// _adcReady
    }
    ppPoints--;
    acquireAV_STOP();

#endif 
    return (ppPoints - 1);
}


#ifdef __AV0NVM

uint16_t acquireAV_NVM(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period) {

#ifdef __VAMP1K_TEST
    printf("acquireAVFFT\n");
#endif

#if defined(__SENSOR_BOARD)
    uint16_t ppPoints;
    uint16_t pIndex;
    // sample_t *pSSBUF = dbuf;
    uint32_t *pSB32H;
    uint32_t *pSB32;
    //
    sample_t Tc, Tpc; //  Period = ( 1 / Freq_INT0 ) * nsample
    bool pm01, pm12;
    point_t points[SAMPLING_AV_PBUFFER];
    int diff;
    //
    acquireAV_INIT();
    //
    Tc = 0;
    Tpc = 0; //
    pIndex = 0;
    ppPoints = 0;
    //
    // nvm defs
    uint16_t nvmRowIndex = 0;
    uint16_t nvmRowCounter = 0;
    uint32_t nvmDepotAddress; // Get address of reserved space
    nvmDepotAddress = __builtin_tbladdress(nvmDepot); // Get address of reserved space

    pSB32H = (uint32_t *) dbuf;
    pSB32 = pSB32H;

    *(pSB32 + nvmRowIndex) = to24bit(*(dbuf - 2), *(dbuf - 1)); // Store <temp>,<wind>
    nvmRowIndex++;

    *(pSB32 + nvmRowIndex) = to24bit(40, 2047); // Store <period>,<scale> (period = value/10000)
    nvmRowIndex++;
    maxpoints--;

    maxpoints--; // Reserve one for last sample

    acquireAV_START(nsec);
    //setTimeout(nsec, 0);

    while ((ppPoints < maxpoints) && !isTimeout()) { // Loop until cycle-time or full filled buffer

        if (_adcReady) { // New data available

            // =============== BEGIN:READ
            _adcReady = false;
            points[pIndex].A = (2047 + ADC1BUF0); // Positive point
            points[pIndex].T = Tc;
            pIndex++;
            Tc++; // T = fSynco/16

            if ((pIndex == SAMPLING_AV_PBUFFER)) { // min 3 points to mach PP

                diff = points[0].A - points[1].A;
                if (diff < 0) {
                    diff = points[1].A - points[0].A;
                }

                if (diff <= g_dev.cnf.calibration.av_filter) { // Valutare nell'intorno del filtro               
                    points[1] = points[2];
                    pIndex = 2;
                } else {
                    diff = points[1].A - points[2].A;
                    if (diff < 0) {
                        diff = points[2].A - points[1].A;
                    }

                    if (diff <= g_dev.cnf.calibration.av_filter) { // Valutare nell'intorno del filtro             
                        pIndex = 2;
                    } else {

                        pm01 = (points[0].A <= (points[1].A + g_dev.cnf.calibration.av_filter)); // _snr = 30;
                        pm12 = (points[1].A <= (points[2].A + g_dev.cnf.calibration.av_filter));

                        if (pm01 != pm12) { // Save PP point

                            /* Save ram
                             *pSSBUF = (points[1].T - Tpc); // Time
                            pSSBUF++;
                             *pSSBUF = points[1].A; // Amplitude
                            pSSBUF++;
                             */

                            /* Save nvm
                             *pSB32 = (points[1].T - Tpc);
                             *pSB32 <<= 12;
                             *pSB32 |= (points[1].A && 0xFFF);
                             */

                            // Save nvm
                            *(pSB32 + nvmRowIndex) = to24bit(points[1].T - Tpc, points[1].A);
                            nvmRowIndex++;

                            if (nvmRowIndex == FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS) { // Row filled write latchs
                                FLASH_Unlock(FLASH_UNLOCK_KEY);
                                FLASH_Erase1Row(nvmDepotAddress + (nvmRowCounter * DEPOT_BLOCK_SIZE));
                                FLASH_WriteRow24(nvmDepotAddress + (nvmRowCounter * DEPOT_BLOCK_SIZE), pSB32); // Write 24 bit Data
                                FLASH_Lock();
                                pSB32 = pSB32H;
                                nvmRowCounter++;
                                nvmRowIndex = 0;
                            }

                            Tpc = points[1].T;
                            ppPoints++;
                            points[0] = points[2];
                            pIndex = 1;

                        } else {

                            points[0] = points[1];
                            points[1] = points[2];
                            pIndex = 2;
                        }
                    }
                }
            }
            // =============== END:READ

        } else {
            if (Tc == 1 && ppPoints == 0) { // save first sample T=0
                *pSB32 = to24bit(points[0].T, points[0].A);
                nvmRowIndex++;
                ppPoints++;
            }
        }// _adcReady

    }
    acquireAV_STOP();

    *(pSB32 + nvmRowIndex) = to24bit(points[SAMPLING_AV_PBUFFER - 1].T - Tpc, points[SAMPLING_AV_PBUFFER - 1].A);
    nvmRowIndex++;
    ppPoints++;

    // save last nvm Row full or partial filled...
    FLASH_Unlock(FLASH_UNLOCK_KEY);
    FLASH_Erase1Row(nvmDepotAddress + (nvmRowCounter * DEPOT_BLOCK_SIZE));
    FLASH_WriteRow24(nvmDepotAddress + (nvmRowCounter * DEPOT_BLOCK_SIZE), pSB32); // Write 24 bit Data
    FLASH_Lock();
    //nvmRowCounter++;

    return (ppPoints);

#else
    *dbuf = 0;
    return (0); // Hardware not supported
#endif   
}
#endif // No NVM

/* ----------------------------------------------------------------------------- 
 MEASURE SUB-SPAN OSCILLATION                                       
 ---------------------------------------------------------------------------- */
uint16_t acquireSS() {

    return (0);
}


