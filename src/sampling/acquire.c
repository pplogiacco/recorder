#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>
// #include "sampling.h"

#include "../utils.h"
#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>

#include "../utils.h"
#include "../modules/RTCC.h"

#include "measurement.h"
#include "ADA2200.h"        // HAL ADA2200
#include "../../libfft.X/src/libfft.h"


/* -------------------------------------------------------------------------- *
 * DEMO SIGNAL
 * 4 single samples,  N/2 sequenced samples
 * { [period],[ampmax],[nT,A],... }
 * -------------------------------------------------------------------------- */
static float const ADC_FQ = 1000; // 1KHz Fix !!! T=0.001 (1ms) 
static sample_t const SIG_MIN_AMP = 100; // 12 bits ( 0..4096 )
static sample_t const SIG_SCALE = 4096; // 12 bits ( 0..2047 )

//const float Pi = 3.141592653589793;
static float const _2Pi = 6.283185308;

uint16_t acquireSig(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t sig_freq, uint16_t sig_amp) {
    sample_t nT = 0, centerscale;
    sample_t *pSSBUF = dbuf;
    float dTr = 0, kTr = 0;

    kTr = _2Pi / (ADC_FQ / sig_freq); // signal frequency (ADC fix 1Khz)
    if (sig_amp < SIG_MIN_AMP) {
        sig_amp = SIG_MIN_AMP;
    }
    maxpoints -= 2; // Add <Frequency> & <Scale> 
    *pSSBUF = ADC_FQ;
    *(pSSBUF + 1) = SIG_SCALE;
    pSSBUF += 2;
    centerscale = SIG_SCALE >> 1;

    while ((nT < maxpoints) && ((nT * (1.0 / ADC_FQ)) <= nsec)) {
        *pSSBUF = (nT == 0) ? 0 : 1;
        *(pSSBUF + 1) = centerscale + (sin(dTr) * (float) sig_amp);
        pSSBUF += 2;
        dTr += kTr;
        //        if (dTr > (_2Pi)) {
        //            dTr -= (_2Pi);
        //        }
        nT++;
    }
    return (nT);
}




/*----------------------------------------------------------------------------*
 * L V D T  ( A E O L I A N  V I B R A T I O N )                              *   
 * acquireAV_INIT(adc_period)  --> tick_time                                  *
 *----------------------------------------------------------------------------*/

volatile static uint8_t _adcReady; // ADC Cycle control
volatile static bool _cycletime;

static sample_t *ptrDB; // buffer pointer 
static sample_t Tc, Tcp;
static uint16_t nSamples;

typedef struct {
    sample_t T;
    sample_t A;
} point_t;

inline static void cycletimer(void) { // called by TMR1 ISR on elapsed cycle
    _cycletime = false;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void) { // , weak
    IFS0bits.AD1IF = 0; // Clear immediatelly
    // _LATB2 ^= 1; // IO_LED2_Toggle() 
    _adcReady++;
}


/////////******************************************************************************
//////// "acquireSYNCO" model is based on TMR3 triggered by Ada2200's SYNCO signal:
//////// 
////////  void acquireSYNCO_INIT(uint16_t tmr_pr3, bool ada_sync)
////////  void acquireSYNCO_START(uint16_t nsec)
////////  void acquireSYNCO_STOP()
//////// 
////////  ADC frequency is computed as fraction of SYNCO frequency:
//////// 
////////     3: 2^3 -1,  PR3 = 063, Synco 38.4 Khz :8  = 4.8Khz, T=0,000208s
////////     4: 2^4 -1,  PR3 = 127, Synco 38.4 Khz :16 = 2.4Khz, T=0,000416s 
////////     5: 2^5 -1,  PR3 = 255, Synco 38.4 Khz :32 = 1.2Khz, T=0.000833s 
////////     6: 2^6 -1,  PR3 = 511, Synco 38.4 Khz :64 =  600Hz, T=0.0017s (1,7 ms)
////////
////////     tmr_pr3 = (1U << (ADC_OPTION - 1)) - 1; // ADC_OPTION = 3,4,5,6
////////
////////  
////////
//////// ******************************************************************************/
////////
////////void acquireSYNCO_INIT(uint16_t tmr_pr3, bool ada_sync) {
////////    
////////#if defined( __PIC24FJ256GA702__ ) // New sensor board & PIC 702
////////
////////    // if(  ada_sync ) {  }
////////    // ______________________TMR3 as pulse counter on T3CK pin (SYNCO)
////////    AV_SYN_SetDigital();
////////    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)
////////    IEC0bits.T3IE = 0; // TMR3 Int call-back disabled (Trig ADC)
////////    T3CONbits.TON = 1;
////////    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
////////    T3CONbits.TECS = 1; // Timery Extended Clock Source (when TCS = 1)
////////    //11 = Generic timer (TxCK) external input
////////    //10 = LPRC Oscillator
////////    //01 = T3CK external clock input
////////    //00 = SOSC
////////    T3CONbits.TCS = 1; // Clock Source 
////////    //1 = External clock from pin, TyCK (on the rising edge)
////////    //0 = Internal clock (FOSC/2)
////////    T3CONbits.TCKPS = 0; // Timer Input Clock (38.4KHz:8 = 4.8Khz)
////////    //11 = 1:256
////////    //10 = 1:64
////////    //01 = 1:8
////////    //00 = 1:1
////////    T3CONbits.TCKPS = 0b00; // Input Clock Prescale (00 = 1:1)
////////    TMR3 = 0x00;
////////    PR3 = tmr_pr3;
////////    IFS0bits.T3IF = 0; // Reset int flag
////////
////////    // ____________________________________Input Analog pins
////////    //    AV_INP_SetAnalog(); // RA0 AN0 (2 DIP20) VRef+
////////    //    AV_INP_SetAnalogInput();
////////    //    AV_INN_SetAnalog(); //  RA1 AN1 (3 DIP20) VRef-
////////    //    AV_INN_SetAnalogInput();
////////    AV_IN_SetAnalogInput(); // RA0 AN0 (2 DIP20) VRef+
////////    // ____________________________________A/D Converter Setup
////////    // ANCFG |= 0x100; // A/D Band Gap Enable (1ms to tune-up)  
////////    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
////////    IPC3bits.AD1IP = 3; // High Interrupt Priority
////////    // AD1CON1bits.ADON = 0; // Converter off
////////    AD1CON2 = 0; // Inputs are not scanned
////////    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1
////////    AD1CHS = 0; // No channels
////////    // ____________________________________Clock and Conversion Mode
////////    AD1CON1 = 0; // No operation in Idle mode (ADSIDL=1)
////////    AD1CON1bits.DMABM = 0; // bit 12 : Extended DMA Buffer Mode Select bit(1)
////////    // 1 = Extended Buffer mode: Buffer address is defined by the DMADSTn register
////////    // 0 = PIA mode: Buffer addresses are defined by the DMA Controller and AD1CON4<2:0>
////////    AD1CON1bits.DMAEN = 0; // bit 11 : Extended DMA/Buffer Enable bit
////////    // 1 = Extended DMA and buffer features are enabled
////////    // 0 = Extended features are disabled
////////    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
////////    AD1CON1bits.FORM = 0b00; // Format (Decimal result, signed, right-justified)
////////    // 11 = Fractional result, signed, left-justified
////////    // 10 = Absolute fractional result, unsigned, left-justified
////////    // 01 = Decimal result, signed, right-justified
////////    // 00 = Absolute decimal result, unsigned, right-justified
////////    AD1CON1bits.SSRC = 2; // Timer 3
////////    // 0000 = SAMP is cleared by software
////////    // 0001 = INT0
////////    // 0010 = Timer3
////////    // 0100 = CTMU trigger
////////    // 0101 = Timer1 (will not trigger during Sleep mode)
////////    // 0110 = Timer1 (may trigger during Sleep mode)
////////    // 0111 = Auto-Convert mode
////////    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
////////
////////    // ____________________________________Buffering & References
////////    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
////////    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
////////    //AD1CON2bits.SMPI = 0b1111; // Interrupt Sample/DMA Increment Rate Select bits
////////    AD1CON2bits.SMPI = 0; // Interrupt Sample/DMA Increment Rate Select bits
////////    //11111 = Interrupts at the completion of the conversion for each 32nd sample
////////    //11110 = Interrupts at the completion of the conversion for each 31st sample
////////    //???
////////    //00001 = Interrupts at the completion of the conversion for every other sample
////////    //00000 = Interrupts at the completion of the conversion for each sample
////////
////////
////////    // ____________________________________Conversion Timing   
////////    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
////////    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit
////////    AD1CON3bits.SAMC = 14; //14; // 16 Auto-Sample Time TAD
////////    AD1CON3bits.ADCS = 7; // 0x3; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
////////    // 00111111 = 64톂CY = TAD
////////    // 00000001 = 2톂CY = TAD
////////    AD1CON3bits.PUMPEN = 0; // If AVDD is < 2.7V enable the Charge Pump 
////////
////////    // ____________________________________Input channels ( Single Ended)
////////
////////    AD1CON5bits.BGREQ = 0; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
////////    //1 = Band gap is enabled when the A/D is enabled and active
////////    //0 = Band gap is not enabled by the A/D
////////    AD1CON5bits.CTMREQ = 0; // CTMU Request bit
////////    //1 = CTMU is enabled when the A/D is enabled and active
////////    //0 = CTMU is not enabled by the A/D
////////    AD1CON2bits.PVCFG = 0; // ADC Positive Reference
////////    // 1 = External VREF+ ( Pin AN0 )
////////    // 0 = AVDD
////////    AD1CON2bits.NVCFG0 = 0; // ADC Negative Reference 
////////    // 1 = External VREF- ( Pin AN1 )
////////    // 0 = AVSS
////////    AD1CHSbits.CH0NA = 0; // S/H- Input A
////////    // 000 = AVSS (NVCFG0) !!!!!!!!!!!!!  
////////
////////    AD1CHSbits.CH0SA = ADA_IN_ADC_CH0SA; // S/H+ Input A 
////////    //11110 = AVDD(1)
////////    //11101 = AVSS(1)
////////    //11100 = Band Gap Reference (VBG)(1)
////////    //10000-11011 = Reserved
////////    //01111 = No external channels connected (used for CTMU)
////////    //01110 = No external channels connected (used for CTMU temperature sensor)
////////    //01101 = AN13
////////    //01100 = AN12
////////    //01011 = AN11
////////    //01010 = AN10
////////    //01001 = AN9
////////    //01000 = AN8
////////    //00111 = AN7
////////    //00110 = AN6
////////    //00101 = AN5
////////    //00100 = AN4
////////    //00011 = AN3
////////    //00010 = AN2
////////    //00001 = AN1
////////    //00000 = AN0
////////
////////    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1
////////
////////#endif // __PIC24FJ256GA702__
////////}
////////

/*******************************************************************************
 "acquireTMR3" model is based on TMR3 base frequency counter: 
  
  void acquireTMR3_INIT(uint16_t freq)
  
 ADC frequency is computed as follow:
 
    // FCY = FOSC/2 = 16Mhz
    // 1/16Mhz = bt ( base tick time )
    // 1/adc_freq = at ( adc tick )
    // PR3 = at/bt + 1
    //
    //F(Hz)	PR3    SMPI     Sampling(Hz)
    // 5000	3200    2       2500
    // 2000	8000    2       1000
    // 1000	16000   2        500
    // 500	32000
    // 250	64000
 
    // (16000000/adc_freq)+1; 
  
  
 
 ******************************************************************************/
void acquireTMR3_INIT(uint16_t fq_pr3) {

#if defined( __PIC24FJ256GA702__ ) // New sensor board & PIC 702
    // ==== TMR3 as base frequency divider ====
    IEC0bits.T3IE = 0; // TMR3 Int call-back disabled (Trig ADC)
    T3CONbits.TON = 1;
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CONbits.TCS = 0; // Clock Source  (0=FOSC/2)/(1=TECS)
    T3CONbits.TECS = 0; // Timery Extended Clock Source (TxCK, LPRC, T3CK)
    T3CONbits.TCKPS = 00; // Input Clock Prescale (00 = 1:1)
    TMR3 = 0x00;
    PR3 = fq_pr3; // Frequency divider
    IFS0bits.T3IF = 0; // Reset int flag

    // ____________________________________Input Analog pins
    // ____________________________________A/D Converter Setup
    // AD1CON1bits.ADON = 0; // Converter off
    //ANCFG |= 0x100; // A/D Band Gap Enable (1ms to tune-up)  
    //    IEC0bits.AD1IE = 0; // Enable A/D conversion interrupt
    //    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    //    IPC3bits.AD1IP = 3; // High Interrupt Priority

    AD1CON2 = 0; // Inputs are not scanned
    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1
    AD1CHS = 0; // No channels
    // ____________________________________Clock and Conversion Mode
    AD1CON1 = 0; // No operation in Idle mode (ADSIDL=1)
    AD1CON1bits.DMABM = 0; // bit 12 : Extended DMA Buffer Mode ( PIA mode )
    AD1CON1bits.DMAEN = 0; // bit 11 : Extended DMA/Buffer (features are disabled)
    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
    AD1CON1bits.FORM = 0b00; // Format (Decimal result, signed, right-justified)
    AD1CON1bits.SSRC = 2; // Timer 3
    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
    // ____________________________________Buffering & References
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
    AD1CON2bits.SMPI = 0; // Interrupt Sample/DMA Increment Rate 
    // ____________________________________Conversion Timing   
    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit
    AD1CON3bits.SAMC = 14; // Auto-Sample Time TAD (12+2 = 14)
    AD1CON3bits.ADCS = 11; //0x7; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
    // 00111111 = 64톂CY = TAD
    // 00000001 = 2톂CY = TAD
    AD1CON3bits.PUMPEN = 0; // If AVDD is < 2.7V enable the Charge Pump 
    // ____________________________________Input channels 
    AD1CON5bits.BGREQ = 0; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
    AD1CON5bits.CTMREQ = 0; // CTMU Request bit
    AD1CON2bits.PVCFG = 0; // ADC Positive Reference
    AD1CON2bits.NVCFG0 = 0; // ADC Negative Reference 
    AD1CHSbits.CH0NA = 0; // S/H- Input A
    AD1CHSbits.CH0SA = ADA_IN_ADC_CH0SA; // S/H+ Input A 
    _adcReady = 0;
#endif // __PIC24FJ256GA702__
}

void acquire_START(uint16_t nsec) {
    _adcReady = 0;
    Tc = 0;
    Tcp = 0;
    nSamples = 0;

    _cycletime = true;
    if (nsec > 0) {
        Timeout_SetCallBack(&cycletimer);
        Timeout_Set(nsec, 0);
    }
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
    AD1CON1bits.ADON = 1; // Start ADC
    //     __delay_ms(5);

}

void acquire_STOP() {
    AD1CON1bits.ADON = 0; // Converter Off
    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
    T3CONbits.TON = 0; 
    Timeout_Unset();
    
}


/******************************************************************************/
/*                    A C Q U I R I N G    R O U T I N E S                    */

/******************************************************************************/


uint16_t acquireAV(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t adc_pr3, uint16_t pp_filter) {
#ifdef __VAMP1K_TEST
    printf("AV0X - RAW / P-P \n");
#endif  

    ptrDB = dbuf;
    //    acquireAV_INIT(adc_pr3, true); // Use ADA2200 Synco 
    acquireTMR3_INIT(adc_pr3); // ADC sampling frequency
    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1

    //__delay(2); // wait to stabilize

    if (pp_filter == 0) { // ---------------- Raw data _AV00

        acquire_START(nsec);

#if defined( __VAMP1K_TEST ) && defined( __VAMP1K_TEST_AV_printf )
        while (!isTimeout()) { // Loop until cycle-time or full filled buffer
            if (_adcReady) { // New data available
                _adcReady--;
                printf("%d \n", ADC1BUF0);
            }// _adcReady
        }
#else        

        while ((nSamples < db_size) && _cycletime) { // Loop until cycle-time or full filled buffer
            if (_adcReady) { // New data available
                _adcReady--;
                *ptrDB = Tc - Tcp; //-Tcp;
                Tcp = Tc;
                ptrDB++;
                *ptrDB = ADC1BUF0; //(SCALE_TOUNSIGNED - ADC1BUF0); // Positive point
                //        _LATB2 ^= 1; // IO_LED2_Toggle() 
                ptrDB++;
                nSamples += 2;
                Tc++; // T = fSynco/16
            }// _adcReady
        }
#endif
        acquire_STOP();

    } else { // ----------------  Peak-Peak _ AV01

        point_t points[3]; // SAMPLING_AV_PBUFFER
        uint16_t pIndex = 0;
        signed short pm01, pm12, lpm = -1;

        db_size -= 2; // Reserve one for last point
        acquire_START(nsec);

        while ((nSamples < db_size) && _cycletime) { // Loop until cycle-time or full filled buffer

            if (_adcReady) { // New data available
                // ---------------- get samples
                _adcReady--;
                //printf("%d \n", ADC1BUF0);                        
                points[pIndex].A = ADC1BUF0; // ADC Positive data !!!!!
                points[pIndex].T = Tc;
                if (pIndex > 0) { // !!! Inizializzare a 2 volte SCALE_... ed elimina IF nel ciclo
                    if (abs((points[pIndex].A) - (points[pIndex - 1].A)) < pp_filter) { // ONLY POSITIVE !!!
                        points[pIndex - 1] = points[pIndex];
                    } else {
                        pIndex++;
                    }
                } else { // save first sample T=0
                    *ptrDB = points[0].T;
                    ptrDB++;
                    *ptrDB = points[0].A;
                    ptrDB++;
                    nSamples += 2;
                    pIndex++;
                }
                Tc++; // n Synco Pulses
                // ---------------- get samples

                if (pIndex == 3) { // min 3 points to mach PP

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
                            *(ptrDB - 2) += (points[1].T - Tcp); // Time
                            Tcp = points[1].T;
                            *(ptrDB - 1) = points[1].A; // Amplitude
                        } else {
                            *ptrDB = (points[1].T - Tcp); // Time
                            Tcp = points[1].T;
                            ptrDB++;
                            *ptrDB = points[1].A; // Amplitude
                            ptrDB++;
                            nSamples += 2;
                        }
                        points[0] = points[2];
                        pIndex = 1;
                        lpm = pm01;

                    } else {
                        points[0] = points[1];
                        points[1] = points[2];
                        pIndex = 2;
                    }
                } // Maching PP
            }// _adcReady
        }

        acquire_STOP();

        *ptrDB = points[pIndex - 1].T - Tcp; // Last Sample Tn  
        ptrDB++;
        *ptrDB = points[pIndex - 1].A; // Amplitude
        nSamples += 2;
    } // End Peak2Peak

    ADA2200_Disable();
    return (nSamples);
}

/*----------------------------------------------------------------------------*
 * A C Q U I R E   A V   R A W   N O   D E L T A T I M E                             *
 *                                                                            *
 *                                                                            *
 *----------------------------------------------------------------------------*/
uint16_t acquireAV_RAW(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t adc_pr3, uint16_t amp_filter) {
#ifdef __VAMP1K_TEST
    printf("AV_RAW-NDT\n");
#endif  

    ptrDB = dbuf;
    acquireTMR3_INIT(adc_pr3); // Use ADA2200 Synco 

    ADA2200_Enable();
    acquire_START(nsec);

#if defined( __VAMP1K_TEST ) && defined( __VAMP1K_TEST_AV_printf )
    while (!isTimeout()) { // Loop until cycle-time or full filled buffer
        if (_adcReady) { // New data available
            _adcReady--;
            printf("%d \n", ADC1BUF0);
        }// _adcReady
    }
#else        
    while ((nSamples < db_size) && _cycletime) { // Loop until cycle-time or full filled buffer
        if (_adcReady) { // New data available
            _adcReady--;
            // Apply amp_filter
            *ptrDB = ADC1BUF0; //(SCALE_TOUNSIGNED - ADC1BUF0); // Positive point
            //     _LATB2 ^= 1; // IO_LED2_Toggle() 
            ptrDB++;
            nSamples++;
        }// _adcReady
    }
#endif
    acquire_STOP();
    ADA2200_Disable();
    return (nSamples);
}


uint16_t acquireAV_FFT(sample_t* dbuf, uint16_t nsec, uint16_t log2_npoints, uint16_t adc_fq) {
    uint16_t npoints = (1U << log2_npoints);
    uint16_t iP = 0;
#ifdef __VAMP1K_TEST
    printf("AV_FFT\n");
#endif 
    
    fft_init(log2_npoints); // Initialize tables 
    acquireTMR3_INIT(adc_fq); // ADC sampling frequency
    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1

    acquire_START(0); // No timeout, full filled buffer
    while ((iP < npoints)) {
        if (_adcReady) {
            _adcReady--;
            //            *(dbuf + iP) = fft_windowing((ADC1BUF0 + ADC1BUF1) >> 1, iP);
            *(dbuf + iP) = fft_windowing(ADC1BUF0, iP);
            //*(dbuf + iP + npoints) = 0;
            iP++;
        } // _adcReady
    };

    acquire_STOP();
    ADA2200_Disable();

    fft_spectrum(dbuf);
    return (iP);
}


/*----------------------------------------------------------------------------*
 * A C Q U I R E    V I B R A T I O N   C O U N T E R
 
  Use splitted buffer: 
    first part as {[<nOCC>,<Freq>,<Amp>],....}  
    second used as sampled buffer 
 *----------------------------------------------------------------------------*/
typedef struct {
    sample_t occ; // Occurencies counter
    sample_t nC; // Number of FFT Coefficient
    sample_t Pw; // Coefficient Spectral Power
} vibration_t;

uint16_t acquireAV_EVC(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t log2_npoints, uint16_t adc_fq, uint16_t fft_min_pw, uint16_t fft_avg_co) {
    uint16_t nP = (1U << log2_npoints);
    uint16_t n2P = (1U << log2_npoints) - 1; // Valid FFT Spectrum Coefficient = (point/2)-1
    uint16_t iP;
    
    sample_t* ptrS = dbuf + (db_size - nP); // Ptr to samples buffer part
    
    uint16_t iV, nV = 0, maxV = (db_size - nP) / 3; // Max vibration type
    vibration_t* ptrV = (vibration_t*) dbuf;
    
    #ifdef __VAMP1K_TEST
    printf("AV_EVC\n");
    #endif 
    
    for (iV = 0; iV < maxV; iV++) { // Also use memset
        ptrV[iV].nC = 0xFF;
    }

    fft_init(log2_npoints); // Initialize tables ( STORE IN FLASH )
    
    acquireTMR3_INIT(adc_fq); // ADC sampling frequency
    
    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1

    while (nsec > 0) {

        iP = 0; // Acquired Points Counter
        acquire_START(0); // No timeout, full filled buffer
        
        while ( iP < nP ) {
            if (_adcReady) {
                _adcReady--;
                *(ptrS + iP) = fft_windowing(ADC1BUF0, iP); // fft_windowing((ADC1BUF0 + ADC1BUF1) >> 1, iP);
                iP++;
            } // _adcReady
        };
        acquire_STOP();


        fft_spectrum(ptrS);

        iP = 0; // Save vibration occurrencies
        while (iP < n2P) { // For all Coefficients
            if (*(ptrS + iP) > fft_min_pw) { // Valid coefficient
                iV = 0;
                while ((iV < nV) && ptrV[iV].nC != iP) {
                    iV++;
                }
                if (iV == nV) { // Add
                    nV++;
                    ptrV[iV].nC = iP;
                    ptrV[iV].Pw = *(ptrS + iP);
                    ptrV[iV].occ = 1;
                } else { // Update
                    ptrV[iV].occ++;
                }
            }
            iP++;
        } // Save

        nsec--;
    }// 

    ADA2200_Disable();

    return (nV * 3);
}


/* -------------------------------------------------------------------------- *
 * E N V I R O M E N T   T E M P E R A T U R E
 * Read analog pin value (ADC)
 * Sum readed values to compute everage on 6 measures.
 * -------------------------------------------------------------------------- */
uint16_t acquireET(sample_t* dbuf) { // RealTime ?
#ifdef __VAMP1K_TEST
    printf("ET\n");
#endif    
#ifndef __SENSOR_BOARD
    *dbuf = -1;
    return (0); // Hardware not supported
#endif

    IEC0bits.AD1IE = 0; // Disable ADC Int
    IFS0bits.AD1IF = 0; // Clear flag
    // ____________________________________ADC Input Pin
    ET_IN_SetAnalogInput(); // Input  AN5/C1INA/C2INC/SCL2/CN7/RB3 (7)

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // ____________________________________ADC setup
    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
    AD1CON1 = 0x2200; // Configure clock and trigger mode.
    AD1CON1bits.FORM = 0; // Absolute decimal result, unsigned, right-justified
    AD1CON1bits.SSRC = 0; // Software conversion trigger (SAMP=0/1)
    AD1CON1bits.ASAM = 0; // AD begins when manually SAMP=0 set)
    AD1CON2 = 0; // ADC voltage reference, buffer fill modes
    AD1CON3 = 0; // sample time = 1Tad, conversion clock as Tcy
    AD1CHSbits.CH0NA = 0; // MUXA Channel 0 Negative Input (000 = AVSS)
    AD1CHSbits.CH0SA = 0B00101; // RB3/AN5 Positive Input
    AD1CSSL = 0; // No inputs are scanned.

#elif defined( __PIC24FJ256GA702__)
    // ____________________________________A/D Converter Setup
    AD1CON1 = 0; // No operation in Idle mode, Converter off
    AD1CON1bits.MODE12 = 0; // Resolution 10 bit (1=12)
    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
    AD1CON2 = 0; // Inputs are not scanned
    AD1CON3 = 0;
    AD1CON3bits.SAMC = 14; // 16 Auto-Sample Time TAD
    AD1CON3bits.ADCS = 7; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
    AD1CON5 = 0; // No CTMU, No Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
    AD1CHS = 0; // No channels
    AD1CHSbits.CH0SA = 0b00101; // S/H+ Input A (AN5)  
    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1
#endif

    // ____________________________________Acquire
    AD1CON1bits.ADON = 1; // Turn on A/D
    //while (0) {
    AD1CON1bits.SAMP = 1; // Start sampling the input
    __delay(1); // Ensure the correct sampling time has elapsed
    AD1CON1bits.SAMP = 0; // End sampling and start conversion
    while (!AD1CON1bits.DONE) {
        Nop(); //printf("adc=%d \n", ADC1BUF0);
    }
    AD1CON1bits.ADON = 0; // ADC Off
    *dbuf = (1024 - ADC1BUF0);
    return (1); // check !!!!!
}



/*----------------------------------------------------------------------------*
 * W I N D  S P E E D                                                         *
 * Count the pulses (rotations) on pin for fixed time period (1sec):          *
 * wind_speed = pulses * wsFactor = m/s                                       *
 *                                                                            *
 *----------------------------------------------------------------------------*/
#define __nWSS 3    // Everage 2 samples
static const uint16_t _nWSS = __nWSS;
volatile int _icycle;
volatile int _wsready;
volatile int _wsptime[__nWSS];

void everySecond(void) {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    if (_wsready) { // Stop
        _wsptime[_icycle] = TMR4;
        _icycle++;
    } else { // Start
        _wsready = 1;
        TMR4 = 0;
    }
#elif defined( __PIC24FJ256GA702__)
    if (_wsready) { // Stop
        _wsptime[_icycle] = TMR2;
        _icycle++;
        _wsready = 0; // re-Start
    } else { // Start
        _wsready = 1;
        TMR2 = 0;
    }
#endif 
}

uint16_t acquireWS(sample_t* dbuf) {
#ifdef __VAMP1K_TEST
    printf("WS\n");
#endif
#if !defined(__SENSOR_BOARD)
    *dbuf = 0;
    return (0); // Hardware not supported
#endif  
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // TMR4 as Pulses Counter
    WS_IN_SetDigital(); // Input AN4/T5CK/T4CK/U1RX/CTED13/CN6/RB2 (6) 
    WS_IN_SetDigitalInput();
    T4CON = 0x00; // Reset TMR4, 16 Bit
    T4CONbits.TCS = 1; //  T4CK pin Clock source
    T4CONbits.TCKPS = 0; // No Prescaler
    TMR4 = 0x00; // TMR4 Counter Register
    PR4 = 0xFFFF; // TMR4 Single Event
    IFS1bits.T4IF = 0; // Reset Int vector
    IEC1bits.T4IE = 0; // Disable Int

    // Initialize
    //memset(...))
    int i;
    for (i = 0; i < _nWSS; i++) {
        _wsptime[i] = 0;
    }
    _icycle = 0;
    _wsready = 0;

    Timeout_SetCallBack(everySecond);
    T4CONbits.TON = 1; // Starts Timery
    Timeout_Set(1, 0); // TMR1 1Second Start !
    while ((_icycle < _nWSS)) {
        Nop();
    }
    Timeout_Unset();
    T4CONbits.TON = 0;

    for (i = 1; i < _icycle; i++) { // Compute average
        _wsptime[1] += _wsptime[i];
    }
    *dbuf = (_wsptime[1] / (_icycle - 1));
    return (_icycle > 0);

#elif defined( __PIC24FJ256GA702__)

    // TMR2 as Pulses Counter    
    WS_IN_SetDigitalInputLow(); // Input RB2 (6) 

    IEC0bits.T2IE = 0; // Disable Int
    IFS0bits.T2IF = 0; // Reset Int vector
    T2CON = 0x00; // Reset TMR2, 16 Bit, No Prescaler
    T2CONbits.TCS = 1; //  Extended Clock Source (TECS))
    T2CONbits.TECS = 1; // T2CK pin 
    TMR2 = 0x00; // TMR2 Counter Register
    PR2 = 0xFFFF; // TMR4 Single event

    _icycle = 0; // Cycle index 
    _wsready = 0; // Start/Stop flag

    Timeout_SetCallBack(everySecond);
    T2CONbits.TON = 1;
    Timeout_Set(1, 0); // TMR1 1Second Start !

    while ((_icycle < _nWSS)) {
    }
    Timeout_Unset();
    T2CONbits.TON = 0;
    for (_icycle = 2; _icycle < _nWSS; _icycle++) { // Compute average
        _wsptime[1] += _wsptime[_icycle];
    }
    *dbuf = (_wsptime[1] / (_nWSS - 1));
    return (true);

#else
    return (false); // Hardware not supported
#endif
}



///*----------------------------------------------------------------------------*
// * A C Q U I R E    A V D M A                                           *
// * Count the pulses (rotations) on pin for fixed time period (1sec):          *
// * wind_speed = pulses * wsFactor = m/s                                       *
// *----------------------------------------------------------------------------*/
//
//void acquireAV_DMA_INIT(uint16_t tmr_pr3, bool ada_sync) {
//
//#if defined( __PIC24FJ256GA702__ ) // New sensor board & PIC 702
//
//    // if(  ada_sync ) {  }
//
//    // ______________________TMR3 as pulse counter on T3CK pin (SYNCO)
//    AV_SYN_SetDigital();
//    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)
//    IEC0bits.T3IE = 0; // TMR3 Int call-back disabled (Trig ADC)
//    T3CONbits.TON = 1;
//    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
//    T3CONbits.TECS = 1; // Timery Extended Clock Source (when TCS = 1)
//    //11 = Generic timer (TxCK) external input
//    //10 = LPRC Oscillator
//    //01 = T3CK external clock input
//    //00 = SOSC
//    T3CONbits.TCS = 1; // Clock Source 
//    //1 = External clock from pin, TyCK (on the rising edge)
//    //0 = Internal clock (FOSC/2)
//    T3CONbits.TCKPS = 0; // Timer Input Clock (38.4KHz:8 = 4.8Khz)
//    //11 = 1:256
//    //10 = 1:64
//    //01 = 1:8
//    //00 = 1:1
//    T3CONbits.TCKPS = 0b00; // Input Clock Prescale (00 = 1:1)
//    TMR3 = 0x00;
//    PR3 = tmr_pr3;
//    IFS0bits.T3IF = 0; // Reset int flag
//
//    // ____________________________________Input Analog pins
//    //    AV_INP_SetAnalog(); // RA0 AN0 (2 DIP20) VRef+
//    //    AV_INP_SetAnalogInput();
//    //    AV_INN_SetAnalog(); //  RA1 AN1 (3 DIP20) VRef-
//    //    AV_INN_SetAnalogInput();
//    AV_IN_SetAnalogInput(); // RA0 AN0 (2 DIP20) VRef+
//    // ____________________________________A/D Converter Setup
//    // ANCFG |= 0x100; // A/D Band Gap Enable (1ms to tune-up)  
//    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
//    IPC3bits.AD1IP = 3; // High Interrupt Priority
//    // AD1CON1bits.ADON = 0; // Converter off
//    AD1CON2 = 0; // Inputs are not scanned
//    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1
//    AD1CHS = 0; // No channels
//    // ____________________________________Clock and Conversion Mode
//    AD1CON1 = 0; // No operation in Idle mode (ADSIDL=1)
//    AD1CON1bits.DMABM = 0; // bit 12 : Extended DMA Buffer Mode Select bit(1)
//    // 1 = Extended Buffer mode: Buffer address is defined by the DMADSTn register
//    // 0 = PIA mode: Buffer addresses are defined by the DMA Controller and AD1CON4<2:0>
//    AD1CON1bits.DMAEN = 0; // bit 11 : Extended DMA/Buffer Enable bit
//    // 1 = Extended DMA and buffer features are enabled
//    // 0 = Extended features are disabled
//    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
//    AD1CON1bits.FORM = 0b00; // Format (Decimal result, signed, right-justified)
//    // 11 = Fractional result, signed, left-justified
//    // 10 = Absolute fractional result, unsigned, left-justified
//    // 01 = Decimal result, signed, right-justified
//    // 00 = Absolute decimal result, unsigned, right-justified
//    AD1CON1bits.SSRC = 2; // Timer 3
//    // 0000 = SAMP is cleared by software
//    // 0001 = INT0
//    // 0010 = Timer3
//    // 0100 = CTMU trigger
//    // 0101 = Timer1 (will not trigger during Sleep mode)
//    // 0110 = Timer1 (may trigger during Sleep mode)
//    // 0111 = Auto-Convert mode
//    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
//
//    // ____________________________________Buffering & References
//    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
//    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
//    //AD1CON2bits.SMPI = 0b1111; // Interrupt Sample/DMA Increment Rate Select bits
//    AD1CON2bits.SMPI = 0; // Interrupt Sample/DMA Increment Rate Select bits
//    //11111 = Interrupts at the completion of the conversion for each 32nd sample
//    //11110 = Interrupts at the completion of the conversion for each 31st sample
//    //???
//    //00001 = Interrupts at the completion of the conversion for every other sample
//    //00000 = Interrupts at the completion of the conversion for each sample
//
//
//    // ____________________________________Conversion Timing   
//    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
//    AD1CON3bits.EXTSAM = 0; // Extended Sampling Time bit
//    AD1CON3bits.SAMC = 14; //14; // 16 Auto-Sample Time TAD
//    AD1CON3bits.ADCS = 7; // 0x3; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
//    // 00111111 = 64톂CY = TAD
//    // 00000001 = 2톂CY = TAD
//    AD1CON3bits.PUMPEN = 0; // If AVDD is < 2.7V enable the Charge Pump 
//
//    // ____________________________________Input channels ( Single Ended)
//
//    AD1CON5bits.BGREQ = 0; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
//    //1 = Band gap is enabled when the A/D is enabled and active
//    //0 = Band gap is not enabled by the A/D
//    AD1CON5bits.CTMREQ = 0; // CTMU Request bit
//    //1 = CTMU is enabled when the A/D is enabled and active
//    //0 = CTMU is not enabled by the A/D
//    AD1CON2bits.PVCFG = 0; // ADC Positive Reference
//    // 1 = External VREF+ ( Pin AN0 )
//    // 0 = AVDD
//    AD1CON2bits.NVCFG0 = 0; // ADC Negative Reference 
//    // 1 = External VREF- ( Pin AN1 )
//    // 0 = AVSS
//    AD1CHSbits.CH0NA = 0; // S/H- Input A
//    // 000 = AVSS (NVCFG0) !!!!!!!!!!!!!  
//
//    //    _ANSB3 = 1; // AN5  TEST !!!!!!!!!!
//    //    _TRISB3 = 1; // Analog Input  
//
//    AD1CHSbits.CH0SA = 0; // 1; // S/H+ Input A 
//    //11110 = AVDD(1)
//    //11101 = AVSS(1)
//    //11100 = Band Gap Reference (VBG)(1)
//    //10000-11011 = Reserved
//    //01111 = No external channels connected (used for CTMU)
//    //01110 = No external channels connected (used for CTMU temperature sensor)
//    //01101 = AN13
//    //01100 = AN12
//    //01011 = AN11
//    //01010 = AN10
//    //01001 = AN9
//    //01000 = AN8
//    //00111 = AN7
//    //00110 = AN6
//    //00101 = AN5
//    //00100 = AN4
//    //00011 = AN3
//    //00010 = AN2
//    //00001 = AN1
//    //00000 = AN0
//
//    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1
//
//#endif // __PIC24FJ256GA702__
//}

//void acquireAV_DMA_START(uint16_t nsec) {
//    _cycletime = true;
//    _adcReady = 0;
//    Tc = 0;
//    Tcp = 0;
//    nSamples = 0;
//    Timeout_SetCallBack(&cycletimer);
//    ADA2200_Synco(0b111); // Enable SYNC
//    __delay(5);
//    Timeout_Set(nsec, 0);
//    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
//    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
//    AD1CON1bits.ADON = 1; // Start ADC
//}
//
//void acquireAV_DMA_STOP() {
//    AD1CON1bits.ADON = 0; // Converter Off
//    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
//    T3CONbits.TON = 0;
//    ADA2200_Disable();
//    Timeout_Unset();
//}
//
//uint16_t acquireAV_DMA(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t adc_pr3, uint16_t pp_filter) {
//#ifdef __VAMP1K_TEST
//    printf("AV0X\n");
//#endif  
//
//    ptrDB = dbuf;
//    acquireSYNCO_INIT(adc_pr3, true); // Use ADA2200 Synco 
//    //__delay(2); // wait to stabilize
//    //        _TRISB2 = 0;
//    //        _ANSB2 = 0;
//    //        _LATB2 = 0;
//
//    { // ----------------  Peak-Peak _ AV01
//
//        point_t points[3]; // SAMPLING_AV_PBUFFER
//        uint16_t pIndex = 0;
//        signed short pm01, pm12, lpm = -1;
//
//        db_size -= 2; // Reserve one for last point
//        acquire_START(nsec);
//
//        while ((nSamples < db_size) && _cycletime) { // Loop until cycle-time or full filled buffer
//
//            if (_adcReady) { // New data available
//
//                // ---------------- get samples
//                _adcReady--;
//                //printf("%d \n", ADC1BUF0);                        
//                points[pIndex].A = ADC1BUF0; // ADC Positive data !!!!!
//                points[pIndex].T = Tc;
//                if (pIndex > 0) { // !!! Inizializzare a 2 volte SCALE_... ed elimina IF nel ciclo
//                    if (abs((points[pIndex].A) - (points[pIndex - 1].A)) < pp_filter) { // ONLY POSITIVE !!!
//                        points[pIndex - 1] = points[pIndex];
//                    } else {
//                        pIndex++;
//                    }
//                } else { // save first sample T=0
//                    *ptrDB = points[0].T;
//                    ptrDB++;
//                    *ptrDB = points[0].A;
//                    ptrDB++;
//                    nSamples += 2;
//                    pIndex++;
//                }
//                Tc++; // n Synco Pulses
//                // ---------------- get samples
//
//                if (pIndex == 3) { // min 3 points to mach PP
//                    pm01 = (points[0].A < points[1].A); // _snr = 30;
//                    pm12 = (points[1].A < points[2].A);
//                    if (pm01 != pm12) { // Save PP point
//
//                        if (pm01 == lpm) { // Over the last one ?
//                            *(ptrDB - 2) += (points[1].T - Tcp); // Time
//                            Tcp = points[1].T;
//                            *(ptrDB - 1) = points[1].A; // Amplitude
//                        } else {
//                            *ptrDB = (points[1].T - Tcp); // Time
//                            Tcp = points[1].T;
//                            ptrDB++;
//                            *ptrDB = points[1].A; // Amplitude
//                            ptrDB++;
//                            nSamples += 2;
//                        }
//                        points[0] = points[2];
//                        pIndex = 1;
//                        lpm = pm01;
//
//                    } else {
//                        points[0] = points[1];
//                        points[1] = points[2];
//                        pIndex = 2;
//                    }
//                } // Maching PP
//            }// _adcReady
//        }
//
//        acquire_STOP();
//
//        *ptrDB = points[pIndex - 1].T - Tcp; // Last Sample Tn  
//        ptrDB++;
//        *ptrDB = points[pIndex - 1].A; // Amplitude
//        nSamples += 2;
//    } // End Peak2Peak
//
//    return (nSamples);
//}
