#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <string.h>
#include "sampling.h"

#include "../utils.h"
#include "../modules/RTCC.h"

#include "../memory/flash.h"

#ifdef __HWDEVICE
#include "ADA2200.h"        // HAL ADA2200
#endif

typedef struct {
    sample_t T; // Use uint32_t and store (Ti+1 - Ti)
    sample_t A;
} point_t;

extern sample_t SSBUF[];
extern measurement_t g_measurement;
extern device_t g_dev;

static sample_t msCounter = 0; // Save in flash

#ifdef __SBNVM
static __prog__ uint8_t nvmDepot[DEPOT_SIZE] __attribute__((space(psv), aligned(DEPOT_SIZE)));
#endif

/* -------------------------------------------------------------------------- *
 * DEMO SIGNAL
 * 4 single samples,  N/2 sequenced samples
 * { [ampmax],[period],[nT,A],... }
 * -------------------------------------------------------------------------- */
const float SIG_T = 0.01; // 10ms (100Hz)
const uint16_t SIG_AMPLITUDE = 2047;
//const float Pi = 3.141592653589793;
const float _2Pi = 6.28;

#ifndef __SIG0_PP

uint16_t acquireSin(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t adc_period) {
    sample_t nT = 0;
    unsigned short maxSize;
    sample_t *pSSBUF = dbuf;
    float Ft, dTr = 0, kTr = 0;

    kTr = _2Pi * (adc_period / 1000); // SIG_T; // milliseconds 

    maxSize = maxpoints - 2; // Add <Period> & <Offset> as single sample 
    *pSSBUF = SIG_AMPLITUDE;
    *(pSSBUF + 1) = (sample_t) (adc_period * 1000); // 
    pSSBUF += 2;

    maxSize >>= 1; // Couples of sequenced samples { [dT,A],... }
    while ((nT < maxSize) && ((nT * SIG_T) <= nsec)) { // !TMR1_Timeout_sec(0)) {
        *pSSBUF = (nT == 0) ? 0 : 1;
        Ft = SIG_AMPLITUDE + (sin(dTr) * SIG_AMPLITUDE);
        *(pSSBUF + 1) = (sample_t) Ft;
        pSSBUF += 2;
        dTr += kTr;
        if (dTr > (_2Pi)) {
            dTr -= (_2Pi);
        }
        nT++;
    }
    return (nT);
}

#else

bool ppFilter(sample_t t, sample_t v, sample_t * ppBUF) {
    static point_t points[SAMPLING_AV_PBUFFER];
    static sample_t pIndex;
    bool PPmatch;
    bool pm01, pm12, pu01, pu12;
    int i;

    if (t == 0) { // Start
        pIndex = 0;
    }

    points[pIndex].A = v; // point
    points[pIndex].T = t;
    pIndex++;

    if ((pIndex == SAMPLING_AV_PBUFFER)) { // Matching P-P
        i = 0;
        PPmatch = false;
        while ((++i < (SAMPLING_AV_PBUFFER - 1)) && !PPmatch) {
            pu01 = (points[i - 1].A == points[i].A);
            pu12 = (points[i].A == points[i + 1].A);
            PPmatch = false;
            if (!(pu01 | pu12)) {
                pm01 = (points[i - 1].A > (points[i].A + g_dev.cnf.calibration.av_filter)); // _snr = 30;
                pm12 = (points[i].A > (points[i + 1].A + g_dev.cnf.calibration.av_filter));
                PPmatch = (!pm01) != (!pm12);
            }
        }

        if (PPmatch) { // Save point
            *ppBUF = points[i].T; // Time
            ppBUF++;
            *ppBUF = points[i].A; // Amplitude
            pIndex = 0;
            while ((++i) < SAMPLING_AV_PBUFFER) {
                points[pIndex++] = points[i];
            }
        } else {

            points[0] = points[pIndex - 1];
            pIndex = 1;
        }
    } // Matching P-P
    return (PPmatch);
}

uint16_t acquireSin(measurement_t *ms, uint16_t nsec) {
    sample_t nT, sCounter, i;
    unsigned short maxSize;
    sample_t *pSSBUF;
    float Ft, dTr = 0, kTr = 0;
    sample_t tprec, ttprec;

    kTr = _2Pi * SIG_T; // ( g_config.calibration.av_period / 1000 );
    ms->dtime = RTCC_CurrentTimeL();
    ms->ss = SSBUF;
    pSSBUF = ms->ss;
    ms->typeset = _SIG0;

    // Single samples
    ms->ns = 2;
    *pSSBUF = SIG_AMPLITUDE; // Max Amplitude
    *(pSSBUF + 1) = (sample_t) (SIG_T * 1000); // Period
    pSSBUF += 2;

    // Sequenced samples
    maxSize = (unsigned short) (SS_BUF_SIZE - ms->ns); // Available buffer
    maxSize >>= 1; // 2 { [dT,A],... }
    sCounter = 0;
    nT = 0;

    while ((sCounter < maxSize) && ((nT * SIG_T) <= nsec)) {
        Ft = SIG_AMPLITUDE + (sin(dTr) * SIG_AMPLITUDE);

        if (ppFilter(nT, Ft, pSSBUF)) {
            pSSBUF += 2;
            sCounter++;
        }
        dTr += kTr;
        if (dTr > (_2Pi)) {
            dTr -= (_2Pi);
        }
        nT++;
    }
    ms->nss = sCounter * 2;


    //    pSSBUF = ms->ss; // Process samples's time ( p.T = delta-time )
    //    tprec = pSSBUF[ms->ns + 0 ];
    //    for (i = 1; i < sCounter; i++) {
    //        ttprec = pSSBUF[ms->ns + (i * 2)];
    //        pSSBUF[ms->ns + (i * 2)] -= tprec;
    //        tprec = ttprec;
    //    }

    return (ms->ns + ms->nss);
}


#endif

/* -------------------------------------------------------------------------- *
 * MEASURE ENVIROMENT TEMPERATURE
 * Read analog pin value (ADC)
 * Sum readed values to compute everage on 6 measures.
 * -------------------------------------------------------------------------- */
uint16_t acquireET(sample_t* dbuf) { // RealTime ?
#ifdef __VAMP1K_TEST
    printf("acquireET\n");
#endif    
#if defined(__HWDEVICE)
    // ____________________________________ADC Input Pin
    ET_IN_SetAnalog();
    ET_IN_SetAnalogInput(); // Input  AN5/C1INA/C2INC/SCL2/CN7/RB3 (7)
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
    // ____________________________________Acquire
    AD1CON1bits.ADON = 1; // Turn on A/D
    //while (0) {
    AD1CON1bits.SAMP = 1; // Start sampling the input
    __delay(1); // Ensure the correct sampling time has elapsed
    AD1CON1bits.SAMP = 0; // End sampling and start conversion
    while (!AD1CON1bits.DONE) {
        //printf("adc=%d \n", ADC1BUF0);
    }
    AD1CON1bits.ADON = 0;
    *dbuf = (sample_t) ADC1BUF0;
    return (1); // check !!!!!
#else
    *dbuf = 0;

    return (0); // Hardware not supported
#endif
}


/*----------------------------------------------------------------------------*
 MEASURE WIND SPEED
 Count the pulses (rotations) on pin for fixed time period (1sec):
    wind_speed = pulses * wsFactor = m/s                
 *----------------------------------------------------------------------------*/

#define _nWSS (SAMPLING_WS_EVERAGE+1)
volatile int _icycle;
volatile int _wsready;
volatile int _wsptime[_nWSS];

void everySecond(void) {
    if (_wsready) { // Stop
        _wsptime[_icycle] = TMR4;
        _icycle++;
    } else { // Start

        _wsready = 1;
        TMR4 = 0;
    }
}

uint16_t acquireWS(sample_t* dbuf) {
    int i;
#ifdef __VAMP1K_TEST
    printf("acquireWS\n");
#endif
#if defined(__HWDEVICE)
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
    for (i = 0; i < _nWSS; i++) {
        _wsptime[i] = 0;
    }
    _icycle = 0;
    _wsready = 0;

    Timeout_SetCallBack(everySecond);
    T4CONbits.TON = 1; // Starts Timery
    setTimeout(1, 0); // TMR1 1Second Start !

    while ((_icycle < _nWSS)) {
        Nop();
    }
    unsetTimeout();
    T4CONbits.TON = 0;

    for (i = 1; i < _icycle; i++) { // Compute average
        _wsptime[1] += _wsptime[i];
    }
    *dbuf = (_wsptime[1] / (_icycle - 1));
    return (_icycle > 0);
#else
    *dbuf = 0;

    return (0); // Hardware not supported
#endif    
}


/* -------------------------------------------------------------------------- */
/* MEASURE AEOLIAN VIBRATION
 *
 *                                                                            */
/* -------------------------------------------------------------------------- */


volatile bool _adcReady;
// volatile sample_t Tc; 

void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void) { // , weak

    IFS0bits.AD1IF = 0; // Clear Int flag immediatelly
    _adcReady = true;
}// End ADC

void acquireAV_INIT() {
    // =============== BEGIN:INIT    
    // TMR3 as pulse counter on T3CK pin (SYNCO)
    AV_SYN_SetDigital();
    AV_SYN_SetDigitalInput(); // Input T3CK/RB15 (SYNCO)
    //
    T2CONbits.T32 = 0; // Configure TMR3 16Bit Operation
    T3CON = 0x00; //Timer 3 Control Register
    T3CONbits.TCS = 1; // clock source T3CK pin
    T3CONbits.TCKPS = 0b00; // 1:8 Prescale value ( Fosc/2/8 = 500Khz )
    TMR3 = 0x00; //TMR3 Timer3 Counter Register  
    //PR3 = 15 ; // Synco 38.4 Khz : 4 = 9.6  Khz  0,104 ms 
    //PR3 = 31 ; // Synco 38.4 Khz : 5 = 7.68 Khz  0,131 ms 
    //PR3 = 63;  // Synco 38.4 Khz : 8 = 4.8Khz    0,208 ms 
    PR3 = 127; // Synco 38.4 Khz : 16 = 2.4Khz  0,416 ms  [(2^7)-1]
    //PR3 = 191; // Synco 38.4 Khz : 24 = 1.6Khz  0,625 ms  [(2^8)-1]
    //PR3 = 255; // Synco 38.4 Khz : 32 = 1.2Khz  0,833 ms  
    // PR3 = adc_freq;
    // 200Hz 5ms 
    IFS0bits.T3IF = 0; // Reset Int vector
    IEC0bits.T3IE = 0; // Int call-back disabled (Trig ADC)

    // ____________________________________Input Analog pins
    AV_INP_SetAnalog(); // RA0 AN0 (2 DIP20) VRef+
    AV_INP_SetAnalogInput();
    AV_INN_SetAnalog(); //  RA1 AN1 (3 DIP20) VRef-
    AV_INN_SetAnalogInput();
    // ____________________________________A/D Converter Setup
    AD1CON1bits.ADON = 0; // Converter off
    AD1CON2 = 0; // Inputs are not scanned
    AD1CSSL = 0; // No scan //ADC1MD bit in the PMD1
    // ____________________________________Clock and Conversion Mode
    AD1CON1 = 0x2200; // No operation in Idle mode (ADSIDL=1)
    AD1CON1bits.MODE12 = 1; // Resolution 12 bit
    AD1CON1bits.SSRC = 0x2; // Timer 3 trig ADC
    AD1CON1bits.ASAM = 1; // Auto-Convert ON ( end sampling and start conversion )
    AD1CON1bits.FORM = 0b01; // Samples Format (Decimal result, signed, right-justified)
    // 11 = Fractional result, signed, left-justified
    // 10 = Absolute fractional result, unsigned, left-justified
    // 01 = Decimal result, signed, right-justified
    // 00 = Absolute decimal result, unsigned, right-justified
    // ____________________________________Buffering Mode
    AD1CON2bits.BUFREGEN = 0; // A/D result buffer is treated as a FIFO
    AD1CON2bits.BUFM = 0; // No alternate half-buffer (starts ADCBUF0)
    AD1CON2bits.SMPI = 0; // Interrupts every samples
    // ____________________________________Conversion Timing
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0; // Clock is derived from the system clock (Tcy= 1/Fcy)
    AD1CON3bits.EXTSAM = 1; // Extended Sampling Time bit
    AD1CON3bits.SAMC = 14; // 16 Auto-Sample Time TAD
    AD1CON3bits.ADCS = 0b010; // 0b00111110; // ADC Clock Select ( 4Tcy=1TAD 250nS*4 = 1uS)
    // 00111111 = 64·TCY = TAD
    // 00000001 = 2·TCY = TAD
    // ____________________________________Input channels & References
    AD1CON5bits.BGREQ = 1; // Band Gap Request bit ( 1 = BGP enabled )
    AD1CON2 = 0; // Inputs are not scanned
    AD1CHS = 0;
    AD1CON2bits.PVCFG = 0b10; // 0b11; // Positive Voltage Reference
    //11 = 4 * Internal VBG(2)
    //10 = 2 * Internal VBG(3)
    //01 = External VREF+
    //00 = AVDD
    AD1CON2bits.NVCFG = 0; // Negative Voltage Reference
    AD1CHSbits.CH0SA = 0; // S/H+ input (00001 = AN1)
    AD1CHSbits.CH0NA = 0b10; // S/H- input (001 = AN0)
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
    IPC3bits.AD1IP = 2; // High Interrupt Priority
    //
    // -------------
    ADA2200_Enable(); // Power-on: SPI1, Sensor Board LINE1
    // -------------
    _adcReady = false;
    // =============== END:INIT   
}

void acquireAV_START() {
    // =============== BEGIN:START   
    IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
    T3CONbits.TON = 1;
    AD1CON1bits.ADON = 1; // Start ADC
    ADA2200_Synco(true); // Enable SYNC 
    // =============== END:START
}

void acquireAV_STOP() {
    // =============== BEGIN:STOP   
    AD1CON1bits.ADON = 0; // Converter Off
    T3CONbits.TON = 0;
    ADA2200_Disable();
    // =============== END:STOP  
}

uint16_t acquireAV(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period) {
#ifdef __VAMP1K_TEST
    printf("acquireAV\n");
#endif
#if defined(__HWDEVICE)
    uint16_t ppPoints;
    uint16_t pIndex;
    sample_t *pSSBUF = dbuf;
    sample_t Tc, Tpc; //  Period = ( 1 / Freq_INT0 ) * nsample
    bool pm01, pm12;
    point_t points[SAMPLING_AV_PBUFFER];
    int diff;

    acquireAV_INIT();

    Tc = 0;
    Tpc = 0; //
    pIndex = 0;
    ppPoints = 0;

    maxpoints--; // Used to store <period> and <scale>
    *pSSBUF = 40; // period = 0,004 ms  (value/10000)
    pSSBUF++;
    *pSSBUF = 2047;
    pSSBUF++;
    maxpoints--; // Reserve one for last sample

    acquireAV_START();
    setTimeout(nsec, 0);

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

                if (diff < g_dev.cnf.calibration.av_filter) { // Valutare nell'intorno del filtro               
                    points[0] = points[1];
                    points[1] = points[2];
                    pIndex = 2;
                } else {
                    diff = points[1].A - points[2].A;
                    if (diff < 0) {
                        diff = points[2].A - points[1].A;
                    }

                    if (diff < g_dev.cnf.calibration.av_filter) { // Valutare nell'intorno del filtro             
                        pIndex = 2;
                    } else {
                        // PPmatch = false;
                         pm01 = (points[0].A > (points[1].A + g_dev.cnf.calibration.av_filter)); // _snr = 30;
                         pm12 = (points[1].A > (points[2].A + g_dev.cnf.calibration.av_filter));
                         if ((!pm01) != (!pm12)) { // Save PP point

                        //pm01 = (points[0].A <= (points[1].A + g_dev.cnf.calibration.av_filter)); // _snr = 30;
                        //pm12 = (points[1].A <= (points[2].A + g_dev.cnf.calibration.av_filter));
                        //if (pm01 != pm12) { // Save PP point
                            
                            *pSSBUF = (points[1].T - Tpc); // Time
                            Tpc = points[1].T;
                            pSSBUF++;

                            *pSSBUF = points[1].A; // Amplitude
                            pSSBUF++;

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
                *pSSBUF = points[0].T;
                pSSBUF++;
                *pSSBUF = points[0].A;
                pSSBUF++;
                ppPoints++;
            }
        }// _adcReady

    }
    acquireAV_STOP();

    *pSSBUF = points[SAMPLING_AV_PBUFFER - 1].T - Tpc; // Last Sample Tn  
    pSSBUF++;
    *pSSBUF = points[SAMPLING_AV_PBUFFER - 1].A; // Amplitude
    ppPoints++;
    return (ppPoints);

#else
    *dbuf = 0;
    return (0); // Hardware not supported
#endif   
}


#ifdef __SBNVM

uint16_t acquireAV_NVM(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period) {

#ifdef __VAMP1K_TEST
    printf("acquireAVFFT\n");
#endif

#if defined(__HWDEVICE)
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

    acquireAV_START();
    setTimeout(nsec, 0);

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

//==============================================================================
//==============================================================================

uint16_t measurementAcquire(measurement_t * ms) {
    sample_t *PtrSSBUF = SSBUF;
    sample_t nsamples;

#ifndef __HWDEVICE  // only demo signal without sensors board 
    g_dev.cnf.general.typeset = _SIG0;
#endif
#ifdef __VAMP1K_TEST
    printf("Start tset=%u\n", g_dev.cnf.general.typeset);
#endif

    switch (g_dev.cnf.general.typeset) { // g_config.general.samplingmode

        case _SIG0: // Demo signal
            ms->ss = SSBUF;
            ms->typeset = _SIG0;
            ms->dtime = RTCC_CurrentTimeL(); // get RTC datetime
            ms->ns = 2; // ET,WS   
            *PtrSSBUF = 0x0; // Single Sample 1
            PtrSSBUF++;
            *PtrSSBUF = 0x0; // Single Sample 2
            PtrSSBUF++;
            nsamples = acquireSin(PtrSSBUF, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - ms->ns) / 2), g_dev.cnf.calibration.av_period); // nsamples
            ms->ns = 4; // <temperature>,<windspeed>,<tick_duration>,<scale_offset> populated by acquireAV
            ms->nss = (nsamples - 1) * 2; // Sequenced [<t,av>,...]
            break;

        case _AV00: // Aeolian Vibration, Points [<ET>,<WS>,<PER>,<OFF>,{<d(T)>,<A>,...}]
            // RETURN: 
            // n-acquired points ( 2 samples each one  )
            // First point is used to store <tick_duration>, <scale_offset>
            // Second to store first sampled value at T=0
            // Last used to store last sampled point at T=last-sampling-tick
            Device_SwitchMode(PW_ON_SAMP_WST); // lastPwrState = Device_SwitchPower(PW_ON_SAMP_WST);
            ms->ss = SSBUF;
            ms->typeset = _AV00;
            ms->dtime = RTCC_CurrentTimeL(); // get RTC datetime
            ms->ns = 2; // ET,WS

            acquireET(PtrSSBUF); // RET: success
            PtrSSBUF++;
            acquireWS(PtrSSBUF); // RET: success
            PtrSSBUF++;
            Device_SwitchMode(PW_ON_SAMP_ADA);

#ifndef __SBNVM
            nsamples = acquireAV(PtrSSBUF, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - ms->ns) / 2), g_dev.cnf.calibration.av_period);
#else            
            nsamples = acquireAV_NVM(PtrSSBUF, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE_NVM - ms->ns) / 2), g_dev.cnf.calibration.av_period);
#endif            
            ms->ns = 4; // <temperature>,<windspeed>,<tick_duration>,<scale_offset> populated by acquireAV
            ms->nss = (nsamples - 1) * 2; // Sequenced [<t,av>,...]
            Device_SwitchMode(PW_ON_DEFAULT); // Device_SwitchPower(lastPwrState);
            // Reorder / Process FFT 
            
            
            break;

        case _AV01: // Aeolian Vibration, Occurencies [<ET>,<WS>,<PER>,<OFF>,{<Occ>,<Amp>,<Per>,...}]
            // RETURN: 
            // classified acquired vibration ( 3 samples each one  )

            break;

        case _SS00: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
            break;
    };

    return (nsamples);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementSave(measurement_t * ms) {
    if ((ms->ns + ms->nss) > 0) {

        msCounter++;
    };
    return (msCounter);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementCounter() {

    return (msCounter);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementLoad(uint16_t index, measurement_t * ms) {
    int16_t msIndex = 0;
    if (index <= msCounter) {

        msIndex = index;
        memcpy(ms, &g_measurement, sizeof (measurement_t));
    }
    return (msIndex);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementDelete(uint16_t index) { // ret: 0/Counter
    int16_t msIndex = 0;
    if (msCounter > 0) {

        msCounter--;
        msIndex = index;
    }
    return (msIndex);
}

// ============================================================================
// ret n samples each measure

unsigned short getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp) {

    switch (mtype) {

        case MEASURE_WT: // Wind speed & Temp
            switch (cmd) {
                case __INIT:

                case __START:
                    // Enables Modules & Pins
                    // Change processor speed
                    // Configure Modules
                    // Start modules
                    break;

                case __READ: // [ ET,WS ]
                    // Read sample
                    acquireET(nsamp);
                    acquireWS((nsamp + 1));
                    return (2);
                    break;

                case __STOP: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
                    // Stop Modules
                    // Change processor speed
                    // Disable Modules & Release Pins
                    break;
            }
            break;

        case MEASURE_WTL: // Wind speed, Temp, Lvdt
            switch (cmd) {
                case __INIT:
                case __START: // Demo signal
                    break;
                case __READ: // Vamp1K Aeolian Vibration [ ET,WS,{A,hT} ]
                    break;
                case __STOP: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
                    break;
            }
            break;


    }
    return 0;
}

void getMeasurementBlock(uint8_t *pbuf, uint16_t offset, uint16_t size) {
#ifdef __SBNVM    
    uint32_t read_data;
    uint32_t nvm_address;
    uint16_t count;

    size /= 3;
    nvm_address = __builtin_tbladdress(nvmDepot); // Get address of flash

    for (count = 0; count < size; count++) {
        read_data = FLASH_ReadWord24(nvm_address + (offset * 2) + count * 2);
        *(pbuf) = read_data >> 16;
        *(pbuf + 1) = read_data >> 8;
        *(pbuf + 2) = read_data;
        pbuf += 3;
    }
#else
    //memcpy(&buffer[offset], &ms.ss[x * BLOCK_MAXSAMPLES], (blockSize * 2));
    memcpy(pbuf, &g_measurement.ss[offset], size);
#endif    
}

