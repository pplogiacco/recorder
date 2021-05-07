#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>

#include "measurement.h"
#include "acquire.h"

#include "../utils.h"
#include "../modules/RTCC.h"
//#include "../memory/flash702.h"

//==============================================================================
// extern sample_t SSBUF[];
static sample_t SSBUF[SS_BUF_SIZE]; // measurement's samples buffer
extern measurement_t g_measurement;
extern device_t g_dev;

static sample_t msCounter = 0; // Use: g_dev.st.measurmanet_counter

#ifdef __AV0NVM
static __prog__ uint8_t nvmDepot[DEPOT_SIZE] __attribute__((space(psv), aligned(DEPOT_SIZE)));
#endif

//==============================================================================

uint16_t measurementAcquire(measurement_t * ms) {
    sample_t *ptrSS = SSBUF;
    sample_t nsamples;
    uint16_t adc_fq;

#ifdef __VAMP1K_TEST
    printf("Tset=%u\n", g_dev.cnf.general.typeset);
    //g_dev.cnf.general.typeset = _AV00;
#endif

    if ((g_dev.cnf.general.typeset == _AV00) || (g_dev.cnf.general.typeset == _AV01) || (g_dev.cnf.general.typeset == _AV04)) {
        ms->ss = ptrSS;
        ms->dtime = RTCC_GetTimeL(); // get RTC datetime
        /* ----------------- TMR3/ADC Frequency    
        3: 2^3 -1,  PR3 = 63, Synco 38.4 Khz : 8 =  4.8Khz, T=0,000208s
        4: 2^4 -1,  PR3 = 127, Synco 38.4 Khz :16 = 2.4Khz, T=0,000416s 
        5: 2^5 -1,  PR3 = 255, Synco 38.4 Khz :32 = 1.2Khz, T=0.000833s 
        6: 2^6 -1,  PR3 = 511, Synco 38.4 Khz :64 = 600Hz, T=0.0017s (1,7 ms) */
        if ((g_dev.cnf.calibration.av_period < 2) || (g_dev.cnf.calibration.av_period > 6)) {
            g_dev.cnf.calibration.av_period = 6; // Default: SYNCO_FREQUENCY / 2^6 = 600Hz
        }
        adc_fq = (1U << (g_dev.cnf.calibration.av_period - 1)) - 1; // Frequency divider, compute PR3 

        // ----------------- Add Single Samples
        Device_SwitchSys(SYS_ON_SAMP_WST); // lastPwrState = Device_SwitchPower(PW_ON_SAMP_WST); 
        acquireET(ptrSS); // RET: success
        ptrSS++;
        acquireWS(ptrSS); // RET: success
        ptrSS++;
        *ptrSS = SYNCO_FREQUENCY >> g_dev.cnf.calibration.av_period; // Compute ADC frequency (Hz)
        ptrSS++;
        *ptrSS = 4200; // Scale Resolution 2^12 (12 Bit ADC)
        ptrSS++;
        ms->ns = 4;
        // -----------------
        Device_SwitchSys(SYS_ON_SAMP_ADA);
    }


    switch (g_dev.cnf.general.typeset) {

        case _AV00: // Aeolian Vibration, RAW
            ms->typeset = _AV00;
            nsamples = acquireAV(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, 0);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;


        case _AV01: // Aeolian Vibration, Peak-Peak
            ms->typeset = _AV01;
            nsamples = acquireAV(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, \
                    (g_dev.cnf.calibration.av_filter < 1) ? 1 : g_dev.cnf.calibration.av_filter);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;

        case _AV02: // Aeolian Vibration, FFT 
            Device_SwitchSys(SYS_ON_SAMP_WST); // lastPwrState = Device_SwitchPower(PW_ON_SAMP_WST);      
            ms->ss = SSBUF;
            ms->typeset = _AV02; // g_dev.cnf.general.typeset;
            ms->dtime = RTCC_GetTimeL(); // get RTC datetime
            ms->ns = 4; // ET,WS

            acquireET(ptrSS); // RET: success
            ptrSS++;
            acquireWS(ptrSS); // RET: success
            ptrSS++;

            switch (g_dev.cnf.calibration.av_period) {
                case 3: // ADC_FRQ_24Khz
                    *ptrSS = 2400; // Sampling freq. (Hz)
                    adc_fq = ADC_FRQ_24Khz; //  PR3=3200U
                    break;
                case 4: // ADC_FRQ_1Khz
                    *ptrSS = 1000; // Sampling freq. (Hz))
                    adc_fq = ADC_FRQ_1Khz; //  PR3=8000U
                    break;

                default: // ADC_FRQ_05Khz
                    *ptrSS = 500; // Sampling freq. (Hz))
                    adc_fq = ADC_FRQ_05Khz; //  PR3=8000U
                    break;
            }
            *(ptrSS + 1) = 1U < 12; // max Scale
            ptrSS += 2;

            Device_SwitchSys(SYS_ON_SAMP_ADA);

            // Manage g_dev parameters  
            // uint16_t acquireAV_FFT(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t adc_fq, uint16_t fft_pw)
            short m = 0;
            short l2 = (SS_BUF_SIZE - ms->ns);

            while (l2 > 0) { // log2_npoints
                m++;
                l2 >>= 1;
            }
            // Force ADC frequency 0.5Khz ( g_dev.cnf.calibration.av_period )

            nsamples = acquireAV_FFT(ptrSS, g_dev.cnf.general.cycletime, 10, \
                                 adc_fq, g_dev.cnf.calibration.av_filter);

            //ms->ns = 4; // <temperature>,<windspeed>,<tick_period>,<scale_offset> populated by acquireAV
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            // Process samples / Format measurament with only significative harmonics
            // Send only ((2^m)/2) samples, from 1 to N2+1
            //ms->ss = SSBUF; // Return 1..N/2+1 
            //

            ms->nss = (nsamples >> 1); // 512 Coefficients ( only positive - half spectrum )
            // int i;
            //            ptrSS = (ms->ss + ms->ns);
            //            for (i = 0; i < ms->nss; i++) {
            //                *(ptrSS + i) = *(ptrSS + i + 1);
            //            }

            break;

        case _AV04: // Aeolian Vibration, No DTime
            ms->typeset = _AV01;
            nsamples = acquireAV(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, \
                    (g_dev.cnf.calibration.av_filter < 1) ? 1 : g_dev.cnf.calibration.av_filter);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;

        case _SS00: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
            break;

        case _SIG0: // Demo signal
            ms->ss = ptrSS;
            ms->typeset = _SIG0;
            ms->dtime = RTCC_GetTimeL(); // get RTC datetime

            *ptrSS = g_dev.cnf.calibration.av_period; // Add <sig_freq> as single sample 
            *(ptrSS + 1) = g_dev.cnf.calibration.av_filter; // Add <sig_maxamp> as single sample 
            ptrSS += 2;
            nsamples = acquireSig(ptrSS, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - 2) / 2), \
                    g_dev.cnf.calibration.av_period, g_dev.cnf.calibration.av_filter);
            ms->ns = 4; // Use first samples in buffer as singles {<adc_freq>,<res_scale>} 
            ms->nss = (nsamples - 1) * 2;
            // { <sig_freq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
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

uint16_t getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp) {

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
#ifdef __AV0NVM    
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


