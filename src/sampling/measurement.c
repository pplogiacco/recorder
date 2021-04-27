#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>
//#include "sampling/sampling.h"

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
    sample_t *ptrSSBUF = SSBUF;
    sample_t nsamples;

#if !defined(__SENSOR_BOARD) || defined(__VAMP1K_TEST_SIG0_FFT)  
    g_dev.cnf.general.typeset = _SIG0;
#endif

#ifdef __VAMP1K_TEST
    printf("Tset=%u\n", g_dev.cnf.general.typeset);
#endif

    switch (g_dev.cnf.general.typeset) { // g_config.general.samplingmode

        case _SIG0: // Demo signal
            ms->ss = SSBUF;
            ms->typeset = _SIG0;
            ms->dtime = RTCC_GetTimeL(); // get RTC datetime

#if !(defined( __PIC24FJ256GA702__ ) && defined(__VAMP1K_TEST_SIG0_FFT))  
            ms->ns = 2; // ET,WS   
            *ptrSSBUF = 0x0; // Single Sample 1
            ptrSSBUF++;
            *ptrSSBUF = 0x0; // Single Sample 2
            ptrSSBUF++;
            //------------------------------------------
            // add singles samples: <tick_duration>, <scale_offset>
            // add couples of { delta-time, amplitude }
            // return nsamples
            nsamples = acquireSig(ms->ss, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - ms->ns) / 2), \
                       g_dev.cnf.calibration.av_period); // av_period is signal frequency in Hz       
            ms->ns = 4; // <value1>, <value2>, <tick_period>, <scale>
            ms->nss = (nsamples - 1) * 2; // Sequenced [<t,av>,...]
#else
            nsamples = acquireSig(ms->ss, 10, (SS_BUF_SIZE / 2), \
                       40); //  frequency in Hz
            ms->ns = 0; // <value1>, <value2>, <tick_period>, <scale>
            ms->nss = nsamples * 2; // 2048 samples 1024 points
            ptrNSS = ms->ss + 2; // Point to first value 

            //FFT_FillSinewave(); // Init tables
            // one cycle sine table required for FFT
            int ii;
            for (ii = 0; ii < N_WAVE; ii++) {
                Sinewave[ii] = float2fix14(sin(6.283 * ((float) ii) / N_WAVE)*0.5);
                window[ii] = float2fix14(1.0 * (1.0 - cos(6.283 * ((float) ii) / (N_WAVE - 1))));
                //window[ii] = float2fix(1.0) ;
            }
            int i; // load input array
            for (i = 0; i < nsamples; i++) {
                *(ptrSSBUF + i) = multfix14(*(ptrNSS + (i * 2) + 1), window[i]);

                //fr[i] = multfix14(v_in[i], window[i]);
                //fi[i] = 0;
            }

            ptrNSS = (ms->ss + nsamples);
            memset(ptrNSS, 0, nsamples);
            printf("FFT (n=%u,m=%u)\n", nsamples, 10);

            /* for (sample_number = 0; sample_number < nSamp - 1; sample_number++) {
                        // window the input and perhaps scale it
                        fr[sample_number] = multfix14(v_in[sample_number], window[sample_number]);
                        fi[sample_number] = 0;
                    }
             */
            // do FFT
            //FFTfix(PtrSSBUF, ptrNSS, LOG2_N_WAVE);

            //---
            //fft(PtrSSBUF, ptrNSS, 10); // Perform the FFT
            //---
            for (i = 0; i < (1U << LOG2_N_WAVE); i++) {
                printf("#%u: %u)\n", i, ptrNSS[i]);
            }
#endif
            break;

        case _AV00: // Aeolian Vibration, Points [<ET>,<WS>,<PER>,<OFF>,{<d(T)>,<A>,...}]
            // RETURN: 
            // n-acquired points ( 2 samples each one  )
            // First point is used to store <tick_duration>, <scale_offset>
            // Second to store first sampled value at T=0
            // Last used to store last sampled point at T=last-sampling-tick
            Device_SwitchSys(SYS_ON_SAMP_WST); // lastPwrState = Device_SwitchPower(PW_ON_SAMP_WST);      
#if !defined(__VAMP1K_TEST_adc_printf)            

            ms->ss = SSBUF;
            ms->typeset = _AV00;
            ms->dtime = RTCC_GetTimeL(); // get RTC datetime
            ms->ns = 2; // ET,WS

            acquireET(ptrSSBUF); // RET: success
            ptrSSBUF++;
            acquireWS(ptrSSBUF); // RET: success
            ptrSSBUF++;
#endif            
            Device_SwitchSys(SYS_ON_SAMP_ADA);

            nsamples = acquireAV00(ptrSSBUF, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - ms->ns) / 2), \
                                  g_dev.cnf.calibration.av_period, g_dev.cnf.calibration.av_filter);

            ms->ns = 4; // <temperature>,<windspeed>,<tick_period>,<scale_offset> populated by acquireAV
            ms->nss = (nsamples) * 2; // Sequenced [<t,av>,...]
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            // Reorder / Process FFT 
            break;

        case _AV01: // Aeolian Vibration, Occurencies [<ET>,<WS>,<PER>,<OFF>,{<Occ>,<Amp>,<Per>,...}]

            Device_SwitchSys(SYS_ON_SAMP_WST); // lastPwrState = Device_SwitchPower(PW_ON_SAMP_WST);      

            ms->ss = SSBUF;
            ms->typeset = _AV01; // g_dev.cnf.general.typeset;
            ms->dtime = RTCC_GetTimeL(); // get RTC datetime
            ms->ns = 4; // ET,WS

            acquireET(ptrSSBUF); // RET: success
            ptrSSBUF++;
            acquireWS(ptrSSBUF); // RET: success
            ptrSSBUF++;

            *ptrSSBUF = 500; // Sampling freq. (Hz))
            ptrSSBUF++;
            *ptrSSBUF = 1U < 12; // max Scale
            ptrSSBUF++;

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
     
            
            nsamples = acquireAV_FFT(ptrSSBUF, g_dev.cnf.general.cycletime, 10, \
                                 ADC_FRQ_05Khz, g_dev.cnf.calibration.av_filter);

            //ms->ns = 4; // <temperature>,<windspeed>,<tick_period>,<scale_offset> populated by acquireAV

            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            // Process samples / Format measurament with only significative harmonics
            // Send only ((2^m)/2) samples, from 1 to N2+1
            ms->nss = nsamples; // send 1024 couple of samples

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


