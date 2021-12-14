#include <xc.h>
#include <math.h>
#include <stdio.h>   // printf
#include <stdlib.h>  // abs
#include <string.h>

#include "acquire.h"
#include "measurement.h"

#include "../utils.h"
#include "../modules/RTCC.h"

#include "../memory/DEE/dee.h"
#include "../memory/SST26VF064B.h"  // Flash SPI

#include "../daas/libdpt.h"

#define DEPOT_OK
//==============================================================================
extern device_t device;
extern sample_t SSBUF[SS_BUF_SIZE]; // global buffer
extern measurement_t lmeas;

//#ifdef __VAMP1K_TEST 
//extern sample_t SSBUF[SS_BUF_SIZE]; // global buffer
//extern measurement_t lmeas;
//#else
//static sample_t SSBUF[SS_BUF_SIZE]; // global buffer
//measurement_t lmeas;
//#endif

// static uint16_t m_counter = 0;
// extern measurement_t tmpMeasurement; // Debug 

//measurement_t * measurementGetPTR() {
//    return (&lmeas);
//}


//==============================================================================

//void measurementInitialize()
//{
//    lmeas.ss = SSBUF;
//    //m_counter = 0; // g_dev.st.meas_counter;
//   DEE_Read(EEA_MEAS_COUNTER, &m_counter);
//}

//uint16_t measurementCounterGet() {
//    return m_counter; // number of available measures
//}

uint16_t measurementAcquire() // ( tset, duration, adc_fq, filter )
{
    sample_t *ptrSS;
    uint16_t nsamples = 0;
    uint8_t m; // FFTs

    typeset_t typeset = device.cnf.general.typeset;
    uint16_t duration = device.cnf.general.cycletime;
    uint16_t fq_Hz = device.cnf.calibration.av_period;
    uint16_t filter = device.cnf.calibration.av_filter;

#ifdef __VAMP1K_TEST
    printf("Acquiring t=%u, freq=%u\n", typeset, (uint8_t) fq_Hz);
#endif

    lmeas.dtime = RTCC_GetTimeL(); // get RTC datetime  
    lmeas.tset = typeset;
    lmeas.ss = &(SSBUF[0]);
    ptrSS = lmeas.ss;

    if (typeset != _SIG0) {
        // ----------------- Add Single Samples
        Device_SwitchSys(SYS_ON_SAMP_WST);
        acquireET(ptrSS); // RET: success
        ptrSS++;
        acquireWS(ptrSS); // RET: success
        Device_SwitchSys(SYS_DEFAULT);
        ptrSS++;
        *ptrSS = fq_Hz; // Sampling Frequency (Hz)
        ptrSS++;
        *ptrSS = 1U << 12; // Sampling Resolution 2^12 (12 Bit ADC)
        ptrSS++;
        lmeas.ns = 4;
        // -----------------
    }

    switch (typeset) {

        case _SIG0: // (02) Test Signal { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
            //tmpM.tset = _SIG0;
            lmeas.nss = acquireSig(ptrSS, duration, SS_BUF_SIZE , fq_Hz, filter, true) - 4;
            lmeas.ns = 4; // <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV00: // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
            //tmpM.tset = _AV00;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            lmeas.nss = acquireAV_RAW(ptrSS, duration, (SS_BUF_SIZE - lmeas.ns), fq_Hz);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV01: // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
            //tmpM.tset = _AV01;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            lmeas.nss = acquireAV_P2P(ptrSS, duration, (SS_BUF_SIZE - lmeas.ns), fq_Hz, filter);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV02: // Aeolian Vibration, FFT 
            //tmpM.tset = _AV02; // typeset;
            //  m = 0;
            //  l2 = (SS_BUF_SIZE - tmpMeasurement.ns);
            //  while (l2 > 0) { // log2_npoints
            //    m++;
            //    l2 >>= 1;
            //  }
            m = 10; // 2048 Point FFT -> 255 Coefficients
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_DFT(ptrSS, duration, m, fq_Hz);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            lmeas.nss = (nsamples >> 1); // 512 Coefficients ( only positive - half spectrum )
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV03: // Aeolian Vibration, Comples FFT 
            lmeas.nss = 0;
            nsamples = 0;
            break;

        case _AV04: // Aeolian Vibration, RAW No DTime
            //tmpM.tset = _AV04;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            lmeas.nss = acquireAV_RNT(ptrSS, duration, (SS_BUF_SIZE - lmeas.ns), fq_Hz);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV05: // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<freq>,<amp>),...]}
            m = 8;
            lmeas.tset = _AV05;
            lmeas.ns++; // Add <duration>
            *ptrSS = duration * (((float) 1 / (fq_Hz << 1)) * (1 << m)); // total time 
            ptrSS++;

            Device_SwitchSys(SYS_ON_SAMP_ADA);
            // -- Based P2P
            sample_t* wbPtr;
            uint16_t wbSize, maxsamples, maxoss, noss;
            uint16_t i, j, nsec;
            point_t* pPtr;
            uint16_t oT, oA;
            oscill_t * oPtr;

            wbSize = 1024; // Work Buffer Size
            maxsamples = ((SS_BUF_SIZE - lmeas.ns) - wbSize);
            maxoss = maxsamples / 3; // Size of tuple in sample_t

            wbPtr = ptrSS + maxsamples; // begin of working buffer
            pPtr = (point_t*) wbPtr; // use working buffer as points

            for (i = 0; i < maxoss; i++) { // Initialize measurement result
                *(ptrSS + i) = 0x0;
            }

            oPtr = (oscill_t *) ptrSS; // use ptrSS as touples of samples
            noss = 0; // tuples counter

            filter = 20; // ignore too close peaks 
            nsec = duration; // compute on fq_Pr3 and wbSize

            do { // _____ Cycle total time....
                nsamples = acquireAV_P2P(wbPtr, 1, wbSize, fq_Hz, filter);
                nsamples = (nsamples >> 1) - 1;
                for (i = 1; i < nsamples; i++) { // All acquired samples less first and last
                    oA = abs(pPtr[i].A - pPtr[i + 1].A); // Oscillation amplitude
                    oT = pPtr[i + 1].T; // << 1; // Oscillation half period

                    j = 0;
                    while ((j < noss) && !((oPtr[j].A == oA) && (oPtr[j].T == oT))) { // Searching 
                        j++;
                    }
                    if (j == noss) { // Add new sample
                        if (noss < maxoss) {
                            oPtr[noss].n = 1;
                            oPtr[noss].T = oT;
                            oPtr[noss].A = oA;
                            noss++;
                        }
                    } else { // Update counter
                        oPtr[j].n++;
                    }
                }
                nsec--;
            } while (nsec > 0);
            // _________
            Device_SwitchSys(SYS_DEFAULT);
            lmeas.nss = noss * 3U; //  Vibration occurrencies
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _AV06: // (15) AVC-DFT { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<c>,<pw>),...]}
            m = 8;
            //tmpM.tset = _AV06;
            lmeas.ns++; // Add <duration>
            *ptrSS = duration * (((float) 1 / fq_Hz) * (1 << m)); // total time 
            ptrSS++;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            lmeas.nss = acquireAV_EVC_DFT(ptrSS, duration, (SS_BUF_SIZE - lmeas.ns), m, fq_Hz, filter, 0);
            Device_SwitchSys(SYS_DEFAULT);
            nsamples = lmeas.nss + lmeas.ns;
            break;

        case _SS00: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
            break;

    };
    return (nsamples);
}
/* -------------------------------------------------------------------------- */

#define HEADER_SIZE_IN_BYTE        10U   // Measurement's header

/* -------------------------------------------------------------------------- */
uint16_t measurementSave() // Uses g_measurement
{
    if ((lmeas.ns + lmeas.nss) > 0) {

#ifdef DEPOT_OK
        
        depotPush((uint8_t*) &(lmeas.ss[0]), ((lmeas.ns + lmeas.nss) *SAMPLE_SIZE_IN_BYTE)); // Save samples
        depotPush((uint8_t*) & lmeas, HEADER_SIZE_IN_BYTE);
#endif        
        device.sts.meas_counter++;
        DEE_Write(EEA_MEAS_COUNTER, device.sts.meas_counter);

    };
    return (device.sts.meas_counter);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementLoad(uint16_t index) { // LIFO !!!!!!!!!!!!!!

    if (device.sts.meas_counter > 0) {
#ifdef DEPOT_OK
        lmeas.ss = &(SSBUF[0]);        
        depotPull((uint8_t*) & lmeas, 0, HEADER_SIZE_IN_BYTE, false); // ok
        depotPull((uint8_t*) &(lmeas.ss[0]), HEADER_SIZE_IN_BYTE, ((lmeas.ns + lmeas.nss) * SAMPLE_SIZE_IN_BYTE), false);
#endif
        return index; //return device.sts.meas_counter; !!!!!!!!
    }
    return (0);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementDelete(uint16_t index) {
    if (device.sts.meas_counter > 0) {
#ifdef DEPOT_OK
        depotDrop(HEADER_SIZE_IN_BYTE + ((lmeas.ns + lmeas.nss) * SAMPLE_SIZE_IN_BYTE));
#endif
        device.sts.meas_counter--;
        DEE_Write(EEA_MEAS_COUNTER, device.sts.meas_counter);
    }
    return (device.sts.meas_counter);
}




// ============================================================================
//
//uint16_t getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp) {
//
//    switch (mtype) {
//
//        case MEASURE_WT: // Wind speed & Temp
//            switch (cmd) {
//                case __INIT:
//
//                case __START:
//                    // Enables Modules & Pins
//                    // Change processor speed
//                    // Configure Modules
//                    // Start modules
//                    break;
//
//                case __READ: // [ ET,WS ]
//                    // Read sample
//                    acquireET(nsamp);
//                    acquireWS((nsamp + 1));
//                    return (2);
//                    break;
//
//                case __STOP: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
//                    // Stop Modules
//                    // Change processor speed
//                    // Disable Modules & Release Pins
//                    break;
//            }
//            break;
//
//        case MEASURE_WTL: // Wind speed, Temp, Lvdt
//            switch (cmd) {
//
//                case __INIT:
//                case __START: // Demo signal
//                    break;
//                case __READ: // Vamp1K Aeolian Vibration [ ET,WS,{A,hT} ]
//                    break;
//                case __STOP: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
//                    break;
//            }
//            break;
//
//
//    }
//    return 0;
//}
//

