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

//==============================================================================
extern device_t g_dev;
extern sample_t SSBUF[SS_BUF_SIZE]; // measurement's samples buffer
extern measurement_t g_measurement;

static uint16_t m_counter = 0;
// extern measurement_t tmpMeasurement; // Debug 


//==============================================================================

void measurementInitialize()
{

    g_measurement.ss = SSBUF;
    m_counter = 0; // g_dev.st.meas_counter;
    // DEE_Read(EEA_MEAS_COUNTER, &m_counter);
}

uint16_t measurementCounterGet()
{
    return m_counter; // number of available measures
}

uint16_t measurementAcquire() // ( tset, duration, adc_fq, filter )
{
    sample_t *ptrSS;
    uint16_t nsamples = 0;
    uint8_t m; // FFTs

    typeset_t typeset = g_dev.cnf.general.typeset;
    uint16_t duration = g_dev.cnf.general.cycletime;
    uint16_t fq_Hz = g_dev.cnf.calibration.av_period;
    uint16_t filter = g_dev.cnf.calibration.av_filter;

#ifdef __VAMP1K_TEST
    printf("Acquiring t=%u, freq=%u\n", typeset, (uint8_t) fq_Hz);
#endif

    g_measurement.tset = typeset;
    g_measurement.dtime = RTCC_GetTimeL(); // get RTC datetime    

    g_measurement.ss = SSBUF;
    ptrSS = SSBUF;

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
        g_measurement.ns = 4;
        // -----------------
    }
    else {

        //        fq_Hz = 100; // Uses first as <sig_freq> 
        //        filter = 1000; // Uses second as <sig_maxamp> 

        *ptrSS = fq_Hz; // Uses first as <sig_freq> 
        ptrSS++;
        *ptrSS = filter; // Uses second as <sig_maxamp> 
        ptrSS++;
        g_measurement.ns = 2;
    }

    switch (typeset) {

    case _SIG0: // (02) Test Signal { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
        //tmpM.tset = _SIG0;
        g_measurement.nss = acquireSig(ptrSS, duration, (SS_BUF_SIZE - g_measurement.ns), fq_Hz, filter, true);
        g_measurement.nss -= 2; // Uses 2 samples in buffer as singles:
        g_measurement.ns += 2; // {<adc_freq>, <amp_scale>} 
        nsamples = g_measurement.nss + g_measurement.ns;
        break;

    case _AV00: // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
        //tmpM.tset = _AV00;
        Device_SwitchSys(SYS_ON_SAMP_ADA);
        g_measurement.nss = acquireAV_RAW(ptrSS, duration, (SS_BUF_SIZE - g_measurement.ns), fq_Hz);
        Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
        nsamples = g_measurement.nss + g_measurement.ns;
        break;

    case _AV01: // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
        //tmpM.tset = _AV01;
        Device_SwitchSys(SYS_ON_SAMP_ADA);
        g_measurement.nss = acquireAV_P2P(ptrSS, duration, (SS_BUF_SIZE - g_measurement.ns), fq_Hz, filter);
        Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
        nsamples = g_measurement.nss + g_measurement.ns;
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
        g_measurement.nss = (nsamples >> 1); // 512 Coefficients ( only positive - half spectrum )
        nsamples = g_measurement.nss + g_measurement.ns;
        break;



    case _AV04: // Aeolian Vibration, RAW No DTime
        //tmpM.tset = _AV04;
        Device_SwitchSys(SYS_ON_SAMP_ADA);
        g_measurement.nss = acquireAV_RNT(ptrSS, duration, (SS_BUF_SIZE - g_measurement.ns), fq_Hz);
        Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
        nsamples = g_measurement.nss + g_measurement.ns;
        break;

    case _AV05: // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<freq>,<amp>),...]}
        m = 8;
        g_measurement.tset = _AV05;
        g_measurement.ns++; // Add <duration>
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
        maxsamples = ((SS_BUF_SIZE - g_measurement.ns) - wbSize);
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
                }
                else { // Update counter
                    oPtr[j].n++;
                }
            }
            nsec--;
        }
        while (nsec > 0);
        // _________
        Device_SwitchSys(SYS_DEFAULT);
        g_measurement.nss = noss * 3U; //  Vibration occurrencies
        nsamples = g_measurement.nss + g_measurement.ns;
        break;

    case _AV06: // (15) AVC-DFT { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<c>,<pw>),...]}
        m = 8;
        //tmpM.tset = _AV06;
        g_measurement.ns++; // Add <duration>
        *ptrSS = duration * (((float) 1 / fq_Hz) * (1 << m)); // total time 
        ptrSS++;
        Device_SwitchSys(SYS_ON_SAMP_ADA);
        g_measurement.nss = acquireAV_EVC_DFT(ptrSS, duration, (SS_BUF_SIZE - g_measurement.ns), m, fq_Hz, filter, 0);
        Device_SwitchSys(SYS_DEFAULT);
        nsamples = g_measurement.nss + g_measurement.ns;
        break;

    case _SS00: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
        break;


    };
    return (nsamples);
}
/* -------------------------------------------------------------------------- */



/******************************************************************************
 
    Measurement storing block
    +-----------+----------+----------+----------+----------+
    | N bytes   | 4 byte   | 1 byte   | 2 byte   | 2 byte   |
    +-----------+----------+----------+----------+----------+
    | Samples.. | dtime    | typeset  | ns       | nss      |
    +-----------+----------+----------+----------+----------+

    measurement_size: header (4+1+2+2) + samples ((ns+nss)*2)
    ptr2Last = +/- measurement_size
 
 ******************************************************************************/

#define HEADER_SIZE_IN_BYTE        10U   // Measurement's header
//
#define SST26_SIZE_KB             8192   // SST26VF064 capacity is 8,388,608 bytes 
#define SST26_SECTOR_SIZE         4096   // 4Kbyte 0..4095
#define SST26_MAX_SECTOR          2048   // 8Mbyte / 4Kbyte
#define SST26_PAGE_SIZE            256   // Page size in byte
#define SST26_MAX_PAGEX             16   // Pages per Sector

void sstInitialize()
{
    uint16_t Sector = 10;
    // Initialize Persistent Status Datas
    DEE_Write(EEA_MEAS_COUNTER, 0); // (dee.h)  
    DEE_Write(EEA_SST26_SECTOR, Sector); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, 0); // (dee.h)  
    // Initialize flash ( Erase chip ) )
    SST26_Enable();
    //    SST26_WREN();
    SST26_Global_Protection_Unlock();
    SST26_Erase_Sector(Sector * SST26_SECTOR_SIZE); // Set 4K in 0xFF state
    SST26_Disable();
}

uint16_t sstFreeSpaceKb()
{ // return 
    uint16_t Sector, Offset;
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  
    return ( SST26_SIZE_KB - (((Sector * SST26_SECTOR_SIZE) + Offset) / 1024));
}

uint16_t sstBlockWrite(uint8_t* dPtr, uint16_t nBytes)
{ // return written bytes
    uint32_t sst_addr;
    uint16_t written, available;
    uint16_t Sector, Offset;

    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  

    written = 0;
    SST26_Enable();
    SST26_Global_Protection_Unlock();

    while (nBytes > 0) {
        available = (SST26_SECTOR_SIZE - Offset);
        if (available == 0) { // Format new sector
            Sector++;
            Offset = 0;
            available = SST26_SECTOR_SIZE;
            SST26_Erase_Sector(Sector * SST26_SECTOR_SIZE); // Set 4K in 0xFF state
        }
        //sst_addr = ((Sector << 16 ) & (Offset<<8));
        sst_addr = (Sector * SST26_SECTOR_SIZE) + Offset;
        if (nBytes <= available) {
            written = SST26_Write(sst_addr, (dPtr + written), nBytes);
        }
        else {
            written = SST26_Write(sst_addr, (dPtr + written), available);
        }
        nBytes -= written;
        Offset += written;
    }
    //    SST26_WRDI();
    SST26_Disable();

    DEE_Write(EEA_SST26_SECTOR, Sector); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, Offset); // (dee.h)  
    return (written);
}

/* -------------------------------------------------------------------------- */
uint16_t sstBlockRelease(uint16_t nBytes)
{
    uint16_t newSector, newOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    //    newSector = Sector - (nBytes / SST_SECTOR_SIZE);
    //    if ((nBytes % SST_SECTOR_SIZE) >= Offset) {
    //        newOffset = (Offset - (nBytes % SST_SECTOR_SIZE));
    //    }
    //    else {
    //        newSector--;
    //        newOffset = SST_SECTOR_SIZE - ((nBytes % SST_SECTOR_SIZE) - Offset);
    //    }

    if (nBytes <= Offset) {
        newOffset = (Offset - nBytes);
        newSector = Sector;
    }
    else {
        newSector = Sector - (nBytes / SST26_SECTOR_SIZE);
        newOffset = Offset - (nBytes % SST26_SECTOR_SIZE);
    }

    // Move in newSector and save newOffset
    SST26_Enable();
    //    SST26_WREN();
    SST26_Global_Protection_Unlock();
    //    SST26_Wait_Busy();

    // Move data to buffer !!! USES SSBUF !!!
    sst_addr = (newSector * SST26_SECTOR_SIZE);
    SST26_Read(sst_addr, newOffset, (uint8_t *) SSBUF);
    // Initialize Sector
    SST26_Erase_Sector(sst_addr); // Set 4K in 0xFF state

    // Move buffer to flash !!! USES SSBUF !!!
    SST26_Write(sst_addr, (uint8_t *) SSBUF, newOffset);
    SST26_Disable();

    DEE_Write(EEA_SST26_SECTOR, newSector); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, newOffset); // (dee.h)  
    return (nBytes);
}

/* -------------------------------------------------------------------------- */
uint16_t sstBlockReadRev(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release)
{ // return readed bytes

    uint16_t iSector, iOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    iSector = Sector - ((nBytes + displacement) / SST26_SECTOR_SIZE);
    if (((nBytes + displacement) % SST26_SECTOR_SIZE) >= Offset) {
        iOffset = (Offset - ((nBytes + displacement) % SST26_SECTOR_SIZE));
    }
    else {

        iSector--;
        iOffset = (SST26_SECTOR_SIZE - (((nBytes + displacement) % SST26_SECTOR_SIZE) - Offset));
    }


    SST26_Enable();

    // sst_addr = ((iSector << 16 ) & (iOffset<<8));
    sst_addr = (iSector * SST26_SECTOR_SIZE) + iOffset;

    nBytes = SST26_Read(sst_addr, nBytes, dPtr);
    return (nBytes);
}

/* -------------------------------------------------------------------------- */


uint16_t measurementSave() // Uses g_measurement
{
    if ((g_measurement.ns + g_measurement.nss) > 0) {
        // DEE_Read(EEA_MEAS_COUNTER, &m_counter);
        //         sstBlockWrite((uint8_t*) g_measurement.ss, ((g_measurement.ns + g_measurement.nss) << 1)); // Save samples
        //         sstBlockWrite((uint8_t*) & g_measurement, HEADER_SIZE_IN_BYTE);
        m_counter++;
        DEE_Write(EEA_MEAS_COUNTER, m_counter);
    };
    return (m_counter);
}

/* -------------------------------------------------------------------------- */
//#define measurementCounter() g_dev.st.meas_counter

/* -------------------------------------------------------------------------- */
uint16_t measurementLoad(uint16_t index, measurement_t *ms)
{
    if (m_counter > 0) {
        //measurementInitialize(ms);
        //      sstBlockReadRev((uint8_t*) & g_measurement, 0, HEADER_SIZE_IN_BYTE, false); // ok
        //      sstBlockReadRev((uint8_t*) g_measurement.ss, HEADER_SIZE_IN_BYTE, ((g_measurement.ns + g_measurement.nss) << 1), false);
        memcpy(ms, &g_measurement, sizeof (measurement_t));
    }
    return (m_counter);
}

uint16_t measurementLoadSamples(uint8_t *pbuf, uint8_t offset, uint8_t size)
{
    memcpy(pbuf, (uint8_t *) (g_measurement.ss + offset) , size);
    return (size);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementDelete(uint16_t index)
{
    if (m_counter > 0) {
        //        sstBlockReadRev((uint8_t*) & g_measurement, 0, HEADER_SIZE_IN_BYTE, false); // ok
        //        sstBlockRelease(HEADER_SIZE_IN_BYTE + ((g_measurement.ns + g_measurement.nss) << 1));
        m_counter--;
        // DEE_Write(EEA_MEAS_COUNTER, g_dev.st.meas_counter);
    }
    return (m_counter);
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

