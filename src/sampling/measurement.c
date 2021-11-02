#include <xc.h>
#include <math.h>
#include <stdio.h>  // printf
#include <stdlib.h>  // abs
#include <string.h>

#include "measurement.h"
#include "acquire.h"

#include "../utils.h"
#include "../modules/RTCC.h"

#include "../memory/SST26VF064B.h"  // Flash SPI
#include "../memory/DEE/dee.h"

//==============================================================================
//extern sample_t SSBUF[];

//static __prog__ sample_t measurement[FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS] __attribute__((space(prog), aligned(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS)));

static sample_t SSBUF[SS_BUF_SIZE]; // measurement's samples buffer
extern measurement_t g_measurement;
extern device_t g_dev;

//static sample_t msCounter = 0; // Use: g_dev.st.measurmanet_counter

#ifdef __AV0NVM
static __prog__ uint8_t nvmDepot[DEPOT_SIZE] __attribute__((space(psv), aligned(DEPOT_SIZE)));
#endif

//==============================================================================

bool measurementInitialize(void) {
    return (true);
}

uint16_t measurementAcquire(measurement_t * ms) {
    sample_t *ptrSS = SSBUF;
    sample_t nsamples = 0;
    uint16_t fq_Hz;
    // FFTs
    short m;

#ifdef __VAMP1K_TEST
    printf("Acquiring t=%u, freq=%u\n", g_dev.cnf.general.typeset, (uint8_t) g_dev.cnf.calibration.av_period);
#endif

    fq_Hz = 2048; // g_dev.cnf.calibration.av_period;

    ms->ss = ptrSS;
    ms->dtime = RTCC_GetTimeL(); // get RTC datetime
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
    ms->ns = 4;
    // -----------------

    switch (g_dev.cnf.general.typeset) {

        case _SIG0: // Demo signal { <sig_freq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
            ms->tset = _SIG0;
            *ptrSS = g_dev.cnf.calibration.av_period; // Add <sig_freq> as single sample 
            *(ptrSS + 1) = g_dev.cnf.calibration.av_filter; // Add <sig_maxamp> as single sample 
            ptrSS += 2;
            nsamples = acquireSig(ptrSS, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - 2) / 2), \
                    g_dev.cnf.calibration.av_period, g_dev.cnf.calibration.av_filter, 1);
            ms->ns = 4; // Use first samples in buffer as singles {<adc_freq>,<res_scale>} 
            ms->nss = (nsamples - 1) * 2;
            break;

        case _AV00: // Aeolian Vibration, RAW
            ms->tset = _AV00;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_RAW(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), fq_Hz);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            ms->nss = nsamples;
            break;

        case _AV04: // Aeolian Vibration, RAW No DTime
            ms->tset = _AV04;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_RNT(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), fq_Hz);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;

        case _AV01: // Aeolian Vibration, Peak-Peak
            ms->tset = _AV01;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            // g_dev.cnf.general.cycletime=1;
            nsamples = acquireAV_P2P(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), fq_Hz, \
                                                                     g_dev.cnf.calibration.av_filter);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;

        case _AV02: // Aeolian Vibration, FFT 
            ms->tset = _AV02; // g_dev.cnf.general.typeset;
            //  m = 0;
            //  l2 = (SS_BUF_SIZE - ms->ns);
            //  while (l2 > 0) { // log2_npoints
            //    m++;
            //    l2 >>= 1;
            //  }
            m = 10;     // 2048 Point FFT -> 255 Coefficients
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_DFT(ptrSS, g_dev.cnf.general.cycletime, m, fq_Hz);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            ms->nss = (nsamples >> 1); // 512 Coefficients ( only positive - half spectrum )
            break;

        case _AV05: // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<freq>,<amp>),...]}
            m = 8;
            ms->tset = _AV05;
            ms->ns++; // Add <duration>
            *ptrSS = g_dev.cnf.general.cycletime * (((float) 1 / (fq_Hz << 1)) * (1 << m)); // total time 
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
            maxsamples = ((SS_BUF_SIZE - ms->ns) - wbSize);
            maxoss = maxsamples / 3; // Size of tuple in sample_t

            wbPtr = ptrSS + maxsamples; // begin of working buffer
            pPtr = (point_t*) wbPtr; // use working buffer as points

            for (i = 0; i < maxoss; i++) { // Initialize measurement result
                *(ptrSS + i) = 0x0;
            }

            oPtr = (oscill_t *) ptrSS; // use ptrSS as touples of samples
            noss = 0; // tuples counter

            g_dev.cnf.calibration.av_filter = 20; // ignore too close peaks 
            nsec = g_dev.cnf.general.cycletime; // compute on fq_Pr3 and wbSize

            do { // _____ Cycle total time....
                nsamples = acquireAV_P2P(wbPtr, 1, wbSize, fq_Hz, g_dev.cnf.calibration.av_filter);
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
            ms->nss = noss * 3U; //  Vibration occurrencies
            break;

        case _AV06: // (15) AVC-DFT { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<c>,<pw>),...]}
            m = 8;
            ms->tset = _AV06;
            ms->ns++; // Add <duration>
            *ptrSS = g_dev.cnf.general.cycletime * (((float) 1 / fq_Hz) * (1 << m)); // total time 
            ptrSS++;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            ms->nss = acquireAV_EVC_DFT(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), m, fq_Hz, g_dev.cnf.calibration.av_filter, 0);
            Device_SwitchSys(SYS_DEFAULT);
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

#define HEADER_SIZE_IN_BYTE     9U

// DEE map
#define EEA_CONFIG            000 // Config: 96 eeprom's byte to store config
#define EEA_MEAS_COUNTER      100 // Status: meas_counter  
#define EEA_RESET_COUNTER     101 // Status: reset counter / RCON map...
#define EEA_SKEYL             102 // Status: skey LSW
#define EEA_SKEYH             103 // Status: skey MSW
#define EEA_SST_SECTOR        126
#define EEA_SST_OFFSET        127
//
#define SST_SECTOR_SIZE      4096        // 4KBytes   
#define SST_SIZE_KB          7812        // 8MBytes 
#define SST_MAX_SECTOR       1953        // 8MBytes / 4Kbytes

uint16_t sstFreeSpaceKb() { // return 
    uint16_t Sector, Offset;
    DEE_Read(EEA_SST_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST_OFFSET, &Offset); // (dee.h)  
    return ( SST_SIZE_KB - (((Sector * SST_SECTOR_SIZE) + Offset) / 1024));
}

uint16_t sstBlockWrite(uint8_t* dPtr, uint16_t nBytes) { // return written bytes
    uint32_t sst_addr;
    uint16_t written, available;
    uint16_t Sector, Offset;

    DEE_Read(EEA_SST_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST_OFFSET, &Offset); // (dee.h)  

    written = 0;
    SST26_Enable();
    SST26_Global_Block_Protection_Unlock();
    SST26_Wait_Busy();
    SST26_WREN();

    while (nBytes > 0) {
        available = (SST_SECTOR_SIZE - Offset);
        if (available == 0) { // Format new sector
            Sector++;
            Offset = 0;
            available = SST_SECTOR_SIZE;
            SST26_Sector_Erase(Sector * SST_SECTOR_SIZE); // Set 4K in 0xFF state
            SST26_Wait_Busy();
        }
        sst_addr = (Sector * SST_SECTOR_SIZE) + Offset;

        if (nBytes < available) {
            available = nBytes;
        }
        written = SST26_Write_Bytes(sst_addr, (dPtr + written), available);
        SST26_Wait_Busy();
        nBytes -= written;
        Offset += available;
    }
    SST26_WRDI();
    SST26_Disable();

    DEE_Write(EEA_SST_SECTOR, Sector); // (dee.h)  
    DEE_Write(EEA_SST_OFFSET, Offset); // (dee.h)  
    return (written);
}

/* -------------------------------------------------------------------------- */
uint16_t sstBlockRelease(uint16_t nBytes) {
    uint16_t newSector, newOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST_OFFSET, &Offset); // (dee.h) 

    newSector = Sector - (nBytes / SST_SECTOR_SIZE);
    if ((nBytes % SST_SECTOR_SIZE) >= Offset) {
        newOffset = (Offset - (nBytes % SST_SECTOR_SIZE));
    } else {
        newSector--;
        newOffset = SST_SECTOR_SIZE - ((nBytes % SST_SECTOR_SIZE) - Offset);
    }

    // Move in newSector and save newOffset
    SST26_Enable();
    SST26_Global_Block_Protection_Unlock();
    SST26_Wait_Busy();

    // Move data to buffer !!! USES SSBUF !!!
    sst_addr = (newSector * SST_SECTOR_SIZE);
    SST26_Read_Bytes(sst_addr, newOffset, (uint8_t *) SSBUF);
    // Initialize Sector
    SST26_Sector_Erase(sst_addr); // Set 4K in 0xFF state
    SST26_Wait_Busy();
    // Move buffer to flash !!! USES SSBUF !!!
    SST26_WREN();
    SST26_Write_Bytes(sst_addr, (uint8_t *) SSBUF, newOffset);
    SST26_Wait_Busy();

    SST26_WRDI();
    SST26_Disable();

    DEE_Write(EEA_SST_SECTOR, newSector); // (dee.h)  
    DEE_Write(EEA_SST_OFFSET, newOffset); // (dee.h)  
    return (nBytes);
}

/* -------------------------------------------------------------------------- */
uint16_t sstBlockReadRev(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release) { // return readed bytes

    uint16_t iSector, iOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST_OFFSET, &Offset); // (dee.h) 

    iSector = Sector - ((nBytes + displacement) / SST_SECTOR_SIZE);
    if (((nBytes + displacement) % SST_SECTOR_SIZE) >= Offset) {
        iOffset = (Offset - ((nBytes + displacement) % SST_SECTOR_SIZE));
    } else {
        iSector--;
        iOffset = SST_SECTOR_SIZE - (((nBytes + displacement) % SST_SECTOR_SIZE) - Offset);
    }

    // Move in newSector and save newOffset
    SST26_Enable();
    SST26_Global_Block_Protection_Unlock();
    SST26_Wait_Busy();

    // Move data to buffer !!! USES SSBUF !!!
    sst_addr = (iSector * SST_SECTOR_SIZE) + iOffset;

    nBytes = SST26_Read_Bytes(sst_addr, nBytes, dPtr);
    return (nBytes);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementSave(measurement_t * ms) {
    if ((ms->ns + ms->nss) > 0) {

        DEE_Read(EEA_MEAS_COUNTER, &g_dev.st.meas_counter);
        sstBlockWrite((uint8_t*) g_measurement.ss, (g_measurement.ns + g_measurement.nss) << 1); // Save samples
        sstBlockWrite((uint8_t*) & g_measurement, HEADER_SIZE_IN_BYTE);
        g_dev.st.meas_counter++;
        DEE_Write(EEA_MEAS_COUNTER, g_dev.st.meas_counter);
        // msCounter++;
    };
    return (g_dev.st.meas_counter);
}

/* -------------------------------------------------------------------------- */
//#define measurementCounter() g_dev.st.meas_counter

/* -------------------------------------------------------------------------- */
uint16_t measurementLoad(uint16_t index, measurement_t * ms) {
    int16_t msIndex = 0;
    if (index <= g_dev.st.meas_counter) {

        sstBlockReadRev((uint8_t*) &g_measurement, 0, HEADER_SIZE_IN_BYTE, false);
        sstBlockReadRev((uint8_t*) g_measurement.ss, HEADER_SIZE_IN_BYTE, (g_measurement.ns + g_measurement.nss) < 1, false);
        msIndex = index;
        memcpy(ms, &g_measurement, sizeof (measurement_t));
    }
    return (msIndex);
}

/* -------------------------------------------------------------------------- */
uint16_t measurementDelete(uint16_t index) { // ret: 0/Counter
    int16_t msIndex = 0;
    if (g_dev.st.meas_counter > 0) {

        sstBlockRelease(HEADER_SIZE_IN_BYTE + ((g_measurement.ns + g_measurement.nss) < 1));

        g_dev.st.meas_counter--;
        DEE_Write(EEA_MEAS_COUNTER, g_dev.st.meas_counter);
        msIndex = g_dev.st.meas_counter;
    }
    return (msIndex);
}

/* -------------------------------------------------------------------------- */
void inline measurementGetBlock(uint8_t *pbuf, uint16_t offset, uint16_t size) {
    memcpy(pbuf, &g_measurement.ss[offset], size);
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




////////////////////uint8_t const _nvm24 = 3; // data-unit 
////////////////////
////////////////////uint8_t write_nvm24(uint16_t page, uint16_t address, uint8_t * towrite) {
////////////////////    //////    bool result = true;
////////////////////    //////    uint32_t nvm_address; // 24 bit address
////////////////////    //////    uint16_t iR, iF;
////////////////////    //////
////////////////////    //////    uint32_t towrite[2U];
////////////////////    //////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & measurement);
////////////////////    //////    FLASH_Unlock(FLASH_UNLOCK_KEY);
////////////////////    //////
////////////////////    //////    result = FLASH_ErasePage(nvm_address);
////////////////////    //////
////////////////////    //////    if (result) {
////////////////////    //////        iR = 0; // Bytes
////////////////////    //////        iF = iR; // Words
////////////////////    //////
////////////////////    //////
////////////////////    //////        //        while (iR < ramConfigSize) { //  Sizeof(config_t)
////////////////////    //////        //            towrite[0] = *(pSrc + iR + 2);
////////////////////    //////        //            towrite[0] <<= 8;
////////////////////    //////        //            towrite[0] |= *(pSrc + iR + 1);
////////////////////    //////        //            towrite[0] <<= 8;
////////////////////    //////        //            towrite[0] |= *(pSrc + iR);
////////////////////    //////        //            towrite[1] = *(pSrc + iR + 5);
////////////////////    //////        //            towrite[1] <<= 8;
////////////////////    //////        //            towrite[1] |= *(pSrc + iR + 4);
////////////////////    //////        //            towrite[1] <<= 8;
////////////////////    //////        //            towrite[1] |= *(pSrc + iR + 3);
////////////////////    //////        //            iR += 6;
////////////////////    //////        //
////////////////////    //////        //            //            printf("%u %u \n", (uint16_t) (w0>>16)& 0xFFFF, (uint16_t)w0 & 0xFFFF);
////////////////////    //////        //            //            printf("%u %u \n", (uint16_t) (w1>>16)& 0xFFFF , (uint16_t)w1 & 0xFFFF);         
////////////////////    //////        //            result &= FLASH_WriteDoubleWord24(nvm_address + iF, towrite[0], towrite[1]);
////////////////////    //////        //            iF += 4U;
////////////////////    //////        //        }
////////////////////    //////
////////////////////    //////    }
////////////////////    //////    FLASH_Lock();
////////////////////    //////    //_LATB2 = 1;
////////////////////}
////////////////////
////////////////////uint16_t read_nvm24(uint16_t page, uint16_t address, uint16_t nbytes, uint8_t * rdata) {
////////////////////
////////////////////    //////////#ifdef __NOFLASH
////////////////////    //////////    Device_ConfigDefaultSet(config);
////////////////////    //////////#else    // Read flash/eeprom 
////////////////////    //////////    uint32_t rdata;
////////////////////    //////////    uint32_t nvm_address;
////////////////////    //////////    uint8_t * pDst;
////////////////////    //////////    uint16_t iR, i, iF;
////////////////////    //////////
////////////////////    //////////    pDst = (uint8_t*) config;
////////////////////    //////////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & nvmConfigDatas);
////////////////////    //////////
////////////////////    //////////    iR = 0;
////////////////////    //////////    iF = iR;
////////////////////    //////////    while (iR < ramConfigSize) { // && iF << flash page
////////////////////    //////////        rdata = FLASH_ReadWord24(nvm_address + iF);
////////////////////    //////////        //printf("%u %u \n", (uint16_t) (rdata>>16) & 0xFFFF , (uint16_t) rdata & 0xFFFF);
////////////////////    //////////        i = 0;
////////////////////    //////////        while ((i < 3) && (iR + i < ramConfigSize)) {
////////////////////    //////////            *(pDst + iR + i) = (rdata & 0xFF);
////////////////////    //////////            rdata >>= 8;
////////////////////    //////////            i++;
////////////////////    //////////        }
////////////////////    //////////        iR += i;
////////////////////    //////////        iF += 2U;
////////////////////    //////////    }
////////////////////    //////////
////////////////////    //////////    if (config->CRC16 != computeCRC16((uint8_t *) config, sizeof (config_t) - 2)) { /// Check XOR checksum
////////////////////    //////////        Device_ConfigDefaultSet(config);
////////////////////    //////////        config->CRC16 = computeCRC16((uint8_t *) config, sizeof (config_t) - 2);
////////////////////    //////////        Device_ConfigWrite((uint8_t*) config); // Write EEprom/Flash
////////////////////    //////////    }
////////////////////    //////////#endif
////////////////////    //////////
////////////////////    //////////#ifdef __VAMP1K_TEST_CONFIG
////////////////////    //////////    printf("Read:\n");
////////////////////    //////////    printConfig();
////////////////////    //////////    _LATB2 = 1; // Led on
////////////////////    //////////#endif
////////////////////    //////////    
////////////////////    //////////    return result;
////////////////////
////////////////////}


//#define DEPOT_ADDR 0x0000
//#define DEPOT_SIZE  0x000

/*

 Device             Program Memory          Write Blocks(1)     Erase Blocks(1)
                    Upper Boundary
                    (Instruction Words)
-------------------------------------------------------------------------------
PIC24FJ256GA70X     02AFFEh (88,064 x 24)       688                 86

- One Write Block = 128 Instruction Words;
- One Erase Block (Page) = 1024 Instruction Words.
  
 +-----------------------------+ 
 | box                         |
 +------+---------------+------+
 | head |    databox    | next |
 +------+---------------+------+
  
 +-----------------------+
 | head             |
 +-----------------------+ 
 |b31,b30|b29-b24|b23,b0 |  
 +-------+-------+-------+
 | state | ....  | size  |
 +-------+-------+-------+
  
  
 +-----------------------+
 | next             |
 +-----------------------+ 
 |b31,b30|b29-b24|b23,b0 |  
 +-------+-------+-------+
 | state | bank  | addr  |
 +-------+-------+-------+
  
 
    BTY ( Bank Type - Flash/Eeprom  )
    AUS ( Allocation Unit Size - bytes ) 
   
    [HDDDDDNHDDDDDNHddddddddNHDDDDDDN.....]
     ^      ^      ^         ^       
     |      |      |         |
    Hp      H1     H2        H3                
  

 */
