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
    sample_t nsamples = 0;
    uint16_t adc_fq = 0;
    uint16_t herz_fq;
    // FFTs
    short m;
    short l2;

#ifdef __VAMP1K_TEST
    printf("Acquiring t=%u\n", g_dev.cnf.general.typeset);
#endif


    switch (g_dev.cnf.calibration.av_period) { // Set PR3 as FCY divider 
        case 3: // ADC_FRQ_24Khz
            herz_fq = 2400; // Sampling freq. (Hz)
            adc_fq = ADC_FRQ_24Khz; //  PR3=3200U
            break;
        case 4: // ADC_FRQ_1Khz
            herz_fq = 1000; // Sampling freq. (Hz))
            adc_fq = ADC_FRQ_1Khz; //  PR3=8000U
            break;

        default: // ADC_FRQ_05Khz
            herz_fq = 500; // Sampling freq. (Hz))
            adc_fq = ADC_FRQ_05Khz; //  PR3=8000U
            break;
    }

    ms->ss = ptrSS;
    ms->dtime = RTCC_GetTimeL(); // get RTC datetime
    // ----------------- Add Single Samples
    Device_SwitchSys(SYS_ON_SAMP_WST);
    acquireET(ptrSS); // RET: success
    ptrSS++;
    acquireWS(ptrSS); // RET: success
    Device_SwitchSys(SYS_DEFAULT);
    ptrSS++;
    *ptrSS = herz_fq; // Sampling Frequency (Hz)
    ptrSS++;
    *ptrSS = 4200; // Sampling Resolution 2^12 (12 Bit ADC)
    ptrSS++;
    ms->ns = 4;
    // -----------------

    switch (g_dev.cnf.general.typeset) {

        case _SIG0: // Demo signal { <sig_freq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
            ms->typeset = _SIG0;
            *ptrSS = g_dev.cnf.calibration.av_period; // Add <sig_freq> as single sample 
            *(ptrSS + 1) = g_dev.cnf.calibration.av_filter; // Add <sig_maxamp> as single sample 
            ptrSS += 2;
            nsamples = acquireSig(ptrSS, g_dev.cnf.general.cycletime, ((SS_BUF_SIZE - 2) / 2), \
                    g_dev.cnf.calibration.av_period, g_dev.cnf.calibration.av_filter);
            ms->ns = 4; // Use first samples in buffer as singles {<adc_freq>,<res_scale>} 
            ms->nss = (nsamples - 1) * 2;

            break;

        case _AV00: // Aeolian Vibration, RAW
            ms->typeset = _AV00;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, 0);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            ms->nss = nsamples;

            break;


        case _AV01: // Aeolian Vibration, Peak-Peak
            ms->typeset = _AV01;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, \
                    (g_dev.cnf.calibration.av_filter < 1) ? 1 : g_dev.cnf.calibration.av_filter);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;

        case _AV04: // Aeolian Vibration, RAW No DTime
            ms->typeset = _AV04;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_RAW(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), adc_fq, \
                                                                     g_dev.cnf.calibration.av_filter);
            ms->nss = nsamples;
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            break;


        case _AV02: // Aeolian Vibration, FFT 
            ms->typeset = _AV02; // g_dev.cnf.general.typeset;

            m = 0;
            l2 = (SS_BUF_SIZE - ms->ns);

            while (l2 > 0) { // log2_npoints
                m++;
                l2 >>= 1;
            }
            m = 10;
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_FFT(ptrSS, g_dev.cnf.general.cycletime, m, adc_fq);
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);
            ms->nss = (nsamples >> 1); // 512 Coefficients ( only positive - half spectrum )
            break;

        case _AV05: // (15) AVC { <ET>,<WS>,<adc_fq>,<adc_res>,<duration>,[ (<n>,<freq>,<amp>),...]}

            ms->typeset = _AV05; // g_dev.cnf.general.typeset;

            m = 0;
            l2 = (SS_BUF_SIZE - ms->ns);

            while (l2 > 0) { // log2_npoints
                m++;
                l2 >>= 1;
            }
            m = 8;
            ms->ns++; // Add <duration>
            *ptrSS = g_dev.cnf.general.cycletime * ((1 / herz_fq) * pow(2, m));
            ptrSS++;

            // Force ADC frequency 0.5Khz ( g_dev.cnf.calibration.av_period )
            //uint16_t acquireAV_EVC(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t log2_npoints,
            //                        uint16_t adc_fq, uint16_t fft_min_pw, uint16_t fft_avg_co)
            Device_SwitchSys(SYS_ON_SAMP_ADA);
            nsamples = acquireAV_EVC(ptrSS, g_dev.cnf.general.cycletime, (SS_BUF_SIZE - ms->ns), m, \
                                 adc_fq, g_dev.cnf.calibration.av_filter, 0);
            //ms->ns = 4; // <temperature>,<windspeed>,<tick_period>,<scale_offset>,[{<period>,<amplitude>},...]
            Device_SwitchSys(SYS_DEFAULT); // Device_SwitchPower(lastPwrState);

            ms->nss = nsamples; //  Vibration occurrencies

            break;

        case _SS00: // Vamp1K encoder Sub-span oscillation: // Raw sample signal
            break;


    };
    return (nsamples);
}

/* -------------------------------------------------------------------------- */

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


uint16_t measurementSave(measurement_t * ms) {
    if ((ms->ns + ms->nss) > 0) {
        msCounter++;
    };

    // trova spazio libero 
    // salva misura 
    // aggiorna lookup table 


    //    depot_nextfree(DEPOT_ADDR);

    // depotAddBegin(uint16_t id, uint16_t size);
    // depotAdd(char* data, uint16_t len);
    // index = depotAddEnd();

    // depotDelete(uint16_t id);
    // uint16_t = depotFreeSpace();  // blocks
    // uint16_t = depotSize();

    // +-------------------------------
    // | id | Offset        
    // +------------------------------
    // |  0 | pointer to first free block  
    // |  1 | pointer to first block of chain 1                
    // | ...| ....
    //
    // block: {datablock, MARKER } , MARKER:NXB/EOF
    //
    // Datablock:
    // pgsize = 512          // min 4K block of flash memory - erase cycle
    // BLOCK_MAXSAMPLES 48   // tranfer 96 Byte each trasmit buffer ( buffer size 128 )
    // 

    /*
        if ((ms->ns + ms->nss) > 0) { // Save measurement
            msCounter++;
        
            totSamplesBlocks = (uint16_t) ((uint16_t) (ms.nss + ms.ns) / BLOCK_MAXSAMPLES);
            spareSamples = (uint16_t) ((uint16_t) (ms.nss + ms.ns) % BLOCK_MAXSAMPLES);
            if (spareSamples > 0) {
                totSamplesBlocks++;
            }

            offset = 0;
            memcpy(&buffer[offset], &totSamplesBlocks, 2);
            offset += 2;
            memcpy(&buffer[offset], &ms.dtime, 4);
            offset += 4;
            buffer[offset++] = ms.typeset;
            memcpy(&buffer[offset], &ms.ns, 2);
            offset += 2;
            memcpy(&buffer[offset], &ms.nss, 2);
            offset += 2;

        
            if (protocolSend(CMD_MEASUREMENT_HEADER, offset, __ACK_TIMEOUT_DEFAULT)) {
                result = true;
                for (x = 0; x < totSamplesBlocks; x++) {
                    blockSize = BLOCK_MAXSAMPLES;
                    if ((spareSamples > 0) && (x == (totSamplesBlocks - 1))) {
                        blockSize = spareSamples;
                    }
                    offset = 0;
                    buffer[offset++] = (uint8_t) (x + 1);

                    //memcpy(&buffer[offset], &ms.ss[x * BLOCK_MAXSAMPLES], (blockSize * 2));
                    measurementGetBlock(&buffer[offset], x * BLOCK_MAXSAMPLES, blockSize * 2); // blocksize multiplo di 3 in bytes 

                    offset += (blockSize * 2);
  
                    // ****************** SAVE IN FLASH 
                    // Add NXB/EOF
                    if (!protocolSend(CMD_MEASUREMENT_BLOCK, offset, __ACK_TIMEOUT_DEFAULT)) {
                        result = false;
                        break;
                    }
                    // ******************
                }
            }
        };
     */




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

/* -------------------------------------------------------------------------- */
void measurementGetBlock(uint8_t *pbuf, uint16_t offset, uint16_t size) {
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
