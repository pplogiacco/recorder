/*
 * File:   measurement.h
 * Author: plogiacco
 * Revision: merge ws lvdt proj dongle
 *
 * Created on October 25, 2020, 5:40 PM
 */
#ifndef MEASUREMENT_H
#define	MEASUREMENT_H

#include <xc.h>         
#include <stdbool.h>
#include "../device.h"            // defs / config params
#include "../memory/libdpt.h"

//typedef unsigned short sample_t;  // ADC FormatAbsolute Integer Format
#define sample_t signed short     // 16Bit
#define SAMPLE_SIZE_IN_BYTE     2


//------------------------------------------------------------------------------
#if ( defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__)) // Flash 32K, SRam 2K, EEprom 512
#define SS_BUF_SIZE 512             
#elif defined(__PIC24FJ256GA702__)
//#define SS_BUF_SIZE     SST26_SECTOR_SIZE_IN_BYTE * SAMPLE_SIZE_IN_BYTE
#define SS_BUF_SIZE 4096 
#endif
#if ( SS_BUF_SIZE < (SST26_SECTOR_SIZE_IN_BYTE / SAMPLE_SIZE_IN_BYTE))
#error SS_BUF_SIZE is too small. Must be >= SST26_SECTOR_SIZE_IN_BYTE*2 ( depotDrop buffering ) !!!
#endif

//------------------------------------------------------------------------------

typedef enum e_typeset {
    _SIG0 = 0x02, // (02) Test signal  { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }           
    _AV00 = 0x0A, // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
    _AV01 = 0x0D, // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
    _AV02 = 0x0C, // (12) Aeolian Vibration, FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV03 = 0x0E, // (14) FFT Complex { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV04 = 0x04, // (04) Aeolian Vibration, RAW without dT ! { <ET>,<WS>,<adc_fq> <res_scale>,<s1>,...,<sn>} 
    _AV05 = 0x0F, // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<freq>,<amp>),...]}
    _AV06 = 0x08, // (08) AVC-DFT { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<nc>,<pw>),...]}
    _SS00 = 0x0B // (11) Sub-Span, raw         
} typeset_t;
//------------------------------------------------------------------------------

typedef struct {
    uint32_t dtime; // Timestamp
    typeset_t tset; // Typeset
    uint16_t ns; // number of single-sample
    uint16_t nss; // number of sequenced-samples
    sample_t* ss; // samples
} measurement_t;


// Typeset Measurements Managing
// -------------------------------------------------------------------------
/*
    MEASURAMENT
 
    -----------+-----+--------------------------------------+---------------
    Field       Size  Description                               
    -----------+-----+--------------------------------------+---------------
    typeset     2     Data specifications
    timestamp   4     Absolute date and time     
    ns          2     Number of Single Samples
    nss         2     Number of Sequenced Samples
    ss          2     Pointer to Samples Buffer ( unsigned word )
    -----------+-----+--------------------------------------+---------------    
 */

//void measurementInitialize();
//
uint16_t measurementAcquire(); // ret: nsamples
//
uint16_t measurementSave(); // ret: measurementCounter()

uint16_t measurementLoad(uint16_t index);

uint16_t measurementDelete(uint16_t index); // ret: measurementCounter()
//
//measurement_t * measurementGetPTR();
//uint16_t measurementCounterGet();

#define measurementCounterGet() device.sts.meas_counter

//uint16_t getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp);

// -------------------------------------------------------------------------
void depotDefaultSet();

uint16_t depotFreeSpaceKb();
// -------------------------------------------------------------------------


#endif	/* MEASUREMENT_H */


