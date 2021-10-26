/*
 * File:   measurement.h
 * Author: plogiacco
 * Revision: merge ws lvdt proj dongle
 *
 * Created on October 25, 2020, 5:40 PM
 */
#ifndef MEASUREMENT_H
#define	MEASUREMENT_H

#include <xc.h>                     // processor files
#include <stdbool.h>
#include "../device.h"              // Typeset defs / config params
//#include "measurement.h"          // Typeset defs / config params

#define sample_t signed short       // fix14_t 

//typedef unsigned short sample_t;  // ADC FormatAbsolute Integer Format
#define FLOAT2INT_FACTOR 100000U
#define SYNCO_FREQUENCY  38400U 
#define SCALE_TOUNSIGNED 1024U      // 12 bit 4096

//------------------------------------------------------------------------------
#if ( defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__)) // Flash 32K, SRam 2K, EEprom 512

#define SS_BUF_SIZE 512             // SRam
#define SS_BUF_SIZE_NVM 1536        // Flash

#elif defined(__PIC24FJ256GA702__)

#define SS_BUF_SIZE 4096           // SRam
//#define SS_BUF_SIZE_NVM 1536      // Flash

#endif
//------------------------------------------------------------------------------

typedef enum e_dimension { // physical phenomenon
    DIM_TR = 0x1, // Time
    DIM_ET = 0x1, // Temperature
    DIM_AV = 0x3, // Vibration (eolian)
    DIM_WS = 0x5, // Wind Speed
    DIM_DD = 0xF // Demo
} dimension_t;

typedef enum e_typeset {
    _SIG0 = 0x02, // (02) Test signal  { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }
    _AV04 = 0x04, // (04) Aeolean Vibration, RAW without dT ! { <ET>,<WS>,<adc_fq> <res_scale>,<s1>,...,<sn>}            
    _AV00 = 0x0A, // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
    _SS00 = 0x0B, // (11) Sub-Span, raw
    _AV02 = 0x0C, // (12) Aeolian Vibration, FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV01 = 0x0D, // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
    _AV03 = 0x0E, // (14) FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV05 = 0x0F //  (15) AVC { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<freq>,<amp>),...]}
} typeset_t;

typedef struct {
    uint32_t dtime; // Timestamp
    typeset_t typeset; // Typeset
    uint16_t ns; // number of single-sample
    uint16_t nss; // number of sequenced-samples
    sample_t* ss; // samples
} measurement_t;

typedef enum e_measure {
    MEASURE_WT = 0x01, // Wind Speed, Enviroment Temperature
    MEASURE_WTL = 0x02 // Wind, Temperature, LVDT
} measure_t;

typedef enum {
    __INIT,
    __START,
    __READ,
    __STOP
} measureCmd_t;


// Typeset Measurements Managing
// -------------------------------------------------------------------------
/*
    MEASURAMENT
 
    -----------+-----+--------------------------------------+---------------
    Field       Size  Description                               
    -----------+-----+--------------------------------------+---------------
    typeset     2     Dimensions and specifications
    timestamp   4     Acquiring absolute date and time     
    ns          2     Number of Single Samples
    nss         2     Number of Sequenced Samples
    ss          2     Pointer to Samples Buffer/Blocks* 
    -----------+-----+--------------------------------------+---------------
    
    typeset: _SIG0
    
    typeset: _AV00
 * ws, temp, period, scala

    //  sample: 16 bits unsigned
    //  Single: { Wind Speed } , { Env.Temperature }
    //  Sequenced: { Amplitude, Half-Period }
    //  ADC_Format: 12Bit(4096) Positive
    //  ADC_Freq: 1.2Khz
    //  Process: P-P Filter
        Optimize to process data's blocks !!!!!!!!
 */
// 
uint16_t measurementAcquire(measurement_t *ms); // ret: nSamples
//
uint16_t measurementSave(measurement_t *ms); // ret: 0/Counter
uint16_t measurementCounter(); // ret: number of stored measurements
uint16_t measurementLoad(uint16_t index, measurement_t *ms); // ret: -1 error or Index ( ONLY LAST ??? FIFO ???)
uint16_t measurementDelete(uint16_t index); // ret: -1 error or Index

void measurementGetBlock(uint8_t *pbuf, uint16_t offset, uint16_t size); // Exchange needs !!!

uint16_t getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp);

// Dimensions Acquiring Routines
// -------------------------------------------------------------------------

#define DEPOT_BLOCK_SIZE FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS * 4 
#define DEPOT_BLOCKS  32  // ONE PAGE 32 ROWS (1024 INSTRUCTIONS * 3 Bytes phantom+data )
#define DEPOT_SIZE 0x1000 // 4096 - phantom bytes


#define  to24bit(t,v) ((((uint32_t)t)<<12) | (v && 0xFFF))
//

// Real Time Sampling
// -------------------------------------------------------------------------


#endif	/* MEASUREMENT_H */


