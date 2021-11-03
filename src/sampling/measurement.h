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

#define sample_t signed short       // fix14_t 
//typedef unsigned short sample_t;  // ADC FormatAbsolute Integer Format

#define FLOAT2INT_FACTOR 100000U
#define SYNCO_FREQUENCY  38400U 
#define SCALE_TOUNSIGNED 1024U      // 12 bit 4096

//------------------------------------------------------------------------------
#if ( defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__)) // Flash 32K, SRam 2K, EEprom 512

#define SS_BUF_SIZE 512             // SRam

#elif defined(__PIC24FJ256GA702__)

#define SS_BUF_SIZE 5120        // SRam

////#define SS_BUF_SIZE_NVM 1536      // Flash
//
//
//#define NVM_INSTRUCTION_IN_BYTE     3      // 1Phantom+3
//#define NVM_INSTRUCTION_IN_ROW      128    // Instruction
//#define NVM_ROW_IN_PAGE             1024   // Rows
//
//#define PAGE_SPACE_IN_BYTE          NVM_INSTRUCTION_IN_BYTE*NVM_INSTRUCTION_IN_ROW*NVM_ROW_IN_PAGE
//
//#define OBJECT_SIZE_IN_BYTE         SS_BUF_SIZE*2
//#define OBJECT_PER_PAGE             PAGE_SPACE_IN_BYTE / OBJECT_SIZE_IN_BYTE
//
//#if OBJECT_PER_PAGE == 0
//#error !!!!!! Minimum data object per page is 1
//#endif



#endif
//------------------------------------------------------------------------------

//typedef enum e_dimension { // physical phenomenon
//    DIM_TR = 0x1, // Time
//    DIM_ET = 0x1, // Temperature
//    DIM_AV = 0x3, // Vibration (eolian)
//    DIM_WS = 0x5, // Wind Speed
//    DIM_DD = 0xF  // Demo
//} dimension_t;

typedef enum e_typeset {
    _SIG0 = 0x02, // (02) Test signal  { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }           
    _AV00 = 0x0A, // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
    _AV01 = 0x0D, // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
    _AV02 = 0x0C, // (12) Aeolian Vibration, FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV03 = 0x0E, // (14) FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
    _AV04 = 0x04, // (04) Aeolian Vibration, RAW without dT ! { <ET>,<WS>,<adc_fq> <res_scale>,<s1>,...,<sn>} 
    _AV05 = 0x0F, // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<freq>,<amp>),...]}
    _AV06 = 0x08, // (08) AVC-DFT { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<nc>,<pw>),...]}
    _SS00 = 0x0B // (11) Sub-Span, raw         
} typeset_t;

typedef struct {
    uint32_t dtime; // Timestamp
    typeset_t tset; // Typeset
    uint16_t ns; // number of single-sample
    uint16_t nss; // number of sequenced-samples
    sample_t* ss; // samples
} measurement_t;

//typedef enum e_measure {
//    MEASURE_WT = 0x01, // Wind Speed, Enviroment Temperature
//    MEASURE_WTL = 0x02 // Wind, Temperature, LVDT
//} measure_t;
//
//typedef enum {
//    __INIT,
//    __START,
//    __READ,
//    __STOP
//} measureCmd_t;


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
// 
uint16_t measurementAcquire(measurement_t *ms); // ret: nSamples
//
//uint16_t measurementCounter(); 
#define measurementCounter() g_dev.st.meas_counter  // ret: number of stored measurements

uint16_t measurementSave(measurement_t *ms); // ret: 0/Counter
uint16_t measurementLoad(uint16_t index, measurement_t *ms); // ret: -1 error or Index ( ONLY LAST ??? FIFO ???)
uint16_t measurementDelete(uint16_t index); // ret: -1 error or Index

void measurementGetBlock(uint8_t *pbuf, uint16_t offset, uint16_t size); // Exchange needs !!!

//uint16_t getRTMeasure(measureCmd_t cmd, measure_t mtype, sample_t *nsamp);

// -------------------------------------------------------------------------
uint16_t sstFreeSpaceKb();
// -------------------------------------------------------------------------


#endif	/* MEASUREMENT_H */


