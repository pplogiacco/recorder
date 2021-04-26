/*
 * File:   sampling.h
 * Author: plogiacco
 * Revision: merge ws lvdt proj dongle
 *
 * Created on October 25, 2020, 5:40 PM
 */
#ifndef SAMPLING_H
#define	SAMPLING_H

#include <xc.h>        // processor files
#include <stdbool.h>
#include "../device.h"    // Typeset defs / config params

#define sample_t uint16_t  // unsigned short
//typedef unsigned short sample_t; // ADC FormatAbsolute Integer Format
#define FLOAT2INT_FACTOR 100000U
#define SYNCO_FREQUENCY  38400U 
#define SCALE_TOUNSIGNED 1024U  // 12 bit 4096

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

// Dimensions Acquiring Routines
// -------------------------------------------------------------------------
#define SAMPLING_ET_EVERAGE 2   // Repeat samplings and compute everage
#define SAMPLING_WS_EVERAGE 2   // Repeat samplings and compute everage
#define SAMPLING_AV_PBUFFER 3   // Buffer to match P-P  points

/* No global callables
uint16_t acquireWS(sample_t *dbuf); // WIND SPEED
uint16_t acquireET(sample_t *dbuf); // ENV.TEMPERATURE
uint16_t acquireAV(sample_t *dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period); // AEOLIAN VIBRATION
//uint16_t acquireAV_NVM(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period);
uint16_t acquireSS(); // ENV.TEMPERATURE
uint16_t acquireSin(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t adc_period); // DEMO SIGNAL
*/

// Real Time Sampling
// -------------------------------------------------------------------------
   
#endif	/* SAMPLING_H */


