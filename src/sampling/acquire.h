#ifndef ACQUIRE_H
#define	ACQUIRE_H

#include <xc.h>        // processor files
#include <stdbool.h>
#include "../device.h"    // Typeset defs / config params
#include "measurement.h"

//#define FLOAT2INT_FACTOR 100000U
//#define SYNCO_FREQUENCY  38400U 
//#define SCALE_TOUNSIGNED 1024U  // 12 bit 4096
//
//#define PR3_FQ_24Khz   3200U
//#define PR3_FQ_1Khz    8000U
//#define PR3_FQ_05Khz  16000U

// Dimensions Acquiring Routines
// -------------------------------------------------------------------------

 // DEMO SIGNAL
uint16_t acquireSig(sample_t* dbuf, uint16_t nsec, uint16_t nsamples, uint16_t sig_freq, uint16_t sig_amp, bool add_deltatick);

uint16_t acquireWS(sample_t *dbuf); // WIND SPEED

uint16_t acquireET(sample_t *dbuf); // ENV.TEMPERATURE

// Acquire Raw data (pp_filter=0 Raw, pp_filter>0 P-P ) 
uint16_t acquireAV_RAW(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t av_period);

// Acquire Raw data no Delta Time
uint16_t acquireAV_RNT(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t adc_pr3);

uint16_t acquireAV_P2P(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t adc_pr3, uint16_t pp_filter);

// Acquire Real FFT Spectrum
uint16_t acquireAV_DFT(sample_t* dbuf, uint16_t nsec, uint16_t log2_npoints, uint16_t adc_pr3);

// Acquire Event Counter 
uint16_t acquireAV_EVC_P2P(sample_t* dbuf, uint16_t nsec, uint16_t dbsize, uint16_t wbsize, uint16_t adc_pr3, uint16_t p2p_filter );
uint16_t acquireAV_EVC_DFT(sample_t* dbuf, uint16_t nsec, uint16_t db_size, uint16_t log2_npoints, uint16_t adc_pr3, uint16_t fft_min_pw, uint16_t fft_avg_co);

uint16_t acquireSS(); // SubSpan


typedef struct {    // Used in P2P
    sample_t T;
    sample_t A;
} point_t;

typedef struct {
    sample_t occ; // Occurencies counter
    sample_t nC; // Number of FFT Coefficient
    sample_t Pw; // Coefficient Spectral Power
} vibration_t;

typedef struct {
    sample_t n; // Occurencies counter
    sample_t T; // Number of FFT Coefficient
    sample_t A; // Coefficient Spectral Power
} oscill_t;

#endif	// ACQUIRE_H

