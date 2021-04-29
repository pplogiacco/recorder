#ifndef ACQUIRE_H
#define	ACQUIRE_H

#include <xc.h>        // processor files
#include <stdbool.h>
#include "../device.h"    // Typeset defs / config params

#define FLOAT2INT_FACTOR 100000U
#define SYNCO_FREQUENCY  38400U 
#define SCALE_TOUNSIGNED 1024U  // 12 bit 4096

// Dimensions Acquiring Routines
// -------------------------------------------------------------------------
uint16_t acquireSig(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t sig_freq, uint16_t sig_amp); // DEMO SIGNAL

uint16_t acquireWS(sample_t *dbuf); // WIND SPEED
uint16_t acquireET(sample_t *dbuf); // ENV.TEMPERATURE

uint16_t acquireAV(sample_t* dbuf, uint16_t nsec, uint16_t maxpoints, uint16_t av_period, uint16_t pp_filter);


uint16_t acquireSS(); // ENV.TEMPERATURE

#define ADC_FRQ_24Khz 3200U
#define ADC_FRQ_1Khz 8000U
#define ADC_FRQ_05Khz 16000U

uint16_t acquireAV_FFT(sample_t* dbuf, uint16_t nsec, uint16_t log2_npoints, uint16_t adc_fq, uint16_t fft_pw);



#endif	// ACQUIRE_H

