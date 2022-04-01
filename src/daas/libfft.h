/******************************************************************************
 * L I B F F T                                               (@) DaaS Project *
 ******************************************************************************
 * Author: plogiacco
 * 
 * Revision history: 
 * Rel.0.0.3 - 15/11/21 
 * 
 * Comments:

 ******************************************************************************
 */
#ifndef LIBFFT_H
#define	LIBFFT_H

// Signal & FFT 
typedef signed short fix14_t;

void fft_init(short log2_n);

fix14_t fft_windowing(fix14_t sample, short i);

void fft_window(fix14_t fr[], fix14_t fi[]);

//void fft_fix(fix14_t fr[], fix14_t fi[], int m);

void fft_spectrum(fix14_t ptrPoints[]);

#endif	// LIBFFT_H

