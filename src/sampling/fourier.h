
#ifndef FOURIER_H
#define	FOURIER_H

// Signal & FFT 
typedef signed short fix14_t;
typedef unsigned short u16_t;

#define LOG2_N_WAVE     10     // log2(N_WAVE)   
#define N_WAVE          1024   // Signal & FFT points  

void fft_init(u16_t log2_n_wave);

inline fix14_t fft_windowing(fix14_t sample, u16_t i);

void fft_window(fix14_t fr[], fix14_t fi[], u16_t n_wave);

void fft_fix(fix14_t fr[], fix14_t fi[], u16_t m);

void fft_spectrum(fix14_t SSBUF[], u16_t m);

#endif	// FOURIER_H

