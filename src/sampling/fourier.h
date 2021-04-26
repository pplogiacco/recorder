
#ifndef FOURIER_H
#define	FOURIER_H

// Signal & FFT 
typedef signed short fix14_t;

#define LOG2_N_WAVE     10     // log2(N_WAVE)   
#define N_WAVE          1024   // Signal & FFT points  

void fft_init();

inline fix14_t fft_windowing(short sample, unsigned short i);

void fft_window(fix14_t fr[], fix14_t fi[], int n_wave);

void fft_fix(fix14_t fr[], fix14_t fi[], int m);

void fft_spectrum(fix14_t SSBUF[], int m);

#endif	// FOURIER_H

