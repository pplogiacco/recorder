#include <xc.h>
#include "libfft.h"

#include <math.h>
#include <stdlib.h> // abs

// Options

#undef __FFT_TEST
#undef __FFT_OUT8BIT

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__)|| defined(__PIC24FV32KA304__))
#undef __FFT_TABLED
//#define LOG2_N_WAVE     10     // log2(N_WAVE)   
//#define N_WAVE          1024   // Signal & FFT points  

#define MAX_LOG2_N_WAVE     10     // log2(N_WAVE)   

#elif defined (__PIC24FJ256GA702__)
#define __FFT_TABLED
//#define LOG2_N_WAVE     10     // log2(N_WAVE)   
//#define N_WAVE          1024   // Signal & FFT points  

#define MAX_LOG2_N_WAVE     10     // log2(N_WAVE)   
#define MAX_N_WAVE          1024   // Signal & FFT points  

#endif


#ifdef __FFT_TEST
//#define __USE_PRINTF
#define __USE_DATAVIS  
#include <stdio.h>  // printf()
#include "mcc_generated_files/uart2.h" // write()
#endif

static short LOG2_N = 0; // log2(N_WAVE)   
static short N_WAVE = 0; // Signal & FFT points  

#define multfix14(a,b) ((fix14_t)((((long)(a))*((long)(b)))>>14)) //multiply two fixed 2.14
#define float2fix14(a) ((fix14_t)((a)*16384.0)) // 2^14
#define fix2float14(a) ((float)(a)/16384.0)
#define absfix14(a) abs(a) 
#define max(a,b) (a>b)?a:b
#define min(a,b) (a<b)?a:b
#define log_min 0x10   
#define _Pi  3.14159265359
#define _2Pi 6.28318530718

#if defined(__FFT_TABLED) 
static fix14_t Sinewave[MAX_N_WAVE]; // a table of sines for the FFT
static fix14_t window[MAX_N_WAVE]; // a table of window values for the FFT
#else
#define SinTab(x,n)  float2fix14( sin(6.283 * ((float) x) / n)*0.5)
#define WinTab(x,n)  float2fix14(1.0 * (1.0 - cos(6.283 * ((float) x) / (n - 1))))
#endif


/******************************************************************************
 * Initialize sin & cos tables to optimize FFT  
 ******************************************************************************/
void fft_init(short log2_n) { // populate one cycle sine table required for FFT, and cosine table for windowing
    if (log2_n != LOG2_N) {
        LOG2_N = (log2_n > MAX_LOG2_N_WAVE)?MAX_LOG2_N_WAVE:log2_n;
        N_WAVE = 1U << LOG2_N; // Signal & FFT points  
#if defined(__FFT_TABLED)     
        short i;
        for (i = 0; i < N_WAVE; i++) {
            Sinewave[i] = float2fix14(sinf(_2Pi * ((float) i) / N_WAVE)*0.5);
            //printf("[%d] = %d", ii, Sinewave[ii]);
            //window[i] = float2fix14(1.0 * (1.0 - cosf(_2Pi * ((float) i) / (N_WAVE - 1))));
            window[i] = float2fix14(1.0) ;
        }
#endif
    }
}

/******************************************************************************
 * Windowing sampled points before compute FFT    
 ******************************************************************************/
inline fix14_t fft_windowing(fix14_t sample, short i) {
#if defined(__FFT_TABLED) 
    return multfix14(sample, window[i]);
#else
    return multfix14( sample, WinTab(i, N_WAVE) );
#endif
}

void fft_window(fix14_t fr[], fix14_t fi[]) {
    int i;
    for (i = 0; i < N_WAVE; i++) { // window the input array
#if defined(__FFT_TABLED) 
        fr[i] = multfix14(fr[i], window[i]);
#else
        fr[i] = multfix14(fr[i], WinTab(i, N_WAVE));
#endif
        fi[i] = 0;
    }
}

/******************************************************************************
 * In-place, real, forward only FFT   
 ******************************************************************************/
void fft_fix(fix14_t fr[], fix14_t fi[], int m) {
    int mr, nn, i, j, L, k, istep, n;
    int qr, qi, tr, ti, wr, wi;

    mr = 0;
    m = LOG2_N;
    n = 1U << m; // Signal points 
    nn = n - 1;

    for (m = 1; m <= nn; ++m) { // decimation in time (bit-reversing)
        L = n;
        do L >>= 1; while (mr + L > nn);
        mr = (mr & (L - 1)) + L;
        if (mr <= m) continue;
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        //ti = fi[m];   // don't need for real input
        //fi[m] = fi[mr];
        //fi[mr] = ti;
    }

    L = 1;
    k = LOG2_N - 1;
    while (L < n) { // 
        istep = L << 1;
        for (m = 0; m < L; ++m) {
            j = m << k;
#if defined(__FFT_TABLED) 
            wr = Sinewave[j + N_WAVE / 4];
            wi = -Sinewave[j];
#else
            wr = SinTab(j + N_WAVE / 4, N_WAVE); 
            wi = -SinTab(j, N_WAVE); 
#endif
            for (i = m; i < n; i += istep) {
                j = i + L;
                tr = multfix14(wr, fr[j]) - multfix14(wi, fi[j]);
                ti = multfix14(wr, fi[j]) + multfix14(wi, fr[j]);
                qr = fr[i] >> 1;
                qi = fi[i] >> 1;
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }
        }
        --k;
        L = istep;
    }
}

void fft_spectrum(fix14_t ptrPoints[]) {
    static fix14_t const zero_point_4 = float2fix14(0.4);
    //static int sx, y, ly;
    static short i, nPoints;

    nPoints = 1U << LOG2_N; // m = log2_N

    fft_fix(ptrPoints, ptrPoints + nPoints, LOG2_N); // do FFT

    int N2 = (nPoints >> 1) + 1; // Harmonics 

    // The magnitude of the FFT is approximated as: 
    //   |amplitude| = max(|Re|,|Im|) + 0.4 * min(|Re|,|Im|)
    // Accurancy is about 4% rms.

    for (i = 1; i < N2; i++) {
        // get the approx magnitude
        ptrPoints[i] = abs(ptrPoints[i]); //>>9
        ptrPoints[i + nPoints] = abs(ptrPoints[i + nPoints]);

        // In-place magnitude compute and hold
        ptrPoints[i] = max(ptrPoints[i], ptrPoints[i + nPoints]) + multfix14(min(ptrPoints[i], ptrPoints[i + nPoints]), zero_point_4);

#if defined(__FFT_OUT8BIT)

        // shifting finds most significant bit:
        //      approxlog  = ly + (fr-y)/(y) + 0.043;

        // for an 8-bit approx (4 bit ly and 4-bit fraction)
        //      ly: 1 <= ly <= 14, omit the 0.043 because it is too small
        int sx, y, ly;

        sx = ptrPoints[i];
        y = 1;
        ly = 0;
        while (sx > 1) {
            y = y * 2;
            ly = ly + 1;
            sx = sx >> 1;
        }

        // shift ly into upper 4-bits as integer part of log
        // take bits below y and shift into lower 4-bits
        ptrPoints[i] = ((ly) << 4) + ((ptrPoints[i] - y)>>(ly - 4));
#endif        
        // bound the noise at low amp
        if (ptrPoints[i] < log_min) ptrPoints[i] = log_min;

#if defined(__USE_DATAVIS ) // Datavis  
        UART2_Write(0x5F);
        UART2_Write(i & 0xFF); // n
        UART2_Write(i >> 8);
        UART2_Write(ptrPoints[i] & 0xFF); // Kn
        UART2_Write(ptrPoints[i] >> 8);
        UART2_Write(0xA0);
#endif
    }
}


