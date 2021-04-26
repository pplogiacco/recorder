#ifndef ADA2200_H
#define	ADA2200_H

#include <xc.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
void ADA2200_Enable();  // 
void ADA2200_Synco(uint8_t synco_trim); 
void ADA2200_Disable(); //     


#define ADA_SPIRS       0x0000   //  Reset & SPI control register
    // [7] Reset  ('1'places the device in reset until 0 is written. All registers to default)
    // [6] set LSB first ( 1 = LSB first )
    // [5] Address increment ( increment mode for multi-byte register access: 0=decrement, 1=increment)
    // [4] 3/4-wire SPI (0=SDIO bidirectional 3Wire. 1= SDO signal is enabled)
    // [3] SDO active   ( mirror))
    // [2] Address increment (mirror)
    // [1] LSB first    (mirror))
    // [0] Reset        (mirror)
    // Default 0x00

#define ADA_DCTRL       0x002A    // Demod control
    // [7] Unused
    // [6] PHASE90
    // [5] Unused
    // [4] Mixer enable (1= last sample taken while RCLK remains held while RCLK is inactive)
    // [6] RCLK select (0=sends the SDO signal on Pin 13,1=sends RCLK signal)
    // [2:0] VOCM select  ( 000 = VOCM pin to VDD/2 Low power mode,
    //                      001= external reference for VOCM,
    //                      010 = set the VOCM pin to VDD/2. Fast settling mode.
    //                      101 = set the VOCM pin to 1.2V )
    // Default 0x18

/* SYSTEM CLOCK
 * Filtering parameters such as corner frequency, the center of the pass band,
 * or the location of the Nyquist frequency depend on the frequency of the clock
 * used to drive the device. 
 * A natural consequence of this fundamental property is that it is possible to 
 * scale these filters by changing the clock frequency or using the on-chip clock
 * dividers.
 * 
 * The ADA2200 has on-chip clock dividers to generate the system clocks.
 * The input clock divider, CLKIN DIV[2:0], sets the input sample rate of the 
 * decimator (Fsi) by dividing the CLKIN signal. 
 * The value of CLKIN DIV[2:0] can be set to 1, 16, 64, or 256.
 * 
 * The output sample rate (Fso) is always 1/8th of the decimator input sample rate.
 * The RCLK divider, RCLK DIV[1:0], sets the frequency of the mixer frequency, fM 
 * (which is also the frequency of RCLK) by dividing Fso by either 4 or 8.
 * 
 */    

#define ADA_CKDIV       0x002B    // Clock configuration
    // [7:5] Unused
    // [4:2] CLKIN DIV  ( The division factor between fCLKIN and fSI.
    //                      000 = divide by 1.
    //                      001 = divide by 16.
    //                      010 = divide by 64.
    //                      100 = divide by 256 )
    // [1:0] RCLK DIV   ( set the division factor between fSO and fM.
    //                      00 = reserved.
    //                      01 = the frequency of RCLK is fSO/4.
    //                      10 = the frequency of RCLK is fSO/8.
    //                      11 = reserved )
    // Default  0x02

#define ADA_PINSW       0x002C  // Digital pin configuration
    // [7:1] Unused
    // [0] RCLK/SDO output enable (1 = RCLK/SDO output pad driver is enabled )
    // Default 0x01

#define ADA_RESET       0x002D  // Core reset
    // [7:1] Unused
    // [0] Core reset
    // Default 0x00

#define ADA_CHECK       0x002E  // Checksum
    // [7:0] Checksum value
    // Default N/A (read only)

#define ADA_EPROM       0x002F   // EEPROM status
    // [7:3] Unused
    // [2] Checksum failed
    // [1] Checksum passed
    // [0] Boot from EEPROM complete
    // Default N/A (read only)

#define ADA_HWVER       0x0006  // Chip version
    // [7:4] Unused 
    // [3:0] Die revision
    // Default (read only)
    
#define ADA_INPUT   0x0028 // Clock Source & Analog Input Configuration
    // [7:2] Unused  
    // [1] INP gain ( 1 = only INP signal is sampled. Additional 6 dB of gain is applied )
    // [0] Clock source select ( 0 = generate a clock by crystal placed between XOUT and CLKIN 
    //                           1 = accept a CMOS level clock on the CLKIN pin. The internal XOUT driver is disabled)
    // Default 0x00
    
    
/* SYNCO (MIXER)
 * The ADA2200 performs the mixing function by holding the output samples 
 * constant for ½ of the RCLK period. 
 * In the default configuration, there are eight output sample periods during
 * each RCLK cycle. 
 * There are four updated output samples while the RCLK signal is high. 
 * While RCLK is low, the fourth updated sample is held constant for four 
 * additional output sample periods.
 *  
 * The RCLK divider, RCLK DIV[1:0], can be set to divide fSO by 4. When this
 * mode is selected, four output sample periods occur during each RCLK cycle.
 * Two output samples occur while the RCLK signal is high. While RCLK is low,
 * the second updated sample is held constant for two output sample periods.
 * 
 * The mixer can be bypassed. When the mixer is bypassed, the output produces
 * an updated sample value every output sample period.
 * One output sample of the ADA2200 is 8 fSI clock cycles long.
 * The SYNCO pulse is 1 fSI clock cycle in duration. 
 * The SYNCO pulse can be programmed to occur at 1 of 16 different timing offsets.
 * The timing offsets are spaced at 1/2 fSI clock cycle intervals and span the 
 * full output sample window.                
 */
    
#define ADA_SYNCO       0x0029  // SYNCO control (MIXER)
    // [7:6] Unused  
    // [5] SYNCO output enable ( 1 = enables the SYNCO output pad driver )
    // [4] SYNCO invert (1 = inverts the SYNCO signal)
    // [3:0] SYNCO edge select ( 16 different edge locations for the SYNCO pulse )
    // Default 0x2D ( 1 = delays the phase between the RCLK output and the strobe )

    
/* IIR FILTER
 * e IIR block operates at the output sample rate (fSO).
 * Output sample rate (fSO) is 1/8th ofthe input sample rate (fSI). 
 * By default, the IIR filter is configured as a band-pass filter with a center 
 * frequency at fSO/8 (fSI/64).
 * This frequency corresponds to the default mixing frequency and assures that
 * input signals in the center of the pass band mix down to dc.
 * 
 * For example, the corner frequency for a 4th-order low-pass filter with a corner
 * frequency at fC = 0.4 fN can be calculated by
 *
 *      fC = 0.4 * 0.5fSO  = 0.4 * 0.5 * 0.125 fSI = 0.025 * 2^(-m) * Fclk
 * 
 * "m" is the divider of the clock frequency and can take values of 1, 2, or 8.
 * Fclk is the frequency of the clock signal applied to CLKIN. 
 * If Fclk = 500 kHz and m = 1, the corner will be located at 12.5 kHz, 
 * Fso = 62.5kHz and Fn = 31.25 kHz. 
 * With all the parameters remaining the same, reducing the clock frequency to
 * 100 kHz would move the corner frequency down to 2.5 kHz.
 * Note that band-pass or band-stop filters will remain with a constant Q. 
 * This means that if the frequency doubles, the width of the pass band will 
 * double as well. In the case of the notch, the stop band doubles.
 * 
 * If a different frequency response is required, the IIR can be programmed for a
 * different response. Register 0x0011 through Register 0x0027 contain coefficient
 * values that program the filter response. 
 * The coefficients can then be loaded into the filter by writing to ADA_UPDFILTER.
 * 
 */
 #define ADA_STROBE     0x0010 // Filter strobe (IIR)
    // [7:2] Unused 
    // [1:0] Load coefficients   (When toggled from 0 to 1, the coefficients are loaded into IIR filter)
    // Default 0x00 ( write 0x03 to update IIR coeffients )
    
#define ADA_FK11 0x0011 // 0xC02
#define ADA_FK12 0x0012 // 0x0F2
#define ADA_FK13 0x0013 // 0x1D2
#define ADA_FK14 0x0014 // 0xD72
#define ADA_FK15 0x0015 // 0xC02
#define ADA_FK16 0x0016 // 0x0F2
#define ADA_FK17 0x0017 // 0xC02
#define ADA_FK18 0x0018 // 0x0F2
#define ADA_FK19 0x0019 // 0x1D2
#define ADA_FK1A 0x001A // 0x972
#define ADA_FK1B 0x001B // 0x7E2
#define ADA_FK1C 0x001C // 0x882
#define ADA_FK1D 0x001D // 0xC02
#define ADA_FK1E 0x001E // 0x0F2
#define ADA_FK1F 0x001F // 0xC02
#define ADA_FK20 0x0020 // 0x0F2
#define ADA_FK21 0x0021 // 0xC02
#define ADA_FK22 0x0022 // 0x0F2
#define ADA_FK23 0x0023 // 0x002
#define ADA_FK24 0x0024 // 0xE02
#define ADA_FK25 0x0025 // 0x232
#define ADA_FK26 0x0026 // 0x022
#define ADA_FK27 0x0027 // 0x242

// Disable the decimator by writing a Logic 1 to Register 0x027 Bit 6.
 
    
 /* Recommended Register Contents for Different Filters:
  * ( Using ADA2200 as a Time-Domain-Filter (Ref. MT-234)
  * 
  * Register Address        BP1     BP2     LP1     LP2     Notch	Default All-Pass
  * 0x0011                  0xC0	0xC0	0x52	0x51	0xC0	0xC0	0x00
  * 0x0012                  0x0F	0x0F	0xAE	0x80	0x4F	0x0F	0xA0
  * 0x0013                  0x36    0xFA    0x52    0x40    0x84    0x1D    0xC0
  * .... 
  */   
    
    
   
#ifdef	__cplusplus
}
#endif
#endif	/* ADA2200_H */



/* AMPLITUDE MEASUREMENTS
 * If the relative phase of the input signal to the ADA2200 remains constant, the
 * output amplitude is directly proportional to the amplitude of the input signal.
 * 
 *  Note that the signal gain is a function of the relative phase of the input signal.
 *  Figure 15 shows the relationship between the cycle mean output and the relative phase. 
 * 
 * The cycle mean output voltage is 
 *      VCYCLEMEAN = Conversion Gain × VIN(RMS) × sin(?REL ? ?DEL) = 1.05 ×VIN(RMS) × sin(?REL ? ?DEL)
 * Therefore, the highest gain, and thus the largest signal-to-noise ratio measurement, is obtained when 
 * operating the ADA2200 with ?REL = ?DEL + 90° = 173°. This value of ?REL is also the operating point 
 * with the lowest sensitivity to changes in the relative phase. Operating with ?REL = ?DEL ? 90° = ?7° offers the same gain and measurement accuracy, but with a sign inversion.
 * 
 * PHASE MEASUREMENTS
 * If the amplitude of the input signal to the ADA2200 remains constant, the output
 * amplitude is a function of the relative phase of the input signal. 
 * The relative phase can be measured as: 
 *      ?REL = sin?1(VCYCLEMEAN/(Conversion Gain VIN(RMS))) + ?DEL 
 * Note that the output voltage scales directly with the input signal amplitude. 
 * A full-scale input signal provides the greatest phase sensitivity (V/°?REL) and
 *  thus the largest signal-to-noise ratio measurement.
The phase sensitivity also varies with relative phase. The sensitivity is at a maximum when ?REL = 83°. 
 * For this reason, the optimal measurement range is for input signals with a relative phase equal to the phase delay of ±45°. This range provides the highest gain and thus the largest signal-to-noise ratio measurement. This range is also the operating point with the lowest sensitivity to changes in the relative phase. Operating at a relative phase equal to the phase delay of ?135° to ?225° offers the same gain and measurement accuracy, but with a sign inversion.
The phase sensitivity with a 4 V p-p differential input operating with a relative phase that is equal to the phase delay results in a phase sensitivity of 36.6 mV/°?REL.
AMPLITUDE AND PHASE MEASUREMENTS
When both the amplitude and relative phase of the input signals are unknown, it is necessary to obtain two orthogonal components of the signal to determine its amplitude, relative phase, or both. These two signal components are referred to as the in-phase (I) and quadrature (Q) components of the signal.
The decimation filter removes all the images above fSO/2 and below fSI ? fSO/2. 

  
 * Disabling the Decimation Filter
 * It is possible to disable the decimation filter; however, that does not eliminate
 * the decimation action. When the decimation filter is disabled, the input sampling
 * occurs at the same rate fSI, and the programmable filter block continues to sample
 * the output of the decimator block at ? of the input rate. Therefore, the maximum 
 * operation frequency of the IIR continues to be fSI/8.
 * 
 * What is accomplished by disabling the decimator is eliminating the decimator 
 * roll-off and the phase shift introduced by it. This may be desired if the 
 * additional antialiasing is not required. 
 * Disable the decimator by writing a Logic 1 to Register 0x027 Bit 6.
 
 */