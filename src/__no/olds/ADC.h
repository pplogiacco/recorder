/* 
 * File:   ADC.h (ADC.c)
 * Author: plogiacco@smartlab.it
 * Date: October 25, 2020, 4:58 PM
 * Content: VAMP1K/Modules HAL AD Converter 
 */

#ifndef ADC_H
#define	ADC_H

#include <xc.h>
#include <stdbool.h>
#include "TMR.h"        // SR defs & TMR3

#ifdef	__cplusplus
extern "C" {
#endif

    // Default settings:
    // * process buffer in FIFO/Half mode 32*2 samples
    
    void ADC1_Enable(); // Manual mode  ( SAMP=1 )
    void ADC1_EnableSR(uint8_t channel, uint16_t t); // Sampre Rate = TMR3 periodiod
    void ADC1_EnableINT0(uint8_t channel); // INT0 pin-change event (also T3CK not implemented !)
    void ADC1_SetCallBack(void (* NewCallBack)(void));  // call after ADC1_Enable.....

    inline void ADC1_Start(unsigned long ns);  // Number of samples to acquire 
    inline bool ADC1_Running(); 
    inline void ADC1_Stop(); 
   
    void ADC1_Disable(); // Free resosurces 

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */


/*___PIC24Fv32KA304__________________________________________________________________________________
 * 
 * 
 SAMPLE-TIME is the time that the A/D module?s S/H amplifier is connected to the analog input pin.
 CONVERSION-TIME is the time required for the A/D Converter to convert the voltage held by the S/H amplifier.
 ADC-SEQUENCE-TIME is the sum of the sample time(s) and the A/D conversion time  
 
 When automatic sampling is used, SAMPLE-TIME can be extended ( AD1CON3bits.SAMC ) between the sampling ends and the conversion starts. 
 The sample time may be started and ended automatically by the A/D Converter?s hardware or under direct program control. 
 There is a minimum sample time to ensure that the S/H amplifier will give sufficient accuracy for the A/D conversion.
 The conversion trigger ends the sampling time and begins an A/D conversion or a repeating sequence.
 * The conversion trigger sources can be taken from a variety of hardware sources or can be controlled directly in software. 
 * The conversion trigger options "auto-conversion" uses a counter and the A/D clock to set the time between auto-conversions. 
 * The conversions without software intervention. 
 
 
 
 * An A/D conversion requires one A/D clock cycle (TAD) to convert each bit of the result, 
 * plus two additional clock cycles, or a total of 14 TAD cycles for a 12-bit conversion. 
 * 
 * The period of the A/D conversion clock is software selected using a 6-bit counter.
 *  
 * There are 64 possible options for TAD, specified by the ADCSX bits in the AD1CON3 register. 
 * 
 * Equation 51-1 gives the TAD value as a function of the ADCSx control bits and the device instruction cycle clock period, TCY.
 * For correct A/D conversions, the A/D conversion clock (TAD) must be selected to ensure a minimum TAD time,
 * as specified by the device family data sheet.
 * 
 * TAD = TCY (ADCSx + 1)
 * TCY = 2/FOSC; Doze mode and PLL are disabled.
 * 
 * ADCSX = (TAD/TCY) ? 1
 *  
 * 
 * 
 * When conversion is complete, the result is loaded into one of the A/D result buffers. 
 * The S/H can be reconnected to the input pin and a CPU interrupt may be generated. 

 * The A/D Converter also has its own dedicated RC clock source that can be used to perform
conversions. The A/D RC clock source should be used when conversions are performed while
the device is in Sleep mode. The RC oscillator is selected by setting the ADRC bit
(AD1CON3<15>). When the ADRC bit is set, the ADCSx bits have no effect on A/D operation.
 * 
 */


/*  ADC1 REGISTERS 
    
AD1CON1bits.ADSIDL = 1; // A/D Stop in Idle Mode bit
                        // 1 = Discontinues module in Idle mode
                        // 0 = Continues module operation in Idle mode

AD1CON1bits.MODE12 =0;  // 12-Bit Operation Mode bit
                        // 0=10bit, 1=12bit A/D operation

AD1CON1bits.FORM  = 0;  // Data Output Format bits 
                        // 11 = Fractional result, signed, left-justified
                        // 10 = Absolute fractional result, unsigned, left-justified
                        // 01 = Decimal result, signed, right-justified
                        // 00 = Absolute decimal result, unsigned, right-justified

AD1CON1bits.SSRC = 0;   // Sample Trigger Source Select bits
                        // 0111 = AUTO-CONVERT (Internal counter ends sampling and starts conversion)
                        // 0101 = Timer1 event ends sampling and starts conversion
                        // 0100 = CTMU event ends sampling and starts conversion
                        // 0011 = Timer5 event ends sampling and starts conversion
                        // 0010 = Timer3 event ends sampling and starts conversion
                        // 0001 = INT0
                        // 0000 = Manual conversion by SAMP bit

 AD1CON1bits.ASAM = 0;  // A/D Sample Auto-Start bit ( AUTO-SAMPLING )
                        // 1 = Sampling begins after last conversion; SAMP bit is auto-set
                        // 0 = Sampling begins when the SAMP bit is manually set
  

 //AD1CON1bits.SAMP = 0;    // AD Sample (write 1 to start S/H, read -> 1 Sampling, 0 Holding )
 //AD1CON1bits.DONE         // A/D Conversion Status: 1 completed, 0 not started/in progress    
 //AD1CON1bits.ADON = 1;    // Converter on/off


 * 
 * 
  
AD1CON2bits.PVCFG= 00;      // Converter Positive Voltage Reference Configuration bits
                            //11 = 4 * Internal VBG(2)
                            //10 = 2 * Internal VBG(3)
                            //01 = External VREF+
                            //00 = AVDD
 
AD1CON2bits.NVCFG = 00;     // Converter Negative Voltage Reference Configuration bits
                            //1 = External VREF-
                            //0 = AVSS

AD1CON2bits.BUFREGEN = 0;   // A/D Buffer Register Enable bit
                            // 1 = Conversion result is loaded into a buffer location determined by the converted channel
                            // 0 = A/D result buffer is treated as a FIFO
 
AD1CON2bits.CSCNA = 0;  // Scan Input Selections for CH0+ S/H Input for MUX A Setting bit
                        // 1 = Scans inputs
                        // 0 = Does not scan inputs
  
AD1CON2bits.BUFS  ;  //  Buffer Fill Status bit(1) !! BUFS is only used when BUFM = 1.
                        // 1 = A/D is filling the upper half of the buffer; user should access data in the lower half
                        // 0 = A/D is filling the lower half of the buffer; user should access data in the upper half
 
AD1CON2bits.SMPI = 2;    
                        // 11111 = Interrupts at the completion of the conversion for each 32nd sample
                        // 11110 = Interrupts at the completion of the conversion for each 31st sample
                        // 00001 = Interrupts at the completion of the conversion for every other sample
                        // 00000 = Interrupts at the completion of the conversion for each sample

AD1CON2bits.BUFM = 0 ;  // Buffer Fill Mode Select bit(1)
                        // 1 = Starts filling AD1BUF0, on the first interrupt and AD1BUF(n/2) on the next (Split Buffer mode)
                        // 0 = Starts filling the buffer at address, ADCBUF0, and each sequential address (FIFO mode)
        
AD1CON2bits.ALTS = 0 ;  // Alternate Input Sample Mode Select bit
                        // 1 = Uses channel input selects for Sample A on the first sample and Sample B on the next sample
                        // 0 = Always uses channel input selects for Sample A
    
AD1CON3bits.ADRC =  ;   // A/D Conversion Clock Source bit
                        // 1 = RC clock
                        // 0 = Clock is derived from the system clock
 * 
AD1CON3bits.EXTSAM= 1 ; // Extended Sampling Time bit
                        // 1 = A/D is still sampling after SAMP = 0
                        // 0 = A/D is finished sampling
 
AD1CON3bits.SAMC =  ;   // Auto-Sample Time Select bits
                        // 11111 = 31 TAD
                        // 00001 = 1 TAD
                        // 00000 = 0 TAD

AD1CON3bits.ADCS= ;     // A/D Conversion Clock Select bits
                        // 11111111-01000000 = Reserved
                        // 00111111 = 64·TCY = TAD
                        // 00000001 = 2·TCY = TAD
                        // 00000000 = TCY = TAD 
 
AD1CON5bits.ASEN = 0;   // Auto-Scan Enable bit(no configure clock source to Auto-Convert)
                        // 1 = Auto-scan is enabled
                        // 0 = Auto-scan is disabled
 * 
AD1CON5bits.LPEN= 0;    //  Low-Power Enable bit
                        // 1 = Returns to Low-Power mode after scan
                        // 0 = Remains in Full-Power mode after scan
 * 
AD1CON5bits.CTMREQ= 0;  // CTMU Request bit
                        // 1 = CTMU is enabled when the A/D is enabled and active
                        // 0 = CTMU is not enabled by the A/D
 * 
AD1CON5bits.BGREQ= 0;   // Band Gap Request bit
                        // 1 = Band gap is enabled when the A/D is enabled and active
                        // 0 = Band gap is not enabled by the A/D

AD1CON5bits.ASINT= 0;   // Auto-Scan (Threshold Detect) Interrupt Mode bits
                        // 11 = Interrupt after a Threshold Detect sequence completed and a valid compare has occurred
                        // 10 = Interrupt after a valid compare has occurred
                        // 01 = Interrupt after a Threshold Detect sequence completed
                        // 00 = No interrupt
 * 
AD1CON5bits.WM= 0;      // Write Mode bits
                        // 11 = Reserved
                        // 10 = Auto-compare only (conversion results are not saved, but interrupts are generated when a valid match, as defined by the CMx and ASINTx bits, occurs)
                        // 01 = Convert and save (conversion results are saved to locations as determined by the register bits when a match, as defined by the CMx bits, occurs)
                        // 00 = Legacy operation (conversion data is saved to a location determined by the buffer register bits)

AD1CON5bits.CM = 0;     // Compare Mode bits
                        // 11 = Outside Window mode (valid match occurs if the conversion result is outside of the window defined by the corresponding buffer pair)
                        // 10 = Inside Window mode (valid match occurs if the conversion result is inside the window defined by the corresponding buffer pair)
                        // 01 = Greater Than mode (valid match occurs if the result is greater than the value in the corresponding buffer register)
                        // 00 = Less Than mode (valid match occurs if the result is less than the value in the corresponding buffer register)


AD1CHS: A/D SAMPLE SELECT REGISTER
----------------------------------
AD1CHSBITS.CH0NA = ;    // Sample A Channel 0 Negative Input Select bits  
                        // 111 = AN6(1)
                        // 110 = AN5(2)
                        // 101 = AN4
                        // 100 = AN3
                        // 011 = AN2
                        // 010 = AN1
                        // 001 = AN0
                        // 000 = AVSS

AD1CHSBITS.CH0SA= ;     // S/H Amplifier Positive Input Select for MUX A Multiplexer Setting bits
                        // 11111 = Unimplemented, do not use
                        // 11110 = AVDD
                        // 11101 = AVSS
                        // 11100 = Upper guardband rail (0.785 * VDD)
                        // 11011 = Lower guardband rail (0.215 * VDD)
                        // 11010 = Internal Band Gap Reference (VBG)(3)
                        // 11001-10010 = Unimplemented, do not use
                        // 10001 = No channels are connected, all inputs are floating (used for CTMU)
                        // 10000 = No channels are connected, all inputs are floating (used for CTMU temperature sensor input)
                        // 01111 = AN15
                        // 01110 = AN14
                        // 01101 = AN13
                        // 01100 = AN12
                        // 01011 = AN11
                        // 01010 = AN10
                        // 01001 = AN9
                        // 01000 = AN8(1)
                        // 00111 = AN7(1)
                        // 00110 = AN6(1)
                        // 00101 = AN5(2)
                        // 00100 = AN4
                        // 00011 = AN3
                        // 00010 = AN2
                        // 00001 = AN1
                        // 00000 = AN0

AD1CHSBITS.CH0NB = ;    // Sample B Channel 0 Negative Input Select bits
AD1CHSBITS.CH0SB= ;     // S/H Amplifier Positive Input Select for MUX B Multiplexer Setting bits 
  

  
 
 

 * 
 */

