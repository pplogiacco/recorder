/* 
 * File:   CMP.h
 * Author: plogiacco
 *
 * Created on November 30, 2020, 3:25 AM
 */

#ifndef CMP_H
#define	CMP_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif


    void IC2_EnableT3RB2(); // Capture event on RB? pin rising edge

    void IC2_SetCallBack(void (* NewCallBack)(void)); // Call after Enable

    inline void IC2_Disable(void);

    inline void IC2_Start(void);

    inline void IC2_Stop(void);

    inline bool IC2_Running(void);

    inline uint16_t IC2_Read(void);




#ifdef	__cplusplus
}
#endif

#endif	/* CMP_H */



/*
 - Catturare lo stato di un pin ad ogni evento di altro modulo ( Trigger mode ) 
 - contare gli eventi che si verificano su un pin ( Async mode )
 - The input capture module can also be configured to function as an external interrupt pin.
 * wake-up from power sleep mode:
   When the input capture module is configured to ICM<2:0> = 111 and the respective
   channel interrupt enable bit is asserted (ICxIE = 1), a rising edge on the capture
   pin will wake-up the device from Sleep (see Section 15.8 ?Input Capture Operation
   in Power-Saving States?).
  
 ICxCON1: INPUT CAPTURE x CONTROL REGISTER 1
 
 bit 15-14 Unimplemented: Read as ?0?
bit 13 ICSIDL: Input Capture x Module Stop in Idle Control bit
        1 = Input capture module halts in CPU Idle mode
        0 = Input capture module continues to operate in CPU Idle mode
bit 12-10 ICTSEL<2:0>: Input Capture x Timer Select bits
        111 = System clock (FOSC/2)
        110 = Reserved
        101 = Reserved
        100 = Timer1
        011 = Timer5
        010 = Timer4
        001 = Timer2
        000 = Timer3
bit 9-7 Unimplemented: Read as ?0?
bit 6-5 ICI<1:0>: Select Number of Captures per Interrupt bits
        11 = Interrupt on every fourth capture event
        10 = Interrupt on every third capture event
        01 = Interrupt on every second capture event
        00 = Interrupt on every capture event
bit 4 ICOV: Input Capture x Overflow Status Flag bit (read-only)
        1 = Input capture overflow occurred
        0 = No input capture overflow occurred
bit 3 ICBNE: Input Capture x Buffer Empty Status bit (read-only)
        1 = Input capture buffer is not empty, at least one more capture value can be read
        0 = Input capture buffer is empty
bit 2-0 ICM<2:0>: Input Capture Mode Select bits
        111 = Interrupt mode: Input capture functions as an interrupt pin only when the
              device is in Sleep or Idle mode (rising edge detect only, all other control 
              bits are not applicable)
        110 = Unused (module disabled)
        101 = Prescaler Capture mode: Capture on every 16th rising edge
        100 = Prescaler Capture mode: Capture on every 4th rising edge
        011 = Simple Capture mode: Capture on every rising edge
        010 = Simple Capture mode: Capture on every falling edge
        001 = Edge Detect Capture mode: Capture on every edge (rising and falling); ICI<1:0 bits do not
              control interrupt generation for this mode
        000 = Input capture module is turned off
 * 
 * 
 * 
 * 
ICxCON2: INPUT CAPTURE x CONTROL REGISTER 2
 * 
bit 15-9 Unimplemented: Read as ?0?
bit 8 IC32: Cascade Two IC Modules Enable bit (32-bit operation)
        1 = ICx and ICy operate in cascade as a 32-bit module (this bit must be set in both modules)
        0 = ICx functions independently as a 16-bit module
bit 7 ICTRIG: Input Capture x Sync/Trigger Select bit
        1 = Triggers ICx from source designated by the SYNCSELx bits
        0 = Synchronizes ICx with source designated by the SYNCSELx bits
bit 6 TRIGSTAT: Timer Trigger Status bit
        1 = Timer source has been triggered and is running (set in hardware, can be set in software)
        0 = Timer source has not been triggered and is being held clear
bit 5 Unimplemented: Read as ?0?
bit 4-0 SYNCSEL<4:0>: Trigger/Synchronization Source Selection bits
        11111 = Reserved
        11110 = Reserved
        11101 = Reserved
        11100 = CTMU(1)
        11011 = A/D(1)
        11010 = Comparator 3(1)
        11001 = Comparator 2(1)
        11000 = Comparator 1(1)
        10111 = Input Capture 4
        10110 = Input Capture 3
        10101 = Input Capture 2
        10100 = Input Capture 1
        10011 = Reserved
        10010 = Reserved
        1000x = Reserved
        01111 = Timer5
        01110 = Timer4
        01101 = Timer3
        01100 = Timer2
        01011 = Timer1
        01010 = Input Capture 5
        01001 = Reserved
        01000 = Reserved
        00111 = Reserved
        00110 = Reserved
        00101 = Output Compare 5
        00100 = Output Compare 4
        00011 = Output Compare 3
        00010 = Output Compare 2
        00001 = Output Compare 1
        00000 = Not synchronized to any other module
 * 
 * 
 * 

 
 */