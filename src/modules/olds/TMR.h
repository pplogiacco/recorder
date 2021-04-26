/* 
 * File:   TMR.h (TMR.c)
 * Author: plogiacco@smartlab.it
 * Date: October 25, 2020, 4:58 PM
 * Content: VAMP1K/Modules HAL Timers  
 */

#ifndef _TMR_H
#define _TMR_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

// _________________________PR3(Fosc=8Mhz/Fcy=4Mhz/NoPs)
#define TMR3T_30Hz  0xFFFF   // 30.55Hz/32.72mS
#define TMR3T_500Hz 0x0FA3   // 500Hz/2mS
#define TMR3T_1KHz  0x7CC   // 1KHz/1ms    
#define TMR3T_4Mhz  0x001    // 40Khz/24us
    
// _________________________PR3(Fosc=8Mhz/Fcy=4Mhz/Ps1:8)
//#define TMR3T_500Hz 0x1F3   // 250Hz/4mS 999 -> 0.2ms
#define TMR3T_4Hz   0xFF00   // 999 -> 0.2ms    
#define TMR3T_60Hz  4170    // 60Hz/16.66 ms
#define TMR3T_250Hz 999     // 250Hz/4mS 999 -> 0.2ms
#define TMR3T_1Khz  0x1F3   // 499 -> 0.1ms 
#define TMR3T_2Khz  0x0C7   // 199 -> 500ns


    void TMR3_Enable(uint16_t period);
    
    void TMR3_EnableADC(uint16_t period);
    
    //void TMR3_EnableT3CK(); // Not implemented
    
    void TMR3_SetCallBack(void (*NewCallBack)(void)); // Call after Enable
    
    void TMR3_Disable(void);    
    
    inline void TMR3_Start(void);

    inline void TMR3_Stop(void);
    
    inline uint16_t TMR3_Counter(void); 
    
    inline void TMR3_Reset(void);
    

#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif //_TMR_H


/*___PIC24Fv32KA304__________________________________________________________________________________
 * 
 * 
 * 
 * Configure Timer2/3 or Timer4/5 for 16/32-bit operation:
 * 
 * 
 CONFIGURE 16BIT TIMERS
 To configure any of the timers for individual 16-bit operation:
 1. Clear the T32 bit corresponding to that timer (T2CON<3> for Timer2 and Timer3 or T4CON<3> for Timer4 and Timer5).
 2. Select the timer prescaler ratio using the TCKPS<1:0> bits.
 3. Set the Clock and Gating modes using the TCS and TGATE bits.
 4. Load the timer period value into the PRx register.
 5. If interrupts are required, set the Timerx Interrupt Enable bit, TxIE; use the Timerx Interrupt Priority bits, TxIP<2:0>, to set the interrupt priority.
 6. Set the TON bit (TxCON<15> = 1).
 * 
 A/D Special Event Trigger
 On each device variant, one Type C timer can generate a special A/D conversion trigger signal
 on a period match, in both 16-bit and 32-bit modes. The timer module provides a conversion start
 signal to the A/D sampling logic.
  ? If T32=0, when a match occurs between the 16-bit timer register (TMRx) and the
    respective 16-bit period register (PRx), the A/D Special Event Trigger signal is generated
  ? If T32=1, when a match occurs between the 32-bit timer (TMRx:TMRy) and the 32-bit
   respective combined period register (PRx:PRy), the A/D Special Event Trigger signal is generated
The Special Event Trigger signal is always generated by the timer. The trigger source must be
selected in the A/D converter control registers. For additional information, refer to
Section 16. ?Analog-to-Digital Converter (ADC)? (DS70621), and the specific device data
sheet 


  The Timer3 timer/counter modules incorporate these features:
   - Software-selectable operation as a 16-bit timer or counter
   - One 16-bit readable and writable Timer Value register
   - Selectable clock source (internal or external) with device clock or SOSC/LPRC options
   - Interrupt-on-overflow
   - Multiple timer gating options, including:
        - User-selectable gate sources and polarity
        - Gate toggle operation
        - Single Pulse (One-Shot) mode
    - Module Reset on ECCP Special Event Trigger

   The FOSC clock source should not be used with the ECCP capture/compare features, 
   select one of the other timer clocking options.
   Timer Gate mode: count when an external source is at High State !!!
 
 
 * TxCON: TIMER(2/3) AND TIMER(4/5) CONTROL REGISTER
  bit 3 T32: 32-Bit Timer Mode Select bit(1)
                1 = Timer2/3 or Timer4/5 form a single 32-bit timer
                0 = Timer2/3 or Timer4/5 act as two 16-bit timers
  ....... the same options of Timer3 & 5 CONTROL REGISTERS 
  IN 32Bit MODE ONLY TIMER2 & TIMER4 CONTROL REGISTER ARE USED !!!!!
  
 

 * TyCON: TIMER3 AND TIMER5 CONTROL REGISTER 
 * ------------------------------------------

 * bit 1 TCS: Timery Clock Source Select bit(1)
            1 = External clock is from the T3CK pin (on the rising edge)
            0 = Internal clock (FOSC/2)

 * bit 5-4 TCKPS<1:0>: Timery Input Clock Prescale Select bits(1)
            11 = 1:256
            10 = 1:64
            01 = 1:8
            00 = 1:1
 *
 * bit 13 TSIDL: Timery Stop in Idle Mode bit(1)
        1 = Discontinues module operation when device enters Idle mode
        0 = Continues module operation in Idle mode

 * bit 6 TGATE: Timery Gated Time Accumulation Enable bit(1)
        When TCS = 1:   This bit is ignored.
        When TCS = 0:
            1 = Gated time accumulation is enabled
            0 = Gated time accumulation is disabled

 * bit 15 TON: Timery On bit(1)
        1 = Starts 16-bit Timery
        0 = Stops 16-bit Timery

 *Note 1: When 32-bit operation is enabled (TxCON<3> = 1), 
          these bits have no effect on Timery operation. 
          All timer functions are set through the TxCON register. (even timer) 
 
 TMR3: Timer Register 16 Bit
   
 PR3:  Period register  16 Bit


 * 
 * 
 */