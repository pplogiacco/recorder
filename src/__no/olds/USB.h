
#ifndef _USB_H
#define _USB_H

#include <xc.h>          // processor files 
#include <stddef.h>
#include <stdbool.h>



#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

    extern void USB_onReceive(); // call-back

    void USB_Enable(void); // UART2

    uint8_t USB_Read(char* buf, uint16_t blen);

    void USB_Write(char byte);
    void USB_WriteBuffer(const char* cbuf, uint16_t blen); // change in bytes

    inline void USB_Disable(void);






#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif  // _UART2_H


/*___PIC24Fv32KA304__________________________________________________________________________________
   
UxMODE: UARTx MODE REGISTER
---------------------------
   
bit 15 UARTEN: UARTx Enable bit
               1 = UARTx is enabled: All UARTx pins are controlled by UARTx, as defined by UEN<1:0>
               0 = UARTx is disabled: All UARTx pins are controlled by port latches and 
                   power consumption is minimal

bit 13 USIDL: UARTx Stop in Idle Mode bit
               1 = Discontinues module operation when the device enters Idle mode
               0 = Continues module operation in Idle mode

bit 12 IREN: IrDA® Encoder and Decoder Enable bit(1)
               1 = IrDA encoder and decoder are enabled
               0 = IrDA encoder and decoder are disabled

bit 11 RTSMD: Mode Selection for UxRTS Pin bit
               1 = UxRTS pin is in Simplex mode
               0 = UxRTS pin is in Flow Control mode

bit 9-8 UEN<1:0>: UARTx Enable bits(2)
               11 = UxTX, UxRX and UxBCLK pins are enabled and used; UxCTS pin is controlled by port latches
               10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
               01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin is controlled by port latches
               00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/UxBCLK pins are controlled by port
                    latches

bit 7 WAKE: Wake-up on Start Bit Detect During Sleep Mode Enable bit
                1 = UARTx will continue to sample the UxRX pin; interrupt is generated on the falling edge, bit is
                    cleared in hardware on the following rising edge
                0 = No wake-up is enabled

bit 6 LPBACK: UARTx Loopback Mode Select bit
                1 = Enables Loopback mode
                0 = Loopback mode is disabled

bit 5 ABAUD: Auto-Baud Enable bit
                1 = Enables baud rate measurement on the next character ? requires reception of a Sync field (55h);
                    cleared in hardware upon completion
                0 = Baud rate measurement is disabled or completed

bit 4 RXINV: Receive Polarity Inversion bit
                1 = UxRX Idle state is ?0?
                0 = UxRX Idle state is ?1?
     
bit 3 BRGH: High Baud Rate Enable bit
                1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
                0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode)

it 2-1 PDSEL<1:0>: Parity and Data Selection bits
                11 = 9-bit data, no parity
                10 = 8-bit data, odd parity
                01 = 8-bit data, even parity
                00 = 8-bit data, no parity
 * 
bit 0 STSEL: Stop Bit Selection bit
                1 = Two Stop bits
                0 = One Stop bit

 * REGISTER 18-1: R (CONTINUED)
Note 1: This feature is is only available for the 16x BRG mode (BRGH = 0).
     
     
     
 * 
 * REGISTER 18-2: UxSTA: UARTx STATUS AND CONTROL REGISTER
 * 

bit 15,13 UTXISEL<1:0>: UARTx Transmission Interrupt Mode Selection bits
        11 = Reserved; do not use
        10 = Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the
            transmit buffer becomes empty
        01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit operations
            are completed
        00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies there is at least
            one character open in the transmit buffer)

bit 14 UTXINV: IrDA® Encoder Transmit Polarity Inversion bit
        If IREN = 0:
            1 = UxTX Idle ?0?
            0 = UxTX Idle ?1?
        If IREN = 1:
            1 = UxTX Idle ?1?
            0 = UxTX Idle ?0?

bit 11 UTXBRK: UARTx Transmit Break bit
            1 = Sends Sync Break on next transmission ? Start bit, followed by twelve ?0? bits, followed by Stop bit;
                cleared by hardware upon completion
            0 = Sync Break transmission is disabled or completed

bit 10 UTXEN: UARTx Transmit Enable bit
    1 = Transmit is enabled; UxTX pin is controlled by UARTx
    0 = Transmit is disabled; any pending transmission is aborted and the buffer is reset. UxTX pin is
        controlled by the PORT register.
 
bit 9 UTXBF: UARTx Transmit Buffer Full Status bit (read-only)
    1 = Transmit buffer is full
    0 = Transmit buffer is not full, at least one more character can be written
  
bit 8 TRMT: Transmit Shift Register Empty bit (read-only)
    1 = Transmit Shift Register is empty and the transmit buffer is empty (the last transmission has
        completed)
    0 = Transmit Shift Register is not empty; a transmission is in progress or queued
 
bit 7-6 URXISEL<1:0>: UARTx Receive Interrupt Mode Selection bits
        11 = Interrupt is set on a RSR transfer, making the receive buffer full (i.e., has 4 data characters)
    10 = Interrupt is set on a RSR transfer, making the receive buffer 3/4 full (i.e., has 3 data characters)
    0x = Interrupt is set when any character is received and transferred from the RSR to the receive buffer;
        receive buffer has one or more characters.
  
bit 5 ADDEN: Address Character Detect bit (bit 8 of received data = 1)
    1 = Address Detect mode is enabled; if 9-bit mode is not selected, this does not take effect
    0 = Address Detect mode is disabled

bit 4 RIDLE: Receiver Idle bit (read-only)
    1 = Receiver is Idle
    0 = Receiver is active

bit 3 PERR: Parity Error Status bit (read-only)
    1 = Parity error has been detected for the current character (character at the top of the receive FIFO)
    0 = Parity error has not been detected

bit 2 FERR: Framing Error Status bit (read-only)
    1 = Framing error has been detected for the current character (character at the top of the receive FIFO)
    0 = Framing error has not been detected

bit 1 OERR: Receive Buffer Overrun Error Status bit (clear/read-only)
    1 = Receive buffer has overflowed
    0 = Receive buffer has not overflowed (clearing a previously set OERR bit (1 ? 0 transition) will reset
        the receiver buffer and the RSR to the empty state)

bit 0 URXDA: UARTx Receive Buffer Data Available bit (read-only)
    1 = Receive buffer has data; at least one more characters can be read
    0 = Receive buffer is empty
  

 
UxTXREG: UARTx TRANSMIT REGISTER
U-x U-x U-x U-x U-x U
 * 
 * bit 15-9 Unimplemented: Read as ?0?
bit 8 UTX8: UARTx Data of the Transmitted Character bit (in 9-bit mode)
bit 7-0 UTX<7:0>: UARTx Data of the Transmitted Character bits
 * 
 * 
UxRXREG: UARTx RECEIVE REGISTER
 * 
bit 15-9 Unimplemented: Read as ?0?
bit 8 URX8: UARTx Data of the Received Character bit (in 9-bit mode)
bit 7-0 URX<7:0>: UARTx Data of the Received Character bits
 * 
 * 
 */