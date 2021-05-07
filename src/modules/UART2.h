
#ifndef _UART2_H
#define _UART2_H

/**
 Section: Included Files
*/

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
/* -------------------------------------------------------------------------- 
    This routine initializes the UART driver instance for : 2 index.
    This routine must be called before any other UART routine is called.
*/
void UART2_Initialize(void);

/* --------------------------------------------------------------------------
    This routine reads a byte of data from the UART2.
*/
uint8_t UART2_Read( void);

/* --------------------------------------------------------------------------
    This routine writes a byte of data to the UART2.
*/
void UART2_Write( uint8_t byte);


/* --------------------------------------------------------------------------
    Indicates of there is data available to read.
*/
bool UART2_IsRxReady(void);

/* --------------------------------------------------------------------------
    Indicates if a byte can be written.
*/
bool UART2_IsTxReady(void);

/* --------------------------------------------------------------------------
    Indicates if all bytes have been transferred.
 */
bool UART2_IsTxDone(void);

/* --------------------------------------------------------------------------
    This routine assigns a function pointer with a transmit callback address.
*/
void UART2_SetTxInterruptHandler(void (* interruptHandler)(void));

/* --------------------------------------------------------------------------
    This routine is a transmit callback function.
*/
void UART2_Transmit_CallBack(void);

/* --------------------------------------------------------------------------
    This routine assigns a function pointer with a receive callback address.
*/
void UART2_SetRxInterruptHandler(void (* interruptHandler)(void));

/* --------------------------------------------------------------------------
    This routine is a receive callback function.
*/
void UART2_Receive_CallBack(void);

void UART2_Enable(void);

void UART2_Disable(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
    
#endif  // _UART2_H