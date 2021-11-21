#ifndef _UART2_H
#define _UART2_H

#include <stdbool.h>
#include <stdint.h>

    // void UART2_Initialize(void);

    void UART2_Enable(void);
    void UART2_Disable(void);

    uint8_t UART2_Read(void);

    void UART2_Write(uint8_t byte);

    bool UART2_IsTxReady(void);

    bool UART2_IsTxDone(void);

    //    void UART2_SetTxInterruptHandler(void (* interruptHandler)(void));
    //    void UART2_Transmit_CallBack(void);
    //    void UART2_SetRxInterruptHandler(void (* interruptHandler)(void));
    //    void UART2_Receive_CallBack(void);

    
    // --------- LibEx
    bool UART2_IsRxReady(void);
    uint16_t UART2_RxBuffer(uint8_t *buff, uint16_t maxSize);
    bool UART2_TxBuffer(uint8_t *buff, uint16_t size);
    void UART2_Flush();
    // -------------


#endif  // _UART2_H