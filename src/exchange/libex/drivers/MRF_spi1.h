#ifndef _MRFSPI1_H
#define _MRFSPI1_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
// #include "../device.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

    // device.h
    //#define MRF24_INT   _RB7    // Shared USB_WK
    //#define MRF24_SS    _RB15   // Chip select
    //#define MRF24_SS_SetHigh()   (_LATB15 = 1)
    //#define MRF24_SS_SetLow()    (_LATB15 = 0)    
    //#define MRF24_SS_SetDigitalInput()  (_TRISB15 = 1)
    //#define SS1_SetHigh()          (_LATB15 = 1)       
    //#define SS1_SetLow()           (_LATB15 = 0)
    //#define SS1_SetDigitalInput()  (_TRISB15 = 1)

#define SPI1_DUMMY_DATA 0x0
#define SPI1_FIFO_FILL_LIMIT 0x8

    //Check to make sure that the FIFO limit does not exceed the maximum allowed limit of 8
#if (SPI1_FIFO_FILL_LIMIT > 8)
#define SPI1_FIFO_FILL_LIMIT 8
#endif

    typedef enum {
        SPI1_SHIFT_REGISTER_EMPTY = 1 << 7,
        SPI1_RECEIVE_OVERFLOW = 1 << 6,
        SPI1_RECEIVE_FIFO_EMPTY = 1 << 5,
        SPI1_TRANSMIT_BUFFER_FULL = 1 << 1,
        SPI1_RECEIVE_BUFFER_FULL = 1 << 0
    } SPI1_STATUS;

    typedef enum {
        SPI1_DRIVER_TRANSFER_MODE_8BIT = 0,
        SPI1_DRIVER_TRANSFER_MODE_16BIT = 1,
        SPI1_DRIVER_TRANSFER_MODE_32BIT = 2,
    } SPI1_TRANSFER_MODE;

    void SPI1_Initialize(void);


    uint8_t SPI1_Exchange8bit(uint8_t data);

    uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived);

    SPI1_STATUS SPI1_StatusGet(void);

    void SPI1_Disable();

#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif //_MRFSPI1_H

