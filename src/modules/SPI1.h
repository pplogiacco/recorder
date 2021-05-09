#ifndef _SPI1_H
#define _SPI1_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../device.h"

//  SPI Mode     CKP  CKE
//  0,0 (0)       0    1
//  0,1 (1)       0    0
//  1,0 (2)       1    1
//  1,1 (3)       1    0

typedef enum {
    MODE0,
    MODE1,
    MODE2,
    MODE3
} SPI_MODE;

typedef enum {
    SPI_100KHZ = 0x4F, // Bit rate 100Khz (Fosc 32Mhz)  
    SPI_2MHZ = 0x03 // Bit rate 2Mhz (Fosc 32Mhz)
} SPI_BRATE;

//#define SPI_100KHZ  0x4F 
//#define SPI_2MHZ    0x03 

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

void SPI1_Enable(SPI_MODE mode, SPI_BRATE speed);

uint8_t SPI1_Exchange8bit(uint8_t data);

uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived);

SPI1_STATUS SPI1_StatusGet(void);

void SPI1_Disable();


#endif //SPI1_H

