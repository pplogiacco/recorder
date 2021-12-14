/*
 * File:   utils.h
 * Author: plogiacco
 *
 * Created on October 26, 2020, 12:15 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#include "device.h"     // FCY

//#include <libpic30.h>   // __delay_ms, require FCY
#include <stdio.h>      // printf
// Bitwise
//
#define _BV(n)  (1 << (n))
#define _LSB(w) ((uint8_t) ((w) & 0xff))
#define _MSB(w)  ((uint8_t) ((w) >> 8U))
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define pinToggle(pin) PINB ^= 1UL _BV()



// Timeout utils
void Timeout_SetCallBack(void (* NewCallBack)(void));
void Timeout_Set(uint16_t nsec, uint16_t nms);
void Timeout_Unset(void);
bool isTimeout(void);


//// The File I/O library requires the user to define the system clock frequency (Hz)
////#include "../mcc_generated_files/clock.h"
//#define _XTAL_FREQ  32000000UL              // 32Mhz Internal (FRC)
////#define _FOSC_      32000000UL
////#define _FCY_       16000000UL              // ( _FOSC_ / 2 )
//#define SYS_CLK_FrequencySystemGet()        32000000UL

#define SYS_CLK_FrequencyPeripheralGet()    16000000UL
#define SYS_CLK_FrequencyInstructionGet()   16000000UL // ( _FOSC_ / 2 )


#define _delay_us(x)             \
    {                               \
        unsigned long _dcnt;        \
        _dcnt=x*((unsigned long)(0.00001/(1.0/SYS_CLK_FrequencyInstructionGet())/6));   \
        while(_dcnt--);             \
    }

#define _delay_ms(x)             \
    {                               \
        unsigned long _dcnt;        \
        unsigned long _ms;          \
        _ms = x;                    \
        while(_ms)                  \
        {                           \
            _dcnt=((unsigned long)(0.001/(1.0/SYS_CLK_FrequencyInstructionGet())/6));    \
            while(_dcnt--);         \
            _ms--;                  \
        }                           \
    }


#define __delay(x) _delay_ms(x)
#define __noop(x) __delay32(x) // delay of the requested number of cycles (min 12)

#define __clearWDT()  ClrWdt()
#define _bs8(n) (1U<<n)

//
//#define FCY SYS_CLK_FrequencyInstructionGet()  // Required for __delayXXX() to work
//#include <libpic30.h>   // __delay_ms, require FCY
//
//#define __delay(x)  __delay_ms(x)
//
//
//#include <stdio.h>      // printf
//#define dprint(x)       printf(x)
//#define _printf(x,y)       printf(x,y)


//typedef union
//{
//    uint16_t V;
//    uint8_t v[2];
//    struct
//    {
//        uint8_t LB;
//        uint8_t HB;
//    } byte;
//} word_t_VAL;

#endif	/* UTILS_H */

