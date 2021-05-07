/*
 * File:   utils.h
 * Author: plogiacco
 *
 * Created on October 26, 2020, 12:15 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#include "device.h"     // FCY
// #define SYSCLK _FOSC_
#define FCY _FCY_               // Required for __delayXXX() to work

#define CLOCK_SystemFrequencyGet()        (_FOSC_)
#define CLOCK_PeripheralFrequencyGet()    (_FOSC_)
#define CLOCK_InstructionFrequencyGet()   (_FCY_)

#include <libpic30.h>   // __delay_ms, require FCY
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

#define __delay(x) __delay_ms(x)
#define __noop(x) __delay32(x) // delay of the requested number of cycles (min 12)


// Timeout utils
void Timeout_SetCallBack(void (* NewCallBack)(void));
void Timeout_Set(uint16_t nsec, uint16_t nms);
void Timeout_Unset(void);
bool isTimeout(void);

#endif	/* UTILS_H */

