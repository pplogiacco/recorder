/*******************************************************************************
 * PIC24FV32KA301/302 (28 Pins)                                                 *
 ********************************************************************************
Module  PIN Signal                      BUS Function
  
PROG 	1 	MCLR Reset  
        2 	AN0                         (S) ADA2200 IN-P 
        3	AN1                         (S) ADA2200 IN-N 
USART2	4	U2RX/RB0/PGD1/RP0               MCP2221
        5	U2TX/RB1/PGC1/RP1               MCP2221
        6	AN4/T5CK/T4CK/RB2           (S) Wind speed sensor
        7	AN5/RB3                     (S) Temperature sensor
POWER	8	Vss
        9	OSCI/AN13/CLKI/CN30/RA2     (S) MRF Chip Select
        10  OSCO/AN14/CLKO/CN29/RA3     (S) Batt Level
        11  SOSCI/AN15/U2RTS/CN1/RB4    (A) ??
        12  SOSCO/SCLKI/U2CTS/CN0/RA4   (S)  
POWER   13  Vdd
        14  RB5 Programming
        15  RB6 Programming
        16  INT0/CN23/RB7/OC1           (S)  Wake-up USB / (diode) MRF Int
I2C1    17  SCL1/C3OUT/CTED10/CN22/RB8
        18  SDA1/T1CK/IC2/CN21/RB9
        19  SDI2/IC1/CTED3/CN9/RA7           POWER ENA 3127 ( verif. Hi/VCAP )*
        20  VCAP
        23  RB12                        (S)  ADA2200 Chip Select
SPI1    21  SDI (S)
        22  SCLK (S)
        24  SDO
        25  AN10/INT1/RB14              (S) Encoder B
        26  AN9/C3INA/T3CK/T2CK/SS1     (S) ADA2200 Synco / Encoder A
POWER   27  Vss
        28  Vdd
 *******************************************************************************/


#ifndef HARDWARE_H
#define	HARDWARE_H

// Firmware / Hardware

#ifdef __HWDONGLE
//
#define __UI
#define __USB
#define __MRF24
#define __SDCARD
#undef __SENSOR_BOARD

#elif defined(__HWDEVICE)
//
#undef __UI
#define __USB
#define __MRF24
#undef __SDCARD
#define __SENSOR_BOARD

#endif  



#if defined(__SENSOR_BOARD) // ____________________________________SENSOR_BOARD 

#define __SPI1  // Signal Bus
#define __I2C1

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__)) || defined(__PIC24FJ256GA702__)

#define ADA_SS                      _RB12  // RB14 INT1/AN10
#define ADA_SS_SetHigh()            _LATB12=1
#define ADA_SS_SetLow()             _LATB12=0
#define ADA_SS_SetDigitalOutput()   _TRISB12=0

#define AV_SYN_SetDigital()         _ANSB15=0  // Digital
#define AV_SYN_SetDigitalInput()    _TRISB15=1 // Input T3CK/RB15 (SYNCO)

#define AV_INP_SetAnalog()          _ANSA0=1   // RA0 AN0 (2 DIP20) VRef+
#define AV_INP_SetAnalogInput()     _TRISA0=1     
#define AV_INN_SetAnalog()          _ANSA1=1   //  RA1 AN1 (3 DIP20) VRef-
#define AV_INN_SetAnalogInput()     _TRISA1=1 

#define ET_IN     _RB3  // AN5/C1INA/C2INC/SCL2/CN7/RB3 (7)
#define ET_IN_SetAnalogInput()    (_TRISB3 = 1 ) 
#define ET_IN_SetAnalog()         (_ANSB3 = 1 ) 

#define WS_IN     _RB2  // AN4/C1INB/C2IND/SDA2/T5CK/T4CK/U1RX/CTED13/CN6/RB2 (6)
#define WS_IN_SetDigital()         (_ANSB2 = 0 ) 
#define WS_IN_SetDigitalInput()    (_TRISB2 = 1 ) 
#endif

#endif // __SENSOR_BOARD


#ifdef __UI // ______________________________________________________________UI
//
#define IO_LED1                     _AN0    
#define IO_LED1_On()                (_LATA0 = 1)
#define IO_LED1_Off()               (_LATA0 = 0)
#define IO_LED1_Toggle()            (_LATA0 ^= 1)
#define IO_LED1_SetDigitalOut()     ()

#define IO_LED2                     _RB2   
#define IO_LED2_On()                (_LATB2 = 1)
#define IO_LED2_Off()               (_LATB2 = 0)
#define IO_LED2_Toggle()            (_LATB2 ^= 1)
#define IO_LED2_SetDigitalOut()     ()

#define IO_SWC1        3   // AN1
#define IO_SWC1_Value()             (!_RA1)
#define IO_SWC1_SetDigitalInput()   ()

#else

#define IO_LED1
#define IO_LED1_On()
#define IO_LED1_Off()
#define IO_LED1_Toggle()
#define IO_LED1_SetDigitalOut()     
#define IO_LED2
#define IO_LED2_On()
#define IO_LED2_Off()
#define IO_LED2_Toggle()
#define IO_LED2_SetDigitalOut()  
#define IO_SWC1
#define IO_SWC1_Value()  0
#define IO_SWC1_SetDigitalInput()
#endif

#ifdef __USB  // ___________________________________________________________USB

#define USB_WK_SetDigitalInput()  _TRISB7 = 1
#define __UART2

#ifdef __HWDEVICE

#ifdef __NOUSB
#define USB_WK_Value()   (0)
#else
#define USB_WK_Value() ( _RB7 )
#endif

#else // HWDONGLE
#define USB_WK_Value() (1)
#endif
#endif // __USB


#if defined(__MRF24) // __________________________________________________MRF24

#define __SPI1
#define MRF24_INT   _RB7   // Shared USB_WK

#if defined(__HWDEVICE)
#define MRF24_SS    _RA2   // Chip select
#define MRF24_SS_SetHigh()   (_LATA2 = 1)
#define MRF24_SS_SetLow()    (_LATA2 = 0)
#define MRF24_SS_SetDigital()  (_ANSA2 = 0)
#define MRF24_SS_SetDigitalOutput()  (_TRISA2 = 0)

#elif defined(__HWDONGLE)
#define MRF24_SS    _RB15   // Chip select
#define MRF24_SS_SetHigh()          (_LATB15 = 1)
#define MRF24_SS_SetLow()           (_LATB15 = 0)
#define MRF24_SS_SetDigital()   // No settings       
#define MRF24_SS_SetDigitalOutput()  (_TRISB15 = 0)
#endif
#endif // __MRF24


/*
#ifdef __UART2
#define UART2_TX      _RB0   // UART2.TX
#define UART2_RX      _RB1   // UART2.RX

#endif

#ifdef __SPI1

#endif

#ifdef __I2C1
//
#define CARD_SCL    17   // I2C1.SCL
#define CARD_SDA    18   // I2C1.SDA
#endif
*/


#endif // HARDWARE_H


