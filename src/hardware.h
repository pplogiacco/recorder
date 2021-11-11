/*******************************************************************************
 * PIC24FJ256GA702 (28 Pins) ( 705 Family )                                    *
 *******************************************************************************
Module  PIN Signal                      BUS Function
  
PROG  	 1 	MCLR Reset  
LVDT+    2 	AN0                         (S) ADA2200 IN-P 
BATLV    3	AN1                         (S) Batt Level   ( EX.ADA2200 IN-N) 
USART2	 4	U2RX/RB0/PGD1/RP0               MCP2221
         5	U2TX/RB1/PGC1/RP1               MCP2221
         6	AN4/T5CK/T4CK/RB2           (S) Wind speed sensor
         7	AN5/RB3                     (S) Temperature sensor
POWER	 8	Vss
         9	OSCI/AN13/CLKI/CN30/RA2     (S) MRF Chip Select
        10  OSCO/RA3                    (S) ??
        11  SOSCI/AN15/U2RTS/CN1/RB4    (A) High SHUTDOWN LTC3127 (main power)
        12  SOSCO/SCLKI/U2CTS/CN0/RA4   (S) CS Flash ( SST26VF064B )
POWER   13  Vdd
        14  RB5 Programming
        15  RB6 Programming
        16  INT0/CN23/RB7/OC1           (S)  Wake-up USB / (diode) MRF Int
I2C1    17  SCL1/C3OUT/CTED10/CN22/RB8
        18  SDA1/T1CK/IC2/CN21/RB9
        19  SDI2/IC1/CTED3/CN9/RA7           POWER ENA 3127 ( verif. Hi/VCAP )*
        20  VCAP
        23  RB12                        (S)  ADA2200 Chip Select
SPI1    21  SDI/RP10                    (S) 
        22  SCLK (S)
        24  SDO
        25  AN10/INT1/RB14              (S) Encoder B
        26  AN9/C3INA/T3CK/T2CK/SS1     (S) High Enable LDO AD151
POWER   27  Vss
        28  Vdd
 *******************************************************************************/

/*******************************************************************************
    Function        ANSx    TRISx   Comments
    Analog Input    1       1       It is recommended to keep ANSx = 1.
    Analog Output   1       1       It is recommended to keep ANSx = 1.
    Digital Input   0       1       Wait one cycle Nop() after configuring
    Digital Output  0       0       Make sure to disable the analog output !!!
 *******************************************************************************/

#ifndef HARDWARE_H
#define	HARDWARE_H

// Firmware / Hardware
// Hardware:
//#define __HWDONGLE
#define __HWDEVICE

// #define __HWDEVICE_V1
// Logic Board V1 + Sensor Board + RF Board
// #define __BOARD_V2
// Logic Board V2 + RF Board

#define __USE_ADG715

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


// Battery Level
#define __BOARD_V2

#ifdef __BOARD_V2

#define BAT_LV_SetAnalogInput()         { _TRISA3=1;  _ANSA3=1; }   // AN3
#define BAT_LV_ADC_CH0SA                5    // S/H+ Input A3

// Default power LTC Buck/Booster
#define PW_SWC_SetDigitalOutputLow()    { _TRISB4=0;  _LATB4=0; } // LTC3127 On
#define PW_SWC_SetHigh()                { _LATB4=1; }          // Off   
#define PW_SWC_SetLow()                 { _LATB4=0; }          // On   

// Low power ADP LDO
#define PW_ADP_SetDigitalOutputLow()    { _TRISB15=0;  _ANSB15=0; _LATB15=0; }   // AN3
#define PW_ADP_SetHigh()                { _LATB4=1; }  // ADP151 On      
#define PW_ADP_SetLow()                 { _LATB4=0; }  // ADP151 Off

#define Device_Power_Save()             { PW_SWC_SetHigh(); PW_ADP_SetHigh(); }                       
#define Device_Power_Default()          { PW_SWC_SetLow(); PW_ADP_SetLow(); }

#define ADA_IN_SetAnalogInput()         { _TRISA1=1;  _ANSA1=1; }   // AN1 
#define ADA_IN_ADC_CH0SA                1     // S/H+ Input A 

#else

#define BAT_LV_SetAnalogInput()   { _TRISA1=1;  _ANSA1=1; }   // AN0
#define BAT_LV_ADC_CH0SA          1    // S/H+ Input A (0=AN0,1=AN1)
#define ADA_IN_SetAnalogInput()   { _TRISA0=1;  _ANSA0=1; }   // AN0 
#define ADA_IN_ADC_CH0SA          0     // S/H+ Input A 

#endif


// Flash memory ( SPI )
#define SST26_SS_SetDigitalOutputHigh()  {  _TRISA4 = 0; _LATA4 = 1; }
#define SST26_SS_SetHigh()   (_LATA4 = 1)
#define SST26_SS_SetLow()    (_LATA4 = 0)

#endif  // __HWDEVICE





#if defined(__SENSOR_BOARD) // ____________________________________SENSOR_BOARD 

#define __SPI1  // Signal Bus
#define __I2C1

#if defined(__PIC24FJ256GA702__)

#define ADA_SS                      _RB12  // RB14 INT1/AN10
#define ADA_SS_SetHigh()            _LATB12=1
#define ADA_SS_SetLow()             _LATB12=0
#define ADA_SS_SetDigitalOutput()   _TRISB12=0

#define AV_SYN_SetDigital()         _ANSB15=0  // Digital
#define AV_SYN_SetDigitalInput()    _TRISB15=1 // Input T3CK/RB15 (SYNCO)

#define AV_IN_SetAnalogInput()      ADA_IN_SetAnalogInput()

#define ET_IN                       _RB3  // AN5/C1INA/C2INC/SCL2/CN7/RB3 (7)
#define ET_IN_SetAnalogInput()      {_TRISB3 = 1; _ANSB3 = 1; }

#define WS_IN                       _RB2  // AN4/C1INB/C2IND/SDA2/T5CK/T4CK/U1RX/CTED13/CN6/RB2 (6)
#define WS_IN_SetDigitalInputLow()  { _TRISB2 = 1; _ANSB2 = 0; _LATB2 = 0;} 
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

#define __UART2
#define USB_WK_SetDigitalInputLow() {  _TRISB7 = 1; _LATB7 = 0;  } // IOCPDBbits.CNPDB7 = 1;
#define USB_Status _RB7 

#endif // __USB

#ifdef __UART2     // __________________________________________________UART2

#define UART2_TX_SetDigitalOutputHigh() { _ANSB1 = 0; _TRISB1 = 1; }
#define UART2_RX_SetDigitalInput()      { _ANSB0 = 0; _TRISB0 = 0; _LATB0 = 1; }
#endif


#if defined(__MRF24) // __________________________________________________MRF24
#define MRF24_INT             _RB7    // Shared USB_WK
#define MRF24_SS             (_RA2)   // Chip select
#define MRF24_SS_SetHigh()   (_LATA2 = 1)
#define MRF24_SS_SetLow()    (_LATA2 = 0)
#define MRF24_SS_SetDigitalOutputHigh()  {_TRISA2 = 0; _ANSA2 = 0; _LATA2 = 1; } 
#endif // __MRF24





#endif // HARDWARE_H



/*******************************************************************************
 * PIC24FV32KA301/302 (28 Pins)                                                *
 *******************************************************************************
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
        12  SOSCO/SCLKI/U2CTS/CN0/RA4   (S) SST226 CS
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