#include "xc.h"
#include <stdbool.h>
#include "test.h"
#include "dev-config.h"

#ifndef DEVICE_H
#define	DEVICE_H

//------------------------------------------------------------------------------
#define __DEVICE_DIN  100100102    // HWVAMP1K (Paolo)
#define __DEVICE_VER    10102     // ver-rel-bld
#define __DEVICE_USB_SR 0310      // Usb enumeration  
//------------------------------------------------------------------------------
#include "hardware.h"
//------------------------------------------------------------------------------

#define SYS_CLK_FrequencySystemGet()         Device_FrequencySystemGet()    // _FOSC_
#define SYS_CLK_FrequencyPeripheralGet()    (Device_FrequencySystemGet()/2)
#define SYS_CLK_FrequencyInstructionGet()   (Device_FrequencySystemGet()/2)
#define FCY                                 (SYS_CLK_FrequencyInstructionGet())

#define Device_IsUsbConnected() USB_Status


typedef enum { // 
    STARTUP, // Switch-on state
    SAMPLING, // Start sampling cycle
    EXCHANGE, // Initiate exchange mode
    EXCHANGE_RT,
    TOSLEEP
} devicestate_t;

//----------------------------------------------------------------------------//

typedef enum {
    SYS_OFF, // LTC shut-down
    SYS_BOOT,
    SYS_IDLE,
    SYS_SLEEP,
    SYS_DSLEEP,
    SYS_DEFAULT, // I2C1,TMR1
    SYS_ON_CHECK, // I2C1,TMR1, ADC
    SYS_ON_EXCHANGE, // I2C1,TMR1, UART
    SYS_ON_SAMP_WST, // I2C1,TMR1, TMR4, ADC
    SYS_ON_SAMP_ADA, // I2C1,TMR1, TMR3, SPI1, ADC
    SYS_ON_SAMP_SS // ND !!!!
} runlevel_t;

bool Device_SwitchSys(runlevel_t rlv); // clock, power, powered modules, pins...

// ------------------------------------------------------
// Testing - CALLED BY "switchSys()" / "getStatus()"

typedef enum {
    SM_RUN,
    SM_IDLE,
    SM_SLEEP,
    SM_DSLEEP,
} sysmode_t;

typedef enum {
    CK_DEFAULT,
    CK_SLOW,
    CK_FAST
} sysclock_t;

void Device_Initialize();

uint16_t Device_CheckHwReset(void); // Cheack reason of reboot 

void Device_SwitchClock(sysclock_t ck);
unsigned long inline Device_FrequencySystemGet();


//----------------------------------------------------------------------------//
#ifdef __USE_ADG715

#define ADG_ADDRESS  0b10010000
#define PW_OFF      (0b00000000)
#define PW_MRF      (0b00001000)  // OK 
#define PW_WST      (0b00000100)  // OK 
#define PW_ADA      (0b11100010)  // OK ( ADA assorbe lo stesso !!)
#define PW_RS1      (0b00000001)  // Battery measurement 
//#define PW_RS2     (0b00000000) 
#define PW_ENC      (0b00010000)

#else  // Use ADG729 

#define ADG_ADDRESS      0b10011000
#define PW_OFF     (0b000000)
#define PW_MRF     (0b000001)  // OK 
#define PW_WST     (0b000010)  // OK 
#define PW_ADA     (0b000100)  // OK ( ADA assorbe lo stesso !!)
#define PW_RS1     (0b001000)
#define PW_RS2     (0b000000)
#define PW_ENC     (0b000000)


#endif


void Device_SwitchADG(uint8_t reg);

uint16_t Device_GetBatteryLevel();

#define __clearWDT()  ClrWdt()
#define _bs8(n) (1U<<n)

// PIC24Fx32KA304 Family Power Managment Register
//#define PMD3_RTCCMD  _bs(6) // Bit 9
//#define PMD4_EEMD    _bs(6) // Bit 4
//#define PMD4_HLVDM   _bs(6) // Bit 4
//
//#define PMD1_U2MD   _bs(6)  // Bit 6
//#define PMD1_I2C1MD _bs(6)  // Bit 7
//#define PMD1_T1MD   _bs(6)  // Bit 11
//#define PMD1_T2MD   _bs(6)  // Bit 12
//#define PMD1_T3MD   _bs(6)  // Bit 13
//#define PMD1_T5MD   _bs(6)  // Bit 15
//#define PMD1_SPI1MD _bs(6)  // Bit 3
//#define PMD1_ADC1MD _bs(6)  // Bit 0




#endif