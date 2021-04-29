#include "xc.h"
#include <stdbool.h>
#include "dev-config.h"

#ifndef DEVICE_H
#define	DEVICE_H

//------------------------------------------------------------------------------
#define __DEVICE_DIN  100200300    // HWVAMP1K (Paolo)
//#define __DEVICE_DIN 333222111  // HWDONGLE (Marco)
//#define __DEVICE_DIN 999888777  // HWDONGLE (Marcello)

#define __DEVICE_VER    1001001   // ver-rel-bld
#define __DEVICE_USB_SR 0310      // Usb enumeration  
//------------------------------------------------------------------------------

// Hardware:
// ---------
//#define __HWDONGLE
#define __HWDEVICE
//#define __HWDEVICE_V2_302

// Firmware:
// ---------

// Dongle:
//#define __DONGLE
//#define __DONGLE_PASSTHRU
//#define __DONGLE_VAMP1K    // Simulate Device ( Demo Signal only !!)
//#define __DONGLE_VAMP1K_AUTOSEND // Simulate Device ( Demo Signal only !!)
//#define __DONGLE_TEST_FILE

// Recorder:
#define __VAMP1K

//#define __NOFLASH    // Use RAM to store config
//#define __NOUSB      // Force to use RF ( USB not connect )
#endif



//#define __VAMP1K_TEST
#ifdef __VAMP1K_TEST

// Testing
//#define __VAMP1K_TEST_HW                      // Perform hardware test
//#define __VAMP1K_TEST_RESET
//#define __VAMP1K_TEST_TIMER
//#define __VAMP1K_TEST_CONFIG 
//#define __VAMP1K_TEST_ADG
//#define __VAMP1K_TEST_USB


#define __VAMP1K_TEST_AV_printf    // Test Acquire/ADC HW

//#define __VAMP1K_TEST_measurement_printf  // Test Measurement format 
//#define __VAMP1K_TEST_measurement_DATAVIS    // send ADC to serial mc datavis
//#define __AV0NVM     // Save samples in flash
#endif


//------------------------------------------------------------------------------
#include "hardware.h"

// The File I/O library requires the user to define the system clock frequency (Hz)
#define _XTAL_FREQ  32000000UL              // 32Mhz Internal (FRC)
#define _FOSC_      32000000UL
#define _FCY_       16000000UL              // ( _FOSC_ / 2 )
#define SYS_CLK_FrequencySystemGet()        _FOSC_
#define SYS_CLK_FrequencyPeripheralGet()    _FOSC_
#define SYS_CLK_FrequencyInstructionGet()   _FCY_


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
    CK_SLOW,
    CK_DEFAULT,
    CK_FAST
} sysclock_t;

void Device_SwitchPins();
void Device_SwitchClock(sysclock_t ck);

//----------------------------------------------------------------------------//
//void Device_Boot(void);
//void Device_Hybernate();
//void Device_Resume();

uint16_t Device_GetPowerLevel();

#define ADG729_ADDRESS      0b10011000

#define PW_ENC     (0)
#define PW_MRF     (1)
#define PW_WST     (2)
#define PW_ADA     (3)
#define PW_RS1     (6)
#define PW_RS2     (7)

uint8_t Device_SwitchADG(uint8_t reg);

#define __clearWDT()  ClrWdt()

#define _bs8(n) (1U<<n)

// PIC24Fx32KA304 Family Power Managment Register
#define PMD3_RTCCMD  _bs(6) // Bit 9
#define PMD4_EEMD    _bs(6) // Bit 4
#define PMD4_HLVDM   _bs(6) // Bit 4

#define PMD1_U2MD   _bs(6)  // Bit 6
#define PMD1_I2C1MD _bs(6)  // Bit 7
#define PMD1_T1MD   _bs(6)  // Bit 11
#define PMD1_T2MD   _bs(6)  // Bit 12
#define PMD1_T3MD   _bs(6)  // Bit 13
#define PMD1_T5MD   _bs(6)  // Bit 15
#define PMD1_SPI1MD _bs(6)  // Bit 3
#define PMD1_ADC1MD _bs(6)  // Bit 0

// bool Device_IsUsbConnected(void);
#define Device_IsUsbConnected() USB_WK_Value()
//#define Device_IsUsbConnected() true
uint16_t Device_CheckHwReset(void);
#endif