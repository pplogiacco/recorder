/*******************************************************************************
 D E V I C E   C O N F I G U R A T I O N   &   S T A T U S
 
 Device_DefaultConfigSet()
 bool Device_WriteConfig(uint8_t * config)
 Device_ReadConfig(config_t * config)
 
 bool Device_GetStatus(status_t * status) 

 void Device_LockKeySet(uint32_t skey)
 
 REQUIRE : device_t g_dev;  // Global configuration 
 *******************************************************************************/

#ifndef DEVCONFIG_H
#define	DEVCONFIG_H

typedef enum { // Power sources
    _LINE = 0x1, // line
    _BATT = 0x3, // batteries
    _HARV = 0x5, // harvesting
    _BAHA = 0x9 // batteries+harvesting
} powermode_t;

//----------------------------------------------------------------------------//
#define CONFIG_T_SIZE 94 // sizeof(general_t) + sizeof(calibration_t) + sizeof(exchange_t) + sizeof(CRC16)
//----------------------------------------------------------------------------//

typedef struct { // General Settings
    uint16_t typeset; // Active measuremnt's typeset
    uint16_t powermode; // Preferred power source  (Default 0)
    uint16_t samplingmode; // (0)Scheduled, (1)Switched-off, (2)On-demand,(3)Continuously    (Default 0)
    uint16_t delaytime; // Delay-before-sampling (seconds)
    uint16_t cycletime; // Sampling-duration (seconds)
    uint32_t startdate; // Start sampling event after date-time
    uint32_t stopdate; // End sampling event after date-time
    uint16_t timezone; // Timezone to reference all date-time values
} general_t; // 20 Bytes

typedef struct { // Calibration/Sampling factors and parameters:
    uint32_t et_factor; // Coefficient to scale to Celsius
    uint32_t ws_factor; // Coefficient to scale to m/s
    uint32_t av_period; // Vibration Amplitude Sampling Frequency Parameter (Period)
    uint32_t av_filter; // Vibration Amplitude Peak-Peak Variations Tollerance
    uint32_t av_factor; // Amplitude Coefficient to scale to micro-meters
    uint32_t ss_factor; // Amplitude to scale to micro-meters
    uint32_t RFU[8]; // !!!
} calibration_t; // 56 Bytes

typedef struct { // Exchange Process:
    uint32_t SKEY; // locked on security-key
    uint16_t vms_mode; // (0) Interrupt mode, (1) Polling mode
    uint16_t attempt_mode; // (0) Try every cycle, (>0) Scheduled every <n> seconds
    uint16_t handshake_timeout; // Timeout on handshake phase
    uint16_t interchar_timeout; // Timeout on data transfer
    uint16_t app_timeout; // Timeout on waiting application commands
    uint16_t retrieve_mode; // (0) Retrieve all datas and delete, (1) Retrieve last meas...
} exchange_t; // 16 Bytes

#define EXCHANGE_ATTEMPTMODE_EVERYCYCLE  0x00  // Try every sampling cycle

typedef struct { // Device Configuration ( 94 Bytes )
    general_t general;
    exchange_t exchange;
    calibration_t calibration;
    uint16_t CRC16; // Generated by VMS
} config_t;

typedef struct { // Device status (Read Only)
    uint32_t DIN; // Device Identifier Number
    uint32_t locked; // locked VMS key (True/False))
    uint16_t version; // Hardware & Firmware (Protocol)  Version
    uint32_t timestamp; // time evaluation reference
    uint16_t power_status; // (1) line, (3) battery, (5) harvesting, (9) battery+harvesting
    uint16_t power_level; // battery/harvesting level/status
    uint16_t storage_space; // available meas?s storage memory
    uint16_t link_status; // RSSI, other
    uint16_t alarm_counter; // wdt, critical errors/reset
    uint16_t meas_counter; // stored measuraments
    uint16_t config_counter; // configuration CRC16
} status_t;

typedef struct {
    config_t cnf; // Active configuration ( 94 Bytes )
    status_t st; // RO status
} device_t;

extern device_t g_dev;  // Global configuration

bool Device_ConfigRead(config_t * config);
bool Device_ConfigWrite(uint8_t *config);
bool Device_GetStatus(status_t *status);
void Device_ConfigDefaultSet(config_t * config);

#endif	// DEVCONFIG_H 

