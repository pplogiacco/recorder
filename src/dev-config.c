
// NVM data space to save WDATA in words format

#include <string.h>
#include <stdio.h>  // printf
#include "device.h"
#include "dev-config.h"
#include "modules/RTCC.h"

#include "memory/DEE/dee.h"
#include "sampling/measurement.h"  // measurementCounter()


//CRC-16 type
#define CRC16_DNP	0x3D65		// DNP, IEC 870, M-BUS, wM-BUS, ...
#define CRC16_CCITT	0x1021		// X.25, V.41, HDLC FCS, Bluetooth, ...
#define POLYNOM CRC16_DNP

uint16_t computeCRC16(uint8_t *strtochk, uint16_t length) {
    uint16_t crc;
    uint8_t i = 0;
    uint16_t aux = 0;
    uint16_t newByte;

    crc = 0x0000; //Initialization of crc to 0x0000 for DNP
    //crc = 0xFFFF; //Initialization of crc to 0xFFFF for CCITT
    while (aux < length) {
        newByte = strtochk[aux];
        for (i = 0; i < 8; i++) {
            if (((crc & 0x8000) >> 8) ^ (newByte & 0x80)) {
                crc = (crc << 1) ^ POLYNOM;
            } else {
                crc = (crc << 1);
            }
            newByte <<= 1;
        }
        aux++;
    }
    return (~crc); //The crc value for DNP it is obtained by NOT operation
}

void Device_ConfigDefaultSet(config_t * config) {
#ifdef __VAMP1K_TEST_CONFIG
    printf("Set Default Config !\n");
#endif   
    
    memset(config, 0, sizeof (config_t));
    // General settings
    config->general.typeset =  _SIG0; // 
    config->general.cycletime = 5;
    config->general.delaytime = 60; // 1 min
    config->general.timezone = 9; // Rome
    // Calibration
    config->calibration.et_factor = 100001; // Formato ?
    config->calibration.ws_factor = 200002;
    config->calibration.av_period = 1024; // default 1Khz
    config->calibration.av_filter = 6; // _snr = 30;
    config->calibration.av_factor = 500005;
    config->calibration.ss_factor = 600006;
    // Exchange
    config->exchange.attempt_mode = CNF_ATTEMPTMODE_EVERYCYCLE;
    config->exchange.app_timeout = 3000; //30sec
    config->exchange.handshake_timeout = 500; //1sec
    config->exchange.interchar_timeout = 100;
    config->exchange.SKEY = 0; // Fuori dal calcolo del CRC
    config->CRC16 = 0x00; // !
    
    sstInitialize();    // DEE Initialized before...
    
}

#ifdef __VAMP1K_TEST_CONFIG

void printConfig() {
    timestamp_t stime; // Use: g_dev.st.lasttime
    RTCC_GetTime(&stime);
    printf("---Time: %u:%u:%u\n#:", stime.hour, stime.min, stime.sec);
    // General settings
    printf("typeset=%u\n", g_dev.cnf.general.typeset);
    printf("cycletime=%u\n", g_dev.cnf.general.cycletime);
    printf("delaytime=%u\n", g_dev.cnf.general.delaytime);
    //printf("timezone=%u", g_dev.cnf.general.timezone);

    // Calibration
    //printf("et_factor=%lu\n", g_dev.cnf.calibration.et_factor);
    //printf("ws_factor=%lu\n", g_dev.cnf.calibration.ws_factor);
    //g_dev.cnf.calibration.av_period = 127; // Synco 38.4 Khz : 16 = 2.4Khz    0,416 ms
    printf("av_filterr=%lu\n", g_dev.cnf.calibration.av_filter);
    //g_dev.cnf.calibration.av_factor = 500005;
    //g_dev.cnf.calibration.ss_factor = 600006;

    // Exchange
    printf("attempt_mode=%u\n", g_dev.cnf.exchange.attempt_mode);
    //    printf("app_timeout=%u", g_dev.cnf.exchange.app_timeout);
    //    printf("handshake_timeout=%u", g_dev.cnf.exchange.handshake_timeout);
    //g_dev.cnf.exchange.interchar_timeout = 100;
    printf("SKEY=%lu\n", g_dev.cnf.exchange.SKEY);
    printf("---CRC16=%u\n", g_dev.cnf.CRC16);
}
#endif

bool Device_ConfigWrite(uint8_t * pSrc) {
    //////////    bool result = true;
    DEE_RETURN_STATUS res = DEE_NO_ERROR;
    uint16_t i, nbyte;
    uint16_t * ptr;
    // Check consistency 
    nbyte = sizeof (config_t);
    memcpy(&g_dev.cnf, pSrc, nbyte); // Update active
    nbyte = nbyte >> 1;
    ptr = (uint16_t*) pSrc;
    i = 0;
    while (res == DEE_NO_ERROR && (i < nbyte)) {
        res = DEE_Write(i, *( ptr + i));
        i++;
    }
    
#ifdef __VAMP1K_TEST_CONFIG
    printf("Write:\n");
    printConfig();
    _LATB2 = 1; // Led on
#endif
    
    return (i == nbyte);

}

bool Device_ConfigRead(config_t * config) {
    bool result = true;

    DEE_RETURN_STATUS res = DEE_NO_ERROR;
    uint16_t i, nbyte;
    uint16_t* ptr;
    ptr = (uint16_t*) config;
    nbyte = sizeof (config_t) >> 1;

    i = 0;
    while (res == DEE_NO_ERROR && (i < nbyte)) {
        res = DEE_Read(i, (ptr + i));
        i++;
    }
    result = (i == nbyte);

#ifdef __VAMP1K_TEST_CONFIG
    printf("Read:\n");
    printConfig();
    _LATB2 = 1; // Led on
#endif

    if (config->CRC16 != computeCRC16((uint8_t *) config, sizeof (config_t) - 2)) { /// Check XOR checksum
        Device_ConfigDefaultSet(config);
        config->CRC16 = computeCRC16((uint8_t *) config, sizeof (config_t) - 2);
        result = Device_ConfigWrite((uint8_t*) config); // Write EEprom/Flash
    }

    return (result);

}

bool Device_StatusGet(status_t * status) {
    uint32_t lctime = RTCC_GetTimeL(); // time evaluation reference

    // Through the calls 
    if (lctime - status->timestamp < STATUS_REFRESH_PERIOD) {
        status->power_level = Device_GetBatteryLevel(); // battery/harvesting level/status
    }

    // Every call
    status->timestamp = lctime;
    status->config_counter = g_dev.cnf.CRC16; // To evaluate VMS alignment

    // Boot time... 
    status->alarm_counter = Device_CheckHwReset(); // Persistent: wdt, critical errors/reset
    status->DIN = __DEVICE_DIN;
    status->version = __DEVICE_VER;

    // Measurement
    //DEE_Read(EEA_MEAS_COUNTER, &g_dev.st.meas_counter); // Persistent: stored measurements (dee.h)   
    status->storage_space = sstFreeSpaceKb(); // available meas storage memory (Kb)

    // Exchange
    status->link_status = Device_IsUsbConnected(); // Exchange: USB / RF-RSSI
    status->locked = (g_dev.cnf.exchange.SKEY > 0); // Locked/not locked !!!

    return true;
}

void Device_SetLockSKEY(uint32_t skey) { // skey: >0 lock, =0 unlock 

}


//==============================================================================
