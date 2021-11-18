
// NVM data space to save WDATA in words format
#include <string.h>
#include <stdio.h>  // printf
#include "device.h"
#include "dev_config.h"
#include "modules/RTCC.h"

#include "memory/DEE/dee.h"
#include "memory/SST26VF064B.h"
#include "sampling/measurement.h"  // measurementCounter()

//CRC-16 type
#define CRC16_DNP	0x3D65		// DNP, IEC 870, M-BUS, wM-BUS, ...
#define CRC16_CCITT	0x1021		// X.25, V.41, HDLC FCS, Bluetooth, ...
#define POLYNOM CRC16_DNP

extern device_t device; // Global Device Configuration/Status
static uint32_t ltime = 0;

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

void Device_StatusRefresh() { // Every call
    device.sts.timestamp = RTCC_GetTimeL(); // time evaluation reference
    // Through the calls 
    if ((device.sts.timestamp - ltime) > STATUS_REFRESH_PERIOD) {
        device.sts.power_level = Device_GetBatteryLevel(); // battery/harvesting level/status
        ltime = device.sts.timestamp;
    }
    device.sts.link_status = Device_IsWiredLinked(); // Exchange: USB / RF-RSSI
}

void Device_StatusRead() {
    // Start-up 
    device.sts.alarm_counter = Device_CheckHwReset(); // Persistent: wdt, critical errors/reset
    device.sts.DIN = __DEVICE_DIN;
    device.sts.version = __DEVICE_VER;
    device.sts.config_counter = device.cnf.CRC16; // To evaluate VMS alignment
    // Memory & Measurement
    DEE_Read(EEA_MEAS_COUNTER, &device.sts.meas_counter); // Persistent: stored measurements (dee.h)   
    device.sts.storage_space = depotFreeSpaceKb(); // available meas storage memory (Kb)
    device.sts.locked = (device.cnf.exchange.SKEY > 0); // Locked/not locked !!!
    Device_StatusRefresh();
}


//
//void Device_StatusDefaultSet() {
//    Device_StatusRead();
//}

void Device_SetLockSKEY(uint32_t skey) { // skey: >0 lock, =0 unlock 

}

void Device_ConfigDefaultSet() {
#ifdef __VAMP1K_TEST_CONFIG
    printf("Set Default Config !\n");
#endif   

    memset(&device.cnf, 0, sizeof (config_t));

    // General settings
    device.cnf.general.typeset = _SIG0; // 
    device.cnf.general.cycletime = 5;   // sec
    device.cnf.general.delaytime = 60; // 1 min
    device.cnf.general.timezone = 9; // Rome
    // Calibration
    device.cnf.calibration.et_factor = 100001; // Formato ?
    device.cnf.calibration.ws_factor = 200002;
    device.cnf.calibration.av_period = 1024; // default 1Khz
    device.cnf.calibration.av_filter = 6; // _snr = 30;
    device.cnf.calibration.av_factor = 500005;
    device.cnf.calibration.ss_factor = 600006;
    // Exchange
    device.cnf.exchange.attempt_mode = CNF_ATTEMPTMODE_EVERYCYCLE;
    device.cnf.exchange.app_timeout = 3000; //30sec
    device.cnf.exchange.handshake_timeout = 500; //1sec
    device.cnf.exchange.interchar_timeout = 100;
    device.cnf.exchange.SKEY = 0; // Fuori dal calcolo del CRC
    device.cnf.CRC16 = 0x00; // !
    //
}

#ifdef __VAMP1K_TEST_CONFIG

void printConfig() {
    timestamp_t stime; // Use: g_dev.st.lasttime
    RTCC_GetTime(&stime);
    printf("---Time: %u:%u:%u\n#:", stime.hour, stime.min, stime.sec);
    // General settings
    printf("typeset=%u\n", device.cnf.general.typeset);
    printf("cycletime=%u\n", device.cnf.general.cycletime);
    printf("delaytime=%u\n", device.cnf.general.delaytime);
    //printf("timezone=%u", g_dev.cnf.general.timezone);

    // Calibration
    //printf("et_factor=%lu\n", g_dev.cnf.calibration.et_factor);
    //printf("ws_factor=%lu\n", g_dev.cnf.calibration.ws_factor);
    //g_dev.cnf.calibration.av_period = 127; // Synco 38.4 Khz : 16 = 2.4Khz    0,416 ms
    printf("av_filterr=%lu\n", device.cnf.calibration.av_filter);
    //g_dev.cnf.calibration.av_factor = 500005;
    //g_dev.cnf.calibration.ss_factor = 600006;

    // Exchange
    printf("attempt_mode=%u\n", device.cnf.exchange.attempt_mode);
    //    printf("app_timeout=%u", g_dev.cnf.exchange.app_timeout);
    //    printf("handshake_timeout=%u", g_dev.cnf.exchange.handshake_timeout);
    //g_dev.cnf.exchange.interchar_timeout = 100;
    printf("SKEY=%lu\n", device.cnf.exchange.SKEY);
    printf("---CRC16=%u\n", device.cnf.CRC16);
}
#endif

bool Device_ConfigWrite(uint8_t * pSrc) {   // Check consistency and COMPUTE CRC !!!!!!!!!!!!!
    DEE_RETURN_STATUS res = DEE_NO_ERROR;
    uint16_t i, nbyte;
    uint16_t * ptr;
    
    nbyte = sizeof (config_t);
    memcpy(&device.cnf, pSrc, nbyte); // Update active
    nbyte = nbyte >> 1;
    ptr = (uint16_t*) pSrc;
    i = 0;
    while (res == DEE_NO_ERROR && (i < nbyte)) {
        res = DEE_Write(i, *(ptr + i));
        i++;
    }

#ifdef __VAMP1K_TEST_CONFIG
    printf("Write:\n");
    printConfig();
    _LATB2 = 1; // Led on
#endif

    return (i == nbyte);
}

//void Device_FactoryDefaultSet() {
//}

bool Device_ConfigRead() {
    DEE_RETURN_STATUS res = DEE_NO_ERROR;
    uint16_t i, nbyte;
    uint16_t* ptr;
    ptr = (uint16_t*) & device.cnf;
    nbyte = sizeof (config_t) >> 1; // Reads word

    i = 0;
    while (res == DEE_NO_ERROR && (i < nbyte)) {
        res = DEE_Read(i, (ptr + i));
        i++;
    }
    // result = (i == nbyte);
#ifdef __VAMP1K_TEST_CONFIG
    printf("Read:\n");
    printConfig();
    _LATB2 = 1; // Led on
#endif

    if (device.cnf.CRC16 != computeCRC16((uint8_t *) & device.cnf, sizeof (config_t) - 2)) { /// Check XOR checksum
        Device_ConfigDefaultSet();
        device.cnf.CRC16 = computeCRC16((uint8_t *) & device.cnf, sizeof (config_t) - 2);
        Device_ConfigWrite((uint8_t*) & device.cnf); // Write EEprom/Flash
        // Factory default !!!!
        //Device_OnFactoryDefaultSet();
        return (false);
    }
    return (true);
}

//==============================================================================

