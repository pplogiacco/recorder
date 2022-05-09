
// NVM data space to save WDATA in words format
#include <string.h>
#include <stdio.h>  // printf
#include "device.h"
#include "dev_config.h"
#include "modules/RTCC.h"

#include "memory/DEE/dee.h"
#include "memory/SST26VF064B.h"
#include "sampling/measurement.h"  // measurementCounter()

extern device_t device; // Global Device Configuration/Status
//static uint32_t ltime = 0;

//------------------------------------------------------------------------------
//CRC-16 type
//------------------------------------------------------------------------------
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


//------------------------------------------------------------------------------

void Device_StatusRefresh() { // Every call
    device.sts.timestamp = RTCC_GetTimeL(); // time evaluation reference
    // Through the calls 
    //////    if ((device.sts.timestamp - ltime) > STATUS_REFRESH_PERIOD) {
    //////        device.sts.power_level = Device_GetBatteryLevel(); // battery/harvesting level/status
    //////        ltime = device.sts.timestamp;
    //////    }
    //device.sts.link_status =  (....  // Exchange: USB / RF-RSSI
}

void Device_StatusRead() {
    // Start-up 
    device.sts.DIN = __DEVICE_DIN; // Read from OTP
    device.sts.version = __DEVICE_VER;
    
    // Configuration ( last version )
    device.sts.config_counter = device.cnf.CRC16; // To evaluate VMS alignment

    // Critical errors/reset
    DEE_Read(EEA_RESET_COUNTER, &device.sts.alarm_counter); // persistent counter (dee.h)   
    device.sts.alarm_counter += Device_CheckHwReset(); // device.sts.alarm_counter

    // Memory & Measurement
    DEE_Read(EEA_MEAS_COUNTER, &device.sts.meas_counter); // Persistent: stored measurements (dee.h)   
    device.sts.storage_space = depotFreeSpaceKb(); // available meas storage memory (Kb)

    // Exchange
    device.sts.link_status = 0; // Exchange: USB / RF-RSSI
    device.sts.locked = (device.cnf.exchange.SKEY > 0); // Locked/not locked !!!
    Device_StatusRefresh();
}

void Device_StatusDefaultSet() {
    device.sts.meas_counter = 0;
    DEE_Write(EEA_MEAS_COUNTER, device.sts.meas_counter); // (dee.h)  
    device.sts.alarm_counter = 0;
    DEE_Write(EEA_RESET_COUNTER, device.sts.alarm_counter); // (dee.h)  

}

void Device_SetLockSKEY(uint32_t skey) { // skey: >0 lock, =0 unlock 

}

bool Device_ConfigWrite(uint8_t * objcnf) { // Check consistency and COMPUTE CRC !!!!!!!!!!!!!
    DEE_RETURN_STATUS res = DEE_NO_ERROR;
    uint16_t i, nbyte;
    uint16_t * ptr;
    config_t * pcnf;

    pcnf = (config_t *) objcnf;
    nbyte = sizeof (config_t);

#ifdef ACCEPT_ALL_CONFIG_OBJ 
    //    pcnf->CRC16 = computeCRC16(objcnf, nbyte - 2);
    {
#else
    if (pcnf->CRC16 != computeCRC16(objcnf, nbyte - 2)) { // Not valid !!!!
        return (false);
    } else {
#endif        
        memcpy(&device.cnf, objcnf, nbyte); // Update active
        nbyte = nbyte >> 1;
        ptr = (uint16_t*) objcnf; // Save in EEprom
        i = 0;
        while (res == DEE_NO_ERROR && (i < nbyte)) {
            res = DEE_Write(i, (uint16_t) *(ptr + i));
            i++;
        }
    }
    return (true);
}

void Device_ConfigDefaultSet() {
    //#ifdef __VAMP1K_TEST_CONFIG
    //    printf("Set Default Config !\n");
    //#endif   

    config_t objcnf;
    memset(&objcnf, 0, sizeof (config_t));

    // General settings
    objcnf.general.typeset = _SIG0; // 
    objcnf.general.cycletime = 10; // sec
    objcnf.general.delaytime = 60; // 1 min
    objcnf.general.timezone = 9; // Rome
    // Calibration
    objcnf.calibration.et_factor = 100001; // Formato ?
    objcnf.calibration.ws_factor = 200002;
    objcnf.calibration.av_period = 10; // default 10 Hz Sample Signal
    objcnf.calibration.av_filter = 1024; // default 1024 Demo Amplitude
    objcnf.calibration.av_factor = 500005;
    objcnf.calibration.ss_factor = 600006;
    // Exchange
    objcnf.exchange.attempt_mode = CNF_ATTEMPTMODE_EVERYCYCLE;
    objcnf.exchange.app_timeout = 3000; //30sec
    objcnf.exchange.handshake_timeout = 500; //1sec
    objcnf.exchange.interchar_timeout = 100;
    objcnf.exchange.SKEY = 0; // Fuori dal calcolo del CRC
    // Set as valid
    objcnf.CRC16 = computeCRC16((uint8_t*) & objcnf, sizeof (config_t) - 2); // !
    Device_ConfigWrite((uint8_t*) & objcnf);
}

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

#ifndef ACCEPT_ALL_CONFIG_OBJ 
    if (device.cnf.CRC16 != computeCRC16((uint8_t *) & device.cnf, sizeof (config_t) - 2)) { /// Check XOR checksum
        //        Device_ConfigDefaultSet();        
        //        Device_ConfigWrite((uint8_t*) & device.cnf); // Write EEprom/Flash
        //        // Factory default !!!!
        return (false);
    }
#endif

    return (true);
}

//==============================================================================

