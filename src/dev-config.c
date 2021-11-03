
// NVM data space to save WDATA in words format
#include <string.h>
#include <stdio.h>  // printf
#include "device.h"
#include "dev-config.h"
#include "modules/RTCC.h"

#include "memory/DEE/dee.h"
#include "sampling/measurement.h"  // measurementCounter()


#ifndef __NOFLASH
//static __prog__ uint8_t nvmConfigDatas[FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS] __attribute__((space(prog), aligned(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS)));
//const unsigned int ramConfigSize = sizeof (config_t);
#endif


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
    config->general.typeset = _AV01; // _SIG0; // 
    config->general.cycletime = 5;
    config->general.delaytime = 60; // 1 min
    config->general.timezone = 9; // Rome
    // Calibration
    config->calibration.et_factor = 100001; // Formato ?
    config->calibration.ws_factor = 200002;
    config->calibration.av_period = 5; // default
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
    
    // Initialize Persistent Status Datas
    DEE_Write(EEA_MEAS_COUNTER, 0); // (dee.h)  
    DEE_Write(EEA_SST_SECTOR, 0); // (dee.h)  
    DEE_Write(EEA_SST_OFFSET, 0); // (dee.h)  
    
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


    ////////#ifndef __NOFLASH // Save config in Flash
    ////////    uint32_t nvm_address; // 24 bit address
    ////////    uint16_t iR, iF;
    ////////
    ////////#if defined( __PIC24FJ256GA702__)
    ////////    uint32_t towrite[2U];
    ////////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & nvmConfigDatas);
    ////////    FLASH_Unlock(FLASH_UNLOCK_KEY);
    ////////
    ////////    result = FLASH_ErasePage(nvm_address);
    ////////#else
    ////////    uint32_t towrite[FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS];
    ////////    nvm_address = __builtin_tbladdress(nvmConfigDatas); // Get address of reserved space
    ////////    FLASH_Unlock(FLASH_UNLOCK_KEY);
    ////////    result = FLASH_Erase1Row(nvm_address); // Erase flash page/row
    ////////#endif
    ////////
    ////////    if (result) {
    ////////        iR = 0; // Bytes
    ////////        iF = iR; // Words
    ////////
    ////////#if defined( __PIC24FJ256GA702__)
    ////////        while (iR < ramConfigSize) { //  Sizeof(config_t)
    ////////            towrite[0] = *(pSrc + iR + 2);
    ////////            towrite[0] <<= 8;
    ////////            towrite[0] |= *(pSrc + iR + 1);
    ////////            towrite[0] <<= 8;
    ////////            towrite[0] |= *(pSrc + iR);
    ////////            towrite[1] = *(pSrc + iR + 5);
    ////////            towrite[1] <<= 8;
    ////////            towrite[1] |= *(pSrc + iR + 4);
    ////////            towrite[1] <<= 8;
    ////////            towrite[1] |= *(pSrc + iR + 3);
    ////////            iR += 6;
    ////////
    ////////            //            printf("%u %u \n", (uint16_t) (w0>>16)& 0xFFFF, (uint16_t)w0 & 0xFFFF);
    ////////            //            printf("%u %u \n", (uint16_t) (w1>>16)& 0xFFFF , (uint16_t)w1 & 0xFFFF);         
    ////////            result &= FLASH_WriteDoubleWord24(nvm_address + iF, towrite[0], towrite[1]);
    ////////            iF += 4U;
    ////////        }
    ////////#else
    ////////        while (iR < ramConfigSize) { //  Sizeof(config_t)
    ////////            towrite[iF] = *(pSrc + iR + 2);
    ////////            towrite[iF] <<= 8;
    ////////            towrite[iF] |= *(pSrc + iR + 1);
    ////////            towrite[iF] <<= 8;
    ////////            towrite[iF] |= *(pSrc + iR);
    ////////            iR += 3;
    ////////            iF += 1U;
    ////////        }
    ////////        // Writes a single row of data from the location given in *data
    ////////        result &= FLASH_WriteRow24(nvm_address, towrite);
    ////////#endif
    ////////    }
    ////////    FLASH_Lock();
    ////////    //_LATB2 = 1;
    ////////#endif  // NoFLASH
    ////////    return result;
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

    //////////#ifdef __NOFLASH
    //////////    Device_ConfigDefaultSet(config);
    //////////#else    // Read flash/eeprom 
    //////////    uint32_t rdata;
    //////////    uint32_t nvm_address;
    //////////    uint8_t * pDst;
    //////////    uint16_t iR, i, iF;
    //////////
    //////////    pDst = (uint8_t*) config;
    //////////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & nvmConfigDatas);
    //////////
    //////////    iR = 0;
    //////////    iF = iR;
    //////////    while (iR < ramConfigSize) { // && iF << flash page
    //////////        rdata = FLASH_ReadWord24(nvm_address + iF);
    //////////        //printf("%u %u \n", (uint16_t) (rdata>>16) & 0xFFFF , (uint16_t) rdata & 0xFFFF);
    //////////        i = 0;
    //////////        while ((i < 3) && (iR + i < ramConfigSize)) {
    //////////            *(pDst + iR + i) = (rdata & 0xFF);
    //////////            rdata >>= 8;
    //////////            i++;
    //////////        }
    //////////        iR += i;
    //////////        iF += 2U;
    //////////    }
    //////////
    //////////    if (config->CRC16 != computeCRC16((uint8_t *) config, sizeof (config_t) - 2)) { /// Check XOR checksum
    //////////        Device_ConfigDefaultSet(config);
    //////////        config->CRC16 = computeCRC16((uint8_t *) config, sizeof (config_t) - 2);
    //////////        Device_ConfigWrite((uint8_t*) config); // Write EEprom/Flash
    //////////    }
    //////////#endif
    //////////
    //////////#ifdef __VAMP1K_TEST_CONFIG
    //////////    printf("Read:\n");
    //////////    printConfig();
    //////////    _LATB2 = 1; // Led on
    //////////#endif
    //////////    
    //////////    return result;
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
    
    DEE_Write(EEA_MEAS_COUNTER, status->meas_counter); // Persistent: stored measurements (dee.h)  
    DEE_Write(EEA_SST_SECTOR, 0); // (dee.h)  
    DEE_Write(EEA_SST_OFFSET, 0); // (dee.h)  
    status->storage_space = sstFreeSpaceKb(); // available meas storage memory (Kb)

    // Exchange
    status->link_status = Device_IsUsbConnected(); // Exchange: USB / RF-RSSI
    status->locked = (g_dev.cnf.exchange.SKEY > 0); // Locked/not locked !!!

    return true;
}

void Device_SetLockSKEY(uint32_t skey) { // skey: >0 lock, =0 unlock 

}


//==============================================================================
