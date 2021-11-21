/*
 * File:   depot.c
 * Author: plogi
 *
 * Created on 18 novembre 2021, 9.57
 */


#include "xc.h"

#include "../utils.h"
// #include "../modules/RTCC.h"

#include "../memory/DEE/dee.h"
#include "../memory/SST26VF064B.h"  // Flash SPI

#include "../sampling/measurement.h"

#include "libdpt.h"


extern device_t device;
extern sample_t SSBUF[SS_BUF_SIZE]; // global buffer
static flash_address_t addr;

#define dee_load_address() uint16_t aw; \
                           DEE_Read(EEA_SST26_HA, &aw);  \
                           addr.a32 = aw;  \
                           DEE_Read(EEA_SST26_LA, &aw);  \
                           addr.a32 << 16; \
                           addr.a32 | aw; 
                           

#define dee_save_address() DEE_Write(EEA_SST26_HA, (uint16_t) (addr.a32 >> 16)); \ 
                           DEE_Write(EEA_SST26_LA, (uint16_t) (addr.a32 & 0xFFFF));

void depotInitialize() {

}

void depotDefaultSet() {
    // Initialize Persistent Data in eeprom
    DEE_Write(EEA_MEAS_COUNTER, 0); // (dee.h)  
    DEE_Write(EEA_SST26_HA, 0); // (dee.h)  
    DEE_Write(EEA_SST26_LA, 0); // (dee.h)  
    addr.a32=0;
    //
    // Initialize flash memory ( Erase First Sector ) )
    SST26_Enable();
    SST26_Global_Protection_Unlock();
    SST26_Erase_Sector(addr.a32); // Set 4K in 0xFF state
    SST26_Disable();
}

uint16_t depotFreeSpaceKb() { // return 
    uint16_t Sector, Offset;
    DEE_Read(EEA_SST26_HA, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_LA, &Offset); // (dee.h)  
    return ( SST26_SIZE_KB - (((Sector * SST26_SECTOR_SIZE) + Offset) / 1024));
}


uint16_t depotPush(uint8_t* dPtr, uint16_t nBytes) { // return written bytes
    //    uint32_t sst_addr;
    uint16_t written, available;

    dee_load_address(); // Load address from DDE
    //

    written = 0;
    SST26_Enable();
    SST26_Global_Protection_Unlock();

    while (nBytes > 0) {
        available = SST26_SECTOR_SIZE - SECTOR_OFFSET(addr);

        if (available == 0) {
            if (addr.sector < SST26_MAX_SECTOR) {
                addr.sector++; // Next 4K Sector
                printf("CAMBIO SETTORE...DISPONIBILI 4K ");
                SST26_Erase_Sector(addr.sector); // Set 4K in 0xFF state
                addr.page = 0;
                addr.offset = 0;
            } else {
                return (0);
            }
        }
        //        sst_addr = (Sector * SST26_SECTOR_SIZE) + Offset;
        printf("Send %u Bytes Addr= %lu)! \n", nBytes, addr.a32);
        if (nBytes <= available) {
            written = SST26_Write(&addr, (dPtr + written), nBytes);
        } else {
            written = SST26_Write(&addr, (dPtr + written), available);
        }
        nBytes -= written;
    }

    SST26_Disable();

    // Save address
    dee_save_address();

    return (written);
}

/* -------------------------------------------------------------------------- */
uint16_t depotDrop(uint16_t nBytes) {
    
//    flash_address_t tmp;
//
//    dee_load_address(); // Load address from DDE
//
//
//    if (nBytes <= SECTOR_OFFSET(addr)) {
//        tmp. = (Offset - nBytes);
//        newSector = Sector;
//    } else {
//        newSector = Sector - (nBytes / SST26_SECTOR_SIZE);
//        newOffset = Offset - (nBytes % SST26_SECTOR_SIZE);
//    }
//
//    // Move in newSector and save newOffset
//    SST26_Enable();
//    //    SST26_WREN();
//    SST26_Global_Protection_Unlock();
//    //    SST26_Wait_Busy();
//
//    // Move data to buffer !!! USES SSBUF !!!
//    sst_addr = (newSector * SST26_SECTOR_SIZE);
//    SST26_Read(sst_addr, newOffset, (uint8_t *) SSBUF);
//    // Initialize Sector
//    SST26_Erase_Sector(sst_addr); // Set 4K in 0xFF state
//
//    // Move buffer to flash !!! USES SSBUF !!!
//    SST26_Write(&sst_addr, (uint8_t *) SSBUF, newOffset);
//    SST26_Disable();
//
//    // Recompute Sector & Offset
//
//    DEE_Write(EEA_SST26_HA, newSector); // (dee.h)  
//    DEE_Write(EEA_SST26_LA, newOffset); // (dee.h)  

    return (nBytes);
}

/* -------------------------------------------------------------------------- */
uint16_t depotPull(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release) { // return readed bytes

    uint16_t iSector, iOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_HA, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_LA, &Offset); // (dee.h) 

    iSector = Sector - ((nBytes + displacement) / SST26_SECTOR_SIZE);

    if (((nBytes + displacement) % SST26_SECTOR_SIZE) >= Offset) {
        iOffset = (Offset - ((nBytes + displacement) % SST26_SECTOR_SIZE));
    } else {
        iSector--;
        iOffset = (SST26_SECTOR_SIZE - (((nBytes + displacement) % SST26_SECTOR_SIZE) - Offset));
    }

    SST26_Enable();

    // sst_addr = ((iSector << 16 ) & (iOffset<<8));
    sst_addr = (iSector * SST26_SECTOR_SIZE) + iOffset;

    nBytes = SST26_Read(sst_addr, nBytes, dPtr);
    return (nBytes);
}
