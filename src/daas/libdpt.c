/*
 * File:   depot.c
 * Author: plogiacco@smartlab.it
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

void depotDefaultSet() {
    // Initialize Persistent Data in eeprom
    DEE_Write(EEA_MEAS_COUNTER, 0); // (dee.h)  
    DEE_Write(EEA_SST26_SECTOR, SST26_SECTOR0); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, 0); // (dee.h)  
    //
    // Initialize flash memory ( Erase First Sector ) )
    SST26_Enable();
    SST26_Global_Protection_Unlock();
    SST26_Erase_Sector(SST26_SECTOR0 * SST26_SECTOR_SIZE_IN_BYTE); // Set 4K in 0xFF state
    SST26_Disable();
}

uint16_t depotFreeSpaceKb() { // return 
    uint16_t Sector, Offset;
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  
    return ( SST26_SIZE_IN_KB - (((Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset) / 1024));
}

uint16_t depotPush(uint8_t* dPtr, uint16_t nBytes) { // return written bytes
    uint32_t sst_addr;
    uint16_t written, wrote; // , available;
    uint16_t Sector, Offset;

    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  

    wrote = 0;
    written = 0;
    SST26_Enable();
    SST26_Global_Protection_Unlock();
//    printf(">depotPush:\n");
    while (nBytes > 0) {

        if (Offset == SST26_SECTOR_SIZE_IN_BYTE) {
            if (Sector < SST26_MAX_SECTOR) {
                Sector++; // Next 4K Sector
//                printf(">CAMBIO SETTORE...DISPONIBILI 4K \n");
                Offset = 0x0;
                SST26_Erase_Sector(Sector * SST26_SECTOR_SIZE_IN_BYTE); // Set 4K in 0xFF state
            } else {
                return (0); // Out of memory !!!!!!!!!!!!!!!!!
            }
        }
        sst_addr = (Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset;

//        printf(">SEND %u Bytes Addr= %lu)!\n", nBytes, sst_addr);

        if (nBytes < (SST26_SECTOR_SIZE_IN_BYTE - Offset)) {
            wrote = SST26_Write(sst_addr, (uint8_t*) (dPtr + written), nBytes);
        } else {
            wrote = SST26_Write(sst_addr, (uint8_t*) (dPtr + written), (SST26_SECTOR_SIZE_IN_BYTE - Offset));
        }
//        printf(">WROTE %u Bytes)!\n", wrote);
        nBytes -= wrote;
        Offset += wrote;
        written += wrote;

    }
    SST26_Disable();

    DEE_Write(EEA_SST26_SECTOR, Sector); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, Offset); // (dee.h)      
    return (written);
}

/* -------------------------------------------------------------------------- */
uint16_t depotPull(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release) { // return readed bytes

    ////    uint16_t iSector, iOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    if (((Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset) < (nBytes + displacement)) {
        return (0);
    }
    sst_addr = (Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset - nBytes - displacement;
//    printf(">RECV %u Bytes Addr= %lu)!\n", nBytes, sst_addr);
    SST26_Enable();
    nBytes = SST26_Read(sst_addr, nBytes, dPtr);

    return (nBytes);
}

/* -------------------------------------------------------------------------- */
uint16_t depotDrop(uint16_t nBytes) {
    uint16_t newSector, newOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    if (((Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset) >= nBytes) {

        sst_addr = (Sector * SST26_SECTOR_SIZE_IN_BYTE) + Offset - nBytes;
//        printf(">DROP %u Bytes (addr=%lu)!\n", nBytes, sst_addr);
        newSector = sst_addr / SST26_SECTOR_SIZE_IN_BYTE;
        newOffset = sst_addr % SST26_SECTOR_SIZE_IN_BYTE;
//        printf(">FROM sect %u, off %u \n", newSector, newOffset);
        // Move in newSector and save newOffset
        SST26_Enable();
        SST26_Global_Protection_Unlock();

        // Move data to buffer !!! USES SSBUF !!!
        sst_addr = newSector * SST26_SECTOR_SIZE_IN_BYTE;
        SST26_Read(sst_addr, newOffset, (uint8_t *) SSBUF);

        // Initialize Sector
        SST26_Erase_Sector(sst_addr); // Set 4K in 0xFF state

        // Move buffer to flash !!! USES SSBUF !!!
        SST26_Write(sst_addr, (uint8_t *) SSBUF, newOffset);
        SST26_Disable();

        DEE_Write(EEA_SST26_SECTOR, newSector); // (dee.h)  
        DEE_Write(EEA_SST26_OFFSET, newOffset); // (dee.h)  
        return (nBytes);
    }
    return (0);
}
