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

void depotDefaultSet() {
    // Initialize Persistent Data in eeprom
    DEE_Write(EEA_MEAS_COUNTER, 0); // (dee.h)  
    DEE_Write(EEA_SST26_SECTOR, SST26_SECTOR0); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, 0); // (dee.h)  
    //
    // Initialize flash memory ( Erase First Sector ) )
    SST26_Enable();
    SST26_Global_Protection_Unlock();
    SST26_Erase_Sector(SST26_SECTOR0 * SST26_SECTOR_SIZE); // Set 4K in 0xFF state
    SST26_Disable();
}

uint16_t depotFreeSpaceKb() { // return 
    uint16_t Sector, Offset;
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  
    return ( SST26_SIZE_KB - (((Sector * SST26_SECTOR_SIZE) + Offset) / 1024));
}

uint16_t depotPush(uint8_t* dPtr, uint16_t nBytes) { // return written bytes
    uint32_t sst_addr;
    uint16_t written, available;
    uint16_t Sector, Offset;

    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h)  

    written = 0;
    SST26_Enable();
    SST26_Global_Protection_Unlock();

    while (nBytes > 0) {
        available = SST26_SECTOR_SIZE - Offset;

        if (available == 0) {
            Sector++; // Next 4K Sector
//            printf("CAMBIO SETTORE...DISPONIBILI 4K ");
            SST26_Erase_Sector(Sector * SST26_SECTOR_SIZE); // Set 4K in 0xFF state
            Offset = 0x0;
        }
        sst_addr = (Sector * SST26_SECTOR_SIZE) + Offset;
//        printf("Send %u Bytes Addr= %lu)! \n", nBytes, sst_addr);
        if (nBytes <= available) {
            written = SST26_Write(sst_addr, (dPtr + written), nBytes);
        } else {
            written = SST26_Write(sst_addr, (dPtr + written), available);
        }

        nBytes -= written;
        Offset += written;
    }

    //    SST26_WRDI();
    SST26_Disable();

    DEE_Write(EEA_SST26_SECTOR, Sector); // (dee.h)  
    DEE_Write(EEA_SST26_OFFSET, Offset); // (dee.h)  
    return (written);
}

/* -------------------------------------------------------------------------- */
uint16_t depotDrop(uint16_t nBytes) {
    uint16_t newSector, newOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    //    newSector = Sector - (nBytes / SST_SECTOR_SIZE);
    //    if ((nBytes % SST_SECTOR_SIZE) >= Offset) {
    //        newOffset = (Offset - (nBytes % SST_SECTOR_SIZE));
    //    }
    //    else {
    //        newSector--;
    //        newOffset = SST_SECTOR_SIZE - ((nBytes % SST_SECTOR_SIZE) - Offset);
    //    }

    if (nBytes <= Offset) {
        newOffset = (Offset - nBytes);
        newSector = Sector;
    } else {
        newSector = Sector - (nBytes / SST26_SECTOR_SIZE);
        newOffset = Offset - (nBytes % SST26_SECTOR_SIZE);
    }

    // Move in newSector and save newOffset
    SST26_Enable();
    //    SST26_WREN();
    SST26_Global_Protection_Unlock();
    //    SST26_Wait_Busy();

    // Move data to buffer !!! USES SSBUF !!!
    sst_addr = (newSector * SST26_SECTOR_SIZE);
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

/* -------------------------------------------------------------------------- */
uint16_t depotPull(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release) { // return readed bytes

    uint16_t iSector, iOffset;
    uint16_t Sector, Offset;
    uint32_t sst_addr;
    //
    DEE_Read(EEA_SST26_SECTOR, &Sector); // (dee.h)  
    DEE_Read(EEA_SST26_OFFSET, &Offset); // (dee.h) 

    iSector = Sector - ((nBytes + displacement) / SST26_SECTOR_SIZE);

    if (((nBytes + displacement) % SST26_SECTOR_SIZE) >= Offset) {
        iOffset = (Offset - ((nBytes + displacement) % SST26_SECTOR_SIZE));
    } else {
        iSector--;
        iOffset = (SST26_SECTOR_SIZE - (((nBytes + displacement) % SST26_SECTOR_SIZE) - Offset));
    }

    SST26_Enable();

    sst_addr = (iSector * SST26_SECTOR_SIZE) + iOffset;

    nBytes = SST26_Read(sst_addr, nBytes, dPtr);
    return (nBytes);
}
