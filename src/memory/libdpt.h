
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

/******************************************************************************
 
    Measurement storing block
    +-----------+----------+----------+----------+----------+
    | N bytes   | 4 byte   | 1 byte   | 2 byte   | 2 byte   |
    +-----------+----------+----------+----------+----------+
    | Samples.. | dtime    | typeset  | ns       | nss      |
    +-----------+----------+----------+----------+----------+

    measurement_size: header (4+1+2+2) + samples ((ns+nss)*2)
    ptr2Last = +/- measurement_size
 
 ******************************************************************************/


#ifndef XC_DEPOT_H
#define	XC_DEPOT_H

//
#define SST26_SIZE_KB             8192   // SST26VF064 capacity is 8,388,608 bytes 
#define SST26_SECTOR_SIZE         4096   // 4Kbyte 0..4095
#define SST26_MAX_SECTOR          2048   //  = 8Mbyte / 4Kbyte
#define SST26_PAGE_SIZE            256   // Page size in byte
#define SST26_PAGEXSECTOR           16   // Pages per Sector

#define SST26_SECTOR0                0   // First used sector

void depotDefaultSet();
uint16_t depotFreeSpaceKb();

uint16_t depotPush(uint8_t* dPtr, uint16_t nBytes);
uint16_t depotDrop(uint16_t nBytes);
uint16_t depotPull(uint8_t* dPtr, uint16_t displacement, uint16_t nBytes, bool release);


#endif	/* XC_DEPOT_H */

