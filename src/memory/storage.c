/*
 * File:   storage.c
 * Author: plogiacco
 *
 * Created on 20 marzo 2021, 8.17
 */

#include "storage.h"
#include "flash702.h"


//#define FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS 1024U

#define DEPOT_SIZE_IN_PAGES 8U
#define DEPOT_SIZE_IN_BYTE  (DEPOT_SIZE_IN_PAGES*FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS*2U)

// Depot size
// 8x3*1024 = 24576
// storage-unit (timestamp,typeset,ns,nss,buffer) = 840 regs ca
// basedate, typeset, ns, freq, scale: 3 delta-date, 1 wind, 1 temp, 1 nss, 3 x nharm

// optimized 6+15 = 21-24 = 1024/96 = 10gg   

//static __prog__ uint8_t nvmDepot0[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot1[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot2[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot3[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot4[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot5[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot6[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));
//static __prog__ uint8_t nvmDepot7[DEPOT_SIZE_IN_BYTE] __attribute__((space(prog), aligned(DEPOT_SIZE_IN_BYTE)));

// const unsigned int ramConfigSize = sizeof (config_t);
//#endif

/*
static uint8_t _DSER_ = 0x00; // Data Storage Error Register 

inline uint8_t getSError(void) {
    
    return _DSER_;
}

void allocateSpace() { // Add space external flash space

    
}

uint16_t getAvailableSpace() { // Compute free space in data-blocks

    return 0;
}

void freeSpace(dspace_t space) { // Clear data-space - LOST ALL STORED DATAS !!!

}

dchain_t * listChains() { // List all stored data's chain 

    return 0;
}

chainid_t newChain(dchain_t * chain) { // creat a new data's chain (0=last+1)

    return 0;
}

void freeChain(dchain_t chain) { // delete data's chain

}

void saveBuffer(void * dbuffer, dchain_t chain) {


}

void addBuffer(void * dbuffer, dchain_t chain) {


}

void loadBuffer(dchain_t chain, void * dbuffer) {


}
*/
