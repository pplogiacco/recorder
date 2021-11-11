
/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef STORAGE_H
#define	STORAGE_H

#include <xc.h> // include processor files 

// Use "Data Allocation Table" in main Flash  

/*
#include <stdbool.h>

#define SPACE_MAX_COUNT 8
#define CHAIN_MAX_COUNT 8

#define DATABLOCK_SIZE 96

typedef uint8_t chainid_t;

typedef struct {    // pointer to data 
    uint32_t npage:8;
    uint32_t offset:24;
} dpointer_t;

typedef struct {    // storage
    chainid_t id;
    //uint8_t hwdev;  // Internal, i2c, SPI, SQI
    //uint8_t mau;    // Min allocation unit (byte)
    dblock_t pfree; // Free space 
    dblock_t pfirstch, plastch; // Chains
} dstorage_t;

typedef struct {    // data space 
    chainid_t id;
    uint8_t hwdev;  // Internal, i2c, SPI, SQI
    uint8_t mau;    // Min allocation unit (byte)
    uint32_t pstart,pend;

    char * pfirstch, plastch; // dchain_t
} dspace_t;

typedef struct {    // data block item
    chainid_t id;
    char * pprev, pnext; // dblock_t
    uint16_t data[DATABLOCK_SIZE];
} dblock_t;

typedef struct {    // Chain item
    chainid_t id;
    dblock_t * phead;
    char * pprev, pnext; // dchain_t
} dchain_t;



void allocateSpace(); // Add space  flash space
uint16_t getAvailableSpace(); // Compute free space in data-blocks

void freeSpace(dspace_t space); // Clear data-space - LOST ALL DATAS !!!
uint8_t getSError(void); // Error register 

dchain_t * listChains(); // List all stored data's chain 
chainid_t newChain(dchain_t * chain); // creat a new data's chain
void freeChain(dchain_t chain); // delete data's chain

void saveBuffer(void * dbuffer, dchain_t chain); // 
void addBuffer(void * dbuffer, dchain_t chain);
void loadBuffer(dchain_t chain, void * dbuffer);

//void ram2chain(void * dbuffer, chainid_t chain); // cpy to storage
//void chain2ram(chainid_t chain, void * dbuffer); // get from storage
//void freechain(chainid_t chain); // free chain's space 
//uint chaincount(); // Chains counter
//uint chainfirst(); // free chain's space 
//uint chainlast(); // free chain's space 
*/

#endif	// STORAGE_H




////////////////////uint8_t const _nvm24 = 3; // data-unit 
////////////////////
////////////////////uint8_t write_nvm24(uint16_t page, uint16_t address, uint8_t * towrite) {
////////////////////    //////    bool result = true;
////////////////////    //////    uint32_t nvm_address; // 24 bit address
////////////////////    //////    uint16_t iR, iF;
////////////////////    //////
////////////////////    //////    uint32_t towrite[2U];
////////////////////    //////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & measurement);
////////////////////    //////    FLASH_Unlock(FLASH_UNLOCK_KEY);
////////////////////    //////
////////////////////    //////    result = FLASH_ErasePage(nvm_address);
////////////////////    //////
////////////////////    //////    if (result) {
////////////////////    //////        iR = 0; // Bytes
////////////////////    //////        iF = iR; // Words
////////////////////    //////
////////////////////    //////
////////////////////    //////        //        while (iR < ramConfigSize) { //  Sizeof(config_t)
////////////////////    //////        //            towrite[0] = *(pSrc + iR + 2);
////////////////////    //////        //            towrite[0] <<= 8;
////////////////////    //////        //            towrite[0] |= *(pSrc + iR + 1);
////////////////////    //////        //            towrite[0] <<= 8;
////////////////////    //////        //            towrite[0] |= *(pSrc + iR);
////////////////////    //////        //            towrite[1] = *(pSrc + iR + 5);
////////////////////    //////        //            towrite[1] <<= 8;
////////////////////    //////        //            towrite[1] |= *(pSrc + iR + 4);
////////////////////    //////        //            towrite[1] <<= 8;
////////////////////    //////        //            towrite[1] |= *(pSrc + iR + 3);
////////////////////    //////        //            iR += 6;
////////////////////    //////        //
////////////////////    //////        //            //            printf("%u %u \n", (uint16_t) (w0>>16)& 0xFFFF, (uint16_t)w0 & 0xFFFF);
////////////////////    //////        //            //            printf("%u %u \n", (uint16_t) (w1>>16)& 0xFFFF , (uint16_t)w1 & 0xFFFF);         
////////////////////    //////        //            result &= FLASH_WriteDoubleWord24(nvm_address + iF, towrite[0], towrite[1]);
////////////////////    //////        //            iF += 4U;
////////////////////    //////        //        }
////////////////////    //////
////////////////////    //////    }
////////////////////    //////    FLASH_Lock();
////////////////////    //////    //_LATB2 = 1;
////////////////////}
////////////////////
////////////////////uint16_t read_nvm24(uint16_t page, uint16_t address, uint16_t nbytes, uint8_t * rdata) {
////////////////////
////////////////////    //////////#ifdef __NOFLASH
////////////////////    //////////    Device_ConfigDefaultSet(config);
////////////////////    //////////#else    // Read flash/eeprom 
////////////////////    //////////    uint32_t rdata;
////////////////////    //////////    uint32_t nvm_address;
////////////////////    //////////    uint8_t * pDst;
////////////////////    //////////    uint16_t iR, i, iF;
////////////////////    //////////
////////////////////    //////////    pDst = (uint8_t*) config;
////////////////////    //////////    nvm_address = FLASH_GetErasePageAddress((uint32_t) & nvmConfigDatas);
////////////////////    //////////
////////////////////    //////////    iR = 0;
////////////////////    //////////    iF = iR;
////////////////////    //////////    while (iR < ramConfigSize) { // && iF << flash page
////////////////////    //////////        rdata = FLASH_ReadWord24(nvm_address + iF);
////////////////////    //////////        //printf("%u %u \n", (uint16_t) (rdata>>16) & 0xFFFF , (uint16_t) rdata & 0xFFFF);
////////////////////    //////////        i = 0;
////////////////////    //////////        while ((i < 3) && (iR + i < ramConfigSize)) {
////////////////////    //////////            *(pDst + iR + i) = (rdata & 0xFF);
////////////////////    //////////            rdata >>= 8;
////////////////////    //////////            i++;
////////////////////    //////////        }
////////////////////    //////////        iR += i;
////////////////////    //////////        iF += 2U;
////////////////////    //////////    }
////////////////////    //////////
////////////////////    //////////    if (config->CRC16 != computeCRC16((uint8_t *) config, sizeof (config_t) - 2)) { /// Check XOR checksum
////////////////////    //////////        Device_ConfigDefaultSet(config);
////////////////////    //////////        config->CRC16 = computeCRC16((uint8_t *) config, sizeof (config_t) - 2);
////////////////////    //////////        Device_ConfigWrite((uint8_t*) config); // Write EEprom/Flash
////////////////////    //////////    }
////////////////////    //////////#endif
////////////////////    //////////
////////////////////    //////////#ifdef __VAMP1K_TEST_CONFIG
////////////////////    //////////    printf("Read:\n");
////////////////////    //////////    printConfig();
////////////////////    //////////    _LATB2 = 1; // Led on
////////////////////    //////////#endif
////////////////////    //////////    
////////////////////    //////////    return result;
////////////////////
////////////////////}


//#define DEPOT_ADDR 0x0000
//#define DEPOT_SIZE  0x000

/*

 Device             Program Memory          Write Blocks(1)     Erase Blocks(1)
                    Upper Boundary
                    (Instruction Words)
-------------------------------------------------------------------------------
PIC24FJ256GA70X     02AFFEh (88,064 x 24)       688                 86

- One Write Block = 128 Instruction Words;
- One Erase Block (Page) = 1024 Instruction Words.
  
 +-----------------------------+ 
 | box                         |
 +------+---------------+------+
 | head |    databox    | next |
 +------+---------------+------+
  
 +-----------------------+
 | head             |
 +-----------------------+ 
 |b31,b30|b29-b24|b23,b0 |  
 +-------+-------+-------+
 | state | ....  | size  |
 +-------+-------+-------+
  
  
 +-----------------------+
 | next             |
 +-----------------------+ 
 |b31,b30|b29-b24|b23,b0 |  
 +-------+-------+-------+
 | state | bank  | addr  |
 +-------+-------+-------+
  
 
    BTY ( Bank Type - Flash/Eeprom  )
    AUS ( Allocation Unit Size - bytes ) 
   
    [HDDDDDNHDDDDDNHddddddddNHDDDDDDN.....]
     ^      ^      ^         ^       
     |      |      |         |
    Hp      H1     H2        H3                
  

 */
