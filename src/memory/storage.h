
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

