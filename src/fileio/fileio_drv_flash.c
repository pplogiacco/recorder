// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2015 Microchip Technology Inc. (www.microchip.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

To request to license the code under the MLA license (www.microchip.com/mla_license), 
please contact mla_licensing@microchip.com
 *******************************************************************************/
//DOM-IGNORE-END
#include "xc.h"
#include <string.h>

#include "fileio_config.h"
#include "fileio.h"
#include "fileio_drv_flash.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Global Variables
 *****************************************************************************/

static FILEIO_MEDIA_INFORMATION mediaInformation;

/******************************************************************************
 * Prototypes
 *****************************************************************************/
void EraseBlock(const uint8_t* dest);
void WriteRow(void);
void WriteByte(unsigned char);

FILEIO_MEDIA_INFORMATION * MediaInitialize(void);
void UnlockAndActivate(uint8_t);

//Arbitrary, but "uncommon" value.  Used by UnlockAndActivateWR() to enhance robustness.
#define NVM_UNLOCK_KEY  (uint8_t)0xB5

/******************************************************************************
 * Function:        uint8_t MediaDetect(void* config)
 *
 * PreCondition:    InitIO() function has been executed.
 *
 * Input:           void
 *
 * Output:          true   - Card detected
 *                  false   - No card detected
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint8_t FILEIO_InternalFlash_MediaDetect(void* config) {
    return true;
}//end MediaDetect

/******************************************************************************
 * Function:        uint16_t SectorSizeRead(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          uint16_t - size of the sectors for this physical media.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint16_t FILEIO_InternalFlash_SectorSizeRead(void* config) {
    return FILEIO_CONFIG_MEDIA_SECTOR_SIZE;
}

/******************************************************************************
 * Function:        uint32_t ReadCapacity(void)
 *
 * PreCondition:    MediaInitialize() is complete
 *
 * Input:           void
 *
 * Output:          uint32_t - size of the "disk" - 1 (in terms of sector count).
 *                  Ex: In other words, this function returns the last valid 
 *                  LBA address that may be read/written to.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint32_t FILEIO_InternalFlash_CapacityRead(void* config) {
    //The SCSI READ_CAPACITY command wants to know the last valid LBA address 
    //that the host is allowed to read or write to.  Since LBA addresses start
    //at and include 0, a return value of 0 from this function would mean the 
    //host is allowed to read and write the LBA == 0x00000000, which would be 
    //1 sector worth of capacity.
    //Therefore, the last valid LBA that the host may access is 
    //DRV_FILEIO_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1.

    return ((uint32_t) DRV_FILEIO_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1);
}

/******************************************************************************
 * Function:        uint8_t InitIO(void)
 *
 * PreCondition:    None
 *
 * Input:           void
 *
 * Output:          true   - Card initialized
 *                  false   - Card not initialized
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 *****************************************************************************/
uint8_t FILEIO_InternalFlash_InitIO(void* config) {
    return true;
}

/******************************************************************************
 * Function:        uint8_t MediaInitialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Returns a pointer to a MEDIA_INFORMATION structure
 *
 * Overview:        MediaInitialize initializes the media card and supporting variables.
 *
 * Note:            None
 *****************************************************************************/
FILEIO_MEDIA_INFORMATION * FILEIO_InternalFlash_MediaInitialize(void* config) {
    mediaInformation.validityFlags.bits.sectorSize = true;
    mediaInformation.sectorSize = FILEIO_CONFIG_MEDIA_SECTOR_SIZE;

    mediaInformation.errorCode = MEDIA_NO_ERROR;
    return &mediaInformation;
}//end MediaInitialize


/******************************************************************************
 * Function:        uint8_t SectorRead(uint32_t sector_addr, uint8_t *buffer)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be stored, see
 *                                'ram_acs.h' for 'block' definition.
 *                                'Block' is dependent on whether internal or
 *                                external memory is used
 *
 * Output:          Returns true if read successful, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorRead reads 512 bytes of data from the card starting
 *                  at the sector address specified by sector_addr and stores
 *                  them in the location pointed to by 'buffer'.
 *
 * Note:            The device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/
//The flash memory is organized differently on the different microcontroller
//families.  Therefore, multiple versions of this function are implemented.

#if defined(__C30__)    //PIC24 or dsPIC33 device (uint16_t organized flash memory)

uint8_t FILEIO_InternalFlash_SectorRead(void* config, uint32_t sector_addr, uint8_t* buffer) {
    uint16_t i;
    uint32_t flashAddress;
    uint8_t TBLPAGSave;
    uint16_t temp;

    //Error check.  Make sure the host is trying to read from a legitimate
    //address, which corresponds to the MSD volume (and not some other program
    //memory region beyond the end of the MSD volume).
    if (sector_addr >= DRV_FILEIO_INTERNAL_FLASH_TOTAL_DISK_SIZE) {
        return false;
    }

    //Save TBLPAG register
    TBLPAGSave = TBLPAG;

    //Compute the 24 bit starting address.  Note: this is a word address, but we
    //only store data in and read from the lower word (even LSB).
    //Starting address will always be even, since MasterBootRecord[] uses aligned attribute in declaration.
    flashAddress = (uint32_t) DRV_FILEIO_INTERNAL_FLASH_CONFIG_FILES_ADDRESS + (uint32_t) (sector_addr * (uint16_t) FILEIO_CONFIG_MEDIA_SECTOR_SIZE);

    //Read a sector worth of data from the flash, and copy to the user specified "buffer".
    for (i = 0; i < (FILEIO_CONFIG_MEDIA_SECTOR_SIZE / 2u); i++) {
        TBLPAG = (uint8_t) (flashAddress >> 16); //Load TBLPAG pointer (upper 8 bits of total address.  A sector could get split at
        //a 16-bit address boundary, and therefore could exist on two TBLPAG pages.
        //Therefore, need to reload TBLPAG every iteration of the for() loop
        temp = __builtin_tblrdl((uint16_t) flashAddress);
        memcpy(buffer, &temp, 2);
        buffer += 2;
        flashAddress += 2u; //Increment address by 2.  No MSD data stored in the upper uint16_t (which only has one implemented byte anyway).

    }

    //Restore TBLPAG register to original value
    TBLPAG = TBLPAGSave;

    return true;
}

#endif


/******************************************************************************
 * Function:        uint8_t SectorWrite(uint32_t sector_addr, uint8_t *buffer, uint8_t allowWriteToZero)
 *
 * PreCondition:    None
 *
 * Input:           sector_addr - Sector address, each sector contains 512-byte
 *                  buffer      - Buffer where data will be read
 *                  allowWriteToZero - If true, writes to the MBR will be valid
 *
 * Output:          Returns true if write successful, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        SectorWrite sends 512 bytes of data from the location
 *                  pointed to by 'buffer' to the card starting
 *                  at the sector address specified by sector_addr.
 *
 * Note:            The sample device expects the address field in the command packet
 *                  to be byte address. Therefore the sector_addr must first
 *                  be converted to byte address. This is accomplished by
 *                  shifting the address left 9 times.
 *****************************************************************************/

volatile unsigned char file_buffer[DRV_FILEIO_INTERNAL_FLASH_CONFIG_ERASE_BLOCK_SIZE] __attribute__((far, aligned));

#define INTERNAL_FLASH_PROGRAM_WORD        0x4003
#define INTERNAL_FLASH_ERASE               0x4042
#define INTERNAL_FLASH_PROGRAM_PAGE        0x4001

#define PTR_SIZE uint16_t

const uint8_t *FileAddress = 0;


uint8_t FILEIO_InternalFlash_SectorWrite(void* config, uint32_t sector_addr, uint8_t* buffer, uint8_t allowWriteToZero) {
#if !defined(DRV_FILEIO_CONFIG_INTERNAL_FLASH_WRITE_PROTECT)
    
    uint16_t i;
    uint8_t j;
    uint16_t offset;
    uint32_t flashAddress;
    uint16_t TBLPAGSave;


    //First, error check the resulting address, to make sure the MSD host isn't trying 
    //to erase/program illegal LBAs that are not part of the designated MSD volume space.
    if (sector_addr >= DRV_FILEIO_INTERNAL_FLASH_TOTAL_DISK_SIZE) {
        return false;
    }

    TBLPAGSave = TBLPAG;

    //First, save the contents of the entire erase page.  To do this, we need to get a pointer to the start of the erase page.
    flashAddress = ((uint32_t) DRV_FILEIO_INTERNAL_FLASH_CONFIG_FILES_ADDRESS + (uint32_t) (sector_addr * FILEIO_CONFIG_MEDIA_SECTOR_SIZE)) & (uint32_t) 0xFFFFFC00; //AND mask 0xFFFFFC00 is to clear the lower bits, so we go back to the start of the erase page.
    //Now save all of the contents of the erase page.
    for (i = 0; i < DRV_FILEIO_INTERNAL_FLASH_CONFIG_ERASE_BLOCK_SIZE;) {
        TBLPAG = (uint8_t) (flashAddress >> 16);
        *(uint16_t*)&file_buffer[i] = __builtin_tblrdl((uint16_t) flashAddress);
        flashAddress += 2u; //Skipping upper word.  Don't care about the implemented byte/don't use it when programming or reading from the sector.
        i += 2u;
    }

    //Now we want to overwrite the file_buffer[] contents for the sector that we are trying to write to.
    //Need to figure out if the buffer[] data goes in the upper sector or the lower sector of the file_buffer[]
    if (sector_addr & 0x00000001) {
        //Odd sector address, must be the high file_buffer[] sector
        offset = FILEIO_CONFIG_MEDIA_SECTOR_SIZE;
    } else {
        offset = 0;
    }

    //Overwrite the file_buffer[] RAM contents for the sector that we are trying to write to.
    for (i = 0; i < FILEIO_CONFIG_MEDIA_SECTOR_SIZE; i++) {
        file_buffer[offset + i] = *buffer++;
    }

    //Now erase the entire erase page of flash memory.  
    //First we need to calculate the actual flash memory address of the erase page.  The starting address of the erase page is as follows:
    flashAddress = ((uint32_t) DRV_FILEIO_INTERNAL_FLASH_CONFIG_FILES_ADDRESS + (uint32_t) (sector_addr * FILEIO_CONFIG_MEDIA_SECTOR_SIZE)) & (uint32_t) 0xFFFFFC00;

    //Peform NVM erase operation.
    NVMCON = INTERNAL_FLASH_ERASE; //Page erase on next WR
    __builtin_tblwtl((uint16_t) flashAddress, 0xFFFF); //Perform dummy write to load address of erase page
    UnlockAndActivate(NVM_UNLOCK_KEY);

    //Now reprogram the erase page with previously obtained contents of the file_buffer[]
    //We only write to the even flash word addresses, the odd word addresses are left blank.  
    //Therefore, we only store 2 bytes of application data for every 2 flash memory word addresses.
    //This "wastes" 1/3 of the flash memory, but it provides extra protection from accidentally executing
    //the data.  It also allows quick/convenient PSV access when reading back the flash contents.
    NVMCON = INTERNAL_FLASH_PROGRAM_PAGE;
    j = 0;
    for (i = 0; i < DRV_FILEIO_INTERNAL_FLASH_CONFIG_ERASE_BLOCK_SIZE;) {
        TBLPAG = (uint8_t) (flashAddress >> 16);
        __builtin_tblwtl((uint16_t) flashAddress, *((uint16_t*) & file_buffer[i]));
        flashAddress++;
        __builtin_tblwth((uint16_t) flashAddress, 0);
        flashAddress++;

        i += 2;
        j += 2;

        //Check if we have reached a program block size boundary.  If so, program the last 128 
        //useful bytes (192 bytes total, but 64 of those are filled with '0' filler bytes).
        if (j >= 128u) {
            j = j - 128u;
            asm("DISI #16"); //Disable interrupts for next few instructions for unlock sequence
            __builtin_write_NVM();
        }
    }

    TBLPAG = TBLPAGSave;
#endif    
    return true;
}


#if !defined(DRV_FILEIO_CONFIG_INTERNAL_FLASH_WRITE_PROTECT)

void EraseBlock(const uint8_t* dest) {

}



#ifndef DRV_FILEIO_INTERNAL_FLASH_CONFIG_UNLOCK_VERIFICATION_FUNCTION
#error "User must define the DRV_FILEIO_INTERNAL_FLASH_CONFIG_UNLOCK_VERIFICATION_FUNCTION macro in the fileio_config.h file.  Click this message for more details in comments."
/* The DRV_FILEIO_INTERNAL_FLASH_CONFIG_UNLOCK_VERIFICATION_FUNCTION macro
 * is used to verify that the system is in a condition where a self write 
 * is valid.  This could include checks for system voltage levels, clocking
 * vs voltage, address checking for write locations, etc.  The prototype of
 * the function that this micro should point to is the following:
 *   bool functionName(void);
 * The functions should return true if the self write is allowed and false
 * if the self write is not allowed. 
 */
#endif

bool DRV_FILEIO_INTERNAL_FLASH_CONFIG_UNLOCK_VERIFICATION_FUNCTION(void);


//------------------------------------------------------------------------------

void UnlockAndActivate(uint8_t UnlockKey) {


    if (DRV_FILEIO_INTERNAL_FLASH_CONFIG_UNLOCK_VERIFICATION_FUNCTION() == false) {
        return;
    }

    //Verify the UnlockKey is the correct value, to make sure this function is 
    //getting executed intentionally, from a calling function that knew it
    //should pass the correct NVM_UNLOCK_KEY value to this function.
    //If this function got executed unintentionally, then it would be unlikely
    //that the UnlockKey variable would have been loaded with the proper value.
    if (UnlockKey != NVM_UNLOCK_KEY) {

        return;
    }




#if defined(__C30__)
    asm("DISI #16"); //Disable interrupts for next few instructions for unlock sequence
    __builtin_write_NVM();
#endif

}
#endif  //end of #if !defined(DRV_FILEIO_CONFIG_INTERNAL_FLASH_WRITE_PROTECT)

/******************************************************************************
 * Function:        uint8_t WriteProtectState(void* config)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          uint8_t    - Returns the status of the "write enabled" pin
 *
 * Side Effects:    None
 *
 * Overview:        Determines if the card is write-protected
 *
 * Note:            None
 *****************************************************************************/

uint8_t FILEIO_InternalFlash_WriteProtectStateGet(void* config) {
#if defined(DRV_FILEIO_CONFIG_INTERNAL_FLASH_WRITE_PROTECT)
    return true;
#else
    return false;
#endif
}

