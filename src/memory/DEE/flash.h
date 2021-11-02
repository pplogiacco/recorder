/**
  @Company
    Microchip Technology Inc.

  @Summary
    Lower level peripheral API for the flash module using Code Configurator Library(CCL).

  @Description
    Lower level peripheral API for the flash module using Code Configurator Library(CCL).
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.61
        MPLAB 	          :  MPLABX v5.45
*/


#ifndef FLASH_H
#define FLASH_H


// DDE
#define DATA_EE_BANKS                     1  // 6
#define DATA_EE_SIZE                      127 // 255
#define DATA_EE_TOTAL_SIZE                (DATA_EE_BANKS * DATA_EE_SIZE)
#define NUM_DATA_EE_PAGES                 2
#define	NUMBER_OF_INSTRUCTIONS_IN_PAGE    1024
#define	NUMBER_OF_INSTRUCTIONS_IN_ROW     128
//#define DEE_STRUCTURE_FLASH_START_ADDRESS 0x4000  //

// If the device has ECC
#define __HAS_ECC	                      1
#define ERASE_WRITE_CYCLE_MAX             10000
#define NUMBER_OF_ROWS_IN_PAGE            (_FLASH_PAGE / _FLASH_ROW)
#define PAGE_AVAILABLE                    1
#define PAGE_CURRENT                      0
#define PAGE_EXPIRED                      0
#define PAGE_NOT_AVAILABLE                0
#define PAGE_NOT_CURRENT                  1
#define PAGE_NOT_EXPIRED                  1
#define STATUS_AVAILABLE                  18
#define STATUS_CURRENT                    19
#define STATUS_EXPIRED                    20
#define ERASE_STATE                       0xFFFFFF





#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "flash_hardware.h"


#define FLASH_WRITE_INSTRUCTION_COUNT 2
#define FLASH_INSTRUCTION_TYPE uint32_t

typedef enum{
    FLASH_NO_ERROR,
    FLASH_INVALID_INSTRUCTION_COUNT,
    FLASH_INVALID_ADDRESS,
    FLASH_INVALID_INSTRUCTION_TYPE,
    FLASH_INVALID_DATA,
    FLASH_WRITE_ERROR,
    FLASH_READ_ERROR,
}FLASH_RETURN_STATUS;

/**
  @Summary
    This function is used to program the flash key .

  @Description
    This function is used to program the flash key, which allows the flash to be programmable.

  @Param
    key to be programmed.

  @Returns
    None
 
  @Example 
    <code>
    FLASH_Unlock(key);
    </code>
*/
void FLASH_Unlock(uint32_t  key);
/**
  @Summary
    This function Locks the flash from programming.

  @Description
    This function Locks the flash from programming by resetting the NVMKEY in memory.

  @Param
    None.

  @Returns
    None
 
  @Example 
    <code>
    FLASH_Lock();
    </code>
*/
void FLASH_Lock(void);
/**
  @Summary
    This function erases specified number of blocks containing(starting from) the specified address.

  @Description
    This function erases the page containing the specified address.

  @Param
    flashAddress : Flash address.

  @Returns
    FLASH_RETURN_STATUS
 
  @Example 
    <code>
    FLASH_EraseBlock(flashAddress, count);
    </code>
*/
FLASH_RETURN_STATUS FLASH_EraseBlock(uint32_t flashAddress, size_t count);    

/**
  @Summary
    This function writes the specified data to the specified address.

  @Description
    This function writes the specified data to the specified address.

  @Param
    flashAddress : Flash address.
    instructions : pointer to the data to be written.

  @Returns
    FLASH_RETURN_STATUS
 
  @Example 
    <code>
    uint32_t data[FLASH_WRITE_INSTRUCTION_COUNT];
    for(int index=0;index<ROW_SIZE;index++)
    {
      data[index] = index;
    }
    FLASH_RETURN_STATUS status = FLASH_WriteInstructions(flashAddress), &data);
    </code>
*/
FLASH_RETURN_STATUS FLASH_WriteInstructions(uint32_t flashAddress, FLASH_INSTRUCTION_TYPE *instructions);

/**
  @Summary
    This function reads the data from the specified address.

  @Description
    This function reads the data from the specified address.

  @Param
    flashAddress : Flash address.
    instructions : pointer to the read data.

  @Returns
    FLASH_RETURN_STATUS
 
  @Example 
    <code>
    uint32_t data[2];
    FLASH_RETURN_STATUS status = FLASH_ReadInstructions(flashAddress), &data);
    </code>
*/
FLASH_RETURN_STATUS FLASH_ReadInstructions(uint32_t flashAddress, FLASH_INSTRUCTION_TYPE *instructions);

/**
  @Summary
    This function returns the flash offset .

  @Description
    This function returns the flash offset.

  @Param
    flashAddress : Flash address.

  @Returns
    uint32_t : flash offset
 
  @Example 
    <code>
    uint32_t flashOffset = FLASH_GetEraseBlockOffset(flashAddress));
    </code>
*/
uint32_t FLASH_GetEraseBlockOffset(uint32_t flashAddress);

/**
  @Summary
    This function returns the block aligned address of the given flash address.

  @Description
    This function returns the block aligned address of the given flash address.

  @Param
    flashAddress : Flash address.

  @Returns
    blockAlignedAddress : Block aligned address.
 
  @Example 
    <code>
    uint32_t blockAlignedAddress = FLASH_GetEraseBlockAddress(flashAddress));
    </code>
*/
uint32_t FLASH_GetEraseBlockAddress(uint32_t flashAddress);




#endif	/* FLASH_H */

