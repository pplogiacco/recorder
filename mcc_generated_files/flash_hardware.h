/**
  FLASH Generated Driver API Header File

  @Company
    Microchip Technology Inc.

  @File Name
    flash_hardware.h

  @Summary
    This is the generated header file for the FLASH driver using Code Configurator Library(CCL)

  @Description
    This header file provides APIs for driver for FLASH.
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.61
        MPLAB 	          :  MPLABX v5.45
*/

#ifndef FLASH_HARDWARE_H
#define FLASH_HARDWARE_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define FLASH_UNLOCK_KEY  0x00AA0055
#define FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS  128
#define FLASH_WRITE_ROW_SIZE_IN_PC_UNITS (FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2)
#define FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS 1024
#define FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS  (FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS*2)
#define FLASH_HAS_ECC  1
#define FLASH_ERASE_PAGE_MASK (~((FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS*2) - 1)) 

/**
  @Summary
    Erases a page.

  @Description
    This routine erases a page indicated by the page aligned address.

  @Param
    address - Flash page aligned address.

  @Returns
    bool - Returns True if operation successful else returns false.
 
  @Example 
    <code>
    FLASH_ErasePage(uint32_t address);
    </code>
*/
bool FLASH_ErasePage(uint32_t address);

/* The address in the Read and Write word functions below must be even */
/**
  @Summary
    Reads a 24 bit instruction from the indicated address in flash.

  @Description
    This routine reads a 24 bit instruction from the indicated address in flash.

  @Param
    address - Flash address.

  @Returns
    uint32_t - 24 bit data.
 
  @Example 
    <code>
    uint32_t data; FLASH_ReadWord24(uint32_t address);
    </code>
*/
uint32_t FLASH_ReadWord24(uint32_t address);

/**
  @Summary
    Reads a 16 bit instruction from the indicated address in flash.

  @Description
    This routine reads a 16 bit instruction from the indicated address in flash.

  @Param
    address - Flash address.

  @Returns
    uint16_t - 16 bit data.
 
  @Example 
    <code>
    uint16_t data; FLASH_ReadWord16(uint32_t address);
    </code>
*/
uint16_t FLASH_ReadWord16(uint32_t address);

/**
  @Summary
    Writes two 24 bit instructions to the indicated address in flash.

  @Description
    This routine writes two 24 bit instructions to the indicated address in flash.

  @Param
    address - Flash address.
    data1   - First 24 bit data to be written(First word).
    data2   - Second 24 bit data to be written(Second word).

  @Returns
    bool - Returns true if operation is successful else returns false.
 
  @Example 
    <code>
    uint32_t data0 = 0x121212;
    uint32_t data1 = 0x212121;
    FLASH_WriteDoubleWord24(uint32_t address, uint32_t Data0, uint32_t Data1);
    </code>
*/
bool FLASH_WriteDoubleWord24(uint32_t address, uint32_t Data0, uint32_t Data1 );

/**
  @Summary
    Writes two, 16 bit instructions to the indicated address in flash.

  @Description
    This routine writes two ,16 bit instructions to the indicated address in flash.

  @Param
    address - Flash address.
    data1   - First 16 bit data to be written(First word).
    data2   - Second 16 bit data to be written(Second word).

  @Returns
    bool - Returns true if operation is successful else returns false.
 
  @Example 
    <code>
    uint32_t data0 = 0x1212;
    uint32_t data1 = 0x2121;
    FLASH_WriteDoubleWord16(uint32_t address, uint16_t Data0, uint16_t Data1);
    </code>
*/
bool FLASH_WriteDoubleWord16(uint32_t flashAddress, uint16_t Data0, uint16_t Data1);


/* Program the flash one row at a time. */

/**
  @Summary
    Writes a single row of data from the location given in *data to the flash location in address.

  @Description
    This routine writes a single row of data from the location given in *data to the flash location in address. 
    Since the flash is only 24 bits wide all data in the upper 8 bits of the source will be lost.The address in 
    *data must be row aligned.

  @Param
    address - Flash address.
    *data    - Address of the data to be written.

  @Returns
    bool - Returns true if operation is successful else returns false.
 
  @Example 
    <code>
    uint32_t data[FLASH_WRITE_INSTRUCTION_COUNT];
    for(int index=0;index<ROW_SIZE;index++)
    {
      data[index] = index;
    }
    FLASH_WriteRow24(uint32_t address, &data);
    </code>
*/
bool FLASH_WriteRow24(uint32_t flashAddress, uint32_t *data);

/**
  @Summary
    Writes a single row of data from the location given in *data to the flash location in address.

  @Description
    This routine writes a single row of data from the location given in *data to the flash location in address. 
    Each 16 bit source data word is stored in the lower 16 bits of each flash entry and the upper 8 bits of the 
    flash is not programmed.The address in *data must be row aligned.

  @Param
    address - Flash address.
    *data   - Address of the data to be written.

  @Returns
    bool - Returns true if operation is successful else returns false.
 
  @Example 
    <code>
    uint16_t data[FLASH_WRITE_INSTRUCTION_COUNT];
    for(int index=0;index<ROW_SIZE;index++)
    {
      data[index] = index;
    }
    FLASH_WriteRow16(uint32_t address, &data);
    </code>
*/
bool FLASH_WriteRow16(uint32_t address, uint16_t *data);

/**
  @Summary
    Returns the page offset the given flash address.

  @Description
    This routiner eturns the page offset the given flash address.

  @Param
    address - Flash address.

  @Returns
    uint32_t - returns offset.
 
  @Example 
    <code>
    uint32_t offset = FLASH_GetErasePageOffset(uint32_t address);
    </code>
*/
uint16_t FLASH_GetErasePageOffset(uint32_t address);

/**
  @Summary
    Returns the page aligned address for a given flash address.

  @Description
    This routine returns the page aligned address for a given flash address..

  @Param
    address - Flash address.

  @Returns
    uint32_t - Returns page aligned flash address.
 
  @Example 
    <code>
    FLASH_GetErasePageAddress(uint32_t address);
    </code>
*/
uint32_t FLASH_GetErasePageAddress(uint32_t address);

#endif	/* FLASH_HARDWARE_H */

