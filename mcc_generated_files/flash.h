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
/*
Copyright (c) [2012-2020] Microchip Technology Inc.  

    All rights reserved.

    You are permitted to use the accompanying software and its derivatives 
    with Microchip products. See the Microchip license agreement accompanying 
    this software, if any, for additional info regarding your rights and 
    obligations.
    
    MICROCHIP SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
    WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
    LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
    AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP OR ITS
    LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT 
    LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE 
    THEORY FOR ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT 
    LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, 
    OR OTHER SIMILAR COSTS. 
    
    To the fullest extend allowed by law, Microchip and its licensors 
    liability will not exceed the amount of fees, if any, that you paid 
    directly to Microchip to use this software. 
    
    THIRD PARTY SOFTWARE:  Notwithstanding anything to the contrary, any 
    third party software accompanying this software is subject to the terms 
    and conditions of the third party's license agreement.  To the extent 
    required by third party licenses covering such third party software, 
    the terms of such license will apply in lieu of the terms provided in 
    this notice or applicable license.  To the extent the terms of such 
    third party licenses prohibit any of the restrictions described here, 
    such restrictions will not apply to such third party software.
*/

#ifndef FLASH_H
#define FLASH_H

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

