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

#include <stddef.h>
#include "flash.h"

FLASH_RETURN_STATUS FLASH_EraseBlock(uint32_t flashAddress, size_t count)
{
    bool status = false;
    uint32_t index=0;
    uint32_t address = flashAddress;

    if (address % FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS != 0)
    {
        return FLASH_INVALID_ADDRESS;
    }
    for (index = 0; index < count; index++)
    {
        status = FLASH_ErasePage(address);
        if (!status)
        {
            return FLASH_INVALID_ADDRESS;
        }
        address = address + FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS;
    }
    return FLASH_NO_ERROR;
}


FLASH_RETURN_STATUS FLASH_WriteInstructions(uint32_t flashAddress, FLASH_INSTRUCTION_TYPE *instructions)
{
    if (instructions == NULL)
    {
        return FLASH_INVALID_DATA;
    }
    if (flashAddress % 2 != 0)
    {
        return FLASH_INVALID_ADDRESS;
    }
    return FLASH_WriteDoubleWord24(flashAddress, *instructions, *(instructions+1)) ? FLASH_NO_ERROR : FLASH_WRITE_ERROR;
}


FLASH_RETURN_STATUS FLASH_ReadInstructions(uint32_t flashAddress, FLASH_INSTRUCTION_TYPE *instructions)
{
    if (instructions == NULL)
    {
        return FLASH_INVALID_DATA;
    }
    if (flashAddress % 2 != 0)
    {
        return FLASH_INVALID_ADDRESS;
    }
    *instructions = FLASH_ReadWord24(flashAddress);
    return FLASH_NO_ERROR;
}


uint32_t FLASH_GetEraseBlockOffset(uint32_t flashAddress)
{
    return FLASH_GetErasePageOffset(flashAddress);
}


uint32_t FLASH_GetEraseBlockAddress(uint32_t flashAddress)
{
    return FLASH_GetErasePageAddress(flashAddress);
}

