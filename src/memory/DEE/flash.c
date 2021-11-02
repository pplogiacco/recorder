

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


