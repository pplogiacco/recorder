;/*******************************************************************************
;  PIC24 and dsPIC33 Assembly language optimized for flash acces
;*******************************************************************************/


    .equ    WRITE_DWORD_CODE,    0x4001
    .equ    ERASE_PAGE_CODE,     0x4003
    .equ    FLASH_WRITE_ROW_CODE, 0x4002
    .equ    ERASE_WRITE_OP_MASK, 0x404F
    .equ    FLASH_ERASE_WRITE_OP_MASK, 0x404F
    .equ    FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS,  128
    .equ    FLASH_ERASE_PAGE_SIZE_IN_INSTRUCTIONS, 1024
    .equ    FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS,     1024*2
    .equ    FLASH_ERASE_PAGE_MASK,        (~(FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS - 1)) 

    .data
    .global _FlashKey
_FlashKey: .long 0xFFFFFFFF

    .text
    .pushsection    .NVM_flash, code	


/**
 * void  FLASH_Unlock(uint32_t key);
 */    
       
    .global         _FLASH_Unlock
    .type           _FLASH_Unlock, @function
    reset
 _FLASH_Unlock:
    mov     #_FlashKey, W2
    mov     W0, [W2++]
    mov     W1, [W2]
    return;
    
/**
 * void  FLASH_Lock();
 * Locks the flash from programming by resetting the NVMKEY in memory
 */    
    
    .global         _FLASH_Lock
    .type           _FLASH_Lock, @function
    .extern         NVMKEY
    
    reset
 _FLASH_Lock:
    clr W0
    clr W1
    rcall _FLASH_Unlock
    clr NVMKEY    
    return;

 /**
 * void FLASH_SendNvmKey(uint32_t key);
 * Procedure to take the NVM key from memory and program the flash controller
 * with it.  A valid key is 0x00AA0055
 */    
    reset       
    .global         _FLASH_SendNvmKey
    .type           _FLASH_SendNvmKey, @function
    .extern         NVMKEY
    .extern         TBLPAG
    
    reset  
_FLASH_SendNvmKey:
    push    W0
    push    W1
    push    W2

    mov    #_FlashKey, w1

    ; Disable interrupts
    mov    INTCON2, W2	    ; Save Global Interrupt Enable bit.
    bclr   INTCON2, #15	    ; Disable interrupts

    ; Write the KEY sequence
    mov    [W1++], W0
    mov    W0,     NVMKEY
    mov    [W1],   W0
    mov    W0,     NVMKEY
    bset   NVMCON, #15
    
    ; Insert two NOPs after programming
    nop
    nop
    
    ; Wait for operation to complete
prog_wait:
    btsc NVMCON, #15
    bra prog_wait
    
   ; Re-enable interrupts,
    btsc    W2,#15
    BSET    INTCON2, #15    ; Restore Global Interrupt Enable bit.

    pop    W2
    pop    W1
    pop    W0
    return
    


/**
 * uint32_t _FLASH_ReadWord24(uint32_t flashAddress);
 * Reads the 24 bit instruction located at the address passed to this function.
 **/    

    reset    
    .global         _FLASH_ReadWord24
    .type           _FLASH_ReadWord24, @function
    .extern         TBLPAG

    
   _FLASH_ReadWord24:
    mov         TBLPAG, W2
    mov         W1, TBLPAG    ; Little endian, w1 has MSW, w0 has LSX
    tblrdh      [W0], W1      ; read MSW of data to high latch
    tblrdl      [W0], W0      ; read LSW of data 
    mov         W2, TBLPAG    ; Restore register, 
    return

/**
 * ;uint16_t FLASH_ReadWord16(uint32_t flashAddress);
 * Reads the 16 bit instruction located at the address passed to this function.
 * Address must be aligned to an even address.
 **/    

    reset    
    .global         _FLASH_ReadWord16
    .type           _FLASH_ReadWord16, @function
    .extern         TBLPAG

    
   _FLASH_ReadWord16:
    mov         TBLPAG, W2
    mov         W1, TBLPAG     ; Little endian, w1 has MSW, w0 has LSX
    tblrdl      [W0], W0       ; read LSW of data
    mov         W2, TBLPAG     ; restore register
    return
   


/**
 * void FLASH_ErasePage(unsigned long flashAddress);
 * Erases the page containing the specified address. Be very careful if calling
 * this function as the function will erase any legal page, 
 *
 * NOTE: This function can take upwards of 45ms on some device families and 
 * target pages to complete. At frequencies appreciably slower than the maximum 
 * device frequency, even longer may be required. Therefore, you may need to 
 * issue a ClrWdt() call before calling this function, assuming the Watchdog 
 * timer is enabled. This also means that you should not configure a watchdog 
 * timeout of less than ~64ms, even when you pre-clear the timeout. This 
 * function does NOT internally clear the watchdog for you as it is packaged as 
 * a library routine where not everyone would desire such behavior.
 * 
 * @param flashAddress  24-bit (unsigned long) specifying the first address on the page
 *                      to be erased. Must be page aligned.
 * 
 *   Registers used:    w0 w1 w2
 *                      TBLPAG Preserved
 *                  
 *   Inputs:
 *   w0,w1 = long data - Address in flash to erase   (24 bits)
 * 
 *  outputs:
 *   0 - Fail
 *   1 - Pass
 *
 **/ 
    
    .global         _FLASH_ErasePage
    .type           _FLASH_ErasePage, @function
    .extern         TBLPAG
    .extern         NVMCON
    .extern         NVMADRU 
    .extern         NVMADR
   reset



_FLASH_ErasePage:

    mov     #FLASH_ERASE_PAGE_SIZE_IN_PC_UNITS-1, w2     ;    get mask and validate all lower bits = 0
    and     w2, w0, w2
    bra     NZ,3f 

    mov     #ERASE_PAGE_CODE, w2
    mov     w2, NVMCON
    mov     w0, NVMADR
    mov     w1, NVMADRU        ; MSB
    
    call    _FLASH_SendNvmKey

    mov     #1, w0                 ; default return true
    btsc    NVMCON, #13            ; if error bit set, 
3:  mov     #0, w0                 ;   return false
    return;

 /**
 * ;void FLASH_WriteDoubleWord24(uint32_t flashAddress, uint32_t instructionData0, uint32_t instructionData1);
 *  Writes two 24-bit instruction to the flash at the flashAdddress 
 *  Only the lower 24 bits of each instruction will be written since the flash
 *  is only 24 bits wide and all data in the upper 8 bits will be lost.
 *
 *
 * @param flashAddress  32 bit value specifying a target address in flash 
 *                      that this firmware will write to.  It needs to be on a
 *                      addresses divisible by 4.
 *  
 *  @param uint32_t instructionData0  24 bit instruction  to be written first.
 *  @param uint32_t instructionData1  24 bit instruction  to be written second.
 *
 *
 *   Registers used:    w0 w1 w2 w3 w4 w5
 *                      TBLPAG Preserved
 *                  
 *   Inputs:
 *   w0,w1 = long data - Address in flash to write   (24 bits)
 *   w2,w3 = long data - 24 bits of data to write to flash   (24 bits)
 *   w4,w5 = long data - 24 bits of data to write to flash   (24 bits)
 * 
 *  outputs:
 *   none 
 *
 **/ 
  
    .global         _FLASH_WriteDoubleWord24
    .type           _FLASH_WriteDoubleWord24, @function
    .extern         TBLPAG
    .extern         NVMCON
    .extern         NVMADRU
    .extern         NVMADR
    reset    

_FLASH_WriteDoubleWord24:
    btsc    NVMCON, #15     ; Loop, blocking until last NVM operation is complete (WR is clear)
    bra     _FLASH_WriteDoubleWord24

    btsc    w0, #0          ; Check for a valid address Bit 0 and Bit 1 clear
    bra     3f
    btsc    w0, #1
    bra     3f

good24:
    mov     W1,NVMADRU
    mov     W0,NVMADR       
    
    mov     #WRITE_DWORD_CODE, W0 
    mov     W0, NVMCON
    
    mov     TBLPAG, W0          ; save it
    mov     #0xFA,W1
    mov     W1,TBLPAG
    mov     #0,W1
    
                                ; Perform the TBLWT instructions to write the latches
    tblwtl  W2,[W1]
    tblwth  W3,[W1++]
    tblwtl  W4,[W1]
    tblwth  w5,[W1++]
    
    call    _FLASH_SendNvmKey


    mov     W0, TBLPAG

    mov     #1, w0               ; default return true
    btsc    NVMCON, #13          ; if error bit set, 
3:  mov     #0, w0               ;   return false

    return;


   	
 /**
 * ;void FLASH_WriteDoubleWord16(uint32_t flashAddress, uint16_t Data0, uint16_t Data1);
 *  Writes two 16-bit words to the flash at the flashAdddress 
 *  The upper 8 bits of the 24 bit flash entry will have its  data programmed as 
 *  0xFF.
  *
 *
 * @param flashAddress  32 bit value specifying a target address in flash 
 *                      that this firmware will write to.  It needs to be on a
 *                      addresses divisible by 4.
 *  
 *  @param uint16_t Data0  16 bit word to be written first.
 *  @param uint16_t Data1  16 bit word to be written second.
 *
 *   Registers used:    w0 w1 w2 w3
 *                      TBLPAG Preserved
 *                  
 *   Inputs:
 *   w0,w1 = long data - Address in flash to write   (24 bits)
 *   w2    = 16 bit data - 16 bits of data to write to flash   
 *   w3    = 16 bit data - 16 bits of data to write to flash   
 * 
 *  outputs:
 *   none 
 *
 **/
 
    .global    _FLASH_WriteDoubleWord16
    .type      _FLASH_WriteDoubleWord16, @function
    .extern    TBLPAG
    .extern    NVMCON
    .extern    NVMADRU
    .extern    NVMADR
    reset    

 _FLASH_WriteDoubleWord16:
    btsc    NVMCON, #15     ; Loop, blocking until last NVM operation is complete (WR is clear)
    bra     _FLASH_WriteDoubleWord16

    btsc    w0, #0          ; Check for a valid address Bit 0 and Bit 1 clear
    bra     3f
    btsc    w0, #1
    bra     3f

good16:
    mov     W1,NVMADRU
    mov     W0,NVMADR  

    mov     TBLPAG, W1      ; save it
    
    mov     #WRITE_DWORD_CODE, W0 
    mov     W0, NVMCON
    
    mov     #0xFA,W0
    mov     W0,TBLPAG
    mov     #0,W0

    tblwtl  W2,[W0]         ; Perform the TBLWT instructions to write the latches
    mov     #0xFF,W2
    tblwth  W2,[W0++]  
    tblwtl  W3,[W0]
    tblwth  W2,[W0++] 
   
    call    _FLASH_SendNvmKey


    mov     w1, TBLPAG

    mov     #1, w0          ; default return true
    btsc    NVMCON, #13     ; if error bit set, 
3:  mov     #0, w0          ;   return false
    return;


   


/**
 * ;void FLASH_WriteRow24(uint32_t flashAddress, uint16_t *data);
 *  Writes a single Row to the address flashAddress from the sourceAddress 
 *  Since this is passed as a 32 bit value
 *  and flash is only 24 bit wide, all data in the upper 8 bits will be lost.
 *
 *
 * @param flashAddress  32 bit value specifying a target address in flash 
 *                      that this firmware will write to.  It needs to be on a
 *                      addresses divisible  by 4.
 *  
 *  @param sourceAddress  16 bit value of the address to read the data from.
 *
 *   Registers used:    w0 w1 w2 w3
 *                      TBLPAG Preserved
 *                  
 *   Inputs:
 *   w0,w1 = long data - Address in flash to write   (24 bits)
 *   w2    = 16 bit - 16 bits of data address to write to flash   (16 bits)
 * 
 *  outputs:
 *   0 - Failed
 *   1 - Passed
 *
 **/   

    .global         _FLASH_WriteRow24
    .type           _FLASH_WriteRow24, @function
    .extern         TBLPAG
    .extern         NVMCON
    .extern         NVMADRU
    .extern         NVMADR
    
    reset
    _FLASH_WriteRow24:
    btsc    NVMCON, #15     ; Loop, blocking until last NVM operation is complete (WR is clear)
    bra     _FLASH_WriteRow24

    mov     #((FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2)-1), w3     ;    get mask and validate all lower bits = 0
    and     w3, w0, w3
    bra     NZ,3f 


    mov     W0,NVMADR       
    mov     W1,NVMADRU
    
    mov     TBLPAG, W1    ; save it

    mov     #0xFA,W0
    mov     W0,TBLPAG

    mov     #FLASH_WRITE_ROW_CODE, W0 
    mov     W0, NVMCON

    mov     #0,W0
    mov     #(FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2), W3

1:
    tblwtl  [W2++],[W0]         ; Perform the TBLWT instructions to write the latches
    tblwth  [W2++],[W0++]
    cp      W3,W0
    bra     NZ, 1b

    call    _FLASH_SendNvmKey


    mov     w1, TBLPAG

    mov     #1, w0               ; default return true
    btsc    NVMCON, #13          ; if error bit set, 
3:  mov     #0, w0               ;   return false
    return;

   





/**
 * ;void FLASH_WriteRow16(uint32_t flashAddress, uint16_t *data);
 *  Writes a single Row to the address flashAddress from the Address *data
 *  There is no data written in bits 24-31 since there is no flash there.
 *  bits 16-23 are programmed as 0xFFs
 *
 *
 * @param flashAddress  32 bit value specifying a target address in flash 
 *                      that this firmware will write to.  It needs to be on a
 *                      addresses divisible by 4.
 *  
 *  @param *data     address of a 16 bit array of read the data from.
 *
 *   Registers used:    w0 w1 w2 w3
 *                      TBLPAG Preserved
 *                  
 *   Inputs:
 *   w0,w1 = long data - Address in flash to write   (24 bits)
 *   w2    = 16 bit - 16 bits of data address to write to flash   (16 bits)
 * 
 *  outputs:
 *   0 - Failed
 *   1 - Passed
 *
 **/   

    .global         _FLASH_WriteRow16
    .type           _FLASH_WriteRow16, @function
    .extern         TBLPAG
    .extern         NVMCON
    .extern         NVMADRU
    .extern         NVMADR
    reset    

    _FLASH_WriteRow16:
    btsc    NVMCON, #15     ; Loop, blocking until last NVM operation is complete (WR is clear)
    bra     _FLASH_WriteRow16

    mov     #((FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2)-1), w3     ;    get mask and validate all lower bits = 0
    and     w3, w0, w3
    bra     NZ,3f 


    mov     W0,NVMADR       
    mov     W1,NVMADRU
    
    mov     TBLPAG, W1	         ; save it

    mov     #0xFA,W0
    mov     W0,TBLPAG

    mov     #FLASH_WRITE_ROW_CODE, W0 
    mov     W0, NVMCON

    mov     #0,W0
    mov     #(FLASH_WRITE_ROW_SIZE_IN_INSTRUCTIONS*2), W3
    mov     #0xFF, W4

1:
    tblwtl  [W2++],[W0]          ; Perform the TBLWT instructions to write the latches
    tblwth  W4    ,[W0++]

    cp      W3,W0
    bra     NZ, 1b

    call    _FLASH_SendNvmKey


    mov     w1, TBLPAG

    mov     #1, w0               ; default return true
    btsc    NVMCON, #13          ; if error bit set, 
3:  mov     #0, w0               ;   return false
    return;
   
   
;uint16_t FLASH_GetErasePageOffset(uint32_t flashAddress)
    .global     _FLASH_GetErasePageOffset
    .type       _FLASH_GetErasePageOffset, @function

_FLASH_GetErasePageOffset:
    mov     #((~FLASH_ERASE_PAGE_MASK) & 0xFFFF), W2
    and     w2, w0, w0
    return

;uint32_t FLASH_GetErasePageAddress(uint32_t flashAddress);

    .global     _FLASH_GetErasePageAddress
    .type       _FLASH_GetErasePageAddress, @function

_FLASH_GetErasePageAddress:
    
    mov     #(FLASH_ERASE_PAGE_MASK & 0xFFFF)	, W2	;LSW
    and     w2, w0, w0
    mov     #((FLASH_ERASE_PAGE_MASK >> 16) & 0xFFFF), W2 ; MSW
    and     w2, w1, w1

    return

   .popsection 
    
