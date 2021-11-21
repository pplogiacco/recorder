#include "SST26VF064B.h"

#include "../modules/SPI1.h"
#include "../device.h"
#include "../utils.h"   // delay_us

static bool disable_spi1; // Shared SPI1

void SST26_Enable() {
    SST26_SS_SetDigitalOutputHigh();
    disable_spi1 = SPI1_Enable(MODE0, SPI_2MHZ); // Shared SPI1
}

void SST26_Disable() {
    SST26_SS_SetHigh();
    if (disable_spi1) { // Shared SPI1
        SPI1_Disable();
    }
}


void SST26_WREN();
void SST26_WRDI();

void SST26_Switch_Power() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0xAB); // send RDSR command 
    //byte = SPI1_Exchange8bit(0xFF); /* receive byte */
    SST26_SS_SetHigh(); /* disable device */
}


/***********************************************************************
    Configuration Register:
 
    0 RES Reserved 0 R
    1 IOC I/O Configuration for SPI Mode
        1 = WP# and HOLD# pins disabled
        0 = WP# and HOLD# pins enabled   R/W
    2 RES Reserved 0 R
    3 BPNV Block-Protection Volatility State ( default 1 )
        1 = No memory block has been permanently locked
        0 = Any block has been permanently locked  R
    4 RES Reserved 0 R
    5 RES Reserved 0 R
    6 RES Reserved 0 R
    7 WPEN Write-Protection Pin (WP#) Enable
        1 = WP# enabled
        0 = WP# disabled	
 ************************************************************************/

/*************************************************************************
 *  Status Register
 * 
 * 0  BUSY Write status ( 1=in progress)
 * 1  WEL Latch status ( 1 write-enabled )
 * 2  WSE Erase status ( 1= Suspended-Erase )
 * 3  WSP Program status ( 1=Suspended-Program)
 * 4  WPLD Write Protection Lock-Down status  ( Lock-Down enabled)
 * 5  SEC1 Security ID status ( 1 = locked ) 
 * 6  Reserved
 * 7  BUSY Write operation status ( 1=in progress) 
 *************************************************************************/
unsigned char SST26_Read_Status() {
    unsigned char byte = 0;
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_READ_STATUS_REG); /* send RDSR command */
    byte = SPI1_Exchange8bit(0xFF); /* receive byte */
    SST26_SS_SetHigh(); /* disable device */
    return byte;
}

unsigned char SST26_Read_Configuration() {
    unsigned char byte = 0;
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x35); /* send RDSR command */
    byte = SPI1_Exchange8bit(0xFF); /* receive byte */
    SST26_SS_SetHigh(); /* disable device */
    return byte;
}

void SST26_Write_Status_Register(unsigned int data1, unsigned char datalen) {
    //For data1 - top 8 bits are status reg bits , lower 8 bits are configuration reg bits
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_WRITE_STATUS_REG);
    SPI1_Exchange8bit((data1 >> 8) & 0xFF);
    if (datalen == 2) {
        SPI1_Exchange8bit((data1) & 0xFF);
    }
    SST26_SS_SetHigh(); /* disable device */
}

/************************************************************************
 * PROCEDURE: Wait_Busy							
 * This procedure waits until device is no longer busy (can be used by	
 * Byte-Program, Page-Program, Sector-Erase, Block-Erase and Chip-Erase).
 * 0x80 -> bit 7 "BUSY Write operation status ( 1=in progress)"
 ************************************************************************/
void SST26_Wait_Busy() {
    while ((SST26_Read_Status() & 0x80) == 0x80) // waste time until not busy
       // SST26_Read_Status();
       // Nop();Nop();Nop();Nop();
        _delay_us(10);
}


/************************************************************************/
/* Reset                                                                */

/************************************************************************/
void SST26_ResetEn() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_RESET_ENABLE);
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_Reset() {
    SST26_ResetEn();
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_RESET);
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_NoOp() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_NOOP);
    SST26_SS_SetHigh(); /* disable device */
}


/************************************************************************/
/* Erase                						*/

/************************************************************************/
void SST26_Erase_Chip() // Erases the entire Chip !!!
{
    SST26_SS_SetLow();
    SPI1_Exchange8bit(SST26_CMD_CHIP_ERASE);
    SST26_SS_SetHigh();
}

void SST26_Erase_Sector(unsigned long Dst) {
    // Sector Addresses: Use AMS - A12, remaining address are don?t care, but must be set to VIL or VIH.
    SST26_WREN();
    SST26_SS_SetLow();
    SPI1_Exchange8bit(SST26_CMD_SECTOR_ERASE); // Erase 4 KBytes (000000H-7FFFFFH)
    SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); // Send the sector offset high byte first
    SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
    SPI1_Exchange8bit(Dst & 0xFF);
    SST26_SS_SetHigh();
    SST26_Wait_Busy();
}

void SST26_Erase_Block(unsigned long Dst) // 8Kbyte, 32 KByte or 64 KByte 
{
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_BULK_ERASE_64K); /* send Block Erase command */
    SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); /* send 3 address bytes */
    SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
    SPI1_Exchange8bit(Dst & 0xFF);
    SST26_SS_SetHigh(); /* disable device */
}


/************************************************************************/
/* Write                                                        	*/

/************************************************************************/

void SST26_WREN() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_WRITE_ENABLE); /* send WREN command */
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_WRDI() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_WRITE_DISABLE); /* send WRDI command */
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_Write_Suspend() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_WRITE_SUSPEND);
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_Write_Resume() {
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_WRITE_RESUME);
    SST26_SS_SetHigh(); /* disable device */
}


#include <stdio.h>      // printf x debug

/*
If the target address for the Page-Program instruction is not the beginning of 
the page boundary (A[7:0] are not all zero), and the number of bytes of data
input exceeds or overlaps the end of the address of the page boundary,
the excess data inputs wrap around and will be programmed at the start of that
target page.
 */
uint16_t SST26_Write(uint32_t addr, uint8_t *dbuf, uint16_t dlen) {

    uint16_t available, written;
    uint16_t index;

    index = (addr & 0xFF); // set to page offset 
    addr = addr & 0xFFFFFF00; // Sector/Page Address

//    printf("__Write: %u bytes (addr:%lu, offs: %u) \n", dlen, addr >> 8, index);
    written = 0;

    while (written < dlen) {

        available = (SST26_PAGE_SIZE - index);
//        printf("DISPONIBILI %u \n", available);

        if (index == SST26_PAGE_SIZE) {
            addr = addr + SST26_PAGE_SIZE; // Next Page
//            printf("CAMBIO PAGINA...");
//            printf("(Addr= %lu), DISPONIBILI 256 ! \n", addr >> 8);
            index = 0x0;
        }
        //available = (255 - poffs);
        SST26_WREN();
        SST26_SS_SetLow(); // Select device
        SPI1_Exchange8bit(SST26_CMD_WRITE_PAGE); // send command "Page Program"  
        SPI1_Exchange8bit((addr >> 16) & 0xFF); // send msb first
        SPI1_Exchange8bit((addr >> 8) & 0xFF); // 24-bit page address
        SPI1_Exchange8bit(index); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//        printf("W: ");

        while ((index < SST26_PAGE_SIZE) && (written < dlen)) {
            SPI1_Exchange8bit(*dbuf); // send bytes 
//            printf("%u,", *dbuf);
            dbuf++;
            index++;
            written++;
        }
//        printf("\n");
//        printf("SCRITTI %u \n", written);
        SST26_SS_SetHigh(); // unselect device 
        SST26_Wait_Busy();
    }
    SST26_WRDI();
    return (written);
}


/************************************************************************/
/* Read                                                             	*/
/* The Read instruction, 03H, is supported in SPI bus protocol          */
/* only with clock frequencies up to 40 MHz.                            */

/************************************************************************/
uint16_t SST26_Read(uint32_t addr, uint16_t dlen, uint8_t *dbuf) {
    uint16_t i = 0;


//    printf("__Read: %u bytes (addr:%lu, offs: %lu) \n", dlen, addr >> 8, (addr & 0xFF));
    SST26_SS_SetLow();
    //SPI1_Exchange8bit(SST26_CMD_READ); // Command  + 3 Address bytes 
    SPI1_Exchange8bit(SST26_CMD_HS_READ); // Command  + 3 Address bytes 
    SPI1_Exchange8bit((addr >> 16) & 0xFF); // send msb first
    SPI1_Exchange8bit((addr >> 8) & 0xFF); // 24-bit page address
    SPI1_Exchange8bit(addr & 0xFF); //  8bit page offset
    SPI1_Exchange8bit(0xFF); //  Dummy char
//    printf("R: ");
    while (i < dlen) {

        *dbuf = SPI1_Exchange8bit(0xFF); // receive byte 
//        printf("%u,", *dbuf);
        dbuf++;
        i++;
    }
//    printf("\n");
    SST26_SS_SetHigh(); // disable device 
    return (i);
}

void SST26_HSRead(uint32_t addr, uint16_t nbytes, uint8_t * dbuf) {
    uint16_t i;
    SST26_SS_SetLow();
    SPI1_Exchange8bit(SST26_CMD_HS_READ); // Command + 3 address bytes + Dummy
    SPI1_Exchange8bit(((addr & 0xFFFFFF) >> 16));
    SPI1_Exchange8bit(((addr & 0xFFFF) >> 8));
    SPI1_Exchange8bit(addr & 0xFF);
    SPI1_Exchange8bit(0xFF); // dummy byte - HS SPI Required !!!
    for (i = 0; i < nbytes; i++) {

        *dbuf = SPI1_Exchange8bit(0xFF); // receive byte 
        dbuf++;
    }
    SST26_SS_SetHigh();
}

/***********************************************************************
 * Security ID
 * 
 * SST26VF064B/064BA offer a 2 KByte Security ID feature divided into two parts:
 *  1)factory-programmed, cannot be changed, 64-bit (0000 ? 0007H)
 *  2)user-programmable, program as desired (0008H ? 07FFH)
 * The Lockout Security ID instruction prevents any future changes to the Security ID. 
 ***********************************************************************/
void SST26_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length) {
    unsigned long i;
    i = 0;
    if (security_length > 2048) {
        security_length = 2048; // Max 2KByte 
    }
    SST26_SS_SetLow();
    SPI1_Exchange8bit(SST26_CMD_READ_SID); // Read Security ID (88H)
    SPI1_Exchange8bit((Dst >> 8) & 0xFF); // two address cycles, and one dummy
    SPI1_Exchange8bit(Dst & 0xFF);
    SPI1_Exchange8bit(Dst & 0xFF); //dummy

    for (i = 0; i < security_length; i++) {

        *security_ID = SPI1_Exchange8bit(0xFF);
        security_ID++;
    }
    SST26_SS_SetHigh();
}

void SST26_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length) {
    unsigned long i = 0;
    if (security_length > 256) {
        security_length = 256;
    }
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_PROG_SID);
    SPI1_Exchange8bit((Dst >> 8) & 0xFF);
    SPI1_Exchange8bit(Dst & 0xFF);
    for (i = 0; i < security_length; i++) {

        SPI1_Exchange8bit(*security_ID);
        security_ID++;
    }
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_LockSID() {

    /* Prior to the operation execute WREN.
     * To execute a Lockout SID, the host drives CE# low, sends the Lockout Security ID command (85H),
     * then drives CE# high. Poll the BUSY bit in the software status register, or wait TPSID, for the 
     * completion of the Lockout Security ID operation.*/
    SST26_WREN();
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_LOCK_SID);
    SST26_SS_SetHigh(); /* disable device */
    void SST26_Wait_Busy();
}



/************************************************************************/
/* Protection                                                           */

/************************************************************************/

void SST26_Global_Protection_Unlock() {

    SST26_WREN();
    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(SST26_CMD_UNPROTECT_GLOBAL);
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_LockBlockProtection() {

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x8d); /* read command */
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_ReadBlockProtection(unsigned int *block_protection_data) {

    unsigned char i;
    i = 0;

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x72);

    for (i = 18; i > 0; i--) {

        *block_protection_data = SPI1_Exchange8bit(0xFF);
        block_protection_data++;
    }
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_WriteBlockProtection(unsigned int *block_protection_data) {

    unsigned char i;
    i = 0;

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x42); /* read command */

    for (i = 18; i > 0; i--) {

        SPI1_Exchange8bit(*block_protection_data);
        block_protection_data++;
    }
    SST26_SS_SetHigh(); /* disable device */
}

void SST26_NonVolWriteLockProtection(unsigned int *block_protection_data) {

    unsigned char i;
    i = 0;

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0xE8); /* read command */

    for (i = 18; i > 0; i--) {

        SPI1_Exchange8bit(*block_protection_data);
        block_protection_data++;
    }
    SST26_SS_SetHigh(); /* disable device */
}




/************************************************************************/
/* PROCEDURE:	SPI_SFDP_Read						*/

/************************************************************************/
unsigned char SST26_SFDP_Read(unsigned long Dst) {

    unsigned char byte = 0;

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x5A); /* read command */
    SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); /* send 3 address bytes */
    SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
    SPI1_Exchange8bit(Dst & 0xFF);
    SPI1_Exchange8bit(0xFF); /*dummy byte*/
    byte = SPI1_Exchange8bit(0xFF);
    SST26_SS_SetHigh(); /* disable device */
    return byte; /* return one byte read */
}




/************************************************************************/
/* PROCEDURE: QuadJ_ID							*/
/*									*/
/* This procedure Reads the manufacturer's ID, device Type and device ID.  It will 	*/
/* use AFh as the command to read the ID.                               */
/* Returns:								*/
/*	ID1(Manufacture's ID = BFh, Device Type =26h , Device ID = 02h)	*/
/*									*/

/************************************************************************/

void SST26_Jedec_ID_Read(int *Manufacturer_Id, int *Device_Type, int *Device_Id) {

    SST26_SS_SetLow(); /* enable device */
    SPI1_Exchange8bit(0x9F); /* send JEDEC ID command (9Fh) */
    *Manufacturer_Id = SPI1_Exchange8bit(0xFF); /* receive byte */
    *Device_Type = SPI1_Exchange8bit(0xFF); /* receive byte */
    *Device_Id = SPI1_Exchange8bit(0xFF); /* receive byte */
    SST26_SS_SetHigh(); /* disable device */

}


