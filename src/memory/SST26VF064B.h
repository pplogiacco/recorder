#ifndef SST26VF064B_H
#define	SST26VF064B_H
#include "xc.h"
/*
  Status register:
     0  BUSY Write status ( 1=in progress)
     1  WEL Latch status ( 1 write-enabled )
     2  WSE Erase status ( 1= Suspended-Erase )
     3  WSP Program status ( 1=Suspended-Program)
     4  WPLD Write Protection Lock-Down status  ( Lock-Down enabled)
     5  SEC1 Security ID status ( 1 = locked ) 
     6  Reserved
     7  BUSY Write operation status ( 1=in progress)

  Configuration register:
 
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

 * 

    memory array is organized in uniform, 4 KByte erasable sectors with the following
    erasable blocks: eight 8 KByte parameter, two 32 KByte overlay, and one-hundred 
    twenty-six 64 KByte overlay blocks.

    22AH A23:A16 02H Byte Program or Page Program
 */


#define SPIFLASH_WRITEENABLE      0x06        // write enable
#define SPIFLASH_WRITEDISABLE     0x04        // write disable

#define SPIFLASH_BLOCKERASE_4K    0x20        // erase one 4K block of flash memory
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 64K block of flash memory
#define SPIFLASH_CHIPERASE        0x60        // chip erase (may take several seconds depending on size)
                                              // but no actual need to wait for completion (instead need to check the status register BUSY bit)
#define SPIFLASH_STATUSREAD       0x05        // read status register
#define SPIFLASH_STATUSWRITE      0x01        // write status register
#define SPIFLASH_ARRAYREAD        0x0B        // read array (fast, need to add 1 dummy byte after 3 address bytes)
#define SPIFLASH_ARRAYREADLOWFREQ 0x03        // read array (low frequency)

#define SPIFLASH_SLEEP            0xB9        // deep power down
#define SPIFLASH_WAKE             0xAB        // deep power wake up
#define SPIFLASH_BYTEPAGEPROGRAM  0x02        // write (1 to 256bytes)
#define SPIFLASH_IDREAD           0x9F        // read JEDEC manufacturer and device ID (2 bytes, specific bytes for each manufacturer and device)

#define SPIFLASH_MACREAD          0x4B        // read unique ID number (MAC)




void SST26_Enable();
void SST26_Disable();
void SST26_Switch_Power();


void SST26_ResetEn();
void SST26_NoOp();
void SST26_Reset();

unsigned char SST26_Read_Status();
unsigned char SST26_Read_Configuration();

unsigned char SST26_SFDP_Read(unsigned long Dst);   // This procedure reads SFDP Table
void SST26_Jedec_ID_Read(int *Manufacturer_Id, int *Device_Type, int *Device_Id);

void SST26_Wait_Busy();
void SST26_WREN();
void SST26_WRDI();

// Read
unsigned char SST26_Read(unsigned long Dst);
unsigned char SST26_HighSpeed_Read(unsigned long Dst);
void SST26_HighSpeed_Read_Cont(unsigned long Dst, unsigned long no_bytes, unsigned int *read_data);
void SST26_Read_Cont(unsigned long Dst, unsigned long no_bytes, unsigned int *read_data);

uint16_t SST26_Read_Bytes(uint32_t addr, uint16_t nbytes, uint8_t *dbuf);

// Erase/Write
void SST26_Chip_Erase();
void SST26_Sector_Erase(unsigned long Dst);
void SST26_Block_Erase(unsigned long Dst);

void SST26_Page_Program(unsigned long Dst, unsigned char *Prog_data);
uint16_t SST26_Write_Bytes(uint32_t addr, uint8_t *dbuf, uint16_t dlen);

void SST26_Write_Suspend();
void SST26_Write_Resume();

void SST26_Write_Status_Register(unsigned int data1, unsigned char datalen);

void SST26_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SST26_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SST26_LockSID();

void SST26_ReadBlockProtection(unsigned int *block_protection_data);
void SST26_WriteBlockProtection(unsigned int *block_protection_data);
void SST26_Global_Block_Protection_Unlock();
void SST26_LockBlockProtection();
void SST26_NonVolWriteLockProtection(unsigned int *block_protection_data);




#endif	/* SST26VF064B_H */

