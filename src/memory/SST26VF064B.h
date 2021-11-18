#ifndef SST26VF064B_H
#define	SST26VF064B_H
#include "xc.h"

/* SST26VF064 capacity is 8,388,608 bytes:
 * (2048 sectors) * ( 4096 bytes per sector divided in 16 pages )   
 * (32768 pages) * (256 bytes per page)
 *
 * Addressing ( 0x000000 - 0x7FFFFF )
 * xxxx 0000 0000 0000    2048 Sectors * 16 Pages ( x 256 bytes )
 */

//#define SST26_SECTOR_SHIFT  12    /* Sector size 1 << 15 = 65,536 */
//#define SST26_NSECTORS      2048
//#define SST26_PAGE_SHIFT    8     /* Page size 1 << 8 = 256 */
//#define SST26_NPAGES        32768

#define SST26_SECTOR_SIZE    4096
#define SST26_PAGE_SIZE      256

// Status Register Bits
#define SST26_WIP              (1 << 0)                /* Bit 0: Write in progress */
#define SST26_WEL              (1 << 1)                /* Bit 1: Write enable latch */
#define SST26_WSE              (1 << 2)                /* Bit 2: Write Suspend-Erase Status */
#define SST26_WSP              (1 << 3)                /* Bit 3: Write Suspend-Program Status */
#define SST26_WPLD             (1 << 4)                /* Bit 4: Write Protection Lock-Down Status */
#define SST26_SEC              (1 << 5)                /* Bit 5: Security ID status */
#define SST26_RES              (1 << 6)                /* Bit 6: RFU */
#define SST26_WIP2             (1 << 7)                /* Bit 7: Write in progress */

//#define SST26_DUMMY     0xa5


typedef enum {  // SST26 Commands
    
    // Status & Configuration
    SST26_CMD_READ_STATUS_REG = 0x05, // Command to read the Flash status register (RDSR)
    SST26_CMD_WRITE_STATUS_REG = 0x01, // Command Write-Status Register (WRSR)

    // Reset
    SST26_CMD_RESET_ENABLE = 0x66, // Reset enable command 
    SST26_CMD_RESET = 0x99, // Command to reset the flash 
    SST26_CMD_NOOP = 0x00, // No operation cmd 

    // Erase
    SST26_CMD_CHIP_ERASE = 0xC7, // Command to perform Chip erase 
    SST26_CMD_SECTOR_ERASE = 0x20, // erase one 4K of flash memory
    SST26_CMD_BULK_ERASE_32K = 0x52, // Command to perform Bulk erase 32K
    SST26_CMD_BULK_ERASE_64K = 0xD8,    // Command to perform Bulk erase 64K

    // Write (Program)
    SST26_CMD_WRITE_ENABLE = 0x06,  // Write enable command (WREN)
    SST26_CMD_WRITE_DISABLE = 0x04, // Write
    SST26_CMD_WRITE_SUSPEND = 0xb0, // Write enable command (WREN)
    SST26_CMD_WRITE_RESUME = 0x30,  // Write enable command (WREN)
    SST26_CMD_WRITE_PAGE = 0x02,    // Page Program 256Bytes (000000H - 7FFFFFH)
                                    // (0x32) SQI Quad Page Program
    // Read
    SST26_CMD_READ = 0x03, // SPI Only Read data (000000H - 7FFFFFH)
    SST26_CMD_HS_READ = 0x0B, // High Speed Read data (000000H - 7FFFFFH)

    // Protection
    SST26_CMD_UNPROTECT_GLOBAL = 0x98, // Global Block-Protection Unlock (ULBPR)
    SST26_CMD_READ_BLOCK_PROT = 0x72, //Read Block-Protection Register (RBPR)
    SST26_CMD_WRITE_BLOCK_PROT = 0x42, // Write Block-Protection Register (WBPR)
    SST26_CMD_LOCK_BLOCK_PROT = 0x8D, // Lock-Down Block-Protection Register (LBPR)
    SST26_CMD_NV_LOCK_BLOCK_PROT = 0xE8, // Non-Volatile Write-Lock Lock-Down Register (nVWLDR)

    // Security ID
    SST26_CMD_READ_SID = 0x88, // Command Read Security ID (0000 ? 07FFH)
    SST26_CMD_PROG_SID = 0xA5, // Program Security ID command cycle (A5H)
    SST26_CMD_LOCK_SID = 0x85, // Lockout Security ID command cycle (85H)

    // SPI/Quad 
    SST26_CMD_ENABLE_QUAD_IO = 0x38, // Command to Enable QUAD IO
    SST26_CMD_RESET_QUAD_IO = 0xFF, // Command to Reset QUAD IO 

    // Others        
    SST26_CMD_JEDEC_ID_READ = 0x9F,     /* Command to read JEDEC-ID of the flash device. */
    SST26_CMD_QUAD_JEDEC_ID_READ = 0xAF,     /* QUAD Command to read JEDEC-ID of the flash device. */

} SST26_CMD;


void SST26_Enable();    // Uses shared SPI1 in MODE0
void SST26_Disable();

// Reset 
void SST26_Reset();
void SST26_NoOp();
void SST26_Wait_Busy();

// Status
unsigned char SST26_Read_Status();
unsigned char SST26_Read_Configuration();
void SST26_Write_Status_Register(unsigned int data1, unsigned char datalen);

// Erase
void SST26_Erase_Chip();    // Erases the entire Chip !!!
void SST26_Erase_Sector(unsigned long Dst); // Erase 4 KBytes (000000H-7FFFFFH)
void SST26_Erase_Block(unsigned long Dst); // 8Kbyte, 32 KByte or 64 KByte 

// Write (Program)
void SST26_Write_Suspend();
void SST26_Write_Resume();
uint16_t SST26_Write(uint32_t addr, uint8_t *dbuf, uint16_t dlen);

// Read
uint16_t SST26_Read(uint32_t addr, uint16_t nbytes, uint8_t *dbuf);
void SST26_HSRead(uint32_t addr, uint16_t nbytes, uint8_t *dbuf);


// Security ID
void SST26_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SST26_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length);
void SST26_LockSID();


unsigned char SST26_SFDP_Read(unsigned long Dst); // This procedure reads SFDP Table
void SST26_Jedec_ID_Read(int *Manufacturer_Id, int *Device_Type, int *Device_Id);


void SST26_ReadBlockProtection(unsigned int *block_protection_data);
void SST26_WriteBlockProtection(unsigned int *block_protection_data);
void SST26_Global_Protection_Unlock();
void SST26_LockBlockProtection();
void SST26_NonVolWriteLockProtection(unsigned int *block_protection_data);

#endif	/* SST26VF064B_H */

