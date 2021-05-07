/* 
 * File:   SST26VF064B_functions.h
 * Author: C03009
 *
 * Created on February 21, 2017, 1:27 PM
 */

#ifndef SST26VF064B_H
#define	SST26VF064B_H


unsigned char SST26_Read_Status();
unsigned char SST26_Read_Configuration();
void SST26_WREN();
void SST26_WRDI();
void SST26_Jedec_ID_Read(int *Manufacturer_Id, int *Device_Type, int *Device_Id);
unsigned char SST26_Read(unsigned long Dst);
void SST26_Read_Cont(unsigned long Dst, unsigned long no_bytes, unsigned int *read_data);
unsigned char SST26_HighSpeed_Read(unsigned long Dst);
void SST26_HighSpeed_Read_Cont(unsigned long Dst, unsigned long no_bytes, unsigned int *read_data);
;
void SST26_Page_Program(unsigned long Dst, unsigned int *Prog_data);
void SST26_Chip_Erase();
void SST26_Sector_Erase(unsigned long Dst);
void SST26_Block_Erase(unsigned long Dst);
void SST26_NoOp();
void SST26_ResetEn();
void SST26_Reset();
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
void SST26_Wait_Busy();
unsigned char SST26_SFDP_Read(unsigned long Dst);



#endif	/* SST26VF064B_H */

