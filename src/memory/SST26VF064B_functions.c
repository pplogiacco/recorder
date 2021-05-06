
#include "SST26VF064B_functions.h"


/************************************************************************/
/* PROCEDURE: Read_Status_Register					*/
/*									*/
/* This procedure reads the status register and returns the byte.	*/
/************************************************************************/


unsigned char SPI_Read_Status_Register()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x05);		/* send RDSR command */
	byte = SPI1_Exchange8bit(0xFF);		/* receive byte */
	CE_SetHigh();			/* disable device */
	return byte;
}

/************************************************************************/
/* PROCEDURE: Read_Configuration_Register				*/
/*									*/
/* This procedure reads the configuration register and returns the byte.*/
/************************************************************************/
unsigned char SPI_Read_Configuration_Register()
{
	unsigned char byte = 0;
	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x35);		/* send RDSR command */
	byte = SPI1_Exchange8bit(0xFF);		/* receive byte */
	CE_SetHigh();			/* disable device */
	return byte;
}



/************************************************************************/
/* PROCEDURE: WREN							*/
/*									*/
/* This procedure enables the Write Enable Latch.               	*/
/************************************************************************/


void SPI_WREN()
{
	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x06);		/* send WREN command */
	CE_SetHigh();			/* disable device */
}

/************************************************************************/
/* PROCEDURE: WRDI							*/
/*									*/
/* This procedure disables the Write Enable Latch.			*/
/************************************************************************/

void SPI_WRDI()
{
	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x04);		/* send WRDI command */
	CE_SetHigh();			/* disable device */
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

void SPI_Jedec_ID_Read(int *Manufacturer_Id, int *Device_Type, int *Device_Id)

{
	

	CE_SetLow();			 /* enable device */
	SPI1_Exchange8bit(0x9F);		 /* send JEDEC ID command (9Fh) */
    	 	*Manufacturer_Id = SPI1_Exchange8bit(0xFF);       /* receive byte */
  	*Device_Type = SPI1_Exchange8bit(0xFF);           /* receive byte */
  	*Device_Id = SPI1_Exchange8bit(0xFF);             /* receive byte */
    CE_SetHigh();			 /* disable device */

}



/************************************************************************/
/* PROCEDURE:	Read							*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/
unsigned char SPI_Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_SetLow();                                /* enable device */
	SPI1_Exchange8bit(0x03);                        /* read command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	byte = SPI1_Exchange8bit(0xFF);
	CE_SetHigh();                              /* disable device */
	return byte;                            /* return one byte read */
}


/************************************************************************/
/* PROCEDURE:	Read_Cont						*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/
void SPI_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned int *read_data)
{
	unsigned long i = 0;
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x03); 			/* read command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	{
		*read_data = SPI1_Exchange8bit(0xFF);	/* receive byte and store at address 80H - FFH */
        read_data++;
    }
	CE_SetHigh();				/* disable device */

}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read						*/
/*									*/
/* This procedure reads one address of the device.  It will return the 	*/
/* byte read in variable byte.						*/
/* Input:								*/
/*		Dst:	Destination Address 000000H - 7FFFFFH		*/
/************************************************************************/
unsigned char SPI_HighSpeed_Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_SetLow();                               /* enable device */
	SPI1_Exchange8bit(0x0B);                        /* read command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	SPI1_Exchange8bit(0xFF);                        /*dummy byte*/
	byte = SPI1_Exchange8bit(0xFF);
	CE_SetHigh();                              /* disable device */
	return byte;                            /* return one byte read */
}

/************************************************************************/
/* PROCEDURE:	HighSpeed_Read_Cont					*/
/*									*/
/* This procedure reads multiple addresses of the device and stores	*/
/* data into 256 byte buffer. Maximum number of bytes read is limited to 256 bytes*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*      	no_bytes	Number of bytes to read	(max = 256)	*/
/************************************************************************/
void SPI_HighSpeed_Read_Cont(unsigned long Dst,unsigned long no_bytes, unsigned int *read_data)
{
	unsigned long i = 0;
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x0B); 			/* read command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	SPI1_Exchange8bit(0xFF);			/*dummy byte*/
	for (i = 0; i < no_bytes; i++)		/* read until no_bytes is reached */
	{
		*read_data = SPI1_Exchange8bit(0xFF);	/* receive byte and store at address 80H - FFH */
        read_data++;
    }
	CE_SetHigh();				/* disable device */
}




/************************************************************************/
/* PROCEDURE:	Page_Program						*/
/*									*/
/* This procedure does page programming.  The destination               */
/* address should be provided.                                  	*/
/* The data array of 256 bytes contains the data to be programmed.      */
/* Since the size of the data array is 256 bytes rather than 256 bytes, this page program*/
/* procedure programs only 256 bytes                                    */
/* Assumption:  Address being programmed is already erased and is NOT	*/
/*		block protected.					*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/*		data_256[256] containing 256 bytes of data will be programmed using this function */
/************************************************************************/

void SPI_Page_Program(unsigned long Dst, unsigned int *Prog_data)
{    
	unsigned int i;
	i=0;

	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x02); 			/* send Byte Program command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	for (i=0;i<256;i++)
	{	SPI1_Exchange8bit(*Prog_data);		/* send byte to be programmed */
	Prog_data++;
    }
	CE_SetHigh();				/* disable device */
}


/************************************************************************/
/* PROCEDURE: Chip_Erase						*/
/*									*/
/* This procedure erases the entire Chip.				*/
/************************************************************************/

void SPI_Chip_Erase()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0xC7);			/* send Chip Erase command (C7h) */
	CE_SetHigh();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Sector_Erase						*/
/*									*/
/* This procedure Sector Erases the Chip.				*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/************************************************************************/


void SPI_Sector_Erase(unsigned long Dst)
{


	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x20);			/* send Sector Erase command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	CE_SetHigh();				/* disable device */
}

/************************************************************************/
/* PROCEDURE: Block_Erase						*/
/*									*/
/* This procedure Block Erases 8Kbyte, 32 KByte or 64 KByte of the Chip.*/
/*									*/
/* Input:								*/
/*		Dst:		Destination Address 000000H - 7FFFFFH	*/
/************************************************************************/

void SPI_Block_Erase(unsigned long Dst)
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0xD8);			/* send Block Erase command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	CE_SetHigh();				/* disable device */
}


/************************************************************************/
/* PROCEDURE: NoOp                                              	*/
/*									*/
/* No operation is performed.                                           */
/************************************************************************/

void SPI_NoOp()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x00);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: ResetEn                                                   */
/*									*/
/* This procedure Enables acceptance of the RST (Reset) operation.	*/
/************************************************************************/

void SPI_ResetEn()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x66);
	CE_SetHigh();				/* disable device */
}




/************************************************************************/
/* PROCEDURE: Reset                                     		*/
/*									*/
/* This procedure resets the device in to normal operating Ready mode.	*/
/*									*/
/************************************************************************/


void SPI_Reset()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x99);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Suspend						*/
/*									*/
/* This procedure suspends Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Suspend()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0xb0);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Resume						*/
/*									*/
/* This procedure resumes Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Resume()
{
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x30);
	CE_SetHigh();				/* disable device */
}



/************************************************************************/
/* PROCEDURE: Write_Status_Register					*/
/*									*/
/* This procedure resumes Program/Erase operation.			*/
/************************************************************************/

void SPI_Write_Status_Register(unsigned int data1, unsigned char datalen)
{	  //For data1 - top 8 bits are status reg bits , lower 8 bits are configuration reg bits
	CE_SetLow();				/* enable device */
	SPI1_Exchange8bit(0x01);
	SPI1_Exchange8bit((data1>>8)&0xff);
	if (datalen==2)
	{
		SPI1_Exchange8bit((data1)&0xff);
	}

	CE_SetHigh();				/* disable device */
}


 /************************************************************************/
/* PROCEDURE:	ReadSID	(Read Security ID)				*/
/*									*/
/* This procedure reads the security ID					*/
/************************************************************************/


void SPI_ReadSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length)
{

	unsigned long i;
	i=0;
	 	if (security_length>2048)
	{ security_length=2048;}

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x88);
	SPI1_Exchange8bit((Dst>>8) & 0xFF);
	SPI1_Exchange8bit(Dst & 0xFF);
	SPI1_Exchange8bit(Dst & 0xFF);  //dummy

	for (i=0;i<security_length;i++)
	{
		*security_ID = SPI1_Exchange8bit(0xFF);
		security_ID++;
	}
	CE_SetHigh();			/* disable device */
}



/************************************************************************/
/* PROCEDURE:	ProgSID	(Program Security ID)                           */
/*									*/
/* This procedure programs the security ID				*/
/*									*/
/************************************************************************/

void SPI_ProgSID(unsigned char *security_ID, unsigned long Dst, unsigned long security_length)
{
	unsigned long i;

	i=0;

 	 	if (security_length>256)
	{ security_length=256;}

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0xa5);
 	SPI1_Exchange8bit((Dst>>8) & 0xFF);
	SPI1_Exchange8bit(Dst & 0xFF);


	for (i=0;i<security_length;i++)
	{
	SPI1_Exchange8bit(*security_ID);
	security_ID++;
	}


	CE_SetHigh();			/* disable device */
}




/************************************************************************/
/* PROCEDURE:	LockSID							*/
/*									*/
/* This procedure Locks the security ID setting				*/
/*									*/
/************************************************************************/

void SPI_LockSID()
{

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x85);
	CE_SetHigh();			/* disable device */
}




  /************************************************************************/
/* PROCEDURE:	ReadBlockProtection			  		*/
/*									*/
/* This procedure reads block protection register			*/
/*									*/
/************************************************************************/

void SPI_ReadBlockProtection(unsigned int *block_protection_data)
{
	
 	unsigned char i;
	i=0;

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x72);

	for (i=18;i>0;i--)
	{	*block_protection_data = SPI1_Exchange8bit(0xFF);
    block_protection_data++;
	}
	CE_SetHigh();			/* disable device */
}



/************************************************************************/
/* PROCEDURE:	WriteBlockProtection					*/
/*									*/
/* This procedure writes to block protection register			*/
/*									*/
/************************************************************************/


void SPI_WriteBlockProtection(unsigned int *block_protection_data)
{

   	unsigned char i;
	i=0;

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x42); 		/* read command */

	for (i=18;i>0;i--)
	{
		SPI1_Exchange8bit(*block_protection_data);
        block_protection_data++;
	}
	CE_SetHigh();			/* disable device */
}
/************************************************************************/
/* PROCEDURE:	Global Block Protection Unlock				*/
/*									*/
/* This procedure clears all block protection				*/
/************************************************************************/
void SPI_Global_Block_Protection_Unlock()
{

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x98); 		/* read command */
	CE_SetHigh();			/* disable device */
}


/************************************************************************/
/* PROCEDURE:	LockBlockProtection					*/
/*									*/
/* This procedure locks the block protection register			*/
/************************************************************************/


void SPI_LockBlockProtection()
{

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0x8d); 		/* read command */
	CE_SetHigh();			/* disable device */
}





/************************************************************************/
/* PROCEDURE:	Non Volatile Write Lock Protection			*/
/*									*/
/* This procedure writes to block protection register			*/
/*									*/
/************************************************************************/

void SPI_NonVolWriteLockProtection(unsigned int *block_protection_data)
{

   	unsigned char i;
	i=0;

	CE_SetLow();			/* enable device */
	SPI1_Exchange8bit(0xE8); 		/* read command */

	for (i=18;i>0;i--)
	{
		SPI1_Exchange8bit(*block_protection_data);
        block_protection_data++;
	}
	CE_SetHigh();			/* disable device */
}


/************************************************************************/
/* PROCEDURE: Wait_Busy							*/
/*									*/
/* This procedure waits until device is no longer busy (can be used by	*/
/* Byte-Program, Page-Program, Sector-Erase, Block-Erase and Chip-Erase).*/
/************************************************************************/



void SPI_Wait_Busy()
{
	while ((SPI_Read_Status_Register()& 0x80) == 0x80)	// waste time until not busy
		SPI_Read_Status_Register();
}

/************************************************************************/
/* PROCEDURE:	SPI_SFDP_Read						*/
/*									*/
/* This procedure reads SFDP Table.					*/
/*									*/
/************************************************************************/
unsigned char SPI_SFDP_Read(unsigned long Dst)
{
	unsigned char byte = 0;

	CE_SetLow();                               /* enable device */
	SPI1_Exchange8bit(0x5A);                        /* read command */
	SPI1_Exchange8bit(((Dst & 0xFFFFFF) >> 16));	/* send 3 address bytes */
	SPI1_Exchange8bit(((Dst & 0xFFFF) >> 8));
	SPI1_Exchange8bit(Dst & 0xFF);
	SPI1_Exchange8bit(0xFF);                	/*dummy byte*/
	byte = SPI1_Exchange8bit(0xFF);
	CE_SetHigh();                      	/* disable device */
	return byte;                    	/* return one byte read */
}

