// Recorder !!!

#include "xc.h"
#include "..\device.h"  // Pins definitions ( Hardware )
#include "SPI1.h"

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void);
void SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData);
uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData);

static bool spi1_enabled = false;

bool SPI1_Enable(SPI_MODE mode, SPI_BRATE speed)
{
    if (!spi1_enabled) {
    PMD1bits.SPI1MD = 0; // Spi1 On
    
#if defined(__PIC24FJ256GA702__)  // Work for SST 

        // ____________________________________SPI Clock & Mode 
        SPI1CON1H = 0x00; // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide;
        // AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; 
        // IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; 
        // URDTEN disabled; IGNTUR disabled; 
        SPI1CON2L = 0x00;
        SPI1STATL = 0x00; // SPIROV disabled; FRMERR disabled; 

        //SPI1BRGL = speed;
        SPI1BRGL = 0x03; // Baud Rate 2Mhz (32Mhz)
        //SPI1BRGL = 0x4F; // Baud Rate 100Khz (32Mhz)

        // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; 
        // SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
        SPI1IMSKL = 0x00;
        // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
        SPI1IMSKH = 0x00;
        // SPI1URDTL 0; 
        SPI1URDTL = 0x00;
        // SPI1URDTH 0; 
        SPI1URDTH = 0x00;
        // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; 
        // SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; 
        // SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; 
        // SPISIDL disabled; ENHBUF enabled; DISSDI enabled; 
        // SPI1CON1L = 0x8031;

        // SPI1 Master, 8Bits
        SPI1CON1Lbits.MSTEN = 1; // Master Mode
        SPI1CON1Lbits.MODE = 0; // Communication is byte-wide  
        SPI1CON2L = 0x0007; // 8 Bits word lenght 

        //  SPI Mode     CKP  CKE
        //  0,0 (0)       0    1
        //  0,1 (1)       0    0
        //  1,0 (2)       1    1
        //  1,1 (3)       1    0

        //SPI_MODE mode = MODE0;

        switch (mode) {
        case MODE0:
            SPI1CON1Lbits.CKE = 1;
            SPI1CON1Lbits.CKP = 0;
            break;
        case MODE1:
            SPI1CON1Lbits.CKE = 0;
            SPI1CON1Lbits.CKP = 0;
            break;
        case MODE2:
            SPI1CON1Lbits.CKE = 1;
            SPI1CON1Lbits.CKP = 1;
            break;
        case MODE3:
            SPI1CON1Lbits.CKE = 0;
            SPI1CON1Lbits.CKP = 1;
            break;
        }

        //    SPI1CON1Lbits.CKP = 0; // MODE?: Clock Polarity (active is a high level)
        //    SPI1CON1Lbits.CKE = 1; // MODE?: Clock Edge (from active to Idle )
        SPI1CON1Lbits.SMP = 1; // Input data is sampled at (0-middle,1-end) of data output

        SPI1CON1Lbits.ENHBUF = 0; // Enhanced buffer disabled (0=Legacy No Buffering)
        // 2Wire:SCK+SDIO,softSS
        SPI1CON1Lbits.DISSCK = 0; // Internal serial clock is enabled
        SPI1CON1Lbits.DISSDO = 0; // SDOx pin is controlled by the module
        SPI1CON1Lbits.DISSDI = 0; // SDIx pin is controlled by the module
        SPI1CON1Hbits.MSSEN = 0; // SPIx slave select is disabled (no SSx)
        //SPIxSTATL: SPIx STATUS REGISTER LOW ( ex SPI1STAT)
        SPI1STATL = 0;
        SPI1CON1Lbits.SPIEN = 1; // Enable SPI 
        SPI1STATLbits.SPIROV = 0; // Receive Overflow Flag (0=NO Overflow).
        spi1_enabled = true;
#endif 
        return(true);
    } else return (false); // False if already started


}

void SPI1_Disable()
{
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    SPI1STATbits.SPIEN = 0;
#endif   

#ifdef __PIC24FJ256GA702__
    SPI1CON1Lbits.SPIEN = 0; // Enable SPI 
#endif
    
    spi1_enabled = false;
    PMD1bits.SPI1MD = 1; // Spi1 Off
}

void SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData)
{

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    while (SPI1STATbits.SPITBF == true) {
    }
    SPI1BUF = *((uint8_t*) pTransmitData);
    while (SPI1STATbits.SRXMPT == true);
    *((uint8_t*) pReceiveData) = SPI1BUF;
#endif   

#ifdef __PIC24FJ256GA702__
    while (SPI1STATLbits.SPITBF == true) {
    }
    SPI1BUFL = *((uint8_t*) pTransmitData);
    while (SPI1STATLbits.SPIRBE == true) {
    };
    *((uint8_t*) pReceiveData) = SPI1BUFL;
#endif
}

uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData)
{

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    uint16_t dataSentCount = 0;
    uint16_t dataReceivedCount = 0;
    uint16_t dummyDataReceived = 0;
    uint16_t dummyDataTransmit = SPI1_DUMMY_DATA;

    uint8_t *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;

    addressIncrement = 1;


    // set the pointers and increment delta
    // for transmit and receive operations
    if (pTransmitData == NULL) {
        sendAddressIncrement = 0;
        pSend = (uint8_t*) & dummyDataTransmit;
    }
    else {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*) pTransmitData;
    }

    if (pReceiveData == NULL) {
        receiveAddressIncrement = 0;
        pReceived = (uint8_t*) & dummyDataReceived;
    }
    else {
        receiveAddressIncrement = addressIncrement;
        pReceived = (uint8_t*) pReceiveData;
    }

    while (SPI1STATbits.SPITBF == true) {

    }
    while (dataSentCount < byteCount) {
        if (SPI1STATbits.SPITBF != true) {
            SPI1BUF = *pSend;

            pSend += sendAddressIncrement;
            dataSentCount++;
        }

        if (SPI1STATbits.SRXMPT == false) {

            *pReceived = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }

    }
    while (dataReceivedCount < byteCount) {
        if (SPI1STATbits.SRXMPT == false) {

            *pReceived = SPI1BUF;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }
    
#endif   

    
#ifdef __PIC24FJ256GA702__
    uint16_t dataSentCount = 0;
    uint16_t dataReceivedCount = 0;
    uint16_t dummyDataReceived = 0;
    uint16_t dummyDataTransmit = SPI1_DUMMY_DATA;

    uint8_t *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;
    uint16_t timeout = 500;

    addressIncrement = 1;

    // set the pointers and increment delta 
    // for transmit and receive operations
    if (pTransmitData == NULL) {
        sendAddressIncrement = 0;
        pSend = (uint8_t*) & dummyDataTransmit;
    }
    else {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*) pTransmitData;
    }

    if (pReceiveData == NULL) {
        receiveAddressIncrement = 0;
        pReceived = (uint8_t*) & dummyDataReceived;
    }
    else {
        receiveAddressIncrement = addressIncrement;
        pReceived = (uint8_t*) pReceiveData;
    }


    while (SPI1STATLbits.SPITBF == true) {

    }

    while (dataSentCount < byteCount) {
        if (SPI1STATLbits.SPITBF != true) {

            SPI1BUFL = *pSend;

            pSend += sendAddressIncrement;
            dataSentCount++;

        }

        if (SPI1STATLbits.SPIRBE == false) {

            *pReceived = SPI1BUFL;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }

    }
    while ((dataReceivedCount < byteCount) && timeout--) { // TIMEOUT !!!!!!!!!!!!!!!
        if (SPI1STATLbits.SPIRBE == false) {

            *pReceived = SPI1BUFL;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }
#endif

    return dataSentCount;
}

uint8_t SPI1_Exchange8bit(uint8_t data)
{
    uint8_t receiveData;
    SPI1_Exchange(&data, &receiveData);
    return (receiveData);
}

uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived)
{
    return (SPI1_ExchangeBuffer(dataTransmitted, byteCount, dataReceived));
}

/**
    The module's transfer mode affects the operation
    of the exchange functions. The table below shows
    the effect on data sent or received:
    |=======================================================================|
    | Transfer Mode  |     Exchange Function      |        Comments         |
    |=======================================================================|
    |                | SPIx_Exchange8bitBuffer()  |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange8bit()        |                         |
    |     8 bits     |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() | Do not use. Only the    |
    |                |----------------------------| lower byte of the 16-bit|
    |                | SPIx_Exchange16bit()       | data will be sent or    |
    |                |                            | received.               |
    |----------------|----------------------------|-------------------------|
    |                | SPIx_Exchange8bitBuffer()  | Do not use. Additional  |
    |                |----------------------------| data byte will be       |
    |                | SPIx_Exchange8bit()        | inserted for each       |
    |                |                            | 8-bit data.             |
    |     16 bits    |----------------------------|-------------------------|
    |                | SPIx_Exchange16bitBuffer() |                         |
    |                |----------------------------|  OK                     |
    |                | SPIx_Exchange16bit()       |                         |
    |----------------|----------------------------|-------------------------|
 
 **/

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void)
{
    if (SPI1CON1bits.MODE16 == 0)
        return SPI1_DRIVER_TRANSFER_MODE_8BIT;
    else
        return SPI1_DRIVER_TRANSFER_MODE_16BIT;
}

SPI1_STATUS SPI1_StatusGet()
{
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    return (SPI1STAT);
#endif   

#ifdef __PIC24FJ256GA702__
    return (SPI1STATL);
#endif
}


////
////#if (0)
////
////#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
////    TRISB = 0x4197; //for TX
////    ANSB = 0x0014; //for RX
////    MRF24_SS_SetDigital();
////
////    // MSTEN Master; DISSDO disabled; PPRE 4:1; SPRE 4:1; MODE16 disabled; SMP Middle; 
////    // DISSCK disabled; CKP Idle:Low, Active:High; CKE Active to Idle;
////    //
////    SPI1CON1 = 0x132;
////
////    // 100110010
////    //        10 = Primary prescale 4:1     ( 32Mhz/2Mhz)
////    //     100 = Secondary prescale 4:1
////    //                                              
////    //    1 = Master mode
////    //   0 = Idle state is a low level; active state is a high level  
////    //  0 = SSx pin is not used by the module;
////    // 1 =  from active to Idle clock state (see bit 6)   
////
////    //    bit 12 DISSCK: Disable SCKx pin bit (SPIx Master modes only)
////    //        1 = Internal SPIx clock is disabled, pin functions as an I/O
////    //        0 = Internal SPIx clock is enabled
////    //    bit 11 DISSDO: Disables SDOx pin bit
////    //        1 = SDOx pin is not used by the module; pin functions as an I/O
////    //        0 = SDOx pin is controlled by the module
////    //    bit 10 MODE16: Word/Byte Communication Select bit
////    //        1 = Communication is word-wide (16 bits)
////    //        0 = Communication is byte-wide (8 bits)
////    //    bit 9 SMP: SPIx Data Input Sample Phase bit
////    //        Master mode:
////    //            1 = Input data is sampled at the end of data output time
////    //            0 = Input data is sampled at the middle of data output time
////    //        Slave mode: SMP must be cleared when SPIx is used in Slave mode.
////    //    bit 8 CKE: Clock Edge Select bit(1)
////    //        1 = Serial output data changes on transition from active clock state to Idle clock state (see bit 6)
////    //        0 = Serial output data changes on transition from Idle clock state to active clock state (see bit 6)
////    //    bit 7 SSEN: Slave Select Enable bit (Slave mode)
////    //        1 = SSx pin is used for Slave mode
////    //        0 = SSx pin is not used by the module; pin is controlled by port function
////    //    bit 6 CKP: Clock Polarity Select bit
////    //        1 = Idle state for clock is a high level; active state is a low level
////    //        0 = Idle state for clock is a low level; active state is a high level
////    //    bit 5 MSTEN: Master Mode Enable bit
////    //        1 = Master mode
////    //        0 = Slave mode
////    //    bit 4-2 SPRE<2:0>: Secondary Prescale bits (Master mode)
////    //        111 = Secondary prescale 1:1
////    //        110 = Secondary prescale 2:1
////    //        ...
////    //    bit 1-0 PPRE<1:0>: Primary Prescale bits (Master mode)
////    //        11 = Primary prescale 1:1
////    //        10 = Primary prescale 4:1
////    //        01 = Primary prescale 16:1
////    //        00 = Primary prescale 64:1
////
////    // SPIBEN enabled; SPIFPOL disabled; SPIFE disabled;
////    SPI1CON2 = 0x01;
////
////
////    // SPITBF disabled; SISEL SPI_INT_SPIRBF; SPIRBF disabled; SPIROV disabled; SPIEN enabled; SRXMPT disabled; SRMPT disabled; SPISIDL disabled; SPIBEC disabled;
////    SPI1STAT = 0x800C;
////
////
////#elif defined(__PIC24FJ256GA702__)
////
////    //    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
////    //    SPI1CON1H = 0x00;
////    //    // WLENGTH 0; 
////    //    SPI1CON2L = 0x00;
////    //    // SPIROV disabled; FRMERR disabled; 
////    //    SPI1STATL = 0x00;
////    //    // SPI1BRGL 0; 
////    //    SPI1BRGL = 0x00;
////    //    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
////    //    SPI1IMSKL = 0x00;
////    //    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
////    //    SPI1IMSKH = 0x00;
////    //    // SPI1URDTL 0; 
////    //    SPI1URDTL = 0x00;
////    //    // SPI1URDTH 0; 
////    //    SPI1URDTH = 0x00;
////    //    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
////    //    SPI1CON1L = 0x8021;
////    // ____________________________________SPI Clock & Mode 
////    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
////    SPI1CON1H = 0x00;
////    // WLENGTH 0; 
////    SPI1CON2L = 0x00;
////    // SPIROV disabled; FRMERR disabled; 
////    SPI1STATL = 0x00;
////    // SPI1BRGL 79; 
////    // SPI1BRGL = 0x4F; // Baud Rate 100Khz (32Mhz)
////    SPI1BRGL = 0x03; // Baud Rate 2Mhz (32Mhz)
////    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
////    SPI1IMSKL = 0x00;
////    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
////    SPI1IMSKH = 0x00;
////    // SPI1URDTL 0; 
////    SPI1URDTL = 0x00;
////    // SPI1URDTH 0; 
////    SPI1URDTH = 0x00;
////    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; 
////    // SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; 
////    // SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; 
////    // SPISIDL disabled; ENHBUF enabled; DISSDI enabled; 
////    // SPI1CON1L = 0x8031;
////
////
////    // SPI1 Master, 8Bits
////    SPI1CON1Lbits.MSTEN = 1; // Master Mode
////    SPI1CON1Lbits.MODE = 0; // Communication is byte-wide  
////    SPI1CON2L = 0x0007; // 8 Bits word lenght 
////    //  Mode?
////    SPI1CON1Lbits.CKE = 1; // MODE?: Clock Edge (from active to Idle )
////    SPI1CON1Lbits.CKP = 0; // MODE?: Clock Polarity (active is a high level)
////    SPI1CON1Lbits.SMP = 1; // Input data is sampled at (0-middle,1-end) of data output
////    SPI1CON1Lbits.ENHBUF = 0; // Enhanced buffer disabled (0=Legacy No Buffering)
////    // 2Wire:SCK+SDIO,softSS
////    SPI1CON1Lbits.DISSCK = 0; // Internal serial clock is enabled
////    SPI1CON1Lbits.DISSDO = 0; // SDOx pin is controlled by the module
////    SPI1CON1Lbits.DISSDI = 0; // SDIx pin is controlled by the module
////    SPI1CON1Hbits.MSSEN = 0; // SPIx slave select support is disabled (no SSx)
////    //SPIxSTATL: SPIx STATUS REGISTER LOW ( ex SPI1STAT)
////    SPI1CON1Lbits.SPIEN = 1; // Enable SPI 
////    SPI1STATLbits.SPIROV = 0; // Receive Overflow Flag (0=NO Overflow).
////#endif
////
////#endif