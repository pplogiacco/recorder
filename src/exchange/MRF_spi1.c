
#include "MRF_spi1.h"
#include "..\device.h"
/**
 Section: File specific functions
 */

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void);
void SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData);
uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData);

/**
 Section: Driver Interface Function Definitions
 */

void SPI1_Initialize(void) {

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    TRISB = 0x4197; //for TX
    ANSB = 0x0014; //for RX
    MRF24_SS_SetDigital();

    // MSTEN Master; DISSDO disabled; PPRE 4:1; SPRE 4:1; MODE16 disabled; SMP Middle; DISSCK disabled; CKP Idle:Low, Active:High; CKE Active to Idle;
    SPI1CON1 = 0x132;
    // SPIBEN enabled; SPIFPOL disabled; SPIFE disabled;
    SPI1CON2 = 0x01;
    // SPITBF disabled; SISEL SPI_INT_SPIRBF; SPIRBF disabled; SPIROV disabled; SPIEN enabled; SRXMPT disabled; SRMPT disabled; SPISIDL disabled; SPIBEC disabled;
    SPI1STAT = 0x800C;
#endif    

#ifdef __PIC24FJ256GA702__

    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI1CON1H = 0x00;
    // WLENGTH 0; 
    SPI1CON2L = 0x00;
    // SPIROV disabled; FRMERR disabled; 
    SPI1STATL = 0x00;
    // SPI1BRGL 0; 
    SPI1BRGL = 0x00;
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x00;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x00;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x00;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x00;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    SPI1CON1L = 0x8021;

#endif


}

void SPI1_Exchange(uint8_t *pTransmitData, uint8_t *pReceiveData) {

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

uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData) {

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
    } else {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*) pTransmitData;
    }

    if (pReceiveData == NULL) {
        receiveAddressIncrement = 0;
        pReceived = (uint8_t*) & dummyDataReceived;
    } else {
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

    uint8_t  *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;

    addressIncrement = 1;

    // set the pointers and increment delta 
    // for transmit and receive operations
    if (pTransmitData == NULL)
    {
        sendAddressIncrement = 0;
        pSend = (uint8_t*)&dummyDataTransmit;
    }
    else
    {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*)pTransmitData;
    }
        
    if (pReceiveData == NULL)
    {
       receiveAddressIncrement = 0;
       pReceived = (uint8_t*)&dummyDataReceived;
    }
    else
    {
       receiveAddressIncrement = addressIncrement;        
       pReceived = (uint8_t*)pReceiveData;
    }


    while( SPI1STATLbits.SPITBF == true )
    {

    }

    while (dataSentCount < byteCount)
    {
        if ( SPI1STATLbits.SPITBF != true )
        {

            SPI1BUFL = *pSend;

            pSend += sendAddressIncrement;
            dataSentCount++;

        }

        if (SPI1STATLbits.SPIRBE == false)
        {

            *pReceived = SPI1BUFL;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }

    }
    while (dataReceivedCount < byteCount)
    {
        if (SPI1STATLbits.SPIRBE == false)
        {

            *pReceived = SPI1BUFL;

            pReceived += receiveAddressIncrement;
            dataReceivedCount++;
        }
    }
#endif
  
    return dataSentCount;
}

uint8_t SPI1_Exchange8bit(uint8_t data) {
    uint8_t receiveData;

    SPI1_Exchange(&data, &receiveData);

    return (receiveData);
}

uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived) {
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
 */
inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void) {
    if (SPI1CON1bits.MODE16 == 0)
        return SPI1_DRIVER_TRANSFER_MODE_8BIT;
    else
        return SPI1_DRIVER_TRANSFER_MODE_16BIT;
}

SPI1_STATUS SPI1_StatusGet() {
 #if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    return (SPI1STAT);
#endif   

#ifdef __PIC24FJ256GA702__
    return(SPI1STATL);
#endif

}

void SPI1_Disable() {
     #if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    SPI1STATbits.SPIEN = 0;
#endif   

#ifdef __PIC24FJ256GA702__

#endif


    
}


/**
 End of File
 */