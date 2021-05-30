#include <string.h>
#include "../device.h"
#include "../utils.h"

#include "../modules/SPI1.h"    // Driver
#include "MRF24J40.h"

#define MRF_RXMCR    0x00
#define MRF_PANIDL   0x01
#define MRF_PANIDH   0x02
#define MRF_SADRL    0x03
#define MRF_SADRH    0x04
#define MRF_RXFLUSH  0x0D
#define MRF_TXNCON   0x1B
#define MRF_PACON2   0x18
#define MRF_WAKECON  0x22
#define MRF_TXSTAT   0x24
#define MRF_SOFTRST  0x2A
#define MRF_TXSTBL   0x2E
#define MRF_INTSTAT  0x31
#define MRF_INTCON   0x32
#define MRF_GPIO     0x33
#define MRF_TRISGPIO 0x34
#define MRF_SLPACK   0x35
#define MRF_RFCTL    0x36
#define MRF_BBREG1   0x39
#define MRF_BBREG2   0x3A
#define MRF_BBREG6   0x3E
#define MRF_CCAEDTH  0x3F
#define MRF_RFCON0   0x200
#define MRF_RFCON1   0x201
#define MRF_RFCON2   0x202
#define MRF_RFCON6   0x206
#define MRF_RFCON7   0x207
#define MRF_RFCON8   0x208
#define MRF_RFSTATE  0x20F
#define MRF_RSSI     0x210
#define MRF_SLPCON0  0x211
#define MRF_SLPCON1  0x220
#define MRF_TESTMODE 0x22F

static uint8_t rssi = 0;
static uint8_t lqi = 0;
static uint16_t srcAddr = 0;
static uint8_t seqNumber = 0;

const uint16_t panId = 0;
const uint8_t type = MRF24J40MA;

int rxSize = 0;
int rxCount = 0;
uint8_t rxBuf[MRF_MAX_RX_FIFO_SIZE];

//#if 0
//// device.h / hardware.h 
//// #define MRF_SS_SetLow()           (_LATB15 = 0)
//// #define MRF_SS_SetHigh()          (_LATB15 = 1)
//
//void MRF24J40_Enable() {
//    // _____SPI1 Master Mode 1 (2Wire:SCK+SDIO,softSS)
//    SPI1STATbits.SPIEN = 0; // Disable module
//    IFS0bits.SPI1IF = 0; // Clear int vect flag
//    IEC0bits.SPI1IE = 0; // DISABLE INT !!!
//    // ________SPI Pins
//    TRISBbits.TRISB14 = 1; // SDI1 (MISO) In
//    TRISBbits.TRISB13 = 0; // SDO1 (MOSI) Out
//    TRISBbits.TRISB12 = 0; // SCK1 Digital Out
//    // ______________Clock & Mode Selection
//    SPI1CON1bits.MODE16 = 0; // Communication is word-wide (16 bits)
//    SPI1CON1bits.MSTEN = 1; // Master Fsck=(Fosc/2)/(Ppre*Spre)==??
//    SPI1CON1bits.PPRE = 0; // Primary prescaler:(0=low,3=High)
//    SPI1CON2bits.FRMEN = 0; // Framed SPIx support DISABLE
//    SPI1CON1bits.CKE = 0; // MODE1: Clock Edge (from Idle to active clock state)
//    SPI1CON1bits.CKP = 1; // MODE1: Clock Polarity (active is a high level)
//    SPI1CON1bits.SMP = 0; // Input data is sampled at (0-middle,1-end) of data output
//    SPI1CON2bits.SPIBEN = 0; // Enhanced buffer enabled (0=Legacy No Buffering)
//    SPI1STATbits.SPIROV = 0; // Receive Overflow Flag (0=NO Overflow).
//    SPI1STATbits.SPIEN = 1; // Enable SPI
//}
//
//void MRF_SetReg8(uint8_t reg, uint8_t data) {
//
//    uint16_t regx = (reg << 1 & 0b01111110) | 0b00000001;
//    regx <<= 8;
//    regx |= data;
//
//    MRF24_SS_SetLow();
//    __delay(1);
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    SPI1BUF = (0x00FF & (regx >> 8));
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    SPI1BUF = data;
//
//    //while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    //SPI1BUF = data;
//    //while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    __delay(2);
//    MRF24_SS_SetHigh();
//}
//
//void MRF_SetReg16(uint16_t reg, uint8_t data) {
//
//    uint16_t regx = ((reg >> 3 | 0b10000000) << 8) | (uint8_t) ((reg << 5 | 0b00010000) & 0x00FF);
//    MRF24_SS_SetLow();
//    __delay(1);
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    SPI1BUF = (0x00FF & (regx >> 8));
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    SPI1BUF = (0x00FF & ((uint8_t) regx));
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    SPI1BUF = data;
//    while (SPI1STATbits.SPITBF == true); // SPIxTXB
//    __delay(2);
//    MRF24_SS_SetHigh();
//}
//#endif



void MRF24J40_writeShort(uint8_t addr, uint8_t data) {
    uint8_t dataTransmitted[2];
    MRF24_SS_SetLow();
    dataTransmitted[0] = ((addr << 1 & 0b01111110) | 0b00000001);
    dataTransmitted[1] = data;
    SPI1_Exchange8bitBuffer(dataTransmitted, 2, NULL);
    MRF24_SS_SetHigh();
}

unsigned char MRF24J40_readShort(uint8_t addr) {
    unsigned char toReturn = 0x00;
    uint8_t dataTransmitted[1];
    uint8_t dataReceived[1];

    memset(dataReceived, 0x00, sizeof (dataReceived));
    dataTransmitted[0] = ((addr << 1) & 0b01111110);
    MRF24_SS_SetLow();
    SPI1_Exchange8bitBuffer(dataTransmitted, 1, NULL);
    SPI1_Exchange8bitBuffer(NULL, 1, dataReceived);
    MRF24_SS_SetHigh();
    toReturn = dataReceived[0];
    return (toReturn);
}

void MRF24J40_writeLong(uint16_t addr, uint8_t data) {

    MRF24_SS_SetLow();
    uint8_t dataTransmitted[3];
    dataTransmitted[0] = (addr >> 3 | 0b10000000);
    dataTransmitted[1] = (addr << 5 | 0b00010000);
    dataTransmitted[2] = data;
    SPI1_Exchange8bitBuffer(dataTransmitted, 3, NULL);
    MRF24_SS_SetHigh();
}

unsigned char MRF24J40_readLong(uint16_t addr) {
    unsigned char toReturn;
    uint8_t dataTransmitted[2];
    uint8_t dataReceived[1];

    dataTransmitted[0] = (addr >> 3 | 0b10000000);
    dataTransmitted[1] = (addr << 5);

    MRF24_SS_SetLow();
    SPI1_Exchange8bitBuffer(dataTransmitted, 2, NULL);
    SPI1_Exchange8bitBuffer(NULL, 1, dataReceived);
    MRF24_SS_SetHigh();
    toReturn = dataReceived[0];
    return (toReturn);
}


void MRF24J40_setPanId(uint16_t panId) {
    //this->panId = panId;
    MRF24J40_writeShort(MRF_PANIDH, panId >> 8);
    MRF24J40_writeShort(MRF_PANIDL, panId);
}

void MRF24J40_setChannel(int channel) {
    if (channel < 11 || channel > 26) {
        return;
    }

    // Change channel
    MRF24J40_writeLong(MRF_RFCON0, (channel - 11) << 4 | 0x03);

    // Perform RF state machine reset and wait for RF circuitry to calibrate
    MRF24J40_writeShort(MRF_RFCTL, 0b00000100);
    MRF24J40_writeShort(MRF_RFCTL, 0);
    __delay_us(200);
}

void MRF24J40_enablePaLna() {
    if (type != MRF24J40MA) {
        MRF24J40_writeShort(MRF_TRISGPIO, 0); // Configure all GPIO pins as input
        MRF24J40_writeShort(MRF_GPIO, 0);
        MRF24J40_writeLong(MRF_TESTMODE, 0b00001111); // Configure RF state machine for automatic PA/LNA control
    }
}

void MRF24J40_reset() {
    MRF24J40_writeShort(MRF_SOFTRST, 0b00000111); // Perform full software reset (RSTPWR = 1, RSTBB = 1, RSTMAC = 1)
}


void MRF24J40_init() {
    
    rxSize = 0;
    rxCount = 0;
    seqNumber = 0;
    MRF_SS_SetDigitalOutputHigh();        
    //MRF24_SS_SetHigh();

    // -----------------------------------------
    // | b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
    // -----------------------------------------
    // MRF24J40 module configuration

#ifdef __DONGLE_PASSTHRU
    MRF24J40_writeShort(MRF_RXMCR, 0b00000001); // Setup RECEIVE MAC b0 1=Receive all packet types with good CRC
#else
    MRF24J40_writeShort(MRF_RXMCR, 0b00000000); // Setup RECEIVE MAC b0 0=Discard packet when there is a MAC address mismatch, illegal frame type, dPAN/sPAN or MAC
#endif

    MRF24J40_writeShort(MRF_PACON2, 0b10011000); // Setup recommended PA/LNA control timing (TXONTS = 0x6)
    MRF24J40_writeShort(MRF_TXSTBL, 0b10010101); // Setup recommended PA/LNA control timing (RFSTBL = 0x9)
    MRF24J40_writeLong(MRF_RFCON0, 0b00000011); // Set recommended value for RF Optimize Control (RFOPT = 0x3)
    MRF24J40_writeLong(MRF_RFCON1, 0b00000010); // Set recommended value for VCO Optimize Control (VCOOPT = 0x2)
    MRF24J40_writeLong(MRF_RFCON2, 0b10000000); // Enable PLL (PLLEN = 1)
    MRF24J40_writeLong(MRF_RFCON6, 0b10010000); // Set recommended value for TX Filter Control and 20MHz Clock Recovery Control
    //  (TXFIL = 1, 20MRECVR = 1)
    MRF24J40_writeLong(MRF_RFCON7, 0b10000000); // Use 100kHz internal oscillator for Sleep Clock (SLPCLKSEL = 0x2)
    MRF24J40_writeLong(MRF_RFCON8, 0b00010000); // Set recommended value for VCO Control (RFVCO = 1)
    MRF24J40_writeLong(MRF_SLPCON0, 0b00000001); // Disable the sleep clock to save power (/SLPCLKEN = 1)
    MRF24J40_writeLong(MRF_SLPCON1, 0b00100001); // Disable CLKOUT pin and set Sleep Clock Divisor to minimum of 0x01 for the
    //  100kHz internal oscillator (/CLOUKTEN = 1, SLPCLKDIV = 0x01)
    MRF24J40_writeShort(MRF_BBREG2, 0b10111000); // Use CCA Mode 1 - Energy above threshold - and set CCA Carrier Sense
    //  Threshold to recommended value (CCAMODE = 0x2, CCACSTH = 0xE)
    MRF24J40_writeShort(MRF_CCAEDTH, 0b01100000); // Set energy detection threshold to recommended value (CCAEDTH = 0x60)
    MRF24J40_writeShort(MRF_BBREG6, 0b01000000); // Calculate RSSI for each packet received (RSSIMODE2 = 1)
    MRF24J40_writeShort(MRF_INTCON, 0b11110111); // Enable RX FIFO reception interrupt (RXIE = 0)

    MRF24J40_enablePaLna(); // Enable PA and LNA control

    MRF24J40_setPanId(panId);
    MRF24J40_setChannel(11); // Set default channel (must keep 11) and reset state machine

    __delay(2);
}

float MRF24J40_getSignalStrength() {
    return rssi * 0.197948 - 87.646977;
}

float MRF24J40_measureSignalStrength() {
    // Disable PA
    if (type != MRF24J40MA) {
        MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure RF state machine for normal operation
        MRF24J40_writeShort(MRF_TRISGPIO, 0b00000110); // Configure GPIO1 and GPIO2 for output
        MRF24J40_writeShort(MRF_GPIO, 0b00000100); // Set GPIO1 0 to disable PA and GPIO2 to 1 to enable LNA
    }

    // Initiate RSSI calculation (RSSIMODE1 = 1)
    MRF24J40_writeShort(MRF_BBREG6, 0b11000000);

    // Wait until conversion is ready and read RSSI
    while (!(MRF24J40_readShort(MRF_BBREG6) & 0b00000001));
    rssi = MRF24J40_readLong(MRF_RSSI);

    // Re-enable PA
    MRF24J40_enablePaLna();

    return MRF24J40_getSignalStrength();
}

void MRF24J40_setAddress(uint16_t addr) {
    srcAddr = addr;
    MRF24J40_writeShort(MRF_SADRH, addr >> 8);
    MRF24J40_writeShort(MRF_SADRL, addr);
}

// -----------------------------------------
// | b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
// -----------------------------------------

void MRF24J40_rxFlush() {
    MRF24J40_writeShort(MRF_RXFLUSH, 0b00000001);
}

void MRF24J40_Disable() {
    // Disable PA and LNA
    switch (type) {
        case MRF24J40MB:
            // GPIO1 and GPIO2 are connected
            MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure RF state machine for normal operation
            MRF24J40_writeShort(MRF_TRISGPIO, 0b00000110); // Configure GPIO1 and GPIO2 for output
            MRF24J40_writeShort(MRF_GPIO, 0); // Set GPIO1 and GPIO2 to 0 to disable PA and LNA
            break;
        case MRF24J40MC:
            // GPIO1, GPIO2 and GPIO3 are connected - GPIO3 enables (high) or disables (low) the PA voltage regulator
            MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure RF state machine for normal operation
            MRF24J40_writeShort(MRF_TRISGPIO, 0b00001110); // Configure GPIO1, GPIO2 and GPIO3 for output
            MRF24J40_writeShort(MRF_GPIO, 0); // Set GPIO1, GPIO2 and GPIO3 to 0 to disable PA, LNA and PA regulator
            break;
        case MRF24J40MD:
        case MRF24J40ME:
            // GPIO0, GPIO1 and GPIO2 are connected
            MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure RF state machine for normal operation
            MRF24J40_writeShort(MRF_TRISGPIO, 0b00000111); // Configure GPIO0, GPIO1 and GPIO2 for output
            MRF24J40_writeShort(MRF_GPIO, 0); // Set GPIO1 and GPIO2 to 0 to disable PA and LNA
            break;
    }

    MRF24J40_writeShort(MRF_SOFTRST, 0b00000100); // Perform Power Management Reset (RSTPWR = 1)
    MRF24J40_writeShort(MRF_WAKECON, 0b10000000); // Enable Immediate Wake-up Mode (IMMWAKE = 1)
    MRF24J40_writeShort(MRF_SLPACK, 0b10000000); // Put the module to sleep (SLPACK = 1)
    
    SPI1_Disable();  // Disable SPI1    
    MRF24_SS_SetLow();  // CS low !
}

void MRF24J40_Enable() {
    SPI1_Enable(MODE0,SPI_2MHZ);
    MRF24J40_reset(); // Perform software reset
    MRF24J40_init(); // Reinitialize all registers
}

void MRF24J40_TxBuffer(uint16_t addr, uint8_t *data, uint8_t length, bool ack) {
    // Send header size, frame size and header
    int j = 0;
    int i = 0;
    MRF24J40_writeLong(i++, MRF_MHR_SIZE); // Header size
    MRF24J40_writeLong(i++, MRF_MHR_SIZE + length); // Frame size
    MRF24J40_writeLong(i++, ack ? 0b01100001 : 0b01000001); // Frame control bits 0-7 (data frame, no security, no pending data, ack disabled, intra-PAN)
    MRF24J40_writeLong(i++, 0b10001000); // Frame control bits 8-15 (16-bit addresses with PAN)
#ifdef __DONGLE_PASSTHRU
    MRF24J40_writeLong(i++, seqNumber); // Sequence number
#else
    MRF24J40_writeLong(i++, seqNumber++); // Sequence number
#endif
    MRF24J40_writeLong(i++, panId); // PAN ID
    MRF24J40_writeLong(i++, panId >> 8);
    MRF24J40_writeLong(i++, addr); // Destination address
    MRF24J40_writeLong(i++, addr >> 8);
    MRF24J40_writeLong(i++, srcAddr); // Source address
    MRF24J40_writeLong(i++, srcAddr >> 8);

    // Send data payload
    for (j = 0; j < length; j++) {
        MRF24J40_writeLong(i++, data[j]);
    }

    // Start transmission (0x37 0x01)
    MRF24J40_writeShort(MRF_TXNCON, ack ? 0b00000101 : 0b00000001);
}

int MRF24J40_RxBuffer(uint8_t* data, int size) {
    int maxSize = size;
    if (size > (rxSize - rxCount)) {
        maxSize = (rxSize - rxCount);
    }
    if (maxSize > 0) {
        memcpy(data, &rxBuf[rxCount], maxSize);
        rxCount += maxSize;
    }
    return maxSize;
}

bool MRF24J40_IsRxReady() {
    return !((rxSize - rxCount) == 0);
}

bool MRF24J40_transmissionDone() {
    return MRF24J40_readShort(MRF_INTSTAT) & 0b00000001;
}

bool MRF24J40_transmissionSuccess() {
    return !(MRF24J40_readShort(MRF_TXSTAT) & 0b00000001);
}

bool MRF24J40_receivePacket() {
    int frameSize;

    // Check RXIF in INTSTAT
    if (MRF24J40_readShort(MRF_INTSTAT) & 0b00001000) {
        // Disable receiver
        MRF24J40_writeShort(MRF_BBREG1, 0b00000100);

        // Packet received, get the number of bytes
        frameSize = MRF24J40_readLong(0x300);

#ifdef __DONGLE_PASSTHRU
        seqNumber = MRF24J40_readLong(0x303);
#endif
        // Copy the message bytes into the user buffer
        rxSize = 0;
        rxCount = 0;
        while (rxSize < (frameSize - MRF_MHR_SIZE - MRF_MFR_SIZE)) {
            rxBuf[rxSize] = MRF24J40_readLong(0x301 + MRF_MHR_SIZE + rxSize);
            rxSize++;
        }

        // Read RSSI and LQI
        lqi = MRF24J40_readLong(0x301 + frameSize);
        rssi = MRF24J40_readLong(0x301 + frameSize + 1);

        // Flush the reception buffer, re-enable receiver
        MRF24J40_rxFlush();

        MRF24J40_writeShort(MRF_BBREG1, 0);

        // Wait until RXIF is cleared (takes a while)
        while (MRF24J40_readShort(MRF_INTSTAT) & 0b00001000);
        return true;
    }
    return false;
}

