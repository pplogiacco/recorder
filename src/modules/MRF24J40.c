// Recorder !!!
// Libex 0.0.17
#include <string.h>
#include "../device.h"
#include "../utils.h"

#include "../modules/SPI1.h"    // Driver
#include "MRF24J40.h"

// Packet Transmission Details
//
// 802.15.4 MAC Data Frame (max size = max PSDU size = 127)
// -------------------------------------------------------------
// | MAC Header (MHR) | Data Payload (MSDU) | MAC Footer (MFR) |
// -------------------------------------------------------------
// |        9         |       1 - 116       |         2        |
// -------------------------------------------------------------
//
// 802.15.4 MAC Header (MHR)
// -------------------------------------------------------
// | Frame Control | Sequence Number | Addressing Fields |
// -------------------------------------------------------
// |       2       |        1        |         6         |
// -------------------------------------------------------
//
// 802.15.4 Addressing Fields (16-bit addressing with PAN ID)
// -------------------------------------------------
// | PAN ID | Destination Address | Source Address |
// -------------------------------------------------
// |    2   |          2          |        2       |
// -------------------------------------------------
//
// MRF24J40 TX FIFO
// ---------------------------------------------------------------------
// | Header Length | Frame Length | Header (MHR) | Data Payload (MSDU) |
// ---------------------------------------------------------------------
// |       1       |       1      |       9      |       1 - 116       |
// ---------------------------------------------------------------------

#define MRF_PAN_ID_SIZE 2
#define MRF_DEST_ADDR_SIZE 2
#define MRF_SRC_ADDR_SIZE 2
#define MRF_ADDR_FIELDS_SIZE (MRF_PAN_ID_SIZE + MRF_DEST_ADDR_SIZE + MRF_SRC_ADDR_SIZE)
#define MRF_MHR_SIZE (2 + 1 + MRF_ADDR_FIELDS_SIZE)
#define MRF_MFR_SIZE 2
#define MRF_MAX_PSDU_SIZE 127
#define MRF_MAX_PAYLOAD_SIZE (MRF_MAX_PSDU_SIZE - MRF_MHR_SIZE - MRF_MFR_SIZE)
#define MRF_MAX_TX_FIFO_SIZE (1 + 1 + MRF_MHR_SIZE + MRF_MAX_PAYLOAD_SIZE)


// Packet Reception Details
//
// MRF24J40 RX FIFO
// ------------------------------------------------------------------------
// | Frame Length | Header (MHR) | Data Payload (MSDU) | FCS | LQI | RSSI |
// ------------------------------------------------------------------------
// |       1      |       9      |       1 - 116       |  2  |  1  |   1  |
// ------------------------------------------------------------------------
#define MRF_MAX_RX_FIFO_SIZE (1 + MRF_MHR_SIZE + MRF_MAX_PAYLOAD_SIZE + 2 + 1 + 1)

#define MRF_RXMCR    0x00 // Use READ/WRITE adjusted values intead !!!!!!!!!!
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
//          RSSITHCCA  ??? 
#define MRF_RFCON0   0x200
#define MRF_RFCON1   0x201
#define MRF_RFCON2   0x202
#define MRF_RFCTRL3  0x203  // Plg
#define MRF_RFCON6   0x206
#define MRF_RFCON7   0x207
#define MRF_RFCON8   0x208
#define MRF_RFSTATE  0x20F
#define MRF_RSSI     0x210
#define MRF_SLPCON0  0x211
#define MRF_SLPCON1  0x220
#define MRF_TESTMODE 0x22F

// SPI Interfacing routines
void MRF24J40_writeShort(uint8_t addr, uint8_t data);
void MRF24J40_writeLong(uint16_t addr, uint8_t data);
uint8_t MRF24J40_readShort(uint8_t addr);
uint8_t MRF24J40_readLong(uint16_t addr);

// Locals
static bool disable_spi1; // Shared SPI1
//static mrftype_t module_type;
static uint16_t net_pan; // Network ID
static uint16_t net_addr; // Local mac

static uint8_t rf_channel; // Channell 
static uint8_t rf_rssi = 0; // Signal intensity    
static uint8_t rf_lqi = 0; // link quality

static uint8_t seqNumber = 0;

int rxSize = 0;
int rxCount = 0;

uint8_t rxBuf[MRF_MAX_RX_FIFO_SIZE];

void MRF24J40_Enable(uint16_t macaddr) {

    uint16_t chk;

    net_addr = macaddr;
    net_pan = MRF24J40_PAN_ID;
    rf_channel = MRF24J40_RF_CHANNEL;

    rxSize = 0;
    rxCount = 0;
    seqNumber = 0;

    MRF24_SS_SetDigitalOutputHigh();
    disable_spi1 = SPI1_Enable(MODE0, SPI_2MHZ); // Shared SPI1

    // MRF24J40 module configuration

    MRF24J40_writeShort(MRF_SOFTRST, 0b00000111); // Perform a software Reset (RSTPWR=1,RSTBB=1, RSTMAC=1)
    do { // The bits will cleared to 0...
        chk = MRF24J40_readShort(MRF_SOFTRST);
    } while (chk & 0x07);
    __delay(1); // Verificare datasheet

    // Return Enable OK "!!!!
    //    printf("Reset OK");

    MRF24J40_writeShort(MRF_PACON2, 0b10011000); // 0x98 Initialize FIFOEN = 1 and TXONTS = 0x6
    MRF24J40_writeShort(MRF_TXSTBL, 0b10010101); // 0x95 Initialize RFSTBL = 0x9

    MRF24J40_writeLong(MRF_RFCON0, 0b00000011); // 0x03 Set recommended value for RF Optimize Control (RFOPT = 0x3)
    //MRF24J40_writeLong(MRF_RFCON1, 0b00000010); // Set recommended value for VCO Optimize Control (VCOOPT = 0x2) ERRATO!!!
    MRF24J40_writeLong(MRF_RFCON1, 0b00000001); // 0x01 Set recommended value for VCO Optimize Control (VCOOPT = 0x2)
    MRF24J40_writeLong(MRF_RFCON2, 0b10000000); // 0x80 Enable PLL (PLLEN = 1)
    MRF24J40_writeLong(MRF_RFCON6, 0b10010000); // 0x90 Set recommended value for TX Filter Control and 20MHz Clock Recovery Control
    MRF24J40_writeLong(MRF_RFCON7, 0b10000000); // 0x80 Use 100kHz internal oscillator for Sleep Clock (SLPCLKSEL = 0x2)
    MRF24J40_writeLong(MRF_RFCON8, 0b00010000); // 0x10 Set recommended value for VCO Control (RFVCO = 1)

    MRF24J40_writeLong(MRF_SLPCON0, 0b00000001); // Disable the sleep clock to save power (SLPCLKEN = 1)
    MRF24J40_writeLong(MRF_SLPCON1, 0b00100001); // Disable CLKOUT and set Sleep Clock Divisor to minimum (CLOUKTEN = 1, SLPCLKDIV = 0x01)

    // Configuration for nonbeacon-enabled devices
    //MRF24J40_writeShort(MRF_BBREG2, 0b10111000); // 0xB8 Use CCA Mode 1 - Energy above threshold - and set CCA Carrier Sense (CCAMODE = 0x2, CCACSTH = 0xE)
    MRF24J40_writeShort(MRF_BBREG2, 0b10111000); // 0x80 Set CCA mode to ED
    MRF24J40_writeShort(MRF_BBREG6, 0b01000000); // 0x40 Set appended RSSI value to RXFIFO (RSSIMODE2 = 1)

    // Make RF communication stable under extreme temperatures
    //    PHYSetLongRAMAddr(RFCTRL0, 0x03);
    //    PHYSetLongRAMAddr(RFCTRL1, 0x02);

    /* Program CCA, RSSI threshold values */
    // MRF24J40_writeShort(WRITE_RSSITHCCA, 0x60);

    MRF24J40_writeShort(MRF_CCAEDTH, 0b01100000); // 0x60 Set energy detection threshold to recommended value (CCAEDTH = 0x60)
    MRF24J40_writeShort(MRF_INTCON, 0b11110111); // Enable RX FIFO reception interrupt (RXIE = 0)

    // Set PanID & MAC

#ifdef MRF24J40_PROMISCUOUS
    MRF24J40_writeShort(MRF_RXMCR, 0b00000001); // Setup RECEIVE MAC b0 1=Receive all packet types with good CRC
#else
    //MRF24J40_setPanId(net_pan);
    MRF24J40_writeShort(MRF_PANIDH, net_pan >> 8);
    MRF24J40_writeShort(MRF_PANIDL, net_pan);
    //MRF24J40_setAddress(net_addr);
    MRF24J40_writeShort(MRF_SADRH, net_addr >> 8);
    MRF24J40_writeShort(MRF_SADRL, net_addr);
    MRF24J40_writeShort(MRF_RXMCR, 0b00000000); // Setup RECEIVE MAC b0 0=Discard packet when there is a MAC address mismatch, illegal frame type, dPAN/sPAN or MAC
#endif

    // Enable interrupts ? See Section 3.3 ?Interrupts?.    

    // Set channel ? See Section 3.4 ?Channel Selection?.
    //MRF24J40_setChannel(rf_channel); // Set default channel (must keep 11) and reset state machine
    MRF24J40_writeLong(MRF_RFCON0, (rf_channel - 11) << 4 | 0x03);

    // Perform RF state machine reset and wait for RF circuitry to calibrate
    MRF24J40_writeShort(MRF_RFCTL, 0b00000100);
    MRF24J40_writeShort(MRF_RFCTL, 0);
    //_delay_us(200);

    // Set transmitter power - See REGISTER 2-62: RF CONTROL REGISTER (ADDRESS: 0x203)
    // MRF24J40_enablePaLna(); 

    // Special MRF24J40 setting for Microchip module
#if ( MODULE_TYPE ==  MRF24J40MA) // No Pa/LNA control

    //MRF24J40_writeShort(MRF_TRISGPIO, 0); // Configure all GPIO pins as input
    MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); //000 = Normal operation (default)
    MRF24J40_writeLong(MRF_RFCTRL3, 0x00); // Power level to be 0dBm

#elif ( MODULE_TYPE == MRF24J40MD)

    MRF24J40_writeShort(MRF_TRISGPIO, 0b00000111); // Configure GPIO0, GPIO1 and GPIO2 for output
    //TESTMODE<2:0>: Test Mode bits
    //111 = GPIO0, GPIO1 and GPIO2 are configured to control an external PA and/or LNA(1)
    //101 = Single Tone Test mode
    //000 = Normal operation (default)
    MRF24J40_writeLong(MRF_TESTMODE, 0b00001111); // configure to operate the external PA/LNA

#if ( APPLICATION_SITE == EUROPE )
    MRF24J40_writeLong(MRF_RFCTRL3, 0x70); // Europe output power set to be -14.9dB
#else
    MRF24J40_writeLong(MRF_RFCTRL3, 0x18); // Usa output power set to be -1.9dB
#endif

#endif

    //    // Define TURBO_MODE if more bandwidth is required
    //    // to enable radio to operate to TX/RX maximum
    //    // 625Kbps
    //#ifdef TURBO_MODE
    //
    //    PHYSetShortRAMAddr(WRITE_BBREG0, 0x01);
    //    PHYSetShortRAMAddr(WRITE_BBREG3, 0x38);
    //    PHYSetShortRAMAddr(WRITE_BBREG4, 0x5C);
    //
    //    PHYSetShortRAMAddr(WRITE_RFCTL, 0x04);
    //    PHYSetShortRAMAddr(WRITE_RFCTL, 0x00);
    //
    //#endif    

    //    17. RFCTL (0x36) = 0x04 ? Reset RF state machine.
    //    18. RFCTL (0x36) = 0x00.

    __delay(1); //    19. Delay at least 192 ?s.
}

void MRF24J40_Disable() {

#if (MRF24J40_MODULE_TYPE == MRF24J40MA) 

#elif (MRF24J40_MODULE_TYPE == MRF24J40MD) // Disable PA and LNA    
    // MRF24J40MD/ME: GPIO1, GPIO2 and GPIO3 are connected
    MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure normal operation
    MRF24J40_writeShort(MRF_GPIO, 0); // Set all GPIO to 0 to disable PA and LNA
#endif

    MRF24J40_writeShort(MRF_SOFTRST, 0b00000100); // Perform Power Management Reset (RSTPWR = 1)
    MRF24J40_writeShort(MRF_WAKECON, 0b10000000); // Enable Immediate Wake-up Mode (IMMWAKE = 1)
    MRF24J40_writeShort(MRF_SLPACK, 0b10000000); // Put the module to sleep (SLPACK = 1)

    if (disable_spi1) { // Shared SPI1
        SPI1_Disable();
    }
    MRF24_SS_SetLow(); // CS low !
}

void MRF24J40_setPanId(uint16_t panid) {
    net_pan = panid;
    MRF24J40_writeShort(MRF_PANIDH, net_pan >> 8);
    MRF24J40_writeShort(MRF_PANIDL, net_pan);
}

void MRF24J40_setAddress(uint16_t addr) {
    net_addr = addr;
    MRF24J40_writeShort(MRF_SADRH, addr >> 8);
    MRF24J40_writeShort(MRF_SADRL, addr);
}

void MRF24J40_setChannel(uint8_t rfchannel) {
    if (rfchannel < 11 || rfchannel > 26) {
        return;
    }
    rf_channel = rfchannel; // Change channel
    MRF24J40_writeLong(MRF_RFCON0, (rfchannel - 11) << 4 | 0x03);

    // Perform RF state machine reset and wait for RF circuitry to calibrate
    MRF24J40_writeShort(MRF_RFCTL, 0b00000100);
    MRF24J40_writeShort(MRF_RFCTL, 0);
    _delay_us(200);
}

//void MRF24J40_enablePaLna() {
//    if (type != MRF24J40MA) {
//        MRF24J40_writeShort(MRF_TRISGPIO, 0); // Configure all GPIO pins as input
//        MRF24J40_writeShort(MRF_GPIO, 0);
//        MRF24J40_writeLong(MRF_TESTMODE, 0b00001111); // Configure RF state machine for automatic PA/LNA control
//    }
//    if (type == MRF24J40ME) {
//        //#if defined(MRF24J40MB) // Special MRF24J40 setting for Microchip MRF24J40MB module
//        // MRF24J40_writeLong(MRF_RFCTRL3, 0x70); // EUROPE output power set to be -14.9dB
//        MRF24J40_writeLong(MRF_RFCTRL3, 0x18); // USA output power set to be -1.9dB   
//        // MRF24J40_writeLong(MRF_RFCTRL3, 0x28);    // MRF24J40 output power set to be -3.7dB for MRF24J40MB/MC
//        // MRF24J40_writeLong(MRF_RFCTRL3, 0x00);    // set to 0dBm, must adjust according to FCC/IC/ETSI requirement
//        //#endif
//    }
//}

void MRF24J40_reset() {
    MRF24J40_writeShort(MRF_SOFTRST, 0b00000111); // Perform full software reset (RSTPWR = 1, RSTBB = 1, RSTMAC = 1)
    //    17. RFCTL (0x36) = 0x04 ? Reset RF state machine.
    //    18. RFCTL (0x36) = 0x00.

}

//void MRF24J40_init() {
//}

float MRF24J40_getSignalStrength() {
    return rf_rssi * 0.197948 - 87.646977;
}

//float MRF24J40_measureSignalStrength() {
//    
//    // Disable PA
//    if (type != MRF24J40MA) {
//        MRF24J40_writeLong(MRF_TESTMODE, 0b00001000); // Configure RF state machine for normal operation
//        MRF24J40_writeShort(MRF_TRISGPIO, 0b00000110); // Configure GPIO1 and GPIO2 for output
//        MRF24J40_writeShort(MRF_GPIO, 0b00000100); // Set GPIO1 0 to disable PA and GPIO2 to 1 to enable LNA
//    }
//
//    // Initiate RSSI calculation (RSSIMODE1 = 1)
//    MRF24J40_writeShort(MRF_BBREG6, 0b11000000);
//
//    // Wait until conversion is ready and read RSSI
//    while (!(MRF24J40_readShort(MRF_BBREG6) & 0b00000001));
//    rssi = MRF24J40_readLong(MRF_RSSI);
//
//    // Re-enable PA
//    //   MRF24J40_enablePaLna();
//
//    return MRF24J40_getSignalStrength();
//}


// -----------------------------------------
// | b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
// -----------------------------------------

void MRF24J40_rxFlush() {
    MRF24J40_writeShort(MRF_RXFLUSH, 0b00000001);
}

void MRF24J40_TxBuffer(uint16_t dstaddr, uint8_t *data, uint8_t length, bool ack) {
    // Send header size, frame size and header
    int j = 0;
    int i = 0;
    MRF24J40_writeLong(i++, MRF_MHR_SIZE); // Header size
    MRF24J40_writeLong(i++, MRF_MHR_SIZE + length); // Frame size
    MRF24J40_writeLong(i++, ack ? 0b01100001 : 0b01000001); // Frame control bits 0-7 (data frame, no security, no pending data, ack disabled, intra-PAN)
    MRF24J40_writeLong(i++, 0b10001000); // Frame control bits 8-15 (16-bit addresses with PAN)

#ifdef MRF24J40_PROMISCUOUS
    MRF24J40_writeLong(i++, seqNumber); // Sequence number
#else
    MRF24J40_writeLong(i++, seqNumber++); // Sequence number
#endif
    MRF24J40_writeLong(i++, net_pan); // PAN ID
    MRF24J40_writeLong(i++, net_pan >> 8);

    MRF24J40_writeLong(i++, dstaddr); // Destination address
    MRF24J40_writeLong(i++, dstaddr >> 8);

    MRF24J40_writeLong(i++, net_addr); // Source address
    MRF24J40_writeLong(i++, net_addr >> 8);

    // Send data payload
    for (j = 0; j < length; j++) {
        MRF24J40_writeLong(i++, data[j]);
    }

    // Start transmission (0x37 0x01)
    MRF24J40_writeShort(MRF_TXNCON, ack ? 0b00000101 : 0b00000001);
}

uint8_t MRF24J40_RxBuffer(uint8_t* data, int size) {
    uint8_t maxSize = size;
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
    if (!((rxSize - rxCount) == 0)) {
        return true;
    } else {
        return MRF24J40_receivePacket();
    }
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

#ifdef MRF24J40_PROMISCUOUS
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
        rf_lqi = MRF24J40_readLong(0x301 + frameSize);
        rf_rssi = MRF24J40_readLong(0x301 + frameSize + 1);

        // Flush the reception buffer, re-enable receiver
        MRF24J40_rxFlush();

        MRF24J40_writeShort(MRF_BBREG1, 0);

        // Wait until RXIF is cleared (takes a while)
        while (MRF24J40_readShort(MRF_INTSTAT) & 0b00001000);
        return true;
    }
    return false;
}

// SPI Interfacing routines
// ---------------------------------------------------------------------------//

void MRF24J40_writeShort(uint8_t addr, uint8_t data) {
    uint8_t dataTransmitted[2];
    MRF24_SS_SetLow();
    dataTransmitted[0] = ((addr << 1 & 0b01111110) | 0b00000001);
    dataTransmitted[1] = data;
    SPI1_Exchange8bitBuffer(dataTransmitted, 2, NULL);
    MRF24_SS_SetHigh();
}

uint8_t MRF24J40_readShort(uint8_t addr) {
    uint8_t toReturn = 0x00;
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

uint8_t MRF24J40_readLong(uint16_t addr) {
    uint8_t toReturn;
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
