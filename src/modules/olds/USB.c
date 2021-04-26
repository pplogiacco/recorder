#include "USB.h"
#include <stdint.h>
#include "../device.h"

static char * volatile rxTail; // OTTIMIZZARE 16BIT MCU !!!!!!!!!!!!!!!!
static char *rxHead;
static char *txTail;
static char * volatile txHead;
static bool volatile rxOverflowed;

#define Q_SIZE 32 // EX_PKT_SIZE * 2;

static char txQueue[Q_SIZE];
static char rxQueue[Q_SIZE];

#define BAUDRATE 115200     // 57600
#define BR_BRG   0x08     // BaudRate = 115200; Frequency = 4000000 Hz; BRG 8;
//#define BR_BRG   0x10     // BaudRate = 57600; Frequency = 4000000 Hz; BRG 16;
//#define BR_BRG     0x22     // BaudRate = 57600; Frequency = 8000000 Hz; BRG 34;

void USB_Enable(void) { // PIC24Fv32KA304 (301-20Pin,V-5Volt)

    // ____________________UART2 PIC24Fv32KA301
    LATBbits.LATB0 = 1; // Set TxPin high
    TRISBbits.TRISB0 = 1; // RB0 Out (4 DIP20)
    TRISBbits.TRISB1 = 0; // RB1 In (5 DIP20)

    U2MODEbits.UARTEN = 0; // Disabling UART Module
    U2MODEbits.BRGH = 1; // High Speed Mode Enable bit
    U2MODEbits.UEN = 0; // xTX and UxRX pins are enabled and used; UxCTS and UxRTS/UxBCLK pins are free
    // Default settings Data Bits = 8; Parity = None; Stop Bits = 1;

    IEC1bits.U2TXIE = 0; // UART2 Disable Interrupts
    IPC7bits.U2TXIP = 3; // Set high priority
    IEC1bits.U2RXIE = 0;
    IPC7bits.U2RXIP = 1;

    U2STA = 0x00; // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled;

    //U2BRG = BR_BRG; // For PIC24, if UART in High Speed Mode (BRGH = 1) divide by 4 otherwise by 16.
    //U2BRG = (unsigned int) ((((double) SYSCLK / 2.0) / ((double) BAUDRATE * (U2MODEbits.BRGH) ? 4 : 16)) - 1.0);
    U2BRG = 0x22;

    txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;
    rxOverflowed = false;

    IEC1bits.U2RXIE = 1; // Enable RX Interrupt
    U2MODEbits.UARTEN = 1; // Enabling UART
    U2STAbits.UTXEN = 1; // Enable UART module
}

inline void USB_Disable(void) {
    U2MODEbits.UARTEN = 0;
}

/********************************* TX
3. Set the UTXEN bit (causes a transmit interrupt, two cycles after being set).
4. Write the data byte to the lower byte of the UxTXREG word.
   The value will be immediately transferred to the Transmit Shift Register (TSR)
   and the serial bit stream will start shifting out with the next rising edge of the baud clock.
5. Alternately, the data byte may be transferred while UTXEN = 0 and then, the user may set UTXEN.
   This will cause the serial bit stream to begin immediately, because the baud clock will
   start from a cleared state.
6. A transmit interrupt will be generated as per interrupt control bit, UTXISELx.
 */

bool USB_IsTxReady(void) {
    uint16_t size;
    char *snapshot_txHead = (char*) txHead;

    if (txTail < snapshot_txHead) {
        size = (snapshot_txHead - txTail - 1); // re-circle
    } else {
        size = (Q_SIZE - (txTail - snapshot_txHead));
    }
    // ( MAX_Q_SIZE - (txTail - snapshot_txHead) - 1 ); }

    return (size != 0);
}

bool USB_IsTxDone(void) {
    if (txTail == txHead) {
        return (bool) U2STAbits.TRMT;
    } // TRMT: Transmit Shift Register Empty bit (read-only)
    return false;
}

void USB_Write(char byte) {
    while (USB_IsTxReady() == 0) {
    }
    *txTail = byte;
    txTail++;
    if (txTail == (txQueue + Q_SIZE)) {
        txTail = txQueue;
    } // Circular
    IEC1bits.U2TXIE = 1; // Enable TX Interrupt
}

void USB_WriteBuffer(const char* cbuf, uint16_t blen) {
    uint16_t i = 0;
    while (i < blen) {
        USB_Write(*(cbuf + i));
        i++;
    }

    /* da testare !
    while( UART2_IsTxReady() == 0) { }  // ensure last TX completed
    if ( blen < ( (txQueue + Q_SIZE) - txTail) ){
     memcpy(cbuf, txTail, blen);
     txTail+=blen;
     IEC1bits.U2TXIE = 1; // Enable TX Interrupt
    } else { // re start circular
        .....
    }
     */
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    if (txHead == txTail) {
        IEC1bits.U2TXIE = 0;
    }// Empty buffer
    else {
        IFS1bits.U2TXIF = 0; // Reset flag
        while (!(U2STAbits.UTXBF == 1)) {
            U2TXREG = *txHead++;
            if (txHead == (txQueue + Q_SIZE)) {
                txHead = txQueue;
            }
            if (txHead == txTail) {
                break;
            } // Are we empty?
        }
    }
}

/********************************* RX
 * 3. A receive interrupt will be generated when one
or more data characters have been received, as
per interrupt control bit, URXISELx.
4. Read the OERR bit to determine if an overrun
error has occurred. The OERR bit must be reset
in software.
5. Read UxRXREG.
The act of reading the UxRXREG character will move
the next character to the top of the receive FIFO,
including a new set of PERR and FERR values.
 */

bool USB_IsRxReady(void) {
    return !(rxHead == rxTail);
}

uint8_t USB_Read(char* buf, uint16_t len) {
    uint8_t n = 0;

    while ((rxHead != rxTail) && (n < len)) {
        *(buf + n) = *rxHead;
        rxHead++;
        n++;
        if (rxHead == (rxQueue + Q_SIZE)) // circular
        {
            rxHead = rxQueue;
        }
    };
    return (n);


    /*
    uint8_t data = 0;
    while (rxHead == rxTail ) { }   // bloccante
    data = *rxHead;
    rxHead++;
    if (rxHead == (rxQueue + Q_SIZE))
    { rxHead = rxQueue; }
    return data;
     */
}

/*
uint8_t UART2_ReadPkt(pkt_t *pkt)   // Return >0 if packet received
{
    uint8_t data = 0;
    while (rxTail != rxHead+Q_SIZE) { } // wait to receive packet

   // pkt =  *rxHead;
    rxHead++;
    if (rxHead == (rxQueue + Q_SIZE))
    { rxHead = rxQueue; }

    return data;
}
 */

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    IFS1bits.U2RXIF = 0;
    while ((U2STAbits.URXDA == 1)) {
        *rxTail = U2RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ((rxTail != (rxQueue + Q_SIZE - 1)) && ((rxTail + 1) != rxHead)) {
            rxTail++;
        } else if ((rxTail == (rxQueue + Q_SIZE - 1)) && (rxHead != rxQueue)) {
            // Pure wrap no collision
            rxTail = rxQueue;
        } else // must be collision
        {
            rxOverflowed = true;
        }
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    if ((U2STAbits.OERR == 1)) {
        U2STAbits.OERR = 0;
    }
    IFS4bits.U2ERIF = 0;
}

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) {
    unsigned int i;
    uint8_t *data = buffer;

    for (i = 0; i < len; i++) {
        while (USB_IsTxReady() == false) {
        }
        USB_Write(*data++);
    }
    return (len);
}


