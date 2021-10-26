// UART2 Exchange - RECORDER

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "xc.h"
#include "UART2.h"


static uint8_t * volatile rxTail;
static uint8_t *rxHead;
static uint8_t *txTail;
static uint8_t * volatile txHead;
static bool volatile rxOverflowed;

/** UART Driver Queue Length */

/* We add one extra byte than requested so that we don't have to have a separate
 * bit to determine the difference between buffer full and buffer empty, but
 * still be able to hold the amount of data requested by the user.  Empty is
 * when head == tail.  So full will result in head/tail being off by one due to
 * the extra byte.
 */
#define UART2_CONFIG_TX_BYTEQ_LENGTH (8+1)
#define UART2_CONFIG_RX_BYTEQ_LENGTH (128+1)

/** UART Driver Queue */
static uint8_t txQueue[UART2_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t rxQueue[UART2_CONFIG_RX_BYTEQ_LENGTH];

void (*UART2_TxDefaultInterruptHandler)(void);
void (*UART2_RxDefaultInterruptHandler)(void);


#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

/* -------------------------------------------------------------------------- */
void UART2_Initialize(void) {
    IEC1bits.U2TXIE = 0;
    IEC1bits.U2RXIE = 0;

    // ___UART2 PIC24Fv32KA301/302
    ANSBbits.ANSB0 = 0;
    ANSBbits.ANSB1 = 0;
    TRISBbits.TRISB0 = 0; // RB0 Out (4 DIP20)
    TRISBbits.TRISB1 = 1; // RB1 In (5 DIP20)
    LATBbits.LATB0 = 1; // Set TxPin high


    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX;
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U2MODE = (0x8008 & ~(1 << 15)); // disabling UART ON bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled;
    U2STA = 0x00;
    // BaudRate = 115200; Frequency = 4000000 Hz; BRG 8;
    // UART_SPBRG   = (SYS_CLK_FrequencySystemGet()/2/16)/BAUD_RATE-1;
    U2BRG = 0x22; // 0x08;

    txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;

    rxOverflowed = false;

    UART2_SetTxInterruptHandler(&UART2_Transmit_CallBack);
    UART2_SetRxInterruptHandler(&UART2_Receive_CallBack);

    IEC1bits.U2RXIE = 1;
    U2MODEbits.UARTEN = 1; // enabling UART ON bit
    U2STAbits.UTXEN = 1;
}

/* --------------------------------------------------------------------------
    Maintains the driver's transmitter state machine and implements its ISR
 */
void UART2_SetTxInterruptHandler(void (* interruptHandler)(void)) {
    if (interruptHandler == NULL) {
        UART2_TxDefaultInterruptHandler = &UART2_Transmit_CallBack;
    } else {
        UART2_TxDefaultInterruptHandler = interruptHandler;
    }
}

/* -------------------------------------------------------------------------- */
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    (*UART2_TxDefaultInterruptHandler)();

    if (txHead == txTail) {
        IEC1bits.U2TXIE = 0;
    } else {
        IFS1bits.U2TXIF = 0;

        while (!(U2STAbits.UTXBF == 1)) {
            U2TXREG = *txHead++;

            if (txHead == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH)) {
                txHead = txQueue;
            }

            // Are we empty?
            if (txHead == txTail) {
                break;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
void __attribute__((weak)) UART2_Transmit_CallBack(void) {

}

/* -------------------------------------------------------------------------- */
void UART2_SetRxInterruptHandler(void (* interruptHandler)(void)) {
    if (interruptHandler == NULL) {
        UART2_RxDefaultInterruptHandler = &UART2_Receive_CallBack;
    } else {
        UART2_RxDefaultInterruptHandler = interruptHandler;
    }
}

/* -------------------------------------------------------------------------- */
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    (*UART2_RxDefaultInterruptHandler)();

    IFS1bits.U2RXIF = 0;

    while ((U2STAbits.URXDA == 1)) {
        *rxTail = U2RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ((rxTail != (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH - 1)) &&
                ((rxTail + 1) != rxHead)) {
            rxTail++;
        } else if ((rxTail == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH - 1)) &&
                (rxHead != rxQueue)) {
            // Pure wrap no collision
            rxTail = rxQueue;
        } else // must be collision
        {
            rxOverflowed = true;
        }
    }
}

/* -------------------------------------------------------------------------- */
void __attribute__((weak)) UART2_Receive_CallBack(void) {

}

/* -------------------------------------------------------------------------- */
void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    if ((U2STAbits.OERR == 1)) {
        U2STAbits.OERR = 0;
    }

    IFS4bits.U2ERIF = 0;
}

/* -------------------------------------------------------------------------- */
uint8_t UART2_Read(void) {
    uint8_t data = 0;

    while (rxHead == rxTail) {
    }

    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH)) {
        rxHead = rxQueue;
    }
    return data;
}

/* -------------------------------------------------------------------------- */
void UART2_Write(uint8_t byte) {
    while (UART2_IsTxReady() == 0) {
    }

    *txTail = byte;

    txTail++;

    if (txTail == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH)) {
        txTail = txQueue;
    }
    IEC1bits.U2TXIE = 1;
}

/* -------------------------------------------------------------------------- */
bool UART2_IsRxReady(void) {
    return !(rxHead == rxTail);
}

/* -------------------------------------------------------------------------- */
bool UART2_IsTxReady(void) {
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*) txHead;

    if (txTail < snapshot_txHead) {
        size = (snapshot_txHead - txTail - 1);
    } else {
        size = (UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1);
    }
    return (size != 0);
}

/* -------------------------------------------------------------------------- */
bool UART2_IsTxDone(void) {
    if (txTail == txHead) {
        return (bool) U2STAbits.TRMT;
    }
    return false;
}

/* -------------------------------------------------------------------------- */
void UART2_Enable(void) {

    //    Priority: 1
    IPC16bits.U2ERIP = 1;
    //    UTXI: U2TX - UART2 Transmitter
    //    Priority: 1
    IPC7bits.U2TXIP = 1;
    //    URXI: U2RX - UART2 Receiver
    //    Priority: 1
    IPC7bits.U2RXIP = 1;

    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
}

/* -------------------------------------------------------------------------- */
void UART2_Disable(void) {
    //    Priority: 1
    IPC16bits.U2ERIP = 0;
    //    UTXI: U2TX - UART2 Transmitter
    //    Priority: 1
    IPC7bits.U2TXIP = 0;
    //    URXI: U2RX - UART2 Receiver
    //    Priority: 1
    IPC7bits.U2RXIP = 0;

    U2MODEbits.UARTEN = 0;
    U2STAbits.UTXEN = 0;
}
#endif 



#if defined(__PIC24FJ256GA702__)

void UART2_Enable(void) {
    IEC1bits.U2TXIE = 0;
    IEC1bits.U2RXIE = 0;
    
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; 
    // USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; 
    // BRGH enabled; URXINV disabled; UEN TX_RX; 
    U2MODE = (0x8008 & ~(1 << 15)); // disabling UART ON bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; URXEN disabled; OERR NO_ERROR_cleared;
    // URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U2STA = 0x00;
    // BaudRate = 115200; Frequency = 16000000 Hz;  U2BRG 34; 
    U2BRG = 0x22;
    // ADMADDR 0; ADMMASK 0; 
    U2ADMD = 0x00;

    txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;

    rxOverflowed = false;

    UART2_SetTxInterruptHandler(&UART2_Transmit_CallBack);
    UART2_SetRxInterruptHandler(&UART2_Receive_CallBack);

     IEC1bits.U2RXIE = 1;    //Make sure to set TxPin LAT bit High !!
     U2MODEbits.UARTEN = 1; // enabling UART ON bit
     U2STAbits.UTXEN = 1;
}


void UART2_Disable(void) {
   // IEC1bits.U2RXIE = 0;    //Make sure to set TxPin LAT bit High !!
    U2MODEbits.UARTEN = 0;
    U2STAbits.UTXEN = 0;
}


void UART2_SetTxInterruptHandler(void (* interruptHandler)(void)) {
    if (interruptHandler == NULL) {
        UART2_TxDefaultInterruptHandler = &UART2_Transmit_CallBack;
    } else {
        UART2_TxDefaultInterruptHandler = interruptHandler;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    (*UART2_TxDefaultInterruptHandler)();

    if (txHead == txTail) {
        IEC1bits.U2TXIE = 0;
    } else {
        IFS1bits.U2TXIF = 0;

        while (!(U2STAbits.UTXBF == 1)) {
            U2TXREG = *txHead++;

            if (txHead == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH)) {
                txHead = txQueue;
            }

            // Are we empty?
            if (txHead == txTail) {
                break;
            }
        }
    }
}

void __attribute__((weak)) UART2_Transmit_CallBack(void) {
}

void UART2_SetRxInterruptHandler(void (* interruptHandler)(void)) {
    if (interruptHandler == NULL) {
        UART2_RxDefaultInterruptHandler = &UART2_Receive_CallBack;
    } else {
        UART2_RxDefaultInterruptHandler = interruptHandler;
    }
}

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    (*UART2_RxDefaultInterruptHandler)();

    IFS1bits.U2RXIF = 0;

    while ((U2STAbits.URXDA == 1)) {
        *rxTail = U2RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ((rxTail != (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH - 1)) &&
                ((rxTail + 1) != rxHead)) {
            rxTail++;
        } else if ((rxTail == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH - 1)) &&
                (rxHead != rxQueue)) {
            // Pure wrap no collision
            rxTail = rxQueue;
        } else // must be collision
        {
            rxOverflowed = true;
        }
    }
}

void __attribute__((weak)) UART2_Receive_CallBack(void) {

}

void __attribute__((interrupt, no_auto_psv)) _U2ErrInterrupt(void) {
    if ((U2STAbits.OERR == 1)) {
        U2STAbits.OERR = 0;
    }

    IFS4bits.U2ERIF = 0;
}

/**
  Section: UART Driver Client Routines
 */

uint8_t UART2_Read(void) {
    uint8_t data = 0;

    while (rxHead == rxTail) {
    }

    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH)) {
        rxHead = rxQueue;
    }
    return data;
}

void UART2_Write(uint8_t byte) {
    while (UART2_IsTxReady() == 0) {
    }

    *txTail = byte;

    txTail++;

    if (txTail == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH)) {
        txTail = txQueue;
    }

    IEC1bits.U2TXIE = 1;
}

bool UART2_IsRxReady(void) {
    return !(rxHead == rxTail);
}

bool UART2_IsTxReady(void) {
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*) txHead;

    if (txTail < snapshot_txHead) {
        size = (snapshot_txHead - txTail - 1);
    } else {
        size = (UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1);
    }

    return (size != 0);
}

bool UART2_IsTxDone(void) {
    if (txTail == txHead) {
        return (bool) U2STAbits.TRMT;
    }

    return false;
}



#endif // __PIC24FJ256GA702__

/* -------------------------------------------------------------------------- */

int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) {
    
#if (1)    
    unsigned int i;
    uint8_t *data = buffer;

    for (i = 0; i < len; i++) {
        while (UART2_IsTxReady() == false) {
        }
        UART2_Write(*data++);
    }
#endif
    
    return (len);
}