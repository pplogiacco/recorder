#ifndef EXCHANGE_H
#define EXCHANGE_H

#include "xc.h"
#include <string.h>
#include <stdbool.h>

#include "sampling.h"

// ! ___________________________________              

typedef enum {
    cmdMeasure,
    cmdRetrieve,
    cmdRetrieveAndDelete,
    cmdSetConfig,
    cmdGetConfig,
    cmdTOSLEEP,
    cmdREBOOT
} ex_cmd_t;
// ! __________________________________                  



/*********************************************************************/
/* Fixed Packet Size Protocol, M2M forced type casting               */

#define ex_t short             // base-type to exchange
#define EX_BLK_SIZE 12         // optimize 12bits samples block transfer
#define EX_OPT_ADIM 2          // packet option array dimension ( 6 Byte )  
#define EX_PKT_SIZE 26         // Bytes.. (EX_BLK_SIZE * 2) +2
#define EX_HSK_MAGIC 0xAAAA    // Magic packet to handshake

typedef enum {
    _HSK = 0x00, // Handshake (PC<->Device)
    _ACK = 0xFF, // Acknowledgement (PC<->Device)           
    _MRQ = 0x06, // Request RT measurements (PC->Device)            
    _TMH = 0x08, // Exchange Measurement Header (PC<-Device)       
    _TMS = 0x0A, // Exchange Measurement Samples  (PC<-Device)   
    _OGT = 0x02, // Request Config Option  (PC->Device)         
    _OTX = 0x03, // Exchange Option value (PC<-Device) 
    _OST = 0x04, // Request Change Option (PC->Device)                
    _CMD = 0x0A // VMS/User control sub-protocol.......
} ex_pkt_t;

typedef struct { // Handshacke
    ex_t key[2]; // 32 Bit
} ex_hsk_t;

typedef struct { // _CGT / _CST Payload  

    struct {
        ex_t nopt; //  noption (0 = null)
        ex_t value[2]; //  value 32 Bit  (Double type)
    } opts[EX_OPT_ADIM]; // (2+4=6)*2 = 12 byte 
} ex_opt_t;

typedef struct { // _TXH Measurement Header
    ex_t typeset;
    ex_t dtime[2]; // Datetime 32 bit
    ex_t nsamp; // Number of sample 
} ex_mhe_t;

typedef struct { // CTRL packet 2 + 24(max)
    ex_pkt_t pkt; // +2 bytes
    // payloadsize
    // payload

    union {
        ex_hsk_t hsk; // 2
        ex_opt_t opt; // 12
        ex_mhe_t hdr; // 12          
        ex_t samples[EX_BLK_SIZE];
    };
    // checksum     
} pkt_t;

/*********************************************************************/


bool exOpen(short timeout);

bool exHandshake(short timeout);
void exSendPacket(pkt_t *pkt);
bool exSendMeasurement(measurement_t *ms);

bool exWaitAck(short timeout);
ex_cmd_t exWaitCmd(short timeout);

void exClose();


// Measurement storage
// -------------
bool loadMeasurement(measurement_t *ms);
bool saveMeasurement(measurement_t *ms);
void freeMeasurement(measurement_t *ms);


#endif // EXCHANGE_H
