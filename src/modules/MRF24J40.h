// Recorder !
#ifndef MRF24J40_HEADER_H
#define	MRF24J40_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.
//#define  FCY  CLOCK_SystemFrequencyGet()     // Instruction cycle frequency, Hz - required for __delayXXX() to work
//#include <libpic30.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */



// Configuration
#define MRF24J40_PAN_ID         01234       // Not managed by init /open Libex routines
#define MRF24J40_RF_CHANNEL     11          // Not managed by init /open Libex routines
//
#define APPLICATION_SITE        EUROPE      
#define MRF24J40_MODULE_TYPE    MRF24J40MA  // MRF24J40MD  !!!! GET FROM Module's register 
    
#undef MRF24J40_PROMISCUOUS      // Accept all packets
#undef MRF24J40_TURBO_MODE     

    void MRF24J40_Enable(uint16_t macaddr);
    void MRF24J40_Disable();

    void MRF24J40_reset();
    //    void MRF24J40_init();
    void MRF24J40_setChannel(uint8_t channel);
    void MRF24J40_setPanId(uint16_t panId);
    void MRF24J40_setAddress(uint16_t addr);

    void MRF24J40_TxBuffer(uint16_t addr, uint8_t *data, uint8_t length, bool ack);

    bool MRF24J40_IsRxReady();
    bool MRF24J40_receivePacket();

    void MRF24J40_rxFlush();

    uint8_t MRF24J40_RxBuffer(uint8_t* data, int size);

    bool MRF24J40_transmissionDone();
    bool MRF24J40_transmissionSuccess();

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MRF24J40_HEADER_H */

