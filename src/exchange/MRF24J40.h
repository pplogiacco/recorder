
#ifndef MRF24J40_HEADER_H
#define	MRF24J40_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.
//#define  FCY  CLOCK_SystemFrequencyGet()     // Instruction cycle frequency, Hz - required for __delayXXX() to work
//#include <libpic30.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define APPLICATION_SITE  EUROPE    
// #define TURBO_MODE    
    
    typedef enum {
        MRF24J40MA, // Simple module no PA/LNA 
        //        MRF24J40MB,
        //        MRF24J40MC,
        MRF24J40MD // MRF24J40ME module with external PA/LNA
    } mrftype_t;



    void MRF24J40_Enable(mrftype_t type, uint8_t channel, uint16_t macaddr, uint16_t panid);
    void MRF24J40_Disable();

    void MRF24J40_reset();
    //    void MRF24J40_init();
    void MRF24J40_setChannel(uint8_t channel);
    void MRF24J40_setPanId(uint16_t panId);
    void MRF24J40_setAddress(uint16_t addr);

    void MRF24J40_TxBuffer(uint16_t addr, uint8_t *data, uint8_t length, bool ack);

    bool inline MRF24J40_IsRxReady();
    bool MRF24J40_receivePacket();

    void MRF24J40_rxFlush();

    uint8_t MRF24J40_RxBuffer(uint8_t* data, int size);

    bool MRF24J40_transmissionDone();
    bool MRF24J40_transmissionSuccess();

    //    float MRF24J40_measureSignalStrength();

    //#define MRF24J40_DONGLE_ADDRESS 0x8888
    //#define MRF24J40_DEVICE_ADDRESS 0xCCCC
    //#define MRF24J40_RF_CHANNEL     11
    //#define MRF24J40_MODULE_TYPE    MRF24J40MD



#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MRF24J40_HEADER_H */

