
#ifndef MRF24J40_HEADER_H
#define	MRF24J40_HEADER_H

#include <xc.h> // include processor files - each processor file is guarded.
//#define  FCY  CLOCK_SystemFrequencyGet()     // Instruction cycle frequency, Hz - required for __delayXXX() to work
//#include <libpic30.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


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

#define MRF24J40_DONGLE_ADDRESS 0x8888
#define MRF24J40_DEVICE_ADDRESS 0xCCCC

    enum MRF24J40Type {
        MRF24J40MA, MRF24J40MB, MRF24J40MC, MRF24J40MD, MRF24J40ME
    };

    void MRF24J40_Enable();

    void MRF24J40_init();

    void MRF24J40_Disable();

    void MRF24J40_setAddress(uint16_t addr);

    void MRF24J40_TxBuffer(uint16_t addr, uint8_t *data, uint8_t length, bool ack);

    bool MRF24J40_IsRxReady();
    bool MRF24J40_receivePacket();

    void MRF24J40_rxFlush();

    int MRF24J40_RxBuffer(uint8_t* data, int size);

    bool MRF24J40_transmissionDone();

    bool MRF24J40_transmissionSuccess();

    float MRF24J40_measureSignalStrength();

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MRF24J40_HEADER_H */

