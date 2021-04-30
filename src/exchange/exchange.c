
#include "xc.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "../utils.h"   // <stdio.h> printf
#include "../device.h" // isUsbConnected()

#include "uart2.h"
#include "MRF_spi1.h"
#include "MRF24J40.h"

#include "exchange.h"

#ifndef __DONGLE_PASSTHRU

extern uint16_t rfDestinationAddr;

/* -------------------------------------------------------------------------- */
devicestate_t exchangeHandler() {
    RealTimeCommandType rtCommand;
    devicestate_t devicemode = TOSLEEP;

    exchangestate_t state = EXCH_OPEN;

    do {
        switch (state) {
            case EXCH_OPEN:
                if (!Exchange_Connect(Device_IsUsbConnected())) {
                    state = EXCH_EXIT;
                } else {
                    state = EXCH_START_DISCOVERY;
                }
                break;
            case EXCH_START_DISCOVERY:
                if (Exchange_sendHandshake()) {
                    state = EXCH_WAIT_COMMAND;
                } else {
                    state = EXCH_EXIT;
                }
                break;
            case EXCH_WAIT_COMMAND:
                Exchange_commandsHandler(&rtCommand);
                state = EXCH_EXIT;
                break;
            case EXCH_EXIT:
                break;
            default:
                break;
        }
    } while (state != EXCH_EXIT);

    Exchange_closeChannel();
    return devicemode;
}

#endif //#ifndef __DONGLE_PASSTHRU

