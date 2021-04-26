/**
 File: exchange.h
 */

#include "../device.h"
#include "protocol.h"

#ifndef EXCHANGE_H
#define EXCHANGE_H


/* -------------------------------------------------------------------------- */
devicestate_t exchangeHandler();

#define Exchange_Connect(x) Exchange_openChannel()
#define Exchange_Disconnect() Exchange_closeChannel()


#endif // EXCHANGE_H
