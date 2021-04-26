
#include "modules\USB.h"
#include "exchange.h"
#include "utils.h"
#include "sampling.h"

bool loadMeasurement(measurement_t *ms) {
    return (true);
};

bool saveMeasurement(measurement_t *ms) {
    return (true);
};

void freeMeasurement(measurement_t *ms) {
};

bool exOpen(short timeout) {
    // called from USB wake-up
    //  #1:  try USB_Open(); se ok non va avanti
    //  #2:  try RF_Open(vms-id, timeout);
    // check USB Connected Pin
    // MCP 2221  USBCFG(GP2) Pin 7 <-> PIC24 ?


    UART2_Enable();


    return (true);
}

void exClose() {
    UART2_Disable();
};

void exSendPacket(pkt_t *pkt) {
    UART2_WriteBuffer((char*) pkt, EX_PKT_SIZE);
}

bool exHandshake(short timeout) {
    pkt_t toex;
    short magic[] = {EX_HSK_MAGIC, EX_HSK_MAGIC};
    UART2_WriteBuffer((char*) magic, sizeof (magic));
    __delay_ms(5); // wait host


    if (UART2_Read((char*) &toex, EX_PKT_SIZE) == EX_PKT_SIZE) {
        if ((toex.hsk.key[0] == toex.hsk.key[1]) && (toex.hsk.key[0] == EX_HSK_MAGIC)) {
            return true;
        }
    }
    return false;
}

bool exWaitAck(short timeout) {
    return (true);
}

ex_cmd_t exWaitCmd(short timeout) // on_Receive
{
    return (cmdRetrieve); // Test !!!
};

bool exSendMeasurement(measurement_t *ms) {
    //UART2_SendBuffer((short*) ms->ss , (ms->ns+ms->nss));
    int i = 0, ns = 0;
    pkt_t tosend;
    tosend.pkt = _TMH; // Trasmit Measurement Header
    tosend.hdr.dtime[0] = (ms->dtime) >> 16;
    tosend.hdr.dtime[1] = (ms->dtime);
    tosend.hdr.nsamp = (ms->ns + ms->nss);
    tosend.hdr.typeset = (ex_t) ms->typeset;
    exSendPacket(&tosend);

    tosend.pkt = _TMS; // Trasmit Measurement Samples
    while (ns < (ms->ns + ms->nss)) {
        tosend.samples[i++] = *(ms->ss + ns);
        ns++;
        if (i == EX_BLK_SIZE) {
            i = 0;
            exSendPacket(&tosend);
        }
    }
    if (i > 0) {
        exSendPacket(&tosend);
    } // Trasmit Measurement Samples
    //ret = UART2_WaitAck(timeout);
    return (true);
}






/*
void sendPkt(pkt_t* spkt)
{
  int x;
  uint8_t *cbuf;
  cbuf = (uint8_t*)spkt;
  for(x=0; x<EX_PKT_SIZE; x++)
  {
    UART2_Write( (uint8_t)x );
    //UART2_Write( *(cbuf+x) );
    //__delay_ms(5); // serial speed
  }
}*/

/*
int waitHost()  // bind vms
{
    int ok =0;
    int timeout = 100;

     pkt_t tosend;
     tosend.cmd = _HCK;
    // tosend.hdr.DIN = 0x0;       // Device serial


     while( UART2_IsTxReady() )
     {
         UART2_WritePkt(&tosend);
         __delay_ms(50); timeout--;
        // UART2_Write(0x00);
        if( UART2_Read()>0 ) {ok=1;}
     }
    return(ok);
}
 */


