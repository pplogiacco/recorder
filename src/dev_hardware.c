
#include "modules/RTCC.h"
#include "memory/DEE/dee.h"
#include "dev_hardware.h"
#include "device.h"

extern device_t device;

/* CLOCK Switching Settings
   COSC = 001; // 8 MHz FRC Oscillator with Postscaler and PLL module (FRCPLL)
   NOSC = 001; // 8 MHz FRC Oscillator with Postscaler and PLL module (FRCPLL)
   CLKLOCK= 0; // Clock Selection Lock Enabled bit
   FCKSM1=1; // Clock and PLL selections:
                      //    1 = locked
                      //    0 = not locked and may be modified by setting the OSWEN bit
   OSCDR= 0; // Low/high-power select is done via the SOSCSRC Configuration bit
   SOSCEN=0;// (Disabled) 32 kHz Secondary Oscillator (SOSC) Enable bit
   OSWEN =1;// Perform clock switch to NOSC<2:0> ) Oscillator Switch Enable bit
   // LOCK = ? ; // 1 -> PLL module is in lock or PLL module start-up timer is satisfied
   // CF = ?; // No clock failure has been detected
 */

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) { // USB Wake-up
    IFS0bits.INT0IF = 0;
    // USB Wake-Up  // MRF24J40 Interrupt
}

//unsigned long inline Device_FrequencySystemGet() {
//    return (SYS_CLOCK);
//}

void Device_SwitchClock(sysclock_t ck) {

#if defined(__PIC24FJ256GA702__)
    __builtin_disable_interrupts();

    device.SYS_CLOCK = 32000000UL;

    switch (ck) {
        case CK_DEFAULT: // 8Mhz - Initiate Clock Set Internal FRC no PLL
            // FNOSC2:FNOSC0: Initial Oscillator Select bits
            // POSCMOD1:POSCMOD0: Primary Oscillator Configuration bits
            CLKDIV = 0x3600; // CPDIV 1:1; PLLEN disabled; DOZE 1:8; RCDIV DCO; DOZEN disabled; ROI disabled;  
            OSCTUN = 0x00; // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; STEN disabled; TUN Center frequency; 
            REFOCONL = 0x00; // ROEN disabled; ROSWEN disabled; ROSEL FOSC; ROOUT disabled; ROSIDL disabled; ROSLP disabled; 
            REFOCONH = 0x00; // RODIV 0; 
            DCOTUN = 0x00;
            DCOCON = 0xF00; // DCOFSEL 32; DCOEN disabled; 
            OSCDIV = 0x00; // DIV 0;  
            OSCFDIV = 0x00; // TRIM 0;

            // CF no clock failure; NOSC FRCPLL; SOSCEN disabled; POSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
            __builtin_write_OSCCONH((uint8_t) (0x01));
            __builtin_write_OSCCONL((uint8_t) (0x01));
            // Wait for Clock switch to occur
            while (OSCCONbits.OSWEN != 0);
            while (OSCCONbits.LOCK != 1);
            break;


        case CK_SLOW: // 4Mhz (Fast RC Oscillator + Postscaler 2:1 )

            break;

        case CK_FAST: // 32Mhz (Fast RC Oscillator 16Mhz + PLL 1:2 )

            break;
    }
#endif
    /*
        // CLOCK Switching Routine
        //asm(" ; New oscillator selection in W0 during any clock switches. \
                     ; DISABLE GLOBAL INT \
                     ;OSCCONH (high byte) Unlock Sequence \
                     MOV #OSCCONH, w1  \
                     MOV #0x78, w2   \
                     MOV #0x9A, w3  \
                     MOV.b w2, [w1]  \
                     MOV.b w3, [w1] \
                     ;Set new oscillator selection \
                     MOV.b WREG, OSCCONH \
                     ;OSCCONL (low byte) unlock sequence   \
                     MOV #OSCCONL, w1  \
                     MOV #0x46, w2    \
                     MOV #0x57, w3   \
                     MOV.b w2, [w1]   \
                     MOV.b w3, [w1]   \
                     ;Start oscillator switch operation (OSWEN = 1)  \
                     BSET OSCCON,#0 \
                     ; ENABLE GLOBAL INT ");

        // Wait for Clock switch
        // while ( !(OSCCON & 0x02) );   // Wait for PLL to lock

        // INTCON1bits.NSTDIS = 1; // Disable nested Int
     */

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    CLKDIV = 0x3000;
    OSCTUN = 0x00;
    REFOCON = 0x00;
    __builtin_write_OSCCONH((uint8_t) (0x01));
    __builtin_write_OSCCONL((uint8_t) (0x00));

#endif
}

/*******************************************************************************
 *                                                                             * 
 *                D E V I C E   S W I T C H S Y S - R U N L E V E L            * 
 *                                                                             *
 ******************************************************************************/


void Device_SysBoot() { // Initialize system 

#if defined (__PIC24FJ256GA702__)
    // Board V2 - Power Managment
    PW_ADP_SetDigitalOutputLow(); // Off
    PW_LTC_SetDigitalOutputLow(); // On 

#endif 
    
    Device_SwitchClock(CK_DEFAULT); // Default clock 32Mhz       
    RTCC_Enable();
    DEE_Init(); // Emulated Data Eprom 


#if defined (__PIC24FJ256GA702__)

    // Setting the Output Latch SFR(s)
    LATA = 0x0000;
    LATB = 0x0001;

    // Setting the GPIO Direction SFR(s)
//    TRISA = 0x000B;
//    TRISB = 0xC7EE;

    // Setting the Weak Pull Up and Weak Pull Down SFR(s)
//    IOCPDA = 0x0000;
//    IOCPDB = 0x0080;
//    IOCPUA = 0x0000;
//    IOCPUB = 0x0300;

    //Setting the Open Drain SFR(s)
    ODCA = 0x0000;
    ODCB = 0x0000;

    // Setting the Analog/Digital Configuration SFR(s)
    ANSA = 0x0008;
    ANSB = 0x0000;

    //  Set the PPS
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS

    // _____________UART2
    UART2_RX_SetDigitalInput();
    UART2_TX_SetDigitalOutputHigh();
    RPOR0bits.RP0R = 0x0005; //RB0->UART2:U2TX
    RPINR19bits.U2RXR = 0x0001; //RB1->UART2:U2RX

    // _____________SPI1 
    RPOR5bits.RP11R = 0x0008; //RB11->SPI1:SCK1OUT
    RPOR6bits.RP13R = 0x0007; //RB13->SPI1:SDO1
    RPINR20bits.SDI1R = 0x000A; // RB10->SPI1:SDI
    MRF24_SS_SetDigitalOutputHigh(); // MRF24J40.c    
    ADA_SS_SetDigitalOutput(); // ADA2200.c
    ADA_SS_SetHigh();
    SST26_SS_SetDigitalOutputHigh(); // SST Memory Bank

    // Board V2 - Not used
    // _____________ADA Synco / Sampling timing
    //    RPINR3bits.T3CKR = 0x000F; //RB15->TMR3:T3CK (Synco/TMR3))
    //    AV_SYN_SetDigital(); // Input T3CK/RB15 (SYNCO)
    //    AV_SYN_SetDigitalInput();

    AV_IN_SetAnalogInput();

    // _____________TMR2:T2CK (Wind Speed)
    RPINR3bits.T2CKR = 0x0002; //RB2->TMR2:T2CK (WindSpeed/TMR2))
    WS_IN_SetDigitalInputLow(); // Input RB2 (6) 

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS

    // _____________INT0 (USB Wake-Up/ MRF24 DTR )
    USB_WK_SetDigitalInputLow(); // (16) INT0/RB7 

    // _____________ADC:AN3 (Measure Batt Level)
    BAT_LV_SetAnalogInput(); // (S) Batt Level ( AN1 )


    LATBbits.LATB8 = 1; //Start with bus in idle mode - both lines high
    LATBbits.LATB9 = 1;
    TRISBbits.TRISB8 = 0; //SCL1 output
    TRISBbits.TRISB9 = 0; //SDA1 output
    //    ET_IN_SetAnalog();      
    //    ET_IN_SetAnalogInput(); // ADC ( Pin 7 AN5/RP3 )

    // ___________________________________________
    //    MRF24_SS_SetDigital();
    //    MRF24_SS_SetDigitalOutput();
    //    MRF24_SS_SetHigh();
    MRF24_SS_SetDigitalOutputHigh();

    Device_SwitchADG(PW_OFF); // All off

#ifdef __VAMP1K_TEST            
    PMD1 = 00; // 0b1111111110111111; // Uart2 enabled
#else
    PMD1 = 0xFF; // All disabled
#endif
    PMD2 = 0xFF; // IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; 
    PMD3 = 0b1111110111111111; // RTCC
    PMD4 = 0xFF; // CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
    PMD5 = 0xFF; // CCP2MD enabled; CCP1MD enabled; CCP4MD enabled; CCP3MD enabled; CCP5MD enabled; 
    PMD6 = 0xFF; // SPI3MD enabled; 
    PMD7 = 0xFF; // DMA1MD enabled; DMA0MD disabled; 
    PMD8 = 0xFF; // CLC1MD enabled; CLC2MD enabled; 
#endif

    __builtin_enable_interrupts();

} // Device_SysBoot()

void Device_SysDefault() {

#if defined(__PIC24FJ256GA702__)

#ifdef __VAMP1K_TEST            
    PMD1 = 00; // 0b1111111110111111; // Uart2 enabled            
    Device_SwitchADG(0xFF); // External Circuits all Off
#else
    PMD1 = 0xFF; // ADC, I2C, SPI, USART, TMR1, TMR2,TMR3
    Device_SwitchADG(PW_OFF); // External Circuits all Off
#endif
    PMD2 = 0xFF;
    PMD3 = 0b1111110111111111; // All Disabled except RTCC
    PMD4 = 0xFF;
    PMD5 = 0xFF;
    PMD6 = 0xFF;
    PMD7 = 0xFF;
    PMD8 = 0xFF;
#endif
}

void Device_SysExchange() {
    // Deselect unused SPI's devices 
    ADA_SS_SetHigh();
    SST26_SS_SetHigh();

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))      
    //ADG729_Switch(_bs8(PW_MRF)); // RF Circuits On
    PMD3bits.RTCCMD = 0; // RTCC
    PMD1bits.I2C1MD = 0;
    PMD1bits.T1MD = 0;
    PMD1bits.SPI1MD = 0; // Spi1 On
    PMD1bits.U2MD = 0; // Uart2 On

#elif defined(__PIC24FJ256GA702__)

    if (!Device_IsWiredLinked()) {
        // Wireless
        PMD1bits.T1MD = 0; // Tmr1 On 
        PMD1bits.SPI1MD = 0; // Spi1 On
        Device_SwitchADG(PW_MRF); //Power switch       
    } else {
        // Wired
        PMD1bits.T1MD = 0; // Tmr1 On 
        PMD1bits.U2MD = 0; // Uart2 On
    }
#endif
}

void Device_SysSleep() {

    Device_SwitchADG(PW_OFF);

#if defined(__PIC24FJ256GA702__)
    PMD1 = 0xFF;
    PMD2 = 0xFF; // IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; 
    PMD3 = 0b1111110111111111; // RTCC
    PMD4 = 0xFF; // CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
    PMD5 = 0xFF; // CCP2MD enabled; CCP1MD enabled; CCP4MD enabled; CCP3MD enabled; CCP5MD enabled; 
    PMD6 = 0xFF; // SPI3MD enabled; 
    PMD7 = 0xFF; // DMA1MD enabled; DMA0MD disabled; 
    PMD8 = 0xFF; // CLC1MD enabled; CLC2MD enabled; 
    //
    //
    // RETEN VREGS MODE
    // -----+-----+----------------------
    //   0     1   Fast Wake-up   (4)
    //   1     0   Retention Sleep (1)
    //   0     0   Sleep           (3)
    //   1     1   Fast Retention (2)
    // -----+-----+-------------------------
    //
    //RCON = 0x0; 
    //RCONbits.SBOREN = 0; // Disable BoR
    RCONbits.RETEN = 1;
    RCONbits.VREGS = 0; // 

    //////////    // __builtin_disable_interrupts();
    //////////
    //////////
    //////////    // Device_Power_Save();
    //////////
    ////////////    Sleep(); // enter in sleep mode
    ////////////    Nop();
    //////////    //     PW_SWC_SetLow(); 
    //////////    //     
    //////////    //     int i;
    //////////    //     for(i=0;i<2000;i++) {
    //////////    //         Nop(); 
    //////////    //     }
    //////////    //     
    //////////    //     PW_ADP_SetLow();
    //////////    //  Device_Power_Default();

#endif
}

runlevel_t Device_SwitchSys(runlevel_t lv) {
    static runlevel_t llv;
    switch (lv) {

            /**********************************
             * B O O T                        *
             **********************************/
        case SYS_BOOT: // Set DOZE MODE !!!
            Device_SysBoot();
            Device_SysDefault();
            break;

            /**********************************
             * D E F A U L T                  *
             **********************************/
        case SYS_DEFAULT: // RTCC,I2C1,TMR1
            Device_SysDefault();
            break;

            /**********************************
             * E X C H A N G E                *
             **********************************/
        case SYS_EXCHANGE: // I2C1,TMR1,SPI1,UART2
            Device_SysExchange();
            break;


            /**********************************
             * S L E E P                      *
             **********************************/
        case SYS_SLEEP:
            Device_SysSleep();
            break;

            /**********************************
             * D E E P   S L E E P            *
             **********************************/
        case SYS_DSLEEP:

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
            asm(";Put the device into Deep Sleep mode \
                 BSET DSCON DSEN;  \
                 BSET DSCON DSEN;  \
                 PWRSAV#SLEEP_MODE; ");

#elif defined(__PIC24FJ256GA702__) 

            //The low-voltage retention regulator is only available when Sleep mode is invoked. 
            //It is controlled by the LPCFG Configuration bit (FPOR<2>) and in firmware by 
            //the RETEN bit (RCON<12>). 
            //LPCFG must be programmed (= 0) and the RETEN bit must be set (= 1) for the 
            //regulator to be enabled.

            //#define Sleep()  __builtin_pwrsav(0)
            //#define Idle()   __builtin_pwrsav(1)

            //            RCON 
            //            DSWAKE = 0;       
            //CLKDIV 
            //PMDx
            //DSCON
            //DSWAKE
            //DSGPR0 
            //DSGPR1
            //            //RCONbits.RETEN
            //            DSWAKEbits
            // Enable USB Wake-Up
            // Enable RTC Alarm Wake-Up

            PMD1 = 0xFF;
            PMD2 = 0xFF; // IC3MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; 
            PMD3 = 0b1111110111111111; // RTCC
            PMD4 = 0xFF; // CTMUMD enabled; REFOMD enabled; LVDMD enabled; 
            PMD5 = 0xFF; // CCP2MD enabled; CCP1MD enabled; CCP4MD enabled; CCP3MD enabled; CCP5MD enabled; 
            PMD6 = 0xFF; // SPI3MD enabled; 
            PMD7 = 0xFF; // DMA1MD enabled; DMA0MD disabled; 
            PMD8 = 0xFF; // CLC1MD enabled; CLC2MD enabled; 

            // set INT0 wake_up
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1; // enables INT0 (for change detection)

            Sleep(); // enter in sleep mode

            IEC0bits.INT0IE = 0; // Disable INT0 (No change detection)

#endif
            break;

        case SYS_IDLE:
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
            asm(";Put the device into Idle mode \
                 PWRSAV#IDLE_MODE; ");
#endif
            break;

        case SYS_OFF:
            break;

        case SYS_ON_CHECK: // I2C1,TMR1,ADC
            break;


            /**********************************
             * S A M P L I N G   W S T        *
             **********************************/
        case SYS_ON_SAMP_WST: // TMR1,TMR4,ADC

            Device_SwitchADG(PW_WST); // Wind & Temp Sensors Circuits On

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))      
            PMD3bits.RTCCMD = 0; // RTCC
            PMD1bits.I2C1MD = 0;
            PMD1bits.T1MD = 0;
            PMD1bits.T4MD = 0;
            PMD1bits.ADC1MD = 0;
#elif defined(__PIC24FJ256GA702__)
            PMD1bits.AD1MD = 0; // ADC On 
            PMD1bits.T1MD = 0; // Tmr1 On 
            PMD1bits.T2MD = 0; // Tmr2 On
            PMD1bits.T3MD = 0; // Tmr3 On
#endif
            break;

            /**********************************
             * S A M P L I N G   A D A        *
             **********************************/
        case SYS_ON_SAMP_ADA: // I2C1,TMR1, TMR3, SPI1, ADC

            Device_SwitchADG(PW_ADA); // LVDT circuits On

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))      
            PMD1bits.I2C1MD = 0;
            PMD1bits.T1MD = 0;
            PMD1bits.T3MD = 0;
            PMD1bits.SPI1MD = 0;
            PMD1bits.ADC1MD = 0;
#elif defined(__PIC24FJ256GA702__)
            PMD1bits.SPI1MD = 0; // SPI On
            PMD1bits.AD1MD = 0; // ADC On 
            PMD1bits.T1MD = 0; // Tmr1 On 
            PMD1bits.T2MD = 0; // Tmr2 On
            PMD1bits.T3MD = 0; // Tmr3 On
#endif
            //Device_SwitchClock(); // 32Mhz
            break;

        case SYS_ON_SAMP_SS: // ND !!!!
            break;



    }
    llv = lv;
    return (lv);
}


/*******************************************************************************
  
      PIC24FJ256GA705 - RCON:
      ---+------------------------------------
       15 TRAPR: Trap Conflict Reset 
       14 IOPUWR: Illegal Opcode Reset
        9 CM: Configuration Mismatch Reset
        7 MCLR: Master Clear Pin Reset
        6 SWR: RESET Instruction
        4 WDT: Watchdog Timer Reset
        1 BOR: Brown-out Reset
        0 POR: Power-on Reset
  
        Return:        

        WdR BoR IoR TpR SwR McR  Err
       +---+---+---+---+---+---+----+
       |7-6|5-4|3-2|1-0|7-6|5-4|3..0|
       +---+---+---+---+---+---+----+
  
  To differentiate a POR from a VBAT wake-up, check the VBAT bit (RCON2<0>).
  If the bit is set on a POR, then the device has returned from VBAT mode.
  RCONbits.SWDTEN ( Only PICs with VBAT pin !)
  
 */
#define RCON_RESET_MASK 0b1100001011010010


void Device_CheckHwReset(void) {
#if defined(__PIC24FJ256GA702__)    
    if (RCON & RCON_RESET_MASK) {
        device.sts.alarm_counter++;
    }
    RCON = RCON & RCON_RESET_MASK;    
#endif
}


