#include <string.h>     // memset
#include "../device.h"  // Pins
#include "../utils.h"   // delay
#include "ADA2200.h"

void ADA2200_EnableSPI1() {
#if defined(__HWDEVICE)   

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // SPI1 Master Mode 1 (2Wire:SCK+SDIO,softSS)
    SPI1STATbits.SPIEN = 0; // Disable module
    IFS0bits.SPI1IF = 0; // Clear int vect flag
    IEC0bits.SPI1IE = 0; // DISABLE INT !!!
    // ____________________________________SPI Pins
    ADA_SS_SetDigitalOutput(); // RB12
    ADA_SS_SetHigh();
    _TRISB11 = 0; // Digital Out SCK1 
    _TRISB13 = 0; // Digital Out SDO1 (MOSI) 
    // _TRISB14=1; // Digital SDI1 (MISO) (In)
    // ____________________________________SPI Clock & Mode 
    SPI1CON1bits.MODE16 = 0; // 1=Communication is word-wide (16 bits)
    SPI1CON1bits.MSTEN = 1; // Master Fsck=(Fosc/2)/(Ppre*Spre)==??
    SPI1CON1bits.PPRE = 0; // Primary prescaler:(0=low,3=High)
    SPI1CON1bits.SPRE = 0; // Primary prescaler:(0=low,3=High)
    SPI1CON2bits.FRMEN = 0; // Framed SPIx support DISABLE (SS Free)
    SPI1CON1bits.CKE = 1; // MODE1: Clock Edge (from Idle to active clock state)
    SPI1CON1bits.CKP = 0; // MODE1: Clock Polarity (active is a high level)
    SPI1CON1bits.SMP = 0; // Input data is sampled at (0-middle,1-end) of data output
    SPI1CON2bits.SPIBEN = 0; // Enhanced buffer enabled (0=Legacy No Buffering)
    SPI1STATbits.SPIROV = 0; // Receive Overflow Flag (0=NO Overflow).
    SPI1STATbits.SPIEN = 1; // Enable SPI

#elif defined( __PIC24FJ256GA702__ )

//    SPI1CON1bits.SPIEN = 0; // Disable module
//    IEC0bits.SPI1IE = 0; // Disable int
//    IFS0bits.SPI1IF = 0; // Clear flag

    // ____________________________________SPI Pins
//    __builtin_write_OSCCONL(OSCCON & 0xbf); //! PPSUnLock    
//    //SPI1 (Master, no SDI)
//    RPOR6bits.RP13R = 7; // 7 SDO1 SPI1 Data Output (Pin 24/ RP13)
//    RPOR5bits.RP11R = 8; // 8 SPI1 Clock Output SCK1OUT (Pin 22/RP11)    
//    //Slave
//    //RPINR20bits.SDI1R = 10; // SPI1 Data Input SDI1 (Pin 21/RP10)
//    //RPINR20bits.SCK1R = 11; // SPI1 Clock Input SCK1IN (Pin 22/RP11)
//    //! Extra digital output as Chip Select
//    TRISBbits.TRISB12 = 0; // ADA_SS_SetDigitalOutput(); 
//    LATBbits.LATB12 = 1; // ADA_SS_SetHigh()        
//    __builtin_write_OSCCONL(OSCCON | 0x40); //! PPSLock

    // ____________________________________SPI Clock & Mode 
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI1CON1H = 0x00;
    // WLENGTH 0; 
    SPI1CON2L = 0x00;
    // SPIROV disabled; FRMERR disabled; 
    SPI1STATL = 0x00;
    // SPI1BRGL 79; 
    SPI1BRGL = 0x4F;  // Baud Rate 100Khz (32Mhz)
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x00;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x00;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x00;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x00;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; ENHBUF enabled; DISSDI enabled; 
    SPI1CON1L = 0x8031;
    
    

    // SPI1 Master, 8Bits
    SPI1CON1Lbits.MSTEN = 1; // Master Mode
    SPI1CON1Lbits.MODE = 0; // Communication is byte-wide  
    SPI1CON2L = 0x0007; // 8 Bits word lenght 
    //  Mode1
    SPI1CON1Lbits.CKE = 1; // MODE1: Clock Edge (from Idle to active clock state)
    SPI1CON1Lbits.CKP = 0; // MODE1: Clock Polarity (active is a high level)
    SPI1CON1Lbits.SMP = 1; // Input data is sampled at (0-middle,1-end) of data output
    SPI1CON1Lbits.ENHBUF = 0; // Enhanced buffer disabled (0=Legacy No Buffering)
    // 2Wire:SCK+SDIO,softSS
    SPI1CON1Lbits.DISSCK = 0; // Internal serial clock is enabled
    SPI1CON1Lbits.DISSDO = 0; // SDOx pin is controlled by the module
    SPI1CON1Lbits.DISSDI = 1; // SDIx pin is not used (3 Wire)
    SPI1CON1Hbits.MSSEN = 0; // SPIx slave select support is disabled (no SSx)
    //SPIxSTATL: SPIx STATUS REGISTER LOW ( ex SPI1STAT)
    SPI1CON1Lbits.SPIEN = 1; // Enable SPI 
    SPI1STATLbits.SPIROV = 0; // Receive Overflow Flag (0=NO Overflow).

    
    
#endif
#endif  // __HWDEVICE
}

void ADA_SetReg8(uint16_t reg, uint8_t data) {
#if defined(__HWDEVICE)       
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // mode 8bit-legacy, disabled OVERFLOW    
    ADA_SS_SetLow();
    Nop();
    __delay(1);
    while (SPI1STATbits.SPITBF == true); // SPIxTXB: MSB
    SPI1BUF = (0x00FF & (reg >> 8));
    while (SPI1STATbits.SPITBF == true); // SPIxTXB: LSB
    SPI1BUF = (0x00FF & ((uint8_t) reg));
    while (SPI1STATbits.SPITBF == true); // SPIxTXB: BYTE
    SPI1BUF = data;
    while (SPI1STATbits.SPITBF == true); // wait to complete !
    __delay(1);
    Nop();
    ADA_SS_SetHigh();
    
#elif defined( __PIC24FJ256GA702__ )
    // mode 8bit-legacy, disabled OVERFLOW    
    ADA_SS_SetLow();
    Nop();
    __delay(1);
    while (SPI1STATLbits.SPITBF == true); // SPIxTXB: MSB
    SPI1BUFL = (0x00FF & (reg >> 8));
    while (SPI1STATLbits.SPITBF == true); // SPIxTXB: LSB
    SPI1BUFL = (0x00FF & ((uint8_t) reg));
    while (SPI1STATLbits.SPITBF == true); // SPIxTXB: BYTE
    SPI1BUFL = data;
    while (SPI1STATLbits.SPITBF == true); // wait to complete !
    __delay(1);
    Nop();
    ADA_SS_SetHigh();
#endif
#endif    
}

void ADA2200_Enable() {

    // Power-on Sensor Board LINE1
    // Power-on and Enable: SPI1, ADC0 

    ADA2200_EnableSPI1(); // SPI1 Master Mode 1 (2Wire:SCK+SDIO,softSS)
    // __delay_ms(1);

    ADA_SetReg8(ADA_SPIRS, 0b01); // Reset & SPI control register (0x00)
    ADA_SetReg8(ADA_SPIRS, 0b00); // Reset to default values... TESTING ONLY !!!!
    //  __delay_ms(1);

    ADA_SetReg8(ADA_INPUT, 0x00); // Clock Source & Input Select (Default 0x00)
    // [7:2] Unused  
    // [1] INP gain ( 1 = only INP signal is sampled. Additional 6 dB of gain is applied )
    // [0] Clock source select ( 0 = generate a clock by crystal placed between XOUT and CLKIN 
    //                           1 = accept a CMOS level clock on the CLKIN pin. )

    ADA_SetReg8(ADA_SYNCO, 0b000000); // SYNCO control  (0x0029)
    // [5] SYNCO output enable
    // [4] SYNCO invert (1 = inverts the SYNCO signal)
    // [3:0] SYNCO edge select ( 16 edge locations for the SYNCO pulse )
    // Default 0x2D ( 1 = delays the phase between the RCLK output and the strobe )

    // ADA_SetReg8(ADA_CKDIV, 0b00110); // Clock Configuration
    ADA_SetReg8(ADA_CKDIV, 0b00001001); // Clock Configuration
    // [7:5] Unused
    // [4:2] CLKIN DIV  ( The division factor between fCLKIN and fSI.
    //                      000 = divide by 1.
    //                      001 = divide by 16.
    //                      010 = divide by 64.
    //                      100 = divide by 256 )
    // [1:0] RCLK DIV   ( set the division factor between fSO and fM.
    //                      00 = reserved.
    //                      01 = the frequency of RCLK is fSO/4.
    //                      10 = the frequency of RCLK is fSO/8.
    //                      11 = reserved )
    // Default  0x02D

    ADA_SetReg8(ADA_DCTRL, 0B11000); // Demod control       
    // [7] Unused
    // [6] PHASE90
    // [5] Unused
    // [4] Mixer enable (1= last sample taken while RCLK remains held while RCLK is inactive)
    // [6] RCLK select (0=sends the SDO signal on Pin 13,1=sends RCLK signal)
    // [2:0] VOCM select  ( 000 = VOCM pin to VDD/2 Low power mode,
    //                      001= external reference for VOCM,
    //                      010 = set the VOCM pin to VDD/2. Fast settling mode.
    //                      101 = set the VOCM pin to 1.2V )
    // Default 0x18

    /*
    // SET FILTER IIR
    ADA_SetReg8(ADA_FK11, IIR_K01);
    ADA_SetReg8(ADA_FK12, IIR_K02);
    ADA_SetReg8(ADA_FK13, IIR_K03);
    ADA_SetReg8(ADA_FK14, IIR_K04);
    ADA_SetReg8(ADA_FK15, IIR_K05);
    ADA_SetReg8(ADA_FK16, IIR_K06);
    ADA_SetReg8(ADA_FK17, IIR_K07);
    ADA_SetReg8(ADA_FK18, IIR_K08);
    ADA_SetReg8(ADA_FK19, IIR_K09);
    ADA_SetReg8(ADA_FK1A, IIR_K10);
    ADA_SetReg8(ADA_FK1B, IIR_K11);
    ADA_SetReg8(ADA_FK1C, IIR_K12);
    ADA_SetReg8(ADA_FK1D, IIR_K13);
    ADA_SetReg8(ADA_FK1E, IIR_K14);
    ADA_SetReg8(ADA_FK1F, IIR_K15);
    ADA_SetReg8(ADA_FK20, IIR_K16);
    ADA_SetReg8(ADA_FK21, IIR_K17);
    ADA_SetReg8(ADA_FK22, IIR_K18);
    ADA_SetReg8(ADA_FK23, IIR_K19);
    ADA_SetReg8(ADA_FK24, IIR_K20);
    ADA_SetReg8(ADA_FK25, IIR_K21);
    ADA_SetReg8(ADA_FK26, IIR_K22);
    ADA_SetReg8(ADA_FK27, IIR_K23);
     */
    
    //ADA_SetReg8(ADA_STROBE, 0b11);    // Update IIR coeff.  
    ADA_SetReg8(ADA_STROBE, 0b1); // Update IIR coeff.  SEBASTIANO !!!
    ADA_SetReg8(ADA_RESET, 1); // Core Reset & Start
    ADA_SetReg8(ADA_RESET, 0);
//    __delay_ms(1);
}

void ADA2200_Synco(uint8_t synco_trim) { // 1..14
    
    ADA_SetReg8(ADA_SYNCO, (synco_trim) ? ( (synco_trim & 0x7) | 0x20) : 0x0); // SYNCO control  (0x0029)       

//#elif  defined(__HWDEVICE_V2_702)
//    ADA_SetReg8(ADA_SYNCO, (sw) ? 0b100011 : 0b000000); // SYNCO control  (0x0029)       
//#else    
//    ADA_SetReg8(ADA_SYNCO, (sw) ? 0b100001 : 0b000000); // SYNCO control  (0x0029)
}

void ADA2200_Disable() {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    SPI1STATbits.SPIEN = 0;// Disable SPI1
    // Power Sensor Board LINE1
    // Disable & Power-off: SPI1, ADC0 
#elif defined( __PIC24FJ256GA702__ )
    SPI1CON1Lbits.SPIEN = 0; // Disable SPI1
#endif
    ADA_SS_SetLow();  // CS Low !
}

