
#include <string.h>
#include <stdio.h>  // printf
#include "device.h"
#include "utils.h"  // include device.h
#include "modules/RTCC.h"
#include "modules/UART2.h"
//#include "memory/DEE/dee.h"


//#include "modules/RTCC.h"
//#if defined(__PIC24FJ256GA702__)   
//#include "memory/flash702.h"
//#else
//#include "memory/flash302.h"
//#endif
//#include "sampling/sampling.h"  // measurementCounter()

extern device_t device;

//static volatile unsigned long SYS_CLOCK;


/*___PIC24Fv32KA304__________________________________________________________________________________


 * OSCCON: OSCILLATOR CONTROL REGISTER
 * -----------------------------------
 *
 * COSC: Current Oscillator Selection bits
            111 = 8 MHz Fast RC Oscillator with Postscaler (FRCDIV)
            110 = 500 kHz Low-Power Fast RC Oscillator (FRC) with Postscaler (LPFRCDIV)
            101 = Low-Power RC Oscillator (LPRC)
            100 = Secondary Oscillator (SOSC)
            011 = Primary Oscillator with PLL module (XTPLL, HSPLL, ECPLL)
            010 = Primary Oscillator (XT, HS, EC)
            001 = 8 MHz FRC Oscillator with Postscaler and PLL module (FRCPLL)
            000 = 8 MHz FRC Oscillator (FRC)

 * NOSC: New Oscillator Selection bits(1)
            111 = 8 MHz Fast RC Oscillator with Postscaler (FRCDIV)
            110 = 500 kHz Low-Power Fast RC Oscillator (FRC) with Postscaler (LPFRCDIV)
            101 = Low-Power RC Oscillator (LPRC)
            100 = Secondary Oscillator (SOSC)
            011 = Primary Oscillator with PLL module (XTPLL, HSPLL, ECPLL)
            010 = Primary Oscillator (XT, HS, EC)
            001 = 8 MHz FRC Oscillator with Postscaler and PLL module (FRCPLL)
            000 = 8 MHz FRC Oscillator (FRC)

 * CLKLOCK: Clock Selection Lock Enabled bit
            If FSCM is enabled (FCKSM1 = 1):
                    1 = Clock and PLL selections are locked
                    0 = Clock and PLL selections are not locked and may be modified by setting the OSWEN bit
            If FSCM is disabled (FCKSM1 = 0):
            Clock and PLL selections are never locked and may be modified by setting the OSWEN bit.

 * LOCK: PLL Lock Status bit(2)
            1 = PLL module is in lock or PLL module start-up timer is satisfied
            0 = PLL module is out of lock, PLL start-up timer is running or PLL is disabled

 * CF: Clock Fail Detect bit
            1 = FSCM has detected a clock failure
            0 = No clock failure has been detected

 * SOSCDRV: Secondary Oscillator Drive Strength bit(3)
            1 = High-power SOSC circuit is selected
            0 = Low/high-power select is done via the SOSCSRC Configuration bit

 * SOSCEN: 32 kHz Secondary Oscillator (SOSC) Enable bit
            1 = Enables the secondary oscillator
            0 = Disables the secondary oscillator

 * OSWEN: Oscillator Switch Enable bit
            1 = Initiates an oscillator switch to the clock source specified by the NOSC<2:0> bits
            0 = Oscillator switch is complete


 * CLKDIV: CLOCK DIVIDER REGISTER
 * ------------------------------
 *
 * ROI: Recover on Interrupt bit
        1 = Interrupts clear the DOZEN bit, and reset the CPU and peripheral clock ratio to 1:1
        0 = Interrupts have no effect on the DOZEN bit

 * DOZE<2:0>: CPU and Peripheral Clock Ratio Select bits
        111 = 1:128
        110 = 1:64
        101 = 1:32
        100 = 1:16
        011 = 1:8
        010 = 1:4
        001 = 1:2
        000 = 1:1

 * DOZEN: Doze Enable bit(1)
        1 = DOZE<2:0> bits specify the CPU and peripheral clock ratio
        0 = CPU and peripheral clock ratio are set to 1:1

 * RCDIV: FRC Postscaler Select bits

        When OSCON.COSC = 111 (8Mhz FRC) :
            111 = 31.25 kHz (divide-by-256)
            110 = 125 kHz (divide-by-64)
            101 = 250 kHz (divide-by-32)
            100 = 500 kHz (divide-by-16)
            011 = 1 MHz (divide-by-8)
            010 = 2 MHz (divide-by-4)
            001 = 4 MHz (divide-by-2) (default)
            000 = 8 MHz (divide-by-1)

        When COSC<2:0> (OSCCON<14:12>) = 110: 500 kHz Low-Power Fast RC Oscillator (FRC) with Postscaler (LPFRCDIV)
            111 = 1.95 kHz (divide-by-256)
            110 = 7.81 kHz (divide-by-64)
            101 = 15.62 kHz (divide-by-32)
            100 = 31.25 kHz (divide-by-16)
            011 = 62.5 kHz (divide-by-8)
            010 = 125 kHz (divide-by-4)
            001 = 250 kHz (divide-by-2) (default)
            000 = 500 kHz (divide-by-1)



 * REFOCON: REFERENCE OSCILLATOR CONTROL REGISTER
 * ----------------------------------------------
 *
 * ROEN: Reference Oscillator Output Enable bit
        1 = Reference oscillator is enabled on REFO pin
        0 = Reference oscillator is disabled

 * ROSSLP: Reference Oscillator Output Stop in Sleep bit
        1 = Reference oscillator continues to run in Sleep
        0 = Reference oscillator is disabled in Sleep

 * ROSEL: Reference Oscillator Source Select bit
        1 = Primary oscillator is used as the base clock(1)
        0 = System clock is used as the base clock; base clock reflects any clock switching of the device

 * RODIV: Reference Oscillator Divisor Select bits
        1111 = Base clock value divided by 32,768
        1110 = Base clock value divided by 16,384
        1101 = Base clock value divided by 8,192
        1100 = Base clock value divided by 4,096
        1011 = Base clock value divided by 2,048
        1010 = Base clock value divided by 1,024
        1001 = Base clock value divided by 512
        1000 = Base clock value divided by 256
        0111 = Base clock value divided by 128
        0110 = Base clock value divided by 64
        0101 = Base clock value divided by 32
        0100 = Base clock value divided by 16
        0011 = Base clock value divided by 8
        0010 = Base clock value divided by 4
        0001 = Base clock value divided by 2
        0000 = Base clock value


 * OSCTUN: FRC OSCILLATOR TUNE REGISTER
 * ------------------------------------
 *
bit 5-0 TUN<5:0>: FRC Oscillator Tuning bits(1)
011111 = Maximum frequency deviation
011110
???
000001
000000 = Center frequency, oscillator is running at factory calibrated frequency
111111
???
100001
100000 = Minimum frequency deviation



 * CLOCK TEST
 * -------------
 * Since the clock chain can be complicated, verifying I got it right is one of the
 * first things I do in a new project. After making sure the part is otherwise
 * initialized and the runtime part of the clock chain is set up as I think it
 * should be, I run code like this:

    loop:
    btg LATB, #0    ;toggle RBO
    bra loop

 * Of course you pick a pin that won't hurt anything if toggled.
 * The BTG instruction takes one cycle, and BRA takes 2.
 * Each loop iteration therefore takes 3 cycles, and one whole period of RB0
 * therefore 6 cycles.Now you look at RB0 with a scope and see if you really
 * get the expected instruction clock frequency divided by 6.
 *
 *
 */


/*
 All peripheral modules (except for I/O ports) also have a second control bit that can disable their
functionality. These bits, known as the Peripheral Module Disable (PMD) bits, are generically
named ?XXXPMD? (using ?XXX? as the mnemonic version of the module?s name, as before).
These bits are located in the PMDx Special Function Registers. In contrast to the Module Enable
bits, the PMD bit must be set (= 1) to disable the module.
 *
 * Power-Saving Features Register Map
 *RCON TRAPR IOPUWR ? ? ? ? CM VREGS EXTR SWR SWDTEN WDTO SLEEP IDLE BOR POR xxxx(1)
CLKDIV ROI DOZE2 DOZE1 DOZE0 DOZEN RCDIV2 RCDIV1 RCDIV0 ? ? ? ? ? ? ? ? 0300
PMDx XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD XXMD 0000
 */


/*

void __attribute__((interrupt, no_auto_psv, weak)) _MI2C1Interrupt(void) {
    IFS1bits.MI2C1IF = 0;
}

void I2C1_Initialize(void) {
    //    i2c1_object.pTrHead = i2c1_tr_queue;
    //    i2c1_object.pTrTail = i2c1_tr_queue;
    //    i2c1_object.trStatus.s.empty = true;
    //    i2c1_object.trStatus.s.full = false;
    //    i2c1_object.i2cErrors = 0;

    // initialize the hardware

    // Baud Rate Generator Value: I2CBRG 78;
    I2C1BRG = 0x4E;
    // ACKEN disabled; STREN disabled; GCEN disabled; SMEN disabled; DISSLW enabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; IPMIEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled;
    I2C1CON = 0x8000;
    // P disabled; S disabled; I2COV disabled; IWCOL disabled;
    I2C1STAT = 0x00;


    // clear the master interrupt flag
    IFS1bits.MI2C1IF = 0;
    // enable the master interrupt
    IEC1bits.MI2C1IE = 0;
    I2C1CONbits.I2CEN = 1; // Enable module
}


#define I2C_TX     I2C1TRN	// Defines the transmit register used to send data.
#define I2C_RX     I2C1RCV	// Defines the receive register used to receive data.

#define I2C1_WRITE_COLLISION_STATUS_BIT         I2C1STATbits.IWCOL      // Defines the write collision status bit.
#define I2C1_ACKNOWLEDGE_STATUS_BIT             I2C1STATbits.ACKSTAT    // I2C ACK status bit.

#define I2C1_START_CONDITION_ENABLE_BIT         I2C1CONbits.SEN		// I2C START control bit.
#define I2C1_REPEAT_START_CONDITION_ENABLE_BIT  I2C1CONbits.RSEN	// I2C Repeated START control bit.
#define I2C1_RECEIVE_ENABLE_BIT                 I2C1CONbits.RCEN	// I2C Receive enable control bit.
#define I2C1_STOP_CONDITION_ENABLE_BIT          I2C1CONbits.PEN		// I2C STOP control bit.
#define I2C1_ACKNOWLEDGE_ENABLE_BIT             I2C1CONbits.ACKEN 	// I2C ACK start control bit.
#define I2C1_ACKNOWLEDGE_DATA_BIT               I2C1CONbits.ACKDT	// I2C ACK data control bit.

 *
    if (I2C1TRN & 0x1) { // R/W Operation ?

        if (I2C1STATbits.ACKSTAT == 0) { // Last detected status: 1=NACK, 0=ACK

            I2C1CONbits.RCEN = 1; // Receive enable control bit

            while (!) {
                // receive data
                // if more I2C1_ACKNOWLEDGE_DATA_BIT = 0;
                // if the last byte ( received 7th bit = 0)  Don't ack it
                // Flag that we will nak the data
                // I2C1_ACKNOWLEDGE_DATA_BIT = 1;
            }

            I2C1CONbits.PEN = 1; // then send a stop
            // Or restart....

        } else { // NACK

            I2C1CONbits.PEN = 1; // then send a stop
        }


    } else { // Write

        I2C1TRN = 0b11111111; // All switch on

        // Send data
    }
}

 */


/*
 The PIC24F Flash program memory array is organized into rows of 32 instructions
 or 96 bytes. RTSP allows the user to erase blocks of 1 row, 2 rows and 4 rows
 (32, 64 and 128 instructions) at a time, and to program one row at a time. 
 It is also possible to program single words.
 
 NVMCON: FLASH MEMORY CONTROL REGISTER

    bit 15 WR: Write Control bit
            1 = Initiates a Flash memory program or erase operation. The operation is self-timed and the bit is
                cleared by hardware once the operation is complete.
            0 = Program or erase operation is complete and inactive
  
    bit 14 WREN: Write Enable bit
            1 = Enables Flash program/erase operations
            0 = Inhibits Flash program/erase operations
  
    bit 13 WRERR: Write Sequence Error Flag bit
            1 = An improper program or erase sequence attempt, or termination, has occurred (bit is set automatically
                on any set attempt of the WR bit)
            0 = The program or erase operation completed normally
  
    bit 12 PGMONLY: Program Only Enable bit(4)
  
    bit 11-7 Unimplemented: Read as ?0?
  
    bit 6 ERASE: Erase/Program Enable bit
            1 = Performs the erase operation specified by NVMOP<5:0> on the next WR command
            0 = Performs the program operation specified by NVMOP<5:0> on the next WR command
    bit 5-0 NVMOP<5:0>: Programming Operation Command Byte bits(1)
            Erase Operations (when ERASE bit is ?1?):
            1010xx = Erases entire boot block (including code-protected boot block)(2)
            1001xx = Erases entire memory (including boot block, configuration block, general block)(2)
            011010 = Erases 4 rows of Flash memory(3)
            011001 = Erases 2 rows of Flash memory(3)
            011000 = Erases 1 row of Flash memory(3)
            0101xx = Erases entire configuration block (except code protection bits)
            0100xx = Erases entire data EEPROM(4)
            0011xx = Erases entire general memory block programming operations
            0001xx = Writes 1 row of Flash memory (when ERASE bit is ?0?)(3)

 * Note 1: All other combinations of NVMOP<5:0> are no operation.
        2: These values are available in ICSP? mode only. Refer to the device programming specification.
        3: The address in the Table Pointer decides which rows will be erased.
        4: This bit is used only while accessing data EEPROM.
 */


/*
 // ----------------------------------------------------------------------------
 //  bool _Device_ReadConfig()
 // 
 //  Read from the Program Space Visibility area and copy to RAM.
 // ----------------------------------------------------------------------------
bool _Device_ReadConfig(void) { // User Managed PSV
    int memptrtemp;
    memptrtemp = PSVPAG; // Save the auto_psv page
    PSVPAG = __builtin_psvpage(nvmConfigDatas); // get address
    CORCONbits.PSV = 1; // Set PSV enable
    // Read values
    memcpy(&g_dev.cnf, nvmConfigDatas, sizeof (config_t));
    PSVPAG = memptrtemp; // Restore auto_psv page
    return true; // Return True if CRC16 is validated
}

 // ----------------------------------------------------------------------------
 //  programParameters()
 //
 //  Description:  Erase block of NVM and write data to FLASH memory
 // ----------------------------------------------------------------------------
bool _Device_WriteConfig(uint8_t * config) {
    unsigned int tbloffset;

    NVMCON = 0b100000001011001; // Erase 2 Rows of nvm (64 instructions 192 Bytes)

    TBLPAG = __builtin_tblpage(nvmConfigDatas); // Load upper FLASH address
    tbloffset = __builtin_tbloffset(nvmConfigDatas); // Offset within page
    __builtin_tblwtl(tbloffset, 0xFFFF); // Dummy write to select the page
    __builtin_disi(6);
    __builtin_write_NVM();

    // write single word, param1, to FLASH
    // In this case the Page hasn't changed so don't have to load the table
    // page register again.
    tbloffset = __builtin_tbloffset(&nvmConfigDatas); // offset within page

    // __builtin_tblwtl(tbloffset, &g_dev.cnf); // load the write buffer
    NVMCON = 0b100000000000100; // (0x4004) One Row Write operation
    // (0x4003)Single Word Write operation
    // Updates latchs

    // FLASHWrite(); // low level erase/write command sequence

    // write single word, param3, to FLASH
    // tbloffset = __builtin_tbloffset(&iParameters[2]); // offset within page
    // __builtin_tblwtl(tbloffset, param3); // load the write buffer
    // NVMCON = 0x4003; // Setup NVMCON for Single Word Write operation
    // FLASHWrite(); // low level erase/write command sequence

    return true;
}
 */

/*
//    When the device is released from Reset, code execution will resume at the device's Reset vector.
// * 
// * The device has a dedicated Deep Sleep Brown-out Reset (DSBOR) and Deep Sleep Watchdog
//Timer Reset (DSWDT) for monitoring voltage and time-out events in Deep Sleep mode.
// *
//The DSBOR and DSWDT are independent of the standard BOR and WDT used with other
//power-managed modes (Run, Idle and Sleep).
//Entering Deep Sleep mode clears the DSWAKE register.
// *
//If enabled, the Real-Time Clock and Calendar (RTCC) continues to operate uninterrupted.
//When a wake-up event occurs in Deep Sleep mode (by Reset, RTCC alarm, external interrupt
//(INT0) or DSWDT), the device will exit Deep Sleep mode and rearm a Power-on Reset (POR).
// *
// */
//void Device_Hybernate() { // Go into Deep Sleep Mode
//    /*
//        If entering Deep Sleep mode,  the SFRs, RAM and program counter will be lost.
//        Be sure to store any relevant device state information in the DSGPRn registers.
//        The stored data can be used to restore the state of the device...
//         1.Switch off all modules except RTCC
//         2.Verify wake-up enabled Int
//         3.Save state ( mode ) information in the DSGPRn
//         4.Check VBAT
//     */
//}
//
//void Device_Resume() { // Called from wake-up event
//    // Check resume from:
//    // 1: Idle
//    // 2: Sleep
//    // 3: Deep Sleep
//    //  Device_SwitchClock(CK_SLOW);
//    //  Device_SwitchPower(PW_DEFAULT);
//    //  Device_SwitchPins(); // Switch Pin & Modules
//    //  Device_ReadConfig(void); // Set new power_mode....
//}
//
///*
//DSCON: DEEP SLEEP CONTROL REGISTER
//bit 15 DSEN: Deep Sleep Enable bit
//1 = Enters Deep Sleep on execution of PWRSAV #0
//0 = Enters normal Sleep on execution of PWRSAV #0
//bit 14-9 Unimplemented: Read as ?0?
//bit 8 RTCCWDIS: RTCC Wake-up Disable bit
//1 = Wake-up from Deep Sleep with RTCC disabled
//0 = Wake-up from Deep Sleep with RTCC enabled
//bit 7-3 Unimplemented: Read as ?0?
//bit 2 ULPWUDIS: ULPWU Wake-up Disable bit
//1 = Wake-up from Deep Sleep with ULPWU disabled
//0 = Wake-up from Deep Sleep with ULPWU enabled
//bit 1 DSBOR: Deep Sleep BOR Event bit(2)
//1 = The DSBOR was active and a BOR event was detected during Deep Sleep
//0 = The DSBOR was not active or was active but did not detect a BOR event during Deep Sleep
//bit 0 RELEASE: I/O Pin State Release bit
//1 = Upon waking from Deep Sleep, I/O pins maintain their previous states to Deep Sleep entry
//0 = Release I/O pins from their state previous to Deep Sleep entry, and allow their respective TRISx and
//LATx bits to control their states
//
//You can poll the IF bits for each enabled interrupt source to determine the source of
//wake-up. When waking up from Deep Sleep mode, poll the bits in the DSWAKE and RCON
//registers to determine the wake-up source.
// */
//
//void Device_Boot(void) { // call each sys boot
//}
//

void Device_SwitchADG(uint8_t reg) { // ADG729_Switch(uint8_t reg)
#if defined(__HWDEVICE) 
    uint16_t i2clpw;

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    // Power on i2c module !!!!!!!
    I2C1CON = 0x1000; // i2c_1
    I2C1BRG = 78; // Baud Rate
    IFS1bits.MI2C1IF = 0; //Clear I2C master Int flag
    I2C1CONbits.I2CEN = 1; //Enable I2C
    I2C1STAT = 0x00;

    I2C1CONbits.SEN = 1; // START condition
    while (I2C1CONbits.SEN) { //HW cleared when complete
    }
    I2C1TRN = ADG_ADDRESS; // Write address+(ADG729: 10011AAX)
    while (I2C1STATbits.TRSTAT) {//HW cleared when TX complete
    }
    I2C1TRN = reg; // Write data
    while (I2C1STATbits.TRSTAT) { //HW cleared when TX complete
    }
    I2C1CONbits.PEN = 1; // Send a stop
    while (I2C1CONbits.PEN) {//HW cleared when complete
    }
    I2C1CONbits.I2CEN = 0; // Disable module

#elif defined( __PIC24FJ256GA702__ )
    i2clpw = PMD1bits.I2C1MD;
    PMD1bits.I2C1MD = 0; // Enable I2C Module
    //______________I2C (ADG729)
    //    LATBbits.LATB8 = 1; //Start with bus in idle mode - both lines high
    //    LATBbits.LATB9 = 1;
    //    TRISBbits.TRISB8 = 0; //SCL1 output
    //    TRISBbits.TRISB9 = 0; //SDA1 output

    I2C1CONL = 0x8000;
    I2C1BRG = 0x4E; // 100Khz @ Fcy=16Mhz (Fosc=32Mhz)
    IFS1bits.MI2C1IF = 0; //Clear I2C master Int flag
    I2C1CONLbits.I2CEN = 1; //Enable I2C
    I2C1STAT = 0x00;
    I2C1CONLbits.SEN = 1; // START condition
    while (I2C1CONLbits.SEN); //HW cleared when complete
    I2C1TRN = ADG_ADDRESS; // Write address+(ADG729: 10011AAX)
    while (I2C1STATbits.TRSTAT); //HW cleared when TX complete
    I2C1TRN = reg; // Write data
    while (I2C1STATbits.TRSTAT); //HW cleared when TX complete
    I2C1CONLbits.PEN = 1; // Send a stop
    while (I2C1CONLbits.PEN); //HW cleared when complete
    I2C1CONLbits.I2CEN = 0; // Disable module
    PMD1bits.I2C1MD = i2clpw;
#endif

#endif // HWDEVICE
}

/*******************************************************************************
Device_GetPowerLevel()
 
Voltage Divider Circuit:
 Set up the ADC with +Reference set to ANx, and -Reference set to AVss.  
 Enable external Voltage Divider Circuit
 Read the ADC value, for 10-bit ADC format, compute:
 
 

      
BandGap: 
 Set up the ADC with +Reference set to AVdd, and -Reference set to AVss.
 Set up BandGap reference to be enabled for ADC.
 Set up ADC channel to read Vbg (the Bandgap Reference voltage).
 Read the ADC value, for 10-bit ADC format, compute:
    Vadc = (ADC Reading / 1024) * AVdd
    AVdd = Vbg * 1024 / (ADC Reading)
 to verify that AVdd is equal to 3.3 Volts +- 5%
 Nominal ADC reading        =    1.2    * 1024 /    3.3     = (Approximately)372
 Max acceptable ADC Reading = (1.2*1.05)* 1024 / (3.3*0.95) = (Approximately)412
 Min acceptable ADC Reading = (1.2*0.95)* 1024 / (3.3*1.05) = (Approximately)337
 *******************************************************************************/
uint16_t Device_GetBatteryLevel() {
#ifdef __PIC24FJ256GA702__
    uint8_t ad1md = PMD1bits.AD1MD; // Enable AD
    uint8_t bat;

    BAT_LV_SetAnalogInput(); // Batteries Level ( ANx )
    Device_SwitchADG(PW_RS1); // Enabler detection circuit

    PMD1bits.AD1MD = 0; // Enable ADC module

    // ____________________________________A/D Converter Setup
    AD1CON1 = 0; // No operation in Idle mode, Converter off
    AD1CON1bits.MODE12 = 0; // Resolution 10 bit (1=12)
    AD1CON1bits.ASAM = 1; // Auto-Convert ON (end sampling and start conversion)
    AD1CON2 = 0; // Inputs are not scanned
    AD1CON3 = 0;
    //AD1CON1bits.SSRC = 0b0111; // Auto-Convert mode
    AD1CON1bits.SSRC = 00; // SAMP is cleared by softwar
    AD1CON3bits.SAMC = 14; // 12 Auto-Sample Time TAD
    AD1CON3bits.EXTSAM = 0; // Extend sampling time
    AD1CON3bits.ADCS = 0x7; // ADC Clock ( 1TAD = 4 TCY -> 250 nS)
    AD1CON5 = 0; // No CTMU, No BandGap
    // AD1CON5bits.BGREQ = 1; // Band Gap Req. ( VBG=1.2V, Vdd = 3.3 Volt +/-5%)
    AD1CHS = 0; // No channels
    AD1CHSbits.CH0SA = BAT_LV_ADC_CH0SA; // S/H+ Input A (0=AN0,1=AN1)
    AD1CSSL = 0; // No Scan, ADC1MD bit in the PMD1

    // ____________________________________Acquire
    AD1CON1bits.ADON = 1; // Turn on A/D
    //while (0) {
    AD1CON1bits.SAMP = 1; // Start sampling the input
    __delay(5); // Ensure the correct sampling time has elapsed
    AD1CON1bits.SAMP = 0; // End sampling and start conversion
    while (!AD1CON1bits.DONE) {
        Nop();
    }
    bat = ADC1BUF0;
    AD1CON1bits.ADON = 0; // ADC Off    
    PMD1bits.AD1MD = ad1md;
    return (bat);
#endif
}
