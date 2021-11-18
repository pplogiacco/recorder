
#include "RTCC.h"

rtcc_t _time;
rtcc_t _time_chk;
rtcc_t _alarm;



static bool rtccTimeInitialized;

void RTCC_TimeReset(bool reset) {
    rtccTimeInitialized = reset;
}

static bool RTCCTimeInitialized(void) {
    return (rtccTimeInitialized);
}

static uint8_t ConvertHexToBCD(uint8_t hexvalue) {
    uint8_t bcdvalue;
    bcdvalue = (hexvalue / 10) << 4;
    bcdvalue = bcdvalue | (hexvalue % 10);
    return (bcdvalue);
}

static uint8_t ConvertBCDToHex(uint8_t bcdvalue) {
    uint8_t hexvalue;
    hexvalue = (((bcdvalue & 0xF0) >> 4)* 10) + (bcdvalue & 0x0F);
    return hexvalue;
}

void __attribute__((weak)) RTCC_CallBack(void) {
    // Add your custom callback code here

}

void __attribute__((interrupt, no_auto_psv)) _ISR _RTCCInterrupt(void) {
    // RTCC callback function
    // RTCC_CallBack();

    _LATB2 ^= 1;

    /* TODO : Add interrupt handling code */
    IFS3bits.RTCIF = 0;
}

void RTCC_Init(void) {
    // Enables the LP OSC for RTCC operation

    // Set the RTCWREN bit
    __builtin_write_RTCWEN();
    // RCFGCALbits.RTCWREN = 1; // RTCC Registers Write Lockout;
    RCFGCALbits.RTCEN = 0; // Disable RTCC
    // When a new value is written to these register bits,
    // the lower half of the MINSEC register should also be
    // written to properly reset the clock prescalers in the RTCC.
    RTCPWCbits.RTCCLK = 1; // 01 = Internal LPRC Oscillator
    //RTCPWCbits.PWCEN = 0; // Power Control is Disabled
    // if (!RTCCTimeInitialized()) {
    // set RTCC time 2020-11-29 15-19-10
    RCFGCALbits.RTCPTR = 3; // start the sequence
    RTCVAL = 0x2021; // YEAR
    RTCVAL = 0x1029; // MONTH-1/DAY-1
    RTCVAL = 0x0015; // WEEKDAY/HOURS
    RTCVAL = 0x1910; // MINUTES/SECONDS
    // Enable RTCC, clear RTCWREN
    RCFGCALbits.RTCEN = 1;
    while (!RCFGCALbits.RTCEN); // the first time, it is recommended to ensure that it reaches to desired logic level
    RCFGCALbits.RTCWREN = 0;
    // }


    /* set Alarm time 2020-11-29 15-19-10
    ALCFGRPTbits.ALRMEN = 0;
    ALCFGRPTbits.ALRMPTR = 2;
    ALRMVAL = 0x1129;
    ALRMVAL = 0x15;
    ALRMVAL = 0x1910;

    // ALRMPTR MIN_SEC; AMASK Every SEC; ARPT 0; CHIME disabled; ALRMEN enabled;
    ALCFGRPT = 0x8400;

    // RTCOUT Alarm Pulse; RTCCLK SOSC; PWCEN disabled; PWCPOL disabled;
    RTCPWC = 0x00;
     */
    //Enable RTCC interrupt
    // IEC3bits.RTCIE = 1;
    //IEC3bits.RTCIE = 0; // Disabled
}

void RTCCUnlock() {

    asm volatile("disi	#5");
    asm volatile("mov #0x55, w7"); // write 0x55 and 0xAA to
    asm volatile("mov w7, _NVMKEY"); //  NVMKEY to disable
    asm volatile("mov #0xAA, w8"); // 	write protection
    asm volatile("mov w8, _NVMKEY");
    asm volatile("bset _RCFGCAL, #13"); // set the RTCWREN bit
    asm volatile("nop");
    asm volatile("nop");
}

/*
void RTCCInit(void) {
    // Enables the LP OSC for RTCC operation
    asm("mov #OSCCON,W1"); // move address of OSCCON to W1
    asm("mov.b #0x02, W0"); // move 8-bit literal to W0, 16-bit.
    asm("mov.b #0x46, W2"); // unlock byte 1 for OSCCONL(low byte)
    asm("mov.b #0x57, W3"); // unlock byte 2 for OSCCONL(low byte)
    // move 8-bit of Wn to OSCCON register
    asm("mov.b W2, [W1]"); // write unlock byte 1
    asm("mov.b W3, [W1]"); // write unlock byte 2
    asm("mov.b W0, [W1]"); // enable SOSCEN

    // Unlock sequence must take place for RTCEN to be written
    RCFGCAL = 0x0000;

    RTCCUnlock();

    RCFGCALbits.RTCEN = 1; // bit15

    //RTCC pin pad conected to RTCC second clock
    //    PADCFG1bits.RTSECSEL = 1;
    RCFGCALbits.RTCOE = 1; //RTCC Output Enable bit


IFS3bits.RTCIF = 0; //clear the RTCC interrupt flag
IEC3bits.RTCIE = 1; //enable the RTCC interrupt

// TO DO: Write the time and date to RTCC as follow.
_time_chk.sec = 0x00;
_time_chk.min = 0x05;
_time_chk.hr = 0x7;
_time_chk.wkd = 0x2;
_time_chk.day = 0x24;
_time_chk.mth = 0x07;
_time_chk.yr = 0x07;
RTCCCalculateWeekDay(); // To calculate and confirm the weekday

// Set it after you change the time and date.
RTCCSet();

// Set alarm
_alarm.sec = 0x01;
_alarm.min = 0x01;
_alarm.hr = 0x01;
_alarm.wkd = 0x01;
_alarm.day = 0x01;
_alarm.mth = 0x01;
RTCCALMSet();
}

 */


void RTCCInit(void) {

    // Enables the LP OSC for RTCC operation
    asm("mov #OSCCON,W1"); // move address of OSCCON to W1
    asm("mov.b #0x02, W0"); // move 8-bit literal to W0, 16-bit.
    asm("mov.b #0x46, W2"); // unlock byte 1 for OSCCONL(low byte)
    asm("mov.b #0x57, W3"); // unlock byte 2 for OSCCONL(low byte)

    // move 8-bit of Wn to OSCCON register
    asm("mov.b W2, [W1]"); // write unlock byte 1
    asm("mov.b W3, [W1]"); // write unlock byte 2
    asm("mov.b W0, [W1]"); // enable SOSCEN

    // Unlock sequence must take place for RTCEN to be written
    RCFGCAL = 0x0000;
    RTCCUnlock();
    RCFGCALbits.RTCEN = 1; // bit15

    //RTCC pin pad conected to RTCC second clock
    //PADCFG1bits.RTSECSEL = 1;
    //RCFGCALbits.RTCOE = 1; //RTCC Output Enable bit

    /* Enable the RTCC interrupt*/
    IFS3bits.RTCIF = 0; //clear the RTCC interrupt flag
    IEC3bits.RTCIE = 1; //enable the RTCC interrupt


    // TO DO: Write the time and date to RTCC as follow.
    _time_chk.sec = 0x00;
    _time_chk.min = 0x05;
    _time_chk.hr = 0x7;
    _time_chk.wkd = 0x2;
    _time_chk.day = 0x24;
    _time_chk.mth = 0x07;
    _time_chk.yr = 0x07;

    //RTCCCalculateWeekDay(); // To calculate and confirm the weekday

    // Set it after you change the time and date.
    //RTCCSet();

    // Set alarm
    _alarm.sec = 0x01;
    _alarm.min = 0x01;
    _alarm.hr = 0x01;
    _alarm.wkd = 0x01;
    _alarm.day = 0x01;
    _alarm.mth = 0x01;
    // RTCCALMSet();
}

/*********************************************************************
 * Function: RTCCProcessEvents
 *
 * Preconditions: RTCCInit must be called before.
 *
 * Overview: The function grabs the current time from the RTCC and
 * translate it into strings.
 *
 * Input: None.
 *
 * Output: It update time and date strings  _time_str, _date_str,
 * and _time, _time_chk structures.
 *
 ********************************************************************/
void RTCCgrab(void) {
    // Process time object only if time is not being set

    // Grab the time
    RCFGCALbits.RTCPTR = 3;
    _time.prt11 = RTCVAL;
    _time.prt10 = RTCVAL;
    _time.prt01 = RTCVAL;
    _time.prt00 = RTCVAL;

    // Grab the time again
    RCFGCALbits.RTCPTR = 3;
    _time_chk.prt11 = RTCVAL;
    _time_chk.prt10 = RTCVAL;
    _time_chk.prt01 = RTCVAL;
    _time_chk.prt00 = RTCVAL;

    // Verify there is no roll-over
    if ((_time.prt00 == _time_chk.prt00) &&
            (_time.prt01 == _time_chk.prt01) &&
            (_time.prt10 == _time_chk.prt10) &&
            (_time.prt11 == _time_chk.prt11)) {
        // Here, you can watch structure _time,
        //	which has the data from RTCC registers.

        // TO DO: do something as you like here.
    }

}

bool RTCC_Grab() {
    while (RCFGCALbits.RTCSYNC == 1); //wait for RTCSYNC bit to become 0
    RCFGCALbits.RTCPTR = 3;
    _time.prt11 = RTCVAL;
    _time.prt10 = RTCVAL;
    _time.prt01 = RTCVAL;
    _time.prt00 = RTCVAL;
}

bool RTCC_TimeGet(timestamp_t *t) {
    uint16_t register_value;
    uint32_t ctmp = 0;
    uint32_t ctime;

    ctime = 0; // Pack Date: year, month, day  ( 15 bits )
    ctime = bcd2i(_time.prt11 & 0x00FF);
    ctime <<= 4; // 12 month ( 4 bit )
    ctime |= (bcd2i(((_time.prt10 & 0xFF00) >> 8)) &0xF);
    ctime <<= 5; // 31 days ( 5 bit )
    ctime |= (bcd2i((_time.prt10 & 0x00FF)) &0x1F);
    ctime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    ctime += bcd2i(_time.prt01 & 0x00FF) * (uint32_t) 3600; // hour
    ctime += (bcd2i(((_time.prt00 & 0xFF00) >> 8))*60) + bcd2i(_time.prt00 & 0x00FF);
    t->lstamp = ctime;
    return true;
}

bool RTCC_TimeGrabGet(timestamp_t *t) {
    uint16_t register_value;
    uint32_t ctmp = 0;
    uint32_t ctime = 0;


    if (RCFGCALbits.RTCSYNC) {
        return false;
    }

    RCFGCALbits.RTCPTR = 3;
    ctime = 0; // Pack Date: year, month, day  ( 15 bits )

    register_value = RTCVAL; // Year ( 6 bit )
    //printf("\n----\nY:%u\n", bcd2i(register_value & 0x00FF));
    ctime = bcd2i(register_value & 0x00FF);

    register_value = RTCVAL; // 12 month, 31 days ( 4+5 bit )
    //printf("M:%u\n", bcd2i(((register_value & 0xFF00) >> 8)));
    ctime <<= 4; // 12 month ( 4 bit )
    ctime |= (bcd2i(((register_value & 0xFF00) >> 8)) &0xF);
    //printf("D:%u\n", bcd2i(register_value & 0x00FF));
    ctime <<= 5; // 31 days ( 5 bit )
    ctime |= (bcd2i((register_value & 0x00FF)) &0x1F);

    ctime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )

    register_value = RTCVAL; //
    //printf("h:%u\n", bcd2i(register_value & 0x00FF));
    ctime += bcd2i(register_value & 0x00FF) * (uint32_t) 3600; // hour

    register_value = RTCVAL;
    //printf("m:%u\n", bcd2i(((register_value & 0xFF00) >> 8)));
    //printf("s:%u\n", bcd2i(register_value & 0x00FF));
    ctime += (bcd2i(((register_value & 0xFF00) >> 8))*60) + bcd2i(register_value & 0x00FF);

    //printf("Packed L=%lu\n", (uint32_t) ctime);
    // Unpack Date
    ctmp = ctime >> 17; // 17 Bits
    //printf("Unpack L=%lu\n", (uint32_t) ctmp);
    //printf("Y:%lu\n", ctmp >> 9);
    //printf("M:%lu\n", (ctmp >> 5)&0xF);
    //printf("D:%lu\n", (ctmp & 0x1F));

    // Unpack Time
    ctmp = ctime & 0x1FFFF; // 17 Bits
    //printf("Unpack=%lu\n", (uint32_t) ctime); // ctime 32 bits
    ctmp = ctime & 0x1FFFF; // 17 Bits
    //printf("h:%lu\n", ((ctmp - (ctmp % 3600)) / 3600));
    ctmp = ctmp % 3600;
    //printf("m:%lu\n", ((ctmp - (ctmp % 60)) / 60));
    ctmp = ctmp % 60;
    //printf("s:%lu\n", ctmp);
    t->lstamp = ctime;
    return true;
}

bool RTCC_TimeGetOLD(timestamp_t *t) {
    uint16_t register_value;
    uint32_t ctime;

    // while (RCFGCALbits.RTCSYNC == 1); //wait for RTCSYNC bit to become 0
    if (RCFGCALbits.RTCSYNC) {
        return false;
    }
    RCFGCALbits.RTCPTR = 3;
    ctime = 0; // Pack Date: year, month, day  ( 15 bits )
    register_value = RTCVAL; // Year ( 6 bit )
    ctime = bcd2i(register_value & 0x00FF);
    register_value = RTCVAL; // 12 month, 31 days ( 4+5 bit )
    ctime <<= 4; // 12 month ( 4 bit )
    ctime |= (bcd2i(((register_value & 0xFF00) >> 8)) &0xF);
    ctime <<= 5; // 31 days ( 5 bit )
    ctime |= (bcd2i((register_value & 0x00FF)) &0x1F);
    ctime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    register_value = RTCVAL; //
    ctime += bcd2i(register_value & 0x00FF) * (uint32_t) 3600; // hour
    register_value = RTCVAL;
    ctime += (bcd2i(((register_value & 0xFF00) >> 8))*60) + bcd2i(register_value & 0x00FF);
    //    *t = ctime;
    //    printf("L:%ul\n", ctime);
    return true;
}

uint32_t RTCC_Time2L(rtcc_t otime) {
    uint32_t ctime = 0;
    ctime = 0; // Pack Date: year, month, day  ( 15 bits )
    ctime = bcd2i(otime.prt00 & 0x00FF);
    ctime <<= 4; // 12 month ( 4 bit )
    ctime |= (bcd2i(((otime.prt01 & 0xFF00) >> 8)) &0xF);
    ctime <<= 5; // 31 days ( 5 bit )
    ctime |= (bcd2i((otime.prt01 & 0x00FF)) &0x1F);
    ctime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    ctime += bcd2i(otime.prt10 & 0x00FF) * (uint32_t) 3600; // hour
    ctime += (bcd2i(((otime.prt11 & 0xFF00) >> 8))*60) + bcd2i(otime.prt11 & 0x00FF);
    return ctime;
}

void RTCC_TimeLUnpack(const uint32_t ctime, rtcc_t *otime) {
    uint32_t ctmp = 0;
    ctmp = ctime >> 17; // Unpack Date 15 Bits
    otime->yr = (unsigned char) (ctmp >> 9);
    otime->mth = (unsigned char) ((ctmp >> 5)&0xF);
    otime->day = (unsigned char) (ctmp & 0x1F);
    ctmp = ctime & 0x1FFFF; // Unpack Time 17 Bits
    otime->hr = (unsigned char) ((ctmp - (ctmp % 3600)) / 3600);
    ctmp = ctmp % 3600;
    otime->min = (unsigned char) ((ctmp - (ctmp % 60)) / 60);
    ctmp = ctmp % 60;
    otime->sec = (unsigned char) ctmp;
}

bool RTCC_TimeGetX(struct tm *currentTime) {
    uint16_t register_value;

    while (RCFGCALbits.RTCSYNC == 1); //wait for RTCSYNC bit to become 0

    if (RCFGCALbits.RTCSYNC) {
        return false;
    }
    RCFGCALbits.RTCPTR = 3;
    register_value = RTCVAL;
    currentTime->tm_year = ConvertBCDToHex(register_value & 0x00FF);
    register_value = RTCVAL;
    currentTime->tm_mon = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_mday = ConvertBCDToHex(register_value & 0x00FF);
    register_value = RTCVAL;
    currentTime->tm_wday = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_hour = ConvertBCDToHex(register_value & 0x00FF);
    register_value = RTCVAL;
    currentTime->tm_min = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_sec = ConvertBCDToHex(register_value & 0x00FF);

    return true;
}

/**
 * This function sets the RTCC value and takes the input time in decimal format
 */

void RTCC_TimeSet(struct tm *initialTime) {
    // Set the RTCWREN bit

    __builtin_write_RTCWEN();

    RCFGCALbits.RTCEN = 0;

    IFS3bits.RTCIF = false;
    IEC3bits.RTCIE = 0;

    // set RTCC initial time
    RCFGCALbits.RTCPTR = 3; // start the sequence
    RTCVAL = ConvertHexToBCD(initialTime->tm_year); // YEAR
    RTCVAL = (ConvertHexToBCD(initialTime->tm_mon) << 8) | ConvertHexToBCD(initialTime->tm_mday); // MONTH-1/DAY-1
    RTCVAL = (ConvertHexToBCD(initialTime->tm_wday) << 8) | ConvertHexToBCD(initialTime->tm_hour); // WEEKDAY/HOURS
    RTCVAL = (ConvertHexToBCD(initialTime->tm_min) << 8) | ConvertHexToBCD(initialTime->tm_sec); // MINUTES/SECONDS

    // Enable RTCC, clear RTCWREN
    RCFGCALbits.RTCEN = 1;
    RCFGCALbits.RTCWREN = 0;

    IEC3bits.RTCIE = 1;

}

/**
 This function reads the RTCC time and returns the date and time in BCD format
 */
bool RTCC_BCDTimeGet(bcdTime_t *currentTime) {
    uint16_t register_value;
    if (RCFGCALbits.RTCSYNC) {
        return false;
    }

    RCFGCALbits.RTCPTR = 3;
    register_value = RTCVAL;
    currentTime->tm_year = register_value;
    register_value = RTCVAL;
    currentTime->tm_mon = (register_value & 0xFF00) >> 8;
    currentTime->tm_mday = register_value & 0x00FF;
    register_value = RTCVAL;
    currentTime->tm_wday = (register_value & 0xFF00) >> 8;
    currentTime->tm_hour = register_value & 0x00FF;
    register_value = RTCVAL;
    currentTime->tm_min = (register_value & 0xFF00) >> 8;
    currentTime->tm_sec = register_value & 0x00FF;

    return true;
}

/**
 This function takes the input date and time in BCD format and sets the RTCC
 */
void RTCC_BCDTimeSet(bcdTime_t *initialTime) {
    // Set the RTCWREN bit

    __builtin_write_RTCWEN();

    RCFGCALbits.RTCEN = 0;

    IFS3bits.RTCIF = false;
    IEC3bits.RTCIE = 0;

    // set RTCC initial time
    RCFGCALbits.RTCPTR = 3; // start the sequence
    RTCVAL = initialTime->tm_year; // YEAR
    RTCVAL = (initialTime->tm_mon << 8) | initialTime->tm_mday; // MONTH-1/DAY-1
    RTCVAL = (initialTime->tm_wday << 8) | initialTime->tm_hour; // WEEKDAY/HOURS
    RTCVAL = (initialTime->tm_min << 8) | initialTime->tm_sec; // MINUTES/SECONDS

    // Enable RTCC, clear RTCWREN
    RCFGCALbits.RTCEN = 1;
    RCFGCALbits.RTCWREN = 0;

    IEC3bits.RTCIE = 1;
}










/*******************************************************************************
uint32_t RTCC_TimeGetL(uint32_t *t) {
    uint16_t register_value;
    uint32_t ctmp = 0;
    uint32_t ctime = 0;

    while (RCFGCALbits.RTCSYNC == 1); //wait for RTCSYNC bit to become 0
    if (RCFGCALbits.RTCSYNC) {
        return false;
    }

    RCFGCALbits.RTCPTR = 3;
    ctime = 0; // Pack Date: year, month, day  ( 15 bits )

    register_value = RTCVAL; // Year ( 6 bit )
    //printf("\n----\nY:%u\n", bcd2i(register_value & 0x00FF));
    ctime = bcd2i(register_value & 0x00FF);

    register_value = RTCVAL; // 12 month, 31 days ( 4+5 bit )
    //printf("M:%u\n", bcd2i(((register_value & 0xFF00) >> 8)));
    ctime <<= 4; // 12 month ( 4 bit )
    ctime |= (bcd2i(((register_value & 0xFF00) >> 8)) &0xF);
    //printf("D:%u\n", bcd2i(register_value & 0x00FF));
    ctime <<= 5; // 31 days ( 5 bit )
    ctime |= (bcd2i((register_value & 0x00FF)) &0x1F);

    ctime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )

    register_value = RTCVAL; //
    //printf("h:%u\n", bcd2i(register_value & 0x00FF));
    ctime += bcd2i(register_value & 0x00FF) * (uint32_t) 3600; // hour

    register_value = RTCVAL;
    //printf("m:%u\n", bcd2i(((register_value & 0xFF00) >> 8)));
    //printf("s:%u\n", bcd2i(register_value & 0x00FF));
    ctime += (bcd2i(((register_value & 0xFF00) >> 8))*60) + bcd2i(register_value & 0x00FF);

    printf("Packed L=%lu\n", (uint32_t) ctime);
    // Unpack Date
    ctmp = ctime >> 17; // 17 Bits
    printf("Unpack L=%lu\n", (uint32_t) ctmp);
    printf("Y:%lu\n", ctmp >> 9);
    printf("M:%lu\n", (ctmp >> 5)&0xF);
    printf("D:%lu\n", (ctmp & 0x1F));

    // Unpack Time
    ctmp = ctime & 0x1FFFF; // 17 Bits
    printf("Unpack=%lu\n", (uint32_t) ctime); // ctime 32 bits
    ctmp = ctime & 0x1FFFF; // 17 Bits
    printf("h:%lu\n", ((ctmp - (ctmp % 3600)) / 3600));
    ctmp = ctmp % 3600;
    printf("m:%lu\n", ((ctmp - (ctmp % 60)) / 60));
    ctmp = ctmp % 60;
    printf("s:%lu\n", ctmp);

    return ctime;
}
 *******************************************************************************/
