
#include "RTCC.h"

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

typedef union tagRTCC {

    struct {
        unsigned char sec;
        unsigned char min;
        unsigned char hr;
        unsigned char wkd;
        unsigned char day;
        unsigned char mth;
        unsigned char yr;
    };

    struct {
        unsigned int prt00;
        unsigned int prt01;
        unsigned int prt10;
        unsigned int prt11;
    };
} rtcc_t;

#elif   defined(__PIC24FJ256GA702__)

typedef union tagRTCC {

    struct {
        uint8_t yr;
        uint8_t mth;
        uint8_t day;
        uint8_t wkd;
        uint8_t hr;
        uint8_t min;
        uint8_t sec;
    };

    struct {
        uint16_t dateh; // Year/Month
        uint16_t datel; // Day/Wday
        uint16_t timeh; // hours/minutes
        uint16_t timel; // seconds
    };
} rtcc_t;
#endif 

static rtcc_t _time;

//static void RTCC_Lock(void);
//static bool rtccTimeInitialized;
//static bool RTCCTimeInitialized(void);
//static uint8_t ConvertHexToBCD(uint8_t hexvalue);
//static uint8_t ConvertBCDToHex(uint8_t bcdvalue);

// macro plg
#define bcd2i(v)  (((((v)& 0xF0)>>4)*10)+((v)& 0x0F))
#define i2bcd(l)  ((((l)/10)<<4)|((l)% 10))

void __attribute__((weak)) RTCC_CallBack(void) {
    // Add your custom callback code here
}
#include <stdio.h>      // printf

void __attribute__((interrupt, no_auto_psv)) _ISR _RTCCInterrupt(void) {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    IFS3bits.RTCIF = 0;
#elif defined( __PIC24FJ256GA702__)

    IFS3bits.RTCIF = 0;
    printf("Allarm!!/n"); // Test

#endif
}

#if defined(  __PIC24FJ256GA702__ )
// Any attempt to write to the RTCEN bit, the RTCCON2L/H registers, 
// or the Date or Time registers, will be ignored as long as WRLOCK is ?1?.
// The Alarm, Power Control and Timestamp registers can be changed when WRLOCK is ?1?.

static void RTCC_Lock(void) {
    asm volatile("DISI #6");
    asm volatile("MOV #NVMKEY, W1");
    asm volatile("MOV #0x55, W2");
    asm volatile("MOV W2, [W1]");
    asm volatile("MOV #0xAA, W3");
    asm volatile("MOV W3, [W1]");
    asm volatile("BSET RTCCON1L, #11");
}
#endif

/* -------------------------------------------------------------------------- */
void RTCC_Enable(void) {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    __builtin_write_RTCWEN();
    // RCFGCALbits.RTCEN = 0; // Disable RTCC
    // RCFGCALbits.RTCWREN = 1; // RTCC Registers Write Lock-out;
    // RTCPWCbits.PWCEN = 0; // Power Control is Disabled
    // When a new value is written to these register bits, the lower half of the MINSEC register 
    // should also be written to properly reset the clock prescalers in the RTCC.
    RTCPWCbits.RTCCLK = 1; // 01 = Internal LPRC Oscillator
    RCFGCALbits.RTCEN = 1; // Enable RTCC, clear RTCWREN
    while (!RCFGCALbits.RTCEN); // the first time, it is recommended to ensure that it reaches to desired logic level
    RCFGCALbits.RTCWREN = 0; // Lock 

#elif defined(  __PIC24FJ256GA702__ )
    __builtin_write_RTCC_WRLOCK();
    //    if (!RTCCTimeInitialized()) {
    //        // set 2021-05-11 02-46-42
    //        DATEH = 0x2105; // Year/Month
    //        DATEL = 0x1102; // Date/Wday
    //        TIMEH = 0x246; // hours/minutes
    //        TIMEL = 0x4200; // seconds
    //    }
    RTCCON2L = 0xC011; // PWCPS 1:1; PS 1:16; CLKSEL LPRC; FDIV 24;
    RTCCON2H = 0x3C7; // DIV 967; 
    RTCCON3L = 0x00; // PWCSTAB 0; PWCSAMP 0; 
    RTCCON1H = 0x0; // Alarm Control Register

    IFS3bits.RTCIF = 0;
    IEC3bits.RTCIE = 1;
    RTCCON1L = 0x8000; // RTCEN enabled; TSAEN (Timestamp A) Enable bit;
    //  RTCCON1Lbits.RTCEN = 1; // Enable RTCC, clear RTCWREN 
    RTCC_Lock();
#endif
}

/* -------------------------------------------------------------------------- */
void RTCC_AlarmSet(timestamp_t *t) {
#if  defined(__PIC24FJ256GA702__)
    //To avoid a false alarm event, the timer and alarm values should only 
    //be changed while the alarm is disabled (ALRMEN = 0).        
    __builtin_write_RTCC_WRLOCK();
    RTCCON1Hbits.ALRMEN = 0; // Disable to modify alarm settings
    RTCSTATLbits.ALMEVT = 0x0;
    while (RTCSTATLbits.ALMSYNC); // 0 = Alarm registers may be written/modified safely
    ALMDATEH = (i2bcd(t->year) << 8) | i2bcd(t->month); // Year/Month
    ALMDATEL = (i2bcd(t->day) << 8); // | i2bcd(weekday); // Date/Wday
    ALMTIMEH = (i2bcd(t->hour) << 8) | i2bcd(t->min); // hours/minutes
    ALMTIMEL = (i2bcd(t->sec) << 8); // seconds           
    //    IFS3bits.RTCIF = 0;
    //    IEC3bits.RTCIE = 1;
    RTCCON1Hbits.ALRMEN = 1; // Enable alrm    
    RTCC_Lock();
    // RTCCON1H = 0x8000; // AMASK Every Half Second; ALMRPT 0; CHIME disabled; ALRMEN enabled; 
#endif
}

/* -------------------------------------------------------------------------- */
void RTCC_AlarmUnset() {
#if  defined(__PIC24FJ256GA702__)
    __builtin_write_RTCC_WRLOCK();
    RTCCON1H = 0x0; // ALRMEN disabled 
    RTCC_Lock();
    IEC3bits.RTCIE = 0;
#endif
}

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

bool RTCC_Grab() {


    if (RCFGCALbits.RTCSYNC) {
        return (false);
    }
    RCFGCALbits.RTCPTR = 3;
    _time.prt11 = RTCVAL; // 
    _time.prt10 = RTCVAL; //
    _time.prt01 = RTCVAL; //
    _time.prt00 = RTCVAL; // 

    return (true);
}

#endif  

#if  defined(__PIC24FJ256GA702__)

void RTCC_Grab() {
    while (RTCSTATLbits.SYNC) {
        Nop();
    }
    //    if (!RTCSTATLbits.SYNC) {
    _time.dateh = DATEH;
    _time.datel = DATEL;
    _time.timeh = TIMEH;
    _time.timel = TIMEL;
    //    }
}
#endif

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

void RTCC_2Timestamp(timestamp_t *t) {
    //t->lstamp = 0; // Pack Date: year, month, day  ( 15 bits )
    t->year = bcd2i(_time.prt11 & 0x00FF);
    t->lstamp = t->year;
    t->month = bcd2i(((_time.prt10 & 0xFF00) >> 8));
    t->lstamp <<= 4; // 12 month ( 4 bit )
    t->lstamp |= (t->month & 0xF);
    t->day = bcd2i((_time.prt10 & 0x00FF));
    t->lstamp <<= 5; // 31 days ( 5 bit )
    t->lstamp |= (t->day & 0x1F);
    t->lstamp <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    t->hour = bcd2i(_time.prt01 & 0x00FF);
    t->lstamp |= (uint32_t) (t->hour) << 12; // hour
    t->min = bcd2i(((_time.prt00 & 0xFF00) >> 8));
    t->sec = bcd2i(_time.prt00 & 0x00FF);
    t->lstamp |= (uint32_t) (t->min) << 6;
    t->lstamp |= t->sec;
}
#endif

#if  defined(__PIC24FJ256GA702__)

void RTCC_2Timestamp(timestamp_t *t) {
    // Populate timestamp_t object ! 
    //        DATEH = 0x2105; // Year/Month
    //        DATEL = 0x1102; // Date/Wday
    //        TIMEH = 0x246;  // hours/minutes
    //        TIMEL = 0x4200; // seconds
    //t->lstamp = 0; // Pack Date: year, month, day  ( 15 bits )

    t->year = bcd2i(((_time.dateh & 0xFF00) >> 8));
    t->lstamp = t->year;
    t->month = bcd2i(_time.dateh & 0x00FF);
    t->lstamp <<= 4; // 12 month ( 4 bit )
    t->lstamp |= (t->month & 0xF);
    //t->day = bcd2i((_time.datel & 0x00FF));
    t->day = bcd2i(((_time.datel & 0xFF00) >> 8));
    t->lstamp <<= 5; // 31 days ( 5 bit )
    t->lstamp |= (t->day & 0x1F);
    t->lstamp <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    t->hour = bcd2i(((_time.timeh & 0xFF00) >> 8));
    t->lstamp |= (uint32_t) (t->hour) << 12; // hour
    t->min = bcd2i(_time.timeh & 0x00FF);
    t->sec = bcd2i(((_time.timel & 0xFF00) >> 8));
    t->lstamp |= (uint32_t) (t->min) << 6;
    t->lstamp |= t->sec;
}
#endif

void RTCC_GetTime(timestamp_t *t) {
    RTCC_Grab();
    RTCC_2Timestamp(t);
}

void RTCC_SetTime(timestamp_t *t, unsigned char weekday) {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    __builtin_write_RTCWEN(); // Set the RTCWREN bit
    RCFGCALbits.RTCPTR = 3; // start the sequence
    RTCVAL = i2bcd(t->year); // YEAR
    RTCVAL = (i2bcd(t->month) << 8) | i2bcd(t->day); // MONTH-1/DAY-1
    RTCVAL = (i2bcd(weekday) << 8) | i2bcd(t->hour); // WEEKDAY/HOURS
    RTCVAL = (i2bcd(t->min) << 8) | i2bcd(t->sec); // MINUTES/SECONDS
    RCFGCALbits.RTCWREN = 0;
#endif   
#ifdef __PIC24FJ256GA702__
    //RTCCON1Lbits.RTCEN = 0;
    __builtin_write_RTCC_WRLOCK();
    DATEH = (i2bcd(t->year) << 8) | i2bcd(t->month); // Year/Month
    DATEL = (i2bcd(t->day) << 8); // | i2bcd(weekday); // Date/Wday
    TIMEH = (i2bcd(t->hour) << 8) | i2bcd(t->min); // hours/minutes
    TIMEL = (i2bcd(t->sec) << 8); // seconds     

    //    ALMDATEH = (i2bcd(t->year) << 8) | i2bcd(t->month); // Year/Month
    //    ALMDATEL = (i2bcd(t->day) << 8); // | i2bcd(weekday); // Date/Wday
    //    ALMTIMEH = (i2bcd(t->hour) << 8) | i2bcd(t->min); // hours/minutes
    //    ALMTIMEL = (i2bcd(t->sec) << 8); // seconds     
    RTCC_Lock();
#endif

}

void Timestamp2Time(timestamp_t *t) {
    uint32_t ctmp;
    // Unpack Date
    ctmp = t->lstamp >> 17; // 17 Bits
    //printf("Unpack L=%lu\n", (uint32_t) ctmp);
    t->year = (uint8_t) (ctmp >> 9);
    t->month = (uint8_t) ((ctmp >> 5) & 0xF);
    t->day = (ctmp & 0x1F);
    ctmp = t->lstamp & 0x1FFFF; // 17 Bits
    // printf("h:%lu\n", ((ctmp - (ctmp % 3600)) / 3600));
    t->hour = ctmp >> 12;
    t->min = (ctmp & 0xFC0) >> 6;
    t->sec = ctmp & 0x3F;
}

void Time2Timestamp(timestamp_t *t) {
    RTCC_2Timestamp(t);
}

/* -------------------------------------------------------------------------- */
// OPTIMIZED GET UINT32_T TIME OBJECT
/* -------------------------------------------------------------------------- */
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

uint32_t RTCC_GetTimeL() {
    uint32_t ltime;
    RTCC_Grab();
    ltime = bcd2i(_time.prt11 & 0x00FF); // year
    ltime <<= 4; // 12 month ( 4 bit )
    ltime |= (bcd2i(((_time.prt10 & 0xFF00) >> 8)) & 0xF);
    ltime <<= 5; // 31 days ( 5 bit )
    ltime |= (bcd2i((_time.prt10 & 0x00FF)) & 0x1F);
    ltime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    ltime |= (uint32_t) (bcd2i(_time.prt01 & 0x00FF)) << 12; // hour
    ltime |= (uint32_t) (bcd2i(((_time.prt00 & 0xFF00) >> 8))) << 6;
    ltime |= bcd2i(_time.prt00 & 0x00FF);
    return (ltime);
}
#endif

#if defined( __PIC24FJ256GA702__)

uint32_t RTCC_GetTimeL() {
    uint32_t ltime;
    RTCC_Grab();
    ltime = bcd2i(((_time.dateh & 0xFF00) >> 8)); // year
    ltime <<= 4; // 12 month ( 4 bit )
    ltime |= (bcd2i(_time.dateh & 0x00FF) & 0xF); // month
    ltime <<= 5; // 31 days ( 5 bit )
    ltime |= (bcd2i((_time.datel & 0xFF00) >> 8) & 0x1F); // day
    ltime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    ltime |= (uint32_t) (bcd2i((_time.timeh & 0xFF00) >> 8)) << 12; // hour
    ltime |= (uint32_t) (bcd2i(_time.timeh & 0x00FF)) << 6; // min
    ltime |= bcd2i((_time.timel & 0xFF00) >> 8); // sec
    return (ltime);
}

uint32_t RTCC_GetTimeL2() {
    uint32_t ltime;
    RTCC_Grab();
    ltime = bcd2i(_time.yr); // year
    ltime <<= 4; // 12 month ( 4 bit )
    ltime |= (uint32_t) (_time.mth & 0xF); // month
    ltime <<= 5; // 31 days ( 5 bit )
    ltime |= (uint32_t) (bcd2i(_time.day) & 0x1F); // day
    ltime <<= 17; // Pack Time: 60sec X 60min X 24h = 86400 sec ( 17 bits )
    ltime |= (uint32_t) (bcd2i(_time.hr) << 12); // hour
    ltime |= (uint32_t) (bcd2i(_time.min) << 6); // min
    ltime |= bcd2i(_time.sec); // sec
    return (ltime);
}
#endif

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))

uint16_t RTCC_GetMinutes() {
    RTCC_Grab();
    return bcd2i((_time.prt00 & 0xFF00) >> 8);
}
#endif


#if defined( __PIC24FJ256GA702__)

uint16_t RTCC_GetMinutes() {
    RTCC_Grab();
    return bcd2i(_time.timeh & 0x00FF);
}
#endif

void RTCC_TimestampAEventManualSet(void) {
    RTCSTATLbits.TSAEVT = 1;
}

bool RTCC_TimestampADataGet(struct tm *currentTime) {
    //    uint16_t register_value;
    if (!RTCSTATLbits.TSAEVT) {
        return false;
    }

    //    register_value = TSADATEH;
    //    currentTime->tm_year = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    //    currentTime->tm_mon = ConvertBCDToHex(register_value & 0x00FF);
    //
    //    register_value = TSADATEL;
    //    currentTime->tm_mday = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    //    currentTime->tm_wday = ConvertBCDToHex(register_value & 0x00FF);
    //
    //    register_value = TSATIMEH;
    //    currentTime->tm_hour = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    //    currentTime->tm_min = ConvertBCDToHex(register_value & 0x00FF);
    //
    //    register_value = TSATIMEL;
    //    currentTime->tm_sec = ConvertBCDToHex((register_value & 0xFF00) >> 8);

    RTCSTATLbits.TSAEVT = 0;

    return true;
}

void RTCC_SetWakeup(uint16_t period) {
    // Get current time
    // if ( attemptmode>0 ) {   // Wake-up to exchange
    // if ( samplingmode==0 ) {   // Wake-up to sampling
    // Power is ok ?

    // set alarm to current time + delaycycle   

}




///*******************************************************************************
//T I M E O U T  
//TMR1  
// *******************************************************************************/
//
//volatile bool _timed = false;
//
//void (*Timeout_CallBack)(void) = NULL; // Interrupt Handler Default CallBack
//
//void Timeout_SetCallBack(void (* NewCallBack)(void)) { // Call after Enable
//    Timeout_CallBack = NewCallBack;
//}
//
//void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
//    IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
//    if (Timeout_CallBack) { // Execute call-back 
//        Timeout_CallBack();
//    } else {
//        _timed = true;
//    }
//}
//
//void Timeout_Set(uint16_t nsec, uint16_t nms) {
//    if ((nsec + nms) > 0) { // set Timeout
//        _timed = false;
//
//#if defined(__PIC24FJ256GA702__)
//
//        T1CON = 0;
//        T1CONbits.TCS = 1; // clock source is selected by T1ECS
//        T1CONbits.TECS = 0b10; // uses the LPRC (31.25 kHz) as the clock source
//        TMR1 = 0; // Clear timer register 
//        if (nsec > 0) { // Scount sec and ignore ms
//            T1CONbits.TCKPS = 0b01; // Select 1:8 Prescaler   
//            PR1 = ( nsec * 0xF22); // Tick = 1s
//        } else { // count ms
//            T1CONbits.TCKPS = 0b00; // Select 1:1 (LPRC 1/31Khz * 0x1E = )
//            PR1 = (nms * 0x1E); // ( 0x1E = 125 = 1ms period )
//        }
//        IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
//        IEC0bits.T1IE = 1; // Enable interrupt
//        T1CONbits.TON = 1; // Enable Timer
//
//#else
//        T1CON = 0;
//        T1CONbits.TCS = 1; // clock source is selected by T1ECS
//        T1CONbits.T1ECS = 0b10; // uses the LPRC (31.25 kHz) as the clock source
//        TMR1 = 0; // Clear timer register 
//        if (nsec > 0) { // Scount sec and ignore ms
//            T1CONbits.TCKPS = 0b11; // Select 1:256 Prescaler (LPRC/256=125Hz)  
//            PR1 = nsec * 125; // Tick = 8ms
//        } else { // count ms
//            T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler (LPRC/64=500Hz)  (00 = 1:1)
//            PR1 = (nms >> 1); // tick = 2ms
//        }
//        IFS0bits.T1IF = 0; // Clear Tmr1 Int Flag
//        IEC0bits.T1IE = 1; // Enable interrupt
//        T1CONbits.TON = 1; // Enable Timer
//#endif        
//    } else {
//        _timed = true;
//    }
//
//}
//// startTimeout()
//
//bool isTimeout(void) { // 2ms x unit
//    if (_timed) {
//        T1CONbits.TON = 0; // Disable Timer    
//    }
//    return (_timed);
//}
//
//// stopTimeout()
//
//void Timeout_Unset(void) {
//    IEC0bits.T1IE = 0; // Disable interrupt
//    T1CON = 0; // Disable Timer   
//    Timeout_CallBack = NULL;
//}
//
//void RTCC_SetWakeup(uint16_t period) {
//    // Get current time
//    // if ( attemptmode>0 ) {   // Wake-up to exchange
//    // if ( samplingmode==0 ) {   // Wake-up to sampling
//    // Power is ok ?
//
//    // set alarm to current time + delaycycle   
//
//}
//
///*
//
// void RTCC_Initialize(void)
//{
//    // Set the RTCWREN bit
//    __builtin_write_RTCWEN();
//
//    RCFGCALbits.RTCEN = 0;
//
//    if(RCON2bits.VBAT & RCONbits.POR)
//    {
//        if(!RTCCTimeInitialized())
//        {
//            // set RTCC time 2016-12-08 09-22-03
//            RCFGCALbits.RTCPTR = 3; // start the sequence
//            RTCVAL = 0x16; // YEAR
//            RTCVAL = 0x1208; // MONTH-1/DAY-1
//            RTCVAL = 0x409; // WEEKDAY/HOURS
//            RTCVAL = 0x2203; // MINUTES/SECONDS
//        }
//    }
//
//    RCON2bits.VBAT = 0;
//    RCONbits.POR = 0;
//
//    // RTCOUT Alarm Pulse; PWSPRE disabled; RTCLK SOSC; PWCPRE disabled; PWCEN disabled; PWCPOL disabled;
//    RTCPWC = 0x0000;
//
//    // Enable RTCC, clear RTCWREN
//    RCFGCALbits.RTCEN = 1;
//    RCFGCALbits.RTCWREN = 0;
//
//    return;
//}
//
// */