
#include "RTCC.h"

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


static rtcc_t _time;



// macro plg
#define bcd2i(v)  (((((v)& 0xF0)>>4)*10)+((v)& 0x0F))
#define i2bcd(l)  ((((l)/10)<<4)|((l)% 10))

void __attribute__((weak)) RTCC_CallBack(void) {
    // Add your custom callback code here
}

void __attribute__((interrupt, no_auto_psv)) _ISR _RTCCInterrupt(void) {
#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    IFS3bits.RTCIF = 0;
#endif   

#ifdef __PIC24FJ256GA702__

#endif

}


#ifdef __PIC24FJ256GA702__

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

void RTCC_Init(void) {

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    __builtin_write_RTCWEN();
    // RCFGCALbits.RTCWREN = 1; // RTCC Registers Write Lockout;
    // RCFGCALbits.RTCEN = 0; // Disable RTCC
    // RTCPWCbits.PWCEN = 0; // Power Control is Disabled
    // When a new value is written to these register bits, the lower half of the MINSEC register 
    // should also be written to properly reset the clock prescalers in the RTCC.
    RTCPWCbits.RTCCLK = 1; // 01 = Internal LPRC Oscillator

    // Enable RTCC, clear RTCWREN
    RCFGCALbits.RTCEN = 1;
    while (!RCFGCALbits.RTCEN); // the first time, it is recommended to ensure that it reaches to desired logic level
    RCFGCALbits.RTCWREN = 0; // Lock 
    // __builtin_write_RTCC_WRLOCK();
    
#elif defined(  __PIC24FJ256GA702__ )
    
    RTCCON1Lbits.RTCEN = 0;

    __builtin_write_RTCC_WRLOCK();

    //   if(!RTCCTimeInitialized())
    //   {
    //       // set 2021-04-17 09-06-16
    //       DATEH = 0x2104;    // Year/Month
    //       DATEL = 0x1706;    // Date/Wday
    //       TIMEH = 0x906;    // hours/minutes
    //       TIMEL = 0x1600;    // seconds
    //   }
    //   // set 2021-04-17 09-06-16
    //   ALMDATEH = 0x2104;    // Year/Month
    //   ALMDATEL = 0x1706;    // Date/Wday
    //   ALMTIMEH = 0x906;    // hours/minutes
    //   ALMTIMEL = 0x1600;    // seconds

    // AMASK Every 10 Minute; ALMRPT 1; CHIME disabled; ALRMEN enabled; 
    RTCCON1H = 0x8401;
    RTCCON1Hbits.ALRMEN = 0; // Alarm disabled
    // PWCPS 1:1; PS 1:1; CLKSEL LPRC; FDIV 0; 
    RTCCON2L = 0x01; // DIV 15499; 
    RTCCON2H = 0x3C8B; // PWCSTAB 0; PWCSAMP 0; 
    RTCCON3L = 0x00; // RTCEN enabled; OUTSEL Power Control; PWCPOE disabled; PWCEN disabled; WRLOCK disabled; PWCPOL disabled; TSAEN disabled; RTCOE disabled; 
    RTCCON1L = 0x8030; // Enable RTCC, clear RTCWREN 
    RTCCON1Lbits.RTCEN = 1;
    RTCC_Lock();

    IEC3bits.RTCIE = 1;
#endif

}

bool RTCC_Grab() {

#if (defined(__PIC24FV32KA301__) || defined(__PIC24FV32KA302__))
    if (RCFGCALbits.RTCSYNC) {
        return (false);
    }
    RCFGCALbits.RTCPTR = 3;
    _time.prt11 = RTCVAL;
    _time.prt10 = RTCVAL;
    _time.prt01 = RTCVAL;
    _time.prt00 = RTCVAL;
#endif  

#if  defined(__PIC24FJ256GA702__)
    if (RTCSTATLbits.SYNC) {
        return false;
    }

    _time.prt11 = DATEH;
    _time.prt10 = DATEL;
    _time.prt01 = TIMEH;
    _time.prt00 = TIMEL;

#endif
    return (true);
}

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

bool RTCC_GetTime(timestamp_t *t) {
    bool rtcc;
    rtcc = RTCC_Grab();
    RTCC_2Timestamp(t);
    return rtcc;
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

    __builtin_write_RTCC_WRLOCK();

    RTCCON1Lbits.RTCEN = 0;

    IFS3bits.RTCIF = false;
    IEC3bits.RTCIE = 0;

    // set RTCC initial time
    DATEH = i2bcd(t->year); // YEAR
    DATEL = (i2bcd(t->month) << 8) | i2bcd(t->day); // MONTH-1/DAY-1
    TIMEH = (i2bcd(weekday) << 8) | i2bcd(t->hour); // WEEKDAY/HOURS
    TIMEL = (i2bcd(t->min) << 8) | i2bcd(t->sec); // MINUTES/SECONDS           
    // Enable RTCC, clear RTCWREN         
    RTCCON1Lbits.RTCEN = 1;
    RTCC_Lock();

    //Enable RTCC interrupt
    IEC3bits.RTCIE = 1;

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

uint32_t RTCC_GetTimeL() {
    uint32_t ltime;
    RTCC_Grab();
    ltime = bcd2i(_time.prt11 & 0x00FF);
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

uint16_t RTCC_GetMinutes() {
    RTCC_Grab();
    return bcd2i((_time.prt00 & 0xFF00) >> 8);
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