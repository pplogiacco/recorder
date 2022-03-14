
#ifndef _RTCC_H
#define _RTCC_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

//    typedef struct {
//        uint8_t sec; // 8 bits
//        uint8_t min; // 6
//        uint8_t hour; // 
//        uint8_t day; // 5 bits
//        uint8_t month; // 4 bits
//        uint8_t year; // 8 bits 
//    } datetime_t;

    typedef struct {
        uint32_t lstamp; // 32 bits
        uint8_t sec; // 8 bits
        uint8_t min; // 6
        uint8_t hour; // 
        uint8_t day; // 5 bits
        uint8_t month; // 4 bits
        uint8_t year; // 8 bits 
    } timestamp_t;

//    typedef union {
//        uint32_t lstamp; // 32 bits
//        timestamp_t dtime;
//    } ltime_t;


    //void L2Time(timestamp_t *t); // Convert time to long 
    //void Time2Timestamp(timestamp_t *t);

    void RTCC_Enable(void);
    void RTCC_SetCallBack(void (* NewCallBack)(void));

    //bool RTCC_Grab();
    void RTCC_GetTime(timestamp_t *t);

    uint32_t RTCC_GetTimeL();
    uint16_t RTCC_GetMinutes();

    void RTCC_SetTime(timestamp_t *t, unsigned char weekday);

    void RTCC_AlarmSet(timestamp_t *t);
    void RTCC_AlarmUnset();

    // Wake-up
    void RTCC_SetWakeup(uint16_t nsec);




#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif // _RTCC_H
