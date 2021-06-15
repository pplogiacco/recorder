
#ifndef _RTCC_H
#define _RTCC_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

    typedef struct {
        uint32_t lstamp; // 32 bits
        uint8_t sec; // 8 bits
        uint8_t min; // 6
        uint8_t hour; // 
        uint8_t day; // 5 bits
        uint8_t month; // 4 bits
        uint8_t year; // 8 bits 
    } timestamp_t;

    typedef struct {
        uint8_t sec; // 8 bits
        uint8_t min; // 6
        uint8_t hour; // 
        uint8_t day; // 5 bits
        uint8_t month; // 4 bits
        uint8_t year; // 8 bits 
    } datetime_t;

    typedef union {
        uint32_t lstamp; // 32 bits
        timestamp_t dtime;
    } ltime_t;


    void RTCC_Enable(void);
    void RTCC_SetCallBack(void (* NewCallBack)(void));
    
    //bool RTCC_Grab();
    void RTCC_GetTime(timestamp_t *t);
    void RTCC_SetTime(timestamp_t *t, unsigned char weekday);

    void RTCC_AlarmSet(timestamp_t *t);
    void RTCC_AlarmUnset();
    // Wake-up
    void RTCC_SetWakeup(uint16_t period);


    uint32_t RTCC_GetTimeL();
    uint32_t RTCC_GetTimeL2();
    uint16_t RTCC_GetMinutes();

    
    void Timestamp2Time(timestamp_t *t);
    //void Time2Timestamp(timestamp_t *t);

    //    // Timeout utils
    //    void Timeout_SetCallBack(void (* NewCallBack)(void));
    //    void Timeout_Set(uint16_t nsec, uint16_t nms);
    //    void Timeout_Unset(void);
    //    bool isTimeout(void);


#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif // _RTCC_H
