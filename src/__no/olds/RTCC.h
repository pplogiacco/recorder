/*
 * File:   RTC.h
 * Author: plogiacco
 *
 * Created on 29 novembre 2020, 15.18
 */


#ifndef _RTCC_H
#define _RTCC_H


#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

    
    // typedef struct tm bcdTime_t;
        
    // Union to access rtcc registers

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

    //plg

    typedef union {
        uint32_t lstamp; // 32 bits
        struct {
            uint8_t year : 6;
            uint8_t month : 4;
            uint8_t day : 5;
            uint8_t hour : 5;
            uint8_t min : 6;
            uint8_t sec : 6;
        };
    } timestamp_t;


    // macro
#define mRTCCDec2Bin(Dec) (10*(Dec>>4)+(Dec&0x0f))
#define mRTCCBin2Dec(Bin) (((Bin/10)<<4)|(Bin%10))
    // plg
#define bcd2i(v)  (((((v)& 0xF0)>>4)*10)+((v)& 0x0F))
#define i2bcd(l)  ((((l)/10)<<4)|((l)% 10))



    void RTCC_Init(void);
    bool RTCC_Grab();
    bool RTCC_TimeGet(timestamp_t *t);
   
    
    
    
    uint32_t RTCC_Time2L(rtcc_t otime);
    void RTCC_TimeLUnpack(const uint32_t ctime, rtcc_t *otime);
    
    //bool RTCC_TimeGet(struct tm *currentTime);
    void RTCC_TimeSet(struct tm *initialTime);

    bool RTCC_BCDTimeGet(bcdTime_t *currentTime);
    void RTCC_BCDTimeSet(bcdTime_t *initialTime);

    void RTCC_TimeReset(bool reset);
    void RTCC_CallBack(void);


#ifdef __cplusplus  // Provide C++ Compatibility
}
#endif

#endif // _RTCC_H

/**
 End of File
 */


/*
The RTCC with Power Control is designed to operate from any one of three clock sources:

 *  the crystal-controlled Secondary Oscillator (SOSC),
 *  the internal RC 31 kHz oscillator (LPRC)
 *  clock signal derived from AC mains power.

Each option provides users with choices of application component count and accuracy over time.
The clock source is selectable in software by using the RTCLK<1:0> bits (RTCPWC<11:10>).
 ifndef MY_RTCC_H
#define	MY_RTCC_H

#include <xc.h>
#include <stdint.h>
#include "My_button.h"

// ??????????
#define MINUTE ((epoch_t)60)
#define HOUR ((epoch_t)60*60)
#define DAY ((epoch_t)60*60*24)

// epoch?????2000?1?1?0?00????????

typedef uint32_t epoch_t;

// ??????????????????

typedef struct {

    union {
        uint8_t flags;

        struct {
            uint8_t ss : 1;
            uint8_t mm : 1;
            uint8_t hh : 1;
            uint8_t DD : 1;
            uint8_t MM : 1;
            uint8_t YY : 1;
        } flag;
    };
} edit_t;

// RTCC?????????????????????

typedef struct {
    epoch_t epoch;
    uint8_t ss;
    uint8_t mm;
    uint8_t hh;
    uint8_t EE;
    uint8_t DD;
    uint8_t MM;
    uint8_t YY;
    uint8_t colon;
    uint8_t halfsec;
    edit_t edit;
} time_t;

// ??????char?const????
extern const char weekday_3char[7][4];

// ????
extern time_t now;
// ????????????????????????????????????????????
extern uint8_t time_change_flag;
// main_init()??????
void RTCC_init(void);
 ***************************** necessary functions *******************************
// from decimal to hex
uint8_t d_to_x(uint8_t dec);
// from hex to decimal
uint8_t x_to_d(uint8_t hex);
// return month length
uint8_t month_length(uint8_t year, uint8_t month);
// quot?div???????????????????????
epoch_t get_quot_rem(epoch_t *quot, uint8_t div);
 ******************************* Transform time *************************
  RTCC????????????????
void RTCC_to_caltime(time_t *tm);
// ????????RTCC????????
void caltime_to_RTCC(time_t *tm);
// Epoch????????????????
void epoch_to_caltime(time_t *tm);
//
void caltime_to_epoch(time_t *tm);
 *
 *
 ********************************* Transform time User Functions *********************************
// ???RTCC???????
void RTCC_from_RTCC(time_t *tm);
// ??????????????????
void RTCC_from_caltime(time_t *tm);
// ??????????????
void RTCC_from_epoch(time_t *tm);
 ******************************** LCD display *********************************
// ???????????????????
void display_dec(char *str, uint8_t dec, uint8_t edit);
// 0802??????????????
void display_time_0802(time_t *tm, char *line0, char *line1);
// 1608??????????????
void display_time_1602(time_t *tm, char *line0, char *line1);
 ******************************** adjust the time ********************************
/ ?????????????
void RTCC_adjust_time_toggle(time_t *tm);
// ?????????????
void RTCC_adjust_time_cursor(time_t *tm);
// ?????????????
void RTCC_adjust_time_inc(time_t *tm);
// ?????????????
void RTCC_adjust_time_dec(time_t *tm);
// 3???????????
void RTCC_adjust_time_button(time_t *tm, button_t *mode, button_t *cnt_inc, button_t *cnt_dec);
// Time sync
void RTCC_task(void);

#endif	 */