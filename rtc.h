// rtc.h
#ifndef RTC_H
#define RTC_H

#include <stdint.h>

typedef struct rtctime_t
{
  uint8_t ss;
  uint8_t mm;
  uint8_t hh;
  uint8_t dd;
  uint8_t mn;
  uint8_t yy;
} rtctime_t;

//-------------- Declaration of function prototypes -----------------
void rtc_init(void);    // Init RTC
void lock_rtc(void);
void unlock_rtc(void);
void reset_rtc(void);
uint8_t bcd2bin(uint8_t);
uint8_t bin2bcd(uint8_t);
void update_wakeup(char hi, char lo);
void disable_rtc_wakeup(void);
void read_rtc(rtctime_t *tm);

#endif
