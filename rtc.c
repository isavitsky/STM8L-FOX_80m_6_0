#include <iostm8L151k6.h>
#include "rtc.h"
#include "config.h"

void rtc_init() {
  CLK_PCKENR2_PCKEN22 = 1;      // Enable clock to RTC
  while ( CLK_CRTCR_RTCSWBSY ); // Wait for LSE to stabilize
  CLK_CRTCR_RTCSEL3 = 1;        // Select LSE for RTC
  // Initialize the RTC
  unlock_rtc();
  RTC_ISR1_INIT = 1;    // Enter RTC Init mode
  while (RTC_ISR1_INITF != 1);  // Wait for RTC Init allowed
  RTC_APRER = 0x7F;     // Async prescaler to 128
  RTC_SPRERH = 0;       // Sync prescaler to 256;
  RTC_SPRERL = 0xFF;
  RTC_ISR1_INIT = 0;    // Exit RTC Init mode
  RTC_CR2_ALRAE = 0;    // Disable RTC Alarm
  RTC_ISR2_ALRAF = 0;   // Reset Alarm flag
  RTC_CR2_ALRAIE = 0;   // Disable RTC Alarm interrupt
  RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
  RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
  RTC_CR2_WUTIE = 0;    // Disable RTC Wakeup Interrupt
  while (RTC_ISR1_WUTWF != 1);  // Wait for Wakeup timer update allowed
  RTC_CR1_WUCKSEL = 4;  // WU clock set to 1 Hz
  RTC_WUTRH = 0;        // Set counter
  RTC_WUTRL = 0;
  lock_rtc();
}

void unlock_rtc() {
    RTC_WPR = 0xCA;       // Open the RTC write protection register
    RTC_WPR = 0x53;
}

void lock_rtc() {
  RTC_WPR = 0xFF;     // Close the RTC write protection register
}

void read_rtc(rtctime_t *tm)
{
  tm->ss = RTC_TR1;     // Read seconds
  tm->mm = RTC_TR2;     // Read minutes
  tm->hh = RTC_TR3;     // Read hours
  tm->dd = RTC_DR1;     // Read days
  tm->mn = RTC_DR2;     // Read months
  tm->yy = RTC_DR3;     // Read years
}

void reset_rtc(void) {
  __disable_interrupt();
  // Initialize the RTC
  unlock_rtc();
  RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
  RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
  RTC_CR2_WUTIE = 0;    // Disable RTC Wakeup Interrupt
  RTC_WUTRH = 0;        // Set counter
  RTC_WUTRL = 0;
  if (!flag.startup_time_set)
  {
    RTC_CR2_ALRAE = 0;    // Disable RTC Alarm
    RTC_ISR2_ALRAF = 0;   // Reset Alarm flag
    RTC_CR2_ALRAIE = 0;   // Disable RTC Alarm interrupt
    RTC_ISR1_INIT = 1;    // Enter RTC Init mode
    while (RTC_ISR1_INITF != 1);  // Wait for RTC Init allowed
    RTC_TR1 = 1;          // seconds
    RTC_TR2 = 2;          // minutes
    RTC_TR3 = 0;          // hours
    RTC_DR1 = 1;          // 1st day
    RTC_DR2 = 0x21;       // January, Monday
    RTC_DR3 = 0x7;        // 2007 year
    RTC_ISR1_INIT = 0;    // Exit RTC Init mode
    flag.delay_not_expired = false;
  } else {
    flag.delay_not_expired = true;
  }
  lock_rtc();
  
  read_rtc(&tm);

  total_prev_ts = bcd2bin(tm.hh) * 60;
  total_prev_ts += tm.mm = bcd2bin(tm.mm);

  __enable_interrupt();
}

uint8_t bcd2bin(uint8_t x)
{
  return (x >> 4) * 10 + (x & 0x0F);
}

uint8_t bin2bcd(uint8_t x)
{
  uint8_t tmp;

  tmp = x / 10;
  x %= 10;
  x |= (tmp << 4);
  return x;
}

void update_wakeup(char hi, char lo)
{
  __disable_interrupt();
  // Setup the RTC Wakeup delay
  unlock_rtc();
  RTC_CR2_WUTE = 0;     // Disable RTC Wakeup (required)
  while (RTC_ISR1_WUTWF != 1);  // Wait for Wakeup timer update allowed
  RTC_WUTRH = hi;        // Set counter
  RTC_WUTRL = lo;
  RTC_CR2_WUTE = 1;     // Enable RTC Wakeup
  RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
  RTC_CR2_WUTIE = 1;    // Enable RTC Wakeup Interrupt
  lock_rtc();
  __enable_interrupt();
}

void disable_rtc_wakeup(void)
{
   __disable_interrupt();
  unlock_rtc();
  RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
  lock_rtc();
  __enable_interrupt();
}
