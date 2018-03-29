// timer.h

#include "config.h"

#ifndef TIMER_H
#define TIMER_H
#define LOG_BASE_2(x) ((x) <= 1 ? 0 : (x) <= 2 ? 1 : (x) <= 4 ? 2 : \
                       (x) <= 8 ? 3 : (x) <= 16 ? 4 : (x) <= 32 ? 5 : \
                       (x) <= 64 ? 6 : (x) <= 128 ? 7 : (x) <= 256 ? 8 : \
                       (x) <= 512 ? 9 : (x) <= 1024 ? 10 : (x) <= 2048 ? 11 : \
                       (x) <= 4096 ? 12 : (x) <= 8192 ? 13 : \
                       (x) <= 16384 ? 14 : 15)
#define TIM_4_PSCR(x) LOG_BASE_2(((((x) * 1000) * F_MASTER) / 256) + 1)
#define TIM_4_ARR(x) (((x) * 1000) * F_MASTER / (1 << TIM_4_PSCR((x))))

//-------------- Declaration of function prototypes -----------------
void stop_tim1(void);
void start_tim2(void);
void start_tim3(void);
void stop_tim3(void);
void start_tim4(void);
void stop_tim4(void);
void delay_ms(uint16_t);

#endif
