#include <iostm8L151k6.h>
#include <stdbool.h>
#include "config.h"
#include "timer.h"

static uint8_t tim4_pscr = TIM_4_PSCR(1); // Prescaler for 1 ms
static uint8_t tim4_arr = (uint8_t)TIM_4_ARR(1); // ARR for 1 ms

void stop_tim1(void)
{
  TIM1_SR1_CC1IF = 0; // clear
  TIM1_IER_UIE = 0;   // disable interrupt gen.
  TIM1_EGR_UG = 0;    // stop generation of updates
  TIM1_BKR_MOE = 0;   // output disable
  CLK_PCKENR2_PCKEN21 = 0; // turn off the clock gating
}

/* TIM2 used for sleep mode indication */
void start_tim2(void) {
    /* Set for ~4s on 2 MHz clock */
    CLK_PCKENR1_PCKEN10 = 1;    // Enable clock to TIM2
    TIM2_PSCR = 7;
    TIM2_ARRH = 255;
    TIM2_ARRL = 255;
    TIM2_SR1_UIF = 0;       // Clr. interrupt flag
    TIM2_IER_UIE = 1;       // Enable interrupt gen.
    TIM2_CR1_CEN = 1;       // Counter enable
}

void stop_tim2(void) {
    TIM2_SR1_UIF = 0;   // Clr. interrupt flag
    TIM2_IER_UIE = 0;   // Disable interrupt gen.
    TIM2_CR1_CEN = 0;   // Counter disable
    CLK_PCKENR1_PCKEN10 = 0;    // Disable clock to TIM3
}

/* TIM3 used for SYNC button tracking */
void start_tim3(void) {
    /* Set for 1ms on 2 MHz clock */
    CLK_PCKENR1_PCKEN11 = 1;    // Enable clock to TIM3
    TIM3_PSCR = 4;
    TIM3_ARRH = 0;
    TIM3_ARRL = 125;
    TIM3_SR1_UIF = 0;       // Clr. interrupt flag
    TIM3_IER_UIE = 1;       // Enable interrupt gen.
    TIM3_CR1_CEN = 1;       // Counter enable
}

void stop_tim3(void) {
    TIM3_SR1_UIF = 0;   // Clr. interrupt flag
    TIM3_IER_UIE = 0;   // Disable interrupt gen.
    TIM3_CR1_CEN = 0;   // Counter disable
    CLK_PCKENR1_PCKEN11 = 0;    // Disable clock to TIM3
}

void start_tim4(void) {
  if ( ! flag.tim4_started ) {
    flag.tim4_started = true;
    CLK_PCKENR1_PCKEN12 = 1;      // Enable TIM4
    TIM4_PSCR = tim4_pscr; // Prescaler
    TIM4_ARR = tim4_arr;   // Auto-reload counter
    TIM4_CNTR = 0;                // To compensate the initialisation of the timer
    TIM4_SR1_UIF = 0;             // Clear interrupt flag
    TIM4_IER_UIE = 1;             // Enable interrupt generation
    TIM4_CR1_CEN = 1;             // Enable counter
  }
}

void stop_tim4(void) {
  flag.tim4_started = false;
  TIM4_IER_UIE = 0;             // Disable interrupt generation
  TIM4_CR1_CEN = 0;             // Disable counter
  CLK_PCKENR1_PCKEN12 = 0;      // Disable TIM4
}

void delay_ms(uint16_t n) {
  global_cnt = n;
  start_tim4();
  while (global_cnt) {
    asm("wfi"); // wait_for_interrupt()
  }
  stop_tim4();
}
