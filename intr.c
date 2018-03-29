#include <iostm8L151k6.h>
#include "config.h"
#include "timer.h"

#pragma vector=EXTI2_vector
__interrupt void EXTI2_IRQ_handler(void) {
  EXTI_SR1_P2F = 1;     // Clear the interrupt flag
}

#pragma vector=EXTI3_vector
__interrupt void EXTI3_IRQ_handler(void) {
  EXTI_SR1_P3F = 1;     // Clear the interrupt flag
}

#pragma vector=EXTI6_vector
__interrupt void EXTI6_IRQ_handler(void) {
  PD_CR2_C26 = 0; // disable exti
  EXTI_SR1_P6F = 1;     // Clear the interrupt flag
  if (SYNC == PRESSED) {
    MANIP=OFF;
    pt_counter =0;
    start_tim3();
  }
}

#pragma vector=TIM1_OVR_UIF_vector
__interrupt void TIM1_UIF_handler(void) {
  uint16_t ccr;

  TIM1_SR1_UIF = 0; // clear interrupt flag

  switch(flag.pwm_mode)
  {
  case -2: // flash_led()
    if (TIM1_CR1_OPM==1 && TIM1_SR1_CC1IF)
    { // stop TIM1
      TIM1_SR1_CC1IF = 0; // clear
      TIM1_CCER1_CC1NE = 0;     // turn off ch1
      TIM1_IER_UIE = 0;         // disable interrupt gen.
      TIM1_EGR_UG = 0;          // stop generation of updates
      TIM1_BKR_MOE = 0;         // output disable
      CLK_PCKENR2_PCKEN21 = 0; // turn off the clock gating
    }
    break;
  default: // fade_led()
    pwm_cnt++;
    if ( pwm_cnt == PWM_PERIOD )
    {
      pwm_cnt =0;
      if ( flag.pwm_dir )
      {
        pwm_inc++;
        if (pwm_inc > 127)
        {
          if ( pwm_limit > 1 ) pwm_limit--;
          if ( flag.pwm_mode == 0 )
          {
            flag.pwm_dir = false;
            pwm_inc--;
          }
          if ( flag.pwm_mode == 1 )
          {
            pwm_inc = 0;
          }
        }
      } else
      {
        pwm_inc--;
        if ( pwm_inc == 0)
        {
          if ( pwm_limit > 1 ) pwm_limit--;
          if ( flag.pwm_mode == 0 )
          {
            flag.pwm_dir = true;
          }
          if ( flag.pwm_mode == -1 )
          {
            pwm_inc = 127;
          }
        }
      }
      ccr = table_128[pwm_inc];
      TIM1_CCR1H = HIBYTE(ccr);
      TIM1_CCR1L = LOBYTE(ccr);
    }
    if ( pwm_limit == 1 ) // stop TIM1
    {
      TIM1_SR1_CC1IF = 0; // clear
      TIM1_CCER1_CC1NE = 0;     // turn off ch1
      TIM1_IER_UIE = 0;         // disable interrupt gen.
      TIM1_EGR_UG = 0;          // stop generation of updates
      TIM1_BKR_MOE = 0;         // output disable
      CLK_PCKENR2_PCKEN21 = 0; // turn off the clock gating
    }
  }
}

#pragma vector=TIM2_OVR_UIF_vector
__interrupt void TIM2_UIF_handler(void) {
  TIM2_SR1_UIF = 0;
  if ( !flag.seance_is_running &&
      !flag.tuning_in_progress &&
        SYNC != PRESSED )
    flash_led(50, 50, 1);
}

#pragma vector=TIM3_OVR_UIF_vector
__interrupt void TIM3_UIF_handler(void) {
  TIM3_SR1_UIF = 0;
  pt_counter++;
  if ( SYNC == PRESSED ) {
    if ( pt_counter == PRESS_SHORT ) MANIP=ON; // for visual effect
    if ( pt_counter == PRESS_LONG ) flash_led(10, 50, 0);
    if ( pt_counter > PRESS_LONG * 3 ) {
          PWR_CTL = PWR_OFF;
          while(1)
            __halt();
    }
  } else {
    MANIP=OFF;
    stop_tim3();
    if (pt_counter > PRESS_SHORT) {
      press = p_long;
      if (pt_counter < PRESS_LONG) {
        press = p_short;
        flag.tuning_requested = true;
      }
    }
    stop_tim1();
    PD_CR2_C26 = 1; // enable exti
  }
}

#pragma vector=TIM4_UIF_vector
__interrupt void TIM4_UIF_handler(void) {
  TIM4_SR1_UIF = 0;
  if (global_cnt)
    global_cnt--;
}

#pragma vector=USART_R_RXNE_vector
__interrupt void USART_R_RXNE_handler(void) {
//   char tmp;

   if (USART1_SR_IDLE) {
    USART1_DR; // implicit read
    if ( rxbuf_ptr > 0 )
      flag.uart_rx_complete = true;
//    MANIP=OFF;
  } 
  if (USART1_SR_RXNE) {
//    MANIP=~MANIP;
    if ( rxbuf_ptr > RXBUF_SZ-1 ) {
      rxbuf_ptr = 0;
    }
    rxbuf[rxbuf_ptr++] = USART1_DR;
  }
}

#pragma vector=RTC_WAKEUP_vector
__interrupt void RTC_WAKEUP_handler(void) /* same handler for Alarm and Wakeup interrupts */
{
  if (RTC_ISR2_WUTF)
  {
    // Reset the Wakeup flag on function entry
    RTC_ISR2_WUTF = 0;

    if(flag.seance_is_running)
    {
      flag.seance_is_running=false;
      update_wakeup(tx_wup_sb_hi, tx_wup_sb_lo);
    } else
    {
      flag.seance_is_running=true;
      update_wakeup(tx_wup_se_hi, tx_wup_se_lo);
    }      
  }
 
  if (RTC_ISR2_ALRAF)
  {
    // Reset the Alarm flag on function entry
    RTC_ISR2_ALRAF = 0;
    flag.delay_not_expired = false;
  }
}

#pragma vector=DMA1_CH0_TC_vector
__interrupt void DMA1_CH0_TC_handler(void)
{
}

#define ADC1_AWD_vector                      0x14
#pragma vector=ADC1_AWD_vector
__interrupt void ADC1_AWD_handler(void)
{
  if ( ADC1_SR_AWD == 1 )
  {
    ultimate_vref=last_vref; // save vref for calibration
    ADC1_SR_AWD = 0;
    ADC1_CR1_AWDIE=0;
    flag.low_bat = true;
  }
}
