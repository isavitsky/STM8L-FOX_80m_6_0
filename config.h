// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "rtc.h"

//-------------- Constant Macros -----------------
#define _CHRG           PA_IDR_IDR2
#define _STBY           PA_IDR_IDR3
#define MANIP           PD_ODR_ODR7
#define PWR_CTL         PD_ODR_ODR0
#define PA_CTL          PA_ODR_ODR4
#define WIFI_CTL        PC_ODR_ODR4
#define MOTOR_CTL       PD_ODR_ODR1
#define SYNC            PD_IDR_IDR6
#define PWR_GOOD        PA_IDR_IDR6
#define ON              1
#define OFF             0
#define OK              1
#define ERR             0
#define CFG_OK          true
#define CFG_ERR         false
#define PA_ENABLE       1
#define PA_DISABLE      0
#define PWR_ON          0
#define PWR_OFF         1
#define WIFI_ENABLE     1
#define WIFI_DISABLE    0
#define MOTOR_ENABLE    1
#define MOTOR_DISABLE   0
#define PRESSED         0       /* SYNC button state */
#define UART_SPEED      38400
#define F_MASTER        2       /* MCU clock in MHz */
#define SWVER           0x60    /* Software version */
#define TXTD_SEC        60      /* TX Time discretion (accounting), seconds */
#define CWSPEED_MIN     5       /* Minimum value of cw_speed in WPM */
#define CWSPEED_MAX     30      /* Maximum value of cw_speed in WPM */
#define CWS_NORMAL      8       /* CW normal speed */
#define CWS_FAST        14      /* CW fast speed */
#define SD_MIN          5       /* Minimum seance duration, s */
#define MAX_RUNTIME     720     /* Maximum foxhunt() run time, minutes */
#define PERIOD          127     /* PWM period */
#define TURN_ON_TIME    2000    /* ms */
#define PRESS_SHORT     20      /* ms */
#define PRESS_LONG      2000    /* ms */
#define PWR_BLINK_INTVL 50      /* Poweroff blink interval, ms */
#define RXBUF_SZ        267     /* UART rx buffer size */
#define DATA_SIZE_BAS   4       /* basic config: fox id, number of foxes,
                                seance duration, manipulation speed, TX freq */
#define DATA_SIZE_XPT   8       /* expert config: fox id, number of foxes,
                                seance duration, manipulation speed, TX freq,
                                correction tone, accu capacity */
#define DATA_SIZE_TIM   10      /* payload size for time setting */
#define FRQ_BASE        3500.0f /* Transmitter base frequency */
#define LOW_BAT         3.6f    /* Low battery voltage */
#define LOW_BAT_HZ      30      /* Low battery freq. shift Hz for
                                  low battery cond. indication */
#define VREF_LSB_MIN    0x60
#define VREF_LSB_MAX    0xA0
#define VREF_DEFAULT    1.224f
#define VDD_FACTORY     3.0f
#define ADC_CONV        4096.0f
#define ADC_DATA_SZ     2       /* adc_data[] size */

#define PWM_PERIOD      45

#define A1_OUT PD_ODR_ODR2
#define A2_OUT PD_ODR_ODR3
#define B1_OUT PB_ODR_ODR0
#define B2_OUT PB_ODR_ODR1
#define FULLSTEP_MINPOS 0       /* Minimum position */
#define FULLSTEP_MAXPOS 260     /* Maximum number of motor full-steps */
#define HALFSTEP_MINPOS 0       /* Minimum position */
#define HALFSTEP_MAXPOS 510     /* Maximum number of motor half-steps */
#define STEP_DELAY      1       /* Stepper delay in ms */
#define STEP_DELAY_INIT 1       /* Stepper delay in ms */
#define PREHEAT_MINTEMP 20.0    /* Lower Temperature for preheating, deg */
//#define AVG_SLOPE       1.639624253  /* Datasheet value for Temp Sensor AVG_Slope */
/*
                                    Procedure for empirical AVG_SLOPE measurement:
                                    1) Measure an ambient temperature Tamb with DS18B20
                                    2) Factory temperature is 90 degrees. (Datasheet)
                                    3) Measure get_vtemp() ADC value for ambient temperature.
                                    4) Vamb = (get_vtemp() * Vref_Factory / Dref_Measured) * 1000.0 (in millivolts)
                                    5) V90 = 1000.0 * (factory_t90_lsb + 0x0300) * VDD_FACTORY / ADC_CONV (in millivolts)
                                    6) AVG_SLOPE = dV/dT = (V90 - Vamb)/(90 - Tamb)
                                    
                                    
*/
#define AVG_SLOPE       1.71875 /* Empirical value for Temp Sensor AVG_Slope in mV/deg. */
#define SCORR_DATA_SZ   4       /* scorr_data[] size */

/* ===================== EEPROM Layout: ======================================*/
#define DATA_SIZE       12      /* EEPROM data size */
#define CFG_SIZE        ( (DATA_SIZE) + 2 ) /* Configuration report size */

#define HWV_IDX   0 /* Hardware vesrion, BCD major/minor rev., read-only */
#define SWV_IDX   1 /* Software vesrion, BCD major/minor rev., read-only */
#define FOX_IDX   2 /* Fox ID, 0..7 */
#define NRF_IDX   3 /* Number of foxes, 1..7 */
#define SDN_IDX   4 /* Seance Duration, s */
#define CWS_IDX   5 /* CW Speed (WPM) */
#define TXF_IDX   6 /* TX Freq (kHz + 3500) */
#define TXC_IDX   7 /* TX tone correction */
#define MAH_IDX   8 /* Accumulator capacity (minutes) high byte (equals to number of calibration points) */
#define MAL_IDX   9 /* Accumulator capacity (minutes) low byte */
#define TWH_IDX  10 /* Total worktime counter (hours) high byte, Read-Only */
#define TWL_IDX  11 /* Total worktime counter (hours) low byte, Read-Only */
/* config[CFG_SIZE] additional values */
#define DDH_IDX  12 /* Depth of discharge calculated in fox_setup() */
#define DDL_IDX  13 /* DoD low byte */

extern bool pwm_dir;
extern uint8_t tx_wup_sb_hi, tx_wup_sb_lo;
extern uint8_t tx_wup_se_hi, tx_wup_se_lo;
extern uint16_t pt_counter;
extern uint8_t pwm_inc, pwm_cnt, pwm_limit;
extern uint16_t global_cnt;
extern uint16_t rxbuf_ptr;
extern uint16_t total_prev_ts;
extern uint32_t global_ms;
extern uint16_t lowbat_short;
extern uint16_t last_vref;  // vref value updated regularly
extern uint16_t ultimate_vref;  // last vref value in low battery condition
extern char rxbuf[RXBUF_SZ];
extern uint16_t adc_data[ADC_DATA_SZ];

extern const uint8_t table_128[128];
extern enum press_t press;
extern struct flag_t flag;
extern rtctime_t tm;

struct flag_t {
  unsigned char calibrate               : 1; /* Accumulator capacity calibration flag */
  unsigned char seance_is_running       : 1;
  unsigned char tim2_started            : 1;
  unsigned char tim3_started            : 1;
  unsigned char tim3_dir                : 1; // CCR direction
  unsigned char tim4_started            : 1;
  unsigned char delay_not_expired       : 1; /* For startup delay and autoshutdown */
  unsigned char startup_time_set        : 1;
  unsigned char sync_done               : 1;
  unsigned char low_bat                 : 1; /* low battery flag */
  unsigned char uart_rx_complete        : 1; /* UART IDLE char received */
  unsigned char blink_mode              : 1; /* LED blink mode for indication */
  unsigned char pwm_dir                 : 1; // used for fade_led
  unsigned char syncpress_detected      : 1; // press of SYNC button detected
  unsigned char tuning_done             : 1; // PA tuning
  unsigned char tuning_requested        : 1; // PA tuning
  unsigned char tuning_in_progress      : 1; // PA tuning
  signed char pwm_mode                  : 2; // -2: flash_led(), -1, 0, 1: fade_led()
};
enum press_t {
  p_none, p_short, p_long
};

enum msg_id {
  REQ_READCFG = 1,
  REP_SENDCFG,
  CMD_WRITEBASIC,
  REP_WRITEBASIC,
  CMD_WRITEXPERT,
  REP_WRITEXPERT,
  CMD_TIMESET,
  REP_TIMESET,
  REQ_TUNING,
  REP_TUNING,   /* report i_ANT array */ 
  REP_TUNMAX,   /* report i_ANT maximum step# and ADC val */
  REQ_BLINC,    /*  request backlash increment */
  REQ_BLDEC,
  REP_BLVAL     /* backlash value report */
};

//-------------- Macros -----------------
#define UART_DIV (int)((( F_MASTER * 1000000.0f ) / UART_SPEED ) + 0.5f )
#define __wait_for_interrupt() asm("wfi")
#define __disable_interrupt() asm("sim")
#define __enable_interrupt() asm("rim")
#define __halt() asm("halt")
#define __reset() asm("dc8 $75") /* Initiate MCU RESET by illegal opcode */
#define HIBYTE(x) ((char*)(&(x)))[0]
#define LOBYTE(x) ((char*)(&(x)))[1]

void tx_off(void);
void tx_on(void);
void flash_led(float freq, uint8_t duty_cycle, uint8_t count);

#endif
