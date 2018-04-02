// STM8L ARDF fox controller
// Ideas inspired by VE2EMM, PE1GRL, PE5EDW and DF1FO
// Version 6.0: September 2017 --- March 2018
//      * based on v5.x


#include <iostm8L151k6.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "timer.h"
#include "intr.h"
#include "uart.h"
#include "min.h"
#include "rtc.h"
#include "si5351a.h"
#include "adc.h"
#include "dma.h"

//-------------- Type Definitions -----------------

//-------------- Global variables --------------
uint16_t global_cnt;    // global counter for TIM4
uint16_t pt_counter;    // SYNC button press time counter
uint16_t rxbuf_ptr = 0; // RX buffer pointer for MIN protocol
uint32_t frq, frq_lb;   // Transmitter freq.
uint8_t seance_duration;
uint8_t number_of_foxes;
uint8_t fox_id;
uint8_t cw_speed;               // CW speed in WPM. Tdit = 1200 / cw_speed, 8 <= cw_speed <= 15
uint8_t t_dit;                  // Tdit = 1200 / cw_speed (for 1 ms tick)
uint8_t tx_wup_start_hi, tx_wup_start_lo;       // tx wake-up start
uint8_t  tx_wup_sb_hi, tx_wup_sb_lo;    // tx wake-up seance begin
uint8_t tx_wup_se_hi, tx_wup_se_lo;     // tx wake-up seance end
uint16_t lowbat_short;          // An integer Vdd representation, used for low batt. indication
uint16_t tx_ss;     // TX Time counter, seconds. Stored in EEPROM with TXTD_SEC discretion
uint16_t tx_mm;     // TX Time counter, minutes. Stored in EEPROM with TXTD_SEC discretion
uint16_t tx_prev_ts;    // TX Time counter previous timestamp
uint16_t total_mm;      // Total worktime minutes counter
uint16_t total_prev_ts;
uint16_t runtime_mm;    // Runtime minutes counter
uint16_t runtime_prev_ts;
uint16_t calib_idx;             // Index for calibration
int8_t cs_half = 0;             // current half step index
int8_t cs_full = 0;             // current full step index
int16_t cur_pos = 0x7fff;       // current shaft position
uint16_t iant_max_pos;          // max Antenna current shaft position index
uint8_t pwm_inc, pwm_cnt, pwm_limit;
uint16_t Dref_Factory;  // Factory Vref int value
uint16_t Dref_Measured; // Measured Vref int value
uint16_t last_vref;     // last measured vref
uint16_t ultimate_vref; // last vref value for calibration

float Vref;             // Factory Vref converted to float
volatile float Temp;             // Temperature, C

rtctime_t tm;   // Real-Time Clock data

struct flag_t flag;     // state flags
struct min_context min_ctx;

enum press_t press;

char rxbuf[RXBUF_SZ];   // rx buffer for UART
uint8_t iant_val[MAX_PAYLOAD];  // i_ANT values
uint8_t config[CFG_SIZE];    // fox config report
uint16_t adc_data[2];    // (PA5) ADC1_IN1 - V_MES, (PD5) ADC1_IN9 - I_ANT
uint16_t scorr_data[SCORR_DATA_SZ];     // i_ANT data for backlash correction

//-------------- Global constants --------------
const char id[24]={ 0x07, 0x0f, 0xff, 0x07, 0x0f, 0x02, 0x07, 0x0f,
                    0x04, 0x07, 0x0f, 0x08, 0x07, 0x0f, 0x10, 0x07,
                    0x0f, 0x20, 0x08, 0xfe, 0xff, 0x05, 0xfe, 0xff };   // MO, MOE, MOI, MOS, MOH, MO5, S, N

const uint8_t table_128[128] = { 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10,
11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 31, 32, 33, 35, 36, 38, 40, 41, 43, 45, 47, 49, 51, 54, 56, 58, 61, 64, 66, 69, 72, 76, 79, 83,
86, 90, 94, 98, 103, 107, 112, 117, 122, 127, 133, 139, 145, 152, 158, 165, 173, 180, 188, 197, 206, 215, 224, 234, 245, 255 };

const uint8_t full_step[] = { 0x09, 0x05, 0x06, 0x0A };
const uint8_t half_step[] = { 0x09, 0x01, 0x05, 0x04, 0x06, 0x02, 0x0A, 0x08 };

//-------------- EEPROM location --------------
#pragma location=0x001000 // 1024 bytes of eeprom @ 0x001000
__no_init char EEPROM_Data[DATA_SIZE];  // We use only DATA_SIZE bytes

#pragma location=0x001000+DATA_SIZE
__no_init int8_t backlash; // backlash correction for motor

#define CALIB_SIZE 8192 /* number of accu calibration points */
#pragma location=0x00e000 // CALIB_SIZE bytes of flash
__no_init uint8_t calib_data_write[CALIB_SIZE];
uint16_t *calib_data_read=(uint16_t *)calib_data_write;

//------------- V_REF location ----------------
#pragma location=0x004910
__near __no_init const char factory_vref_lsb;
//------------- TS_REF location ----------------
#pragma location=0x004911
__near __no_init const char factory_t90_lsb;

//-------------- Declaration of function prototypes -----------------
void mcu_init(void);    // Init the MCU registers
void basic_setup(void);
void charge(void);      // Accumulator charge task
void check_pwr(void);   // Check if we power on the device
void min_tx_byte(uint8_t port, uint8_t byte);
void wait_start(void);
void flash_led(float, uint8_t, uint8_t);
void fade_led(int8_t, uint8_t);
void startup_delay(void);
void foxhunt(void);
void tx_on(void);
void tx_off(void);
void one_symbol(char);
void tone(uint8_t);
void no_tone(uint8_t);
void tx_time_accounting(void);
void total_time_accounting(void);
void delay_tx_ms(uint16_t n);
void motor_reset_outs(void);
void motor_sweep(void);
void adc_init(void);
void dma_init(void);
void tune_pa(void);
void check_motor(void);
void motor_preheat(void);
bool fox_setup(void);
static inline void unlock_eeprom(void);
static inline void lock_eeprom(void);

volatile uint16_t dTemp;

void main( void ) {
  mcu_init();
  basic_setup();
  charge();
  check_pwr();
  rtc_init();
  fox_setup();
  // Temporary kludge. Remove after testing
  //PD_CR2_C26 = 1; // SYNC button interrupt enable
  wait_start();
  reset_rtc();
  start_tim2(); // sleep mode indication
  //flag.delay_not_expired=true;
  startup_delay();
////////////////////////////
//    unlock_rtc();
//    RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
//    RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
//    RTC_CR2_WUTIE = 0;    // Disable RTC Wakeup Interrupt
//    RTC_CR2_ALRAE = 0;    // Disable RTC Alarm
//    RTC_ISR2_ALRAF = 0;   // Reset Alarm flag
//    RTC_CR2_ALRAIE = 0;   // Disable RTC Alarm interrupt
//    lock_rtc();
//    if ( !flag.tuning_done ) tune_pa();
//    start_tim4();
//    tx_on();
//    dma_init();
//    adc_init();
//    flag.seance_is_running = true;
//    while(1) {
//      //one_symbol(0x2d);
//      for (int i = 0; i < 3; i++)
//      {
//        uint8_t tmp = fox_id * 3 + i;
//        one_symbol(id[tmp]);
//      }
//      if (fox_id <= 5) /* Send 'S' and 'N' without word space for better sounding */
//        no_tone(4);
//    }
////////////////////////////
  foxhunt();
}

void mcu_init(void) {
  __disable_interrupt();
  
  // PA2 - !CHRG (p2)
  PA_DDR_DDR2 = 0;      // in
  PA_CR1_C12 = 1;       // wpu
  PA_CR2_C22 = 0;       // extint
  EXTI_CR1_P2IS = 0x3;  // interrupt on both rising and falling edge

  // PA3 - !STBY (p3)
  PA_DDR_DDR3 = 0;      // in
  PA_CR1_C13 = 1;       // wpu
  PA_CR2_C23 = 0;       // no extint

  // PA4 - PA_CTL (p4)
  PA_DDR_DDR4 = 1;      // out
  PA_CR1_C14 = 1;       // push-pull

  // PA5 - V_MES (p5)
  PA_DDR_DDR5 = 0;      // in
  PA_CR1_C15 = 0;       // float

  // PA6 - PWR_GOOD (p6)
  PA_DDR_DDR6 = 0;      // in
  PA_CR1_C16 = 0;       // float

  // PB0 - B1 (p13)
  PB_DDR_DDR0 = 1;      // out
  PB_CR1_C10 = 1;       // push-pull

  // PB1 - B2 (p14)
  PB_DDR_DDR1 = 1;      // out
  PB_CR1_C11 = 1;       // push-pull

  // PC0 - SDA (p25)
  //PC_DDR_DDR0 = 1;      // out
  //PC_CR1_C10 = 0;       // o-d

  // PC1 - SCL (p26)
  //PC_DDR_DDR1 = 1;      // out
  //PC_CR1_C11 = 0;       // o-d

  // PC2 - UART_RX (p27)
  PC_DDR_DDR2 = 0;      // in
  PC_CR1_C12 = 1;       // wpu

  // PC3 - UART_TX (p28)
  PC_DDR_DDR3 = 1;      // out
  PC_CR1_C13 = 0;       // o-d

  // PC4 - WIFI_CTL (p29)
  PC_DDR_DDR4 = 1;      // out
  PC_CR1_C14 = 1;       // p-p

  // PD0 - PWR_CTL (p9)
  PD_DDR_DDR0 = 1;      // out
  PD_CR1_C10 = 0;       // o-d

  // PD1 - MOTOR_CTL (p10)
  PD_DDR_DDR1 = 1;      // out
  PD_CR1_C11 = 1;       // p-p

  // PD2 - A1 (p11)
  PD_DDR_DDR2 = 1;      // out
  PD_CR1_C12 = 1;       // p-p

  // PD3 - A2 (p12)
  PD_DDR_DDR3 = 1;      // out
  PD_CR1_C13 = 1;       // p-p

  // PD5 - I_ANT (p22)
  PD_DDR_DDR5 = 0;      // in
  PD_CR1_C15 = 0;       // float

  // PD6 - SYNC (p22)
  PD_DDR_DDR6 = 0;      // in
  PD_CR1_C16 = 1;       // wpu
  PD_CR2_C26 = 0;       // extint
  //EXTI_CR2_P6IS = 0x3; // interrupt on raising and falling edge only
  EXTI_CR2_P6IS = 0x2; // interrupt on falling edge only

  // PD7 - MANIP (p24)
  PD_DDR_DDR7 = 1;      // out
  PD_CR1_C17 = 1;       // p-p
  PD_CR2_C27 = 1;       // limit speed

  ITC_SPR4_VECT14SPR = 1;       // Lowest priority for SYNC button
  ITC_SPR5_VECT18SPR = 0;       // Medium priority for ADC1
  ITC_SPR6_VECT23SPR = 0;       // Medium priority for TIM1
  ITC_SPR6_VECT21SPR = 0;       // Medium priority for TIM3
  ITC_SPR7_VECT25SPR = 0;       // Medium priority for TIM4
  ITC_SPR7_VECT27SPR = 0;       // Medium priority for USART1
  ITC_SPR8_VECT28SPR = 0;       // Medium priority for USART1
  ITC_SPR2_VECT4SPR = 3;        // Highest priority for RTC interrupt

  __enable_interrupt();
}

void basic_setup() {
  PWR_CTL = PWR_OFF;
  WIFI_CTL = WIFI_DISABLE;
  PA_CTL = PA_DISABLE;
  MOTOR_CTL = MOTOR_DISABLE;

  i2c_init();
  i2cSendRegister(24, 0x00); // CLK_DIS_STATE to logic 0
  i2cSendRegister(03, 0xFF); // CLKx_OEB output disable
  i2cSendRegister(16, 0x80); // power down CLK0
  i2cSendRegister(17, 0x80); // power down CLK1
  i2cSendRegister(18, 0x80); // power down CLK2

  flag.delay_not_expired = false;
  flag.low_bat = false;
  flag.calibrate = false;
  flag.startup_time_set = false;
  flag.syncpress_detected = false;
  flag.tuning_in_progress = false;
  press = p_none;
  if ( EEPROM_Data[SWV_IDX] != SWVER ) {
    unlock_eeprom();
    EEPROM_Data[SWV_IDX] = SWVER;
    lock_eeprom();
  }
  /*
    Vdd = VDD_FACTORY * Dref_Factory / Dref_Measured
  */
  Dref_Factory = 0x0600 + factory_vref_lsb;
  Vref = (float)Dref_Factory * VDD_FACTORY / ADC_CONV;
  //while(PWR_GOOD==0);
  delay_ms(1);
  Dref_Measured = get_vref();
  lowbat_short = (uint16_t)(LOW_BAT * (float)Dref_Measured / Vref / 2.0);
}

void charge() {
  delay_ms(5); // ME4057 datasheet suggests for 4 ms comparator time to settle
  if ( _CHRG == 0 && _STBY == 1 ) {
    // charging mode
    flag.pwm_dir = true;
    if (_CHRG == 0) fade_led(0, 0);
    while (_CHRG == 0);
    stop_tim1(); // used by fade_led()
    if ( _CHRG == 1 && _STBY == 0 ) {
      // charge complete
      MANIP = ON;
      stop_tim4();
      // Update capacity registers
      while(1) __halt();
    }
    // otherwise indicate an error
    flash_led(0.5, 50, 0);
    while (1);
  }
  if ( _CHRG == 1 && _STBY == 0 && SYNC != PRESSED ) { // already charged
    MANIP = OFF;
    while(1) __halt();
  }
}

void check_pwr() {
  if ( SYNC == PRESSED ) {
    delay_ms(TURN_ON_TIME);
    if ( SYNC == PRESSED ) {
      PWR_CTL = PWR_ON;
      MANIP = ON;
      while ( SYNC == PRESSED );
      MANIP = OFF;
      PD_CR2_C26 = 1; // SYNC button interrupt enable
    }
  }
}

void min_tx_byte(uint8_t port, uint8_t byte) {
  uart_send_char(byte);
}

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port) {
  uint8_t i, x[1];

  switch (min_id) {
  case REQ_READCFG:
    min_send_frame(&min_ctx, REP_SENDCFG, (uint8_t *)config, CFG_SIZE);
    break;
  case CMD_WRITEBASIC:
    if ( len_payload != DATA_SIZE_BAS )
      min_send_frame(&min_ctx, REP_WRITEBASIC, ERR, 1);
    else {
      unlock_eeprom();
      EEPROM_Data[FOX_IDX] = min_payload[0];
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      // Set CW Speed according to Fox Id
      if ( EEPROM_Data[FOX_IDX] < 8 )
        EEPROM_Data[CWS_IDX] = CWS_NORMAL;
      else
        EEPROM_Data[CWS_IDX] = CWS_FAST;
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      EEPROM_Data[NRF_IDX] = min_payload[1];
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      EEPROM_Data[SDN_IDX] = min_payload[2];
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      EEPROM_Data[TXF_IDX] = min_payload[3];
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      lock_eeprom();
      if ( fox_setup() ) {
        x[0] = OK;
        min_send_frame(&min_ctx, REP_WRITEBASIC, x, 1);
      } else {
        x[0] = ERR;
        min_send_frame(&min_ctx, REP_WRITEBASIC, x, 1);
      }
    }
    break;
  case CMD_WRITEXPERT:
    if ( len_payload != DATA_SIZE_XPT ) {
      x[0] = ERR;
      min_send_frame(&min_ctx, REP_WRITEXPERT, x, 1);
    } else {
      unlock_eeprom();
      for(i=2; i< DATA_SIZE_XPT+2;i++) { // bypass first two bytes
        EEPROM_Data[i] = min_payload[i-2];
        while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
      }
      lock_eeprom();
      if ( fox_setup() ) {
        x[0] = OK;
        min_send_frame(&min_ctx, REP_WRITEXPERT, x, 1);
      } else {
        x[0] = ERR;
        min_send_frame(&min_ctx, REP_WRITEXPERT, ERR, 1);
      }
    }
    break;
  case CMD_TIMESET:
    if ( len_payload != DATA_SIZE_TIM )
      min_send_frame(&min_ctx, REP_TIMESET, ERR, 1);
    else {
      unlock_rtc();
      RTC_ISR1_INIT = 1;    // Enter RTC Init mode
      while (RTC_ISR1_INITF != 1);  // Wait for RTC Init allowed
      RTC_TR1 = bin2bcd(min_payload[1]);        // seconds
      RTC_TR2 = bin2bcd(min_payload[2]);        // minutes
      RTC_TR3 = bin2bcd(min_payload[3]);        // hours
      RTC_DR1 = bin2bcd(min_payload[4]);        // day
      RTC_DR2 = bin2bcd(min_payload[5]);        // month
      RTC_DR3 = bin2bcd(min_payload[6]);        // year
      RTC_ISR1_INIT = 0;  // Exit RTC Init mode
      RTC_ISR2_ALRAF = 0;    // Reset Alarm flag
      RTC_CR2_ALRAE = 0;    // Disable the RTC Alarm (required)
      while (RTC_ISR1_ALRAWF == 0);  // Wait for RTC Alarm Init allowed
      RTC_ALRMAR1 = 0;          // Set Alarm seconds
      RTC_ALRMAR2 = bin2bcd(min_payload[7]);   // Set Alarm minutes
      RTC_ALRMAR3 = bin2bcd(min_payload[8]);   // Set Alarm hours
      RTC_ALRMAR4 = bin2bcd(min_payload[9]);   // Set Alarm date
      RTC_CR2_ALRAE = 1;      // Enable RTC Alarm
      lock_rtc();
      unlock_eeprom();
      EEPROM_Data[SDN_IDX] = min_payload[0];
      lock_eeprom();
      flag.startup_time_set = true;
      flag.delay_not_expired = true;
      x[0] = ERR;
      if ( fox_setup() ) x[0] = OK;
      min_send_frame(&min_ctx, REP_TIMESET, x, 1);
    }
    break;
  case REQ_TUNING:
    tune_pa();
    min_send_frame(&min_ctx, REP_TUNING, iant_val, MAX_PAYLOAD);
    min_send_frame(&min_ctx, REP_TUNMAX, (uint8_t *)scorr_data, sizeof(scorr_data));
    x[0]=backlash;
    min_send_frame(&min_ctx, REP_BLVAL, x, 1);
    break;
  case REQ_BLINC:
    if (backlash < INT8_MAX)
    {
      unlock_eeprom();
      backlash++;
      lock_eeprom();
    }
    x[0]=backlash;
    min_send_frame(&min_ctx, REP_BLVAL, x, 1);
    break;
  case REQ_BLDEC:
    if (backlash > INT8_MIN)
    {
      unlock_eeprom();
      backlash--;
      lock_eeprom();
    }
    x[0]=backlash;
    min_send_frame(&min_ctx, REP_BLVAL, x, 1);
    break;
  }
}

uint32_t min_time_ms(void)
{
  return global_ms;
}

uint16_t min_tx_space(uint8_t port)
{
  // This is a lie
  return 266U;
}

void min_tx_start(uint8_t port)
{
  while (USART1_SR_TXE == 0); // wait for TX buffer to empty
  return;
}
void min_tx_finished(uint8_t port)
{
  while (USART1_SR_TC == 0); // wait for fill in TX buffer
  return;
}

void wait_start(void) {
  WIFI_CTL = WIFI_ENABLE;
  uart_init();
  min_init_context(&min_ctx, 0);
  unlock_rtc();
  RTC_CR2_ALRAIE = 0;    // Disable the RTC Alarm interrupt
  lock_rtc();
  while(!flag.sync_done)
  {
    if ( flag.uart_rx_complete )
    {
      flag.uart_rx_complete = false;
      min_poll(&min_ctx, (uint8_t *)rxbuf, (uint8_t)rxbuf_ptr);
      rxbuf_ptr = 0;
    }
    if ( press == p_short )
    {
      flag.sync_done = true;
      WIFI_CTL = WIFI_DISABLE;
      stop_tim1();
      stop_tim4();
      uart_stop();
      unlock_rtc();
      RTC_CR2_ALRAIE = 1;    // Enable the RTC Alarm interrupt
      lock_rtc();
      tx_off();
      press = p_none;
    }
    MANIP=ON;
    //__wait_for_interrupt();
  }
  RTC_CR2_ALRAIE = 1;      // Enable RTC Alarm interrupt
}

static inline void unlock_eeprom(void) {
  FLASH_DUKR = 0xAE; // First unlock key
  FLASH_DUKR = 0x56; // Second unlock key
}

static inline void lock_eeprom(void) {
  FLASH_IAPSR_DUL = 0; // Clear DUL bit
}

static inline void unlock_flash(void) {
  FLASH_PUKR = 0x56;
  FLASH_PUKR = 0xAE;
}

static inline void lock_flash(void) {
  FLASH_IAPSR_PUL = 0; // Clear PUL bit
}

bool fox_setup(void) {
  bool status = CFG_OK;
  uint16_t tmp, calib_sz;
  
  tx_ss = 0;
  tx_mm = 0;
  tx_prev_ts = 0;
  total_mm = 0;
  total_prev_ts = 0;
  runtime_mm = 0;
  runtime_prev_ts = 0;

  /* Init flags */
  flag.sync_done = false;

  /* Get current config */
  for (uint8_t i=0;i<DATA_SIZE;i++)
  {
    config[i]=EEPROM_Data[i];
  }
  // number of stored calibration points
  calib_sz = (EEPROM_Data[MAH_IDX] << 8) | EEPROM_Data[MAL_IDX];

  fox_id = EEPROM_Data[FOX_IDX] % 8;
  number_of_foxes = EEPROM_Data[NRF_IDX] % 8;
  seance_duration = EEPROM_Data[SDN_IDX];
  cw_speed = EEPROM_Data[CWS_IDX];

  if (number_of_foxes == 0)
  {
    number_of_foxes = 1;
    status = CFG_ERR;
  }
    
  if (cw_speed < CWSPEED_MIN || cw_speed > CWSPEED_MAX)
    status = CFG_ERR;

  if (seance_duration < SD_MIN)
    status = CFG_ERR;

  // Setup accumulator calibration
  if (EEPROM_Data[MAH_IDX] & EEPROM_Data[MAL_IDX] == 0xFF) {
    flag.calibrate = true;
    number_of_foxes = 1;
  } else
  /* calculate current charge level */
  {
    // add some load on accu
    MOTOR_CTL = MOTOR_ENABLE;
    A1_OUT = 0;
    A2_OUT = 1;
    B1_OUT = 0;
    B2_OUT = 1;
    adc_deinit();
    delay_ms(500); // measure delay
    tmp=get_vin();
    motor_reset_outs();
    MOTOR_CTL = MOTOR_DISABLE;
    config[DDH_IDX] = 0;
    config[DDL_IDX] = 0;
    if (calib_data_read[0])
    {
      for(uint16_t i=0;i<calib_sz;i++)
      {
        if (tmp<=calib_data_read[i])
        {
          config[DDH_IDX] = HIBYTE(i);
          config[DDL_IDX] = LOBYTE(i);
        }
      }
    }
  }

  // frequency in Hz
  frq = (uint32_t)(( FRQ_BASE + EEPROM_Data[TXF_IDX] ) * 1000.0f +
                   EEPROM_Data[TXC_IDX] * 4 - 500.0f);
  frq_lb = frq + LOW_BAT_HZ;
  si5351aSetFrequency(frq);

  // dit time, ms
  t_dit = (uint8_t)(1200 / cw_speed);
  
  if (number_of_foxes > 1) {
    if (fox_id > 1)
    {
      /* Compute RTC Wakeup values before the 1st cycle */
      tmp = (fox_id - 1) * seance_duration - 1;
      tx_wup_start_hi = HIBYTE(tmp);
      tx_wup_start_lo = LOBYTE(tmp);
    }
    
    /* Compute RTC Wakeup values to start the TX chain within cycle */
    tmp = (number_of_foxes - 1) * seance_duration - 1;
    tx_wup_sb_hi = HIBYTE(tmp);
    tx_wup_sb_lo = LOBYTE(tmp);

    /* Compute RTC Wakeup values to stop the TX chain after seance end */
    tmp = seance_duration - 1;
    tx_wup_se_hi = HIBYTE(tmp);
    tx_wup_se_lo = LOBYTE(tmp);
  }

  return status;
}

void flash_led(float freq, uint8_t duty_cycle, uint8_t count) {
  uint16_t pscr, arr, ccr;

  flag.pwm_mode = -2;
  pscr = 1000U;
  arr = (uint16_t)(F_MASTER * 1000000UL / pscr / freq - 1);
  ccr = (arr + 1) / 100 * duty_cycle;
  CLK_PCKENR2_PCKEN21 = 1;      // Enable clock to TIM1
  TIM1_PSCRH = HIBYTE(pscr);    // Prescaler
  TIM1_PSCRL = LOBYTE(pscr);
  TIM1_ARRH = HIBYTE(arr);      // Auto-reload reg
  TIM1_ARRL = LOBYTE(arr);
  TIM1_CCR1H = HIBYTE(ccr);
  TIM1_CCR1L = LOBYTE(ccr);
  TIM1_CCMR1_OC1PE = 1;         // preload register enabled
  TIM1_CCMR1_OC1M = 0x06;       // PWM mode 1
  if ( count )
  {
    TIM1_CR1_OPM = 1;           // one pulse mode
    TIM1_RCR = count - 1;
  } else
  {
    TIM1_CR1_OPM = 0;
  }
  TIM1_CCER1_CC1NP = 1;
  TIM1_CCER1_CC1NE = 1;       // Enable TIM1_CH1N as output, active high
  TIM1_SR1_UIF = 0;     // Clr. interrupt flag
  TIM1_IER_UIE = 1;     // Enable interrupt gen.
  TIM1_EGR_UG = 1;      // UG (update generation) is required for PWM
  TIM1_BKR_MOE = 1;     // enable output
  TIM1_CR1_CEN = 1;     // Counter enable
}

void fade_led(int8_t mode, uint8_t count) {
  if ( count ) pwm_limit = count + 1;
  switch (mode)
  {
  case -1:
    flag.pwm_mode = -1;
    flag.pwm_dir = false;
    pwm_inc = 128;
    break;
  case 0:
    flag.pwm_mode = 0;
    flag.pwm_dir = true;
    break;
  default:
    flag.pwm_mode = 1;
    flag.pwm_dir = true;
  }

  CLK_PCKENR2_PCKEN21 = 1;      // Enable clock to TIM1
  TIM1_PSCRH = 0;    // Prescaler
  TIM1_PSCRL = 0;
  TIM1_ARRH = 0;      // Auto-reload reg
  TIM1_ARRL = 0xff;
  TIM1_CCR1H = 0;       // counter
  TIM1_CCR1L = 0;
  TIM1_CCMR1_OC1PE = 1;         // preload register enabled
  TIM1_CCMR1_OC1M = 0x06;       // PWM mode 1
  TIM1_CR1_OPM = 0;             // one time mode
  TIM1_CCER1_CC1NP = 0;         // polarity
  TIM1_CCER1_CC1NE = 1;       // Enable TIM1_CH1N as output, active high
  TIM1_SR1_UIF = 0;     // Clr. update interrupt flag
  TIM1_IER_UIE = 1;     // enable interrupt gen.
  TIM1_EGR_UG = 1;      // UG (update generation) is required for PWM
  TIM1_BKR_MOE = 1;     // enable output
  TIM1_CR1_CEN = 1;     // Counter enable
}

void startup_delay(void) {
  flag.tuning_requested = false;
  while (flag.delay_not_expired)
  {
    if ( flag.tuning_requested )
      tune_pa();
    else
      __wait_for_interrupt();
  }
}

void foxhunt(void) {
  uint8_t i, tmp;

  adc_deinit();
  dma_deinit();
  
  // beacon mode
  if ( number_of_foxes == 1 )
  {
    stop_tim2(); // not used in this mode
    // disable RTC alarms and wakeups
    unlock_rtc();
    RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
    RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
    RTC_CR2_WUTIE = 0;    // Disable RTC Wakeup Interrupt
    RTC_CR2_ALRAE = 0;    // Disable RTC Alarm
    RTC_ISR2_ALRAF = 0;   // Reset Alarm flag
    RTC_CR2_ALRAIE = 0;   // Disable RTC Alarm interrupt
    lock_rtc();

    tx_time_accounting();
    total_time_accounting();
    if ( !flag.tuning_done ) tune_pa();
    start_tim4();
    tx_on();
    dma_init();
    adc_init();
    flag.seance_is_running = true;
    while (1)
    {
      /* Send Fox ID in a loop */
      for (i = 0; i < 3; i++)
      {
        tmp = fox_id * 3 + i;
        one_symbol(id[tmp]);
      }
      if (fox_id <= 5) /* Send 'S' and 'N' without word space for better sounding */
        no_tone(4);
      /* End of Fox ID */

      tx_time_accounting();
      total_time_accounting();
    }
  }
  
  if (fox_id > 1)
  {
    tx_off();
    update_wakeup(tx_wup_start_hi, tx_wup_start_lo);
    while (!flag.seance_is_running)
    {
      if ( flag.tuning_requested )
        tune_pa();
      else
        __wait_for_interrupt();
    }
  } else {
    update_wakeup(tx_wup_se_hi, tx_wup_se_lo);
    flag.seance_is_running = true;
  }

  if ( !flag.tuning_done ) tune_pa();
  tx_time_accounting();
  total_time_accounting();
  while (1)
  {
    start_tim4();
    dma_init(); // track low battery only within a seance
    adc_init();
    tx_on();
    while(flag.seance_is_running)
    {
      if ( !flag.tuning_done ) tune_pa();
      /* Send Fox ID in a loop */
      for (i = 0; i < 3; i++)
      {
        tmp = fox_id * 3 + i;
        one_symbol(id[tmp]);
      }
      if (fox_id <= 5) /* Send 'S' and 'N' without word space for better sounding */
        no_tone(4);
      /* End of Fox ID */
    }
    tx_off();
    stop_tim4();
    adc_deinit(); // do not track low battery in the pause
    dma_deinit();
    while (!flag.seance_is_running)
    {
      if ( flag.tuning_requested )
        tune_pa();
      else
        __wait_for_interrupt();
    }
    tx_time_accounting();
    total_time_accounting();
  }
}

void tx_on(void) {
  // we have no other means to turn off the device when number_of_foxes==1
  if ( number_of_foxes > 1 ) PD_CR2_C26 = 0; // SYNC button interrupt disable
  i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A); // _PDN disable
  i2cSendRegister(3, 0xFE); // output enable
  PA_CTL = PA_ENABLE;
}

void tx_off(void) {
  MANIP = OFF;
  PA_CTL = PA_DISABLE;
  i2cSendRegister(3, 0xFF); // output disable
  i2cSendRegister(SI_CLK0_CONTROL, 0xCF | SI_CLK_SRC_PLL_A); // _PDN enable
  // we have no other means to turn off the device when number_of_foxes==1
  if ( number_of_foxes > 1 ) PD_CR2_C26 = 1; // SYNC button interrupt enable
}

void tx_time_accounting()
{
  uint16_t tmp;

  // Finish the clibration:
  //   write calibration results
  //   and indicate calibration end when done
  if (flag.low_bat & flag.calibrate) {
    adc_deinit();
    dma_deinit();
    // Disable RTC, we don't need it anymore
    disable_rtc_wakeup();
    CLK_PCKENR2_PCKEN22 = 0;      // Disable clock to RTC

    tx_off(); // turn off the TX chain
    flag.seance_is_running = false;

    // save calibration time
    calib_idx++;
    unlock_flash();
    calib_data_write[calib_idx*2]=HIBYTE(ultimate_vref);
    while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
    calib_data_write[calib_idx*2+1]=LOBYTE(ultimate_vref);
    while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
    lock_flash();
    calib_idx++; // number of calibration points
    unlock_eeprom();
    EEPROM_Data[MAH_IDX] = HIBYTE(calib_idx);
    while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
    EEPROM_Data[MAL_IDX] = LOBYTE(calib_idx);
    while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
    lock_eeprom();
    flash_led(1, 50, 0);
    while (1);
  }
  
//  if (flag.low_bat)
//    return; /* Avoid updating on low battery condition */

  read_rtc(&tm);

  // Convert current time from BCD to binary and calculate seconds
  tmp = bcd2bin(tm.mm) * 60 + bcd2bin(tm.ss);

  // first call of this function
  if (tx_prev_ts == 0 && tx_ss == 0) {
    tx_prev_ts = tmp;
    if (flag.calibrate) {
      // add some load on accu
      MOTOR_CTL = MOTOR_ENABLE;
      A1_OUT = 0;
      A2_OUT = 1;
      B1_OUT = 0;
      B2_OUT = 1;
      adc_deinit();
      tmp=get_vin();
      motor_reset_outs();
      MOTOR_CTL = MOTOR_DISABLE;
      unlock_flash();
      calib_data_write[(calib_idx*2)]=HIBYTE(tmp);
      while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
      calib_data_write[(calib_idx*2)+1]=LOBYTE(tmp);
      while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
      lock_flash();
    }
    return;
  }
  
  if (tmp != tx_prev_ts)
  {
    if (tmp < tx_prev_ts)
    {
      // next hour is started over
      tx_ss += 3600 - tx_prev_ts + tmp;
    } else
    {
      tx_ss += tmp - tx_prev_ts;
    }
    tx_prev_ts = tmp;

    /* Save calibration values every TXTD_SEC seconds */
    if ((tx_ss >= TXTD_SEC) && flag.calibrate)
    {
      adc_deinit();
      dma_deinit();
      calib_idx++;
      unlock_flash();
      calib_data_write[calib_idx*2]=HIBYTE(last_vref);
      while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
      calib_data_write[calib_idx*2+1]=LOBYTE(last_vref);
      while(!(FLASH_IAPSR_EOP)); // Wait while write is finished
      lock_flash();
      adc_init();
      dma_init();
      tx_ss %= 60;
    }
  }
}

void total_time_accounting()
{
  uint16_t tmp = 0;
  uint8_t hi, lo;

  read_rtc(&tm);

  // Convert current time from BCD to binary
  tm.mm = bcd2bin(tm.mm);
  tm.hh = bcd2bin(tm.hh);

  tmp = tm.hh * 60;
  tmp += tm.mm;

  // first call of this function
  if (runtime_prev_ts == 0 && tx_ss == 0) {
    runtime_prev_ts = tmp;
    return;
  }

  if (tmp != runtime_prev_ts)
  {
    if (tmp < runtime_prev_ts)
    {
      // next day is started over
      runtime_mm += 1440 - runtime_prev_ts + tmp;
    } else
    {
      runtime_mm += tmp - runtime_prev_ts;
    }
    runtime_prev_ts = tmp;
  }

  if (tmp != total_prev_ts)
  {
    if (tmp < total_prev_ts)
    {
      // next day is started over
      total_mm += 1440 - total_prev_ts + tmp;
    } else
    {
      total_mm += tmp - total_prev_ts;
    }
    total_prev_ts = tmp;

    hi = EEPROM_Data[TWH_IDX];
    lo = EEPROM_Data[TWL_IDX];
    tmp = hi;
    tmp <<= 8;
    tmp += lo;
    tmp += total_mm / 60;

    unlock_eeprom();
    if (hi != HIBYTE(tmp))
    {
      EEPROM_Data[TWH_IDX] = HIBYTE(tmp);
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
    }
    if (lo != LOBYTE(tmp))
    {
      EEPROM_Data[TWL_IDX] = LOBYTE(tmp);
      while(!(FLASH_IAPSR_EOP)); // Wait while EEPROM write is finished
    }
    lock_eeprom();
    total_mm %= 60;
  }
  // Runtime TIMEOUT
  if ( runtime_mm >= MAX_RUNTIME ) {
    PWR_CTL=PWR_OFF; // power off should be enough
    // assertion:
    tx_off();
    // disable RTC alarms and wakeups
    unlock_rtc();
    RTC_CR2_WUTE = 0;     // Disable RTC Wakeup
    RTC_ISR2_WUTF = 0;    // Reset WU interrupt flag
    RTC_CR2_WUTIE = 0;    // Disable RTC Wakeup Interrupt
    RTC_CR2_ALRAE = 0;    // Disable RTC Alarm
    RTC_ISR2_ALRAF = 0;   // Reset Alarm flag
    RTC_CR2_ALRAIE = 0;   // Disable RTC Alarm interrupt
    lock_rtc();
    flash_led(0.5, 50, 0);
    while(1);
  }
}

/************************ CW generation ******************/
void one_symbol(char cw_char)// ------ send a character -----------
{
  char t_char;

  if (flag.seance_is_running && cw_char != 0xFE)
  {
    if(cw_char == 0)
    {
      no_tone(5);
    }
    else if (cw_char != 0xFF)
    {
      while(cw_char > 1 && flag.seance_is_running)
      {
        t_char = cw_char & 0x01;
        cw_char >>= 1;                
        if(t_char)               // dah
         {
           tone(3);
           no_tone(1);
         }
         else                     // dit
         {
           tone(1);
           no_tone(1);
         }
      }
    }
    if (fox_id <= 5)
      no_tone(2);
    else
      no_tone(1); // Less word space for S and N
  }
}

void tone(uint8_t dits_number)// ------------ dits --------------
{
  uint32_t fq;

  /* Alternate the DDS frequency to indicate the low battery condition */
  if (flag.low_bat)
  {
    fq = frq_lb;
    frq_lb = frq;
    frq = fq;
    i2cSendRegister(SI_CLK0_CONTROL, 0xCF | SI_CLK_SRC_PLL_A); // _PDN enable
    si5351aSetFrequency(frq);
    i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A); // _PDN disable
  }

  MANIP = ON;
  delay_tx_ms(dits_number * t_dit);
  last_vref=adc_data[0];
  MANIP = OFF;
}

void no_tone(uint8_t dits_number)// ---------- spaces -------------
{
  MANIP = OFF;
  delay_tx_ms(dits_number * t_dit);
}

void delay_tx_ms(uint16_t n)
{
  if (flag.seance_is_running)
  {
    global_cnt = n;
    while (global_cnt && flag.seance_is_running) {
      __wait_for_interrupt();
    }
  }
}

void motor_reset_outs() {
  A1_OUT = 0;
  A2_OUT = 0;
  B1_OUT = 0;
  B2_OUT = 0;
}

void motor_preheat() {
  uint16_t preheat_delay;
  uint16_t Dt90_Factory = factory_t90_lsb + 0x0300;
  float Vdd, Vt90, Vtamb, adc_resolution;

  Vdd = VDD_FACTORY * Dref_Factory / Dref_Measured;
  adc_resolution = 1000.0 * Vdd / ADC_CONV; // in mV
  Vt90 = 1000.0 * VDD_FACTORY * Dt90_Factory / ADC_CONV; // 90 degree temperature in mV
  Vtamb = adc_resolution * get_vtemp(); // Ambiance temperature in mV
  Temp = 90.0 - (Vt90 - Vtamb) / AVG_SLOPE;
  if (Temp < PREHEAT_MINTEMP)
  {
    // heating speed is about 20 deg/min ~ 1/3 deg/s
    //preheat_delay = (uint16_t)((PREHEAT_MINTEMP - Temp) * 3.0);
    preheat_delay = 40;
    if (preheat_delay < 65)
      preheat_delay *= 1000;
    else
      preheat_delay = 50000;
    MOTOR_CTL = MOTOR_ENABLE;
    A1_OUT = 0;
    A2_OUT = 1;
    B1_OUT = 0;
    B2_OUT = 1;
    flash_led(1.5, 50, 0);
    delay_ms(preheat_delay);
    stop_tim1();
    MANIP=OFF;
    motor_reset_outs();
    MOTOR_CTL = MOTOR_DISABLE;
  }
}

bool step_ccw_full(uint8_t step_delay) {
  uint8_t t;

  if (cur_pos < FULLSTEP_MAXPOS)
  {
    if (++cs_full > 3) cs_full = 0;
    t = full_step[cs_full];
    B2_OUT = t & 0x01;
    t >>= 1;
    B1_OUT = t & 0x01;
    t >>= 1;
    A2_OUT = t & 0x01;
    t >>= 1;
    A1_OUT = t & 0x01;
    delay_ms(step_delay);
    cur_pos++;
    return true;
  }
  return false;
}

bool step_cw_full(uint8_t step_delay) {
  uint8_t t;

  if (cur_pos > FULLSTEP_MINPOS)
  {
    if (--cs_full < 0) cs_full = 3;
    t = full_step[cs_full];
    B2_OUT = t & 0x01;
    t >>= 1;
    B1_OUT = t & 0x01;
    t >>= 1;
    A2_OUT = t & 0x01;
    t >>= 1;
    A1_OUT = t & 0x01;
    delay_ms(step_delay);
    cur_pos--;
    return true;
  }
  return false;
}

bool step_ccw_half(uint8_t step_delay) {
  uint8_t t;

  if (cur_pos < HALFSTEP_MAXPOS)
  {
    if (++cs_half > 7) cs_half = 0;
    t = half_step[cs_half];
    B2_OUT = t & 0x01;
    t >>= 1;
    B1_OUT = t & 0x01;
    t >>= 1;
    A2_OUT = t & 0x01;
    t >>= 1;
    A1_OUT = t & 0x01;
    delay_ms(step_delay);
    cur_pos++;
    return true;
  }
  return false;
}

bool step_cw_half(uint8_t step_delay) {
  uint8_t t;

  if (cur_pos > HALFSTEP_MINPOS)
  {
    if (--cs_half < 0) cs_half = 7;
    t = half_step[cs_half];
    B2_OUT = t & 0x01;
    t >>= 1;
    B1_OUT = t & 0x01;
    t >>= 1;
    A2_OUT = t & 0x01;
    t >>= 1;
    A1_OUT = t & 0x01;
    delay_ms(step_delay);
    cur_pos--;
    return true;
  }
  return false;
}

void shaft_moveto(uint16_t pos, uint8_t step_delay) {
  while (cur_pos > pos && step_cw_half(step_delay));
  while (cur_pos < pos &&  step_ccw_half(step_delay));
}

void shaft_init() {
  if ( cur_pos > HALFSTEP_MAXPOS )
    cur_pos = HALFSTEP_MAXPOS;

  shaft_moveto(HALFSTEP_MINPOS, STEP_DELAY_INIT);
  motor_reset_outs();
}

char dig[10];
static char *itoa_simple_helper(char *dest, int i) {
  if (i <= -10) {
    dest = itoa_simple_helper(dest, i/10);
  }
  *dest++ = '0' - i%10;
  return dest;
}

char *itoa_simple(char *dest, int i) {
  char *s = dest;
  if (i < 0) {
    *s++ = '-';
  } else {
    i = -i;
  }
  *itoa_simple_helper(s, i) = '\0';
  return dest;
}

uint16_t shaft_findmax(void) {
  uint16_t t, c, max;
  float d;

  t = 0; c=0; max=0;
  d = (float)HALFSTEP_MAXPOS/(float)MAX_PAYLOAD;

//  if ( flag.tuning_in_progress ) MANIP=ON;
  while(ADC1_SR_EOC == 0);
  while(ADC1_SR_EOC == 0);
  c = adc_data[1];
  iant_val[(uint8_t)((float)cur_pos/d)] = adc_data[1]/16;
  // do the sweep
  while (cur_pos < HALFSTEP_MAXPOS)
  {
    step_ccw_half(STEP_DELAY);
//    if ( flag.tuning_in_progress ) MANIP=ON;
    while(ADC1_SR_EOC == 0);
    while(ADC1_SR_EOC == 0); // double convertion to make sure it's really done
    t = adc_data[1];
//    if ( flag.tuning_in_progress ) MANIP=OFF;
    iant_val[(uint8_t)((float)cur_pos/d)] = t/16;
    if (t >= c)
    {
      c = t;
      max = cur_pos;
    }
  }
  scorr_data[0]=max; // save max position for html survey
  scorr_data[2]=c; // save max i_ANT value for html survey
  return max;
}

void tune_pa(void)
{
  int16_t t;

  PD_CR2_C26 = 0; // SYNC button interrupt disable

  //motor_preheat();
  MOTOR_CTL = MOTOR_ENABLE;
  shaft_init();

  flag.tuning_requested = false;
  i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A); // _PDN disable
  i2cSendRegister(3, 0xFE); // output enable
  PA_CTL = PA_ENABLE;
  MANIP = ON;
  flag.tuning_in_progress = true;

  adc_deinit();
  dma_deinit();
  dma_init();
  adc_init();
  t = shaft_findmax();
  MANIP = OFF;
  t += backlash;
  shaft_moveto(t, STEP_DELAY);
  motor_reset_outs();
  MOTOR_CTL = MOTOR_DISABLE;

  MANIP = ON;
  while(ADC1_SR_EOC == 0);
  while(ADC1_SR_EOC == 0); // double convertion to make sure it's really done
  scorr_data[3]=adc_data[1]; // save current i_ANT value for html survey
  scorr_data[1]=cur_pos; // save current position for html survey
  

  if (!flag.seance_is_running)
  {
    adc_deinit();
    dma_deinit();
    PA_CTL = PA_DISABLE;
    i2cSendRegister(3, 0xFF); // output disable
    i2cSendRegister(SI_CLK0_CONTROL, 0xCF | SI_CLK_SRC_PLL_A); // _PDN enable
    MANIP = OFF;
  }
  flag.tuning_done = true;
  flag.tuning_in_progress = false;
  PD_CR2_C26 = 1; // SYNC button interrupt enable
}

void check_motor(void)
{
  MOTOR_CTL = MOTOR_ENABLE;
  shaft_init();
  motor_reset_outs();
  MOTOR_CTL = MOTOR_DISABLE;
  MANIP=OFF; // Annoying BUG!
}

/*
=== SNIP ==================================================================

 MORSE ENCODING ...

 One morse character per BYTE, bitwise, LSB to MSB.

 0 = dit, 1 = dah.  The byte is shifted to the right bit by bit, until the
 last 1 is left, this 1 is an END OF CHARACTER indicator.
 A maximum of 7 elements can be encoded, (error) is excluded.

                        SWITCHES
     CODE               MSD  LSD   HEX    Comments
    ------              ---  ---   ---    --------------------
 KN -.--.   00101101      2   13    2d      Go only
 SK ...-.-  01101000      5    8    58      Clear
 AR .-.-.   00101010      2   10    2a      Over, end of message
 BT -...-   00110001      3    1    31      Pause
 AS .-...   00100010      2    2    22      Wait, stand by
  / -..-.   00101001      2    9    29
  0 -----   00111111      3   15    3f
  1 .----   00111110      3   14    3e
  2 ..---   00111100      3   12    3c
  3 ...--   00111000      3    8    38
  4 ....-   00110000      3    0    30
  5 .....   00100000      2    0    20
  6 -....   00100001      2    1    21
  7 --...   00100011      2    3    23
  8 ---..   00100111      2    7    27
  9 ----.   00101111      2   15    2f
  A .-      00000110      0    6    06
  B -...    00010001      1    1    11
  C -.-.    00010101      1    5    15
  D -..     00001001      0    9    09
  E .       00000010      0    2    02
  F ..-.    00010100      1    4    14
  G --.     00001011      0   11    0b
  H ....    00010000      1    0    10
  I ..      00000100      0    4    04
  J .---    00011110      1   14    1e
  K -.-     00001101      0   13    0d
  L .-..    00010010      1    2    12
  M --      00000111      0    7    07
  N -.      00000101      0    5    05
  O ---     00001111      0   15    0f
  P .--.    00010110      1    6    16
  Q --.-    00011011      1   11    1b
  R .-.     00001010      0   10    0a
  S ...     00001000      0    8    08
  T -       00000011      0    3    03
  U ..-     00001100      0   12    0c
  V ...-    00011000      1    8    18
  W .--     00001110      0   14    0e
  X -..-    00011001      1    9    19
  Y -.--    00011101      1   13    1d
  Z --..    00010011      1    3    13
  SPACE     00000000      0    0    00     Word space (special exception)
  NULL      11111110      -    -    FE     No character (for ARDF triplets)
  END       11111111     15   15    ff     End character (for ARDF triplets)

*/
