// si5351a.h
#ifndef SI5351A_H
#define SI5351A_H

#include <stdint.h>
#include "config.h"

//-------------- Macros definitions -----------------
#define F_MASTER_HZ             ( F_MASTER * 1000000UL )
#define F_I2C_HZ                100000UL /* 100 kHz */
#define I2C_CCR                 F_MASTER_HZ / 2 / F_I2C_HZ
#define XTAL_FREQ	        25000000UL /* Si5351 crystal frequency */
#define SI5351_BUS_BASE_ADDR	0xC0
#define SI_CLK0_CONTROL	16			/* Register definitions */
#define SI_SYNTH_PLL_A	26
#define SI_SYNTH_MS_0		42
#define SI_PLL_RESET		177
#define SI_R_DIV_1		0			/* R-division ratio definitions */
#define SI_CLK_SRC_PLL_A	0

//Задать таймаут в микросекундах
#define set_tmo_us(time)  i2c_timeout = (unsigned long int)(F_MASTER * time)

//Задать таймаут в миллисекундах
#define set_tmo_ms(time)  i2c_timeout = (unsigned long int)(F_MASTER * time * 1000)

#define tmo               i2c_timeout--

//Ожидание наступления события event
//в течении времени timeout в мс
#define wait_event(event, timeout) set_tmo_ms(timeout);\
                                   while(event && --i2c_timeout);\
                                   if(!i2c_timeout) return 1;
                                   
#define i2cSendRegister(reg, data) i2c_wr_reg(SI5351_BUS_BASE_ADDR, (uint8_t)(reg), (uint8_t)(data))

//-------------- Declaration of function prototypes -----------------
void i2c_init(void);
uint8_t i2c_wr_reg(uint8_t address, uint8_t reg_addr, uint8_t data);
void si5351aSetFrequency(uint32_t frequency);

#endif
