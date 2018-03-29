#include <iostm8L151k6.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "si5351a.h"

enum si5351_clock {SI5351_CLK0, SI5351_CLK1, SI5351_CLK2, SI5351_CLK3,
	SI5351_CLK4, SI5351_CLK5, SI5351_CLK6, SI5351_CLK7};

//Таймаут ожидания события I2C
static unsigned long int i2c_timeout;

void i2c_init() {
  uint16_t ccr;

  CLK_PCKENR1_PCKEN13 = 1; // enable clock to i2c
  I2C1_FREQR_FREQ = F_MASTER;
  I2C1_CR1_PE = 0; // turn off i2c
  I2C1_CCRH_F_S = 0; // i2c slow mode

  ccr = I2C_CCR;

  I2C1_TRISER_TRISE = F_MASTER + 1;
  I2C1_CCRL = ccr & 0xFF;
  I2C1_CCRH = (ccr >> 8) & 0x0F;
  I2C1_CR1_PE = 1;
  I2C1_CR2_ACK = 1;
}

uint8_t i2c_wr_reg(uint8_t address, uint8_t reg_addr, uint8_t data) {

  //Ждем освобождения шины I2C
  wait_event(I2C1_SR3_BUSY, 10);
    
  //Генерация СТАРТ-посылки
  I2C1_CR2_START = 1;
  //Ждем установки бита SB
  wait_event(!I2C1_SR1_SB, 1);
  
  
  //Записываем в регистр данных адрес ведомого устройства
  I2C1_DR = address & 0xFE;
  //Ждем подтверждения передачи адреса
  wait_event(!I2C1_SR1_ADDR, 1);
  //Очистка бита ADDR чтением регистра SR3
  I2C1_SR3;
  
  
  //Ждем освобождения регистра данных
  wait_event(!I2C1_SR1_TXE, 1);
  //Отправляем адрес регистра
  I2C1_DR = reg_addr;
  
  //Отправка данных
  //Ждем освобождения регистра данных
  wait_event(!I2C1_SR1_TXE, 1);
  I2C1_DR = data;
  
  //Ловим момент, когда DR освободился и данные попали в сдвиговый регистр
  wait_event(!(I2C1_SR1_TXE && I2C1_SR1_BTF), 1);
  
  //Посылаем СТОП-посылку
  I2C1_CR2_STOP = 1;
  //Ждем выполнения условия СТОП
  wait_event(I2C1_CR2_STOP, 1);

  return 0;
}

void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
	uint32_t P1;					// PLL config register P1
	uint32_t P2;					// PLL config register P2
	uint32_t P3;					// PLL config register P3

	P1 = (uint32_t)(128 * ((float)num / (float)denom));
	P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
	P2 = (uint32_t)(128 * ((float)num / (float)denom));
	P2 = (uint32_t)(128 * num - denom * P2);
	P3 = denom;

        i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
	i2cSendRegister(pll + 1, (P3 & 0x000000FF));
	i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
	i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
	i2cSendRegister(pll + 4, (P1 & 0x000000FF));
	i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
	i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
	i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}

void setupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv)
{
	uint32_t P1;					// Synth config register P1
	uint32_t P2;					// Synth config register P2
	uint32_t P3;					// Synth config register P3

	P1 = 128 * divider - 512;
	P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
	P3 = 1;

	i2cSendRegister(synth + 0,   (P3 & 0x0000FF00) >> 8);
	i2cSendRegister(synth + 1,   (P3 & 0x000000FF));
	i2cSendRegister(synth + 2,   ((P1 & 0x00030000) >> 16) | rDiv);
	i2cSendRegister(synth + 3,   (P1 & 0x0000FF00) >> 8);
	i2cSendRegister(synth + 4,   (P1 & 0x000000FF));
	i2cSendRegister(synth + 5,   ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
	i2cSendRegister(synth + 6,   (P2 & 0x0000FF00) >> 8);
	i2cSendRegister(synth + 7,   (P2 & 0x000000FF));
}

void si5351aSetFrequency(uint32_t frequency)
{
	uint32_t pllFreq;
	uint32_t xtalFreq = XTAL_FREQ;
	uint32_t l;
	float f;
	uint8_t mult;
	uint32_t num;
	uint32_t denom;
	uint32_t divider;

	divider = 900000000UL / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal 
									// PLL frequency: 900MHz
	if (divider % 2) divider--;		// Ensure an even integer division ratio

	pllFreq = divider * frequency;	// Calculate the pllFrequency: the divider * desired output frequency

	mult = pllFreq / xtalFreq;		// Determine the multiplier to get to the required pllFrequency
	l = pllFreq % xtalFreq;			// It has three parts:
	f = l;							// mult is an integer that must be in the range 15..90
	f *= 1048575;					// num and denom are the fractional parts, the numerator and denominator
	f /= xtalFreq;					// each is 20 bits (range 0..1048575)
	num = (uint32_t)f;						// the actual multiplier is  mult + num / denom
	denom = 1048575;				// For simplicity we set the denominator to the maximum 1048575


									// Set up PLL A with the calculated multiplication ratio
	setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
									// Set up MultiSynth divider 0, with the calculated divider. 
									// The final R division stage can divide by a power of two, from 1..128. 
									// reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
									// If you want to output frequencies below 1MHz, you have to use the 
									// final R division stage
	setupMultisynth(SI_SYNTH_MS_0, divider, SI_R_DIV_1);
									// Reset the PLL. This causes a glitch in the output. For small changes to 
									// the parameters, you don't need to reset the PLL, and there is no glitch
	i2cSendRegister(SI_PLL_RESET, 0xA0);	
									// Finally switch on the CLK0 output (0x4F)
									// and set the MultiSynth0 input to be PLL A
	//i2cSendRegister(SI_CLK0_CONTROL, 0x4F | SI_CLK_SRC_PLL_A);
        i2cSendRegister(SI_CLK0_CONTROL, 0xCF | SI_CLK_SRC_PLL_A);
}
