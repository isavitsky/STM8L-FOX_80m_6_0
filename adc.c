#include <iostm8L151k6.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "adc.h"

void isort(uint16_t *array, uint8_t n) {
    uint8_t i, j;
    uint16_t temp;
    
    //Iterate start from second element 
    for (i = 1; i < n; i++) {
        j = i;
        //Iterate and compare till it satisfies condition 
        while ( j > 0 && array[j] < array[j-1]) {
            //Swaping operation
            temp = array[j];
            array[j]   = array[j-1];
            array[j-1] = temp;
            j--;
        }
    }
}

uint16_t get_vref(void)
{
  uint16_t vref;

  CLK_PCKENR2_PCKEN20 = 1;
  ADC1_CR1_ADON = 1;
  /* delay ~3 us for 2 MHz clock */
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  ADC1_CR2_PRESC = 1; // Prescaler = clk/2
  ADC1_CR3_SMTP2 = 7; // Set 384 clock cycles per sample
  ADC1_SQR1_DMAOFF = 1; // DMA off
  ADC1_SQR1_CHSEL_S28 =1;
  ADC1_TRIGR1_VREFINTON = 1;
  ADC1_CR1_START=1;
  while (ADC1_SR_EOC == 0);
  vref = (ADC1_DRH << 8);
  vref |= ADC1_DRL;
  ADC1_SQR1_CHSEL_S28 =0;
  ADC1_TRIGR1_VREFINTON =0;
  ADC1_CR1_ADON =0;
  CLK_PCKENR2_PCKEN20 =0;
  return vref;
}

uint16_t get_vin(void)
{
  uint16_t vin;

  CLK_PCKENR2_PCKEN20 = 1;
  ADC1_CR1_ADON = 1;
  /* delay ~3 us for 2 MHz clock */
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  ADC1_CR2_PRESC = 1; // Prescaler = clk/2
  ADC1_CR3_SMTP2 = 7; // Set 384 clock cycles per sample
  ADC1_SQR1_DMAOFF = 1; // DMA off
  ADC1_SQR4_CHSEL_S1=1;
  ADC1_TRIGR4_TRIG1=1;
  ADC1_CR1_START=1;
  while (ADC1_SR_EOC == 0);
  vin = (ADC1_DRH << 8);
  vin |= ADC1_DRL;
  ADC1_SQR4_CHSEL_S1=0;
  ADC1_TRIGR4_TRIG1=0;
  ADC1_CR1_ADON =0;
  CLK_PCKENR2_PCKEN20 =0;
  return vin;
}

uint16_t get_vtemp(void)
{
  uint8_t i;
  uint16_t dts[TS_AR_SZ];
//  uint16_t vtemp;

  CLK_PCKENR2_PCKEN20 = 1;
  ADC1_CR1_ADON = 1;
  ADC1_TRIGR1_TSON = 1;
  for(i=0;i<50;i++) asm("nop"); //delay 25 us for TSON
  ADC1_CR2_PRESC = 1; // Prescaler = clk/2
  ADC1_CR3_SMTP2 = 7; // Set 384 clock cycles per sample
  ADC1_SQR1_DMAOFF = 1; // DMA off
  ADC1_SQR1_CHSEL_S29 =1; // select Temp Sensor
  for (i=0;i<TS_AR_SZ;i++)
  {
    ADC1_CR1_START=1;
    while (ADC1_SR_EOC == 0);
    dts[i] = (ADC1_DRH << 8);
    dts[i] |= ADC1_DRL;
//    vtemp = (ADC1_DRH << 8);
//    vtemp |= ADC1_DRL;
  }
  ADC1_SQR1_CHSEL_S29 =0;
  ADC1_TRIGR1_TSON =0;
  ADC1_CR1_ADON =0;
  CLK_PCKENR2_PCKEN20 =0;
  isort(dts, TS_AR_SZ);
  return dts[TS_AR_SZ/2];
//  return vtemp;
}

void start_adc(void)
{
  CLK_PCKENR2_PCKEN20 = 1;
  ADC1_CR1_ADON = 1;
  /* delay ~3 us for 2 MHz clock */
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  ADC1_CR2_PRESC = 1; // Prescaler = clk/2
  ADC1_CR2_SMTP1 = 7; // Set 384 clock cycles per sample
  ADC1_SQR1_DMAOFF = 1; // DMA off
}

void stop_adc(void)
{
  ADC1_CR1_ADON =0;
  CLK_PCKENR2_PCKEN20 =0;
}

uint16_t get_iant(void)
{
  uint16_t val;

  ADC1_SQR3_CHSEL_S9=1;
  ADC1_TRIGR3_TRIG9=1;
  ADC1_CR1_START=1;
  while (ADC1_SR_EOC == 0);
  val = (ADC1_DRH << 8);
  val |= ADC1_DRL;
  ADC1_SQR3_CHSEL_S9=0;
  ADC1_TRIGR3_TRIG9=0;
  return val;
}

void adc_init(void)
{
  __disable_interrupt();
  CLK_PCKENR2_PCKEN20 = 1;
  ADC1_CR1_ADON = 1;
  /* delay ~3 us for 2 MHz clock */
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  asm("nop");
  ADC1_CR2_PRESC = 1; // Prescaler = clk/2
  ADC1_CR2_SMTP1 = 7; // Set 384 clock cycles per sample
  ADC1_SQR1_DMAOFF = 0; // DMA on
  ADC1_CR1_CONT=1;      //CONTINUOUS MODE
  ADC1_CR3_CHSEL=1;     // CH1 for AWD 
  ADC1_LTRH=HIBYTE(lowbat_short);
  ADC1_LTRL=LOBYTE(lowbat_short);
  flag.low_bat = false;
  ADC1_SQR3_CHSEL_S9=1;
  ADC1_SQR4_CHSEL_S1=1;
  ADC1_TRIGR3_TRIG9=1;
  ADC1_TRIGR4_TRIG1=1;
  ADC1_CR1_AWDIE=1;     // ENABLE AWD INTR
  ADC1_CR1_START=1;
  __enable_interrupt();
}

void adc_deinit(void)
{
  ADC1_CR1_START=0; // stop
  ADC1_CR1_AWDIE=0;
  ADC1_TRIGR3_TRIG9=0;
  ADC1_TRIGR4_TRIG1=0;
  ADC1_SQR3_CHSEL_S9=0;
  ADC1_SQR4_CHSEL_S1=0;
  ADC1_CR1_CONT=0;
  ADC1_CR3_CHSEL=0;
  ADC1_CR2_SMTP1 = 0;
  ADC1_CR2_PRESC = 0;
  ADC1_CR1_ADON = 0;
  CLK_PCKENR2_PCKEN20 = 0;
}

