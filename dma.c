#include <iostm8L151k6.h>
#include <stdint.h>
#include "config.h"
#include "dma.h"

#define ADC1_DR_BASE 0x5344
void dma_init(void) {
  CLK_PCKENR2_PCKEN24 = 1;
  
  DMA1_C0PARH = 0x53;      // source memory address
  DMA1_C0PARL = 0x44;
  
  DMA1_C0M0ARH = (uint16_t)(adc_data)>>8;     // destination
  DMA1_C0M0ARL = (uint8_t)(((uint16_t)adc_data) & 0xff);  

  DMA1_C0NDTR =ADC_DATA_SZ;
  
  DMA1_C0CR_CIRC = 1;   // circular mode
  DMA1_C0CR_MINCDEC = 1; // increment mode
  DMA1_C0SPR_TSIZE=1;   // 16-bit transfer
  DMA1_C0CR_TCIE = 0;   // full transaction interrupt disable
  
  DMA1_C0CR_EN = 1;     // channel enable
  DMA1_GCSR_GEN=1;      // DMA enable
}

void dma_deinit(void) {
  DMA1_GCSR_GEN=0;      // DMA disable
  DMA1_C0CR_EN = 0;     // channel disable
  DMA1_C0SPR_TSIZE=0;
  DMA1_C0CR_MINCDEC = 0;
  DMA1_C0CR_CIRC = 0;
  DMA1_C0NDTR =0;
  CLK_PCKENR2_PCKEN24 = 0;
}
