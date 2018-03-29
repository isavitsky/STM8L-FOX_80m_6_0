// adc.h
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define TS_AR_SZ        19      /* Temp Sensor values array size */
//-------------- Declaration of function prototypes -----------------
uint16_t get_vref(void);
uint16_t get_vin(void);
uint16_t get_vtemp(void);
uint16_t get_iant(void);
void start_adc(void);
void stop_idc(void);
void adc_init(void);
void adc_deinit(void);

#endif
