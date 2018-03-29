// uart.h
#ifndef UART_H
#define UART_H

#include <stdint.h>

//-------------- Declaration of function prototypes -----------------
void uart_init(void);
void uart_stop(void);
void uart_send_char(uint8_t c);
char uart_get_char(void);
void uart_putsn(const char *, uint8_t);
void uart_puts(const char *);

#endif
