#include <iostm8L151k6.h>
#include "uart.h"
#include "config.h"

void uart_init(void)
{
  // Configure PC2 (USART1_RX) for input
  PC_DDR_DDR2 = 0;      // in
  PC_CR1_C12 = 0;       // float
  PC_CR2_C22 = 0;       // no extint

  // Configure PCC (USART1_TX) for output
  PC_DDR_DDR3 = 1; // out
  PC_CR1_C13 = 1; // pp
  PC_CR2_C23 = 1; // fast
  PC_ODR_ODR3 = 0; // high

  CLK_PCKENR1_PCKEN15 = 1;      // Enable clock to USART1
  // Set speed and 8,N,1 mode
  USART1_CR1_M = 0;     // 8 bits
  USART1_CR1_PIEN = 0;  // Disable PARITY ERROR interrupt
  USART1_CR2_TEN = 1;   // TX enable
  USART1_CR2_REN = 1;   // RX enable
  USART1_CR2_RIEN = 1;  // Enable RX interrupt
  USART1_CR2_ILIEN = 1; // Enable IDLE LINE interrupt
  USART1_CR3_CLKEN = 0; // Sync clock disable
  USART1_CR3_STOP0 = 0; // 1 stop bit
  USART1_CR3_STOP1 = 0; //  --/--

  USART1_BRR2 = (char)(((UART_DIV & 0xF000)>> 8) + (UART_DIV & 0x000F));  // set
  USART1_BRR1 = (char)((UART_DIV & 0x0FF0) >> 4);                         // baudrate
}

void uart_stop(void)
{
  while (USART1_SR_TC == 0); // disable only after tx complete
  USART1_CR2_TEN = 0;   // TX disable
  USART1_CR2_REN = 0;   // RX disable
  USART1_CR2_RIEN = 0;  // Disable RX interrupt
  CLK_PCKENR1_PCKEN15 = 0;      // Disable clock to USART1
  
  // Configure PC2 (USART1_RX) for output
  PC_DDR_DDR2 = 1;      // in
  PC_CR1_C12 = 0;       // open drain
  PC_CR1_C14 = 0;       // open drain
}

void uart_send_char(uint8_t c)
{
  while (USART1_SR_TXE == 0);
  USART1_DR = c;
}

void uart_putsn(const char *str, uint8_t len)
{
  for(uint8_t i=0; i<len;i++) uart_send_char(str[i]);
}

void uart_puts(const char *str)
{
  while (str && *str)
    uart_send_char(*str++);
}

char uart_get_char(void)
{
  while(USART1_SR_RXNE == 0); // Wait until char is available
  return (USART1_DR);       // Get it
}
