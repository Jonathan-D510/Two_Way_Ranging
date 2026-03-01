#include "stm32g4xx.h"

static void fault_uart2_putc(char c)
{
    while ((USART2->ISR & USART_ISR_TXE) == 0) { }
    USART2->TDR = (uint8_t)c;
}

static void fault_uart2_puts(const char *s)
{
    while (*s) fault_uart2_putc(*s++);
}

/* Call this from the Cube HardFault_Handler */
void fault_print_hardfault(void)
{
    fault_uart2_puts("\r\n*** HARDFAULT ***\r\n");
}
