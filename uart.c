#include "uart.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *g_huart = NULL;

void dbg_init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
}

void dbg_print(const char *s)
{
    if (!g_huart || !s) return;
    HAL_UART_Transmit(g_huart, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

void dbg_printf(const char *fmt, ...)
{
    if (!g_huart || !fmt) return;

    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    HAL_UART_Transmit(g_huart, (uint8_t*)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}
