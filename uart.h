#pragma once
#include "stm32g4xx_hal.h"

void dbg_init(UART_HandleTypeDef *huart);
void dbg_print(const char *s);
void dbg_printf(const char *fmt, ...);
