#pragma once
#include "stm32g4xx_hal.h"
#include <stdint.h>

void dw3000_port_init(void);
void dw3000_hard_reset(void);
void dw3000_wakeup_pulse(void);
void dw3000_on_exti(uint16_t gpio_pin);
