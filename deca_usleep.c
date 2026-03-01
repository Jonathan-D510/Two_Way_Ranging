#include "deca_usleep.h"
#include "stm32g4xx_hal.h"

/*
 * Microsecond delay using DWT->CYCCNT.
 * Works well on Cortex-M4 (STM32G431).
 */

static uint8_t dwt_inited = 0;

static void dwt_init_once(void)
{
    if (dwt_inited) return;
    dwt_inited = 1;

    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void deca_usleep(uint32_t time_us)
{
    if (time_us == 0) return;

    /* For long delays, split (keeps CYCCNT math safe) */
    if (time_us >= 1000U)
    {
        /* Do millisecond part with HAL_Delay */
        uint32_t ms = time_us / 1000U;
        uint32_t rem_us = time_us % 1000U;
        HAL_Delay(ms);
        time_us = rem_us;
        if (time_us == 0) return;
    }

    dwt_init_once();

    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * time_us;

    /* Wait until elapsed cycles reached */
    while ((DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}
