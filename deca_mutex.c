#include "stm32g4xx_hal.h"
#include "deca_device_api.h"

decaIrqStatus_t decamutexon(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return (decaIrqStatus_t)primask;
}

void decamutexoff(decaIrqStatus_t s)
{
    if (((uint32_t)s) == 0U)
    {
        __enable_irq();
    }
}
