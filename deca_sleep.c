#include "stm32g4xx_hal.h"
#include "deca_device_api.h"

void deca_sleep(unsigned int time_ms)
{
    HAL_Delay((uint32_t)time_ms);
}
