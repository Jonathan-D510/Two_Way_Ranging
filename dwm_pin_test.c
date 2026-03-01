#include "main.h"
#include "uart.h"
#include "deca_usleep.h"
#include "dwm_diag.h"

void dwm_pin_test_once(void)
{
    dbg_print("PINTEST: CS high->low pulses...\r\n");

    for (int i = 0; i < 5; i++)
    {
        HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET);
        deca_usleep(200);
        HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_RESET);
        deca_usleep(200);
        HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET);
        deca_usleep(200);
    }

    dbg_print("PINTEST: raw devid...\r\n");
    uint32_t id = 0;
    int r = dwm_raw_read_devid(&id);
    dbg_printf("PINTEST: raw ret=%d id=0x%08lX\r\n", r, (unsigned long)id);
}
