#include "dwm_port.h"
#include "main.h"
#include "deca_device_api.h"
#include "deca_usleep.h"

/* If you want prints here, include uart.h, but keep it minimal for bring-up */
/* #include "uart.h" */

void dw3000_port_init(void)
{
    /* Safe default states */
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET);

    /* EXTON high enables module power on many boards */
    HAL_GPIO_WritePin(DWM_EXTON_GPIO_Port, DWM_EXTON_Pin, GPIO_PIN_SET);

    /* WAKEUP idle low */
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);

    /* RST idle high */
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_SET);

    deca_usleep(2000);
}

void dw3000_hard_reset(void)
{
    /* Active low reset */
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_RESET);
    deca_usleep(2000);
    HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_SET);
    deca_usleep(5000);
}

void dw3000_wakeup_pulse(void)
{
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_SET);
    deca_usleep(1000);
    HAL_GPIO_WritePin(DWM_WAKEUP_GPIO_Port, DWM_WAKEUP_Pin, GPIO_PIN_RESET);
    deca_usleep(1000);
}

void dw3000_on_exti(uint16_t gpio_pin)
{
    if (gpio_pin == DWM_IRQ_Pin)
    {
        dwt_isr();
    }
}

/* HAL calls this automatically from stm32g4xx_it.c */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    dw3000_on_exti(GPIO_Pin);
}
