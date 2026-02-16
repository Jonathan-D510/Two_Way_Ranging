#include "main.h"
#include "stm32g4xx_hal.h"
#include "dwm_port.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

#ifndef DEBUG_UART
#define DEBUG_UART 1
#endif

void dbg(const char *s)
{
#if DEBUG_UART
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
#else
    (void)s;
#endif
}

#define DWM_CS_PORT    DWM_CS_GPIO_Port
#define DWM_CS_PIN     DWM_CS_Pin
#define DWM_RST_PORT   DWM_RST_GPIO_Port
#define DWM_RST_PIN    DWM_RST_Pin
#define DWM_WAKE_PORT  DWM_WAKEUP_GPIO_Port
#define DWM_WAKE_PIN   DWM_WAKEUP_Pin

/*
 * IMPORTANT:
 * Your logs show dwt_readdevid() becomes WRONG after "spi fast".
 * So "fast" must be conservative. Start with prescaler 16 (safe),
 * and only later try 8/4 once everything is stable.
 */
#define DWM_SPI_SLOW   SPI_BAUDRATEPRESCALER_16
#define DWM_SPI_FAST   SPI_BAUDRATEPRESCALER_16   // <-- keep SAME for now (stability first)

static inline void delay_cycles(volatile int n)
{
    while (n--) { __asm volatile ("nop"); }
}

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(DWM_CS_PORT, DWM_CS_PIN, GPIO_PIN_RESET);
    delay_cycles(200); // CS setup
}

static inline void cs_high(void)
{
    delay_cycles(200); // CS hold
    HAL_GPIO_WritePin(DWM_CS_PORT, DWM_CS_PIN, GPIO_PIN_SET);
    delay_cycles(200);
}

static void spi_set(uint32_t prescaler)
{
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.BaudRatePrescaler = prescaler;
    (void)HAL_SPI_Init(&hspi1);
}

void dwm_set_spi_slow(void) { spi_set(DWM_SPI_SLOW); }
void dwm_set_spi_fast(void) { spi_set(DWM_SPI_FAST); }

void dwm_port_init(void)
{
    HAL_GPIO_WritePin(DWM_CS_PORT,   DWM_CS_PIN,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(DWM_RST_PORT,  DWM_RST_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(DWM_WAKE_PORT, DWM_WAKE_PIN, GPIO_PIN_SET);
}

void dwm_reset_device(void)
{
    dbg("RESET: start\r\n");

    HAL_GPIO_WritePin(DWM_WAKE_PORT, DWM_WAKE_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(DWM_RST_PORT, DWM_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(DWM_RST_PORT, DWM_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);

    dbg("RESET: done\r\n");
}

void dwm_wakeup_io(void)
{
    HAL_GPIO_WritePin(DWM_WAKE_PORT, DWM_WAKE_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(DWM_WAKE_PORT, DWM_WAKE_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
}

#define SPI_TO  50

int dwm_spi_write(const uint8_t* hdr, uint16_t hlen,
                  const uint8_t* data, uint32_t dlen)
{
    cs_low();

    if (HAL_SPI_Transmit(&hspi1, (uint8_t*)hdr, hlen, SPI_TO) != HAL_OK)
    {
        cs_high();
        dbg("SPI TX hdr fail\r\n");
        return -1;
    }

    if (data && dlen)
    {
        if (HAL_SPI_Transmit(&hspi1, (uint8_t*)data, dlen, SPI_TO) != HAL_OK)
        {
            cs_high();
            dbg("SPI TX data fail\r\n");
            return -2;
        }
    }

    cs_high();
    return 0;
}

int dwm_spi_read(const uint8_t* hdr, uint16_t hlen,
                 uint8_t* data, uint32_t dlen)
{
    cs_low();

    if (HAL_SPI_Transmit(&hspi1, (uint8_t*)hdr, hlen, SPI_TO) != HAL_OK)
    {
        cs_high();
        dbg("SPI TX(read hdr) fail\r\n");
        return -1;
    }

    if (data && dlen)
    {
        if (HAL_SPI_Receive(&hspi1, data, dlen, SPI_TO) != HAL_OK)
        {
            cs_high();
            dbg("SPI RX fail\r\n");
            return -2;
        }
    }

    cs_high();
    return 0;
}
