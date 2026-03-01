#include "dwm_spi_if.h"
#include "deca_interface.h"
#include "deca_spi.h"
#include "stm32g4xx_hal.h"
#include "main.h"

/* Pull in the SPI handle from CubeMX */
extern SPI_HandleTypeDef hspi1;

static void spi_apply_prescaler(uint32_t presc)
{
    HAL_SPI_DeInit(&hspi1);
    hspi1.Init.BaudRatePrescaler = presc;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    (void)HAL_SPI_Init(&hspi1);
}

static int32_t dwm_readfromspi_wrap(uint16_t headerLength,
                                   uint8_t *headerBuffer,
                                   uint16_t readLength,
                                   uint8_t *readBuffer)
{
    return (int32_t)readfromspi(headerLength,
                               (const uint8_t*)headerBuffer,
                               (uint32_t)readLength,
                               readBuffer);
}

static int32_t dwm_writetospi_wrap(uint16_t headerLength,
                                  const uint8_t *headerBuffer,
                                  uint16_t bodyLength,
                                  const uint8_t *bodyBuffer)
{
    return (int32_t)writetospi(headerLength,
                              headerBuffer,
                              (uint32_t)bodyLength,
                              bodyBuffer);
}

/* Driver calls these around init / after init */
static void spi_set_slow(void) { spi_apply_prescaler(SPI_BAUDRATEPRESCALER_64); }
static void spi_set_fast(void) { spi_apply_prescaler(SPI_BAUDRATEPRESCALER_4);  }

static struct dwt_spi_s g_spi_if =
{
    .readfromspi = dwm_readfromspi_wrap,
    .writetospi  = dwm_writetospi_wrap,
    .setslowrate = spi_set_slow,
    .setfastrate = spi_set_fast,
};

void *dwm_get_spi_if(void)
{
    return (void *)&g_spi_if;
}
