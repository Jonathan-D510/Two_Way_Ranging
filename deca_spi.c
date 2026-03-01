#include "stm32g4xx_hal.h"
#include "main.h"
#include "deca_spi.h"
#include "uart.h"

extern SPI_HandleTypeDef hspi1;

#define DWM_SPI_TIMEOUT_MS   (100U)

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(DWM_CS_GPIO_Port, DWM_CS_Pin, GPIO_PIN_SET);
}

static void spi_recover(const char *tag, HAL_StatusTypeDef st)
{
    uint32_t err = HAL_SPI_GetError(&hspi1);
    dbg_printf("SPI %s fail: st=%d err=0x%08lX state=%d\r\n",
               tag, (int)st, (unsigned long)err, (int)hspi1.State);

    (void)HAL_SPI_Abort(&hspi1);
    (void)HAL_SPI_DeInit(&hspi1);
    (void)HAL_SPI_Init(&hspi1);
}

static int spi_tx(const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, DWM_SPI_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        spi_recover("TX", st);
        return -1;
    }
    return 0;
}

static int spi_rx(uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_SPI_Receive(&hspi1, buf, len, DWM_SPI_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        spi_recover("RX", st);
        return -1;
    }
    return 0;
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
               uint32_t bodyLength, const uint8_t *bodyBuffer)
{
    cs_low();

    if (headerLength && headerBuffer)
    {
        if (spi_tx(headerBuffer, headerLength) != 0)
        {
            cs_high();
            return -1;
        }
    }

    if (bodyLength && bodyBuffer)
    {
        while (bodyLength)
        {
            uint16_t chunk = (bodyLength > 0xFFFFU) ? 0xFFFFU : (uint16_t)bodyLength;
            if (spi_tx(bodyBuffer, chunk) != 0)
            {
                cs_high();
                return -2;
            }
            bodyBuffer += chunk;
            bodyLength -= chunk;
        }
    }

    cs_high();
    return 0;
}

int readfromspi(uint16_t headerLength, const uint8_t *headerBuffer,
                uint32_t bodyLength, uint8_t *bodyBuffer)
{
    cs_low();

    if (headerLength && headerBuffer)
    {
        if (spi_tx(headerBuffer, headerLength) != 0)
        {
            cs_high();
            return -1;
        }
    }

    if (bodyLength && bodyBuffer)
    {
        while (bodyLength)
        {
            uint16_t chunk = (bodyLength > 0xFFFFU) ? 0xFFFFU : (uint16_t)bodyLength;
            if (spi_rx(bodyBuffer, chunk) != 0)
            {
                cs_high();
                return -2;
            }
            bodyBuffer += chunk;
            bodyLength -= chunk;
        }
    }

    cs_high();
    return 0;
}
