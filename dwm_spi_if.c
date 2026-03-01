#include "dwm_spi_if.h"
#include "deca_interface.h"   /* struct dwt_spi_s */
#include "deca_spi.h"

/* Wrappers to match the signatures expected by this driver build */

static int32_t dwm_readfromspi_wrap(uint16_t headerLength,
                                   uint8_t *headerBuffer,
                                   uint16_t readLength,
                                   uint8_t *readBuffer)
{
    /* Cast headerBuffer to const because your deca_spi takes const */
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

/* Optional SPI rate hooks. Keep as no-ops for now. */
static void spi_set_slow(void) { }
static void spi_set_fast(void) { }

/* Must match your deca_interface.h field names (these appear to be correct for your build) */
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
