#include "dwm_diag.h"
#include "deca_spi.h"

/*
 * DW3xxx SPI header format is compatible with DW1000-style access for basic regs:
 * - DEV_ID register is register ID 0x00, no subaddress, 4 bytes.
 * - For a READ of reg 0x00, header is 1 byte: 0x00
 * This should return something like 0xDECA0302 if wiring/mode are correct.
 */
int dwm_raw_read_devid(uint32_t *out_devid)
{
    if (!out_devid) return -1;

    uint8_t hdr[1] = { 0x00 };     /* REG_ID = 0, READ */
    uint8_t b[4]   = { 0 };

    int r = readfromspi(1, hdr, 4, b);
    if (r != 0) return r;

    /* Little-endian assembly */
    uint32_t v = (uint32_t)b[0]
               | ((uint32_t)b[1] << 8)
               | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 24);

    *out_devid = v;
    return 0;
}
