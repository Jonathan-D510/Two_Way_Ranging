#pragma once
#include <stdint.h>
#include "deca_device_api.h"

/*
 * DW3xxx compatibility helpers:
 * - Some older examples use dwt_read32bitreg/dwt_write32bitreg and “*_timestamp64” helpers.
 * - DW3000 API primarily uses read_reg/write_reg and 40-bit timestamps.
 */

#ifndef dwt_read32bitreg
#define dwt_read32bitreg(addr)        dwt_read_reg((addr))
#endif

#ifndef dwt_write32bitreg
#define dwt_write32bitreg(addr, val)  dwt_write_reg((addr), (val))
#endif

/* DW3xxx provides a mask for “all RX errors”, not a single RXERR bit. */
#ifndef DWT_INT_RXERR_BIT_MASK
#define DWT_INT_RXERR_BIT_MASK        (SYS_STATUS_ALL_RX_ERR)
#endif

static inline uint64_t uwb_get_u40(const uint8_t *p)
{
    return  ((uint64_t)p[0])        |
            ((uint64_t)p[1] << 8)   |
            ((uint64_t)p[2] << 16)  |
            ((uint64_t)p[3] << 24)  |
            ((uint64_t)p[4] << 32);
}

static inline void uwb_put_u40(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
}

/* Read 40-bit TX timestamp as uint64. */
static inline uint64_t dwt_readtxtimestamp64(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    return uwb_get_u40(ts);
}

/* Read 40-bit RX timestamp as uint64 (IPATOV path is correct for non-STS ranging). */
static inline uint64_t dwt_readrxtimestamp64(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp_ipatov(ts);
    return uwb_get_u40(ts);
}
