#pragma once
#include <stdint.h>
#include "deca_device_api.h"

/* If your code is using the old names, map them to DW3xxx API. */
#ifndef dwt_read32bitreg
#define dwt_read32bitreg(addr)        dwt_read_reg((addr))
#endif

#ifndef dwt_write32bitreg
#define dwt_write32bitreg(addr, val)  dwt_write_reg((addr), (val))
#endif

/* Old code expects a generic RX error bit; DW3xxx provides a mask of all RX errors. */
#ifndef DWT_INT_RXERR_BIT_MASK
#define DWT_INT_RXERR_BIT_MASK        (SYS_STATUS_ALL_RX_ERR)
#endif

/* Convert a 40-bit (5-byte) DW timestamp to uint64_t. */
static inline uint64_t dw_ts40_to_u64(const uint8_t ts[5])
{
    return  ((uint64_t)ts[0])        |
            ((uint64_t)ts[1] << 8)   |
            ((uint64_t)ts[2] << 16)  |
            ((uint64_t)ts[3] << 24)  |
            ((uint64_t)ts[4] << 32);
}

/* DW3xxx API: TX timestamp is read into 5-byte buffer. */
static inline uint64_t dwt_readtxtimestamp64(void)
{
    uint8_t ts[5];
    dwt_readtxtimestamp(ts);
    return dw_ts40_to_u64(ts);
}

/* For “normal” ranging without STS segmentation, use the Ipatov RX timestamp helper. */
static inline uint64_t dwt_readrxtimestamp64(void)
{
    uint8_t ts[5];
    dwt_readrxtimestamp_ipatov(ts);
    return dw_ts40_to_u64(ts);
}
