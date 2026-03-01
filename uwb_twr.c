#include "uwb_twr.h"
#include "deca_device_api.h"
#include "uart.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define FCODE_POLL   0xE0
#define FCODE_RESP   0xE1

#define SPEED_OF_LIGHT 299702547.0f

static uint8_t  g_my_id = 0;
static uint8_t  g_seq   = 0;

static uint64_t get_u40(const uint8_t *p)
{
    uint64_t v = 0;
    v |= (uint64_t)p[0];
    v |= (uint64_t)p[1] << 8;
    v |= (uint64_t)p[2] << 16;
    v |= (uint64_t)p[3] << 24;
    v |= (uint64_t)p[4] << 32;
    return v;
}

static uint64_t read_tx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    return get_u40(ts);
}

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return get_u40(ts);
}

static void clear_status_all(void)
{
    dwt_writesysstatuslo(0xFFFFFFFFu);
    dwt_writesysstatushi(0xFFFFFFFFu);
}

static uint32_t rx_err_mask_lo(void)
{
    return (DWT_INT_RXPHE_BIT_MASK |
            DWT_INT_RXFCE_BIT_MASK |
            DWT_INT_RXFSL_BIT_MASK |
            DWT_INT_RXFTO_BIT_MASK |
            DWT_INT_RXPTO_BIT_MASK |
            DWT_INT_RXSTO_BIT_MASK |
            DWT_INT_ARFE_BIT_MASK  |
            DWT_INT_CIAERR_BIT_MASK);
}

static void print_float_bits(const char *tag, float x)
{
    union { float f; uint32_t u; } v;
    v.f = x;
    dbg_printf("%s: f=%.6f bits=0x%08lX\r\n", tag, (double)x, (unsigned long)v.u);
}

void uwb_twr_init(uint8_t my_id)
{
    g_my_id = my_id;
    g_seq = 0;

    dwt_forcetrxoff();
    clear_status_all();

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    dbg_printf("TWR: init my_id=%u\r\n", (unsigned)g_my_id);
}

void uwb_twr_initi(uint8_t my_id)
{
    uwb_twr_init(my_id);
}

int uwb_twr_range_to(uint8_t target_id, float *out_m)
{
    static uint8_t poll[4];
    uint8_t rxbuf[64];

    poll[0] = FCODE_POLL;
    poll[1] = g_my_id;
    poll[2] = target_id;
    poll[3] = g_seq;

    dbg_printf("TWR: TX POLL bytes=%02X %02X %02X %02X\r\n",
               poll[0], poll[1], poll[2], poll[3]);

    g_seq++;

    dwt_forcetrxoff();
    clear_status_all();

    if (dwt_writetxdata(sizeof(poll), poll, 0) != DWT_SUCCESS)
        return -11;

    /* txFrameLength includes 2-byte CRC */
    dwt_writetxfctrl((uint16_t)(sizeof(poll) + 2U), 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
        return -10;

    while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) { }

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK |
                         DWT_INT_TXPHS_BIT_MASK |
                         DWT_INT_TXPRS_BIT_MASK |
                         DWT_INT_TXFRB_BIT_MASK);

    uint64_t t1 = read_tx_ts40();

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t t0 = HAL_GetTick();
    for (;;)
    {
        uint32_t st_lo = dwt_readsysstatuslo();
        uint32_t st_hi = dwt_readsysstatushi();

        if (st_lo & DWT_INT_RXFCG_BIT_MASK)
            break;

        if (st_lo & rx_err_mask_lo())
        {
            dwt_writesysstatuslo(rx_err_mask_lo());
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if ((HAL_GetTick() - t0) > 200)
        {
            dwt_forcetrxoff();
            clear_status_all();
            return -21;
        }
    }

    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    uint16_t len = dwt_getframelength(NULL);
    if (len > sizeof(rxbuf)) len = sizeof(rxbuf);
    dwt_readrxdata(rxbuf, len, 0);

    uint64_t t4 = read_rx_ts40();

    /* Expect 4+5+5 payload bytes (CRC already stripped by readrxdata length? depends on driver)
       We'll just require at least 14 payload bytes. */
    if (len < (4 + 5 + 5))
        return -30;

    if (rxbuf[0] != FCODE_RESP)
        return -31;

    if (rxbuf[1] != target_id || rxbuf[2] != g_my_id || rxbuf[3] != (uint8_t)(g_seq - 1))
        return -32;

    uint64_t t2 = get_u40(&rxbuf[4]);
    uint64_t t3 = get_u40(&rxbuf[9]);

    int64_t Tround = (int64_t)(t4 - t1);
    int64_t Treply = (int64_t)(t3 - t2);

    double tof_dtu = ((double)Tround - (double)Treply) * 0.5;
    double tof_s   = tof_dtu * (double)DWT_TIME_UNITS;

    float dist_m = (float)(tof_s * (double)SPEED_OF_LIGHT);
    if (out_m) *out_m = dist_m;

    /* If float printing shows blank, it’s likely NaN.
       Print bits so we can see it. */
    if (!isfinite(dist_m))
    {
        dbg_printf("TWR: dist not finite! Tround=%ld Treply=%ld t1=%lu t2=%lu t3=%lu t4=%lu\r\n",
                   (long)Tround, (long)Treply,
                   (unsigned long)t1, (unsigned long)t2, (unsigned long)t3, (unsigned long)t4);
        print_float_bits("TWR: dist", dist_m);
        return -40;
    }

    return 0;
}
