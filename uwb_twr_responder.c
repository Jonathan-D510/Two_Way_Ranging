#include "uwb_twr_responder.h"
#include "deca_device_api.h"
#include "uart.h"
#include <string.h>
#include <stdint.h>

#define FCODE_POLL   0xE0
#define FCODE_RESP   0xE1

#define RESP_DELAY_US   10000U

static uint8_t g_my_id = 0;
static uint32_t g_fail_count = 0;

static void put_u40(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
}

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

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);
    return get_u40(ts);
}

static uint64_t us_to_dtu(uint32_t us)
{
    double dtu_per_s = 1.0 / DWT_TIME_UNITS;
    double dtu = ((double)us) * 1e-6 * dtu_per_s;
    return (uint64_t)(dtu + 0.5);
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

void uwb_twr_responder_init(uint8_t my_id)
{
    g_my_id = my_id;
    g_fail_count = 0;

    dwt_forcetrxoff();
    clear_status_all();

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dbg_printf("TWR_RESP: init my_id=%u\r\n", (unsigned)g_my_id);
}

void uwb_twr_responder_step(void)
{
    uint32_t st = dwt_readsysstatuslo();

    if (st & rx_err_mask_lo())
    {
        dwt_writesysstatuslo(rx_err_mask_lo());
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    if ((st & DWT_INT_RXFCG_BIT_MASK) == 0)
        return;

    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    uint8_t rxbuf[64];
    uint16_t len = dwt_getframelength(NULL);
    if (len > sizeof(rxbuf)) len = sizeof(rxbuf);
    dwt_readrxdata(rxbuf, len, 0);

    if (len < 4 || rxbuf[0] != FCODE_POLL)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint8_t src = rxbuf[1];
    uint8_t dst = rxbuf[2];
    uint8_t seq = rxbuf[3];

    if (dst != g_my_id)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint64_t t2 = read_rx_ts40();

    /* Compute t3 and ALIGN to 512 DTU boundary (clear lower 9 bits) */
    uint64_t t3 = t2 + us_to_dtu(RESP_DELAY_US);
    t3 &= ~((uint64_t)0x1FF); /* align */

    dwt_setdelayedtrxtime((uint32_t)(t3 >> 8));

    uint8_t resp[4 + 5 + 5];
    resp[0] = FCODE_RESP;
    resp[1] = g_my_id;
    resp[2] = src;
    resp[3] = seq;
    put_u40(&resp[4], t2);
    put_u40(&resp[9], t3);

    dwt_forcetrxoff();
    clear_status_all();

    if (dwt_writetxdata(sizeof(resp), resp, 0) != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    dwt_writetxfctrl((uint16_t)(sizeof(resp) + 2U), 0, 0);

    int32_t rc = dwt_starttx(DWT_START_TX_DELAYED);
    if (rc != DWT_SUCCESS)
    {
        g_fail_count++;
        if ((g_fail_count % 50u) == 1u)
            dbg_printf("RESP: delayed TX fail rc=%ld\r\n", (long)rc);

        dwt_forcetrxoff();
        clear_status_all();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    while ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) == 0) { }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
