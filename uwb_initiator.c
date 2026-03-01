#include "uwb_initiator.h"
#include "uwb_common.h"
#include "deca_device_api.h"
#include "uart.h"
#include <string.h>

static uint8_t g_seq = 0;
static uint8_t rx_buf[32];
static uwb_frame_t tx_poll;

static inline void clear_events(uint32_t m)
{
    dwt_writesysstatuslo(m);
}

void uwb_initiator_init(void)
{
    dwt_forcetrxoff();

    /* Antenna delays: keep 0 for now */
    dwt_setrxantennadelay(0);
    dwt_settxantennadelay(0);

    dbg_print("INIT: ready\r\n");
}

int uwb_send_poll_wait_resp(uint8_t target_id, uint8_t *got_id, uint8_t *got_seq)
{
    g_seq++;

    tx_poll.m0 = UWB_POLL_MAGIC0;
    tx_poll.m1 = UWB_POLL_MAGIC1;
    tx_poll.m2 = UWB_POLL_MAGIC2;
    tx_poll.m3 = UWB_POLL_MAGIC3;
    tx_poll.id = target_id;
    tx_poll.seq = g_seq;
    tx_poll.rsv0 = 0;
    tx_poll.rsv1 = 0;

    dwt_forcetrxoff();

    /* Clear old events */
    clear_events(0xFFFFFFFFu);

    /* TX poll */
    dwt_writetxdata(UWB_FRAME_LEN, (uint8_t*)&tx_poll, 0);
    dwt_writetxfctrl(UWB_FRAME_LEN, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    /* Wait TX done */
    uint32_t s;
    do {
        s = dwt_readsysstatuslo();
    } while ((s & DWT_INT_TXFRS_BIT_MASK) == 0);

    clear_events(DWT_INT_TXFRS_BIT_MASK);

    /* Enable RX */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Wait for RX good or known RX error types */
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 100)
    {
        s = dwt_readsysstatuslo();

        if (s & DWT_INT_RXFCG_BIT_MASK) break;

        /* These masks exist in your build (RXERR doesn't) */
        if (s & (DWT_INT_RXFTO_BIT_MASK |
                 DWT_INT_RXPHE_BIT_MASK |
                 DWT_INT_RXFCE_BIT_MASK))
        {
            clear_events(DWT_INT_RXFTO_BIT_MASK |
                         DWT_INT_RXPHE_BIT_MASK |
                         DWT_INT_RXFCE_BIT_MASK);
            return -1;
        }
    }

    s = dwt_readsysstatuslo();
    if ((s & DWT_INT_RXFCG_BIT_MASK) == 0)
    {
        return -2; /* timeout waiting for frame */
    }

    clear_events(DWT_INT_RXFCG_BIT_MASK);

    uint8_t rng = 0;
    uint16_t len = dwt_getframelength(&rng);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);

    dwt_readrxdata(rx_buf, len, 0);

    if (len < UWB_FRAME_LEN) return -3;

    uwb_frame_t *resp = (uwb_frame_t*)rx_buf;
    if (resp->m0 != UWB_RESP_MAGIC0 || resp->m1 != UWB_RESP_MAGIC1 ||
        resp->m2 != UWB_RESP_MAGIC2 || resp->m3 != UWB_RESP_MAGIC3)
    {
        return -4;
    }

    if (got_id)  *got_id  = resp->id;
    if (got_seq) *got_seq = resp->seq;

    if (resp->id == target_id && resp->seq == g_seq) return 0;

    return -5;
}
