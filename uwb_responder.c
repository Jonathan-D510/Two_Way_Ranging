#include "uwb_responder.h"
#include "uwb_common.h"
#include "deca_device_api.h"
#include "uart.h"
#include <string.h>

static uint8_t g_my_id = 0;
static uint8_t rx_buf[32];
static uwb_frame_t tx_resp;

static inline void clear_events(uint32_t m)
{
    dwt_writesysstatuslo(m);
}

void uwb_responder_init(uint8_t my_id)
{
    g_my_id = my_id;

    dwt_forcetrxoff();

    dwt_setrxantennadelay(0);
    dwt_settxantennadelay(0);

    dbg_printf("RESP: init my_id=%u\r\n", g_my_id);
}

void uwb_responder_step(void)
{
    dwt_forcetrxoff();
    clear_events(0xFFFFFFFFu);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t s;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < 50)
    {
        s = dwt_readsysstatuslo();

        if (s & DWT_INT_RXFCG_BIT_MASK) break;

        if (s & (DWT_INT_RXFTO_BIT_MASK |
                 DWT_INT_RXPHE_BIT_MASK |
                 DWT_INT_RXFCE_BIT_MASK))
        {
            clear_events(DWT_INT_RXFTO_BIT_MASK |
                         DWT_INT_RXPHE_BIT_MASK |
                         DWT_INT_RXFCE_BIT_MASK);
            return;
        }
    }

    s = dwt_readsysstatuslo();
    if ((s & DWT_INT_RXFCG_BIT_MASK) == 0) return;

    clear_events(DWT_INT_RXFCG_BIT_MASK);

    uint8_t rng = 0;
    uint16_t len = dwt_getframelength(&rng);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);

    dwt_readrxdata(rx_buf, len, 0);

    if (len < UWB_FRAME_LEN) return;

    uwb_frame_t *poll = (uwb_frame_t*)rx_buf;

    if (poll->m0 != UWB_POLL_MAGIC0 || poll->m1 != UWB_POLL_MAGIC1 ||
        poll->m2 != UWB_POLL_MAGIC2 || poll->m3 != UWB_POLL_MAGIC3)
    {
        return;
    }

    if (poll->id != g_my_id) return;

    tx_resp.m0 = UWB_RESP_MAGIC0;
    tx_resp.m1 = UWB_RESP_MAGIC1;
    tx_resp.m2 = UWB_RESP_MAGIC2;
    tx_resp.m3 = UWB_RESP_MAGIC3;
    tx_resp.id = g_my_id;
    tx_resp.seq = poll->seq;
    tx_resp.rsv0 = 0;
    tx_resp.rsv1 = 0;

    dwt_forcetrxoff();
    clear_events(0xFFFFFFFFu);

    dwt_writetxdata(UWB_FRAME_LEN, (uint8_t*)&tx_resp, 0);
    dwt_writetxfctrl(UWB_FRAME_LEN, 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    do {
        s = dwt_readsysstatuslo();
    } while ((s & DWT_INT_TXFRS_BIT_MASK) == 0);

    clear_events(DWT_INT_TXFRS_BIT_MASK);

    dbg_printf("RESP: replied id=%u seq=%u\r\n", g_my_id, tx_resp.seq);
}
