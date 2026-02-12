//twr pong
// G431_DWM_Responder/Core/Src/twr_pong.c

#include "twr_pong.h"
#include "deca_device_api.h"
#include "dwm_port.h"
#include "dwm_init.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

/* Simple 4-byte protocol:
 *  Initiator sends: 'P','I', seq, 0x0C
 *  Responder sends: 'P','O', seq, 0x0C
 */

static uint8_t tx_pong_msg[] = { 'P', 'O', 0x00, 0x0C };
static uint8_t rx_buf[16];

void twr_pong_init(void)
{
    dbg("twr_pong_init: ready\r\n");
}

void twr_pong_step(void)
{
    uint32_t status;
    char buf[64];

    dbg("before step\r\n");
    dbg("twr_pong: step start\r\n");

    /* Put DW3000 into RX immediately */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dbg("twr_pong: RX enabled\r\n");

    /* Wait until we see any RX-related event bit set */
    do
    {
        status = dwt_readsysstatuslo();
    } while ((status & (DWT_INT_RXFCG_BIT_MASK |
                        DWT_INT_RXFTO_BIT_MASK |
                        DWT_INT_RXPHE_BIT_MASK |
                        DWT_INT_RXFCE_BIT_MASK |
                        DWT_INT_RXFSL_BIT_MASK |
                        DWT_INT_RXPTO_BIT_MASK |
                        DWT_INT_RXOVRR_BIT_MASK |
                        DWT_INT_RXSTO_BIT_MASK)) == 0U);

    /* Check for RX errors / timeout */
    if (status & (DWT_INT_RXFTO_BIT_MASK  |
                  DWT_INT_RXPHE_BIT_MASK  |
                  DWT_INT_RXFCE_BIT_MASK  |
                  DWT_INT_RXFSL_BIT_MASK  |  /* RXFSL */
                  DWT_INT_RXPTO_BIT_MASK  |
                  DWT_INT_RXOVRR_BIT_MASK |
                  DWT_INT_RXSTO_BIT_MASK))
    {
        dbg("twr_pong: RX error/timeout\r\n");

        /* Clear all RX-related status bits we just tested */
        dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK  |
                             DWT_INT_RXPHE_BIT_MASK  |
                             DWT_INT_RXFCE_BIT_MASK  |
                             DWT_INT_RXFSL_BIT_MASK  |
                             DWT_INT_RXPTO_BIT_MASK  |
                             DWT_INT_RXOVRR_BIT_MASK |
                             DWT_INT_RXSTO_BIT_MASK);

        dbg("after step\r\n");
        return;
    }

    /* If we get here, we have a good frame */
    uint16_t len = dwt_getframelength(NULL);
    if (len > sizeof(rx_buf)) len = sizeof(rx_buf);

    memset(rx_buf, 0, sizeof(rx_buf));
    dwt_readrxdata(rx_buf, len, 0);


    /* ---- NEW: 32-bit RX timestamp logging ---- */
    {
        uint32_t t_rx32 = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
        snprintf(buf, sizeof(buf),
                 "twr_pong: t_rx32=0x%08lX\r\n",
                 (unsigned long)t_rx32);
        dbg(buf);
    }
    /* ----------------------------------------- */

    /* Very basic check: first bytes must be 'P','I' */
    if (rx_buf[0] != 'P' || rx_buf[1] != 'I')
    {
        dbg("twr_pong: frame not PING\r\n");
        dbg("after step\r\n");
        return;
    }

    snprintf(buf, sizeof(buf),
             "twr_pong: RX OK, len>=4, first: %02X %02X %02X %02X\r\n",
             rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    dbg(buf);

    /* Build PONG using same seq and trailer */
    tx_pong_msg[2] = rx_buf[2];
    tx_pong_msg[3] = 0x0C;

    dwt_writetxfctrl(sizeof(tx_pong_msg), 0, 0);
    dwt_writetxdata(sizeof(tx_pong_msg), tx_pong_msg, 0);


    /* Transmit PONG immediately and wait for TXFRS */
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
    {
        dbg("twr_pong: TX started (PONG)\r\n");

        /* Wait for TXFRS */
        do
        {
            status = dwt_readsysstatuslo();
        } while ((status & DWT_INT_TXFRS_BIT_MASK) == 0U);

        dbg("twr_pong: TXFRS (PONG sent)\r\n");

        /* Clear TXFRS */
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    }
    else
    {
        dbg("twr_pong: dwt_starttx failed\r\n");
    }

    dbg("after step\r\n");
}
