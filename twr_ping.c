// G431_DWM_Initiator/Core/Src/twr_ping.c

#include "twr_ping.h"
#include "deca_device_api.h"
#include "dwm_port.h"
#include "dwm_init.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

static uint8_t tx_ping_msg[] = { 'P', 'I', 0x00, 0x0C };
static uint8_t rx_buf[16];
static uint8_t ping_seq = 0;

void twr_ping_init(void)
{
    dbg("twr_ping_init: ready\r\n");
    ping_seq = 0;
}

void twr_ping_step(void)
{
    int32_t ret;
    uint32_t status = 0;
    char buf[128];

    dbg("twr_ping: step start\r\n");

    /* Prepare PING frame */
    tx_ping_msg[2] = ping_seq;

    /* Tell DW3000 how many bytes in TX buffer */
    dwt_writetxfctrl(sizeof(tx_ping_msg), 0, 0);

    /* Write frame to TX buffer */
    dwt_writetxdata(sizeof(tx_ping_msg), tx_ping_msg, 0);

    /* Start transmission and wait for it to complete */
    ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (ret != DWT_SUCCESS)
    {
        snprintf(buf, sizeof(buf), "twr_ping: dwt_starttx ret=%ld\r\n", (long)ret);
        dbg(buf);
        return;
    }

    dbg("twr_ping: TX started\r\n");

    /* Block until TX done (TXFRS) */
    do
    {
        status = dwt_readsysstatuslo();
    } while ((status & DWT_INT_TXFRS_BIT_MASK) == 0U);

    dbg("twr_ping: TXFRS (TX done)\r\n");

    /* Clear TXFRS bit */
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    /* Enable RX immediately to wait for PONG */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dbg("twr_ping: RX enabled (waiting for PONG)\r\n");

    /* Poll for RX events (RX good / RX error / timeout) */
    uint32_t t0 = HAL_GetTick();
    const uint32_t timeout_ms = 50;

    while (1)
    {
        status = dwt_readsysstatuslo();

        /* No event yet? Check SW timeout */
        if ((status & (DWT_INT_RXFCG_BIT_MASK |
                       DWT_INT_RXFTO_BIT_MASK |
                       DWT_INT_RXPHE_BIT_MASK |
                       DWT_INT_RXFCE_BIT_MASK |
                       DWT_INT_RXFSL_BIT_MASK |
                       DWT_INT_RXPTO_BIT_MASK |
                       DWT_INT_RXOVRR_BIT_MASK |
                       DWT_INT_RXSTO_BIT_MASK)) == 0U)
        {
            uint32_t t_now = HAL_GetTick();
            if ((t_now - t0) > timeout_ms)
            {
                dbg("twr_ping: no event before SW timeout in RX phase\r\n");
                //dwt_rxreset();
                ping_seq++;
                return;
            }
            continue;
        }

        /* We saw some RX event bits; break out and handle */
        break;
    }

    snprintf(buf, sizeof(buf), "twr_ping: status(RX)=0x%08lX\r\n", (unsigned long)status);
    dbg(buf);

    /* Handle RX errors / timeouts first */
    if (status & (DWT_INT_RXFTO_BIT_MASK  |
                  DWT_INT_RXPHE_BIT_MASK  |
                  DWT_INT_RXFCE_BIT_MASK  |
                  DWT_INT_RXFSL_BIT_MASK  |
                  DWT_INT_RXPTO_BIT_MASK  |
                  DWT_INT_RXOVRR_BIT_MASK |
                  DWT_INT_RXSTO_BIT_MASK))
    {
        dbg("twr_ping: no response (RX timeout or error)\r\n");

        /* Clear all RX-related status bits and exit */
        dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK  |
                             DWT_INT_RXPHE_BIT_MASK  |
                             DWT_INT_RXFCE_BIT_MASK  |
                             DWT_INT_RXFSL_BIT_MASK  |
                             DWT_INT_RXPTO_BIT_MASK  |
                             DWT_INT_RXOVRR_BIT_MASK |
                             DWT_INT_RXSTO_BIT_MASK);
        ping_seq++;
        return;
    }

    /* If we get here, we must have RXFCG set -> good frame */
    if (status & DWT_INT_RXFCG_BIT_MASK)
    {
        dbg("twr_ping: RX OK (got PONG)\r\n");

        /* Clear RXFCG bit */
        uint16_t len = dwt_getframelength(NULL);
        if (len > sizeof(rx_buf)) len = sizeof(rx_buf);

        memset(rx_buf, 0, sizeof(rx_buf));
        dwt_readrxdata(rx_buf, len, 0);


        snprintf(buf, sizeof(buf),
                 "twr_ping: RX data: %02X %02X %02X %02X\r\n",
                 rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        dbg(buf);

        /* ---- NEW: 32-bit TX/RX timestamp logging ---- */
        {
            uint32_t t_tx32 = dwt_readtxtimestamplo32();
            uint32_t t_rx32 = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
            uint32_t dt32   = t_rx32 - t_tx32;    // wraps mod 2^32

            /* ---- convert DTU → seconds → meters (integer-friendly) ---- */
            double dtu = 15.65e-12;                 // device time unit (15 ps)
            double c   = 299792458.0;            // speed of light (m/s)

            double rtt_s = (double)dt32 * dtu;
            double one_s = rtt_s * 0.5;
            double dist_m = one_s * c;

            /* convert to integers for printing (no %f required) */
            uint32_t rtt_us = (uint32_t)(rtt_s * 1e6 + 0.5);          // round to nearest microsecond
            uint32_t one_us = (uint32_t)(one_s * 1e6 + 0.5);
            uint32_t dist_mm = (uint32_t)(dist_m);     // mm resolution
            uint32_t dist_cm = dist_mm * 100;

            snprintf(buf, sizeof(buf),
                "twr_ping: dt=%lu ticks -> rtt=%lu us, one-way=%lu us, dist=%lu m (%lu cm)\r\n",
                (unsigned long)dt32,
                (unsigned long)rtt_us,
                (unsigned long)one_us,
                (unsigned long)dist_mm,
                (unsigned long)dist_cm);
            dbg(buf);



        }
        /* ------------------------------------------- */

        /* Very basic sanity: expect 'P','O',same seq,0x0C */
        /* Basic sanity: just check 'P','O' and matching seq */
/*
        if (rx_buf[0] == 'P' &&
            rx_buf[1] == 'O' &&
            rx_buf[2] == ping_seq)
        {
            dbg("twr_ping: valid PONG frame\r\n");
        }
        else
        {
            dbg("twr_ping: PONG frame format mismatch\r\n");
        }

*/
        ping_seq++;
        return;
    }

    /* Should not reach here, but just in case */
    dbg("twr_ping: unexpected status combination\r\n");
    ping_seq++;
}
