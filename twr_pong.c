#include "twr_pong.h"

#include "deca_device_api.h"
#include "main.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

#define PAN_ID    0xCAFE
#define ADDR_INIT 0x1234
#define ADDR_RESP 0x5678

#define FCF0 0x41
#define FCF1 0x88

#define FCS_LEN    2
#define RX_FCS_LEN 2

static void p(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void p_hex16(const char *tag, const uint8_t *b, uint16_t len)
{
    char buf[256];
    uint16_t n = (len > 16) ? 16 : len;
    int off = snprintf(buf, sizeof(buf), "%s len=%u:", tag, (unsigned)len);
    for (uint16_t i = 0; i < n && off < (int)sizeof(buf) - 4; i++)
        off += snprintf(buf + off, sizeof(buf) - off, " %02X", b[i]);
    off += snprintf(buf + off, sizeof(buf) - off, "\r\n");
    p(buf);
}

#define RESP_DELAY_UUS   (12000ULL)
#define UUS_TO_DWT       (65536ULL)

static uint8_t rx_buf[256];

/*
 * PONG frame:
 * header(9) + app(13) = 22 bytes
 * App: 'P','O',seq, T2(5), T3(5)
 */
static uint8_t tx_pong[22] = {
    FCF0, FCF1, 0,
    (uint8_t)(PAN_ID & 0xFF), (uint8_t)(PAN_ID >> 8),
    (uint8_t)(ADDR_INIT & 0xFF), (uint8_t)(ADDR_INIT >> 8),   // DST = initiator
    (uint8_t)(ADDR_RESP & 0xFF), (uint8_t)(ADDR_RESP >> 8),   // SRC = responder

    'P','O', 0,
    0,0,0,0,0,   // T2
    0,0,0,0,0    // T3
};

static void write_ts40(uint8_t *buf, int idx, uint64_t ts)
{
    for (int i = 0; i < 5; i++) buf[idx + i] = (uint8_t)((ts >> (8*i)) & 0xFF);
}

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, (dwt_ip_sts_segment_e)0);

    uint64_t v = 0;
    for (int i = 0; i < 5; i++) v |= ((uint64_t)ts[i]) << (8*i);
    return v;
}

static int find_ping3(const uint8_t *buf, uint16_t len, uint8_t *seq_out, int *idx_out)
{
    if (len <= RX_FCS_LEN) return 0;
    uint16_t usable = (uint16_t)(len - RX_FCS_LEN);
    if (usable < 3) return 0;

    for (uint16_t i = 0; i + 2 < usable; i++) {
        if (buf[i] == 'P' && buf[i+1] == 'I') {
            *seq_out = buf[i+2];
            *idx_out = (int)i;
            return 1;
        }
    }
    return 0;
}

static void rx_reenable_clean(void)
{
    dwt_forcetrxoff();
    HAL_Delay(1);
    dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void twr_pong_init(void)
{
    p("twr_pong_init ready\r\n");
    rx_reenable_clean();
}

void twr_pong_step(void)
{
    uint32_t status = dwt_readsysstatuslo();

    if (status & DWT_INT_RXFCG_BIT_MASK)
    {
        uint16_t len = dwt_getframelength(NULL);
        if (len > sizeof(rx_buf)) len = sizeof(rx_buf);

        dwt_readrxdata(rx_buf, len, 0);
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

        p_hex16("PONG: got", rx_buf, len);

        uint8_t seq = 0xFF;
        int idx = -1;

        if (find_ping3(rx_buf, len, &seq, &idx))
        {
            uint64_t T2 = read_rx_ts40();

            uint64_t tx_time_32h = (T2 + (RESP_DELAY_UUS * UUS_TO_DWT)) >> 8;
            dwt_setdelayedtrxtime((uint32_t)tx_time_32h);

            static uint8_t mac_seq = 0;
            tx_pong[2]  = mac_seq++;
            tx_pong[11] = seq;

            write_ts40(tx_pong, 12, T2);

            uint64_t T3 = (tx_time_32h << 8);
            write_ts40(tx_pong, 17, T3);

            dwt_writetxdata(sizeof(tx_pong), tx_pong, 0);

            // IMPORTANT: your build expects length INCLUDING FCS
            dwt_writetxfctrl((uint16_t)(sizeof(tx_pong) + FCS_LEN), 0, 0);

            if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
            {
                while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) { }
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

                char b[96];
                snprintf(b, sizeof(b), "OK RX seq=%u (sig@%d) -> TX pong\r\n",
                         (unsigned)seq, idx);
                p(b);
            }
            else
            {
                p("TX pong FAIL (missed slot) -> recover\r\n");
                rx_reenable_clean();
                return;
            }
        }

        rx_reenable_clean();
        return;
    }

    if (status & (DWT_INT_RXFTO_BIT_MASK |
                  DWT_INT_RXPTO_BIT_MASK |
                  DWT_INT_RXFSL_BIT_MASK |
                  DWT_INT_RXFCE_BIT_MASK))
    {
        rx_reenable_clean();
        return;
    }
}
