#include "twr_ping.h"
#include "deca_device_api.h"
#include "main.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;

#define PAN_ID    0xCAFE
#define ADDR_INIT 0x1234
#define ADDR_RESP 0x5678

#define FCF0 0x41
#define FCF1 0x88

#define FCS_LEN     2
#define RX_FCS_LEN  2

// Calibrated to center your ~40 cm cluster:
// RAW ~37.42 m at true ~0.40 m -> OFFSET ~37.02 m
#define OFFSET_MM   37020u

// Reject sudden jumps bigger than this (mm)
#define JUMP_REJECT_MM  300u   // 0.30 m

static void p(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void p_hexN(const char *tag, const uint8_t *b, uint16_t len, uint16_t nmax)
{
    char buf[512];
    uint16_t n = (len > nmax) ? nmax : len;
    int off = snprintf(buf, sizeof(buf), "%s len=%u:", tag, (unsigned)len);
    for (uint16_t i = 0; i < n && off < (int)sizeof(buf) - 4; i++)
        off += snprintf(buf + off, sizeof(buf) - off, " %02X", b[i]);
    off += snprintf(buf + off, sizeof(buf) - off, "\r\n");
    p(buf);
}

static void i64_to_dec(char *out, size_t out_sz, int64_t v)
{
    if (out_sz == 0) return;

    char tmp[32];
    int neg = 0;
    uint64_t x;

    if (v < 0) { neg = 1; x = (uint64_t)(-v); }
    else       { x = (uint64_t)v; }

    int i = 0;
    do {
        tmp[i++] = (char)('0' + (x % 10));
        x /= 10;
    } while (x && i < (int)sizeof(tmp));

    size_t pos = 0;
    if (neg && pos + 1 < out_sz) out[pos++] = '-';

    while (i-- > 0 && pos + 1 < out_sz) out[pos++] = tmp[i];
    out[pos] = 0;
}

static uint64_t ts40_from5(const uint8_t ts[5])
{
    uint64_t v = 0;
    for (int i = 0; i < 5; i++) v |= ((uint64_t)ts[i]) << (8*i);
    return v;
}

static uint64_t read_tx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readtxtimestamp(ts);
    return ts40_from5(ts);
}

static uint64_t read_rx_ts40(void)
{
    uint8_t ts[5] = {0};
    dwt_readrxtimestamp(ts, (dwt_ip_sts_segment_e)0);
    return ts40_from5(ts);
}

static uint64_t read_ts40_from_msg(const uint8_t *buf, int idx)
{
    uint64_t v = 0;
    for (int i = 0; i < 5; i++) v |= ((uint64_t)buf[idx + i]) << (8*i);
    return v;
}

#ifndef DWT_TIME_UNITS
#define DWT_TIME_UNITS (1.0/499.2e6/128.0)
#endif

static uint32_t tof_ticks_to_mm(int64_t tof_dtu)
{
    const double C = 299702547.0;
    double meters = ((double)tof_dtu) * DWT_TIME_UNITS * C;
    if (meters < 0) meters = 0;
    double mm = meters * 1000.0;
    if (mm < 0) mm = 0;
    if (mm > 4.0e9) mm = 4.0e9;
    return (uint32_t)(mm + 0.5);
}

static void format_dist(char *out, size_t out_sz, uint32_t mm)
{
    uint32_t m_int  = mm / 1000;
    uint32_t m_frac = mm % 1000;

    uint32_t cm10     = mm;
    uint32_t cm_int   = cm10 / 10;
    uint32_t cm_frac1 = cm10 % 10;

    uint32_t inches10 = (uint32_t)(((uint64_t)mm * 1000ULL + 127ULL) / 254ULL);
    uint32_t total_inches = inches10 / 10;
    uint32_t inch_frac1   = inches10 % 10;

    uint32_t feet       = total_inches / 12;
    uint32_t inch_whole = total_inches % 12;

    snprintf(out, out_sz,
             "%lu.%03lu m = %lu.%01lu cm = %lu ft %lu.%01lu in",
             (unsigned long)m_int, (unsigned long)m_frac,
             (unsigned long)cm_int, (unsigned long)cm_frac1,
             (unsigned long)feet, (unsigned long)inch_whole, (unsigned long)inch_frac1);
}

/*
 * PING frame: header(9) + app(3) = 12 bytes
 * App: 'P','I',seq
 */
static uint8_t tx_ping[12] = {
    FCF0, FCF1, 0,
    (uint8_t)(PAN_ID & 0xFF), (uint8_t)(PAN_ID >> 8),
    (uint8_t)(ADDR_RESP & 0xFF), (uint8_t)(ADDR_RESP >> 8),
    (uint8_t)(ADDR_INIT & 0xFF), (uint8_t)(ADDR_INIT >> 8),
    'P','I', 0
};

static int find_pong(const uint8_t *buf, uint16_t len, uint8_t want_seq, int *idx_out)
{
    if (len <= RX_FCS_LEN) return 0;
    uint16_t usable = (uint16_t)(len - RX_FCS_LEN);
    if (usable < 13) return 0;

    for (uint16_t i = 0; i + 12 < usable; i++) {
        if (buf[i] == 'P' && buf[i+1] == 'O' && buf[i+2] == want_seq) {
            *idx_out = (int)i;
            return 1;
        }
    }
    return 0;
}

void twr_ping_init(void)
{
    p("twr_ping_init ready\r\n");
    {
        char b[120];
        snprintf(b, sizeof(b), "PING: OFFSET_MM=%lu JUMP_REJECT_MM=%lu\r\n",
                 (unsigned long)OFFSET_MM, (unsigned long)JUMP_REJECT_MM);
        p(b);
    }
}

void twr_ping_step(void)
{
    static uint8_t app_seq = 0;
    static uint8_t mac_seq = 0;

    // last accepted corrected measurement (mm)
    static uint32_t last_corr_mm = 0;
    static uint8_t  have_last = 0;

    tx_ping[2]  = mac_seq++;
    tx_ping[11] = app_seq++;

    p_hexN("PING: tx", tx_ping, sizeof(tx_ping), 32);

    dwt_forcetrxoff();
    HAL_Delay(2);
    dwt_writesysstatuslo(0xFFFFFFFF);

    dwt_writetxdata(sizeof(tx_ping), tx_ping, 0);
    dwt_writetxfctrl((uint16_t)(sizeof(tx_ping) + FCS_LEN), 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        p("PING: TX FAIL\r\n");
        return;
    }

    uint32_t t0 = HAL_GetTick();
    while (!(dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK)) {
        if (HAL_GetTick() - t0 > 50) {
            p("PING: TX TIMEOUT\r\n");
            dwt_forcetrxoff();
            return;
        }
    }
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

    uint64_t T1 = read_tx_ts40();

    {
        char b[64];
        snprintf(b, sizeof(b), "ping app_seq=%u\r\n", (unsigned)tx_ping[11]);
        p(b);
    }

    dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 800) {

        uint32_t status = dwt_readsysstatuslo();

        if (status & DWT_INT_RXFCG_BIT_MASK) {

            uint8_t rx[256];
            uint16_t len = dwt_getframelength(NULL);
            if (len > sizeof(rx)) len = sizeof(rx);

            dwt_readrxdata(rx, len, 0);
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            uint64_t T4 = read_rx_ts40();

            p_hexN("PING: got", rx, len, 24);

            int idx = -1;
            if (find_pong(rx, len, tx_ping[11], &idx)) {

                uint64_t T2 = read_ts40_from_msg(rx, idx + 3);
                uint64_t T3 = read_ts40_from_msg(rx, idx + 8);

                int64_t Ra = (int64_t)(T4 - T1);
                int64_t Rb = (int64_t)(T3 - T2);
                int64_t tof_dtu = (Ra - Rb) / 2;

                uint32_t raw_mm = tof_ticks_to_mm(tof_dtu);

                uint32_t corr_mm = 0;
                if (raw_mm > OFFSET_MM) corr_mm = raw_mm - OFFSET_MM;

                // jump reject
                uint32_t used_mm = corr_mm;
                uint8_t rejected = 0;

                if (have_last) {
                    uint32_t diff = (corr_mm > last_corr_mm) ? (corr_mm - last_corr_mm) : (last_corr_mm - corr_mm);
                    if (diff > JUMP_REJECT_MM) {
                        used_mm = last_corr_mm;  // hold last
                        rejected = 1;
                    } else {
                        last_corr_mm = corr_mm;
                    }
                } else {
                    last_corr_mm = corr_mm;
                    have_last = 1;
                }

                char sToF[40];
                i64_to_dec(sToF, sizeof(sToF), tof_dtu);

                char raw_str[120], cor_str[120], use_str[120];
                format_dist(raw_str, sizeof(raw_str), raw_mm);
                format_dist(cor_str, sizeof(cor_str), corr_mm);
                format_dist(use_str, sizeof(use_str), used_mm);

                char out[420];
                snprintf(out, sizeof(out),
                         "OK seq=%u tof_dtu=%s\r\n"
                         "  RAW : %s\r\n"
                         "  CORR: %s\r\n"
                         "  USED: %s%s\r\n",
                         (unsigned)tx_ping[11], sToF,
                         raw_str, cor_str, use_str,
                         rejected ? "  (REJECTED jump)" : "");
                p(out);

            } else {
                p("PING: not my pong\r\n");
            }

            dwt_forcetrxoff();
            return;
        }

        if (status & (DWT_INT_RXFTO_BIT_MASK |
                      DWT_INT_RXPTO_BIT_MASK |
                      DWT_INT_RXFSL_BIT_MASK |
                      DWT_INT_RXFCE_BIT_MASK))
        {
            dwt_writesysstatuslo(DWT_INT_RXFTO_BIT_MASK |
                                 DWT_INT_RXPTO_BIT_MASK |
                                 DWT_INT_RXFSL_BIT_MASK |
                                 DWT_INT_RXFCE_BIT_MASK);
            p("PING: RX err/timeout flag\r\n");
            dwt_forcetrxoff();
            return;
        }
    }

    p("PING: RX wait timeout\r\n");
    dwt_forcetrxoff();
}
