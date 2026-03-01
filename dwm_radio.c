#include "dwm_radio.h"
#include "deca_device_api.h"
#include "uart.h"

/* === Antenna delay tuning ===
 * Start with something non-zero.
 * You will adjust these until measured distance matches a known distance.
 */
#ifndef ANT_DLY_TX
#define ANT_DLY_TX  22050
#endif

#ifndef ANT_DLY_RX
#define ANT_DLY_RX  22050
#endif

int dwm_radio_apply_default(void)
{
    int32_t ir = dwt_initialise(DWT_DW_INIT);
    dbg_printf("RADIO: dwt_initialise ret=%ld\r\n", (long)ir);
    if (ir != DWT_SUCCESS) return -10;

    dwt_config_t cfg = {0};
    cfg.chan = 5;
    cfg.txPreambLength = DWT_PLEN_128;
    cfg.rxPAC = DWT_PAC8;
    cfg.txCode = 9;
    cfg.rxCode = 9;
    cfg.sfdType = 0;
    cfg.dataRate = DWT_BR_6M8;
    cfg.phrMode = DWT_PHRMODE_STD;
    cfg.phrRate = DWT_PHRRATE_STD;
    cfg.sfdTO = (129 + 8 - 8);
    cfg.stsMode = DWT_STS_MODE_OFF;
    cfg.stsLength = DWT_STS_LEN_64;
    cfg.pdoaMode = DWT_PDOA_M0;

    int32_t cr = dwt_configure(&cfg);
    dbg_printf("RADIO: dwt_configure ret=%ld\r\n", (long)cr);
    if (cr != DWT_SUCCESS) return -11;

    dwt_txconfig_t txrf = {0};
    txrf.PGdly = 0x34;
    txrf.power = 0xFDFDFDFD;
    txrf.PGcount = 0;
    dwt_configuretxrf(&txrf);
    dbg_printf("RADIO: txrf configured (PGdly=0x%02X power=0x%08lX)\r\n",
               txrf.PGdly, (unsigned long)txrf.power);

    dwt_configureframefilter(0, 0);
    dbg_print("FF: disabled via dwt_configureframefilter(0,0)\r\n");

    /* Apply antenna delays (tune these!) */
    dwt_settxantennadelay(ANT_DLY_TX);
    dwt_setrxantennadelay(ANT_DLY_RX);
    dbg_printf("RADIO: antDly TX=%lu RX=%lu\r\n",
               (unsigned long)ANT_DLY_TX, (unsigned long)ANT_DLY_RX);

    dbg_print("RADIO: default config applied\r\n");
    return 0;
}
