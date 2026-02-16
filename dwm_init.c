#include "dwm_init.h"
#include "dwm_port.h"

#include "main.h"
#include "stm32g4xx_hal.h"

#include "deca_device_api.h"
#include "deca_interface.h"

#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart2;

/* Required by your EXTI callback (and other files) */
volatile int g_dwt_ready = 0;

/* Provided by Qorvo/driver glue */
extern struct dwt_spi_s *get_dwt_spi_ops(void);
extern const struct dwt_driver_s dw3000_driver;
extern void dwm_wakeup_io(void);

static struct dwt_probe_s g_probe;

static void p(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

bool dwm_init_minimal(void)
{
    char buf[96];

    p("INIT start\r\n");

    dwm_port_init();
    p("INIT: port ok\r\n");

    p("RESET: start\r\n");
    dwm_reset_device();
    p("RESET: done\r\n");

    /* ---------- SLOW SPI for probe/early init ---------- */
    dwm_set_spi_slow();
    p("INIT: spi slow\r\n");

    /* ---------- PROBE ---------- */
    g_probe.dw  = NULL;
    g_probe.spi = (void*)get_dwt_spi_ops();
    g_probe.wakeup_device_with_io = dwm_wakeup_io;

    static struct dwt_driver_s * const drivers[] = {
        (struct dwt_driver_s *)&dw3000_driver
    };
    g_probe.driver_list    = (struct dwt_driver_s **)drivers;
    g_probe.dw_driver_num  = 1;

    p("INIT: probing...\r\n");
    int ret = dwt_probe(&g_probe);
    snprintf(buf, sizeof(buf), "INIT: dwt_probe ret=%d\r\n", ret);
    p(buf);
    if (ret != DWT_SUCCESS) {
        p("INIT FAIL: probe\r\n");
        return false;
    }

    /* ---------- IDLERC ---------- */
    p("INIT: idlerc...\r\n");
    int32_t rc = dwt_checkidlerc();
    snprintf(buf, sizeof(buf), "INIT: idlerc rc=%ld\r\n", (long)rc);
    p(buf);
    if (rc == 0) {
        p("INIT FAIL: idlerc\r\n");
        return false;
    }

    /* ---------- INITIALISE ---------- */
    p("INIT: initialise...\r\n");
    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        p("INIT FAIL: initialise\r\n");
        return false;
    }
    p("INIT: initialise OK\r\n");

    /*
     * If you previously saw dwt_configure() hang, doing a softreset here
     * can help recover the state machine.
     */
    p("INIT: softreset before config\r\n");
    dwt_softreset(1);
    HAL_Delay(2);
    dwt_forcetrxoff();
    HAL_Delay(2);

    /* ---------- CONFIG (keep SLOW SPI during configure) ---------- */
    p("INIT: configuring radio...\r\n");

    dwt_config_t cfg = {0};

    /* Use Channel 5 as you mentioned you set it back */
    cfg.chan            = 5;

    /* These are “safe defaults” to get packets moving */
    cfg.txPreambLength  = DWT_PLEN_128;
    cfg.rxPAC           = DWT_PAC8;
    cfg.txCode          = 9;
    cfg.rxCode          = 9;
    cfg.sfdType         = 0;                 /* standard 8-bit IEEE SFD */
    cfg.dataRate        = DWT_BR_6M8;
    cfg.phrMode         = DWT_PHRMODE_STD;
    cfg.phrRate         = DWT_PHRRATE_STD;
    cfg.sfdTO           = (128 + 64 - 8);    /* common rule-of-thumb */
    cfg.stsMode         = DWT_STS_MODE_OFF;
    cfg.stsLength       = DWT_STS_LEN_64;    /* ignored when STS off */
    cfg.pdoaMode        = DWT_PDOA_M0;

    if (dwt_configure(&cfg) != DWT_SUCCESS) {
        p("INIT FAIL: config\r\n");
        return false;
    }

    p("INIT: config OK\r\n");

    /* ---------- NOW FAST SPI ---------- */
    dwm_set_spi_fast();
    p("INIT: spi fast\r\n");

    /* You will later replace these with calibrated values */
    dwt_settxantennadelay(16384);
    dwt_setrxantennadelay(16384);

    dwt_forcetrxoff();

    p("INIT SUCCESS\r\n");
    g_dwt_ready = 1;
    return true;
}

uint32_t dwm_read_devid(void)
{
    return dwt_readdevid();
}
