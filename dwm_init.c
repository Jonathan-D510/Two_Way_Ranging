#include "dwm_init.h"
#include "dwm_port.h"
#include "deca_device_api.h"
#include "uart.h"
#include "dwm_diag.h"
#include "dwm_spi_if.h"
#include <string.h>

/* Driver descriptor (exists when USE_DRV_DW3000 is defined) */
extern const struct dwt_driver_s dw3000_driver;

int dw3000_init(uint32_t *out_devid)
{
    dbg_print("dwm_init: port_init...\r\n");
    dw3000_port_init();
    dbg_print("dwm_init: port_init OK\r\n");

    dbg_print("dwm_init: hard_reset...\r\n");
    dw3000_hard_reset();
    dbg_print("dwm_init: hard_reset OK\r\n");

    dbg_print("dwm_init: wakeup_pulse...\r\n");
    dw3000_wakeup_pulse();
    dbg_print("dwm_init: wakeup_pulse OK\r\n");

    /* RAW SPI sanity check (no driver) */
    dbg_print("dwm_init: raw read DEV_ID...\r\n");
    uint32_t raw_id = 0;
    int rr = dwm_raw_read_devid(&raw_id);
    dbg_printf("dwm_init: raw read ret=%d raw_id=0x%08lX\r\n", rr, (unsigned long)raw_id);

    if (raw_id == 0x00000000UL || raw_id == 0xFFFFFFFFUL)
    {
        dbg_print("dwm_init: RAW_ID invalid -> check SPI/CS/power/reset\r\n");
        if (out_devid) *out_devid = raw_id;
        return -999;
    }

    /* Build the probe interface struct expected by this API version */
    static struct dwt_driver_s *driver_list[1];
    driver_list[0] = (struct dwt_driver_s *)&dw3000_driver;

    struct dwt_probe_s probe;
    memset(&probe, 0, sizeof(probe));

    probe.dw = NULL;
    probe.spi = dwm_get_spi_if();               /* <-- KEY CHANGE */
    probe.wakeup_device_with_io = dw3000_wakeup_pulse;
    probe.driver_list = driver_list;
    probe.dw_driver_num = 1;

    dbg_print("dwm_init: calling dwt_probe...\r\n");
    int32_t pr = dwt_probe(&probe);
    dbg_printf("dwm_init: dwt_probe ret=%ld\r\n", (long)pr);

    dbg_print("dwm_init: dwt_readdevid...\r\n");
    uint32_t id = dwt_readdevid();
    dbg_printf("dwm_init: devID=0x%08lX\r\n", (unsigned long)id);

    if (out_devid) *out_devid = id;
    return (int)pr;
}
