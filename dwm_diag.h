#pragma once
#include <stdint.h>

/* Returns 0 on success, nonzero on failure. */
int dwm_raw_read_devid(uint32_t *out_devid);
