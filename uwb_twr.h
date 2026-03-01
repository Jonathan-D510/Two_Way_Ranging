#pragma once
#include <stdint.h>

void uwb_twr_init(uint8_t my_id);

/* You call this name in main.c already */
void uwb_twr_initi(uint8_t my_id);

/* Returns 0 on success, negative on failure; out_m in meters. */
int uwb_twr_range_to(uint8_t target_id, float *out_m);
