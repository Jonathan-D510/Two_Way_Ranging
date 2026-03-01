#pragma once
#include <stdint.h>

void uwb_initiator_init(void);

/* Returns 0 on success, <0 on failure; fills got_id/seq */
int uwb_send_poll_wait_resp(uint8_t target_id, uint8_t *got_id, uint8_t *got_seq);
