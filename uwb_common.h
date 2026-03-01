#pragma once
#include <stdint.h>

#define UWB_POLL_MAGIC0 'P'
#define UWB_POLL_MAGIC1 'O'
#define UWB_POLL_MAGIC2 'L'
#define UWB_POLL_MAGIC3 'L'

#define UWB_RESP_MAGIC0 'R'
#define UWB_RESP_MAGIC1 'E'
#define UWB_RESP_MAGIC2 'S'
#define UWB_RESP_MAGIC3 'P'

#define UWB_FRAME_LEN   8

typedef struct __attribute__((packed)) {
    uint8_t m0, m1, m2, m3;
    uint8_t id;
    uint8_t seq;
    uint8_t rsv0;
    uint8_t rsv1;
} uwb_frame_t;
