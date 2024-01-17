#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------- 定义和静态变量 ------------------ */

extern uint8_t longComID, comm_mode_flag;
extern uint8_t reg_buf[6];

void dfr0715_multinet_init(void);

void comm_reply(uint8_t* buf, uint8_t size);

void long_command_handle(uint8_t id, uint8_t* data);

void get_feed_data(bool is_get_raw_channel, int16_t* buffer, int buffer_len);

#ifdef __cplusplus
}
#endif
