#ifndef _ESP_SDIO_API_H_
#define _ESP_SDIO_API_H_
#include "esp_sdio_decl.h"

int esp32_read_reg(struct esp32_sdio_context *context, u32 reg, u8 *data, u16 size);
int esp32_read_block(struct esp32_sdio_context *context, u32 reg, u8 *data, u16 size);
int esp32_write_reg(struct esp32_sdio_context *context, u32 reg, u8 *data, u16 size);
int esp32_write_block(struct esp32_sdio_context *context, u32 reg, u8 *data, u16 size);

#endif
