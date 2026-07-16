/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 *
 * SDIO buffer/datapath config advertised by the CP in the boot init event.
 * Carried as EH_PRIV_SDIO_BUF_CONFIG (0x1B, TLV_V2 wire); the FG_V2 build
 * emits the same 5-byte layout under upstream tag 5 (esp_priv_rx_buf_config).
 * Layout and enum values mirror upstream fg-v2 exactly — do not reorder.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EH_SDIO_CFG_BUF_BLOCK  512u  /* bufsz fields are in 512-byte units */

enum eh_sdio_cfg_transport {
    EH_SDIO_CFG_TPORT_SDIO = 0,
    EH_SDIO_CFG_TPORT_SPI  = 1,
};

/* NOT the legacy 0x18 encoding (there: packet=0/stream=1) — never mix. */
enum eh_sdio_cfg_tx_mode {
    EH_SDIO_CFG_TXMODE_SW_AGGR = 0,
    EH_SDIO_CFG_TXMODE_STREAM  = 1,
    EH_SDIO_CFG_TXMODE_PACKET  = 2,
};

struct eh_priv_sdio_buf_config {
    uint8_t transport;       /* enum eh_sdio_cfg_transport */
    uint8_t e2h_mode;        /* enum eh_sdio_cfg_tx_mode, CP→host */
    uint8_t e2h_bufsz_512B;  /* CP max E2H transfer / 512 → host RX cap */
    uint8_t h2e_mode;        /* enum eh_sdio_cfg_tx_mode, host→CP */
    uint8_t h2e_bufsz_512B;  /* CP H2E recv buffer / 512 → host TX aggregate cap */
} __attribute__((packed));

_Static_assert(sizeof(struct eh_priv_sdio_buf_config) == 5,
               "sdio buf-config wire layout must be exactly 5 bytes");

typedef enum {
    EH_SDIO_CFG_OK = 0,
    EH_SDIO_CFG_MALFORMED,   /* bad length/transport/mode/size → host must fail boot */
} eh_sdio_cfg_verdict_t;

/* Validate a received buf-config TLV value. Structure-level only; the caller
 * still checks the sizes against its own allocation ceiling (fail boot on
 * exceed — no silent degrade). */
static inline eh_sdio_cfg_verdict_t
eh_sdio_cfg_parse(const uint8_t *val, uint8_t len, struct eh_priv_sdio_buf_config *out)
{
    if (!val || len != sizeof(struct eh_priv_sdio_buf_config))
        return EH_SDIO_CFG_MALFORMED;
    if (val[0] != EH_SDIO_CFG_TPORT_SDIO)
        return EH_SDIO_CFG_MALFORMED;
    if (val[1] > EH_SDIO_CFG_TXMODE_PACKET || val[3] > EH_SDIO_CFG_TXMODE_PACKET)
        return EH_SDIO_CFG_MALFORMED;
    if (val[2] == 0 || val[4] == 0)
        return EH_SDIO_CFG_MALFORMED;
    if (out) {
        out->transport      = val[0];
        out->e2h_mode       = val[1];
        out->e2h_bufsz_512B = val[2];
        out->h2e_mode       = val[3];
        out->h2e_bufsz_512B = val[4];
    }
    return EH_SDIO_CFG_OK;
}

#ifdef __cplusplus
}
#endif
