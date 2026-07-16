/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "eh_common_interface.h"
#include "eh_common_header.h"
#include "eh_frame.h"
#include "eh_host_mcu_transport.h"
#include "eh_host_mcu_transport_channels.h"
#include "eh_host_mcu_transport_priv.h"
#include "eh_host_port_dma.h"
#include "eh_frame.h"
#include "esp_netif_types.h"

#define TAG "eh_chan"

eh_host_channel_t *chan_arr[ESP_MAX_IF];

#if EH_HOST_PORT_WIFI_TX_DATA_THROTTLE_EN
volatile uint32_t wifi_tx_throttling;
#endif

esp_err_t eh_host_transport_tx(uint8_t if_type, uint8_t if_num,
                                const uint8_t *payload, uint16_t payload_len,
                                uint8_t flags)
{
    if (!payload || payload_len == 0) return ESP_FAIL;

    if (eh_host_wifi_tx_is_throttled() &&
        (if_type == ESP_STA_IF || if_type == ESP_AP_IF)) {
#if defined(ESP_ERR_ESP_NETIF_TX_FAILED)
        return ESP_ERR_ESP_NETIF_TX_FAILED;
#else
        return ESP_ERR_ESP_NETIF_NO_MEM;
#endif
    }

    size_t total = sizeof(struct esp_payload_header) + payload_len;
    /* MUST be DMA-capable; sdmmc/SPI master reject non-DMA buffers. */
    uint8_t *buf = (uint8_t *)eh_host_port_dma_alloc(total);
    if (!buf) return ESP_ERR_NO_MEM;

    memcpy(buf + sizeof(struct esp_payload_header), payload, payload_len);

    /* Bus owns lifetime of `buf`. */
    interface_buffer_handle_t bh = {
        .priv_buffer_handle = buf,
        .if_type            = if_type,
        .if_num             = if_num,
        .flags              = flags,
        .payload            = buf + sizeof(struct esp_payload_header),
        .payload_len        = payload_len,
        .payload_zcopy      = 0,
        .free_buf_handle    = eh_host_port_dma_free,
    };
    eh_frame_encode(buf, &bh, payload_len);
    int rc = eh_host_mcu_transport_tx(&bh);
    return (rc >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t eh_host_transport_tx_zerocopy(uint8_t if_type, uint8_t if_num,
                                         uint8_t *payload, uint16_t payload_len,
                                         uint8_t flags)
{
    if (!payload || payload_len == 0) return ESP_FAIL;

    if (eh_host_wifi_tx_is_throttled() &&
        (if_type == ESP_STA_IF || if_type == ESP_AP_IF)) {
#if defined(ESP_ERR_ESP_NETIF_TX_FAILED)
        return ESP_ERR_ESP_NETIF_TX_FAILED;
#else
        return ESP_ERR_ESP_NETIF_NO_MEM;
#endif
    }

    /* Caller-promised headroom. */
    uint8_t *frame_start = payload - sizeof(struct esp_payload_header);

    /* Zero-copy — caller retains ownership of `frame_start`. */
    interface_buffer_handle_t bh = {
        .priv_buffer_handle = NULL,
        .if_type            = if_type,
        .if_num             = if_num,
        .flags              = flags,
        .payload            = payload,
        .payload_len        = payload_len,
        .payload_zcopy      = 1,
        .free_buf_handle    = NULL,
    };
    eh_frame_encode(frame_start, &bh, payload_len);
    int rc = eh_host_mcu_transport_tx(&bh);
    return (rc >= 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t sta_tx(void *h, void *buf, size_t len)
{
    (void)h;
    if (!buf || len == 0 || len > UINT16_MAX) return ESP_FAIL;
    return eh_host_transport_tx(ESP_STA_IF, 0, buf, (uint16_t)len, 0);
}
static esp_err_t ap_tx(void *h, void *buf, size_t len)
{
    (void)h;
    if (!buf || len == 0 || len > UINT16_MAX) return ESP_FAIL;
    return eh_host_transport_tx(ESP_AP_IF, 0, buf, (uint16_t)len, 0);
}
static esp_err_t serial_tx(void *h, void *buf, size_t len)
{
    (void)h;
    if (!buf || len == 0 || len > UINT16_MAX) return ESP_FAIL;
    return eh_host_transport_tx(ESP_SERIAL_IF, 0, buf, (uint16_t)len, 0);
}

eh_host_channel_t *eh_host_transport_get_channel(uint8_t if_type)
{
    return (if_type < ESP_MAX_IF) ? chan_arr[if_type] : NULL;
}

eh_host_channel_t *eh_host_transport_add_channel(void *api_chan,
                                                  uint8_t if_type,
                                                  uint8_t secure,
                                                  eh_host_channel_tx_fn_t *tx_out,
                                                  eh_host_channel_rx_fn_t rx)
{
    if (if_type >= ESP_MAX_IF || !tx_out || !rx) return NULL;

    if (chan_arr[if_type]) {
        free(chan_arr[if_type]);
        chan_arr[if_type] = NULL;
    }

    eh_host_channel_t *chan = (eh_host_channel_t *)calloc(1, sizeof(*chan));
    if (!chan) return NULL;

    chan->if_type  = if_type;
    chan->secure   = secure;
    chan->rx       = rx;
    chan->api_chan = api_chan ? api_chan : chan;

    switch (if_type) {
    case ESP_STA_IF:    chan->tx = sta_tx;    break;
    case ESP_AP_IF:     chan->tx = ap_tx;     break;
    case ESP_SERIAL_IF: chan->tx = serial_tx; break;
    default:
        free(chan);
        return NULL;
    }

    *tx_out = chan->tx;
    chan_arr[if_type] = chan;
    return chan;
}

int eh_host_transport_remove_channel(eh_host_channel_t *channel)
{
    if (!channel || channel->if_type >= ESP_MAX_IF) return -1;
    if (chan_arr[channel->if_type] != channel) return -1;
    chan_arr[channel->if_type] = NULL;
    free(channel);
    return 0;
}
