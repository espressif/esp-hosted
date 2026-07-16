/* SPDX-License-Identifier: Apache-2.0 */
/* MCU adapters between upstream esp_remote_channel API and our channel registry. */

#include "esp_hosted_wifi_remote_glue.h"
#include "eh_host_mcu_transport_channels.h"

#include <stdint.h>
#include "esp_err.h"

struct eh_host_channel_s;

esp_remote_channel_t esp_hosted_add_channel(esp_remote_channel_config_t config,
        esp_remote_channel_tx_fn_t *tx, const esp_remote_channel_rx_fn_t rx)
{
    if (!config || !tx || !rx) return NULL;
    return (esp_remote_channel_t)
        eh_host_transport_add_channel(NULL,
                                       (uint8_t)config->if_type,
                                       (uint8_t)config->secure,
                                       (esp_err_t (**)(void *, void *, size_t))tx,
                                       (esp_err_t (*)(void *, void *, void *, size_t))rx);
}

esp_err_t esp_hosted_remove_channel(esp_remote_channel_t channel)
{
    if (!channel) return ESP_ERR_INVALID_ARG;
    return eh_host_transport_remove_channel((eh_host_channel_t *)channel) == 0
           ? ESP_OK : ESP_FAIL;
}

int esp_hosted_tx(uint8_t iface_type, uint8_t iface_num,
                  uint8_t *payload_buf, uint16_t payload_len,
                  uint8_t buff_zerocopy,
                  uint8_t *buffer_to_free,
                  void (*free_buf_func)(void *ptr),
                  uint8_t flags)
{
    int rc;
    if (buff_zerocopy) {
        rc = (int)eh_host_transport_tx_zerocopy(iface_type, iface_num,
                                                 payload_buf, payload_len, flags);
    } else {
        rc = (int)eh_host_transport_tx(iface_type, iface_num,
                                        (const uint8_t *)payload_buf,
                                        payload_len, flags);
    }
    if (buffer_to_free && free_buf_func) free_buf_func(buffer_to_free);
    return rc;
}
