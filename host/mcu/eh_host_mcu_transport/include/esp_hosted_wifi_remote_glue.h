/* SPDX-License-Identifier: Apache-2.0 */
/* Channel-management public surface (esp_hosted_add_channel / _remove_channel). */

#ifndef EH_COMPAT_ESP_HOSTED_WIFI_REMOTE_GLUE_H_
#define EH_COMPAT_ESP_HOSTED_WIFI_REMOTE_GLUE_H_

#include <stdbool.h>
#include <stddef.h>

/* Prefer upstream types when the esp_wifi_remote umbrella is available.
 * The forward-decl block below is only for non-IDF toolchain tests; on
 * IDF builds upstream defines these and our decls would collide. */
#if __has_include(<esp_wifi_remote.h>)
#  include "esp_err.h"
#  include "esp_wifi_types.h"
#  include "esp_wifi_remote.h"
typedef wifi_interface_t esp_hosted_if_type_t;
#else
typedef int esp_err_t;
typedef void *esp_remote_channel_t;
typedef int esp_hosted_if_type_t;
typedef struct esp_remote_channel_config *esp_remote_channel_config_t;
#endif

/* Struct body — upstream declares the type opaque; we populate it here
 * because esp_hosted_add_channel callers construct it locally. */
struct esp_remote_channel_config {
    esp_hosted_if_type_t if_type;
    bool secure;
};

#define ESP_HOSTED_CHANNEL_CONFIG_DEFAULT() { .secure = true }

typedef esp_err_t (*esp_remote_channel_rx_fn_t)(void *h, void *buffer,
                                                void *buff_to_free, size_t len);
typedef esp_err_t (*esp_remote_channel_tx_fn_t)(void *h, void *buffer, size_t len);

esp_remote_channel_t esp_hosted_add_channel(esp_remote_channel_config_t config,
        esp_remote_channel_tx_fn_t *tx, const esp_remote_channel_rx_fn_t rx);

esp_err_t esp_hosted_remove_channel(esp_remote_channel_t channel);

#endif /* EH_COMPAT_ESP_HOSTED_WIFI_REMOTE_GLUE_H_ */
