/* SPDX-License-Identifier: Apache-2.0 */

/* Idempotent linker wraps for two IDF init functions that examples
 * frequently call after hosted's own host-side bring-up has already
 * called them.  Vanilla IDF returns ESP_ERR_INVALID_STATE on a second
 * call, which trips ESP_ERROR_CHECK and aborts the example — even
 * though the state requested is what the caller wanted.
 *
 * The wraps gate the real call on a static flag, so a second invocation
 * is a cheap return-ESP_OK and never reaches IDF.  This lets upstream
 * IDF example sources (which use bare ESP_ERROR_CHECK) drop in
 * unmodified on a hosted host.
 *
 * Activation: linker flags -Wl,--wrap=esp_netif_init and
 * -Wl,--wrap=esp_event_loop_create_default in this component's
 * CMakeLists.txt.
 */

#include <stdbool.h>
#include "esp_err.h"

extern esp_err_t __real_esp_netif_init(void);
extern esp_err_t __real_esp_event_loop_create_default(void);

esp_err_t __wrap_esp_netif_init(void)
{
    static bool s_done = false;
    if (s_done) {
        return ESP_OK;
    }
    esp_err_t ret = __real_esp_netif_init();
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_done = true;
        return ESP_OK;
    }
    return ret;
}

esp_err_t __wrap_esp_event_loop_create_default(void)
{
    static bool s_done = false;
    if (s_done) {
        return ESP_OK;
    }
    esp_err_t ret = __real_esp_event_loop_create_default();
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_done = true;
        return ESP_OK;
    }
    return ret;
}
