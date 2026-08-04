/* SPDX-License-Identifier: Apache-2.0 */

#include "sdkconfig.h"

#if CONFIG_ESP_HOSTED_CP

#include "esp_err.h"
#include "eh_cp.h"

esp_err_t esp_hosted_init(void);
esp_err_t esp_hosted_deinit(void);

esp_err_t esp_hosted_init(void)   { return eh_cp_init(); }
esp_err_t esp_hosted_deinit(void) { return eh_cp_deinit(); }

#if CONFIG_ESP_HOSTED_AUTO_CALL_INIT_BEFORE_APP_MAIN && defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void eh_cp_auto_init_task(void *arg)
{
    (void)arg;
    esp_hosted_init();
    vTaskDelete(NULL);
}

void eh_cp_auto_init_ctor(void);
__attribute__((constructor)) void eh_cp_auto_init_ctor(void)
{
    xTaskCreate(eh_cp_auto_init_task, "eh_cp_init", 4096, NULL, 5, NULL);
}
#endif

#endif
