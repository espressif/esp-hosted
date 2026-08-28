/* SPDX-License-Identifier: Apache-2.0 */
/* Optional NVS deep-sleep/wake counters. See app_ps_stats.h for the rationale. */

#include "app_ps_stats.h"

#if CONFIG_APP_PS_NVS_STATS

#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs.h"

static const char *TAG = "app_ps_stats";

#define NS      "ps_stats"
#define K_BOOTS "boots"
#define K_WAKES "wakes"
#define K_CAUSE "cause"

static uint32_t s_boots, s_wakes;
static uint8_t  s_cause;

void app_ps_stats_update_on_boot(void)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    (void)nvs_get_u32(h, K_BOOTS, &s_boots);
    (void)nvs_get_u32(h, K_WAKES, &s_wakes);

    s_boots++;

    /* A defined wake cause means we resumed from deep sleep, not a cold boot. */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        s_wakes++;
        s_cause = (uint8_t)cause;
        (void)nvs_set_u8(h, K_CAUSE, s_cause);
    } else {
        (void)nvs_get_u8(h, K_CAUSE, &s_cause);
    }

    (void)nvs_set_u32(h, K_BOOTS, s_boots);
    (void)nvs_set_u32(h, K_WAKES, s_wakes);
    (void)nvs_commit(h);
    nvs_close(h);
}

void app_ps_stats_report(void)
{
    ESP_LOGI(TAG, "[ps_stats] boots=%lu wakes=%lu last_wake_cause=%u%s",
             (unsigned long)s_boots, (unsigned long)s_wakes, s_cause,
             (s_boots && s_wakes + 1 == s_boots) ? " (first boot was cold)" : "");
}

#endif /* CONFIG_APP_PS_NVS_STATS */
