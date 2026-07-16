/* SPDX-License-Identifier: Apache-2.0 */
/* IDF power/reset/sleep. reset_slave pulses RST GPIO; returns
 * NOT_CONFIGURED if no pin wired. power_save_* uses esp_pm. */

#include "eh_host_port_master_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_power.h"

#if EH_HOST_PORT_HAS_POWER

#ifdef ESP_PLATFORM
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"           /* SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP */

#include "eh_host_transport_config.h"

static const char *EH_HOST_PORT_RESET_TAG = "eh_host_port_reset";

/* Wake-mode enum names differ across IDF revisions. */
#if defined(ESP_SLEEP_GPIO_WAKEUP_GPIO_HIGH) && defined(ESP_SLEEP_GPIO_WAKEUP_GPIO_LOW)
#  define EH_PORT_SLEEP_WAKE_HIGH ESP_SLEEP_GPIO_WAKEUP_GPIO_HIGH
#  define EH_PORT_SLEEP_WAKE_LOW  ESP_SLEEP_GPIO_WAKEUP_GPIO_LOW
#else
#  define EH_PORT_SLEEP_WAKE_HIGH ESP_GPIO_WAKEUP_GPIO_HIGH
#  define EH_PORT_SLEEP_WAKE_LOW  ESP_GPIO_WAKEUP_GPIO_LOW
#endif

#define EH_HOST_PORT_RESET_BASELINE_MS     10
#define EH_HOST_PORT_RESET_PULSE_MS        10


eh_host_port_err_t eh_host_port_reset_slave(void)
{
    eh_gpio_pin_t rst = { .port = NULL, .pin = -1 };
    eh_host_transport_err_t terr = eh_host_transport_get_reset_config(&rst);
    if (terr != EH_HOST_TRANSPORT_RC_OK) {
        ESP_LOGE(EH_HOST_PORT_RESET_TAG,
                 "transport_get_reset_config rc=%d", (int)terr);
        return EH_HOST_PORT_ERR;
    }
    if (rst.pin < 0) {
        /* integrator opted out of in-band reset; not a failure */
        ESP_LOGI(EH_HOST_PORT_RESET_TAG,
                 "RST pin not configured (-1); skipping slave reset");
        return EH_HOST_PORT_ERR_NOT_CONFIGURED;
    }

    const gpio_num_t pin = (gpio_num_t)rst.pin;

    ESP_LOGV(EH_HOST_PORT_RESET_TAG, "Resetting slave on pin %d (active=%d)",
             (int)pin, EH_HOST_PORT_RESET_VAL_ACTIVE);

    /* push-pull output before level-set so first edge is observable */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t rc = gpio_config(&io);
    if (rc != ESP_OK) {
        ESP_LOGE(EH_HOST_PORT_RESET_TAG, "gpio_config(%d) rc=%d", (int)pin, (int)rc);
        return EH_HOST_PORT_ERR;
    }

    gpio_set_level(pin, EH_HOST_PORT_RESET_VAL_INACTIVE);
    vTaskDelay(pdMS_TO_TICKS(EH_HOST_PORT_RESET_BASELINE_MS));
    gpio_set_level(pin, EH_HOST_PORT_RESET_VAL_ACTIVE);
    vTaskDelay(pdMS_TO_TICKS(EH_HOST_PORT_RESET_PULSE_MS));
    gpio_set_level(pin, EH_HOST_PORT_RESET_VAL_INACTIVE);
    vTaskDelay(pdMS_TO_TICKS(EH_HOST_PORT_RESET_POST_DELAY_MS));

    ESP_LOGD(EH_HOST_PORT_RESET_TAG,
             "reset done pin=%d parked=%d (active=%d inactive=%d)",
             (int)pin, gpio_get_level(pin),
             EH_HOST_PORT_RESET_VAL_ACTIVE, EH_HOST_PORT_RESET_VAL_INACTIVE);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_restart_host(void)
{
    ESP_LOGE("eh_host_xport", "TRANSPORT_FAILURE: restarting host");
    esp_restart();
    return EH_HOST_PORT_OK; /* unreachable */
}

eh_host_port_err_t eh_host_port_power_init(void) { return EH_HOST_PORT_OK; }

#if EH_HOST_PORT_HAS_POWER_SAVE
#include "esp_sleep.h"

eh_host_port_err_t eh_host_port_power_save_config(const eh_host_port_power_save_config_cfg_t *cfg)
{
    (void)cfg;
    /* policy wiring is app-specific; enter() applies */
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_power_save_enter(eh_host_port_ps_type_t ps_type)
{
    switch (ps_type) {
    case EH_HOST_PORT_PS_LIGHT_SLEEP:
        return (esp_light_sleep_start() == ESP_OK) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
    case EH_HOST_PORT_PS_DEEP_SLEEP:
        /* wake source must be armed first or chip never returns */
        esp_deep_sleep_start();
        return EH_HOST_PORT_OK; /* unreachable */
    case EH_HOST_PORT_PS_NONE:
    case EH_HOST_PORT_PS_CUSTOM:
    default:
        return EH_HOST_PORT_ERR_NOSYS;
    }
}

eh_host_port_err_t eh_host_port_power_save_wakeup_gpio_config(
        eh_host_port_ps_type_t ps_type,
        int                    gpio,
        int                    wake_level)
{
    if (ps_type != EH_HOST_PORT_PS_DEEP_SLEEP) {
        /* TODO: light-sleep needs gpio_wakeup_* — not wired */
        return EH_HOST_PORT_ERR_NOSYS;
    }
    if (gpio < 0) {
        /* opted out of GPIO wakeup */
        return EH_HOST_PORT_OK;
    }
    wake_level = wake_level ? 1 : 0;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = wake_level ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = wake_level ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t rc = gpio_config(&cfg);
    if (rc != ESP_OK) {
        ESP_LOGE(EH_HOST_PORT_RESET_TAG,
                 "wakeup_gpio_config: gpio_config(%d) rc=%d", gpio, rc);
        return EH_HOST_PORT_ERR;
    }
    /* pre-condition output latch to anti-level */
    gpio_set_level((gpio_num_t)gpio, wake_level ? 0 : 1);

#if SOC_GPIO_SUPPORT_HP_PERIPH_PD_SLEEP_WAKEUP
    /* Newer IDF: deep-sleep/periph-PD GPIO wake API. */
    rc = esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
            1ULL << gpio,
            wake_level ? EH_PORT_SLEEP_WAKE_HIGH : EH_PORT_SLEEP_WAKE_LOW);
#elif SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    /* Older IDF: per-GPIO deep-sleep wake API. */
    rc = esp_deep_sleep_enable_gpio_wakeup(
            1ULL << gpio,
            wake_level ? ESP_GPIO_WAKEUP_GPIO_HIGH : ESP_GPIO_WAKEUP_GPIO_LOW);
#else
    /* ESP32 / S2 / S3 — no per-GPIO deep-sleep wakeup API; fall back
     * to ext1 (mask + mode). Same observable behaviour for single-pin
     * setups; some targets require RTC-IO-capable pins. */
    rc = esp_sleep_enable_ext1_wakeup(
            1ULL << gpio,
            wake_level ? ESP_EXT1_WAKEUP_ANY_HIGH : ESP_EXT1_WAKEUP_ALL_LOW);
#endif
    if (rc != ESP_OK) {
        ESP_LOGE(EH_HOST_PORT_RESET_TAG,
                 "deep-sleep gpio-wakeup arm (pin=%d, level=%d) rc=%d",
                 gpio, wake_level, rc);
        return EH_HOST_PORT_ERR;
    }
    ESP_LOGI(EH_HOST_PORT_RESET_TAG,
             "deep-sleep wakeup armed: pin=%d wake_level=%s",
             gpio, wake_level ? "HIGH" : "LOW");
    return EH_HOST_PORT_OK;
}

#if EH_HOST_PORT_HAS_WAKEUP_REASON
eh_host_port_wakeup_reason_t eh_host_port_wakeup_reason_get(void)
{
    if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
        return EH_HOST_PORT_WAKEUP_UNKNOWN;
    }
#if defined(ESP_IDF_VERSION) && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0))
    /* IDF >=6.0 bitmap API: bit-position is BIT(enum_val), not the enum
     * value itself — see esp_hw_support/sleep_modes.c uses of BIT(...). */
    uint32_t causes = (uint32_t)esp_sleep_get_wakeup_causes();
    if (causes == 0U) {
        return EH_HOST_PORT_WAKEUP_UNKNOWN;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) != 0U) {
        return EH_HOST_PORT_WAKEUP_TIMER;
    }
    if ((causes & (BIT(ESP_SLEEP_WAKEUP_EXT0) |
                   BIT(ESP_SLEEP_WAKEUP_EXT1) |
                   BIT(ESP_SLEEP_WAKEUP_GPIO))) != 0U) {
        return EH_HOST_PORT_WAKEUP_GPIO;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_UART)) != 0U) {
        return EH_HOST_PORT_WAKEUP_UART;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_TOUCHPAD)) != 0U) {
        return EH_HOST_PORT_WAKEUP_TOUCH;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_BT)) != 0U) {
        return EH_HOST_PORT_WAKEUP_BT;
    }
    if ((causes & BIT(ESP_SLEEP_WAKEUP_WIFI)) != 0U) {
        return EH_HOST_PORT_WAKEUP_WIFI;
    }
    return EH_HOST_PORT_WAKEUP_OTHER;
#else
    /* Fallback for older IDF where only single-cause enum is available. */
    switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_TIMER:      return EH_HOST_PORT_WAKEUP_TIMER;
    case ESP_SLEEP_WAKEUP_EXT0:
    case ESP_SLEEP_WAKEUP_EXT1:
    case ESP_SLEEP_WAKEUP_GPIO:       return EH_HOST_PORT_WAKEUP_GPIO;
    case ESP_SLEEP_WAKEUP_UART:       return EH_HOST_PORT_WAKEUP_UART;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:   return EH_HOST_PORT_WAKEUP_TOUCH;
    case ESP_SLEEP_WAKEUP_BT:         return EH_HOST_PORT_WAKEUP_BT;
    case ESP_SLEEP_WAKEUP_WIFI:       return EH_HOST_PORT_WAKEUP_WIFI;
    case ESP_SLEEP_WAKEUP_UNDEFINED:  return EH_HOST_PORT_WAKEUP_UNKNOWN;
    default:                          return EH_HOST_PORT_WAKEUP_OTHER;
    }
#endif
}
#endif /* EH_HOST_PORT_HAS_WAKEUP_REASON */

#endif /* EH_HOST_PORT_HAS_POWER_SAVE */

#endif /* ESP_PLATFORM */

#endif /* EH_HOST_PORT_HAS_POWER */
