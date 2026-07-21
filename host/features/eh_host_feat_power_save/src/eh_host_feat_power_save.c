/* SPDX-License-Identifier: Apache-2.0 */
/* Host-side power-save lifecycle. */

#include <errno.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "eh_host_port_master_config.h"
#include "eh_host_power_save.h"
#include "esp_err.h"
#include "esp_log.h"

#if EH_HOST_TYPE_MCU
#include "eh_host_mcu_transport.h"
#include "eh_host_port_err.h"
#include "eh_host_port_gpio.h"
#include "eh_host_port_power.h"
#include "eh_host_port_tags.h"
#include "eh_host_port_task.h"
#include "esp_timer.h"
#include "eh_host_transport_config.h"
#endif

#define TAG "eh_ps"

static volatile int s_init_done = 0;
static volatile int s_active    = 0;

#if EH_HOST_TYPE_MCU
static esp_timer_handle_t           s_auto_enter_timer = NULL;
static eh_host_power_save_type_t  s_auto_enter_type  = EH_HOST_POWER_SAVE_TYPE_LIGHT_SLEEP;

/* ISR state; 500ms grace filters sleep/wake glitches. */
static volatile uint32_t s_last_wakeup_time_ms = 0;
static volatile uint8_t  s_reset_in_progress   = 0;
static eh_host_port_gpio_desc_t s_wakeup_gpio_desc  = { 0 };
static int                 s_wakeup_armed      = 0;
#endif

int eh_host_power_save_enabled(void)
{
#if EH_HOST_FEAT_POWER_SAVE_READY && EH_HOST_TYPE_MCU
    return 1;
#else
    return 0;
#endif
}

#if EH_HOST_TYPE_MCU
static void EH_HOST_PORT_IRAM_ATTR wakeup_gpio_isr_handler(const eh_host_port_gpio_desc_t *gpio,
                                                       void *ctx)
{
    (void)ctx;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_last_wakeup_time_ms < 500) return;
    if (s_active || s_reset_in_progress) return;
    int level = eh_host_port_gpio_get(gpio);
    if (level < 0 || level != EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL) return;
    s_reset_in_progress = 1;
    eh_host_port_restart_host();
}

static int wakeup_gpio_arm(void)
{
    int pin = EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO;
    if (pin < 0 || s_wakeup_armed) return 0;

    s_wakeup_gpio_desc.port = 0;
    s_wakeup_gpio_desc.pin  = (uint32_t)pin;

    eh_host_port_gpio_config_cfg_t in_cfg = {
        .gpio  = s_wakeup_gpio_desc,
        .dir   = EH_HOST_PORT_GPIO_DIR_INPUT,
        .pull  = (EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL == 1)
                   ? EH_HOST_PORT_GPIO_PULL_DOWN : EH_HOST_PORT_GPIO_PULL_UP,
        .flags = 0,
    };
    if (eh_host_port_gpio_config(&in_cfg) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "wakeup gpio_config(pin=%d) failed", pin);
        return -EIO;
    }

    eh_host_port_gpio_intr_enable_cfg_t cfg = {
        .gpio  = s_wakeup_gpio_desc,
        .mode  = (EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL == 1)
                   ? EH_HOST_PORT_GPIO_INTR_POSEDGE : EH_HOST_PORT_GPIO_INTR_NEGEDGE,
        .cb    = wakeup_gpio_isr_handler,
        .ctx   = NULL,
        .flags = 0,
    };
    if (eh_host_port_gpio_intr_enable(&cfg) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "wakeup gpio_intr_enable(pin=%d) failed", pin);
        return -EIO;
    }
    s_wakeup_armed = 1;
    ESP_LOGI(TAG, "Armed wakeup GPIO %d (active=%d)",
             pin, EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL);
    return 0;
}

static void wakeup_gpio_disarm(void)
{
    if (!s_wakeup_armed) return;
    eh_host_port_gpio_intr_disable(&s_wakeup_gpio_desc);
    s_wakeup_armed = 0;
}
#endif /* EH_HOST_TYPE_MCU */

int eh_host_power_save_init(void)
{
#if EH_HOST_TYPE_MCU
    if (s_init_done) return 0;

    eh_host_port_err_t prc = eh_host_port_power_init();
    if (prc != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "port_power_init: %d", (int)prc);
        return -EIO;
    }
    wakeup_gpio_arm();    /* best-effort */
    s_init_done = 1;

    /* Bus driver already did the wire-level slave notify during bringup. */
    if (eh_host_port_wakeup_reason_get() != EH_HOST_PORT_WAKEUP_UNKNOWN) {
        s_last_wakeup_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }

    return 0;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int eh_host_power_save_deinit(void)
{
#if EH_HOST_TYPE_MCU
    wakeup_gpio_disarm();
    if (s_auto_enter_timer) {
        esp_timer_delete(s_auto_enter_timer);
        s_auto_enter_timer = NULL;
    }
#endif
    s_init_done = 0;
    s_active    = 0;
    return 0;
}

int eh_host_power_saving(void) { return s_active; }

int eh_host_woke_from_power_save(void)
{
#if EH_HOST_TYPE_MCU && EH_HOST_PORT_HAS_WAKEUP_REASON
    return (eh_host_port_wakeup_reason_get() != EH_HOST_PORT_WAKEUP_UNKNOWN) ? 1 : 0;
#else
    return 0;
#endif
}

#if EH_HOST_TYPE_MCU
/* 0 = desc populated, 1 = pin -1 (opt-out), <0 = lookup error. */
static int reset_pin_to_port_desc(eh_host_port_gpio_desc_t *out)
{
    eh_gpio_pin_t pin = { .port = NULL, .pin = -1 };
    if (eh_host_transport_get_reset_config(&pin) != EH_HOST_TRANSPORT_RC_OK) {
        ESP_LOGE(TAG, "transport_get_reset_config failed");
        return -EIO;
    }
    if (pin.pin < 0) return 1;          /* opted out */
    out->port = (uint32_t)(uintptr_t)pin.port;
    out->pin  = (uint32_t)pin.pin;
    return 0;
}
#endif

int eh_host_hold_slave_reset_gpio_pre_power_save(void)
{
#if EH_HOST_TYPE_MCU
    eh_host_port_gpio_desc_t desc = { 0 };
    int rc = reset_pin_to_port_desc(&desc);
    if (rc < 0) return rc;
    if (rc > 0) return 0;               /* integrator opted out */
    eh_host_port_err_t prc = eh_host_port_gpio_hold(&desc, true);
    if (prc != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "gpio_hold(en, pin=%u): %d", (unsigned)desc.pin, (int)prc);
        return -EIO;
    }
    return 0;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int eh_host_release_slave_reset_gpio_post_wakeup(void)
{
#if EH_HOST_TYPE_MCU
    eh_host_port_gpio_desc_t desc = { 0 };
    int rc = reset_pin_to_port_desc(&desc);
    if (rc < 0) return rc;
    if (rc > 0) return 0;
    eh_host_port_err_t prc = eh_host_port_gpio_hold(&desc, false);
    if (prc != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "gpio_hold(dis, pin=%u): %d", (unsigned)desc.pin, (int)prc);
        return -EIO;
    }
    return 0;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int eh_host_power_save_start(eh_host_power_save_type_t power_save_type)
{
#if EH_HOST_TYPE_MCU
    /* Order matters: slave must be notified BEFORE reset GPIO is held;
     * 50 ms gap then required between hold-reset and deep-sleep entry. */

    if (s_active) {
        return 0;
    }

    if (!s_init_done) {
        ESP_LOGE(TAG, "power-save driver not initialized");
        return -EINVAL;
    }

    /* Only deep sleep is supported; light sleep doesn't pair with CP-always-on. */
    if (power_save_type != EH_HOST_POWER_SAVE_TYPE_DEEP_SLEEP) {
        ESP_LOGE(TAG, "unsupported power-save type: %d (only DEEP_SLEEP)",
                 (int)power_save_type);
        return -EINVAL;
    }

    int rc = eh_host_mcu_transport_inform_slave_ps_enter();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to notify slave: ps_enter rc=%d", rc);
        return -1;
    }

    /* Race guard — wakeup ISR may have just fired. */
    if (s_reset_in_progress) {
        ESP_LOGE(TAG, "reset in progress; aborting PS entry");
        return -1;
    }

    /* Tear down wake-GPIO interrupt before reusing the pin for deep-sleep. */
    wakeup_gpio_disarm();

    if (eh_host_hold_slave_reset_gpio_pre_power_save() != 0) {
        ESP_LOGE(TAG, "hold_slave_reset_gpio failed");
        return -1;
    }

    /* 50 ms gap: slave needs time to commit PS-start before bus+reset change. */
    eh_host_port_task_delay_ms(50);

    eh_host_port_err_t wprc = eh_host_port_power_save_wakeup_gpio_config(
            (eh_host_port_ps_type_t)power_save_type,
            EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO,
            EH_HOST_FEAT_POWER_SAVE_WAKEUP_GPIO_LEVEL);
    if (wprc != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "wakeup_gpio_config rc=%d", (int)wprc);
        eh_host_release_slave_reset_gpio_post_wakeup();
        return -EIO;
    }

    s_active = 1;
    eh_host_port_err_t prc = eh_host_port_power_save_enter((eh_host_port_ps_type_t)power_save_type);

    s_active = 0;
    s_last_wakeup_time_ms = (esp_timer_get_time() / 1000);
    eh_host_release_slave_reset_gpio_post_wakeup();
    eh_host_mcu_transport_inform_slave_ps_exit();

    return (prc == EH_HOST_PORT_OK) ? 0 : -EIO;
#else
    (void)power_save_type;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int eh_host_power_save_stop(void)
{
#if EH_HOST_TYPE_MCU
    /* Do NOT gate on s_init_done: bus driver calls this on wake-from-PS
     * before the auto-init walk reaches power_save_init. With an init
     * guard the CP never learns the host is awake. */
    if (eh_host_mcu_transport_inform_slave_ps_exit() != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "Failed to notify slave, host power save is stopped");
        return -1;
    }

    s_active = 0;
    s_last_wakeup_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return 0;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

#if EH_HOST_TYPE_MCU
static void auto_enter_cb(void *ctx)
{
    (void)ctx;
    if (s_init_done && !s_active) {
        eh_host_power_save_start(s_auto_enter_type);
    }
}
#endif

int eh_host_power_save_timer_start(uint32_t time_ms)
{
#if EH_HOST_TYPE_MCU
    if (!s_init_done) return -EINVAL;
    if (!s_auto_enter_timer) {
        esp_timer_create_args_t cfg = {
            .callback = auto_enter_cb,
            .arg      = NULL,
            .name     = "host_ps_auto",
        };
        if (esp_timer_create(&cfg, &s_auto_enter_timer) != ESP_OK) return -EIO;
    } else {
        esp_timer_stop(s_auto_enter_timer);
    }
    return (esp_timer_start_once(s_auto_enter_timer, (uint64_t)time_ms * 1000) == ESP_OK) ? 0 : -EIO;
#else
    (void)time_ms;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int eh_host_power_save_timer_stop(void)
{
#if EH_HOST_TYPE_MCU
    if (s_auto_enter_timer) esp_timer_stop(s_auto_enter_timer);
    return 0;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

#if EH_HOST_TYPE_MCU && CONFIG_ESP_HOSTED_HOST_FEAT_POWER_SAVE_AUTO_INIT
/* Priority 100: runs early so other features can rely on s_init_done. */
#include "eh_host_auto_init.h"
EH_HOST_FEAT_REGISTER(eh_host_power_save_init,
                      eh_host_power_save_deinit,
                      "power_save", 175);
#endif
