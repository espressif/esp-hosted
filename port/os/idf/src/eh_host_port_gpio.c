/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* SPDX-License-Identifier: Apache-2.0 */
/* IDF GPIO. ISR service installed lazily on first intr_enable. */

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_gpio.h"
#include "eh_host_port_tags.h"

#if EH_HOST_PORT_HAS_GPIO

#ifdef ESP_PLATFORM
#include <sys/queue.h>
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "eh_host_port_gpio";

static gpio_mode_t to_idf_mode(eh_host_port_gpio_dir_t d)
{
    switch (d) {
    case EH_HOST_PORT_GPIO_DIR_INPUT:        return GPIO_MODE_INPUT;
    case EH_HOST_PORT_GPIO_DIR_OUTPUT:       return GPIO_MODE_OUTPUT;
    case EH_HOST_PORT_GPIO_DIR_INPUT_OUTPUT: return GPIO_MODE_INPUT_OUTPUT;
    default:                                 return GPIO_MODE_INPUT;
    }
}

eh_host_port_err_t eh_host_port_gpio_config(const eh_host_port_gpio_config_cfg_t *cfg)
{
    if (!cfg) return EH_HOST_PORT_ERR_INVAL;
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << cfg->gpio.pin),
        .mode         = to_idf_mode(cfg->dir),
        .pull_up_en   = (cfg->pull == EH_HOST_PORT_GPIO_PULL_UP),
        .pull_down_en = (cfg->pull == EH_HOST_PORT_GPIO_PULL_DOWN),
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return (gpio_config(&c) == ESP_OK) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}

eh_host_port_err_t eh_host_port_gpio_set(const eh_host_port_gpio_desc_t *gpio,
                                eh_host_port_gpio_level_t level)
{
    if (!gpio) return EH_HOST_PORT_ERR_INVAL;
    return (gpio_set_level((gpio_num_t)gpio->pin, (uint32_t)level) == ESP_OK)
           ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}

int eh_host_port_gpio_get(const eh_host_port_gpio_desc_t *gpio)
{
    if (!gpio) return EH_HOST_PORT_ERR_INVAL;
    return gpio_get_level((gpio_num_t)gpio->pin);
}

eh_host_port_err_t eh_host_port_gpio_set_drive(const eh_host_port_gpio_desc_t *gpio,
                                      eh_host_port_gpio_drive_t drive)
{
    if (!gpio) return EH_HOST_PORT_ERR_INVAL;
    gpio_drive_cap_t cap;
    switch (drive) {
    case EH_HOST_PORT_GPIO_DRIVE_WEAK:    cap = GPIO_DRIVE_CAP_0; break;
    case EH_HOST_PORT_GPIO_DRIVE_MEDIUM:  cap = GPIO_DRIVE_CAP_1; break;
    case EH_HOST_PORT_GPIO_DRIVE_DEFAULT: cap = GPIO_DRIVE_CAP_2; break;
    case EH_HOST_PORT_GPIO_DRIVE_STRONG:  cap = GPIO_DRIVE_CAP_3; break;
    default:                         return EH_HOST_PORT_ERR_INVAL;
    }
    return (gpio_set_drive_capability((gpio_num_t)gpio->pin, cap) == ESP_OK)
           ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}

#if EH_HOST_PORT_HAS_GPIO_INTR

static int s_isr_svc_installed;

static gpio_int_type_t to_idf_intr(eh_host_port_gpio_intr_mode_t m)
{
    switch (m) {
    case EH_HOST_PORT_GPIO_INTR_POSEDGE:    return GPIO_INTR_POSEDGE;
    case EH_HOST_PORT_GPIO_INTR_NEGEDGE:    return GPIO_INTR_NEGEDGE;
    case EH_HOST_PORT_GPIO_INTR_ANYEDGE:    return GPIO_INTR_ANYEDGE;
    case EH_HOST_PORT_GPIO_INTR_LOW_LEVEL:  return GPIO_INTR_LOW_LEVEL;
    case EH_HOST_PORT_GPIO_INTR_HIGH_LEVEL: return GPIO_INTR_HIGH_LEVEL;
    default:                           return GPIO_INTR_DISABLE;
    }
}

typedef struct {
    eh_host_port_gpio_desc_t     gpio;
    eh_host_port_gpio_isr_cb_t   cb;
    void                   *ctx;
} isr_slot_t;

// list to keep track of gpio isr entries
struct entry {
    isr_slot_t slot;
    SLIST_ENTRY(entry) entries;
};
SLIST_HEAD(slisthead, entry);
static struct slisthead head;
static bool head_inited = false;

static void EH_HOST_PORT_IRAM_ATTR gpio_isr_dispatch(void *arg)
{
    struct entry *s = arg;
    if (s && s->slot.cb) s->slot.cb(&s->slot.gpio, s->slot.ctx);
}

/*
 * Notes:
 * - accept but don't add to list if a cfg reuses a registered gpio, with same cb
 * - reject with error if a cfg reuses a registered gpio, but with a different cb
 */
eh_host_port_err_t eh_host_port_gpio_intr_enable(const eh_host_port_gpio_intr_enable_cfg_t *cfg)
{
    if (!head_inited) {
        SLIST_INIT(&head);
        head_inited = true;
    }

    if (!cfg || !cfg->cb) return EH_HOST_PORT_ERR_INVAL;

    struct entry *s;

    // check if gpio is already in the list
    SLIST_FOREACH(s, &head, entries) {
        if (s->slot.gpio.pin == cfg->gpio.pin) {
            if (s->slot.cb == cfg->cb) {
                ESP_LOGD(TAG, "gpio intr with same cb already registered");
                return EH_HOST_PORT_OK;
            } else {
                ESP_LOGW(TAG, "gpio intr with different cb already registered: rejecting");
                return EH_HOST_PORT_ERR_INVAL;
            }
        }
    }

    // register the gpio and cb
    gpio_set_intr_type((gpio_num_t)cfg->gpio.pin, to_idf_intr(cfg->mode));
    if (!s_isr_svc_installed) {
        if (gpio_install_isr_service(0) == ESP_OK) s_isr_svc_installed = 1;
    }
    s = calloc(1, sizeof(*s));
    if (!s) return EH_HOST_PORT_ERR_NOMEM;
    s->slot.gpio = cfg->gpio;
    s->slot.cb   = cfg->cb;
    s->slot.ctx  = cfg->ctx;
    if (gpio_isr_handler_add((gpio_num_t)cfg->gpio.pin, gpio_isr_dispatch, s) != ESP_OK) {
        free(s);
        return EH_HOST_PORT_ERR;
    }
    // add s to our list
    SLIST_INSERT_HEAD(&head, s, entries);
    return EH_HOST_PORT_OK;
}

eh_host_port_err_t eh_host_port_gpio_intr_disable(const eh_host_port_gpio_desc_t *gpio)
{
    if (!gpio) return EH_HOST_PORT_ERR_INVAL;
    if (!head_inited) return EH_HOST_PORT_OK; // no gpio isrs registered yet

    struct entry *s;

    // iterate through the list and remove the entry, if found
    SLIST_FOREACH(s, &head, entries) {
        if (s->slot.gpio.pin == gpio->pin) {
            gpio_set_intr_type((gpio_num_t)gpio->pin, GPIO_INTR_DISABLE);
            gpio_isr_handler_remove((gpio_num_t)gpio->pin);
            SLIST_REMOVE(&head, s, entry, entries);
            free(s);
            return EH_HOST_PORT_OK;
        }
    }
    ESP_LOGD(TAG, "gpio intr not found: maybe already unregistered");
    return EH_HOST_PORT_OK; // gpio isr not found in list
}

#endif /* EH_HOST_PORT_HAS_GPIO_INTR */

#if EH_HOST_PORT_HAS_GPIO_HOLD
eh_host_port_err_t eh_host_port_gpio_hold(const eh_host_port_gpio_desc_t *gpio, bool hold)
{
    if (!gpio) return EH_HOST_PORT_ERR_INVAL;
    esp_err_t rc = hold ? gpio_hold_en((gpio_num_t)gpio->pin)
                        : gpio_hold_dis((gpio_num_t)gpio->pin);
    return (rc == ESP_OK) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR;
}
#endif

#endif /* ESP_PLATFORM */

#endif /* EH_HOST_PORT_HAS_GPIO */
