/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ESP_SYSTEM_H__
#define __ESP_SYSTEM_H__

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shutdown handlers — IDF runs them on esp_restart(). On Linux there's
 * no analogous facility (the OS process exits via _exit/atexit), so the
 * registration is a no-op. Keeps IDF-lifted code compiling without
 * #ifdefs at the call site. */
typedef void (*shutdown_handler_t)(void);

static inline esp_err_t esp_register_shutdown_handler(shutdown_handler_t h)
{
    (void)h;
    return ESP_OK;
}

static inline esp_err_t esp_unregister_shutdown_handler(shutdown_handler_t h)
{
    (void)h;
    return ESP_OK;
}

/**
 * @brief Reset reasons
 */
typedef enum {
    ESP_RST_UNKNOWN,    //!< Reset reason can not be determined
    ESP_RST_POWERON,    //!< Reset due to power-on event
    ESP_RST_EXT,        //!< Reset by external pin (not applicable for ESP32)
    ESP_RST_SW,         //!< Software reset via esp_restart
    ESP_RST_PANIC,      //!< Software reset due to exception/panic
    ESP_RST_INT_WDT,    //!< Reset (software or hardware) due to interrupt watchdog
    ESP_RST_TASK_WDT,   //!< Reset due to task watchdog
    ESP_RST_WDT,        //!< Reset due to other watchdogs
    ESP_RST_DEEPSLEEP,  //!< Reset after exiting deep sleep mode
    ESP_RST_BROWNOUT,   //!< Brownout reset (software or hardware)
    ESP_RST_SDIO,       //!< Reset over SDIO
    ESP_RST_USB,        //!< Reset by USB peripheral
    ESP_RST_JTAG,       //!< Reset by JTAG
    ESP_RST_EFUSE,      //!< Reset due to efuse error
    ESP_RST_PWR_GLITCH, //!< Reset due to power glitch detected
    ESP_RST_CPU_LOCKUP, //!< Reset due to CPU lock up (double exception)
} esp_reset_reason_t;

#ifdef __cplusplus
}
#endif

#endif /* __ESP_SYSTEM_H__ */
