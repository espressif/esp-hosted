/*
 * SPDX-FileCopyrightText: 2021-2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_feat_host_ps_internal.h"
#include "eh_caps.h"
#include "eh_interface.h"
//#include "eh_cp_extension.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"
#include "eh_transport_cp.h"
/* host_power_save.c — no ops registry dependency */
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_wifi.h"
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split.h"
#endif

static char *TAG = "host_ps";

#if EH_CP_FEAT_HOST_PS_READY

  static uint8_t power_save_on;

int is_host_power_saving(void)
{
	return power_save_on;
}

  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	SemaphoreHandle_t wakeup_sem;

	#define GPIO_HOST_WAKEUP (EH_CP_FEAT_HOST_PS_WAKEUP_GPIO)
	#define GPIO_HOST_WAKEUP_LEVEL (EH_CP_FEAT_HOST_PS_WAKEUP_GPIO_LEVEL)

	#define set_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL)
	#define reset_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, !GPIO_HOST_WAKEUP_LEVEL)
  #endif
  static void (*host_wakeup_cb)(void);
#endif

extern interface_context_t *if_context;
extern interface_handle_t *if_handle;

int is_host_wakeup_needed(interface_buffer_handle_t *buf_handle)
{
	int wakup_needed = 0;
	char reason[100] = "";
#if EH_CP_FEAT_HOST_PS_READY
	/* Host is awake — no wakeup needed */
	if (!power_save_on) {
		return 0;
	}

	uint8_t *buf_start;

	buf_start = buf_handle->payload;


#if EH_CP_TRANSPORT_SPI_HD || EH_CP_TRANSPORT_UART || EH_CP_TRANSPORT_SPI
	/* Flow control packet cannot miss */
	if (buf_handle->throttle_cmd) {
		strlcpy(reason, "flow_ctl_pkt", sizeof(reason));
		wakup_needed = 1;
		goto end;
	}
#endif

	if (!buf_start) {
		/* Do not wake up */
		strlcpy(reason, "NULL_TxBuff", sizeof(reason));
		wakup_needed = 0;
		goto end;
	}

	/* Wake up for serial msg */
	switch (buf_handle->if_type) {

		case ESP_SERIAL_IF:
			  strlcpy(reason, "serial tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_HCI_IF:
			  strlcpy(reason, "bt tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_PRIV_IF:
			  strlcpy(reason, "priv tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_TEST_IF:
			  strlcpy(reason, "test tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_STA_IF:

			  /* TODO: parse packet if lwip split not configured.
			   * Decide if packets need to reach to host or not
			   **/
			  strlcpy(reason, "sta tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_AP_IF:
			  strlcpy(reason, "ap tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;
	}

end:
#else
	strlcpy(reason, "host_ps_disabled", sizeof(reason));
	wakup_needed = 0;
#endif

	if (wakup_needed) {
		ESP_LOGI(TAG, "Wakeup needed, reason %s", reason);
	} else {
		ESP_LOGI(TAG, "Wakeup not needed");
	}
	return wakup_needed;
}


int host_power_save_init(void (*fn_host_wakeup_cb)(void))
{
#if EH_CP_FEAT_HOST_PS_READY
#if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	assert(GPIO_HOST_WAKEUP != -1);
	/* Configuration for the OOB line */
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << GPIO_HOST_WAKEUP),
	};

	ESP_LOGI(TAG, "Host wakeup: IO%u, level:%u", GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP));
	esp_err_t gpio_ret = gpio_config(&io_conf);
	ESP_LOGI(TAG, "Host wakeup gpio_config ret=%d", gpio_ret);

	/* Configure pull based on wakeup level */
	if (GPIO_HOST_WAKEUP_LEVEL) {
		gpio_pulldown_en(GPIO_HOST_WAKEUP);
	} else {
		gpio_pullup_en(GPIO_HOST_WAKEUP);
	}

	reset_host_wakeup_gpio();
	ESP_LOGI(TAG, "Host wakeup: IO%u, level:%u (active %s)",
	         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP),
	         GPIO_HOST_WAKEUP_LEVEL ? "HIGH" : "LOW");

	/* Debug: drive high once and read back */
	gpio_set_level(GPIO_HOST_WAKEUP, 1);
	ESP_LOGI(TAG, "Host wakeup: IO%u drive=1 read=%u", GPIO_HOST_WAKEUP,
	         gpio_get_level(GPIO_HOST_WAKEUP));
	gpio_set_level(GPIO_HOST_WAKEUP, 0);
	ESP_LOGI(TAG, "Host wakeup: IO%u drive=0 read=%u", GPIO_HOST_WAKEUP,
	         gpio_get_level(GPIO_HOST_WAKEUP));

	assert(wakeup_sem = xSemaphoreCreateBinary());
	xSemaphoreGive(wakeup_sem);
#endif

	host_wakeup_cb = fn_host_wakeup_cb;
#endif
	return 0;
}

int host_power_save_set_wakeup_cb(void (*fn_host_wakeup_cb)(void))
{
#if EH_CP_FEAT_HOST_PS_READY
	host_wakeup_cb = fn_host_wakeup_cb;
	return 0;
#else
	(void)fn_host_wakeup_cb;
	return 0;
#endif
}

int host_power_save_deinit(void)
{
#if EH_CP_FEAT_HOST_PS_READY
#if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	if (wakeup_sem) {
		xSemaphoreTake(wakeup_sem, portMAX_DELAY);
		xSemaphoreGive(wakeup_sem);
		vSemaphoreDelete(wakeup_sem);
		wakeup_sem = NULL;
	}
#endif
	host_wakeup_cb = NULL;
#endif
	return 0;
}

#define GET_CURR_TIME_IN_MS() esp_timer_get_time()/100

/* Add new callback function for ESP Timer */
#if EH_CP_FEAT_HOST_PS_READY && EH_CP_FEAT_HOST_PS_DEEP_SLEEP
static void clean_wakeup_gpio_timer_cb(void* arg)
{
	reset_host_wakeup_gpio();
	ESP_EARLY_LOGI(TAG, "Cleared wakeup gpio, IO%u", GPIO_HOST_WAKEUP);
}
#endif

int wakeup_host_mandate(uint32_t timeout_ms)
{
#if EH_CP_FEAT_HOST_PS_READY && EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	esp_timer_handle_t timer = NULL;
	esp_err_t ret = ESP_OK;
	uint64_t start_time = GET_CURR_TIME_IN_MS();
	uint8_t wakeup_success = 0;
	esp_timer_create_args_t timer_args = {
		.callback = &clean_wakeup_gpio_timer_cb,
		.name = "host_wakeup_timer",
	};

	ESP_LOGI(TAG, "WAKE UP Host!!!!!\n");
	ESP_LOGW(TAG, "wakeup_start: gpio=%u level=%u power_save_on=%u if_state=%u",
	         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP),
	         power_save_on, if_handle ? if_handle->state : 0);

	do {
		if (!power_save_on) {
			ESP_LOGI(TAG, "Host awake (power_save_on=0), stop wakeup loop");
			break;
		}
		gpio_set_direction(GPIO_HOST_WAKEUP, GPIO_MODE_OUTPUT);
		ESP_LOGW(TAG, "wakeup_toggle: set gpio=%u level(before)=%u",
		         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP));
		esp_err_t set_ret = gpio_set_level(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL);
		ESP_LOGW(TAG, "wakeup_toggle: gpio_set_level ret=%d", set_ret);
		ESP_LOGW(TAG, "wakeup_toggle: set gpio=%u level(after)=%u",
		         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP));

		/* Create ESP Timer once; reuse each pulse */
		if (!timer) {
			ret = esp_timer_create(&timer_args, &timer);
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "Failed to create timer for host wakeup");
				break;
			}
		}

		/* Start one-shot timer (10ms) */
		ret = esp_timer_start_once(timer, 10000); /* 10ms in microseconds */
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to start timer for host wakeup");
			esp_timer_delete(timer);
			break;
		}
		vTaskDelay(100);

		if (wakeup_sem) {
			/* wait for host resume */
			ret = xSemaphoreTake(wakeup_sem, pdMS_TO_TICKS(100));
			ESP_LOGW(TAG, "wakeup_sem: take ret=%d", ret);
			if (ret == pdPASS) {
				ESP_LOGW(TAG, "wakeup_sem: give");
				xSemaphoreGive(wakeup_sem);
				wakeup_success = 1;
				break;
			}
		}

		if (GET_CURR_TIME_IN_MS() - start_time > timeout_ms) {
			/* timeout */
			ESP_LOGI(TAG, "%s:%u timeout Curr:%llu start:%llu timeout:%lu",
					__func__,__LINE__, GET_CURR_TIME_IN_MS(), start_time, timeout_ms);
			break;
		}

	} while (1);

	/* Clean up timer if it's still active */
	if (timer) {
		esp_timer_stop(timer);
		esp_timer_delete(timer);
	}

	return wakeup_success;

#else
	return 1;
#endif
}

int wakeup_host(uint32_t timeout_ms)
{
#if EH_CP_FEAT_HOST_PS_READY

	int wakeup_success = 0;

	if(!power_save_on) {
		return 1;
	}

	if (!if_handle || !if_context) {
		ESP_LOGE(TAG, "Failed to wakeup, if_handle or if_context is NULL");
		return 0;
	}

	ESP_LOGI(TAG, "if_handle->state: %u", if_handle->state);
	if (if_handle->state < DEACTIVE) {
		ESP_LOGI(TAG, "%s:%u Re-Initializing driver\n", __func__, __LINE__);

		/* host wakeup mandated in sdio init */
		wakeup_success = 1;
		if_handle = if_context->if_ops->init();
		if (!if_handle) {
			ESP_LOGE(TAG, "%s:%u Failed to initialize driver\n", __func__, __LINE__);
			return ESP_FAIL;
		}
	}

	if (power_save_on) {
		wakeup_success = wakeup_host_mandate(timeout_ms);
		ESP_LOGI(TAG, "host %s woke up", power_save_on ? "not" : "");
	}

	return wakeup_success;
#else
	return 1;
#endif
}

int host_power_save_alert(uint32_t ps_evt)
{
#if EH_CP_FEAT_HOST_PS_READY
	/* Running in interrupt context - Keep it short and simple */
	BaseType_t do_yeild = pdFALSE;

	if (ESP_POWER_SAVE_ON == ps_evt) {
		ESP_EARLY_LOGI(TAG, "Host Sleep");
  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
		if (wakeup_sem) {
			/* Host sleeping */
			if (xPortInIsrContext()) {
				xSemaphoreTakeFromISR(wakeup_sem, &do_yeild);
			} else {
				xSemaphoreTake(wakeup_sem, 0);
			}
		}
  #endif
		power_save_on = 1;

		if (!if_handle || !if_context || if_handle->state < DEACTIVE) {
			ESP_EARLY_LOGE(TAG, "%s:%u Failed to bring down transport", __func__, __LINE__);
		}

		if (if_handle->state >= DEACTIVE) {
			if (!if_context->if_ops || !if_context->if_ops->deinit) {
				ESP_EARLY_LOGI(TAG, "%s:%u if_context->if_ops->deinit not available", __func__, __LINE__);
			} else {
				ESP_EARLY_LOGI(TAG, "%s:%u Deinitializing driver", __func__, __LINE__);
				if_context->if_ops->deinit(if_handle);
				/* if_handle->state would be changed to DEINIT */
			}
		}
	} else if ((ESP_POWER_SAVE_OFF == ps_evt) || (ESP_OPEN_DATA_PATH == ps_evt)) {
		ESP_EARLY_LOGI(TAG, "Host Awake, transport state: %u", if_handle->state);

		power_save_on = 0;
#if EH_CP_FEAT_WIFI_READY
		eh_cp_feat_wifi_replay_connected_event_if_needed();
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
		eh_cp_feat_nw_split_replay_status_if_needed();
#endif
		if (host_wakeup_cb) {
			host_wakeup_cb();
		}
  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
		if (wakeup_sem) {
			ESP_EARLY_LOGI(TAG, "Giving wakeup semaphore");
			if (xPortInIsrContext()) {
				xSemaphoreGiveFromISR(wakeup_sem, &do_yeild);
			} else {
				xSemaphoreGive(wakeup_sem);
			}
		}
  #endif
	} else {
		ESP_EARLY_LOGI(TAG, "Ignore event[%u]", ps_evt);
	}

	if (do_yeild == pdTRUE && xPortInIsrContext()) {
		portYIELD_FROM_ISR();
	}
#endif
	return 0;
}
