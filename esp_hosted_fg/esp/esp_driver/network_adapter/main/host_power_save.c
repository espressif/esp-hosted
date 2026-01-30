/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "host_power_save.h"
#include "adapter.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"
#include "adapter.h"

static char *TAG = "host_ps";

#if H_HOST_PS_ALLOWED
  SemaphoreHandle_t wakeup_sem;

  uint8_t power_save_on;
  #define GPIO_HOST_WAKEUP (CONFIG_HOST_WAKEUP_GPIO)

  /* Assuming wake-up gpio neg 'level' interrupt */
  #define set_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, 1)
  #define reset_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, 0)

  static void oobTimerCallback( TimerHandle_t xTimer );
  static void (*host_wakeup_cb)(void);
  int64_t host_wakeup_time = 0;
#endif


int is_host_wakeup_needed(interface_buffer_handle_t *buf_handle)
{
	int wakup_needed = 0;
	char reason[100] = "";
#if H_HOST_PS_ALLOWED
	uint8_t *buf_start;

	buf_start = buf_handle->payload;

#if 0
	/* Flow conttrol packet cannot miss */
	if (buf_handle->flow_ctl_en) {
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

			/* User can optionally parse frame/packet, to selectively forward this packet to host while in power save, or drop it here.
			 *
			 * Additionally, if network split is enabled, nw_split_router.c can also
			 * *selectively* filter packets on criteria which can decide if packet should be :
			 * a. consumed locally at coprocessor OR
			 * b. drop OR
			 * c. forwarding it to host
			 *
			 * Network Split criteria (if enabled) applied before reaching her.
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


void host_power_save_init(void (*fn_host_wakeup_cb)(void))
{
#if H_HOST_PS_ALLOWED
	/* Configuration for the OOB line */
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_OUTPUT,
		.pin_bit_mask=(1ULL<<GPIO_HOST_WAKEUP)
	};


	ESP_LOGI(TAG, "Host wakeup: IO%u, level:High", GPIO_HOST_WAKEUP);
	gpio_config(&io_conf);
	reset_host_wakeup_gpio();

	assert(wakeup_sem = xSemaphoreCreateBinary());
	xSemaphoreGive(wakeup_sem);
	//xSemaphoreTake(wakeup_sem, 0);
	host_wakeup_cb = fn_host_wakeup_cb;
#endif
}

void host_power_save_deinit(void)
{
#if H_HOST_PS_ALLOWED
	if (wakeup_sem) {
		/* Dummy take and give sema before deleting it */
		xSemaphoreTake(wakeup_sem, portMAX_DELAY);
		xSemaphoreGive(wakeup_sem);
		vSemaphoreDelete(wakeup_sem);
		wakeup_sem = NULL;
	}
#endif
}

#define GET_CURR_TIME_IN_MS() esp_timer_get_time()/1000
int wakeup_host(uint32_t timeout_ms)
{
#if H_HOST_PS_ALLOWED
	TimerHandle_t xTimer = NULL;
	esp_err_t ret = ESP_OK;
	uint64_t start_time = GET_CURR_TIME_IN_MS();
	uint8_t wakeup_success = 0;

	ESP_LOGI(TAG, "WAKE UP Host!!!!!\n");

	do {
		set_host_wakeup_gpio();
		xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(10) , pdFALSE, 0, oobTimerCallback);
		if (xTimer == NULL) {
			ESP_LOGE(TAG, "Failed to create timer for host wakeup");
			break;
		}

		ret = xTimerStart(xTimer, 0);
		if (ret != pdPASS) {
			ESP_LOGE(TAG, "Failed to start timer for host wakeup");
			break;
		}

		if (wakeup_sem) {
			/* wait for host resume */
			ret = xSemaphoreTake(wakeup_sem, pdMS_TO_TICKS(100));

			if (ret == pdPASS) {
				/* usleep(100*1000);*/
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

	return wakeup_success;

#else
	return 1;
#endif
}

void host_power_save_alert(uint32_t ps_evt)
{
#if H_HOST_PS_ALLOWED
	BaseType_t do_yeild = pdFALSE;

	if (ESP_POWER_SAVE_ON == ps_evt) {
		ESP_EARLY_LOGI(TAG, "Host Sleep");
		if (wakeup_sem) {
			/* Host sleeping */
			xSemaphoreTakeFromISR(wakeup_sem, &do_yeild);
		}
		power_save_on = 1;

	} else if (ESP_POWER_SAVE_OFF == ps_evt) {
		ESP_EARLY_LOGI(TAG, "Host Awake");
		power_save_on = 0;

		/* Update wakeup timestamp */
		host_wakeup_time = esp_timer_get_time() / 1000; /* Convert to ms */

		if (host_wakeup_cb) {
			host_wakeup_cb();
		}
		if (wakeup_sem) {
			xSemaphoreGiveFromISR(wakeup_sem, &do_yeild);
		}
	} else {
		ESP_EARLY_LOGI(TAG, "Ignore event[%u]", ps_evt);
	}

	if (do_yeild == pdTRUE) {
		portYIELD_FROM_ISR();
	}
#endif
}

#if H_HOST_PS_ALLOWED
static void oobTimerCallback( TimerHandle_t xTimer )
{
    xTimerDelete(xTimer, 0);
	reset_host_wakeup_gpio();
}
#endif

