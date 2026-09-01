/*
 * SPDX-FileCopyrightText: 2021-2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "eh_cp_feat_host_ps_internal.h"
#include "eh_cp_feat_host_ps.h"
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
#include "eh_cp_host_ps_state.h"
/* host_power_save.c — no ops registry dependency */
#if EH_CP_FEAT_WIFI_READY
#include "eh_cp_feat_wifi.h"
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
#include "eh_cp_feat_nw_split.h"
#endif

static char *TAG = "ehcp_host_ps";

#if EH_CP_FEAT_HOST_PS_READY

  #define power_save_on (!eh_cp_host_ps_reachable())

  /* Serialises PS_ON transport teardown against the wakeup re-init path. */
  static SemaphoreHandle_t s_ps_transition_mtx;

  static inline void ps_transition_lock(void)
  {
	if (s_ps_transition_mtx) {
		xSemaphoreTake(s_ps_transition_mtx, portMAX_DELAY);
	}
  }

  static inline void ps_transition_unlock(void)
  {
	if (s_ps_transition_mtx) {
		xSemaphoreGive(s_ps_transition_mtx);
	}
  }

int is_host_power_saving(void)
{
	return power_save_on;
}

  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	SemaphoreHandle_t wakeup_sem;

	#define GPIO_HOST_WAKEUP (EH_CP_FEAT_HOST_PS_WAKEUP_GPIO)
	#define GPIO_HOST_WAKEUP_LEVEL (EH_CP_FEAT_HOST_PS_WAKEUP_GPIO_LEVEL)

	#if EH_CP_FEAT_HOST_PS_WAKEUP_GPIO < 0
	#error "Host power-save deep sleep needs a valid host-wakeup GPIO. Set CONFIG_ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO for your board - it is the only line that wakes a sleeping host, so a default here would silently pick a wrong pin."
	#endif

	#define set_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL)
	#define reset_host_wakeup_gpio() gpio_set_level(GPIO_HOST_WAKEUP, !GPIO_HOST_WAKEUP_LEVEL)

	static esp_err_t configure_host_wakeup_gpio(uint32_t gpio_num, uint8_t level)
	{
		gpio_config_t io_conf = {
			.intr_type    = GPIO_INTR_DISABLE,
			.mode         = GPIO_MODE_OUTPUT,
			.pin_bit_mask = (1ULL << gpio_num),
		};

		esp_err_t ret = gpio_config(&io_conf);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "gpio_config(IO%lu): %d", (unsigned long)gpio_num, ret);
			return ret;
		}
		ret = level ? gpio_pulldown_en(gpio_num) : gpio_pullup_en(gpio_num);
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "pull en (IO%lu, level=%u): %d",
			         (unsigned long)gpio_num, level, ret);
		}
		return ret;
	}
  #endif
  static host_power_save_callbacks_t hps_cbs;
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

			  /* TODO(host-ps): when lwip split is disabled, inspect the
			   * packet and decide whether host wake-up is required. */
			  strlcpy(reason, "sta tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		case ESP_AP_IF:
			  strlcpy(reason, "ap tx msg", sizeof(reason));
			  wakup_needed = 1;
			  goto end;
			  break;

		default:
			  break;   /* unknown if_type: fall through to end (no wake) */
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


#define EH_CP_HOST_PS_BOOT_WAKE_PULSE_MS   1

/* Once, at boot: a CP restart during host sleep otherwise deadlocks - the host
 * wakes only on this line, and we drive it only on traffic it must first cause. */
static void pulse_host_wakeup_on_boot(void)
{
#if EH_CP_FEAT_HOST_PS_WAKE_HOST_ON_CP_BOOT
	static bool done;

	if (done || GPIO_HOST_WAKEUP == -1) {
		return;
	}
	done = true;

	set_host_wakeup_gpio();
	vTaskDelay(pdMS_TO_TICKS(EH_CP_HOST_PS_BOOT_WAKE_PULSE_MS));
	reset_host_wakeup_gpio();
	ESP_LOGI(TAG, "Host wakeup: pulsed IO%u once at our boot", GPIO_HOST_WAKEUP);
#endif
}

/* Wake/reset the host before starting CP communication.
 * This prevents the system from remaining in a boot deadlock if the CP crashes. */
void eh_cp_feat_host_ps_cp__boot_wake_host_early(void)
{
#if EH_CP_FEAT_HOST_PS_READY && EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	if (GPIO_HOST_WAKEUP == -1) {
		return;
	}
	configure_host_wakeup_gpio(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL);
	reset_host_wakeup_gpio();          /* park inactive before the pulse */
	pulse_host_wakeup_on_boot();       /* no-op unless WAKE_HOST_ON_CP_BOOT */
#endif
}


int host_power_save_init(const host_power_save_callbacks_t *cbs)
{
#if EH_CP_FEAT_HOST_PS_READY
#if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	assert(GPIO_HOST_WAKEUP != -1);

	esp_err_t gpio_ret = configure_host_wakeup_gpio(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL);
	ESP_LOGI(TAG, "Host wakeup: IO%u, level:%u (configure ret=%d)",
	         GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL, gpio_ret);

	reset_host_wakeup_gpio();
	ESP_LOGI(TAG, "Host wakeup: IO%u, level:%u (active %s)",
	         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP),
	         GPIO_HOST_WAKEUP_LEVEL ? "HIGH" : "LOW");

	pulse_host_wakeup_on_boot();

	assert(wakeup_sem = xSemaphoreCreateBinary());
	xSemaphoreGive(wakeup_sem);
#endif

	if (!s_ps_transition_mtx) {
		assert(s_ps_transition_mtx = xSemaphoreCreateMutex());
	}

	/* Start worker before publishing ops to avoid the core-feature dependency cycle. */
	eh_cp_feat_host_ps_worker_start();
	{
		static const eh_cp_host_ps_ops_t ops = {
			.wakeup_needed = eh_cp_feat_host_ps_is_host_wakeup_needed,
			.wakeup        = eh_cp_feat_host_ps_wakeup_host,
			.on_ps_event   = eh_cp_feat_host_ps_post_alert,
		};
		eh_cp_host_ps_register_ops(&ops);
	}

	if (cbs) {
		hps_cbs = *cbs;
	} else {
		memset(&hps_cbs, 0, sizeof(hps_cbs));
	}
#endif
	return 0;
}

int host_power_save_set_callbacks(const host_power_save_callbacks_t *cbs)
{
#if EH_CP_FEAT_HOST_PS_READY
	if (cbs) {
		hps_cbs = *cbs;
	} else {
		memset(&hps_cbs, 0, sizeof(hps_cbs));
	}
	return 0;
#else
	(void)cbs;
	return 0;
#endif
}

int host_power_save_deinit(void)
{
#if EH_CP_FEAT_HOST_PS_READY
	eh_cp_feat_host_ps_worker_stop();
#if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
	if (wakeup_sem) {
		xSemaphoreTake(wakeup_sem, portMAX_DELAY);
		xSemaphoreGive(wakeup_sem);
		vSemaphoreDelete(wakeup_sem);
		wakeup_sem = NULL;
	}
#endif
	if (s_ps_transition_mtx) {
		vSemaphoreDelete(s_ps_transition_mtx);
		s_ps_transition_mtx = NULL;
	}
	memset(&hps_cbs, 0, sizeof(hps_cbs));
#endif
	return 0;
}

#define GET_CURR_TIME_IN_MS() (esp_timer_get_time()/1000)

#if EH_CP_FEAT_HOST_PS_READY
/* Cap on how long a wake caller waits for the host to come up before giving up
 * (a stuck wake becomes a counted drop at the caller, never a hang). Covers
 * portMAX_DELAY too. */
#define EH_CP_HOST_PS_WAKE_WAIT_MAX_MS 8000u
#endif

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

	ret = configure_host_wakeup_gpio(GPIO_HOST_WAKEUP, GPIO_HOST_WAKEUP_LEVEL);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "configure_host_wakeup_gpio rc=%d", ret);
		return ESP_FAIL;
	}
	reset_host_wakeup_gpio();

	ESP_LOGW(TAG, "wakeup_start: gpio=%u level=%u power_save_on=%u if_state=%u",
	         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP),
	         power_save_on, if_handle ? if_handle->state : 0);

	do {
		if (!power_save_on) {
			ESP_LOGI(TAG, "Host awake (power_save_on=0), stop wakeup loop");
			break;
		}
		ESP_LOGW(TAG, "wakeup_toggle: set gpio=%u level(before)=%u",
		         GPIO_HOST_WAKEUP, gpio_get_level(GPIO_HOST_WAKEUP));
		esp_err_t set_ret = set_host_wakeup_gpio();
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

	if (!if_handle || !if_context) {
		ESP_LOGE(TAG, "Failed to wakeup, if_handle or if_context is NULL");
		return 0;
	}

	uint64_t start = GET_CURR_TIME_IN_MS();
	uint32_t cap = (timeout_ms > EH_CP_HOST_PS_WAKE_WAIT_MAX_MS)
	                   ? EH_CP_HOST_PS_WAKE_WAIT_MAX_MS : timeout_ms;

	for (;;) {
		ps_transition_lock();

		/* AWAKE: nothing to do (idempotent). */
		if (eh_cp_host_ps_get() == EH_HOST_PS_AWAKE) {
			ps_transition_unlock();
			return 1;
		}

		/* ASLEEP -> WAKING: this caller wins the wake. Re-init a FRESH transport
		 * to match the host, which rebuilt its side on its own wake, then pulse.
		 * The compare-and-set makes exactly one caller the pulse owner. */
		if (eh_cp_host_ps_transition(EH_HOST_PS_ASLEEP, EH_HOST_PS_WAKING)) {
			if (hps_cbs.host_power_save_off_prepare_cb)
				hps_cbs.host_power_save_off_prepare_cb();
			if (if_handle->state < DEACTIVE) {
				if_handle = if_context->if_ops->init();
				if (!if_handle) {
					ps_transition_unlock();
					ESP_LOGE(TAG, "%s:%u Failed to initialize driver", __func__, __LINE__);
					return ESP_FAIL;
				}
				eh_cp_recv_kick();
			}
			ps_transition_unlock();
			return wakeup_host_mandate(timeout_ms);   /* pulse until the host acks */
		}

		/* ANNOUNCED (host still going to sleep) or WAKING (another caller owns
		 * the pulse): do not wake now - converge. Wait briefly and re-check. */
		ps_transition_unlock();
		if (GET_CURR_TIME_IN_MS() - start > cap)
			return 0;                     /* caller drops (counted) */
		vTaskDelay(pdMS_TO_TICKS(20));
	}
#else
	return 1;
#endif
}

int host_power_save_alert(uint32_t ps_evt)
{
#if EH_CP_FEAT_HOST_PS_READY

	if (ESP_POWER_SAVE_ON == ps_evt) {
		ESP_LOGI(TAG, "Host Sleep");

		/* USER CALLBACK: Prepare to enter power save */
		if (hps_cbs.host_power_save_on_prepare_cb) {
			hps_cbs.host_power_save_on_prepare_cb();
		}

  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
		if (wakeup_sem) {
			/* Host sleeping — clear any stale post; non-blocking take. */
			xSemaphoreTake(wakeup_sem, 0);
		}
  #endif
		ps_transition_lock();

		/* Tear down only from ANNOUNCED. If the phase already left ANNOUNCED a
		 * wake is in flight - do nothing (the wake path owns the transport). */
		if (eh_cp_host_ps_get() != EH_HOST_PS_ANNOUNCED) {
			ESP_LOGI(TAG, "wake already in flight, not sleeping");
		} else if (!if_handle || !if_context || if_handle->state < DEACTIVE) {
			ESP_LOGE(TAG, "%s:%u Failed to bring down transport", __func__, __LINE__);
		} else if (!if_context->if_ops || !if_context->if_ops->deinit) {
			ESP_LOGI(TAG, "%s:%u if_context->if_ops->deinit not available", __func__, __LINE__);
		} else {
			ESP_LOGI(TAG, "%s:%u Deinitializing driver", __func__, __LINE__);
			if_context->if_ops->deinit(if_handle);
			/* if_handle->state would be changed to DEINIT */
			if (hps_cbs.host_power_save_on_ready_cb) {
				hps_cbs.host_power_save_on_ready_cb();
			}
			eh_cp_host_ps_transition(EH_HOST_PS_ANNOUNCED, EH_HOST_PS_ASLEEP);
		}
		ps_transition_unlock();
	} else if ((ESP_POWER_SAVE_OFF == ps_evt) || (ESP_OPEN_DATA_PATH == ps_evt)) {
		ESP_LOGI(TAG, "Host Awake, transport state: %u", if_handle->state);

		/* USER CALLBACK: Prepare to exit power save */
		if (hps_cbs.host_power_save_off_prepare_cb) {
			hps_cbs.host_power_save_off_prepare_cb();
		}

		/* Wake recv_task after clearing the power-save gate. */
		eh_cp_recv_kick();
#if EH_CP_FEAT_WIFI_READY
		eh_cp_feat_wifi_replay_connected_event_if_needed();
#endif
#if EH_CP_FEAT_NW_SPLIT_READY
		eh_cp_feat_nw_split_replay_status_if_needed();
#endif
  #if EH_CP_FEAT_HOST_PS_DEEP_SLEEP
		if (wakeup_sem) {
			ESP_LOGI(TAG, "Giving wakeup semaphore");
			xSemaphoreGive(wakeup_sem);
		}
  #endif

		/* USER CALLBACK: Power save off, device ready */
		if (hps_cbs.host_power_save_off_ready_cb) {
			hps_cbs.host_power_save_off_ready_cb();
		}
	} else {
		ESP_LOGI(TAG, "Ignore event[%u]", ps_evt);
	}
#endif
	return 0;
}
