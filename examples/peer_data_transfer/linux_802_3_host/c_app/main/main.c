/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Linux host_c_app: peer_data echo demo with size + pattern verification. */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include "eh_host_port_task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "eh_host.h"
#include "eh_host_core.h"
#include "eh_host_feat_peer_data.h"
#include "esp_check.h"

/* Example msg-IDs — request/response pairs, plus one overflow probe. */
#define MSG_ID_CAT      1
#define MSG_ID_MEOW     2
#define MSG_ID_DOG      3
#define MSG_ID_WOOF     4
#define MSG_ID_HUMAN    5
#define MSG_ID_HELLO    6
#define MSG_ID_GHOST    99

#define PEER_DATA_MAX_PAYLOAD_SIZE  8166

static const char *TAG = "peer_data_example";

/* Static context the framework returns as-is on every callback. */
typedef struct {
    char home[32];
    char likes[32];
} animal_ctx_t;

static animal_ctx_t meow_ctx  = { "cozy apartment",  "sunny window" };
static animal_ctx_t woof_ctx  = { "backyard kennel", "chew toy"     };
static animal_ctx_t hello_ctx = { "suburban house",  "couch"        };

static uint32_t total_sent = 0;
static uint32_t total_received = 0;
static uint32_t total_bytes_sent = 0;
static uint32_t total_bytes_received = 0;
static uint32_t data_mismatch_count = 0;

static bool verify_user_ptr(void *user, void *expected_ctx)
{
	return (user != NULL) && (user == expected_ctx);
}

static bool verify_received_data(const uint8_t *data, size_t data_len, uint32_t request_msg_id)
{
	for (size_t i = 0; i < data_len; i++) {
		uint8_t expected = ((i + request_msg_id) & 0xFF);
		if (data[i] != expected) {
			ESP_LOGE(TAG, "   ❌ Pattern mismatch at offset %zu: expected 0x%02x, got 0x%02x",
					i, expected, data[i]);
			return false;
		}
	}
	return true;
}

static uint32_t get_random_size_for_msg_id(uint32_t msg_id)
{
	switch (msg_id) {
		case MSG_ID_CAT:    return (rand() % 1000) + 1;
		case MSG_ID_DOG:    return (rand() % 3000) + 1000;
		case MSG_ID_HUMAN:  return (rand() % 4166) + 4000;
		case MSG_ID_GHOST:  return PEER_DATA_MAX_PAYLOAD_SIZE+100;
		default:            return 64;
	}
}


static void meow_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &meow_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: MEOW      (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_CAT)) {
		ESP_LOGI(TAG, "slave ---> host: MEOW      (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: MEOW      (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

static void woof_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &woof_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: WOOF      (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_DOG)) {
		ESP_LOGI(TAG, "slave ---> host: WOOF      (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: WOOF      (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

static void hello_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &hello_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: HELLO     (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_HUMAN)) {
		ESP_LOGI(TAG, "slave ---> host: HELLO     (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: HELLO     (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

/* Returns malloc'd buffer (caller frees) filled with msg_id-unique pattern. */
static uint8_t* create_test_data(uint32_t size, uint32_t msg_id)
{
	if (size > PEER_DATA_MAX_PAYLOAD_SIZE) {
		return NULL;
	}

	uint8_t *data = (uint8_t *)malloc(size);
	if (!data) {
		return NULL;
	}

	for (uint32_t j = 0; j < size; j++) {
		data[j] = ((j + msg_id) & 0xFF);
	}

	return data;
}

static esp_err_t send_custom_data_checked(uint32_t msg_id, const uint8_t *data, uint32_t size)
{
	if (!data || size > PEER_DATA_MAX_PAYLOAD_SIZE) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t ret = eh_host_peer_data_send(msg_id, data, size);

	if (ret == ESP_OK) {
		total_sent++;
		total_bytes_sent += size;
	}

	return ret;
}

static void rpc_test_task(void *pvParameters)
{
	ESP_LOGI(TAG, "\n\n");
	ESP_LOGI(TAG, "----------------------------------------");
	ESP_LOGI(TAG, "Custom RPC Echo Test");
	ESP_LOGI(TAG, "----------------------------------------");
	ESP_LOGI(TAG, "Testing message IDs with size ranges:");
	ESP_LOGI(TAG, "CAT→MEOW (1-1000 bytes)");
	ESP_LOGI(TAG, "DOG→WOOF (1000-4000 bytes)");
	ESP_LOGI(TAG, "HUMAN→HELLO (4000-8166 bytes)");
	ESP_LOGI(TAG, "GHOST (tests handler overflow)");
	ESP_LOGI(TAG, "----------------------------------------");

	const uint32_t msg_ids[] = {MSG_ID_CAT, MSG_ID_DOG, MSG_ID_HUMAN};
	const char *msg_names[] = {"CAT", "DOG", "HUMAN"};

	for (int cycle = 0; cycle < 10; cycle++) {
		ESP_LOGI(TAG, "\n\n--- Cycle %d ---", cycle + 1);

		for (int i = 0; i < 3; i++) {
			uint32_t msg_id = msg_ids[i];
			uint32_t size = get_random_size_for_msg_id(msg_id);

			ESP_LOGI(TAG, "slave <--- host: %-5s     (%" PRIu32 " bytes Tx)", msg_names[i], size);

			uint8_t *test_data = create_test_data(size, msg_id);
			if (!test_data) {
				ESP_LOGE(TAG, "failed to allocate ❌");
				continue;
			}

			esp_err_t ret = send_custom_data_checked(msg_id, test_data, size);

			if (ret == ESP_OK) {
				ESP_LOGD(TAG, "sent ✅");
			} else {
				ESP_LOGE(TAG, "failed ❌");
			}

			free(test_data);

			/* Avoid flooding the log; not required in real apps. */
			eh_host_port_task_delay_ms(200);
		}

	}

	ESP_LOGI(TAG, "\n--- Testing GHOST (exceeds max payload) ---");
	uint32_t ghost_size = get_random_size_for_msg_id(MSG_ID_GHOST);
	ESP_LOGI(TAG, "slave <--- host: GHOST     (%" PRIu32 " bytes Tx)", ghost_size);

	uint8_t *ghost_data = create_test_data(ghost_size, MSG_ID_GHOST);
	if (!ghost_data) {
		ESP_LOGI(TAG, "expected failure ✅ (size exceeds max)");
	} else {
		esp_err_t ret = send_custom_data_checked(MSG_ID_GHOST, ghost_data, ghost_size);
		free(ghost_data);
		if (ret != ESP_OK) {
			ESP_LOGI(TAG, "send failed (expected case)");
		} else {
			ESP_LOGE(TAG, "unexpected success ❌");
		}
	}

	eh_host_port_task_delay_ms(2000);

	ESP_LOGI(TAG, "----------------------------------------");
	ESP_LOGI(TAG, "Test Summary");
	ESP_LOGI(TAG, "----------------------------------------");
	ESP_LOGI(TAG, "Messages sent:        %" PRIu32 "", total_sent);
	ESP_LOGI(TAG, "Responses received:   %" PRIu32 "", total_received);
	ESP_LOGI(TAG, "Bytes sent:           %" PRIu32 "", total_bytes_sent);
	ESP_LOGI(TAG, "Bytes received:       %" PRIu32 "", total_bytes_received);

	if (total_sent && (total_sent == total_received) && (data_mismatch_count == 0)) {
		ESP_LOGI(TAG, "Data validation:      ✅ ALL PASSED");
		ESP_LOGI(TAG, "Result:               ✅ PASS");
	} else {
		ESP_LOGE(TAG, "Data validation:      ❌ %" PRIu32 " FAILURES", data_mismatch_count);
		ESP_LOGE(TAG, "Result:               ❌ FAIL");
	}
	ESP_LOGI(TAG, "----------------------------------------");

	return;
}

static int s_task_rc = 0;

static void app_task(void *arg)
{
    (void)arg;

	srand((unsigned)time(NULL));

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ret = eh_host_init_linux_default();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ESP-Hosted init failed: %s", esp_err_to_name(ret));
		s_task_rc = -1; return;
	}

	ret = eh_host_connect_to_slave();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Connect to slave failed: %s", esp_err_to_name(ret));
		s_task_rc = -1; return;
	}

	/* user pointer (animal_ctx_t) comes back as-is in each callback. */
	ret = eh_host_peer_data_register(MSG_ID_MEOW, meow_callback, &meow_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register MEOW callback: %s", esp_err_to_name(ret));
		s_task_rc = -1; return;
	}

	ret = eh_host_peer_data_register(MSG_ID_WOOF, woof_callback, &woof_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register WOOF callback: %s", esp_err_to_name(ret));
		s_task_rc = -1; return;
	}

	ret = eh_host_peer_data_register(MSG_ID_HELLO, hello_callback, &hello_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register HELLO callback: %s", esp_err_to_name(ret));
		s_task_rc = -1; return;
	}

	ESP_LOGI(TAG, "Response callbacks registered: MEOW, WOOF, HELLO");

	rpc_test_task(NULL);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    eh_host_port_task_create_cfg_t cfg = { .fn = app_task, .name = "app" };
    eh_host_port_task_t *t = NULL;
    if (eh_host_port_task_create(&cfg, &t) != EH_HOST_PORT_OK) {
        ESP_LOGE(TAG, "failed to spawn app task"); return 1;
    }
    eh_host_port_task_join(t);
    eh_host_port_task_destroy(t);
    if (s_task_rc != 0) return s_task_rc;
    pause();
    return 0;
}
