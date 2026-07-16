/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file peer_data_example.c
 * @brief Custom RPC Echo Demo with Verification - Host Side
 *
 * Demonstrates custom message IDs with random data sizes.
 * Tests size ranges from small (1 byte) to maximum (8166 bytes).
 * Includes GHOST message to test exceeding max configured callbacks.
 *
 * Example Message IDs:
 * - MSG_ID_CAT/MEOW: Small messages (1-1000 bytes)
 * - MSG_ID_DOG/WOOF: Medium messages (1000-4000 bytes)
 * - MSG_ID_HUMAN/HELLO: Large messages (4000-8166 bytes)
 * - MSG_ID_GHOST: Exceeds max handlers (should fail gracefully)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
/* Linux-flavour: replace direct FreeRTOS task usage with the
 * platform-native eh_host_port_task API.  The rest of the IDF API surface
 * (esp_hosted_*, ESP_LOG*) is byte-for-byte with upstream — see
 * examples_layout.md §3a portability rule. */
#include "eh_host_port_task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_hosted.h"
#include "esp_check.h"

/* Example Message IDs - use any uint32_t except 0xFFFFFFFF */
#define MSG_ID_CAT      1   /**< Request: small data */
#define MSG_ID_MEOW     2   /**< Response: echo small data */
#define MSG_ID_DOG      3   /**< Request: medium data */
#define MSG_ID_WOOF     4   /**< Response: echo medium data */
#define MSG_ID_HUMAN    5   /**< Request: large data */
#define MSG_ID_HELLO    6   /**< Response: echo large data */
#define MSG_ID_GHOST    99  /**< Test: exceeds max configured handlers */

/* Maximum payload size for custom RPC (empirically determined) */
#define PEER_DATA_MAX_PAYLOAD_SIZE  8166

static const char *TAG = "peer_data_example";

/**
 * @brief Static context passed as 'user' at callback registration.
 *
 * Describes fixed properties of the entity owning this callback.
 * The framework returns this pointer as-is on every invocation —
 * the callback uses it without needing any global state.
 */
typedef struct {
    char home[32];   /**< Where this animal lives */
    char likes[32];  /**< What this animal enjoys */
} animal_ctx_t;

/* One context per response handler — static, never modified after init */
static animal_ctx_t meow_ctx  = { "cozy apartment",  "sunny window" };
static animal_ctx_t woof_ctx  = { "backyard kennel", "chew toy"     };
static animal_ctx_t hello_ctx = { "suburban house",  "couch"        };

/* Statistics tracking */
static uint32_t total_sent = 0;
static uint32_t total_received = 0;
static uint32_t total_bytes_sent = 0;
static uint32_t total_bytes_received = 0;
static uint32_t data_mismatch_count = 0;

/**
 * @brief Verify the user pointer returned by the framework matches what was registered.
 *
 * The user pointer passed at registration is returned as-is on every invocation.
 * This function confirms that contract holds.
 *
 * @param user         Pointer received in the callback
 * @param expected_ctx Pointer that was passed at registration
 * @return true if they match and are non-NULL, false otherwise
 */
static bool verify_user_ptr(void *user, void *expected_ctx)
{
	return (user != NULL) && (user == expected_ctx);
}

/**
 * @brief Verify received data matches expected pattern for given msg_id
 */
static bool verify_received_data(const uint8_t *data, size_t data_len, uint32_t request_msg_id)
{
	/* Verify pattern unique to request_msg_id: each byte should be ((index + msg_id) & 0xFF) */
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

/**
 * @brief Generate random size for given message ID (spread across full range)
 */
static uint32_t get_random_size_for_msg_id(uint32_t msg_id)
{
	switch (msg_id) {
		case MSG_ID_CAT:    return (rand() % 1000) + 1;          // 1-1000 bytes
		case MSG_ID_DOG:    return (rand() % 3000) + 1000;       // 1000-4000 bytes
		case MSG_ID_HUMAN:  return (rand() % 4166) + 4000;       // 4000-8166 bytes (max)
		case MSG_ID_GHOST:  return PEER_DATA_MAX_PAYLOAD_SIZE+100;     // overflow
		default:            return 64;
	}
}


/**
 * @brief Callback for receiving MEOW response from slave
 */
static void meow_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	/* Verify the user pointer came back as registered */
	if (!verify_user_ptr(user, &meow_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: MEOW      (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	/* Verify against CAT request pattern */
	if (verify_received_data(data, data_len, MSG_ID_CAT)) {
		ESP_LOGI(TAG, "slave ---> host: MEOW      (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: MEOW      (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

/**
 * @brief Callback for receiving WOOF response from slave
 */
static void woof_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	/* Verify the user pointer came back as registered */
	if (!verify_user_ptr(user, &woof_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: WOOF      (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	/* Verify against DOG request pattern */
	if (verify_received_data(data, data_len, MSG_ID_DOG)) {
		ESP_LOGI(TAG, "slave ---> host: WOOF      (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: WOOF      (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

/**
 * @brief Callback for receiving HELLO response from slave
 */
static void hello_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	/* Verify the user pointer came back as registered */
	if (!verify_user_ptr(user, &hello_ctx)) {
		ESP_LOGW(TAG, "slave ---> host: HELLO     (%zu bytes Rx) [unexpected user ptr]", data_len);
	}

	total_received++;
	total_bytes_received += data_len;

	/* Verify against HUMAN request pattern */
	if (verify_received_data(data, data_len, MSG_ID_HUMAN)) {
		ESP_LOGI(TAG, "slave ---> host: HELLO     (%zu bytes Rx) .. Verified, all OK!", data_len);
	} else {
		data_mismatch_count++;
		ESP_LOGE(TAG, "slave ---> host: HELLO     (%zu bytes Rx) ❌ data mismatch", data_len);
	}
}

/**
 * @brief Allocate and fill buffer with unique test pattern per msg_id
 * @return Allocated buffer (caller must free), or NULL on error
 */
static uint8_t* create_test_data(uint32_t size, uint32_t msg_id)
{
	if (size > PEER_DATA_MAX_PAYLOAD_SIZE) {
		return NULL;
	}

	uint8_t *data = (uint8_t *)malloc(size);
	if (!data) {
		return NULL;
	}

	/* Fill with pattern unique to this msg_id */
	for (uint32_t j = 0; j < size; j++) {
		data[j] = ((j + msg_id) & 0xFF);
	}

	return data;
}

/**
 * @brief Send data and update statistics
 */
static esp_err_t send_custom_data_checked(uint32_t msg_id, const uint8_t *data, uint32_t size)
{
	if (!data || size > PEER_DATA_MAX_PAYLOAD_SIZE) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t ret = esp_hosted_send_custom_data(msg_id, data, size);

	if (ret == ESP_OK) {
		total_sent++;
		total_bytes_sent += size;
	}

	return ret;
}

/**
 * @brief Task to send different message types with random sizes
 */
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

	/* Send each message type 3 times with random sizes */
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

			/* Without this all prints will just dump very fast
			 * In practical applications, no such delay is required
			 * */
			eh_host_port_task_delay_ms(200);
		}

	}

	/* Test GHOST message - exceeds max payload size */
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

	/* Print summary */
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

	/* Linux-flavour: upstream calls vTaskDelete(NULL); on linux_host the
	 * task body runs synchronously from app_main, so a plain return
	 * suffices. */
	return;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

	/* Initialize random seed.
	 * Linux-flavour: upstream uses xTaskGetTickCount(); time() is the
	 * portable equivalent that works without a FreeRTOS scheduler. */
	srand((unsigned)time(NULL));

	/* Initialize NVS */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	/* Initialize ESP-Hosted */
	ret = esp_hosted_init();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ESP-Hosted init failed: %s", esp_err_to_name(ret));
		return -1;
	}

	ret = esp_hosted_connect_to_slave();
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Connect to slave failed: %s", esp_err_to_name(ret));
		return -1;
	}

	/* Register callbacks for response message IDs */
	/* Each callback is registered with a static animal_ctx_t as the user pointer.
	 * The framework returns this pointer as-is on every invocation — the callback
	 * uses it to log where the animal lives and what it likes.
	 * Passing NULL instead is also valid if no context is needed. */
	ret = esp_hosted_register_custom_callback(MSG_ID_MEOW, meow_callback, &meow_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register MEOW callback: %s", esp_err_to_name(ret));
		return -1;
	}

	ret = esp_hosted_register_custom_callback(MSG_ID_WOOF, woof_callback, &woof_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register WOOF callback: %s", esp_err_to_name(ret));
		return -1;
	}

	ret = esp_hosted_register_custom_callback(MSG_ID_HELLO, hello_callback, &hello_ctx);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register HELLO callback: %s", esp_err_to_name(ret));
		return -1;
	}

	ESP_LOGI(TAG, "Response callbacks registered: MEOW, WOOF, HELLO");

	/* Linux-flavour: upstream spawns a FreeRTOS task here:
	 *   xTaskCreate(rpc_test_task, "rpc_test_task", 8192, NULL, 5, NULL);
	 * On linux_host the test body runs synchronously — main() blocks
	 * until rpc_test_task() returns, then app_main_entry exits cleanly. */
	rpc_test_task(NULL);
    return 0;
}
