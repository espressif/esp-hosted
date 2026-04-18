// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2025 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

/**
 * @file app_peer_data_transfer.c
 * @brief Host-side peer data transfer example (mirrors MCU host example).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>
#include "app_peer_data_transfer.h"

/* Example Message IDs - use any uint32_t except 0xFFFFFFFF */
#define MSG_ID_CAT      1   /* Request: small data */
#define MSG_ID_MEOW     2   /* Response: echo small data */
#define MSG_ID_DOG      3   /* Request: medium data */
#define MSG_ID_WOOF     4   /* Response: echo medium data */
#define MSG_ID_HUMAN    5   /* Request: large data */
#define MSG_ID_HELLO    6   /* Response: echo large data */
#define MSG_ID_GHOST    99  /* Test: exceeds max configured handlers */

#define PEER_DATA_MAX_PAYLOAD_SIZE  8166

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

static int verify_user_ptr(void *user, void *expected_ctx)
{
	return (user != NULL) && (user == expected_ctx);
}

static int verify_received_data(const uint8_t *data, size_t data_len, uint32_t request_msg_id)
{
	for (size_t i = 0; i < data_len; i++) {
		uint8_t expected = ((i + request_msg_id) & 0xFF);
		if (data[i] != expected) {
			printf("   Pattern mismatch at offset %zu: expected 0x%02x, got 0x%02x\n",
			       i, expected, data[i]);
			return 0;
		}
	}
	return 1;
}

static uint32_t get_random_size_for_msg_id(uint32_t msg_id)
{
	switch (msg_id) {
		case MSG_ID_CAT:    return (rand() % 1000) + 1;
		case MSG_ID_DOG:    return (rand() % 3000) + 1000;
		case MSG_ID_HUMAN:  return (rand() % 4166) + 4000;
		case MSG_ID_GHOST:  return PEER_DATA_MAX_PAYLOAD_SIZE + 100;
		default:            return 64;
	}
}

static uint8_t *create_test_data(uint32_t size, uint32_t msg_id)
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

static int send_custom_data_checked(uint32_t msg_id, const uint8_t *data, uint32_t size)
{
	if (!data || size > PEER_DATA_MAX_PAYLOAD_SIZE) {
		return FAILURE;
	}

	int ret = esp_hosted_send_custom_data(msg_id, data, size);
	if (ret == SUCCESS) {
		total_sent++;
		total_bytes_sent += size;
	}
	return ret;
}

static void meow_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &meow_ctx)) {
		printf("host <-- slave: MEOW (%zu bytes) [unexpected user ptr]\n", data_len);
	}
	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_CAT)) {
		printf("host <-- slave: MEOW (%zu bytes) .. OK!\n", data_len);
	} else {
		data_mismatch_count++;
		printf("host <-- slave: MEOW (%zu bytes) data mismatch\n", data_len);
	}
}

static void woof_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &woof_ctx)) {
		printf("host <-- slave: WOOF (%zu bytes) [unexpected user ptr]\n", data_len);
	}
	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_DOG)) {
		printf("host <-- slave: WOOF (%zu bytes) .. OK!\n", data_len);
	} else {
		data_mismatch_count++;
		printf("host <-- slave: WOOF (%zu bytes) data mismatch\n", data_len);
	}
}

static void hello_callback(uint32_t msg_id, const uint8_t *data, size_t data_len, void *user)
{
	if (!verify_user_ptr(user, &hello_ctx)) {
		printf("host <-- slave: HELLO (%zu bytes) [unexpected user ptr]\n", data_len);
	}
	total_received++;
	total_bytes_received += data_len;

	if (verify_received_data(data, data_len, MSG_ID_HUMAN)) {
		printf("host <-- slave: HELLO (%zu bytes) .. OK!\n", data_len);
	} else {
		data_mismatch_count++;
		printf("host <-- slave: HELLO (%zu bytes) data mismatch\n", data_len);
	}
}

static void print_summary(void)
{
	printf("\n");
	printf("----------------------------------------\n");
	printf("Test Summary\n");
	printf("----------------------------------------\n");
	printf("Messages sent:        %" PRIu32 "\n", total_sent);
	printf("Responses received:   %" PRIu32 "\n", total_received);
	printf("Bytes sent:           %" PRIu32 "\n", total_bytes_sent);
	printf("Bytes received:       %" PRIu32 "\n", total_bytes_received);

	if (total_sent && (total_sent == total_received) && (data_mismatch_count == 0)) {
		printf("Data validation:      ALL PASSED\n");
		printf("Result:               PASS\n");
	} else {
		printf("Data validation:      %" PRIu32 " FAILURES\n", data_mismatch_count);
		printf("Result:               FAIL\n");
	}
	printf("----------------------------------------\n");
}

int peer_data_example_run(void)
{
	srand((unsigned int)time(NULL));

	if (esp_hosted_register_custom_callback(MSG_ID_MEOW, meow_callback, &meow_ctx) != SUCCESS) {
		printf("Failed to register MEOW callback\n");
		return FAILURE;
	}
	if (esp_hosted_register_custom_callback(MSG_ID_WOOF, woof_callback, &woof_ctx) != SUCCESS) {
		printf("Failed to register WOOF callback\n");
		return FAILURE;
	}
	if (esp_hosted_register_custom_callback(MSG_ID_HELLO, hello_callback, &hello_ctx) != SUCCESS) {
		printf("Failed to register HELLO callback\n");
		return FAILURE;
	}

	printf("\n\n");
	printf("----------------------------------------\n");
	printf("Custom RPC Echo Test\n");
	printf("----------------------------------------\n");
	printf("Testing message IDs with size ranges:\n");
	printf("CAT→MEOW (1-1000 bytes)\n");
	printf("DOG→WOOF (1000-4000 bytes)\n");
	printf("HUMAN→HELLO (4000-8166 bytes)\n");
	printf("GHOST (tests handler overflow)\n");
	printf("----------------------------------------\n");

	const uint32_t msg_ids[] = {MSG_ID_CAT, MSG_ID_DOG, MSG_ID_HUMAN};
	const char *msg_names[] = {"CAT", "DOG", "HUMAN"};

	for (int cycle = 0; cycle < 10; cycle++) {
		printf("\n\n--- Cycle %d ---\n", cycle + 1);

		for (int i = 0; i < 3; i++) {
			uint32_t msg_id = msg_ids[i];
			uint32_t size = get_random_size_for_msg_id(msg_id);

			printf("host --> slave: %s (%" PRIu32 " bytes), ", msg_names[i], size);

			uint8_t *test_data = create_test_data(size, msg_id);
			if (!test_data) {
				printf("failed to allocate\n");
				continue;
			}

			if (send_custom_data_checked(msg_id, test_data, size) == SUCCESS) {
				printf("sent\n");
			} else {
				printf("failed\n");
			}

			free(test_data);
			usleep(200 * 1000);
		}
	}

	printf("\n--- Testing GHOST (exceeds max payload) ---\n");
	uint32_t ghost_size = get_random_size_for_msg_id(MSG_ID_GHOST);
	printf("host --> slave: GHOST (%" PRIu32 " bytes), ", ghost_size);

	uint8_t *ghost_data = create_test_data(ghost_size, MSG_ID_GHOST);
	if (!ghost_data) {
		printf("expected failure (size exceeds max)\n");
	} else {
		if (send_custom_data_checked(MSG_ID_GHOST, ghost_data, ghost_size) != SUCCESS) {
			printf("send failed (expected case)\n");
		} else {
			printf("unexpected success\n");
		}
		free(ghost_data);
	}

	usleep(2000 * 1000);
	print_summary();
	return SUCCESS;
}
