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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <time.h>
#include "test.h"
#include "ctrl_api.h"
#include "app_custom_rpc.h"
#include "esp_hosted_custom_rpc.h"

/* Global variable for verification results in demo3 */
static volatile int g_verification_result = 0;
static uint8_t *g_original_data = NULL;
static uint32_t g_original_data_len = 0;

/* -------------- Demo 1 : Send packed RPC request. Slave acknowledges. -------------- */
int custom_rpc_demo1_request_only_ack(void) {
	uint8_t *recv_data = NULL;
	uint32_t recv_data_len = 0;
	void (*recv_data_free_func)(void*) = NULL;

	/* Create a byte stream with a specific pattern */
	uint32_t send_data_len = 1500;
	uint8_t *send_data = calloc(1, send_data_len);
	if (!send_data) {
		printf("Failed to allocate memory for send data\n");
		return FAILURE;
	}

	/* Fill with a simple pattern */
	for (uint32_t i = 0; i < send_data_len; i++) {
		send_data[i] = ((i * 2) % 256);
	}

	printf("[Demo 1] Sending packed RPC request (custom req num: %u). No Response expected. Only Ack expected.\n",
			CUSTOM_RPC_REQ_ID__ONLY_ACK);

	/* Use a message ID that doesn't expect echo back */
	uint32_t custom_msg_id = CUSTOM_RPC_REQ_ID__ONLY_ACK;

	int ret = test_custom_rpc_unserialised_request(custom_msg_id, send_data, send_data_len,
			&recv_data, &recv_data_len, &recv_data_free_func);

	if (ret != SUCCESS) {
		free(send_data);
		return FAILURE;
	}

	/* For this demo, we don't expect a meaningful response */
	printf("[Demo 1] Slave Ack for RPC request.\n");

	/* Clean up */
	free(send_data);

	/* As safe measure, free the received data, if any */
	if (recv_data_free_func && recv_data) {
		recv_data_free_func(recv_data);
	}

	return SUCCESS;
}

/* -------------- Demo 2 : Send packed RPC request. Slave echoes back as response. -------------- */
int custom_rpc_demo2_request_echo_back_as_response(void) {
	uint8_t *recv_data = NULL;
	uint32_t recv_data_len = 0;
	void (*recv_data_free_func)(void*) = NULL;
	int ret = SUCCESS;

	/* Create a byte stream with a specific pattern for verification */
	uint32_t send_data_len = 1500;
	uint8_t *send_data = calloc(1, send_data_len);
	if (!send_data) {
		printf("Failed to allocate memory for send data\n");
		return FAILURE;
	}

	/* Fill with a recognizable pattern */
	for (uint32_t i = 0; i < send_data_len; i++) {
		send_data[i] = (i % 256);
	}

	/* Use echo back response to verify data integrity */
	uint32_t custom_msg_id = CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE;

	printf("[Demo 2] Sending a demo byte buffer of %u size to slave, expecting echo back as response.\n",
			send_data_len);

	if (test_custom_rpc_unserialised_request(custom_msg_id, send_data, send_data_len,
				&recv_data, &recv_data_len, &recv_data_free_func) != SUCCESS) {
		printf("Failed to send custom RPC unserialised message\n");
		free(send_data);
		return FAILURE;
	}

	/* Verify received data matches what was sent */
	printf("Received %u bytes of data back as response\n", recv_data_len);

	if (recv_data_len != send_data_len) {
		printf("Data length mismatch! Sent: %u, Received: %u\n",
				send_data_len, recv_data_len);
		free(send_data);
		if (recv_data_free_func && recv_data) {
			recv_data_free_func(recv_data);
		}
		return FAILURE;
	}

	/* Compare byte by byte */
	int mismatch = 0;
	for (uint32_t i = 0; i < send_data_len; i++) {
		if (send_data[i] != recv_data[i]) {
			printf("Data mismatch at byte %u: sent 0x%02x, received 0x%02x\n",
					i, send_data[i], recv_data[i]);
			mismatch = 1;
			break;
		}
	}

	if (!mismatch) {
		printf("Data verification successful - [All %u TX bytes] = [All %u RX bytes]!\n",
				send_data_len, recv_data_len);
	} else {
		ret = FAILURE;
	}

	/* Clean up */
	free(send_data);
	if (recv_data_free_func && recv_data) {
		recv_data_free_func(recv_data);
	}

	return ret;
}

/* -------------- Demo 3 : Send packed RPC request. Slave echoes back as event. -------------- */
/* Function to set the reference data for verification */
static void custom_rpc_set_verification_reference(uint8_t *data, uint32_t len) {
	g_original_data = data;
	g_original_data_len = len;
	g_verification_result = 0; /* Reset result */
}

/* Custom event handler to verify the data received in the event */

int custom_rpc_event_handler(ctrl_cmd_t *app_event) {
	if (test_validate_ctrl_event(app_event)) {
		printf("%s invalid event[%u]\n", __func__, app_event->msg_id);
		CLEANUP_CTRL_MSG(app_event);
		return FAILURE;
	}

	if (app_event->msg_id == CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG) {
		custom_rpc_unserialised_data_t *p_e = &app_event->u.custom_rpc_unserialised_data;

		/* Process based on custom event ID */
		switch (p_e->custom_msg_id) {
			case CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST:
				printf("[Demo 3] Received echo back event with %u bytes of data\n", p_e->data_len);

#if 0
				/* Print data hexdump */
				printf("Data: ");
				for (size_t i = 0; i < p_e->data_len && i < 32; i++) {
					printf("%02X ", p_e->data[i]);
					if ((i + 1) % 8 == 0) {
						printf(" ");
					}
				}
				if (p_e->data_len > 32) {
					printf("... (%u more bytes)", p_e->data_len - 32);
				}
				printf("\n");
#endif

				/* Verify the data if we have a reference */
				if (!g_original_data || !g_original_data_len) {
					printf("[Demo 3] Verification reference data not set!\n");
					g_verification_result = -1;
					break;
				}

				if (g_original_data_len != p_e->data_len) {
					printf("[Demo 3] Data length mismatch! Sent: %u, Received: %u\n",
							g_original_data_len, p_e->data_len);
					g_verification_result = -1;
					break;
				}

				/* Verify received data matches what was sent originally */
				int mismatch = 0;
				for (uint32_t i = 0; i < p_e->data_len && i < g_original_data_len; i++) {
					if (g_original_data[i] != p_e->data[i]) {
						printf("Data mismatch at byte %" PRIu32 ": sent 0x%02x, received 0x%02x\n",
								i, g_original_data[i], p_e->data[i]);
						mismatch = 1;
						g_verification_result = -1;
						break;
					}
				}

				if (!mismatch) {
					printf("[Demo 3] Verified success: (All Rx bytes in event) = (All Tx bytes in request)\n");
					g_verification_result = 1;
				}
				break;

			default:
				printf("[Demo 3] Unhandled custom RPC event ID [%u] with data length: %u bytes\n",
						p_e->custom_msg_id, p_e->data_len);

#if 0
				/* Print data hexdump */
				printf("Data: ");
				for (size_t i = 0; i < p_e->data_len && i < 32; i++) {
					printf("%02X ", p_e->data[i]);
				}
				if (p_e->data_len > 32) {
					printf("... (%u more bytes)", p_e->data_len - 32);
				}
				printf("\n");
#endif
				break;
		}
	} else {
		printf("Unhandled base RPC message: %u\n", app_event->msg_id);
	}

	CLEANUP_CTRL_MSG(app_event);
	return SUCCESS;
}

int custom_rpc_demo3_request_echo_back_as_event(void) {
	uint8_t *recv_data = NULL;
	uint32_t recv_data_len = 0;
	void (*recv_data_free_func)(void*) = NULL;
	int ret = SUCCESS;
	int verification_timeout = 5; /* seconds */

	/* Create a byte stream with a specific pattern for verification */
	uint32_t send_data_len = 1500;
	uint8_t *send_data = calloc(1, send_data_len);
	if (!send_data) {
		printf("[Demo 3] Failed to allocate memory for send data\n");
		return FAILURE;
	}

	/* Fill with a pattern that can be verified at event receipt */
	for (uint32_t i = 0; i < send_data_len; i++) {
		send_data[i] = ((i * 3) % 256);
	}

	/* Store the original callback before setting our custom one */
	ctrl_resp_cb_t original_handler = NULL;

	original_handler = get_event_callback(CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG);
	if (!original_handler) {
		printf("[Demo 3] No original event handler registered for CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG\n");
	}

	/* Set reference data for verification handler */
	custom_rpc_set_verification_reference(send_data, send_data_len);

	/* Register custom handler */
	printf("[Demo 3] Setting custom event handler for verification\n");
	if (set_event_callback(CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG, custom_rpc_event_handler) != CALLBACK_SET_SUCCESS) {
		printf("[Demo 3] Failed to set custom event handler\n");
		free(send_data);
		return FAILURE;
	}

	printf("[Demo 3] Sending Custom Packed RPC request to slave. Expecting ack response. The data sent should be echoed back as an custom RPC event\n");

	/* Use a message ID that will trigger an event response */
	/* Slave is coded to respond with 'CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST'  when 'CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT' is received
	 * You can change both sides as you wish */
	uint32_t custom_msg_id = CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT;

	if (test_custom_rpc_unserialised_request(custom_msg_id, send_data, send_data_len,
				&recv_data, &recv_data_len, &recv_data_free_func) != SUCCESS) {
		printf("Failed to send custom RPC unserialised message\n");
		ret = FAILURE;
		goto cleanup;
	}

	/*
	 * For this demo, we don't need to verify the response data since we're expecting
	 * an event to be triggered. The event will be handled by the event callback.
	 */
	printf("[Demo 3] Request sent successfully. Waiting for event callback verification...\n");

	/* Clean up response data */
	if (recv_data_free_func && recv_data) {
		recv_data_free_func(recv_data);
	}

	/* Wait for verification to complete with timeout */
	time_t start_time = time(NULL);
	while (g_verification_result == 0 && (time(NULL) - start_time) < verification_timeout) {
		sleep(1);
		printf("Waiting for event verification...\n");
	}

	if (g_verification_result == 0) {
		printf("Verification timed out - event might not have been received\n");
		ret = FAILURE;
	} else if (g_verification_result < 0) {
		printf("Verification failed - data mismatch\n");
		ret = FAILURE;
	} else {
		printf("Verification completed successfully\n");
	}

cleanup:
	/* Reset data reference */
	custom_rpc_set_verification_reference(NULL, 0);

	/* Restore original handler */
	printf("[Demo 3] Restoring original event handler\n");

	/* Re-register the original event handler */
	if (set_event_callback(CTRL_EVENT_CUSTOM_RPC_UNSERIALISED_MSG, original_handler) != CALLBACK_SET_SUCCESS) {
		printf("[Demo 3] Failed to restore original event handlers\n");
		ret = FAILURE;
	}

	/* Clean up send data */
	free(send_data);

	return ret;
}
