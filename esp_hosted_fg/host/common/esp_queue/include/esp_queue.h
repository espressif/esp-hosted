/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2022 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 */

#ifndef __ESP_QUEUE_H__
#define __ESP_QUEUE_H__

#define ESP_QUEUE_SUCCESS               0
#define ESP_QUEUE_ERR_UNINITALISED      -1
#define ESP_QUEUE_ERR_MEMORY            -2

typedef struct q_element {
	void *buf;
	int buf_len;
} esp_queue_elem_t;

/* Queue based on Linked List */
typedef struct esp_queue_node {
	void *data;
	struct esp_queue_node* next;
} q_node_t;

typedef struct esp_queue {
	q_node_t *front, *rear;
} esp_queue_t;

esp_queue_t* create_esp_queue(void);
void *esp_queue_get(esp_queue_t* q);
int esp_queue_put(esp_queue_t* q, void *data);
void esp_queue_destroy(esp_queue_t** q);

#endif /*__ESP_QUEUE_H__*/
