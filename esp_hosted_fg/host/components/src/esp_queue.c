/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2022 Espressif Systems (Shanghai) PTE LTD
 * SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include "esp_queue.h"

/* create new node */
static q_node_t * new_q_node(void *data)
{
	q_node_t* new_node = (q_node_t*)malloc(sizeof(q_node_t));
	if (!new_node)
		return NULL;
	new_node->data = data;
	new_node->next = NULL;
	return new_node;
}

/* Create app queue */
esp_queue_t* create_esp_queue(void)
{
	esp_queue_t* q = (esp_queue_t*)malloc(sizeof(esp_queue_t));
	if (!q)
		return NULL;

	q->front = q->rear = NULL;
	return q;
}

/* Put element in app queue */
int esp_queue_put(esp_queue_t* q, void *data)
{
	q_node_t* new_node = NULL;

	if (!q) {
		printf("q undefined\n");
		return ESP_QUEUE_ERR_UNINITALISED;
	}

	new_node = new_q_node(data);
	if (!new_node) {
		printf("malloc failed in qpp_q_put\n");
		return ESP_QUEUE_ERR_MEMORY;
	}

	/* queue empty condition */
	if (q->rear == NULL) {
		q->front = q->rear = new_node;
		return ESP_QUEUE_SUCCESS;
	}

	q->rear->next = new_node;
	q->rear = new_node;
	return ESP_QUEUE_SUCCESS;
}

/* Get element in app queue */
void *esp_queue_get(esp_queue_t* q)
{
	void * data = NULL;
	q_node_t* temp = NULL;

	if (!q || q->front == NULL)
		return NULL;

	/* move front one node ahead */
	temp = q->front;

	if (!temp)
		return NULL;

	q->front = q->front->next;

	data = temp->data;

	free(temp);
	temp = NULL;

	/* If front is NULL, change rear also as NULL */
	if (q->front == NULL)
		q->rear = NULL;

	return data;
}

void esp_queue_destroy(esp_queue_t** q)
{
	q_node_t* temp = NULL;

	if (!q || !*q)
		return;

	while ((*q)->front) {

		temp = (*q)->front;
		(*q)->front = (*q)->front->next;

		if (temp->data) {
			free(temp->data);
			temp->data = NULL;
		}

		free(temp);
		temp = NULL;
	}

	free(*q);
	*q = NULL;
}
