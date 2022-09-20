/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
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
