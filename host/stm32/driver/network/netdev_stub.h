// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** prevent recursive inclusion **/
#ifndef __NETDEV_STUB_H
#define __NETDEV_STUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "common.h"
#include "netdev_if.h"

#define MAX_IF_NAME_SIZE    256
#define MAX_INTERFACE       2

#define NETDEV_STATE_DOWN   0
#define NETDEV_STATE_UP     1

#define RX_QUEUE_SIZE       50

struct network_handle;

struct netdev {
	/* Interface name */
	char name[MAX_IF_NAME_SIZE];

	/* Driver API's */
	struct netdev_ops *net_ops;

	/* Netdev state */
	uint8_t state;

	/*Application handle */
	struct network_handle *net_handle;

	/* Rx queue */
	QueueHandle_t rx_q;

	/* Driver priv */
	void *priv;
};

struct network_handle {
	struct netdev *ndev;
	void (* net_rx_callback)();
};

/**
  * @brief  initialize detdev
  * @param  None
  * @retval None
  */
void netdev_init();

/**
  * @brief  open netdev
  * @param  ndev - netdev
  * @retval 0 on success
  */
int netdev_open(struct netdev *ndev);

/**
  * @brief  close netdev
  * @param  ndev - netdev
  * @retval None
  */
void netdev_close(struct netdev *ndev);

/**
  * @brief  get netdev handle from interface name
  * @param  if_name - interface name
  * @retval netdev handle
  */
struct netdev * netdev_get(char *if_name);
#ifdef __cplusplus
}
#endif

#endif

