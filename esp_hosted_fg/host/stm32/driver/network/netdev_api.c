// SPDX-License-Identifier: Apache-2.0
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

#include "netdev_api.h"

/**
  * @brief  init network interface
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
int network_init(void)
{
	netdev_init();
	return STM_OK;
}


/**
  * @brief  open and return network interface handle
  * @param  if_name - interfae to open
  *         net_rx_callback - callback called on incoming data
  * @retval handle of network interface
  */
struct network_handle * network_open(char *if_name, void (* net_rx_callback)(struct network_handle *))
{
	struct netdev *ndev = NULL;
	struct network_handle *net_handle = NULL;

	if (!if_name)
		return NULL;

	ndev = netdev_get(if_name);

	if (!ndev) {
		printf ("Invalid interface name\n");
		return NULL;
	}

	/* create network handle */
	net_handle = malloc(sizeof(struct network_handle));

	net_handle->ndev = ndev;
	net_handle->net_rx_callback = net_rx_callback;

	if (netdev_open(ndev)) {
		printf ("Failed to setup netdev\n");
		free(net_handle);
		return NULL;
	}

	ndev->net_handle = net_handle;

	return net_handle;
}


/**
  * @brief  read from network interface
  * @param  handle - network interface handle
  *         net_rx_callback - callback called on incoming data
  *         xTicksToWait - wait for ticks
  * @retval buffer read
  */
struct pbuf * network_read(struct network_handle *handle, TickType_t xTicksToWait)
{
	struct pbuf *buffer = NULL;

	if (!handle || !handle->ndev)
		return NULL;

	buffer = malloc(sizeof(struct pbuf));

	if (!buffer)
		return NULL;

	xQueueReceive(handle->ndev->rx_q, buffer, xTicksToWait);

	return buffer;
}


/**
  * @brief  write onnetwork interface
  * @param  handle - network interface handle
  *         buffer - buffer to transmit
  * @retval 0 on success
  */
int network_write(struct network_handle *net_handle, struct pbuf *buffer)
{
	struct netdev *ndev;
	int ret;

	if (!net_handle || !buffer)
		return STM_FAIL;

	ndev = net_handle->ndev;

	if (ndev && (ndev->state == NETDEV_STATE_UP)) {
		if (ndev->net_ops && ndev->net_ops->netdev_xmit) {
			ret = ndev->net_ops->netdev_xmit(ndev, buffer);
		}
	}

	return ret;
}


/**
  * @brief  close network interface
  * @param  handle - network interface handle
  * @retval 0 on success
  */
int network_close(struct network_handle *net_handle)
{
	netdev_close(net_handle->ndev);
	free(net_handle);

	return STM_OK;
}


/**
  * @brief  destroy network interface
  * @param  None
  * @retval None
  */
int network_deinit(void)
{
	return STM_OK;
}
