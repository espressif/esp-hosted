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

/** prevent recursive inclusion **/
#ifndef __NETDEV_IF_H
#define __NETDEV_IF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Network data buffer
 */
struct pbuf {
	/* Data buffer */
	uint8_t *payload;
	/* Length of data buffer */
	uint16_t len;
};

/**
 * @brief Network device instance
 */
typedef struct netdev * netdev_handle_t;

/**
 * @brief Network operations implemented by driver
 */
struct netdev_ops {
	/* Open device */
	int (* netdev_open) (netdev_handle_t netdev);
	/* Close device */
	int (* netdev_close) (netdev_handle_t netdev);
	/* Transmit packet */
	int (* netdev_xmit) (netdev_handle_t netdev, struct pbuf *net_buf);
};

/**
 * @brief Allocate network device instance for given interface name
 * @param sizeof_priv - sizeof driver private instance
 * @param name - Interface name
 * @retval network device instance
 */
netdev_handle_t netdev_alloc(uint32_t sizeof_priv, char *name);

/**
 * @brief Free network device instance
 * @param netdev - Network device instance
 * @retval none
 */
void netdev_free(netdev_handle_t netdev);

/**
 * @brief Returns address of driver priv instance from network device instance
 * @param netdev - Network device instance
 * @retval - Driver priv instance
 */
void * netdev_get_priv(netdev_handle_t netdev);

/**
 *  @brief Register a network interface
 *  @param netdev - Network device instance
 *  @param netdev_ops - Network operations implemented by driver
 *  @retval -   on success: 0
 *              on failure: -1
 */
int netdev_register(netdev_handle_t netdev, struct netdev_ops *ops);

/**
 * @brief Unregister network interfcace
 * @param netdev - Network device instance
 * @retval -    on success: 0
 *              on failure: -1
 */
int netdev_unregister(netdev_handle_t netdev);

/**
 * @brief Rx handler of network stack
 * @param netdev - Network device instance
 * @param pbuf - received data buffer
 */
int netdev_rx(netdev_handle_t netdev, struct pbuf *net_buf);

#ifdef __cplusplus
}
#endif

#endif
