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
#ifndef __NETDEV_API_H
#define __NETDEV_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include "netdev_stub.h"

/**
  * @brief  init network interface
  * @param  None
  * @retval STM_OK/STM_FAIL
  */
int network_init(void);

/**
  * @brief  open and return network interface handle
  * @param  if_name - interfae to open
  *         net_rx_callback - callback called on incoming data
  * @retval handle of network interface
  */
struct network_handle * network_open(char *if_name, void (* net_rx_callback)(struct network_handle *));

/**
  * @brief  read from network interface
  * @param  handle - network interface handle
  *         net_rx_callback - callback called on incoming data
  *         xTicksToWait - wait for ticks
  * @retval buffer read
  */
struct pbuf * network_read(struct network_handle *handle, TickType_t xTicksToWait);

/**
  * @brief  write onnetwork interface
  * @param  handle - network interface handle
  *         buffer - buffer to transmit
  * @retval 0 on success
  */
int network_write(struct network_handle *, struct pbuf *buffer);

/**
  * @brief  close network interface
  * @param  handle - network interface handle
  * @retval 0 on success
  */
int network_close(struct network_handle *net_handle);

/**
  * @brief  destroy network interface
  * @param  None
  * @retval None
  */
int network_deinit(void);
#ifdef __cplusplus
}
#endif

#endif
