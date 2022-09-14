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

/*prevent recursive inclusion */
#ifndef __VHCI_IF_H
#define __VHCI_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* vhci interface handle */
struct vhci_handle_s;
typedef struct vhci_handle_s vhci_handle_t;

int ble_vhci_init(void);
vhci_handle_t *ble_vhci_open(void (*hci_rx_callback)(struct vhci_handle_s *, void *), void *arg);

/**
 * @brief Close vhci interface
 * @param  vhci_hdl - handle
 */
void ble_vhci_close(vhci_handle_t *vhci_hdl);

/**
 * @brief  vhci interface read non blocking
 * @param  vhci_hdl - handle
 *         rlen - output param, number of bytes read
 * @retval rbuffer - ready buffer read on vhci inerface
 */
uint8_t *ble_vhci_read(const vhci_handle_t *vhci_hdl, uint16_t *rlen);

/**
 * @brief vhci interface write
 * @param  vhci_hdl - handle
 *         wlen - number of bytes to write
 *         wbuffer - buffer to send
 * @retval errno code
 */
int ble_vhci_write(const vhci_handle_t *vhci_hdl, uint8_t *wbuffer, const uint16_t wlen);

/**
 * @brief vhci rx handler is called by spi driver when there
 *        is incoming data with interface type is vhci.
 * @param  buf_handle - handle
 *         wlen - number of bytes to write
 *         rxbuff - buffer from spi driver
 *         rx_len - size of rxbuff
 * @retval None
 */
int vhci_rx_handler(uint8_t if_num, uint8_t *rxbuff, uint16_t rx_len);

void ble_vhci_free_rx(void *ptr);

#ifdef __cplusplus
}
#endif

#endif /* __VHCI_IF_H */
