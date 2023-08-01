// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2021 Espressif Systems (Shanghai) PTE LTD
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
#ifndef __SDIO_HOST_H
#define __SDIO_HOST_H

/** Includes **/

/** constants/macros **/
#define MAX_SDIO_SCRATCH_REG_SUPPORTED  8

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/

/**
 * Init SDIO host and slave
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_init(void);

/**
 * Block until an SDIO interrupt is received
 *
 * Slave uses D1 line to signal interrupt condition to the host
 * This function can be used to wait for the interrupt
 *
 * @param  timeout - time to wait for the interrupt, in ms
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_wait_int(uint32_t timeout);

/** Get interrupt bits of SDIO slave
 *
 * @param  intr_st - Output of the masked interrupt bits. set to NULL if only raw bits are read
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_get_intr(uint32_t *intr_st);

/** Clear interrupt bits of SDIO slave. All the bits set in the mask will be cleared, while other bits will stay the same
 *
 * @param  intr_mask - Mask of interrupt bits to clear
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_clear_intr(uint32_t intr_mask);

/** Get a packet from SDIO slave
 *
 * @param  [out] out_data - Data output address
 *         size - The size of the output buffer, if the buffer is smaller than
 *              the size of data to receive from slave, the driver returns ``ESP_ERR_NOT_FINISHED``
 *         [out] out_length - Output of length the data actually received from slave
 *         wait_ms - Time to wait before timeout, in ms
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_get_packet(void* out_data, size_t size, size_t *out_length, uint32_t wait_ms);

/** Send a packet to the SDIO slave
 *
 * @param  start - Start address of the packet to send
 *         length - Length of data to send, if the packet is over-size,
 *              the it will be divided into blocks and hold into different buffers automatically
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_send_packet(const void* start, uint32_t length);

/** Send a interrupt signal to the SDIO slave
 *
 * @param  intr_no - interrupt number, now only support 0
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_host_send_intr(uint8_t intr_no);

#endif /* __SDIO_HOST_H */
