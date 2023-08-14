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
#ifndef __SDIO_API_H
#define __SDIO_API_H

/** Includes **/

/** constants/macros **/

/** Exported Structures **/

/** Exported variables **/

/** Inline functions **/

/** Exported Functions **/

/**
 * Probe and initialize SDIO slave using given host
 *
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_init(void);

/**
 * Write blocks of data to an SDIO card using CMD53
 *
 * This function performs write operation using CMD53 in block mode
 * For byte mode, see sdio_driver_write_bytes
 *
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - data to be written
 *         len - number of bytes to read, must be divisible by the card block size
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_blocks(uint32_t function, uint32_t addr, void* buffer, uint32_t len);

/**
 * Read blocks of data from an SDIO card using CMD53
 *
 * This function performs read operation using CMD53 in block mode
 * For byte mode, see sdio_driver_read_bytes
 *
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - buffer which receives the data read from card
 *         len - number of bytes to read, must be divisible by the card block size
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_blocks(uint32_t function, uint32_t addr, void* buffer, uint32_t len, uint8_t multi_blocks);

/**
 * Read multiple bytes from an SDIO card using CMD53
 *
 * This function performs read operation using CMD53 in byte mode
 * For block mode, see sdio_driver_read_blocks
 *
 * @param  function - IO function number
 *         addr - byte address within IO function where reading starts
 *         buffer - buffer which receives the data read from card
 *         len - number of bytes to read
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_bytes(uint32_t function, uint32_t addr, void* buffer, uint32_t len, uint8_t multi_blocks);

/**
 * Write multiple bytes to an SDIO card using CMD53
 *
 * This function performs write operation using CMD53 in byte mode
 * For block mode, see sdio_driver_write_blocks
 *
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - data to be written
 *         len - number of bytes to write
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_bytes(uint32_t function, uint32_t addr, void* buffer, uint32_t len);

/**
 * Read one byte from SDIO slave using CMD52
 *
 * @param  function - IO function number
 *         reg - byte address within IO function
 *         [out]out_byte - output, receives the value read from the card
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_byte(uint32_t function, uint32_t reg, uint8_t *out_byte);

/**
 * Write one byte to SDIO slave using CMD52
 *
 * @param  function - IO function number
 *         reg - byte address within IO function
 *         in_byte - value to be written
 *         [out]out_byte - if not NULL, receives new byte value read from the
 *          card (read-after-write)
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_byte(uint32_t function, uint32_t reg, uint8_t in_byte, uint8_t* out_byte);

#endif /* __SDIO_API_H */
