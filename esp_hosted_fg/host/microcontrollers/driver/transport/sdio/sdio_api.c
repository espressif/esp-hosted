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

/** Includes **/
#include "common.h"
#include "sdio_ll.h"

/** Constants/Macros **/

#define CLK_IN_MHZ                      1000000
#define SDIO_CLOCK                      10*CLK_IN_MHZ
#define MAX_SUPPORTED_SDIO_CLOCK        10*CLK_IN_MHZ

/**
 * @brief  Probe and initialize SDIO slave using given host
 * @param  None
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_init(void)
{
	sdio_init_t sdio_init = {
		.width = WIDTH_4,
		/* Max supported now is MAX_SUPPORTED_SDIO_CLOCK
		 * Anything beyond MAX_SUPPORTED_SDIO_CLOCK will automatically be limited
		 **/
		.clock = SDIO_CLOCK
	};

	if (sdio_init.clock > MAX_SUPPORTED_SDIO_CLOCK) {
		/* Limit clock to maximum supported */
		sdio_init.clock = MAX_SUPPORTED_SDIO_CLOCK;
	}

	STM32SdioInit(sdio_init);
	return STM_OK;
}

/**
 * @brief  Read multiple bytes from an SDIO card using CMD53
 * @param  function - IO function number
 *         addr - byte address within IO function where reading starts
 *         buffer - buffer which receives the data read from card
 *         len - number of bytes to read
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_bytes(uint32_t function, uint32_t addr, void* buffer,
		uint32_t len, uint8_t multi_blocks)
{
	int ret = 0;
	ret = STM32ReadData(function, addr, buffer, len, multi_blocks);
	if (ret) {
		printf("%s %d CMD53 error\r\n",__func__, __LINE__);
		return STM_FAIL;
	}

	return STM_OK;
}

/**
 * @brief  Write multiple bytes to an SDIO card using CMD53
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - data to be written
 *         len - number of bytes to write
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_bytes(uint32_t function, uint32_t addr,
		void* buffer, uint32_t len)
{
	int ret = STM32WriteData(function, addr, buffer, len);
	if (ret < 0) {
		printf("%s CMD53 write error\r\n",__func__);
		return STM_FAIL;
	}
	return STM_OK;
}

/**
 * @brief  Write blocks of data to an SDIO card using CMD53
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - data to be written
 *         len - number of bytes to read, must be divisible by the card block size.
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_blocks(uint32_t function, uint32_t addr,
		void* buffer, uint32_t len)
{
	return sdio_driver_write_bytes(function, addr, buffer, len);
}

/**
 * @brief  Read blocks of data from an SDIO card using CMD53
 * @param  function - IO function number
 *         addr - byte address within IO function where writing starts
 *         buffer - buffer which receives the data read from card
 *         len - number of bytes to read, must be divisible by the card block size.
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_blocks(uint32_t function, uint32_t addr,
		void* buffer, uint32_t len, uint8_t multi_blocks)
{
	stm_ret_t ret = STM_OK;

	ret = sdio_driver_read_bytes(function, addr, buffer, len, multi_blocks);
	if (ret) {
		printf("%s %d CMD53 error\r\n",__func__, __LINE__);
		return STM_FAIL;
	}
	return ret;
}

/**
 * @brief  Read one byte from SDIO slave using CMD52
 * @param  function - IO function number
 *         reg - byte address within IO function
 *         [out]out_byte - output, receives the value read from the card
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_read_byte(uint32_t function, uint32_t reg, uint8_t *out_byte)
{
	uint8_t func = function & 0xf;
	uint8_t result = STM32ReadReg(func, reg);
	if (out_byte) {
		*out_byte = result;
	}
	return STM_OK;
}

/**
 * @brief  Write one byte to SDIO slave using CMD52
 * @param  function - IO function number
 *         reg - byte address within IO function
 *         in_byte - value to be written
 *         [out]out_byte - if not NULL, receives new byte value read from
 *         the card (read-after-write).
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
stm_ret_t sdio_driver_write_byte(uint32_t function, uint32_t reg,
		uint8_t in_byte, uint8_t* out_byte)
{
	uint8_t func = function & 0xf;
	uint8_t result = STM32WriteReg(func, reg, in_byte);
	if (out_byte) {
		*out_byte = result;
	}
	return STM_OK;
}
