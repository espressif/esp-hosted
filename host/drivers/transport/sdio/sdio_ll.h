/**
  ******************************************************************************
  * File Name          : sdio_ll.h
  * Description        : This file provides code for the configuration
  *                      of the SDIO instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDIO_LL_H
#define __SDIO_LL_H

/** Includes **/

/** Constants/Macros **/
typedef enum {
	WIDTH_1,     // 1 Bit
	WIDTH_4      // 4 Bit
} sdio_width_t;

/** Structures/Unions **/
typedef struct {
	sdio_width_t width;
	uint32_t clock;
}sdio_init_t;

/** function declarations **/
/**
 * @brief Initialization for SDIO peripheral
 *
 * This function communicates with slave peripheral and initializes with
 * expected frequency and bus width
 *
 * @param  sdio_init - structure holds the SDIO parameters to initialize slave
 * @retval None
 */
void STM32SdioInit(sdio_init_t sdio_init);

/**
 * @brief Read byte value with expected SDIO register address and SDIO function
 *
 * This function triggers CMD52 to read byte from expected SDIO register
 * For multi-byte read, please refer STM32ReadData
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 * @retval Byte read
 */
uint8_t STM32ReadReg(uint8_t func, uint32_t addr);

/**
 * @brief Write byte value to expected SDIO register address and SDIO function
 *
 * This function triggers CMD52 to write byte to expected SDIO register
 * For multi-byte write, please refer STM32WriteData
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 *         value - Byte to be written
 * @retval SDIO R1 response
 */
uint8_t STM32WriteReg(uint8_t func, uint32_t addr, uint8_t value);

/**
 * @brief Write data of exp size with expected SDIO register address and SDIO function
 *
 * This function checks if slave has sufficient buffer to write the data
 * If yes, it further writes data to specified register.
 * Note : data greater than 4 bytes is expected to be aligned of ESP_BLOCK_SIZE
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 *         data - Buffer to written
 *         size - Buffer size in bytes
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
int STM32WriteData(uint8_t func, uint32_t addr, const void *data, uint32_t size);

/**
 * @brief Read data of specified size with expected SDIO register address and SDIO function
 *
 * This function should be called only after HOST_SLC0_RX_NEW_PACKET_INT_ST is set
 * by the slave on ESP_SLAVE_INT_ST_REG register.
 * This function checks how many bytes to be read first,
 * followed by read in ESP_BLOCK_SIZE steps
 * Note : Slave should take care if data greater than 4 bytes, it is expected to
 *        write in ESP_BLOCK_SIZE alignement
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 *         data - Buffer to store input data
 *         size - Num of bytes to read
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
int STM32ReadData(uint8_t func, uint32_t addr, void *data, uint32_t size, uint8_t multi_blocks);

#endif /*__SDIO_LL_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
