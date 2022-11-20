/**
  ******************************************************************************
  * File Name          : sdio_ll.c
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

//TODO: DMA is not working yet
#define WIFI_USEDMA                     (0)

/** Includes **/
#include "sdio_reg.h"
#include "sdio_ll.h"
#include "trace.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "gpio.h"

#if WIFI_USEDMA
#include "stm32f4xx_ll_dma.h"
#endif


/** Constants/Macros **/
#define SDMMC_R4_READYBIT               BIT(31)
#define SDMMC_R4_ERRORBITS              ((uint32_t)0x7F000000U)
#define SDMMC_R4_NOERROR                ((uint32_t)0x20000000U)
#define SDMMC_R4_OCRBITS                ((uint32_t)0x00FFFFFFU)

#define ESP_DATA_MIN_ADDRESS            (0x1e800)

#define CMD52_WRITE                     BIT(31)
#define CMD52_READAFTERWRITE            BIT(27)
#define CMD53_WRITE                     BIT(31)
#define CMD53_BLOCKMODE                 BIT(27)
#define CMD53_INCREMENTING              BIT(26)
#define CMD53_TIMEOUT                   (10000000)
#define CCCR_BUS_WIDTH_4                (2<<0)

#define SDIO_IO_RESET_FUNC              (6)
#define SDIO_IO_RESET_VAL               (0x8)
#define SDIO_CMD_SEND_RETRY             (3)

#define SDIO_CLK_DIV			(238)

/** Macros/Constants **/
#define CHECK_SDIO_PRINT_ERR(ErR) {\
	if (ErR) { \
		printf("%s: %u err %lu\r\n",__func__,__LINE__,ErR); \
	} \
}

/** Global Variables **/
static SDIO_CmdInitTypeDef sdio_cmd = {0};
static SemaphoreHandle_t semahandle = {0};
static SD_HandleTypeDef hsd = {0};

/** External variables **/
extern SemaphoreHandle_t sdio_recv_SemHandle;


/** function definition **/

/** Local Functions **/

/**
 * @brief Calculates clock divider to be used from expected clock
 * frequency and hardware reference frequency
 *
 * @param  freq - Input frequency for which divider to be calculated
 *         preal - [OUT] Frequency to be set after calculating divider
 * @retval Divider value
 */
static uint8_t CalcClockDivider(uint32_t freq, uint32_t *preal)
{
	int divider;
	uint32_t sdioclk;

	sdioclk = HAL_RCC_GetPCLK2Freq();
	if (freq == 0)
		freq = 1;

	divider = sdioclk / freq - 2;
	if (sdioclk % freq != 0)
		divider++;
	if (divider < 0)
		divider = 0;
	else if (divider > 255)
		divider = 255;

	if (preal) {
		*preal = sdioclk / (divider + 2);
#if DEBUG_TRANSPORT
		printf("sdioclk=%luHz\n\rreq_freq=%luHz\n\rout_freq=%luHz\n\rdiv=%d\n\r",
				sdioclk, freq, *preal, divider);
	} else {
		printf("sdioclk=%luHz\n\rreq_freq=%luHz\n\rdiv=%d\n\r",
				sdioclk, freq, divider);
#endif
	}
	return divider & 0xff;
}

/**
 * @brief Send CMD52
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 *         data - data byte to be read/write
 *         flag - SDIO CMD52 flags
 * @retval None
 */
static void SendCMD52(uint8_t func, uint32_t addr, uint8_t data, uint32_t flags)
{
	sdio_cmd.Argument = (func << 28) | (addr << 9) | data | flags;
	sdio_cmd.CmdIndex = 52;
	sdio_cmd.CPSM = SDIO_CPSM_ENABLE;
	sdio_cmd.Response = SDIO_RESPONSE_SHORT;
	sdio_cmd.WaitForInterrupt = SDIO_WAIT_NO;
	SDIO_SendCommand(SDIO, &sdio_cmd);
}

/**
 * @brief Send CMD53
 *
 * @param  func - SDIO function
 *         addr - SDIO address
 *         count - [IN] number of bytes/blocks to be set to read/write
 *         flag - SDIO CMD53 flags
 * @retval None
 */
static void SendCMD53(uint8_t func, uint32_t addr, uint16_t count, uint32_t flags)
{
	sdio_cmd.Argument = (func << 28) | (addr << 9) | (count & 0x1ff) | flags;
	sdio_cmd.CmdIndex = 53;
	sdio_cmd.CPSM = SDIO_CPSM_ENABLE;
	sdio_cmd.Response = SDIO_RESPONSE_SHORT;
	sdio_cmd.WaitForInterrupt = SDIO_WAIT_NO;
	SDIO_SendCommand(SDIO, &sdio_cmd);
}

/**
 * @brief Initialize GPSIO in SDIO AF
 *        Please check sdio_reg.h for pin definitions
 *
 * @param  None
 * @retval None
 */
static void SdioGpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_SDIO_GPIO_ENABLE();

	GPIO_InitStruct.Pin = USR_SDIO_D0_Pin_Pin | USR_SDIO_D1_Pin_Pin | \
						  USR_SDIO_D2_Pin_Pin|USR_SDIO_D3_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USR_SDIO_CLK_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
	HAL_GPIO_Init(USR_SDIO_CLK_Pin_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USR_SDIO_CMD_Pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
	HAL_GPIO_Init(USR_SDIO_CMD_Pin_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief This function checks if data read/written is failed and for what cause
 *
 * @param  msg_title - function name, who is checking
 *         line - line number, who is checking
 * @retval 0 on success, number of err count otherwise
 */
static int CheckError(const char *msg_title, uint32_t line)
{
	int err = 0;

	if ((__SDIO_GET_FLAG(SDIO, SDIO_FLAG_CCRCFAIL) != RESET) &&
			(sdio_cmd.CmdIndex != 5))
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_CCRCFAIL);
		err++;
		printf("%s:%lu CMD%lu CRC failed!\n\r", msg_title, line, sdio_cmd.CmdIndex);
	}
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_CTIMEOUT) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_CTIMEOUT);
		err++;
#if !DEBUG_TRANSPORT
		if(sdio_cmd.CmdIndex != 5)
#endif
			printf("%s:%lu CMD%lu timeout!\n\r", msg_title, line, sdio_cmd.CmdIndex);
	}
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_DCRCFAIL) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_DCRCFAIL);
		err++;
		printf("%s:%lu data CRC failed!\n\r", msg_title, line);
	}
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_DTIMEOUT) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_DTIMEOUT);
		err++;
		printf("%s:%lu data timeout!\n\r", msg_title, line);
	}
#if defined(SDIO_STA_STBITERR)
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_STBITERR) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_STBITERR);
		err++;
		printf("%s:%lu start bit error!\n\r", msg_title, line);
	}
#endif
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_TXUNDERR) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_TXUNDERR);
		err++;
		printf("%s:%lu data underrun!\n\r", msg_title, line);
	}
	if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_RXOVERR) != RESET)
	{
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_RXOVERR);
		err++;
		printf("%s:%lu data overrun!\n\r", msg_title, line);
	}
#if WIFI_USEDMA
	if (LL_DMA_IsActiveFlag_TE4(DMA2))
	{
		LL_DMA_ClearFlag_TE4(DMA2);
		err++;
		printf("%s:%lu DMA transfer error!\n\r", msg_title, line);
	}
#endif
	return err;
}

/**
 * @brief This function checks if response recieved
 * It retries till SDIO_CMD_SEND_RETRY times to send command for getting response
 *
 * @param  SDIOx - SDIO peripheral
 * @retval 0 on success
 */
static int WaitForResponse(const char *msg_title, uint32_t line)
{
	uint8_t cmd_retry = 0;
	int err = 0;
	/* TODO: Give breathing space for command response
	 * Although, this is trade-off for higher response time
	 * Need to get proper fix for this
	 * */
	hard_delay(1);

	do {
		if (cmd_retry == SDIO_CMD_SEND_RETRY)
			break;

		if (cmd_retry != 0) {
			/* re-send */
			SDIO_SendCommand(SDIO, &sdio_cmd);
		}
		cmd_retry++;

		/* Wait till command send completed */
		while (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_CMDACT) != RESET);
		err = CheckError(msg_title, line);

		/* Check untill no response condition */
	} while (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_CMDREND) == RESET);
	__SDIO_CLEAR_FLAG(SDIO, SDIO_STATIC_CMD_FLAGS);

	return err;
}


/**
 * @brief This function checks if command is sent successfully or not
 *
 * @param  SDIOx - SDIO peripheral
 * @retval 0 on success
 */
static uint32_t SDMMC_GetCmdError(SDIO_TypeDef *SDIOx)
{
	/* 8 is the number of required instructions cycles for the below loop statement
	 * The SDIO_CMDTIMEOUT is expressed in ms
	 */
	register uint32_t count = SDIO_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

	do
	{
		if (count-- == 0U)
		{
			return SDMMC_ERROR_TIMEOUT;
		}

	}while(!__SDIO_GET_FLAG(SDIOx, SDIO_FLAG_CMDSENT));

	/* Clear all the static flags */
	__SDIO_CLEAR_FLAG(SDIOx, SDIO_STATIC_CMD_FLAGS);

	return SDMMC_ERROR_NONE;
}

/**
 * @brief Low level function to init SDIO peripheral
 *
 * This function may need to be ported if the host/slave hardware is changed
 *
 * @param  sd_init - structure holds the SDIO parameters to initialize slave
 * @retval None
 */
static void port_MX_SDIO_SD_Init(SD_InitTypeDef sd_init)
{
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = sd_init.ClockEdge;
	hsd.Init.ClockBypass = sd_init.ClockBypass;
	hsd.Init.ClockPowerSave = sd_init.ClockPowerSave;
	hsd.Init.BusWide = sd_init.BusWide;
	hsd.Init.HardwareFlowControl = sd_init.HardwareFlowControl;
	hsd.Init.ClockDiv = sd_init.ClockDiv;
	HAL_SD_Init(&hsd);
}

/**
 * @brief Low level function to init SDIO peripheral
 *
 * This function may need to be ported if the host/slave hardware is changed
 *
 * @param  init_para - structure holds the SDIO parameters to initialize slave
 * @retval STM_OK for success or failure from enum stm_ret_t
 */
static stm_ret_t SdioDriverInit(SD_InitTypeDef init_para)
{
	SD_InitTypeDef Init = {0};
	uint16_t sdio_rca;
	uint8_t bus_width;
	__HAL_RCC_SDIO_CLK_ENABLE();
#if WIFI_USEDMA
	__HAL_RCC_DMA2_CLK_ENABLE();
#endif

	__SDIO_ENABLE_IT(SDIO, SDIO_IT_SDIOIT);

	/* Default SDIO peripheral config for SD card initialization */
	/* Keep clock lower (<400k) for initial command sequence */
	Init.ClockDiv = SDIO_CLK_DIV;

	Init.BusWide = init_para.BusWide;

	/* Initialize SDIO peripheral interface with default config */
	port_MX_SDIO_SD_Init(Init);

	/* SDIO interrupt Init */
	HAL_NVIC_SetPriority(SDIO_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(SDIO_IRQn);

	/* Enable SDIO */
	__SDIO_OPERATION_ENABLE(SDIO);

#if WIFI_USEDMA
	/* Enable DMA */
	__SDIO_DMA_ENABLE(SDIO);
#endif

	/** SDIO IO specific slave initialization command sequence **/

	/* SDIO slave reset - I/O reset: CCCR I/O Abort register bit 3 */
	SendCMD52(SDIO_FUNC_0, SDIO_IO_RESET_FUNC, SDIO_IO_RESET_VAL, CMD52_WRITE);

	/* Command 0 - SD reset */
	sdio_cmd.Argument         = 0U;
	sdio_cmd.CmdIndex         = 0;
	sdio_cmd.Response         = SDIO_RESPONSE_NO;
	sdio_cmd.WaitForInterrupt = SDIO_WAIT_NO;
	sdio_cmd.CPSM             = SDIO_CPSM_ENABLE;
	SDIO_SendCommand(SDIO, &sdio_cmd);
	CHECK_SDIO_PRINT_ERR(SDMMC_GetCmdError(SDIO));

	/* Command 5 - IO_SEND_OP_COND */
	sdio_cmd.Argument = 0U;
	sdio_cmd.CmdIndex = 5;
	sdio_cmd.Response = SDIO_RESPONSE_SHORT;
	sdio_cmd.WaitForInterrupt = SDIO_WAIT_NO;
	sdio_cmd.CPSM = SDIO_CPSM_ENABLE;
	SDIO_SendCommand(SDIO, &sdio_cmd);
	WaitForResponse(__FUNCTION__,__LINE__);
	HAL_Delay(20);

	/* Command 5 - Set VDD Voltage Window: 3.2~3.4V */
//	sdio_cmd.Argument = 0x200000;
	sdio_cmd.Argument = 0x00ff8000;

	SDIO_SendCommand(SDIO, &sdio_cmd);
	WaitForResponse(__FUNCTION__,__LINE__);
	SDIO_GetResponse(SDIO, SDIO_RESP1);

	/* Command 3 - Get WiFi address (CMD3: SEND_RELATIVE_ADDR,
	 * Ask the card to publish a new relative address (RCA)) */
	sdio_cmd.Argument = 0;
	sdio_cmd.CmdIndex = 3;
	SDIO_SendCommand(SDIO, &sdio_cmd);
	WaitForResponse(__FUNCTION__,__LINE__);
	sdio_rca = SDIO_GetResponse(SDIO, SDIO_RESP1) >> 16;
#if DEBUG_TRANSPORT
	printf("Relative Card Address: 0x%04x\n\r", sdio_rca);
#endif

	/* Command 7 - Select WiFi (SELECT/DESELECT_CARD) */
	//sdio_cmd.Argument = sdio_rca << 16;	
	sdio_cmd.Argument = 0x00010000;
	sdio_cmd.CmdIndex = 7;
	SDIO_SendCommand(SDIO, &sdio_cmd);
	WaitForResponse(__FUNCTION__,__LINE__);
#if DEBUG_TRANSPORT
	printf("Card selected! RESP1_%08lx\n\r", SDIO_GetResponse(SDIO, SDIO_RESP1));
#endif

	/* Above sequence is needed while communicating to IO only device.
	 * In case of failure/timeouts despite of maximum retry, IO would be unusable
	 */

	SDIO_Init(SDIO, init_para);

	bus_width = STM32ReadReg(SDIO_FUNC_0, SD_IO_CCCR_BUS_WIDTH);

	if(init_para.BusWide == SDIO_BUS_WIDE_4B){
#if DEBUG_TRANSPORT
		printf("Use 4bit bus width\n\r");
#endif
		bus_width |= CCCR_BUS_WIDTH_4;
	}else if(init_para.BusWide == SDIO_BUS_WIDE_1B){
#if DEBUG_TRANSPORT
		printf("Use 1bit bus width\n\r");
#endif
		bus_width &= ~CCCR_BUS_WIDTH_4;
	} else {
		printf("Illegal bus width\n\r");
		return STM_FAIL;
	}
	//STM32WriteReg(SDIO_FUNC_0, SD_IO_CCCR_BUS_WIDTH, bus_width);
	STM32WriteReg(SDIO_FUNC_0, SD_IO_CCCR_BUS_WIDTH, 0x02);
	STM32WriteReg(SDIO_FUNC_0, 0x02, 0x02);
	STM32WriteReg(SDIO_FUNC_0, 0x10, 0x00);
	STM32WriteReg(SDIO_FUNC_0, 0x11, 0x02);
	STM32WriteReg(SDIO_FUNC_0, 0x110, 0x00);
	STM32WriteReg(SDIO_FUNC_0, 0x111, 0x02);

	return STM_OK;
}

/**
 * @brief Calculate block number for expected number of bytes
 *
 * @param  func - SDIO function
 *         psize - [IN] number of bytes
 *         write_flag - In case called from Write functions
 * @retval Number of blocks
 */
static uint16_t GetBlockNum(uint8_t func, uint32_t *psize,
		uint8_t write_flag, SDIO_DataInitTypeDef *sdio_data)

{
	uint16_t block_num = 0;
	uint16_t blocksize = ESP_BLOCK_SIZE;

	if(*psize == 4) {

		sdio_data->DataBlockSize = SDIO_DATABLOCK_SIZE_4B;

	} else if (*psize >= blocksize|| write_flag) {

		sdio_data->DataBlockSize = SDIO_DATABLOCK_SIZE_512B;

		block_num = *psize / blocksize;
		if (*psize % blocksize != 0)
			block_num++;
		*psize = block_num * blocksize;

	} else {

		sdio_data->DataBlockSize = SDIO_DATABLOCK_SIZE_1B;
		*psize = (*psize + 3) & ~3;
	}

	return block_num;
}

/** Exported Function **/

/**
 * @brief Intended Empty function
 *
 * This function will be automatically invoked by default
 * We want to delay the initialization and do it manually
 *
 * @param  None
 * @retval None
 */
void MX_SDIO_SD_Init(void)
{
	/* This function is set blank intentionally.
	 * IOC file import generates main.c, which invokes
	 * this function automatically.
	 * As ST driver does not correctly initialize I/O part,
	 * Init sequence and SDIO initialization is done manually.
	 * While porting to another hardware,
	 * may want to remove this definition to avoid duplicate
	 */
}

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
int STM32WriteData(uint8_t func, uint32_t addr, const void *data, uint32_t len)
{
	int i, err = 0;
	uint32_t size = len;
	uint16_t block_num = 0;
	static SDIO_DataInitTypeDef sdio_data = {0};
	uint32_t cmd53_flags = CMD53_WRITE;

	/* Data timeout is kept sufficiently high
	 * Depending upon use case, this could be optimized lower
	 * */
	sdio_data.DataTimeOut = SDMMC_DATATIMEOUT;

#if WIFI_USEDMA
	LL_DMA_InitTypeDef dma;
#else
	const uint32_t *p = data;
#endif

	if ((uintptr_t)data & 3)
	{
		printf("%s: data must be 4-byte aligned!\n\r", __FUNCTION__);
		return STM_FAIL_ALIGNMENT;
	}
	if (size == 0)
	{
		printf("%s: size cannot be 0!\n\r", __FUNCTION__);
		return STM_FAIL_INVALID_ARG;
	}
	xSemaphoreTake(semahandle, portMAX_DELAY);

	block_num = GetBlockNum(func, &size, 1, &sdio_data);

	cmd53_flags |= CMD53_INCREMENTING;

#if WIFI_USEDMA
	//SDIO->DCTRL = SDIO_DCTRL_SDIOEN | SDIO_DCTRL_DMAEN;
	SDIO->DCTRL = SDIO_DCTRL_DMAEN;
#else
	//SDIO->DCTRL = SDIO_DCTRL_SDIOEN;
	//SDIO->DCTRL = 0;
#endif
	do {
		if (block_num)
		{
			sdio_data.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
			SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE);
		}
		else
		{
			sdio_data.TransferMode = SDIO_TRANSFER_MODE_STREAM;
			SendCMD53(func, addr, size, cmd53_flags);
		}
	} while(WaitForResponse(__FUNCTION__,__LINE__));

	/* TODO: This delay is needed so that reader task would
	 * get breathing time for checking new packets
	 * */
	while((uint32_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == (uint32_t)RESET);
#if DEBUG_TRANSPORT
	//printf("CMD53 response\n\r");
#endif

#if WIFI_USEDMA

	dma.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
	dma.MemoryOrM2MDstAddress = (uint32_t)data;
	dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
	dma.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	dma.Mode = LL_DMA_MODE_NORMAL;
	dma.NbData = size / 4;
	dma.PeriphOrM2MSrcAddress = (uint32_t)&SDIO->FIFO;
	dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
	dma.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	dma.Priority = LL_DMA_PRIORITY_VERYHIGH;
	LL_DMA_Init(DMA2, LL_DMA_CHANNEL_4, &dma);
	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_4);
#endif

	sdio_data.DataLength = size;
	sdio_data.DPSM = SDIO_DPSM_ENABLE;
	sdio_data.TransferDir = SDIO_TRANSFER_DIR_TO_CARD;
	SDIO_ConfigData(SDIO, &sdio_data);


#if !WIFI_USEDMA
	while (size > 0)
	{
		while (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_TXFIFOF) != RESET);
		size -= 4;
		/* Write 4 bytes */
		SDIO_WriteFIFO(SDIO, (uint32_t *)p);
		p++;
		/* Wait untill there is space in FIFO */
		err += CheckError(__FUNCTION__,__LINE__);
		if (err) {
#if DEBUG_TRANSPORT
			printf("err while writing FIFO\n\r");
#endif
			break;
		}
	}
#endif

	while (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_TXACT) != RESET);

	i = 0;
	while (1)
	{
		if ((__SDIO_GET_FLAG(SDIO, SDIO_FLAG_DATAEND)) ||
		    (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_DBCKEND)))
			break;

		err += CheckError(__FUNCTION__,__LINE__);
		if (err) {
#if DEBUG_TRANSPORT
			printf("err before completing DATA END\n\r");
#endif
			break;
		}

		i++;
		if (i == CMD53_TIMEOUT)
		{
			printf("%s: timeout!\n\r", __FUNCTION__);
			err++;
			break;
		}
	}

#if DEBUG_TRANSPORT
	//printf("\n\rData:\n\r===> Data : %s\n\r",(char*)data);
#endif

	sdio_data.DPSM = SDIO_DPSM_DISABLE;
	SDIO_ConfigData(SDIO, &sdio_data);
#if WIFI_USEDMA
	/* Clear DMA flag */
	LL_DMA_ClearFlag_GI4(DMA2);
	/* Close DMA */
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
#endif
	__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_DATAEND | SDIO_FLAG_DBCKEND);
	//SDIO->DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_RWSTART |
	//			  SDIO_DCTRL_SDIOEN | SDIO_DCTRL_DTDIR;

	err += CheckError(__FUNCTION__,__LINE__);
	xSemaphoreGive(semahandle);
	if (err != 0) {
#if DEBUG_TRANSPORT
		printf("err while writing\n\r");
#endif
		return STM_FAIL;
	}
	return STM_OK;
}

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
uint8_t STM32WriteReg(uint8_t func, uint32_t addr, uint8_t value)
{
	SendCMD52(func, addr, value, CMD52_WRITE | CMD52_READAFTERWRITE);
	WaitForResponse(__FUNCTION__,__LINE__);
	return SDIO_GetResponse(SDIO, SDIO_RESP1) & 0xff;
}

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
uint8_t STM32ReadReg(uint8_t func, uint32_t addr)
{
	SendCMD52(func, addr, 0, 0);
	WaitForResponse(__FUNCTION__,__LINE__);
	return SDIO_GetResponse(SDIO, SDIO_RESP1) & 0xff;
}

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
int STM32ReadData(uint8_t func, uint32_t addr, void *data,
		uint32_t size, uint8_t multi_blocks)
{
	int i, err = 0;
	uint16_t block_num = 0;
	static SDIO_DataInitTypeDef sdio_data = {0};
	uint32_t cmd53_flags = 0;

	/* Data timeout is kept sufficiently high
	 * Depending upon use case, this could be optimized lower
	 * */
	sdio_data.DataTimeOut = SDMMC_DATATIMEOUT;

#if WIFI_USEDMA
	LL_DMA_InitTypeDef dma;
#else
	uint32_t *p = data;
#endif

	if ((uintptr_t)data & 3)
	{
		printf("%s: data must be 4-byte aligned!\n\r", __FUNCTION__);
		return STM_FAIL_ALIGNMENT;
	}
	if (size == 0)
	{
		printf("%s: size cannot be 0!\n\r", __FUNCTION__);
		return STM_FAIL_INVALID_ARG;
	}

	xSemaphoreTake(semahandle, portMAX_DELAY);

	block_num = GetBlockNum(func, &size, 0, &sdio_data);
#if DEBUG_TRANSPORT
	if (size > 4) {
	//	printf("size after getblocknum: %lu\n\r",size);
	}
#endif

#if WIFI_USEDMA
	//SDIO->DCTRL = SDIO_DCTRL_SDIOEN | SDIO_DCTRL_DMAEN;
	SDIO->DCTRL = SDIO_DCTRL_DMAEN;
	//__SDIO_DISABLE_IT(SDIO, SDIO_FLAG_SDIOIT);
#else
	//SDIO->DCTRL = 0;
#endif

#if WIFI_USEDMA
	dma.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
	dma.Mode = LL_DMA_MODE_NORMAL;
	dma.NbData = size / 4;
	dma.PeriphOrM2MSrcAddress = (uint32_t)&SDIO->FIFO;
	dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
	dma.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	dma.Priority = LL_DMA_PRIORITY_VERYHIGH;
	dma.MemoryOrM2MDstAddress = (uint32_t)data;
	dma.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;

	LL_DMA_Init(DMA2, LL_DMA_CHANNEL_4, &dma);
	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_4);
#endif
	cmd53_flags |= CMD53_INCREMENTING;

	// May need in case of DMA?
	if (block_num)
	{
		sdio_data.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
		SendCMD53(func, addr, block_num, cmd53_flags | CMD53_BLOCKMODE);
	}
	else
	{
		sdio_data.TransferMode = SDIO_TRANSFER_MODE_STREAM;
		SendCMD53(func, addr, size, cmd53_flags);
	}

	sdio_data.DataLength = size;
	sdio_data.DPSM = SDIO_DPSM_ENABLE;
	sdio_data.TransferDir = SDIO_TRANSFER_DIR_TO_SDIO;
	SDIO_ConfigData(SDIO, &sdio_data);

	{
		volatile int delay = 50;
		while(delay--);
	}

#if !WIFI_USEDMA
	int timeout_not_reached = 1000000;
	while (size > 0 && timeout_not_reached)
	{
		if (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_RXDAVL) != RESET)
		{
			size -= 4;
			*p++ = SDIO_ReadFIFO(SDIO);

		}
		else
		{

			err += CheckError(__FUNCTION__,__LINE__);
			if (err) {
#if DEBUG_TRANSPORT
				printf("Err while Reading from FIFO\n\r");
#endif
				break;
			}
		}
		timeout_not_reached--;
	}
	if (!timeout_not_reached) {
		printf("timed out while reading!!!\n\r");
		/* try to rectify the situation */
		sdio_data.DPSM = SDIO_DPSM_DISABLE;
		SDIO_ConfigData(SDIO, &sdio_data);
		__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_CMDREND | \
				SDIO_FLAG_DATAEND | SDIO_FLAG_DBCKEND);
		__SDIO_CLEAR_FLAG(SDIO, SDIO_STATIC_CMD_FLAGS);
		xSemaphoreGive(semahandle);
		return STM_FAIL;
	}
#endif

	i = 0;
	while (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_CMDACT) != RESET ||
	       __SDIO_GET_FLAG(SDIO, SDIO_FLAG_DATAEND) == RESET)
	{
		err += CheckError(__FUNCTION__,__LINE__);
		if (err)
			break;

		i++;
		if (i == CMD53_TIMEOUT)
		{
			printf("%s: timeout!\n\r", __FUNCTION__);
			err++;
			break;
		}
	}

	{
		volatile int delay = 50;
		while(delay--);
	}

	/* Disable DPSM */
	sdio_data.DPSM = SDIO_DPSM_DISABLE;
	SDIO_ConfigData(SDIO, &sdio_data);

#if WIFI_USEDMA
	LL_DMA_ClearFlag_GI4(DMA2);
	LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_4);
#endif

	__SDIO_CLEAR_FLAG(SDIO, SDIO_FLAG_CMDREND | \
			SDIO_FLAG_DATAEND | SDIO_FLAG_DBCKEND);

	//SDIO->DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_RWSTART |
	//			  SDIO_DCTRL_SDIOEN | SDIO_DCTRL_DTDIR;

	if(addr > ESP_DATA_MIN_ADDRESS) {
		__SDIO_ENABLE_IT(SDIO, SDIO_FLAG_SDIOIT);
	}

	err += CheckError(__FUNCTION__,__LINE__);
	xSemaphoreGive(semahandle);
	if (err != 0)
		return STM_FAIL;
	return STM_OK;
}

/**
 * @brief Initialization for SDIO peripheral
 *
 * This function communicates with slave peripheral and initializes with
 * expected frequency and bus width
 *
 * @param  sdio_init - structure holds the SDIO parameters to initialize slave
 * @retval None
 */
void STM32SdioInit(sdio_init_t sdio_init)
{
	uint32_t freq;
	SD_InitTypeDef Init;
	SdioGpioInit();
	semahandle = xSemaphoreCreateMutex();
	assert(semahandle);
	Init.ClockEdge = SDIO_CLOCK_EDGE_FALLING;
	Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	if(sdio_init.width == WIDTH_4){
		Init.BusWide = SDIO_BUS_WIDE_4B;
	} else {
		Init.BusWide = SDIO_BUS_WIDE_1B;
	}
	Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
	Init.ClockDiv = CalcClockDivider(sdio_init.clock, &freq);

	if (SdioDriverInit(Init))
	{
		printf("STM32 SDIO driver init error\n\r");
		return;
	}
}

/**
 * @brief  This function handles SDIO interrupt request
 * @param  hsd: Pointer to SD handle
 * @retval None
 */
void SDIO_IRQHandler(void)
{
	uint32_t intrpt_raised = (__SDIO_GET_FLAG(SDIO, SDIO_FLAG_SDIOIT) != RESET);
	BaseType_t xHigherPriorityTaskWoken;

	if(intrpt_raised){

		__HAL_SD_DISABLE_IT(&hsd, SDIO_IT_SDIOIT);
		/* Unblock receive task, to check if interrupt is servable */
		xSemaphoreGiveFromISR(sdio_recv_SemHandle,&xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
