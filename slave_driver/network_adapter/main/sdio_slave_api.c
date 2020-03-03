#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "interface.h"
#include "adapter.h"
#include "sdio_slave_api.h"
#include "driver/sdio_slave.h"
#include "soc/sdio_slave_periph.h"

#define SDIO_SLAVE_QUEUE_SIZE 20
#define BUFFER_SIZE     2048
#define BUFFER_NUM      20
uint8_t sdio_slave_rx_buffer[BUFFER_NUM][BUFFER_SIZE];

interface_context_t context;
static const char TAG[] = "SDIO_SLAVE";

static esp_err_t sdio_init();
static int32_t sdio_write(uint8_t if_type, uint8_t if_num, uint8_t* payload, int32_t payload_len);
static esp_err_t sdio_read(interface_handle_t *if_handle, uint8_t **out_addr, size_t *out_len);
static esp_err_t sdio_reset();
static void sdio_read_done(interface_handle_t *if_handle);
static void sdio_deinit();

if_ops_t if_ops = {
	.init = sdio_init,
	.write = sdio_write,
	.read = sdio_read,
	.reset = sdio_reset,
	.read_post_process = sdio_read_done,
	.deinit = sdio_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	memset(&context, 0, sizeof(context));

	context.type = SDIO;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	memset(&context, 0, sizeof(context));
	return 0;
}

static void event_cb(uint8_t val)
{
	if (val == RESET) {
		sdio_reset();
		return;
	}

	if (context.event_handler) {
		context.event_handler(val);
	}

}

static esp_err_t sdio_init()
{
	esp_err_t ret = 0;
	sdio_slave_config_t config = {
		.sending_mode       = SDIO_SLAVE_SEND_STREAM,
		.send_queue_size    = SDIO_SLAVE_QUEUE_SIZE,
		.recv_buffer_size   = BUFFER_SIZE,
		.event_cb           = event_cb,
		/* Note: For small devkits there may be no pullups on the board.
		   This enables the internal pullups to help evaluate the driver
		   quickly. However the internal pullups are not sufficient and not
		   reliable, please make sure external pullups are connected to the
		   bus in your real design.
		   */
		//.flags              = SDIO_SLAVE_FLAG_INTERNAL_PULLUP,
	};
	sdio_slave_buf_handle_t handle;

	ret = sdio_slave_initialize(&config);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	for(int i = 0; i < BUFFER_NUM; i++) {
		handle = sdio_slave_recv_register_buf(sdio_slave_rx_buffer[i]);
		assert(handle != NULL);

		ret = sdio_slave_recv_load_buf(handle);
		if (ret != ESP_OK) {
			sdio_slave_deinit();
			ESP_LOGE(TAG, "%s %d\n", __func__, __LINE__);
			return ret;
		}
	}

	sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
			SDIO_SLAVE_HOSTINT_BIT0 |
			SDIO_SLAVE_HOSTINT_BIT1 |
			SDIO_SLAVE_HOSTINT_BIT2 |
			SDIO_SLAVE_HOSTINT_BIT3 |
			SDIO_SLAVE_HOSTINT_BIT4 |
			SDIO_SLAVE_HOSTINT_BIT5 |
			SDIO_SLAVE_HOSTINT_BIT6 |
			SDIO_SLAVE_HOSTINT_BIT7);

	ret = sdio_slave_start();
	if (ret != ESP_OK) {
		sdio_slave_deinit();
		ESP_LOGE(TAG, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	ESP_LOGE(TAG, "%s %d %d\n", __func__, __LINE__, ret);
	return ret;
}

static int32_t sdio_write(uint8_t if_type, uint8_t if_num, uint8_t* payload, int32_t payload_len)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len = payload_len + sizeof (struct esp_payload_header);
	uint8_t* sendbuf = NULL;
	struct esp_payload_header *header;

	if (payload_len < 0 || payload == NULL) {
		ESP_LOGE(TAG , "Invalid arguments, len:%d", payload_len);
		return ESP_FAIL;
	}

	sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
	if (sendbuf == NULL) {
		ESP_LOGE(TAG , "Malloc send buffer fail!");
		return ESP_FAIL;
	}

	header = (struct esp_payload_header *) sendbuf;

	memset (header, 0, sizeof(struct esp_payload_header));

	/* Initialize header */
	header->if_type = if_type;
	header->if_num = if_num;
	header->len = payload_len;
	header->offset = sizeof(struct esp_payload_header);

	memcpy(sendbuf + header->offset, payload, payload_len);
	ret = sdio_slave_transmit(sendbuf, total_len);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
		free(sendbuf);
		return ESP_FAIL;
	}
	free(sendbuf);
	return payload_len;
}

static esp_err_t sdio_read(interface_handle_t *if_handle, uint8_t **out_addr, size_t *out_len)
{
	esp_err_t ret = ESP_OK;

	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to sdio_read");
		return ESP_FAIL;
	}

	ret = sdio_slave_recv(&(if_handle->buf_handle), out_addr, out_len, portMAX_DELAY);
	return ret;
}

static esp_err_t sdio_reset()
{
	esp_err_t ret;

	sdio_slave_stop();

	ret = sdio_slave_reset();
	if (ret != ESP_OK)
		return ret;

	ret = sdio_slave_start();
	if (ret != ESP_OK)
		return ret;

	while(1) {
		sdio_slave_buf_handle_t handle;

		/* Return buffers to driver */
		ret = sdio_slave_send_get_finished(&handle, 0);
		if (ret != ESP_OK)
			break;

		ret = sdio_slave_recv_load_buf(handle);
		ESP_ERROR_CHECK(ret);
	}

	return ESP_OK;
}

static void sdio_read_done(interface_handle_t *if_handle)
{
	if (if_handle)
		sdio_slave_recv_load_buf(if_handle->buf_handle);
}

static void sdio_deinit()
{
	sdio_slave_stop();
	sdio_slave_reset();
}
