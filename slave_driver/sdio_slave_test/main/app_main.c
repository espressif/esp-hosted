/* SDIO example, slave (uses sdio slave driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "driver/sdio_slave.h"
#include "esp_log.h"
#include "rom/lldesc.h"
#include "rom/queue.h"
#include "soc/soc.h"
#include "soc/sdio_slave_periph.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <unistd.h>
#include "xtensa/core-macros.h"
#include "esp_wifi_internal.h"

#include <esp_at.h>
/*
   sdio slave example.

   This example is supposed to work together with the sdio host example. It uses the pins as follows:

     *   Host      Slave
     *   IO14      CLK
     *   IO15      CMD
     *   IO2       D0
     *   IO4       D1
     *   IO12      D2
     *   IO13      D3

    This is the only pins that can be used in standard ESP modules. The other set of pins (6, 11, 7, 8, 9, 10)
    are occupied by the spi bus communicating with the flash.

    Protocol Above the ESP slave service:
        - Interrupts:
            0 is used to notify the slave to read the register 0.

        - Registers:
            - 0 is the register to hold tasks. Bits:
                - 0: the slave should reset.
                - 1: the slave should send interrupts.
                - 2: the slave should write the shared registers acoording to the value in register 1.
            - 1 is the register to hold test value.
            - other registers will be written by the slave for testing.

        - FIFO:
            The receving FIFO is size of 256 bytes.
            When the host writes something to slave recv FIFO, the slave should return it as is to the sending FIFO.

    The host works as following process:

        1. reset the slave.
        2. tell the slave to write registers and read them back.
        3. tell the slave to send interrupts to the host.
        4. send data to slave FIFO and read them back.
        5. loop step 4.
   */

#define SDIO_SLAVE_QUEUE_SIZE 20

#define BUFFER_SIZE     2048
#define BUFFER_NUM      20

#define EV_STR(s) "================ "s" ================"
static const char TAG[] = "example_slave";
static const char TAG_RX[] = "H -> S";
static const char TAG_RX_S[] = "CONTROL H -> S";
static const char TAG_TX[] = "S -> H";
static const char TAG_TX_S[] = "CONTROL S -> H";
uint8_t buffer[BUFFER_NUM][BUFFER_SIZE];
uint8_t buf[BUFFER_SIZE];
volatile uint8_t action = 0;
volatile uint8_t datapath = 0;
volatile uint8_t sta_connected = 0;

uint32_t from_wlan_count = 0;
uint32_t to_host_count = 0;
uint32_t to_host_sent_count = 0;

static SemaphoreHandle_t sdio_write_lock = NULL;

/* Ring Buffer: WLAN driver -> sdio interface */
RingbufHandle_t rb_handle_wlan_to_sdio = NULL;
RingbufHandle_t rb_handle_sdio_to_wlan = NULL;
#define RING_BUFFER_SIZE	1600

enum PACKET_TYPE {
	DATA_PACKET = 0,
};

enum HOST_INTERRUPTS {
	START_DATA_PATH = 0,
	STOP_DATA_PATH,
};

enum INTERFACE_TYPE {
	STA_INTF = 0,
	AP_INTF,
	SERIAL_INTF = (1<<1),
};
struct payload_header {
	uint8_t 		 pkt_type:2;
	uint8_t 		 if_type:3;
	uint8_t 		 if_num:3;
	uint8_t			 reserved1;
	uint16_t                 len;
	uint16_t                 offset;
	uint8_t                  reserved2[2];
} __attribute__((packed));

static struct rx_data {
    uint8_t valid;
    int len;
    uint8_t data[1024];
} r;
//#define min(x, y) ((x) < (y) ? (x) : (y))
static inline int min(int x, int y) {
    return (x < y) ? x : y;
}

#if 0
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	ESP_LOGI(TAG, "Event: %d\r\n", event->event_id);

	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
			break;
		case SYSTEM_EVENT_STA_CONNECTED:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_CONNECTED");
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
			/* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
			esp_wifi_connect();
			break;
		default:
			break;
	}
	return ESP_OK;
}
#endif

typedef struct {
	uint8_t		if_type;
	uint16_t	len;
	void 		*buf;
	void		*buf_ptr;
}wlan_buffer;

esp_err_t wlan_ap_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	uint16_t buf_size;
	wlan_buffer wlan_buf;

	if (!buffer || !eb) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	buf_size = xRingbufferGetCurFreeSize(rb_handle_wlan_to_sdio);
	if (buf_size < sizeof(wlan_buf)) {
		ESP_LOGE(TAG, "WLAN -> SDIO: Not enough buffer space available: %d\n", buf_size);
		goto DONE;
	}

	/* Prepare buffer descriptor */
	memset(&wlan_buf, 0, sizeof(wlan_buf));

	wlan_buf.if_type = AP_INTF;
	wlan_buf.len = len;
	wlan_buf.buf = buffer;
	wlan_buf.buf_ptr = eb;

	ret = xRingbufferSend(rb_handle_wlan_to_sdio, &wlan_buf, sizeof(wlan_buf), portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "WLAN -> SDIO: Failed to write to ring buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}

esp_err_t wlan_sta_rx_callback(void *buffer, uint16_t len, void *eb)
{
	esp_err_t ret = ESP_OK;
	uint16_t buf_size;
	wlan_buffer wlan_buf;

	from_wlan_count++;

/*	if (from_wlan_count - to_host_count > 10)*/
/*	ESP_LOGE(TAG, "WLAN -> SDIO: [%d %d %d]", from_wlan_count, to_host_count, to_host_sent_count);*/

	if (!buffer || !eb) {
		if (eb) {
			esp_wifi_internal_free_rx_buffer(eb);
		}
		return ESP_OK;
	}

	buf_size = xRingbufferGetCurFreeSize(rb_handle_wlan_to_sdio);
	if (buf_size < sizeof(wlan_buf)) {
		ESP_LOGE(TAG, "WLAN -> SDIO: Not enough buffer space available: %d\n", buf_size);
		goto DONE;
	}

	/* Prepare buffer descriptor */
	memset(&wlan_buf, 0, sizeof(wlan_buf));

	wlan_buf.if_type = STA_INTF;
	wlan_buf.len = len;
	wlan_buf.buf = buffer;
	wlan_buf.buf_ptr = eb;

	ret = xRingbufferSend(rb_handle_wlan_to_sdio, &wlan_buf, sizeof(wlan_buf), portMAX_DELAY);

	if (ret != pdTRUE) {
		ESP_LOGE(TAG, "WLAN -> SDIO: Failed to write to ring buffer\n");
		goto DONE;
	}

	return ESP_OK;

DONE:
	esp_wifi_internal_free_rx_buffer(eb);
	return ESP_OK;
}


//reset counters of the slave hardware, and clean the receive buffer (normally they should be sent back to the host)
static esp_err_t slave_reset()
{
    esp_err_t ret;
    sdio_slave_stop();
    ret = sdio_slave_reset();
    if (ret != ESP_OK) return ret;
    ret = sdio_slave_start();
    if (ret != ESP_OK) return ret;

    //Since the buffer will not be sent any more, we return them back to receving driver
    while(1) {
        sdio_slave_buf_handle_t handle;
        ret = sdio_slave_send_get_finished(&handle, 0);
        if (ret != ESP_OK) break;
        ret = sdio_slave_recv_load_buf(handle);
        ESP_ERROR_CHECK(ret);
    }
    return ESP_OK;
}

int32_t write_data(uint8_t if_type, uint8_t if_num, uint8_t* data, int32_t len)
{
    esp_err_t ret = ESP_OK;
    int32_t total_len = len + sizeof (struct payload_header);
    uint8_t* sendbuf = NULL;
    struct payload_header *header;

    if (len < 0 || data == NULL) {
        ESP_LOGE(TAG , "Write data error, len:%d", len);
        return -1;
    }

    sendbuf = heap_caps_malloc(total_len, MALLOC_CAP_DMA);
    if (sendbuf == NULL) {
        ESP_LOGE(TAG , "Malloc send buffer fail!");
        return 0;
    }

    header = (struct payload_header *) sendbuf;

    memset (header, 0, sizeof(struct payload_header));

    /* Initialize header */
    header->pkt_type = DATA_PACKET;
    header->if_type = if_type;
    header->if_num = if_num;
    header->len = len;
    header->offset = sizeof(struct payload_header);

    memcpy(sendbuf + header->offset, data, len);

    ret = sdio_slave_transmit(sendbuf, total_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG , "sdio slave transmit error, ret : 0x%x\r\n", ret);
    }


    free(sendbuf);
    return len;
}

//we use the event callback (in ISR) in this example to get higer responding speed
//note you can't do delay in the ISR
//``sdio_slave_wait_int`` is another way to handle interrupts
static void event_cb(uint8_t pos)
{
    ESP_EARLY_LOGE(TAG, "event: %d", pos);
#if 1
    switch(pos) {
        case START_DATA_PATH:
		datapath = 1;
		break;

	case STOP_DATA_PATH:
		datapath = 0;
		break;
    }
#endif
}

/* Send data to host */
void send_task(void* pvParameters)
{
	wlan_buffer *wlan_buf = NULL;
	size_t size = 0;
	int t1, t2, t_total = 0;
	int d_total = 0;


	while (1) {
#if 0
		if (datapath && !started) {
			ESP_LOGE (TAG, "Start WIFI\n");
			ESP_ERROR_CHECK( esp_wifi_start() );

			/* register RX callback */
			esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t)wlan_rx_callback);
			started = 1;
			usleep(1000);
		}
#endif

		wlan_buf = (wlan_buffer *) xRingbufferReceive(rb_handle_wlan_to_sdio, &size, portMAX_DELAY);

		if (datapath) {
			if (wlan_buf) {
				to_host_count++;
				/* Send data */
				t1 = XTHAL_GET_CCOUNT();

/*				xSemaphoreTake(sdio_write_lock, (portTickType)portMAX_DELAY);*/
				write_data(wlan_buf->if_type, 0, wlan_buf->buf, wlan_buf->len);
/*				xSemaphoreGive(sdio_write_lock);*/

				t2 = XTHAL_GET_CCOUNT();
				t_total += t2 - t1;
				d_total += wlan_buf->len;

/*				usleep(100);*/
/*				ESP_LOG_BUFFER_HEXDUMP(TAG_TX, wlan_buf->buf, 8, ESP_LOG_INFO);*/

				/* Post processing */
				if (wlan_buf->buf_ptr)
					esp_wifi_internal_free_rx_buffer(wlan_buf->buf_ptr);

				vRingbufferReturnItem(rb_handle_wlan_to_sdio, wlan_buf);
				to_host_sent_count++;
			} else {
				/*			ESP_LOGE (TAG_TX, "No data available\n");*/
			}
			if (t_total) {
/*				printf("TX complete. Total time spent in tx = %d for %d bytes\n", t_total, d_total);*/
				t_total = 0;
			}

		} else {
			if (wlan_buf) {
				if (wlan_buf->buf_ptr)
					esp_wifi_internal_free_rx_buffer(wlan_buf->buf_ptr);

				vRingbufferReturnItem(rb_handle_wlan_to_sdio, wlan_buf);
			}

			ESP_LOGE (TAG_TX, "Data path stopped");
			sleep(1);
		}
	}
}

void wlan_rx_task(void* pvParameters)
{
	wlan_buffer *wlan_buf = NULL;
	size_t size = 0;

	while (1) {
		wlan_buf = (wlan_buffer *) xRingbufferReceive(rb_handle_sdio_to_wlan, &size, portMAX_DELAY);

		if (!wlan_buf) {
			continue;
		}

		if ((wlan_buf->if_type == STA_INTF) && sta_connected) {
			esp_wifi_internal_tx(ESP_IF_WIFI_STA, wlan_buf->buf, wlan_buf->len);
		} else if (wlan_buf->if_type == AP_INTF) {
			esp_wifi_internal_tx(ESP_IF_WIFI_AP, wlan_buf->buf, wlan_buf->len);
		} else if (wlan_buf->if_type == SERIAL_INTF) {
			//        write_data(SERIAL_INTF, 0, ptr, length);
			memcpy(r.data, wlan_buf->buf, min(wlan_buf->len, sizeof(r.data)));
			r.valid = 1;
			r.len = min(wlan_buf->len, sizeof(r.data));
			ESP_LOG_BUFFER_HEXDUMP(TAG_RX_S, r.data, r.len, ESP_LOG_INFO);
			esp_at_port_recv_data_notify(r.len, portMAX_DELAY);
		}

		if (wlan_buf->buf_ptr)
			sdio_slave_recv_load_buf(wlan_buf->buf_ptr);

		vRingbufferReturnItem(rb_handle_sdio_to_wlan, wlan_buf);
	}
}

/* Get data from host */
void recv_task(void* pvParameters)
{
	sdio_slave_buf_handle_t handle;
	size_t length = 0, len = 0;
	uint8_t* ptr = NULL;
	struct payload_header *header;
	uint16_t buf_size;
	wlan_buffer wlan_buf;

	for (;;) {

		// receive data from SDIO host
		esp_err_t ret = sdio_slave_recv(&handle, &ptr, &length, portMAX_DELAY);
		if (ret != ESP_OK) {
			/*				ESP_LOGE(TAG, "Recv error,ret:%x", ret);*/
			continue;
		}

		len = length;

		if (length) {
			header = (struct payload_header *) ptr;
			ptr += header->offset;
			length -= header->offset;

			/*				ESP_LOGE(TAG_RX, "%d %d %d %d", length, header->offset, len,*/
			/*						header->len);*/
/*			ESP_LOG_BUFFER_HEXDUMP(TAG_RX, ptr, 8, ESP_LOG_INFO);*/
			buf_size = xRingbufferGetCurFreeSize(rb_handle_sdio_to_wlan);

			if (buf_size < sizeof(wlan_buf)) {
				ESP_LOGE(TAG, "SDIO -> WLAN: Not enough buffer space available: %d\n", buf_size);
				sdio_slave_recv_load_buf(handle);
				continue;
			}

			/* Prepare buffer descriptor */
			memset(&wlan_buf, 0, sizeof(wlan_buf));

			wlan_buf.if_type = header->if_type;
			wlan_buf.len = header->len;
			wlan_buf.buf = ptr;
			wlan_buf.buf_ptr = handle;

			ret = xRingbufferSend(rb_handle_sdio_to_wlan, &wlan_buf, sizeof(wlan_buf), portMAX_DELAY);
			if (ret != pdTRUE) {
				ESP_LOGE(TAG, "SDIO -> WLAN: Failed to write to ring buffer\n");
				sdio_slave_recv_load_buf(handle);
			}

			/*				esp_wifi_internal_tx(ESP_IF_WIFI_STA, ptr, header->len);*/
			// free recv buffer
			/*	usleep(100);*/
		}
	}
}

static int32_t at_sdio_hosted_read_data(uint8_t *data, int32_t len)
{
    ESP_LOGE(TAG, "at_sdio_hosted_read_data\n");
    len = min(len, r.len);
    if (r.valid) {
        memcpy(data, r.data, len);
        r.valid = 0;
        r.len = 0;
    } else {
        printf("No data to be read\n");
    }
    return len;
}
static int32_t at_sdio_hosted_write_data(uint8_t* data, int32_t len)
{
    ESP_LOGE(TAG, "at_sdio_hosted_write_data %d\n", len);

/*    xSemaphoreTake(sdio_write_lock, (portTickType)portMAX_DELAY);*/
    write_data(SERIAL_INTF, 0, data, len);
/*    xSemaphoreGive(sdio_write_lock);*/

    ESP_LOG_BUFFER_HEXDUMP(TAG_TX_S, data, len, ESP_LOG_INFO);
    return len;
}
uint32_t esp_at_get_task_stack_size(void)
{
    return 4096;
}
void at_interface_init(void)
{
/*    xSemaphoreTake(sdio_write_lock, (portTickType)portMAX_DELAY);*/
/*    write_data(SERIAL_INTF, 0, (uint8_t *)"\r\nready\r\n", 9);*/
/*    xSemaphoreGive(sdio_write_lock);*/

    esp_at_device_ops_struct esp_at_device_ops = {
        .read_data = at_sdio_hosted_read_data,
        .write_data = at_sdio_hosted_write_data,
        .get_data_length = NULL,
        .wait_write_complete = NULL,
    };
    esp_at_device_ops_regist(&esp_at_device_ops);
}
void at_set_echo_flag(bool enable);
static esp_err_t at_wifi_event_handler(void *ctx, system_event_t *event)
{
    esp_err_t ret = esp_at_wifi_event_handler(ctx, event);

    if (event->event_id == SYSTEM_EVENT_STA_CONNECTED) {
	    ESP_LOGE (TAG, "STA connected: Registered callback\n");
	    esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_STA, (wifi_rxcb_t) wlan_sta_rx_callback);
	    sta_connected = 1;
    } else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED) {
	    ESP_LOGE (TAG, "STA disconnected\n");
	    sta_connected = 0;
    } else if (event->event_id == SYSTEM_EVENT_AP_START) {
	    ESP_LOGE (TAG, "AP START\n");
	    esp_wifi_internal_reg_rxcb(ESP_IF_WIFI_AP, (wifi_rxcb_t) wlan_ap_rx_callback);
    }

    return ret;
}

static void initialise_wifi(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK( esp_event_loop_init(at_wifi_event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_init_internal(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

//Main application
void app_main()
{
    esp_err_t ret;

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
#ifdef CONFIG_SDIO_DAT2_DISABLED
    /* For slave chips with 3.3V flash, DAT2 pullup conflicts with the pulldown
       required by strapping pin (MTDI). We can either burn the EFUSE for the
       strapping or just disable the DAT2 and work in 1-bit mode.
     */
    config.flags |= SDIO_SLAVE_FLAG_DAT2_DISABLED;
#endif
    sdio_slave_buf_handle_t handle;

    ret = sdio_slave_initialize(&config);
    ESP_ERROR_CHECK(ret);

    for(int i = 0; i < BUFFER_NUM; i++) {
        handle = sdio_slave_recv_register_buf(buffer[i]);
        assert(handle != NULL);

        ret = sdio_slave_recv_load_buf(handle);
        ESP_ERROR_CHECK(ret);
    }

    sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
            SDIO_SLAVE_HOSTINT_BIT0);

    sdio_slave_start();

    rb_handle_wlan_to_sdio = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
    assert(rb_handle_wlan_to_sdio != NULL);

    rb_handle_sdio_to_wlan = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
    assert(rb_handle_sdio_to_wlan != NULL);

    //Initialize NVS
    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    sdio_write_lock = xSemaphoreCreateBinary();

    assert (sdio_write_lock != NULL);

    /* by default it will be in taken state.. release it */
    xSemaphoreGive(sdio_write_lock);

    ESP_LOGI(TAG, EV_STR("slave ready"));
    xTaskCreate(recv_task , "sdio_recv_task" , 4096 , NULL , 18 , NULL);
    xTaskCreate(send_task , "sdio_send_task" , 4096 , NULL , 18 , NULL);
    xTaskCreate(wlan_rx_task , "wlan_task" , 4096 , NULL , 18 , NULL);

    at_interface_init();
    esp_at_module_init(1, (uint8_t *)"custom_version 1.0");
    at_set_echo_flag(false);
    if(esp_at_base_cmd_regist() == false) {
        printf("regist base cmd fail\r\n");
        return 0;
    }
#if 1
    nvs_flash_init();
    tcpip_adapter_init();
    initialise_wifi();
    if(esp_at_wifi_cmd_regist() == false) {
        printf("regist wifi cmd fail\r\n");
    }
#endif
}


