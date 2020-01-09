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
#define BUFFER_NUM      10

#define EV_STR(s) "================ "s" ================"
static const char TAG[] = "example_slave";
static const char TAG_RX[] = "RX";
//static const char TAG_TX[] = "TX";
uint8_t buffer[BUFFER_NUM][BUFFER_SIZE];
uint8_t buf[BUFFER_SIZE];
volatile uint8_t action = 0;

enum PACKET_TYPE {
	DATA_PACKET = 0,
};

enum INTERFACE_TYPE {
	STA_INTF = 0,
	AP_INTF,
    SERIAL_INTF = (1<<1),
};
struct payload_header {
	uint8_t 			     pkt_type:2;
	uint8_t 			     if_type:3;
	uint8_t 			     if_num:3;
	uint8_t			         reserved1;
	uint16_t                 len;
	uint16_t                 offset;
	uint8_t                  reserved2[2];
} __attribute__((packed));

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
    action = 1;
#if 1
    switch(pos) {
        case 0:
            break;
    }
#endif
}

void send_task(void* pvParameters)
{
	int count = 1;
	uint8_t temp[1300] = {0x01, 0x00, 0x5E, 0x00, 0x00, 0x0D,  0xC2, 0x03, 0x3D, 0x80, 0x00, 0x01, 0x08, 0x00, 0x45, 0xC0, 0x00, 0x36, 0x00, 0xA3, 0x00, 0x00, 0x01, 0x67, 0xCD, 0xE4, 0x0A, 0x00, 0x00, 0x0D, 0xE0, 0x00, 0x00, 0x0D, 0x20, 0x00, 0xB5, 0x2E, 0x00, 0x01, 0x00, 0x02, 0x00, 0x69, 0x00, 0x14, 0x00, 0x04, 0xD7, 0x70, 0x51, 0xAB, 0x00, 0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x15, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00};
    int t1, t2, t_total = 0;
    int d_total = 0;
	memcpy(buf, temp, sizeof(temp));

    	while (1) {

	    if (action) {
		    sleep(4);
		    ESP_LOGE(TAG, "send data to host");
		    action = 0;

		    while (count <= 10) {
#if 0
			    for (int i=0; i < BUFFER_SIZE; i++) {
				    buf[i] = count;
			    }
#endif

			    count++;
//			    ESP_LOGE(TAG, "Tx: %d", count);
                t1 = XTHAL_GET_CCOUNT();
			    write_data(STA_INTF, 0, buf, sizeof(temp));
                t2 = XTHAL_GET_CCOUNT();
                t_total += t2 - t1;
                d_total += sizeof(temp);
//			    usleep(10000);
		    }

		    count = 0;
	    }
        if (t_total) {
            printf("TX complete. Total time spent in tx = %d for %d bytes\n", t_total, d_total);
            t_total = 0;
        }
	    sleep(1);
    }
}

static struct rx_data {
    uint8_t valid;
    int len;
    uint8_t data[1024];
} r;
//#define min(x, y) ((x) < (y) ? (x) : (y))
static inline int min(int x, int y) {
    return (x < y) ? x : y;
}
void recv_task(void* pvParameters)
{
    sdio_slave_buf_handle_t handle;
    size_t length = 0;
    uint8_t* ptr = NULL;
    struct payload_header *header;

    for (;;) {

        // receive data from SDIO host
        esp_err_t ret = sdio_slave_recv(&handle, &ptr, &length, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Recv error,ret:%x", ret);
            continue;
        }

	header = (struct payload_header *) ptr;

	ptr += header->offset;
	length -= header->offset;

	if (length) {
        ESP_LOGE(TAG_RX, "Recv %d %d %d %d %d\n", header->pkt_type, header->if_type, header->if_num, header->len, header->offset);
		ESP_LOG_BUFFER_HEXDUMP(TAG_RX, ptr, 8, ESP_LOG_INFO);
	}

    /* Implement echo functionality for serial interface */
    if (header->if_type == SERIAL_INTF) {
//        write_data(SERIAL_INTF, 0, ptr, length);
        memcpy(r.data, ptr, min(length, sizeof(r.data)));
        r.valid = 1;
        r.len = min(length, sizeof(r.data));
        esp_at_port_recv_data_notify(length, portMAX_DELAY);
    }
	// free recv buffer
	sdio_slave_recv_load_buf(handle);
/*	usleep(100);*/
    }
}
static int32_t at_sdio_hosted_read_data(uint8_t *data, int32_t len)
{
    printf("at_sdio_hosted_read_data\n");
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
    printf("at_sdio_hosted_write_data %d\n", len);
    write_data(SERIAL_INTF, 0, data, len);
    return len;
}
uint32_t esp_at_get_task_stack_size(void)
{
    return 4096;
}
void at_interface_init(void)
{
    write_data(SERIAL_INTF, 0, (uint8_t *)"\r\nready\r\n", 9);
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

    ESP_LOGI(TAG, EV_STR("slave ready"));
    xTaskCreate(recv_task , "at_sdio_recv_task" , 4096 , NULL , 18 , NULL);
    xTaskCreate(send_task , "at_sdio_send_task" , 4096 , NULL , 18 , NULL);

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


