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
#include "sdkconfig.h"
#include <unistd.h>

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
uint8_t buffer[BUFFER_NUM][BUFFER_SIZE];
uint8_t buf[BUFFER_SIZE];
uint8_t action = 0;

enum PACKET_TYPE {
	DATA_PACKET = 0,
};

enum INTERFACE_TYPE {
	STA_INTF = 0,
	AP_INTF,
};
struct payload_header {
	uint8_t 			     pkt_data;
	uint8_t			             reserved1;
	uint16_t                             len;
	uint16_t                             offset;
	uint8_t                              reserved2[2];
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

int32_t write_data(uint8_t* data, int32_t len)
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
    header->pkt_data = (STA_INTF << 2) | DATA_PACKET;
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
	uint8_t temp[] = {0x01, 0x00, 0x5E, 0x00, 0x00, 0x0D,  0xC2, 0x03, 0x3D, 0x80, 0x00, 0x01, 0x08, 0x00, 0x45, 0xC0, 0x00, 0x36, 0x00, 0xA3, 0x00, 0x00, 0x01, 0x67, 0xCD, 0xE4, 0x0A, 0x00, 0x00, 0x0D, 0xE0, 0x00, 0x00, 0x0D, 0x20, 0x00, 0xB5, 0x2E, 0x00, 0x01, 0x00, 0x02, 0x00, 0x69, 0x00, 0x14, 0x00, 0x04, 0xD7, 0x70, 0x51, 0xAB, 0x00, 0x13, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x15, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00};

	memcpy(buf, temp, sizeof(temp));

    	while (1) {

	    if (action) {
		    sleep(10);
		    ESP_LOGE(TAG, "send data to host");
		    action = 0;

		    while (count <= 100) {
#if 0
			    for (int i=0; i < BUFFER_SIZE; i++) {
				    buf[i] = count;
			    }
#endif

			    count++;
			    ESP_LOGE(TAG, "Tx: %d", count);
			    write_data(buf, sizeof(temp));
			    usleep(10000);
		    }
	    }

	    sleep(1);
    }
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

#if 0
	if (length) {
		ESP_LOGE(TAG, "RX: %d %d  %d %d %d %d", header->pkt_data,
				header->reserved1, header->len,
				header->offset, header->reserved2[0],
				header->reserved2[1]);
		ESP_LOG_BUFFER_HEXDUMP(TAG, ptr, 8, ESP_LOG_INFO);
/*		ESP_LOGE(TAG, "RX: %d", ptr[0]);*/
	}
#endif

	// free recv buffer
	sdio_slave_recv_load_buf(handle);
/*	usleep(100);*/
    }
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
            SDIO_SLAVE_HOSTINT_BIT0
#if 0
	    |
            SDIO_SLAVE_HOSTINT_BIT1|
            SDIO_SLAVE_HOSTINT_BIT2|
            SDIO_SLAVE_HOSTINT_BIT3|
            SDIO_SLAVE_HOSTINT_BIT4|
            SDIO_SLAVE_HOSTINT_BIT5|
            SDIO_SLAVE_HOSTINT_BIT6|
            SDIO_SLAVE_HOSTINT_BIT7
#endif
	    );

    sdio_slave_start();

    ESP_LOGI(TAG, EV_STR("slave ready"));
    xTaskCreate(recv_task , "at_sdio_recv_task" , 4096 , NULL , 18 , NULL);
    xTaskCreate(send_task , "at_sdio_send_task" , 4096 , NULL , 18 , NULL);

#if 0
    int count = 1;
    while (1) {
	    if (action) {
		    ESP_LOGE(TAG, "send data to host");
		    action = 0;

		    while (count <= 100) {
			    for (int i=0; i < BUFFER_SIZE; i++) {
				    buf[i] = count;
			    }

			    count++;
			    write_data(buf, BUFFER_SIZE);
			    usleep(100);
		    }
	    }
	    usleep(100);
    }
#endif
}

#if 0
    ESP_LOGI(TAG, EV_STR("slave ready"));

    for(;;) {
        //receive data and send back to host.
        size_t length;
        uint8_t *ptr;

        const TickType_t non_blocking = 0;
        ret = sdio_slave_recv(&handle, &ptr, &length, non_blocking);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "handle: %p, recv len: %d, data:", handle, length);
            ESP_LOG_BUFFER_HEXDUMP(TAG, ptr, length, ESP_LOG_INFO);
            /* If buffer is no longer used, call sdio_slave_recv_load_buf to return it here.  Since we wants to show how
             * to share large buffers between drivers here (we share between sending and receiving), keep the buffer
             * until the buffer is sent by sending driver.
             */

            //send the received buffer to host, with the handle as the argument
            ret = sdio_slave_send_queue(ptr, length, handle, non_blocking);
            if (ret == ESP_ERR_TIMEOUT) {
                // send failed, direct return the buffer to rx
                ESP_LOGE(TAG, "send_queue full, discard received.");
                ret = sdio_slave_recv_load_buf(handle);
            }
            ESP_ERROR_CHECK(ret);
        }

        // if there's finished sending desc, return the buffer to receiving driver
        for(;;){
            sdio_slave_buf_handle_t handle;
            ret = sdio_slave_send_get_finished(&handle, 0);
            if (ret == ESP_ERR_TIMEOUT) break;
            ESP_ERROR_CHECK(ret);
            ret = sdio_slave_recv_load_buf(handle);
            ESP_ERROR_CHECK(ret);
        }

        if (s_job != 0) {
            for(int i = 0; i < 8; i++) {
                if (s_job & BIT(i)) {
                    ESP_LOGI(TAG, EV_STR("%s"), job_desc[i+1]);
                    s_job &= ~BIT(i);

                    switch(BIT(i)) {
                    case JOB_SEND_INT:
                        ret = task_hostint();
                        ESP_ERROR_CHECK(ret);
                        break;
                    case JOB_RESET:
                        ret = slave_reset();
                        ESP_ERROR_CHECK(ret);
                        break;
                    case JOB_WRITE_REG:
                        ret = task_write_reg();
                        ESP_ERROR_CHECK(ret);
                        break;
                    }
                }
            }
        }
        vTaskDelay(1);
    }
}
#endif

