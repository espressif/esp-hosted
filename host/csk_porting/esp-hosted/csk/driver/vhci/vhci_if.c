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
//

/** Includes **/
#include "vhci_if.h"
#include "adapter.h"
#include "spi_drv.h"

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_queue.h"

/** Macros / Constants **/
#define MAX_VHCI_INTF 				1
#define TO_VHCI_INFT_QUEUE_SIZE 	50

typedef enum { INIT, ACTIVE, DESTROY } vhci_state_e;

typedef struct vhci_handle_s {
    csk_os_queue_t queue;
	uint8_t if_type;
	uint8_t if_num;
	uint8_t state;
	void (* hci_rx_callback)(struct vhci_handle_s *, void *);
	void *arg;
} vhci_handle_t;

/* data structures needed for vhci driver */
static vhci_handle_t *s_vhci_handle[MAX_VHCI_INTF] = {NULL};
static uint8_t conn_num = 0;

int ble_vhci_init(void)
{
	for (int i = 0; i < MAX_VHCI_INTF; i++) {
		s_vhci_handle[i] = NULL;
	}
	return 0;
}

vhci_handle_t *ble_vhci_open(void (*hci_rx_callback)(struct vhci_handle_s *, void *), void *arg)
{
	if (conn_num >= MAX_VHCI_INTF) {
		return NULL;
	}
	vhci_handle_t *vhci_hdl = csk_os_malloc(sizeof(vhci_handle_t));
	if (vhci_hdl == NULL) {
		return NULL;
	}
	vhci_hdl->if_type = ESP_HCI_IF;
	vhci_hdl->if_num = conn_num;
	vhci_hdl->state = INIT;
	vhci_hdl->hci_rx_callback = hci_rx_callback;
	vhci_hdl->arg = arg;
	int ret = csk_os_queue_create(&vhci_hdl->queue,
									TO_VHCI_INFT_QUEUE_SIZE,
									sizeof(interface_buffer_handle_t));
	if (ret != 0) {
		csk_os_free(vhci_hdl);
		return NULL;
	}
	vhci_hdl->state = ACTIVE;
	s_vhci_handle[conn_num++] = vhci_hdl;
	return vhci_hdl;
}

/**
 * @brief Get vhci handle for iface_num
 * @param  iface_num - vhci connection number
 * @retval vhci_hdl - output handle of vhci interface
 */
static vhci_handle_t *get_vhci_handle(const uint8_t iface_num)
{
	if ((iface_num < MAX_VHCI_INTF) && (s_vhci_handle[iface_num]) &&
			(s_vhci_handle[iface_num]->state == ACTIVE)) {
		return s_vhci_handle[iface_num];
	}
	return NULL;
}

void ble_vhci_close(vhci_handle_t *vhci_hdl)
{
	if (vhci_hdl == NULL) {
		return;
	}
	if (vhci_hdl->queue) {
        csk_os_queue_delete(vhci_hdl->queue);
        vhci_hdl->queue = NULL;
	}

	/* reset connection */
	s_vhci_handle[vhci_hdl->if_num] = NULL;
	if (vhci_hdl) {
		csk_os_free(vhci_hdl);
		vhci_hdl = NULL;
	}
	conn_num--;
}

uint8_t *ble_vhci_read(const vhci_handle_t *vhci_hdl, uint16_t *rlen)
{
	/* This is a non-blocking call */
	interface_buffer_handle_t buf_handle = {0};

	/* Initial value */
	*rlen = 0;

	/* check if vhci interface valid */
	if ((!vhci_hdl) || (vhci_hdl->state != ACTIVE)) {
		printk("vhci invalid interface\n\r");
		return NULL;
	}

	/* This is non blocking receive.
	 *
	 * In case higher layer using vhci
	 * interface needs to make blocking read, it should register
	 * vhci_rx_callback through vhci_init.
	 * vhci_rx_callback is notification mechanism to implementer of vhci
	 * interface. Higher layer will understand there is data is ready
	 * through this notification. Then it will call vhci_read API
	 * to receive actual data.
	 *
	 * As an another design option, vhci_rx_callback can also be
	 * thought of incoming data indication, i.e. asynchronous rx
	 * indication, which can be used by higher layer in seperate
	 * dedicated rx task to receive and process rx data.
	 *
	 * In our example, first approach of blocking read is used.
	 */
    int ret = csk_os_queue_receive(vhci_hdl->queue, &buf_handle, 0);
    if (ret != 0) {
        printk("[%s] serial queue recv failed \n", __FUNCTION__ );
        return NULL;
    }

	/* proceed only if payload and length are sane */
	if (!buf_handle.payload || !buf_handle.payload_len) {
		return NULL;
	}

	*rlen = buf_handle.payload_len;

	return buf_handle.payload;
}

int ble_vhci_write(const vhci_handle_t *vhci_hdl, uint8_t *wbuffer, const uint16_t wlen)
{
	if ((!vhci_hdl) || (vhci_hdl->state != ACTIVE)) {
		printk("vhci invalid interface for write\n\r");
		return STM_FAIL;
	}
	return send_to_slave(vhci_hdl->if_type, vhci_hdl->if_num, wbuffer, wlen);
}

int vhci_rx_handler(uint8_t if_num, uint8_t *rxbuff, uint16_t rx_len)
{
	interface_buffer_handle_t buf_handle = {0};
	vhci_handle_t *vhci_hdl = NULL;

	vhci_hdl = get_vhci_handle(if_num);

	if ((!vhci_hdl) || (vhci_hdl->state != ACTIVE)) {
		printk("vhci interface not registered yet\n\r");
		return -1;
	}
	buf_handle.if_type = ESP_HCI_IF;
	buf_handle.if_num = if_num;
	buf_handle.payload_len = rx_len;
	buf_handle.payload = rxbuff;
	buf_handle.priv_buffer_handle = rxbuff;
	buf_handle.free_buf_handle = csk_os_free;

	/* send to vhci queue */
    int ret = csk_os_queue_send(vhci_hdl->queue, &buf_handle, CSK_OS_WAIT_FOREVER);
    if (ret != 0) {
        printk("Failed send vhciif queue[%u]\n\r", if_num);
        return -1;
    }

	/* Indicate higher layer about data ready for consumption */
	if (vhci_hdl->hci_rx_callback) {
		(*vhci_hdl->hci_rx_callback)(vhci_hdl, vhci_hdl->arg);
	}

	return 0;
}

void ble_vhci_free_rx(void *ptr)
{
	return csk_os_free(ptr);
}
