/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2020 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include <linux/mutex.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "esp_if.h"
#include "esp_sdio_api.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif
#include <linux/kthread.h>

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	printk(KERN_ERR "%s: CMD53 read/write error at %d\n", __func__, __LINE__);	\
} while (0);

struct esp_sdio_context sdio_context;

#ifdef CONFIG_ENABLE_MONITOR_PROCESS
struct task_struct *monitor_thread;
#endif

static int init_context(struct esp_sdio_context *context);
static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size);
/*int deinit_context(struct esp_adapter *adapter);*/

static const struct sdio_device_id esp_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_2) },
	{}
};

static void print_capabilities(u32 cap)
{
	printk(KERN_INFO "Features supported are:\n");
	if (cap & ESP_WLAN_SDIO_SUPPORT)
		printk(KERN_INFO "\t * WLAN\n");
	if ((cap & ESP_BT_UART_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT)) {
		printk(KERN_INFO "\t * BT/BLE\n");
		if (cap & ESP_BT_UART_SUPPORT)
			printk(KERN_INFO "\t   - HCI over UART\n");
		if (cap & ESP_BT_SDIO_SUPPORT)
			printk(KERN_INFO "\t   - HCI over SDIO\n");

		if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
			printk(KERN_INFO "\t   - BT/BLE dual mode\n");
		else if (cap & ESP_BLE_ONLY_SUPPORT)
			printk(KERN_INFO "\t   - BLE only\n");
		else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
			printk(KERN_INFO "\t   - BR EDR only\n");
	}
}

static void esp_process_interrupt(struct esp_sdio_context *context, u32 int_status)
{
	if (!context) {
		return;
	}

	if (int_status & ESP_SLAVE_RX_NEW_PACKET_INT) {
		esp_process_new_packet_intr(context->adapter);
	}
}

static void esp_handle_isr(struct sdio_func *func)
{
	struct esp_sdio_context *context = NULL;
	u32 int_status = 0;
	int ret;

	if (!func) {
		return;
	}

	context = sdio_get_drvdata(func);

	if (!context) {
		return;
	}

	/* Read interrupt status register */
	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) &int_status, sizeof(int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	esp_process_interrupt(context, int_status);

	/* Clear interrupt status */
	ret = esp_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) &int_status, sizeof(int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);
}

int generate_slave_intr(struct esp_sdio_context *context, u8 data)
{
	if (!context)
		return -EINVAL;

	return esp_write_reg(context, ESP_SLAVE_SCRATCH_REG_7, &data,
			sizeof(data), ACQUIRE_LOCK);
}

static void deinit_sdio_func(struct sdio_func *func)
{
	sdio_claim_host(func);
	/* Release IRQ */
	sdio_release_irq(func);
	/* Disable sdio function */
	sdio_disable_func(func);
	sdio_release_host(func);
	sdio_set_drvdata(func, NULL);
}

static int esp_slave_get_tx_buffer_num(struct esp_sdio_context *context, u32 *tx_num, u8 is_lock_needed)
{
    u32 len;
    int ret = 0;

    ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (uint8_t*)&len, 4, is_lock_needed);

    if (ret)
	    return ret;

    len = (len >> 16) & ESP_TX_BUFFER_MASK;
    len = (len + ESP_TX_BUFFER_MAX - context->tx_buffer_count) % ESP_TX_BUFFER_MAX;

    *tx_num = len;

    return ret;
}

static int esp_get_len_from_slave(struct esp_sdio_context *context, u32 *rx_size, u8 is_lock_needed)
{
    u32 len, temp;
    int ret = 0;

    ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
		    (u8 *) &len, sizeof(len), is_lock_needed);

    if (ret)
	    return ret;

    len &= ESP_SLAVE_LEN_MASK;

    if (len >= context->rx_byte_count)
	    len = (len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
    else {
	    /* Handle a case of roll over */
	    temp = ESP_RX_BYTE_MAX - context->rx_byte_count;
	    len = temp + len;

	    if (len > ESP_RX_BUFFER_SIZE) {
		    printk(KERN_INFO "%s: Len from slave[%d] exceeds max [%d]\n",
				    __func__, len, ESP_RX_BUFFER_SIZE);
	    }
    }
    *rx_size = len;

    return 0;
}


static void flush_sdio(struct esp_sdio_context *context)
{
	struct sk_buff *skb;

	if (!context || !context->adapter)
		return;

	while (1) {
		skb = read_packet(context->adapter);

		if (!skb) {
			break;
		}

		if (skb->len)
			printk (KERN_INFO "%s: Flushed %d bytes\n", __func__, skb->len);
		dev_kfree_skb(skb);
	}
}

static void esp_remove(struct sdio_func *func)
{
	struct esp_sdio_context *context;
	context = sdio_get_drvdata(func);

	printk(KERN_INFO "%s -> Remove card", __func__);

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	esp_serial_cleanup();
#endif

#ifdef CONFIG_ENABLE_MONITOR_PROCESS
	if (monitor_thread)
		kthread_stop(monitor_thread);
#endif

	if (context) {
		generate_slave_intr(context, BIT(ESP_CLOSE_DATA_PATH));
		msleep(100);

		flush_sdio(context);

		if (context->adapter) {
			esp_remove_card(context->adapter);

			if (context->adapter->hcidev) {
				esp_deinit_bt(context->adapter);
			}

		}

		memset(context, 0, sizeof(struct esp_sdio_context));
	}

	deinit_sdio_func(func);

	printk (KERN_INFO "%s: Context deinit %d - %d\n", __func__, context->rx_byte_count,
			context->tx_buffer_count);

}

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};

static int init_context(struct esp_sdio_context *context)
{
	int ret = 0;
	u32 val = 0;

	if (!context) {
		return -EINVAL;
	}

	/* Initialize rx_byte_count */
	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) &val, sizeof(val), ACQUIRE_LOCK);
	if (ret)
		return ret;

/*	printk(KERN_DEBUG "%s: LEN %d\n", __func__, (val & ESP_SLAVE_LEN_MASK));*/

	context->rx_byte_count = val & ESP_SLAVE_LEN_MASK;

	/* Initialize tx_buffer_count */
	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) &val,
			sizeof(val), ACQUIRE_LOCK);

	if (ret)
		return ret;

	val = ((val >> 16) & ESP_TX_BUFFER_MASK);

/*	printk(KERN_DEBUG "%s: BUF_CNT %d\n", __func__, val);*/

	if (val >= ESP_MAX_BUF_CNT)
		context->tx_buffer_count = val - ESP_MAX_BUF_CNT;
	else
		context->tx_buffer_count = 0;

/*	printk(KERN_DEBUG "%s: Context init %d - %d\n", __func__, context->rx_byte_count,*/
/*			context->tx_buffer_count);*/

	context->adapter = esp_get_adapter();

	if (!context->adapter)
		printk (KERN_ERR "%s: Failed to get adapter\n", __func__);

	context->adapter->if_type = ESP_IF_TYPE_SDIO;

	return ret;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave, data_left, len_to_read, size, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;
	struct esp_sdio_context *context;

	if (!adapter || !adapter->if_context) {
		printk (KERN_ERR "%s: INVALID args\n", __func__);
		return NULL;
	}

	context = adapter->if_context;

	sdio_claim_host(context->func);

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp_get_len_from_slave(context, &len_from_slave, LOCK_ALREADY_ACQUIRED);

/*	printk (KERN_DEBUG "LEN FROM SLAVE: %d\n", len_from_slave);*/

	if (ret || !len_from_slave) {
		sdio_release_host(context->func);
		return NULL;
	}

	size = ESP_BLOCK_SIZE * 4;

	if (len_from_slave > size) {
		printk(KERN_INFO "Rx large packet: %d\n", len_from_slave);
	}

	skb = esp_alloc_skb(len_from_slave);

	if (!skb) {
		printk (KERN_ERR "%s: SKB alloc failed\n", __func__);
		sdio_release_host(context->func);
		return NULL;
	}

	skb_put(skb, len_from_slave);
	pos = skb->data;

	data_left = len_from_slave;

	do {
		num_blocks = data_left/ESP_BLOCK_SIZE;

#if 0
		if (!context->rx_byte_count) {
			start_time = ktime_get_ns();
		}
#endif

		if (num_blocks) {
			len_to_read = num_blocks * ESP_BLOCK_SIZE;
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, len_to_read, LOCK_ALREADY_ACQUIRED);
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3), LOCK_ALREADY_ACQUIRED);
		}

		if (ret) {
			printk (KERN_ERR "%s: Failed to read data\n", __func__);
			dev_kfree_skb(skb);
			sdio_release_host(context->func);
			return NULL;
		}

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;
		context->rx_byte_count = context->rx_byte_count % ESP_RX_BYTE_MAX;

	} while (data_left > 0);

	sdio_release_host(context->func);

	return skb;
}

static int write_packet(struct esp_adapter *adapter, u8 *buf, u32 size)
{
	u32 block_cnt = 0, buf_needed = 0;
	u32 buf_available = 0;
	int ret = 0;
	u8 *pos = NULL;
	u32 data_left, len_to_send, pad;
	struct esp_sdio_context *context;

	if (!adapter || !adapter->if_context || !buf || !size) {
		printk (KERN_ERR "%s: Invalid args\n", __func__);
		return -EINVAL;
	}

	context = adapter->if_context;

	sdio_claim_host(context->func);

	buf_needed = (size + ESP_RX_BUFFER_SIZE - 1) / ESP_RX_BUFFER_SIZE;

	ret = esp_slave_get_tx_buffer_num(context, &buf_available, LOCK_ALREADY_ACQUIRED);

/*	printk(KERN_ERR "%s: TX -> Available [%d], needed [%d]\n", __func__, buf_available, buf_needed);*/

	if (buf_available < buf_needed) {
		printk(KERN_DEBUG "%s: Not enough buffers available: availabale [%d], needed [%d]\n", __func__,
				buf_available, buf_needed);
		sdio_release_host(context->func);
		return -ENOMEM;
	}

	pos = buf;
	data_left = len_to_send = 0;

	data_left = size;
	pad = ESP_BLOCK_SIZE - (data_left % ESP_BLOCK_SIZE);
	data_left += pad;


	do {
		block_cnt = data_left / ESP_BLOCK_SIZE;
		len_to_send = data_left;
		ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
				pos, (len_to_send + 3) & (~3), LOCK_ALREADY_ACQUIRED);

		if (ret) {
			printk (KERN_ERR "%s: Failed to send data\n", __func__);
			sdio_release_host(context->func);
			return ret;
		}
/*		printk (KERN_ERR "--> %d %d %d\n", block_cnt, data_left, len_to_send);*/

		data_left -= len_to_send;
		pos += len_to_send;
	} while (data_left);

	context->tx_buffer_count += buf_needed;
	context->tx_buffer_count = context->tx_buffer_count % ESP_TX_BUFFER_MAX;

	sdio_release_host(context->func);

	return 0;
}

static struct esp_sdio_context * init_sdio_func(struct sdio_func *func)
{
	struct esp_sdio_context *context = NULL;
	int ret = 0;

	if (!func)
		return NULL;

	context = &sdio_context;

	context->func = func;

	sdio_claim_host(func);

	/* Enable Function */
	ret = sdio_enable_func(func);
	if (ret) {
		return NULL;
	}

	/* Register IRQ */
	ret = sdio_claim_irq(func, esp_handle_isr);
	if (ret) {
		sdio_disable_func(func);
		return NULL;
	}

	/* Set private data */
	sdio_set_drvdata(func, context);

	context->state = ESP_CONTEXT_INIT;

	sdio_release_host(func);

	return context;
}

#ifdef CONFIG_ENABLE_MONITOR_PROCESS
static int monitor_process(void *data)
{
	u32 val, intr, len_reg, rdata, old_len = 0;
	struct esp_sdio_context *context = (struct esp_sdio_context *) data;
	struct sk_buff *skb;

	while (!kthread_should_stop()) {
		msleep(5000);

		val = intr = len_reg = rdata = 0;

		esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
				(u8 *) &val, sizeof(val), ACQUIRE_LOCK);

		len_reg = val & ESP_SLAVE_LEN_MASK;

		val = 0;
		esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) &val,
				sizeof(val), ACQUIRE_LOCK);

		rdata = ((val >> 16) & ESP_TX_BUFFER_MASK);

		esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
				(u8 *) &intr, sizeof(intr), ACQUIRE_LOCK);


		if (len_reg > context->rx_byte_count) {
			if (old_len && (context->rx_byte_count == old_len)) {
				printk (KERN_DEBUG "Monitor thread ----> [%d - %d] [%d - %d] %d\n",
						len_reg, context->rx_byte_count,
						rdata, context->tx_buffer_count, intr);

				skb = read_packet(context->adapter);

				if (!skb)
					continue;

				if (skb->len)
					printk (KERN_DEBUG "%s: Flushed %d bytes\n", __func__, skb->len);

				/* drop the packet */
				dev_kfree_skb(skb);
			}
		}

		old_len = context->rx_byte_count;
	}

	do_exit(0);
	return 0;
}
#endif

static int esp_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct esp_sdio_context *context = NULL;
	int ret = 0;
	uint32_t cap = 0;

	if (func->num != 1) {
		return -EINVAL;
	}

	printk(KERN_INFO "%s: ESP network device detected\n", __func__);

	context = init_sdio_func(func);

	if (!context) {
		return -ENOMEM;
	}

	generate_slave_intr(context, BIT(ESP_RESET));

	msleep(200);

	ret = init_context(context);
	if (ret) {
		deinit_sdio_func(func);
		return ret;
	}

#ifdef CONFIG_SUPPORT_ESP_SERIAL
	ret = esp_serial_init((void *) context->adapter);
	if (ret != 0) {
		esp_remove(func);
		printk(KERN_ERR "Error initialising serial interface\n");
		return ret;
	}
#endif

	ret = esp_add_card(context->adapter);
	if (ret) {
		esp_remove(func);
		printk (KERN_ERR "Failed to add card\n");
		deinit_sdio_func(func);
		return ret;
	}

	/* Read slave capabilities */
	esp_read_reg(context, ESP_SLAVE_SCRATCH_REG_0,
			(u8 *) &cap, sizeof(cap), ACQUIRE_LOCK);

	context->adapter->capabilities = cap;

	print_capabilities(cap);

	if (esp_is_bt_supported_over_sdio(cap)) {
		esp_init_bt(context->adapter);
	}

	context->state = ESP_CONTEXT_READY;

#ifdef CONFIG_ENABLE_MONITOR_PROCESS
	monitor_thread = kthread_run(monitor_process, context, "Monitor process");

	if (!monitor_thread)
		printk (KERN_ERR "Failed to create monitor thread\n");
#endif


	msleep(200);
	generate_slave_intr(context, BIT(ESP_OPEN_DATA_PATH));
	return ret;
}

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= "esp_sdio",
	.id_table	= esp_devices,
	.probe		= esp_probe,
	.remove		= esp_remove,
};

int esp_init_interface_layer(struct esp_adapter *adapter)
{
	if (!adapter)
		return -EINVAL;

	adapter->if_context = &sdio_context;
	adapter->if_ops = &if_ops;
	sdio_context.adapter = adapter;

	return sdio_register_driver(&esp_sdio_driver);
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}
