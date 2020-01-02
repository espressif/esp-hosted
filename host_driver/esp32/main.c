#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include "esp_sdio_decl.h"
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include "esp_sdio_api.h"
#include "esp.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mangesh Malusare <mangesh.malusare@espressif.com>");
MODULE_DESCRIPTION("SDIO driver for ESP32 module");
MODULE_VERSION("0.01");

u8 *buf;
u8 *rx_buf;

struct esp_adapter *adapter;

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	printk(KERN_ERR "%s: CMD53 read/write error at %d\n", __func__, __LINE__);	\
} while (0);


static const struct sdio_device_id esp32_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID, ESP_DEVICE_ID_2) },
	{}
};


static int esp32_get_len_from_slave(struct esp32_sdio_context *context, u32 *rx_size)
{
    u32 len;
    int ret = 0;

    ret = esp32_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
		    (u8 *) &len, sizeof(len));

    if (ret)
	    return ret;

    len &= ESP_SLAVE_LEN_MASK;

    len = (len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
    *rx_size = len;
    return 0;
}

u64 start_time, end_time;

static int esp32_get_packets(struct esp32_sdio_context *context)
{
	u32 len_from_slave, data_left, len_to_read, size, num_blocks;
	int ret = 0;
	u8 *pos;

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp32_get_len_from_slave(context, &len_from_slave);

	if (ret)
		return ret;

	if (!rx_buf) {
		printk(KERN_ERR "No rx buffers available on host\n");
		return -ENOMEM;
	}

	size = ESP_BLOCK_SIZE * 4;
/*	printk(KERN_ERR "MANGESH: %s: Slave has a data: 0x%x\n", __func__, len_from_slave);*/

	pos = rx_buf;

	if (len_from_slave > size) {
		len_from_slave = size;
	}

	data_left = len_from_slave;

	do {
		num_blocks = data_left/ESP_BLOCK_SIZE;

		if (!context->rx_byte_count) {
			start_time = ktime_get_ns();
		}

		if (num_blocks) {
			len_to_read = num_blocks * ESP_BLOCK_SIZE;
			ret = esp32_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, len_to_read);
		} else {
			len_to_read = data_left;
			/* 4 byte aligned length */
			ret = esp32_read_block(context,
					ESP_SLAVE_CMD53_END_ADDR - len_to_read,
					pos, (len_to_read + 3) & (~3));
		}

		if (ret) {
			printk (KERN_ERR "%s: Failed to read data\n", __func__);
			return ret;
		}

		data_left -= len_to_read;
		pos += len_to_read;
		context->rx_byte_count += len_to_read;

	} while (data_left);

	if (context->rx_byte_count >= 204800) {
		end_time = ktime_get_ns();
		printk(KERN_ERR "---> Total bytes received: %d\n", context->rx_byte_count);
		printk(KERN_ERR "---> In time: %llu\n", (end_time - start_time));
	}

/*	print_hex_dump_bytes("Rx:", DUMP_PREFIX_NONE, buf, len_from_slave);*/
	printk(KERN_ERR "%s RX --> %d\n", __func__, rx_buf[0]);

	return ret;
}

static int esp32_slave_get_tx_buffer_num(struct esp32_sdio_context *context, u32 *tx_num)
{
    u32 len;
    int ret = 0;

    ret = esp32_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (uint8_t*)&len, 4);

    if (ret)
	    return ret;

    len = (len >> 16) & ESP_TX_BUFFER_MASK;
/*    printk (KERN_ERR "%s: Buf cnt form reg %d\n", __func__, len);*/
    len = (len + ESP_TX_BUFFER_MAX - context->tx_buffer_count) % ESP_TX_BUFFER_MAX;

    *tx_num = len;

    return ret;
}

static int esp32_send_packet(struct esp32_sdio_context *context, u8 *buf, u32 size)
{
	u32 block_cnt = 0, buf_needed = 0;
	u32 buf_available = 0;
	int ret = 0;
	u8 *pos = NULL;
	u32 data_left, len_to_send;

	buf_needed = (size + ESP_RX_BUFFER_SIZE - 1) / ESP_RX_BUFFER_SIZE;

	ret = esp32_slave_get_tx_buffer_num(context, &buf_available);

	printk(KERN_ERR "%s: TX -> Available [%d], needed [%d]\n", __func__, buf_available, buf_needed);

	if (buf_available < buf_needed) {
		printk(KERN_ERR "%s: Not enough buffers available\n", __func__);
		return -ENOMEM;
	}

	pos = buf;
	data_left = len_to_send = 0;

	data_left = size;

	do {
		block_cnt = data_left / ESP_BLOCK_SIZE;

		if (block_cnt) {
			len_to_send = block_cnt * ESP_BLOCK_SIZE;
			ret = esp32_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, len_to_send);
		} else {
			len_to_send = data_left;
			ret = esp32_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, (len_to_send + 3) & (~3));
		}

		if (ret) {
			printk (KERN_ERR "%s: Failed to send data\n", __func__);
			return ret;
		}

		data_left -= len_to_send;
		pos += len_to_send;
	} while (data_left);
	context->tx_buffer_count += buf_needed;


	return 0;
}

static void esp32_process_interrupt(struct esp32_sdio_context *context, u32 int_status)
{
	struct esp_adapter *adapter = container_of(context, struct esp_adapter, context);

	if (!context) {
		return;
	}

	sdio_release_host(context->func);
	if (int_status & ESP_SLAVE_RX_NEW_PACKET_INT) {
/*		esp32_get_packets(context);*/

#if 1
		if (adapter) {
/*			printk (KERN_ERR "%s: NEW PACKET INT from interface: %d\n",*/
/*					__func__, adapter->if_type);*/
			queue_work(adapter->rx_workqueue, &adapter->rx_work);

		}
#endif
	}
	sdio_claim_host(context->func);

	if (int_status & ESP_SLAVE_RX_UNDERFLOW_INT) {
		printk(KERN_ERR "MANGESH: %s: Buufer underflow at Slave\n", __func__);
	}

	if (int_status & ESP_SLAVE_TX_OVERFLOW_INT) {
		printk(KERN_ERR "MANGESH: %s: Buffer overflow at Slave\n", __func__);
	}
}

static void esp32_handle_isr(struct sdio_func *func)
{
	struct esp32_sdio_context *context = NULL;
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
	ret = esp32_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) &int_status, sizeof(int_status));

	/* Clear interrupt status */
	ret = esp32_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) &int_status, sizeof(int_status));
	if (int_status)
/*		printk (KERN_ERR "MANGESH: %s -> interrupt status [0x%x]\n", __func__, int_status);*/

	CHECK_SDIO_RW_ERROR(ret);

	esp32_process_interrupt(context, int_status);

	CHECK_SDIO_RW_ERROR(ret);
}

static int esp32_register_sdio_interrupt(struct sdio_func *func)
{
	int ret;

	sdio_claim_host(func);
	ret = sdio_claim_irq(func, esp32_handle_isr);
	sdio_release_host(func);

	return ret;
}

static int esp32_probe(struct sdio_func *func,
				  const struct sdio_device_id *id)
{
	struct esp32_sdio_context *context = NULL;
	int ret = 0, i, j =0;
	u8 data = 0;
	u32 len = 0;

#if 0
	printk(KERN_ERR "MANGESH: %s -> Probe Start", __func__);

	if (id)
		printk(KERN_ERR "MANGESH: %s -> Device: %d %d %d\n", __func__, id->class,
				id->vendor, id->device);

	if (func)
		printk(KERN_ERR "MANGESH: %s -> Function num: %d\n", __func__, func->num);
#endif
	if (func->num != 1) {
		return -EINVAL;
	}

	context = &adapter->context;

	context->func = func;

/*	ret = esp32_register_sdio_interrupt(func);*/

	sdio_claim_host(func);

	/* Enable Function */
	ret = sdio_enable_func(func);

	/* Register IRQ */
	ret = sdio_claim_irq(func, esp32_handle_isr);

	/* Set private data */
	sdio_set_drvdata(func, context);

	sdio_release_host(func);

#if 0
	/* Read length */
	ret = esp32_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) &len, sizeof(len));
	printk (KERN_ERR "MANGESH-> Wait for data from slave: 0x%x\n", len);
#endif

	buf = kmalloc(2048, GFP_KERNEL);

	if (buf)
		memset(buf, 0, 2048);

	rx_buf = kmalloc(2048, GFP_KERNEL);

	if (rx_buf)
		memset(rx_buf, 0, 2048);

	msleep(200);
	data = 1;
	esp32_write_reg(context, (ESP_SLAVE_SCRATCH_REG_7), &data, sizeof(data));
#if 0
	for (j = 0; j < 100; j++) {
		for (i = 0; i < 2048; i++)
			buf[i] = j+1;

		ret = esp32_send_packet(context, buf, 2048);
		msleep(1);

/*		printk (KERN_ERR "---> Wrote 512 bytes [%d]\n", ret);*/
	}

#endif
	return ret;
}

static void esp32_remove(struct sdio_func *func)
{
	struct esp32_sdio_context *context;

	printk(KERN_ERR "MANGESH: %s -> Remove card", __func__);

	if (func)
		printk(KERN_ERR "MANGESH: %s -> Function num: %d\n", __func__, func->num);

	if (buf)
		kfree(buf);

	if (rx_buf)
		kfree(rx_buf);

	sdio_claim_host(func);
	/* Release IRQ */
	sdio_release_irq(func);
	/* Disable sdio function */
	sdio_disable_func(func);
	sdio_release_host(func);
	/* Free context memory */
	context = sdio_get_drvdata(func);

	memset(context, 0, sizeof(struct esp32_sdio_context));

}

static void esp_rx_work (struct work_struct *work)
{
	struct esp_adapter *adapter = container_of(work, struct esp_adapter, rx_work);
	esp32_get_packets(&adapter->context);
}

#if 0
static void esp_rx_tasklet(unsigned long data)
{
	struct esp_adapter *adapter = (struct esp_adapter *) data;
	sdio_release_host(adapter->context.func);
	esp32_get_packets(&adapter->context);
	sdio_claim_host(adapter->context.func);
}
#endif

static int init_adapter(void)
{
	int ret = 0;

	adapter = kzalloc(sizeof(struct esp_adapter), GFP_KERNEL);

	if (!adapter) {
		return -ENOMEM;
	}

	adapter->if_type = ESP_IF_TYPE_SDIO;

#if 1
	adapter->rx_workqueue = create_workqueue("ESP_RX_WORK_QUEUE");

	if (!adapter->rx_workqueue) {
		kfree(adapter);
		return -ENOMEM;
	}

	INIT_WORK(&adapter->rx_work, esp_rx_work);
#endif

	return ret;
}

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= "esp32_sdio",
	.id_table	= esp32_devices,
	.probe		= esp32_probe,
	.remove		= esp32_remove,
};

static int __init esp32_init(void)
{
	int ret = 0;
	printk(KERN_INFO "Module start\n");

	/* Init driver */
	ret = init_adapter();

	if (ret)
		return ret;

	sdio_register_driver(&esp_sdio_driver);
	return 0;
}

static void __exit esp32_exit(void)
{
	printk(KERN_INFO "Module end\n");
	sdio_unregister_driver(&esp_sdio_driver);
}

module_init(esp32_init);
module_exit(esp32_exit);
