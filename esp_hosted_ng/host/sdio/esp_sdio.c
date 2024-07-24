// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 */
#include "utils.h"
#include <linux/mutex.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include "esp_if.h"
#include "esp_sdio_api.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#include <linux/kthread.h>
#include "esp_stats.h"
#include "esp_utils.h"
#include "include/esp_kernel_port.h"

extern u32 raw_tp_mode;
#define MAX_WRITE_RETRIES       2
#define TX_MAX_PENDING_COUNT    200
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

#define CHECK_SDIO_RW_ERROR(ret) do {			\
	if (ret)						\
	esp_err("CMD53 read/write error at %d\n", __LINE__);	\
} while (0);

struct esp_sdio_context sdio_context;
static atomic_t tx_pending;
static atomic_t queue_items[MAX_PRIORITY_QUEUES];

#ifdef CONFIG_ENABLE_MONITOR_PROCESS
struct task_struct *monitor_thread;
#endif
struct task_struct *tx_thread;

volatile u8 host_sleep;

static int init_context(struct esp_sdio_context *context);
static struct sk_buff *read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
/*int deinit_context(struct esp_adapter *adapter);*/

static const struct sdio_device_id esp_devices[] = {
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_1, ESP_DEVICE_ID_ESP32_2) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_ESP32C6_1) },
	{ SDIO_DEVICE(ESP_VENDOR_ID_2, ESP_DEVICE_ID_ESP32C6_2) },
	{}
};

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
	u32 *int_status;
	int ret;

	if (!func) {
		return;
	}

	if (host_sleep)
		return;

	context = sdio_get_drvdata(func);

	if (!(context) ||
	    !(context->adapter) ||
	    (atomic_read(&context->adapter->state) < ESP_CONTEXT_RX_READY)) {
		return;
	}

	int_status = kmalloc(sizeof(u32), GFP_ATOMIC);

	if (!int_status) {
		return;
	}

	/* Read interrupt status register */
	ret = esp_read_reg(context, ESP_SLAVE_INT_ST_REG,
			(u8 *) int_status, sizeof(*int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	esp_process_interrupt(context, *int_status);

	/* Clear interrupt status */
	ret = esp_write_reg(context, ESP_SLAVE_INT_CLR_REG,
			(u8 *) int_status, sizeof(*int_status), ACQUIRE_LOCK);
	CHECK_SDIO_RW_ERROR(ret);

	kfree(int_status);
}

int generate_slave_intr(void *context, u8 data)
{
	u8 *val;
	int ret = 0;

	context = (struct esp_sdio_context*) context;
	if (!context)
		return -EINVAL;

	val = kmalloc(sizeof(u8), GFP_KERNEL);

	if (!val) {
		return -ENOMEM;
	}

	*val = data;

	ret = esp_write_reg(context, ESP_SLAVE_SCRATCH_REG_7, val,
			sizeof(*val), ACQUIRE_LOCK);

	kfree(val);

	return ret;
}

static void deinit_sdio_func(struct sdio_func *func)
{
	sdio_set_drvdata(func, NULL);
	sdio_claim_host(func);
	/* Release IRQ */
	sdio_release_irq(func);
	/* Disable sdio function */
	sdio_disable_func(func);
	sdio_release_host(func);
}

static int esp_slave_get_tx_buffer_num(struct esp_sdio_context *context, u32 *tx_num, u8 is_lock_needed)
{
	u32 *len = NULL;
	int ret = 0;

	len = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!len) {
		return -ENOMEM;
	}

	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) len, sizeof(*len), is_lock_needed);

	if (ret) {
		kfree(len);
		return ret;
	}

	*len = (*len >> 16) & ESP_TX_BUFFER_MASK;
	*len = (*len + ESP_TX_BUFFER_MAX - context->tx_buffer_count) % ESP_TX_BUFFER_MAX;

	*tx_num = *len;

	kfree(len);
	return ret;
}

int esp_deinit_module(struct esp_adapter *adapter)
{
	/* Second & onward bootup cleanup is not required for SDIO:
	 * As Removal of SDIO triggers complete Deinit and SDIO insertion/
	 * detection, triggers probing which does initialization.
	 */
	return 0;
}

static int esp_get_len_from_slave(struct esp_sdio_context *context, u32 *rx_size, u8 is_lock_needed)
{
	u32 *len;
	u32 temp;
	int ret = 0;

	len = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!len) {
		return -ENOMEM;
	}

	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) len, sizeof(*len), is_lock_needed);

	if (ret) {
		kfree(len);
		return ret;
	}

	*len &= ESP_SLAVE_LEN_MASK;

	if (*len >= context->rx_byte_count)
		*len = (*len + ESP_RX_BYTE_MAX - context->rx_byte_count) % ESP_RX_BYTE_MAX;
	else {
		/* Handle a case of roll over */
		temp = ESP_RX_BYTE_MAX - context->rx_byte_count;
		*len = temp + *len;

		if (*len > ESP_RX_BUFFER_SIZE) {
			esp_info("Len from slave[%d] exceeds max [%d]\n",
					*len, ESP_RX_BUFFER_SIZE);
		}
	}
	*rx_size = *len;

	kfree(len);
	return 0;
}


#if 0
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
			esp_info("Flushed %d bytes\n", skb->len);
		dev_kfree_skb(skb);
		skb = NULL;
	}
}
#endif

static void esp_remove(struct sdio_func *func)
{
	struct esp_sdio_context *context;
	uint8_t prio_q_idx = 0;

	context = sdio_get_drvdata(func);

	if (func->num != 1) {
		return;
	}
#ifdef CONFIG_ENABLE_MONITOR_PROCESS
	if (monitor_thread)
		kthread_stop(monitor_thread);
#endif
	if (context) {
		for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++)
			skb_queue_purge(&(sdio_context.tx_q[prio_q_idx]));
	}

	if (tx_thread)
		kthread_stop(tx_thread);

	if (context) {
		generate_slave_intr(context, BIT(ESP_CLOSE_DATA_PATH));
		msleep(100);

		if (context->adapter) {
			esp_remove_card(context->adapter);

			if (context->adapter->hcidev) {
				esp_deinit_bt(context->adapter);
			}
		}


		if (context->func) {
			deinit_sdio_func(context->func);
			context->func = NULL;
			context->adapter->dev = NULL;
		}
		memset(context, 0, sizeof(struct esp_sdio_context));
	}
	esp_dbg("ESP SDIO cleanup completed\n");
}

static struct esp_if_ops if_ops = {
	.read		= read_packet,
	.write		= write_packet,
};

static int get_firmware_data(struct esp_sdio_context *context)
{
	u32 *val;
	int ret = 0;

	val = kmalloc(sizeof(u32), GFP_KERNEL);

	if (!val) {
		return -ENOMEM;
	}

	/* Initialize rx_byte_count */
	ret = esp_read_reg(context, ESP_SLAVE_PACKET_LEN_REG,
			(u8 *) val, sizeof(*val), ACQUIRE_LOCK);
	if (ret) {
		kfree(val);
		return ret;
	}

	esp_info("Rx Pre ====== %d\n", context->rx_byte_count);
	context->rx_byte_count = *val & ESP_SLAVE_LEN_MASK;
	esp_info("Rx Pos ======  %d\n", context->rx_byte_count);

	/* Initialize tx_buffer_count */
	ret = esp_read_reg(context, ESP_SLAVE_TOKEN_RDATA, (u8 *) val,
			sizeof(*val), ACQUIRE_LOCK);

	if (ret) {
		kfree(val);
		return ret;
	}

	*val = ((*val >> 16) & ESP_TX_BUFFER_MASK);
	esp_info("Tx Pre ======  %d\n", context->tx_buffer_count);

	if (*val >= ESP_MAX_BUF_CNT)
		context->tx_buffer_count = (*val) - ESP_MAX_BUF_CNT;
	else
		context->tx_buffer_count = 0;
	esp_info("Tx Pos ======  %d\n", context->tx_buffer_count);

	kfree(val);
	return ret;
}

static int init_context(struct esp_sdio_context *context)
{
	int ret = 0;
	uint8_t prio_q_idx = 0;

	if (!context) {
		return -EINVAL;
	}

	ret = get_firmware_data(context);
	if (ret)
		return ret;

	context->adapter = esp_get_adapter();

	if (unlikely(!context->adapter))
		esp_err("Failed to get adapter\n");

	for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++) {
		skb_queue_head_init(&(sdio_context.tx_q[prio_q_idx]));
		atomic_set(&queue_items[prio_q_idx], 0);
	}

	context->adapter->if_type = ESP_IF_TYPE_SDIO;

	return ret;
}

static struct sk_buff *read_packet(struct esp_adapter *adapter)
{
	u32 len_from_slave, data_left, len_to_read, size, num_blocks;
	int ret = 0;
	struct sk_buff *skb;
	u8 *pos;
	struct esp_sdio_context *context;

	if (!adapter || !adapter->if_context) {
		esp_err("INVALID args\n");
		return NULL;
	}

	context = adapter->if_context;

	if (!context || !context->func) {
		esp_err("Invalid context/state\n");
		return NULL;
	}

	sdio_claim_host(context->func);

	data_left = len_to_read = len_from_slave = num_blocks = 0;

	/* Read length */
	ret = esp_get_len_from_slave(context, &len_from_slave, LOCK_ALREADY_ACQUIRED);

	if (ret || !len_from_slave) {
		sdio_release_host(context->func);
		return NULL;
	}

	size = ESP_BLOCK_SIZE * 4;

	if (len_from_slave > size) {
		esp_info("Rx large packet: %d\n", len_from_slave);
	}

	skb = esp_alloc_skb(len_from_slave);

	if (!skb) {
		esp_err("SKB alloc failed\n");
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
			esp_err("Failed to read data - %d [%u - %d]\n", ret, num_blocks, len_to_read);
			atomic_set(&context->adapter->state, ESP_CONTEXT_DISABLED);
			dev_kfree_skb(skb);
			skb = NULL;
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

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
	u32 max_pkt_size = ESP_RX_BUFFER_SIZE - sizeof(struct esp_payload_header);
	struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;
	struct esp_skb_cb *cb = NULL;
	uint8_t prio = PRIO_Q_LOW;

	if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len) {
		esp_err("Invalid args\n");
		if (skb) {
			dev_kfree_skb(skb);
			skb = NULL;
		}

		return -EINVAL;
	}

	if (skb->len > max_pkt_size) {
		esp_err("Drop pkt of len[%u] > max SDIO transport len[%u]\n",
				skb->len, max_pkt_size);
		dev_kfree_skb(skb);
		skb = NULL;
		return -EPERM;
	}

	cb = (struct esp_skb_cb *)skb->cb;
	if (cb && cb->priv && (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)) {
		esp_tx_pause(cb->priv);
		dev_kfree_skb(skb);
		skb = NULL;
/*		esp_err("TX Pause busy");*/
		return -EBUSY;
	}

	/* Enqueue SKB in tx_q */
	atomic_inc(&tx_pending);

	/* Notify to process queue */
	if (payload_header->if_type == ESP_INTERNAL_IF)
		prio = PRIO_Q_HIGH;
	else if (payload_header->if_type == ESP_HCI_IF)
		prio = PRIO_Q_MID;
	else
		prio = PRIO_Q_LOW;

	atomic_inc(&queue_items[prio]);
	skb_queue_tail(&(sdio_context.tx_q[prio]), skb);

	return 0;
}

static int is_sdio_write_buffer_available(u32 buf_needed)
{
#define BUFFER_AVAILABLE        1
#define BUFFER_UNAVAILABLE      0

	int ret = 0;
	static u32 buf_available;
	struct esp_sdio_context *context = &sdio_context;
	u8 retry = MAX_WRITE_RETRIES;

	/*If buffer needed are less than buffer available
	  then only read for available buffer number from slave*/
	if (buf_available < buf_needed) {
		while (retry) {
			ret = esp_slave_get_tx_buffer_num(context, &buf_available, ACQUIRE_LOCK);

			if (buf_available < buf_needed) {

				/* Release SDIO and retry after delay*/
				retry--;
				usleep_range(10, 50);
				continue;
			}

			break;
		}
	}

	if (buf_available >= buf_needed)
		buf_available -= buf_needed;

	if (!retry) {
		/* No buffer available at slave */
		return BUFFER_UNAVAILABLE;
	}

	return BUFFER_AVAILABLE;
}

static int tx_process(void *data)
{
	int ret = 0;
	u32 block_cnt = 0;
	u32 buf_needed = 0;
	u8 *pos = NULL;
	u32 data_left, len_to_send, pad;
	struct sk_buff *tx_skb = NULL;
	struct esp_adapter *adapter = (struct esp_adapter *) data;
	struct esp_sdio_context *context = NULL;
	struct esp_skb_cb *cb = NULL;
	u8 retry;

	context = adapter->if_context;

	while (!kthread_should_stop()) {

		if (atomic_read(&context->adapter->state) < ESP_CONTEXT_READY) {
			msleep(10);
			esp_err("not ready");
			continue;
		}

		if (host_sleep) {
			/* TODO: Use wait_event_interruptible_timeout */
			msleep(100);
			continue;
		}

		if (atomic_read(&queue_items[PRIO_Q_HIGH]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_HIGH]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_HIGH]);
		} else if (atomic_read(&queue_items[PRIO_Q_MID]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_MID]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_MID]);
		} else if (atomic_read(&queue_items[PRIO_Q_LOW]) > 0) {
			tx_skb = skb_dequeue(&(context->tx_q[PRIO_Q_LOW]));
			if (!tx_skb) {
				continue;
			}
			atomic_dec(&queue_items[PRIO_Q_LOW]);
		} else {
			/* esp_verbose("not ready high=%d mid=%d low=%d\n",
					atomic_read(&queue_items[PRIO_Q_HIGH]),
					atomic_read(&queue_items[PRIO_Q_MID]),
					atomic_read(&queue_items[PRIO_Q_LOW])); */
			msleep(1);
			continue;
		}

		if (atomic_read(&tx_pending))
			atomic_dec(&tx_pending);

		retry = MAX_WRITE_RETRIES;

		/* resume network tx queue if bearable load */
		cb = (struct esp_skb_cb *)tx_skb->cb;
		if (cb && cb->priv && atomic_read(&tx_pending) < TX_RESUME_THRESHOLD) {
			esp_tx_resume(cb->priv);
#if TEST_RAW_TP
			if (raw_tp_mode != 0) {
				esp_raw_tp_queue_resume();
			}
#endif
		}

		buf_needed = (tx_skb->len + ESP_RX_BUFFER_SIZE - 1) / ESP_RX_BUFFER_SIZE;

		/*If SDIO slave buffer is available to write then only write data
		else wait till buffer is available*/
		ret = is_sdio_write_buffer_available(buf_needed);
		if (!ret) {
			dev_kfree_skb(tx_skb);
			continue;
		}

		pos = tx_skb->data;
		data_left = len_to_send = 0;

		data_left = tx_skb->len;
		pad = ESP_BLOCK_SIZE - (data_left % ESP_BLOCK_SIZE);
		data_left += pad;


		do {
			block_cnt = data_left / ESP_BLOCK_SIZE;
			len_to_send = data_left;
			ret = esp_write_block(context, ESP_SLAVE_CMD53_END_ADDR - len_to_send,
					pos, (len_to_send + 3) & (~3), ACQUIRE_LOCK);

			if (ret) {
				esp_err("Failed to send data: %d %d %d\n", ret, len_to_send, data_left);
				break;
			}

			data_left -= len_to_send;
			pos += len_to_send;
		} while (data_left);

		if (ret) {
			/* drop the packet */
			dev_kfree_skb(tx_skb);
			continue;
		}

		context->tx_buffer_count += buf_needed;
		context->tx_buffer_count = context->tx_buffer_count % ESP_TX_BUFFER_MAX;

		dev_kfree_skb(tx_skb);
		tx_skb = NULL;
	}

	do_exit(0);
	return 0;
}

static struct esp_sdio_context *init_sdio_func(struct sdio_func *func, int *sdio_ret)
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
		esp_err("sdio_enable_func ret: %d\n", ret);
		if (sdio_ret)
			*sdio_ret = ret;
		sdio_release_host(func);

		return NULL;
	}

	/* Register IRQ */
	ret = sdio_claim_irq(func, esp_handle_isr);
	if (ret) {
		esp_err("sdio_claim_irq ret: %d\n", ret);
		sdio_disable_func(func);

		if (sdio_ret)
			*sdio_ret = ret;
		sdio_release_host(func);

		return NULL;
	}

	/* Set private data */
	sdio_set_drvdata(func, context);

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
				esp_dbg("Monitor thread ----> [%d - %d] [%d - %d] %d\n",
						len_reg, context->rx_byte_count,
						rdata, context->tx_buffer_count, intr);

				skb = read_packet(context->adapter);

				if (!skb)
					continue;

				if (skb->len)
					esp_dbg("Flushed %d bytes\n", skb->len);

				/* drop the packet */
				dev_kfree_skb(skb);
				skb = NULL;
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

	if (func->num != 1) {
		return -EINVAL;
	}

	esp_info("ESP network device detected\n");

	context = init_sdio_func(func, &ret);;
	atomic_set(&tx_pending, 0);

	if (!context) {
		if (ret)
			return ret;
		else
			return -EINVAL;
	}

	if (sdio_context.sdio_clk_mhz) {
		struct mmc_host *host = func->card->host;
		u32 hz = sdio_context.sdio_clk_mhz * NUMBER_1M;
		/* Expansion of mmc_set_clock that isnt exported */
		if (hz < host->f_min)
			hz = host->f_min;
		if (hz > host->f_max)
			hz = host->f_max;
		host->ios.clock = hz;
		host->ops->set_ios(host, &host->ios);
	}

	ret = init_context(context);
	if (ret) {
		deinit_sdio_func(func);
		return ret;
	}

	tx_thread = kthread_run(tx_process, context->adapter, "esp_TX");

	if (!tx_thread)
		esp_err("Failed to create esp_sdio TX thread\n");

	context->adapter->dev = &func->dev;
	atomic_set(&context->adapter->state, ESP_CONTEXT_RX_READY);
	generate_slave_intr(context, BIT(ESP_OPEN_DATA_PATH));


#ifdef CONFIG_ENABLE_MONITOR_PROCESS
	monitor_thread = kthread_run(monitor_process, context, "Monitor process");

	if (!monitor_thread)
		esp_err("Failed to create monitor thread\n");
#endif

	esp_dbg("ESP SDIO probe completed\n");

	return ret;
}

static int esp_suspend(struct device *dev)
{
	struct sdio_func *func = NULL;
	struct esp_sdio_context *context = NULL;

	if (!dev) {
		esp_info("Failed to inform ESP that host is suspending\n");
		return -1;
	}

	func = dev_to_sdio_func(dev);

	esp_info("----> Host Suspend\n");
	msleep(1000);

	context = sdio_get_drvdata(func);

	if (!context) {
		esp_info("Failed to inform ESP that host is suspending\n");
		return -1;
	}

	host_sleep = 1;

	generate_slave_intr(context, BIT(ESP_POWER_SAVE_ON));
	msleep(10);

	sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
#if 0
	/* Enale OOB IRQ and host wake up */
	enable_irq(SDIO_OOB_IRQ);
	enable_irq_wake(SDIO_OOB_IRQ);
#endif
	return 0;
}

static int esp_resume(struct device *dev)
{
	struct sdio_func *func = NULL;
	struct esp_sdio_context *context = NULL;

	if (!dev) {
		esp_info("Failed to inform ESP that host is awake\n");
		return -1;
	}

	func = dev_to_sdio_func(dev);

	esp_info("-----> Host Awake\n");
#if 0
	/* Host woke up.. Disable OOB IRQ */
	disable_irq_wake(SDIO_OOB_IRQ);
	disable_irq(SDIO_OOB_IRQ);
#endif


	context = sdio_get_drvdata(func);

	if (!context) {
		esp_info("Failed to inform ESP that host is awake\n");
		return -1;
	}

	/*     generate_slave_intr(context, BIT(ESP_RESET));*/
	get_firmware_data(context);
	msleep(100);
	generate_slave_intr(context, BIT(ESP_POWER_SAVE_OFF));
	host_sleep = 0;
	return 0;
}

static const struct dev_pm_ops esp_pm_ops = {
	.suspend = esp_suspend,
	.resume = esp_resume,
};

static const struct of_device_id esp_sdio_of_match[] = {
	{ .compatible = "espressif,esp_sdio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp_sdio_of_match);

/* SDIO driver structure to be registered with kernel */
static struct sdio_driver esp_sdio_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= esp_devices,
	.probe		= esp_probe,
	.remove		= esp_remove,
	.drv = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.pm = &esp_pm_ops,
		.of_match_table = esp_sdio_of_match,
	},
};

int esp_init_interface_layer(struct esp_adapter *adapter, u32 speed)
{
	if (!adapter)
		return -EINVAL;

	adapter->if_context = &sdio_context;
	adapter->if_ops = &if_ops;
	sdio_context.adapter = adapter;
	sdio_context.sdio_clk_mhz = speed;

	return sdio_register_driver(&esp_sdio_driver);
}

int esp_validate_chipset(struct esp_adapter *adapter, u8 chipset)
{
	int ret = -1;

	switch(chipset) {
	case ESP_FIRMWARE_CHIP_ESP32:
	case ESP_FIRMWARE_CHIP_ESP32C6:
		adapter->chipset = chipset;
		esp_info("Chipset=%s ID=%02x detected over SDIO\n", esp_chipname_from_id(chipset), chipset);
		ret = 0;
		break;
	case ESP_FIRMWARE_CHIP_ESP32S2:
	case ESP_FIRMWARE_CHIP_ESP32S3:
	case ESP_FIRMWARE_CHIP_ESP32C2:
	case ESP_FIRMWARE_CHIP_ESP32C3:
		esp_err("Chipset=%s ID=%02x not supported for SDIO\n", esp_chipname_from_id(chipset), chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	default:
		esp_err("Unrecognized Chipset ID=%02x\n", chipset);
		adapter->chipset = ESP_FIRMWARE_CHIP_UNRECOGNIZED;
		break;
	}

	return ret;
}

int esp_adjust_spi_clock(struct esp_adapter *adapter, u8 spi_clk_mhz)
{
	/* SPI bus specific call, silently discard */
	return 0;
}

void esp_deinit_interface_layer(void)
{
	sdio_unregister_driver(&esp_sdio_driver);
}
