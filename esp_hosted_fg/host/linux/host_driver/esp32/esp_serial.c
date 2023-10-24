// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "esp.h"
#include "esp_rb.h"
#include "esp_api.h"

#define ESP_SERIAL_MAJOR      221
#define ESP_SERIAL_MINOR_MAX  2
#define ESP_RX_RB_SIZE        4096
#define ESP_SERIAL_MAX_TX     4096

//#define ESP_SERIAL_TEST

static struct esp_serial_devs {
	struct cdev cdev;
	int dev_index;
	esp_rb_t rb;
	void *priv;
	struct mutex lock;
} devs[ESP_SERIAL_MINOR_MAX];

static uint8_t serial_init_done;

static ssize_t esp_serial_read(struct file *file, char __user *user_buffer, size_t size, loff_t *offset)
{
	struct esp_serial_devs *dev = NULL;
	int ret_size = 0;
	dev = (struct esp_serial_devs *) file->private_data;
	ret_size = esp_rb_read_by_user(&dev->rb, user_buffer, size, !(file->f_flags & O_NONBLOCK));
	if (ret_size == 0) {
		return -EAGAIN;
	}
	return ret_size;
}

static ssize_t esp_serial_write(struct file *file, const char __user *user_buffer, size_t size, loff_t * offset)
{
	struct esp_payload_header *hdr = NULL;
	u8 *tx_buf = NULL;
	struct esp_serial_devs *dev = NULL;
	struct sk_buff * tx_skb = NULL;
	int ret = 0;
	size_t total_len = 0;
	size_t frag_len = 0;
	u32 left_len = size;
	static u16 seq_num = 0;
	u8 flag = 0;
	u8 *pos;

	if (size > ESP_SERIAL_MAX_TX) {
		printk(KERN_ERR "%s: Exceed max tx buffer size [%zu]\n", __func__, size);
		return 0;
	}

	seq_num++;
	dev = (struct esp_serial_devs *) file->private_data;
	pos = (u8 *) user_buffer;

	do {
		/* Fragmentation support
		 *  - Fragment large packets into multiple 1500 byte packets
		 *  - MORE_FRAGMENT bit in flag tells if there are more fragments expected
		 **/
		if (left_len > ETH_DATA_LEN) {
			frag_len = ETH_DATA_LEN;
			flag = MORE_FRAGMENT;
		} else {
			frag_len = left_len;
			flag = 0;
		}

		total_len = frag_len + sizeof(struct esp_payload_header);

		tx_skb = esp_alloc_skb(total_len);
		if (!tx_skb) {
			printk (KERN_ERR "%s: SKB alloc failed\n", __func__);
			return (size - left_len);
		}

		tx_buf = skb_put(tx_skb, total_len);

		hdr = (struct esp_payload_header *) tx_buf;

		memset (hdr, 0, sizeof(struct esp_payload_header));

		hdr->if_type = ESP_SERIAL_IF;
		hdr->if_num = dev->dev_index;
		hdr->len = cpu_to_le16(frag_len);
		hdr->seq_num = cpu_to_le16(seq_num);
		hdr->offset = cpu_to_le16(sizeof(struct esp_payload_header));
		hdr->flags |= flag;

		ret = copy_from_user(tx_buf + hdr->offset, pos, frag_len);
		if (ret) {
			dev_kfree_skb(tx_skb);
			printk(KERN_ERR "%s, Error copying buffer to send serial data\n", __func__);
			return (size - left_len);
		}
		hdr->checksum = cpu_to_le16(compute_checksum(tx_skb->data, (frag_len + sizeof(struct esp_payload_header))));

		/* print_hex_dump(KERN_INFO, "esp_serial_tx: ", DUMP_PREFIX_ADDRESS, 16, 1, pos, frag_len, 1 ); */

		ret = esp_send_packet(dev->priv, tx_skb);
		if (ret) {
			printk (KERN_ERR "%s: Failed to transmit data, error %d\n", __func__, ret);
			return (size - left_len);
		}

		left_len -= frag_len;
		pos += frag_len;
	} while(left_len);

	return size;
}

static long esp_serial_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "%s IOCTL %d\n", __func__, cmd);
	return 0;
}

static int esp_serial_open(struct inode *inode, struct file *file)
{
	struct esp_serial_devs *devs = NULL;

	devs = container_of(inode->i_cdev, struct esp_serial_devs, cdev);
	file->private_data = devs;

	return 0;
}

static unsigned int esp_serial_poll(struct file *file, poll_table *wait)
{
    struct esp_serial_devs *dev = (struct esp_serial_devs *)file->private_data;
    unsigned int mask = 0;

    mutex_lock(&dev->lock);
    poll_wait(file, &dev->rb.wq,  wait);

    if (dev->rb.rp != dev->rb.wp) {
        mask |= (POLLIN | POLLRDNORM) ;   /* readable */
    }
    if (get_free_space(&dev->rb)) {
        mask |= (POLLOUT | POLLWRNORM) ;  /* writable */
    }

    mutex_unlock(&dev->lock);
    return mask;
}

const struct file_operations esp_serial_fops = {
	.owner = THIS_MODULE,
	.open = esp_serial_open,
	.read = esp_serial_read,
	.write = esp_serial_write,
	.unlocked_ioctl = esp_serial_ioctl,
	.poll = esp_serial_poll
};

int esp_serial_data_received(int dev_index, const char *data, size_t len)
{
	int ret = 0, ret_len = 0;
	if (dev_index >= ESP_SERIAL_MINOR_MAX) {
		return -EINVAL;
	}

	while (ret_len != len) {
		ret = esp_rb_write_by_kernel(&devs[dev_index].rb,
				data+ret_len, (len-ret_len));
		if (ret <= 0) {
			break;
		}
		ret_len += ret;
	}
	if (ret <= 0) {
		return ret;
	}
	if (ret_len != len) {
		printk(KERN_ERR "%s, RB full, no space to receive. Dropping packet",__func__);
	}

	return ret_len;
}

#ifdef ESP_SERIAL_TEST
static int thread_fn(void *unused)
{
	int i = 100;

	while(i--) {
		esp_rb_write_by_kernel(&devs[0].rb, "alphabetagamma", 14);
		ssleep(1);
	}
	printk(KERN_INFO "%s, Thread stopping\n", __func__);
	do_exit(0);
	return 0;
}
#endif

int esp_serial_init(void *priv)
{
	int err = 0, i = 0;

	if (!priv) {
		printk(KERN_ERR "esp: %s: failed. NULL adapter\n", __func__);
		return -1;
	}

	err = register_chrdev_region(MKDEV(ESP_SERIAL_MAJOR, 0), ESP_SERIAL_MINOR_MAX, "esp_serial_driver");
	if (err) {
		printk(KERN_ERR "%s, Error registering chrdev region %d\n", __func__, err);
		return -1;
	}

	for (i = 0; i < ESP_SERIAL_MINOR_MAX; i++) {
		cdev_init(&devs[i].cdev, &esp_serial_fops);
		devs[i].dev_index = i;
		cdev_add(&devs[i].cdev, MKDEV(ESP_SERIAL_MAJOR, i), 1);
		esp_rb_init(&devs[i].rb, ESP_RX_RB_SIZE);
		devs[i].priv = priv;
		mutex_init(&devs[i].lock);
	}

#ifdef ESP_SERIAL_TEST
	kthread_run(thread_fn, NULL, "esptest-thread");
#endif
	serial_init_done = 1;
	return 0;
}

void esp_serial_cleanup(void)
{
	int i = 0;

	for (i = 0; serial_init_done && i < ESP_SERIAL_MINOR_MAX; i++) {
		if (!devs[i].cdev.ops)
			cdev_del(&devs[i].cdev);

		esp_rb_cleanup(&devs[i].rb);
		mutex_destroy(&devs[i].lock);
	}

	unregister_chrdev_region(MKDEV(ESP_SERIAL_MAJOR, 0), ESP_SERIAL_MINOR_MAX);

	serial_init_done = 0;
	return;
}

int esp_serial_reinit(void *priv)
{
	if (serial_init_done)
		esp_serial_cleanup();

	return esp_serial_init(priv);
}
