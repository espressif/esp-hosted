/*
 * The citation should list that the code comes from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.No warranty is attached;
 *
 * */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include "esp_rb.h"

int esp_rb_init(esp_rb_t *rb, size_t sz)
{
	init_waitqueue_head(&(rb->wq));

	rb->buf = kmalloc(sz, GFP_KERNEL);
	if (!rb->buf) {
		printk(KERN_ERR "%s, Failed to allocate memory for rb\n", __func__);
		return -ENOMEM;
	}

	rb->end = rb->buf + sz;
	rb->rp = rb->wp = rb->buf;
	rb->size = sz;

	sema_init(&(rb->sem), 1);
	return 0;
}

ssize_t esp_rb_read_by_user(esp_rb_t *rb, const char __user *buf, size_t sz, int block)
{
	size_t read_len = 0, temp_len = 0;

	if (down_interruptible(&rb->sem)) {
		return -ERESTARTSYS; /* Signal interruption */
	}

	while (rb->rp == rb->wp) {
		up(&rb->sem);
		if (block == 0) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(rb->wq, (rb->rp != rb->wp))) {
			return -ERESTARTSYS; /* Signal interruption */
		}
		if (down_interruptible(&rb->sem)) {
			return -ERESTARTSYS;
		}
	}

	if (rb->wp > rb->rp) {
		read_len = min(sz, (size_t)(rb->wp - rb->rp));
	} else {
		read_len = min(sz, (size_t)(rb->end - rb->rp));
	}

	if (copy_to_user((void *)buf, rb->rp, read_len)) {
		up(&rb->sem);
		printk(KERN_WARNING "%s, Incomplete/Failed read\n", __func__);
		return -EFAULT;
	}

	rb->rp += read_len;
	if (rb->rp == rb->end) {
		rb->rp = rb->buf;
	}

	if (read_len < sz) {
		temp_len = min(sz-read_len,(size_t)(rb->wp - rb->rp));
		if (copy_to_user((void *)buf+read_len, rb->rp, temp_len)) {
			up(&rb->sem);
			printk(KERN_WARNING "%s, Incomplete/Failed read\n", __func__);
			return -EFAULT;
		}
	}

	rb->rp += temp_len;
	read_len += temp_len;

	up(&rb->sem);

	return read_len;
}

size_t get_free_space(esp_rb_t *rb)
{
	if (rb->rp == rb->wp) {
		return rb->size - 1;
	} else {
		return ((rb->rp + rb->size - rb->wp) % rb->size) - 1;
	}
}

ssize_t esp_rb_write_by_kernel(esp_rb_t *rb, const char *buf, size_t sz)
{
	size_t write_len = 0, temp_len = 0;

	if (down_interruptible(&rb->sem)) {
		return -ERESTARTSYS;
	}

	if (get_free_space(rb) == 0) {
		up(&rb->sem);
		printk(KERN_ERR "%s, Ringbuffer full, no space to write\n", __func__);
		return 0;
	}

	sz = min(sz, get_free_space(rb));
	if (rb->wp >= rb->rp) {
		write_len = min(sz, (size_t)(rb->end - rb->wp));
	} else {
		write_len = min(sz, (size_t)(rb->rp - rb->wp - 1));
	}

	memcpy(rb->wp, buf, write_len);
	rb->wp += write_len;
	if (rb->wp == rb->end) {
		rb->wp = rb->buf;
	}

	if (write_len < sz) {
		temp_len = (size_t)min(sz-write_len,(size_t)(rb->rp - rb->wp - 1));
		memcpy(rb->wp, buf+write_len, temp_len);
	}

	rb->wp += temp_len;
	write_len += temp_len;

	up(&rb->sem);

	wake_up_interruptible(&rb->wq);

	return write_len;
}

void esp_rb_cleanup(esp_rb_t *rb)
{
	kfree(rb->buf);
	rb->buf = rb->end = rb->rp = rb->wp = NULL;
	rb->size = 0;
	return;
}
