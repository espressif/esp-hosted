/*
 * The citation should list that the code comes from the book "Linux Device
 * Drivers" by Alessandro Rubini and Jonathan Corbet, published
 * by O'Reilly & Associates.No warranty is attached;
 *
 * */

#ifndef _ESP_RB_H_
#define _ESP_RB_H_

typedef struct esp_rb {
	wait_queue_head_t wq;		/* waitqueue to wait for data */
	unsigned char *buf, *end;	/* actual queue start and end */
	size_t size;
	unsigned char *rp, *wp;		/* current read/write pointers */
	struct semaphore sem;		/* Mutex to protect rb */
} esp_rb_t;

int esp_rb_init(esp_rb_t *rb, size_t sz);
void esp_rb_cleanup(esp_rb_t *rb);
int esp_rb_read_by_user(esp_rb_t *rb, const char __user *buf, size_t sz, int block);
int esp_rb_write_by_kernel(esp_rb_t *rb, const char *buf, size_t sz);
int get_free_space(esp_rb_t *rb);

#endif
