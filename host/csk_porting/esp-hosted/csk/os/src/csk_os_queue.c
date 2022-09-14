#include <zephyr.h>

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_queue.h"

typedef struct {
	struct k_msgq *msgq;
	char *msgq_buffer;
} z_msgq_t;

int csk_os_queue_create(csk_os_queue_t *queue, uint32_t queue_len, uint32_t item_size)
{
	z_msgq_t *z_queue = csk_os_malloc(sizeof(z_msgq_t));
	if (z_queue == NULL) {
		return -ENOMEM;
	}

	z_queue->msgq = csk_os_malloc(sizeof(struct k_msgq));
	z_queue->msgq_buffer = csk_os_calloc(queue_len, item_size);
	if (z_queue->msgq == NULL || (z_queue->msgq_buffer == NULL)) {
		goto __cleanup;
	}
	k_msgq_init(z_queue->msgq, z_queue->msgq_buffer, item_size, queue_len);
	*queue = z_queue;
	return 0;

__cleanup:
	if (z_queue->msgq) {
		csk_os_free(z_queue->msgq);
		z_queue->msgq = NULL;
	}
	if (z_queue->msgq_buffer) {
		csk_os_free(z_queue->msgq_buffer);
		z_queue->msgq_buffer = NULL;
	}
	if (z_queue) {
		csk_os_free(z_queue);
		z_queue = NULL;
	}
	return -ENOMEM;
}

int csk_os_queue_delete(csk_os_queue_t queue)
{
	z_msgq_t *z_queue = queue;
	k_msgq_cleanup(z_queue->msgq);
	csk_os_free(z_queue->msgq);
	z_queue->msgq = NULL;
	csk_os_free(z_queue->msgq_buffer);
	z_queue->msgq_buffer = NULL;
	csk_os_free(z_queue);
	return 0;
}

int csk_os_queue_send(csk_os_queue_t queue, const void *item, uint32_t wait_ms)
{
	z_msgq_t *z_queue = queue;
	k_timeout_t timeout = (wait_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER :
							(wait_ms == 0) ? K_NO_WAIT : K_MSEC(wait_ms);
	int ret = k_msgq_put(z_queue->msgq, item, timeout);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int csk_os_queue_receive(csk_os_queue_t queue, void *item, uint32_t wait_ms)
{
	z_msgq_t *z_queue = queue;
	k_timeout_t timeout = (wait_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER :
							(wait_ms == 0) ? K_NO_WAIT : K_MSEC(wait_ms);
	int ret = k_msgq_get(z_queue->msgq, item, timeout);
	if (ret != 0) {
		return ret;
	}
	return 0;
}
