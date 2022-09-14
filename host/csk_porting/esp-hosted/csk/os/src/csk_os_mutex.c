#include <zephyr.h>

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_mutex.h"

typedef struct {
	struct k_mutex *mutex;	
} z_mutex_t;

int csk_os_mutex_create(csk_os_mutex_t *mutex)
{
	int ret = 0;
	z_mutex_t *z_mutex = csk_os_malloc(sizeof(z_mutex_t));
	if (z_mutex == NULL) {
		return -ENOMEM;
	}

	z_mutex->mutex = csk_os_malloc(sizeof(struct k_mutex));
	if (z_mutex->mutex == NULL) {
		ret = -ENOMEM;
		goto __cleanup;
	}

	ret = k_mutex_init(z_mutex->mutex);
	if (ret != 0) {
		goto __cleanup;
	}

	*mutex = z_mutex;
	return 0;

__cleanup:
	if (z_mutex->mutex) {
		csk_os_free(z_mutex->mutex);
		z_mutex->mutex = NULL;
	}
	if (z_mutex) {
		csk_os_free(z_mutex);
		z_mutex = NULL;
	}
	return ret;
}

int csk_os_mutex_delete(csk_os_mutex_t mutex)
{
	z_mutex_t *z_mutex = mutex;
	csk_os_free(z_mutex->mutex);
	z_mutex->mutex = NULL;
	csk_os_free(z_mutex);
	z_mutex = NULL;
	return 0;
}

int csk_os_mutex_lock(csk_os_mutex_t mutex, uint32_t wait_ms)
{
	z_mutex_t *z_mutex = mutex;
    k_timeout_t timeout = (wait_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER :
							(wait_ms == 0) ? K_NO_WAIT : K_MSEC(wait_ms);
	int ret = k_mutex_lock(z_mutex->mutex, timeout);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int csk_os_mutex_unlock(csk_os_mutex_t mutex)
{
	z_mutex_t *z_mutex = mutex;
	int ret = k_mutex_unlock(z_mutex->mutex);
	if (ret != 0) {
		return ret;
	}
	return 0;
}
