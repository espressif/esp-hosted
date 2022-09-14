#include <zephyr.h>

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_semaphore.h"

typedef struct {
	struct k_sem *sem;
} z_sem_t;

int csk_os_semaphore_create(csk_os_semaphore_t *sem, uint32_t init_count, uint32_t max_count)
{
	int ret = 0;
	z_sem_t *z_sem = csk_os_malloc(sizeof(z_sem_t));
	if (z_sem == NULL) {
		return -ENOMEM;
	}

    z_sem->sem = csk_os_malloc(sizeof(struct k_sem));
	if (z_sem->sem == NULL) {
		ret = -ENOMEM;
		goto __cleanup;
	}

	ret = k_sem_init(z_sem->sem, init_count, max_count);
    if (ret != 0) {
		goto __cleanup;
    }
	*sem = z_sem;
	return 0;

__cleanup:
	if (z_sem->sem) {
		csk_os_free(z_sem->sem);
		z_sem->sem = NULL;
	}
	if (z_sem) {
		csk_os_free(z_sem);
		z_sem = NULL;
	}
	return ret;
}

int csk_os_semaphore_delete(csk_os_semaphore_t sem)
{
	z_sem_t *z_sem = sem;
	csk_os_free(z_sem->sem);
	z_sem->sem = NULL;
	csk_os_free(z_sem);
	z_sem = NULL;
	return 0;
}

int csk_os_semaphore_take(csk_os_semaphore_t sem, uint32_t wait_ms)
{
	z_sem_t *z_sem = sem;
	k_timeout_t timeout = (wait_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER :
							(wait_ms == 0) ? K_NO_WAIT : K_MSEC(wait_ms);
	int ret = k_sem_take(z_sem->sem, timeout);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

int csk_os_semaphore_give(csk_os_semaphore_t sem)
{
	z_sem_t *z_sem = sem;
	k_sem_give(z_sem->sem);
	return 0;
}

