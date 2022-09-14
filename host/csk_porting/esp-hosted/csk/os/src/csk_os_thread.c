#include <zephyr.h>
#include <stdlib.h>
#include <stdio.h>
#include <device.h>

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_thread.h"

#define Z_THREAD_STACK_SIZE_ADJUST(size) \
	(ROUND_UP((size), ARCH_STACK_PTR_ALIGN) + K_THREAD_STACK_RESERVED)

typedef struct {
	struct k_thread thread;
	k_thread_stack_t *stack;
} z_thread_t;

typedef struct thread_handle_delete_item {
    sys_snode_t node;
    z_thread_t *delete_handle;
} thread_handle_delete_item_t;

static sys_slist_t s_thread_delete_list;
static struct k_work_delayable s_thread_delete_work;

K_MUTEX_DEFINE(thread_delete_mutex);

static int thread_delete_list_free(void)
{
    thread_handle_delete_item_t *entry, *next;
	k_mutex_lock(&thread_delete_mutex, K_FOREVER);
    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&s_thread_delete_list, entry, next, node) {
		z_thread_t *thread_handle = entry->delete_handle;
		csk_os_free(thread_handle->stack);
		thread_handle->stack = NULL;
		csk_os_free(thread_handle);
		thread_handle = NULL;
		sys_slist_find_and_remove(&s_thread_delete_list, &entry->node);
		csk_os_free(entry);
		entry = NULL;
    }
	k_mutex_unlock(&thread_delete_mutex);
    return 0;
}

void csk_os_thread_yield(void)
{
	k_yield();
}

void csk_os_thread_sleep(uint32_t msec)
{
	k_msleep(msec);
}

csk_os_thread_t csk_os_thread_get_current(void)
{
	k_tid_t current_tid = k_current_get();
	z_thread_t *z_thread = CONTAINER_OF(current_tid, z_thread_t, thread);
	return (csk_os_thread_t)z_thread;
}

static void zep_thread_entry(void *real_entry, void *real_arg, void *arg3)
{
    csk_thread_entry_t thread_handler = real_entry;
    thread_handler(real_arg);
}

int csk_os_thread_create(csk_os_thread_t *thread, const char *name,
                          csk_thread_entry_t entry, void *arg,
                          int priority, uint32_t stackSize)
{
	z_thread_t *z_thread = csk_os_malloc(sizeof(z_thread_t));
	if (z_thread == NULL) {
		return -ENOMEM;
	}

	int ret = 0;
	z_thread->stack = csk_os_malloc(Z_THREAD_STACK_SIZE_ADJUST(stackSize));
	if (z_thread->stack == NULL) {
		ret = -ENOMEM;
		goto __cleanup;
	}

	k_tid_t tid = k_thread_create(&z_thread->thread, z_thread->stack,
									Z_THREAD_STACK_SIZE_ADJUST(stackSize),
                    				&zep_thread_entry, (void *)entry, arg, NULL,
									priority, 0, K_NO_WAIT);
	__ASSERT(tid == &z_thread->thread, "tid not valid");
	(void) tid;
	k_thread_name_set(&z_thread->thread, name);
	*thread = z_thread;
	return 0;

__cleanup:
	if (z_thread->stack) {
		csk_os_free(z_thread->stack);
		z_thread->stack = NULL;
	}
	if (z_thread) {
		csk_os_free(z_thread);
	}
	return ret;
}

int csk_os_thread_delete(csk_os_thread_t thread)
{
	bool is_delete_self;
	z_thread_t *z_thread = NULL;

	k_sched_lock();
	k_tid_t current_tid = k_current_get();
    if (thread == NULL) {
		z_thread = CONTAINER_OF(current_tid, z_thread_t, thread);
		is_delete_self = true;
    } else {
		z_thread = thread;
		if (k_current_get() == (&z_thread->thread)) {
			is_delete_self = true;
		} else {
			is_delete_self = false;
		}
	}

	if(is_delete_self) {
		k_work_cancel_delayable(&s_thread_delete_work);
		k_work_reschedule(&s_thread_delete_work, K_MSEC(50));
		k_sched_unlock();
		k_thread_abort(&z_thread->thread);
		return 0;
	}
	k_sched_unlock();
	int ret = 0;
	k_thread_abort(&z_thread->thread);
	if (true == k_is_in_isr()) {
		ret = k_thread_join(&z_thread->thread, K_NO_WAIT);
	} else {
		ret = k_thread_join(&z_thread->thread, K_FOREVER);
	}
	if (0 != ret) {
		return ret;
	}
	csk_os_free(z_thread->stack);
	z_thread->stack = NULL;
	csk_os_free(z_thread);
	return 0;
}

static void thread_delete_handler(struct k_work *work)
{
	thread_delete_list_free();
}

static int os_thread_init(const struct device *unused)
{
	ARG_UNUSED(unused);
	sys_slist_init(&s_thread_delete_list);
	k_work_init_delayable(&s_thread_delete_work, thread_delete_handler);
	return 0;
}

SYS_INIT(os_thread_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
