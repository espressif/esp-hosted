#pragma once

#include "csk_os_common/csk_os_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Thread entry definition, which is a pointer to a function */
typedef void (*csk_thread_entry_t)(void *);

/** @brief Thread handle definition */
typedef void *csk_os_thread_t;

/**
 * @brief Sleep for the given milliseconds
 *
 * This function causes the calling thread to sleep and block for the given
 * milliseconds.
 *
 * @param[in] msec Milliseconds to sleep
 * @return None
 */
void csk_os_thread_sleep(uint32_t msec);

/**
 * @brief Yield to another thread of equal priority
 *
 * Yielding is where a thread volunteers to leave the running state, without
 * being pre-empted, and before its time slice has been fully utilized.
 *
 * @return None
 */
void csk_os_thread_yield(void);

/**
 * @brief Get the handle of the current running thread
 * @return Handle of the current running thread
 */
csk_os_thread_t csk_os_thread_get_current(void);

/**
 * @brief Create and start a thread
 *
 * This function starts a new thread. The new thread starts execution by
 * invoking entry(). The argument arg is passed as the sole argument of entry().
 *
 * @note After finishing execution, the new thread should call csk_os_thread_delete()
 *       to delete itself. Failing to do this and just returning from entry()
 *       will result in undefined behavior.
 *
 * @param[in] thread Pointer to the thread object
 * @param[in] name A descriptive name for the thread. This is mainly used to
 *                 facilitate debugging.
 * @param[in] entry Entry, which is a function pointer, to the thread function
 * @param[in] arg The sole argument passed to entry()
 * @param[in] priority The priority at which the thread will execute
 * @param[in] stackSize The number of bytes the thread stack can hold
 * @retval int, XR_OS_OK on success
 */
int csk_os_thread_create(csk_os_thread_t *thread, const char *name,
                                    csk_thread_entry_t entry, void *arg,
                                    int priority, uint32_t stackSize);

/**
 * @brief Terminate the thread
 * @note Only memory that is allocated to a thread by the kernel itself is
 *       automatically freed when a thread is deleted. Memory, or any other
 *       resource, that the application (rather than the kernel) allocates
 *       to a thread must be explicitly freed by the application when the task
 *       is deleted.
 * @param[in] thread Pointer to the thread object to be deleted.
 *                   A thread can delete itself by passing NULL in place of a
 *                   valid thread object.
 * @retval int, XR_OS_OK on success
 */
int csk_os_thread_delete(csk_os_thread_t thread);

#ifdef __cplusplus
}
#endif
