#pragma once

#include "csk_os_common/csk_os_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mutex object definition
 */
typedef void *csk_os_mutex_t;

/**
 * @brief Create and initialize a mutex object
 * @note A mutex can only be locked by a single thread at any given time.
 * @param[in] mutex Pointer to the mutex object
 * @retval int, 0 on success
 */
int csk_os_mutex_create(csk_os_mutex_t *mutex);

/**
 * @brief Delete the mutex object
 * @param[in] mutex Pointer to the mutex object
 * @retval int, 0 on success
 */
int csk_os_mutex_delete(csk_os_mutex_t mutex);

/**
 * @brief Lock the mutex object
 * @note A mutex can only be locked by a single thread at any given time. If
 *       the mutex is already locked, the caller will be blocked for the
 *       specified time duration.
 * @param[in] mutex Pointer to the mutex object
 * @param[in] waitMS The maximum amount of time (in millisecond) the thread
 *                   should remain in the blocked state to wait for the mutex
 *                   to become unlocked.
 *                   XR_OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval int, 0 on success
 */
int csk_os_mutex_lock(csk_os_mutex_t mutex, uint32_t wait_ms);

/**
 * @brief Unlock the mutex object previously locked using XR_OS_MutexLock()
 * @note The mutex should be unlocked from the same thread context from which
 *       it was locked.
 * @param[in] mutex Pointer to the mutex object
 * @retval int, 0 on success
 */
int csk_os_mutex_unlock(csk_os_mutex_t mutex);

#ifdef __cplusplus
}
#endif
