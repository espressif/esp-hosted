#pragma once

#include "csk_os_common/csk_os_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *csk_os_semaphore_t;

/**
 * @brief Create and initialize a counting semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @param[in] init_count The count value assigned to the semaphore when it is
 *                      created.
 * @param[in] max_count The maximum count value that can be reached. When the
 *                     semaphore reaches this value it can no longer be
 *                     released.
 * @retval int, 0 on success
 */
int csk_os_semaphore_create(csk_os_semaphore_t *sem, uint32_t init_count, uint32_t max_count);

/**
 * @brief Delete the semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @retval int, 0 on success
 */
int csk_os_semaphore_delete(csk_os_semaphore_t sem);

/**
 * @brief Wait until the semaphore object becomes available
 * @param[in] sem Pointer to the semaphore object
 * @param[in] wait_ms The maximum amount of time (in millisecond) the thread
 *                   should remain in the blocked state to wait for the
 *                   semaphore to become available.
 *                   XR_OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval int, 0 on success
 */
int csk_os_semaphore_take(csk_os_semaphore_t sem, uint32_t wait_ms);

/**
 * @brief Release the semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @retval int, 0 on success
 */
int csk_os_semaphore_give(csk_os_semaphore_t sem);

#ifdef __cplusplus
}
#endif
