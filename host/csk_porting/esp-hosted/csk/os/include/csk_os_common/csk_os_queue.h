#pragma once

#include "csk_os_common/csk_os_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Queue object definition
 */

typedef void *csk_os_queue_t;

/**
 * @brief Create and initialize a queue object
 * @param[in] queue Pointer to the queue object
 * @param[in] queue_len The maximum number of items that the queue can hold at
 *                     any one time.
 * @param[in] item_size The size, in bytes, of each data item that can be stored
 *                     in the queue.
 * @retval OS_Status, OS_OK on success
 */

int csk_os_queue_create(csk_os_queue_t *queue, uint32_t queue_len, uint32_t item_size);

/**
 * @brief Delete the queue object
 * @param[in] queue Pointer to the queue object
 * @retval int, 0 on success
 */
int csk_os_queue_delete(csk_os_queue_t queue);

/**
 * @brief Send (write) an item to the back of the queue
 * @param[in] queue Pointer to the queue object
 * @param[in] item Pointer to the data to be copied into the queue.
 *                 The size of each item the queue can hold is set when the
 *                 queue is created, and that many bytes will be copied from
 *                 item into the queue storage area.
 * @param[in] wait_ms The maximum amount of time the thread should remain in the
 *                   blocked state to wait for space to become available on the
 *                   queue, should the queue already be full.
 *                   XR_OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval int, 0 on success
 */
int csk_os_queue_send(csk_os_queue_t queue, const void *item, uint32_t wait_ms);

/**
 * @brief Receive (read) an item from the queue
 * @param[in] queue Pointer to the queue object
 * @param[in] item Pointer to the memory into which the received data will be
 *                 copied. The length of the buffer must be at least equal to
 *                 the queue item size which is set when the queue is created.
 * @param[in] wait_ms The maximum amount of time the thread should remain in the
 *                   blocked state to wait for data to become available on the
 *                   queue, should the queue already be empty.
 *                   XR_OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval int, 0 on success
 */
int csk_os_queue_receive(csk_os_queue_t queue, void *item, uint32_t wait_ms);

#ifdef __cplusplus
}
#endif
