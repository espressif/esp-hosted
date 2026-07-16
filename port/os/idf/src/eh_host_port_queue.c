/* SPDX-License-Identifier: Apache-2.0 */
/* IDF FreeRTOS xQueue thin wrap. timeout_ms: 0=non-blocking, WAIT_FOREVER=portMAX_DELAY. */

#include "eh_host_port_config.h"

#if EH_HOST_PORT_HAS_QUEUE

#include "eh_host_port_err.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_master_config.h"

#ifdef ESP_PLATFORM

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdlib.h>

struct eh_host_port_queue { QueueHandle_t h; };

eh_host_port_queue_t *eh_host_port_queue_create(uint32_t depth, size_t item_size)
{
    if (depth == 0 || item_size == 0) {
        return NULL;
    }
    eh_host_port_queue_t *q = calloc(1, sizeof(*q));
    if (!q) {
        return NULL;
    }
    q->h = xQueueCreate(depth, item_size);
    if (!q->h) {
        free(q);
        return NULL;
    }
    return q;
}

void eh_host_port_queue_destroy(eh_host_port_queue_t *q)
{
    if (!q) {
        return;
    }
    if (q->h) {
        vQueueDelete(q->h);
    }
    free(q);
}

static TickType_t ticks_from_ms(uint32_t timeout_ms)
{
    if (timeout_ms == EH_HOST_PORT_WAIT_FOREVER) {
        return portMAX_DELAY;
    }
    if (timeout_ms == 0) {
        return 0;
    }
    return pdMS_TO_TICKS(timeout_ms);
}

eh_host_port_err_t eh_host_port_queue_send(eh_host_port_queue_t *q,
                                 const void *item,
                                 uint32_t timeout_ms)
{
    if (!q || !item) {
        return EH_HOST_PORT_ERR_INVAL;
    }
    BaseType_t r = xQueueSend(q->h, item, ticks_from_ms(timeout_ms));
    return (r == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

eh_host_port_err_t eh_host_port_queue_receive(eh_host_port_queue_t *q,
                                    void *out_item,
                                    uint32_t timeout_ms)
{
    if (!q || !out_item) {
        return EH_HOST_PORT_ERR_INVAL;
    }
    BaseType_t r = xQueueReceive(q->h, out_item, ticks_from_ms(timeout_ms));
    return (r == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

eh_host_port_err_t eh_host_port_queue_peek(eh_host_port_queue_t *q,
                                 void *out_item,
                                 uint32_t timeout_ms)
{
    if (!q || !out_item) {
        return EH_HOST_PORT_ERR_INVAL;
    }
    BaseType_t r = xQueuePeek(q->h, out_item, ticks_from_ms(timeout_ms));
    return (r == pdTRUE) ? EH_HOST_PORT_OK : EH_HOST_PORT_ERR_TIMEOUT;
}

uint32_t eh_host_port_queue_count(const eh_host_port_queue_t *q)
{
    if (!q || !q->h) {
        return 0;
    }
    return (uint32_t)uxQueueMessagesWaiting(q->h);
}

#endif /* ESP_PLATFORM */
#endif /* EH_HOST_PORT_HAS_QUEUE */
