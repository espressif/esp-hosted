/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_sync.h — mutex, condition variable, binary semaphore
 *
 * Group header — all synchronisation primitives live together so the
 * compiler-visible surface can be toggled per implementation at one
 * site.  Each primitive is individually gated; a port-type may enable
 * mutex+cond while omitting semaphores, or vice versa.
 *
 * Timeout contract for `_wait_ms` APIs:
 *   timeout_ms == 0                         → poll (non-blocking)
 *   timeout_ms == EH_HOST_PORT_WAIT_FOREVER → block until signalled
 *   0 < timeout_ms < FOREVER                → EH_HOST_PORT_ERR_TIMEOUT on deadline
 *   signal received in time                 → EH_HOST_PORT_OK
 *
 * ISR-safety:
 *   Only `eh_host_port_sem_post_from_isr` is ISR-safe, and only when
 *   EH_HOST_PORT_HAS_SEM_POST_FROM_ISR is 1.  All other APIs in this file
 *   are task-context only.  Calling a non-ISR API from an ISR is
 *   undefined behaviour.
 */

#ifndef EH_HOST_PORT_SYNC_H_
#define EH_HOST_PORT_SYNC_H_

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Block-forever sentinel for `_wait_ms` / queue / event-group APIs.
 * Defined unconditionally so any TU including this header sees it
 * even when its primitive group is gated off. */
#ifndef EH_HOST_PORT_WAIT_FOREVER
#  define EH_HOST_PORT_WAIT_FOREVER  ((uint32_t)0xFFFFFFFFu)
#endif

/* ── Mutex ─────────────────────────────────────────────────────── */
#if EH_HOST_PORT_HAS_MUTEX

eh_host_port_mutex_t *eh_host_port_mutex_create(void);
void             eh_host_port_mutex_destroy(eh_host_port_mutex_t *m);
void             eh_host_port_mutex_lock(eh_host_port_mutex_t *m);
void             eh_host_port_mutex_unlock(eh_host_port_mutex_t *m);

/* Bounded lock acquire — same `_wait_ms` semantics as the rest of the file.
 * EH_HOST_PORT_OK on lock held; EH_HOST_PORT_ERR_TIMEOUT on deadline miss
 * (caller does not hold the lock); EH_HOST_PORT_ERR_INVAL on NULL. */
eh_host_port_err_t   eh_host_port_mutex_lock_wait_ms(eh_host_port_mutex_t *m,
                                                     uint32_t timeout_ms);

#endif /* EH_HOST_PORT_HAS_MUTEX */

/* ── Condition variable ────────────────────────────────────────── */
#if EH_HOST_PORT_HAS_COND

eh_host_port_cond_t *eh_host_port_cond_create(void);
void            eh_host_port_cond_destroy(eh_host_port_cond_t *c);

/* Atomic release-of-mutex + wait-on-signal; re-acquires the mutex
 * before returning.  Caller MUST hold `m` on entry. */
void            eh_host_port_cond_wait(eh_host_port_cond_t *c, eh_host_port_mutex_t *m);

/* As above but bounded; returns EH_HOST_PORT_ERR_TIMEOUT on deadline miss. */
eh_host_port_err_t   eh_host_port_cond_wait_ms(eh_host_port_cond_t *c, eh_host_port_mutex_t *m,
                                      uint32_t timeout_ms);

void            eh_host_port_cond_signal(eh_host_port_cond_t *c);
void            eh_host_port_cond_broadcast(eh_host_port_cond_t *c);

#endif /* EH_HOST_PORT_HAS_COND */

/* ── Binary semaphore ──────────────────────────────────────────── */
#if EH_HOST_PORT_HAS_SEM

eh_host_port_sem_t  *eh_host_port_sem_create(void);

eh_host_port_sem_t  *eh_host_port_sem_create_counting(uint32_t max_count);

void            eh_host_port_sem_destroy(eh_host_port_sem_t *s);

eh_host_port_err_t   eh_host_port_sem_post(eh_host_port_sem_t *s);
eh_host_port_err_t   eh_host_port_sem_wait_ms(eh_host_port_sem_t *s, uint32_t timeout_ms);

/* Non-blocking take.  Returns EH_HOST_PORT_OK if a post was consumed,
 * EH_HOST_PORT_ERR_TIMEOUT if the sem was not posted, EH_HOST_PORT_ERR_INVAL
 * on null.  Useful for drain-check loops ("did another post arrive
 * while I was processing?") without the overhead of a cond wait. */
eh_host_port_err_t   eh_host_port_sem_try_wait(eh_host_port_sem_t *s);

#if EH_HOST_PORT_HAS_SEM_POST_FROM_ISR
/* ISR-safe post — sole ISR entry point in the port layer today.
 * Implementations must ensure the call is interrupt-reentrant and
 * yields to a higher-priority waiter where the target scheduler
 * supports it (e.g. portYIELD_FROM_ISR on FreeRTOS). */
eh_host_port_err_t   eh_host_port_sem_post_from_isr(eh_host_port_sem_t *s);
#endif

#endif /* EH_HOST_PORT_HAS_SEM */

/* ── Event group: bit-vector with multi-waiter wait_bits ──
 * FreeRTOS-shaped timeout: 0=poll, EH_HOST_PORT_WAIT_FOREVER=block,
 * >0=bounded. Returns observed bits; clear_on_exit clears atomically. */
#if EH_HOST_PORT_HAS_EVENT_GROUP

eh_host_port_event_group_t *eh_host_port_event_group_create(void);
void                   eh_host_port_event_group_delete(eh_host_port_event_group_t *g);

eh_host_port_event_bits_t   eh_host_port_event_group_set_bits(
                            eh_host_port_event_group_t *g,
                            eh_host_port_event_bits_t   bits_to_set);

eh_host_port_event_bits_t   eh_host_port_event_group_clear_bits(
                            eh_host_port_event_group_t *g,
                            eh_host_port_event_bits_t   bits_to_clear);

eh_host_port_event_bits_t   eh_host_port_event_group_wait_bits(
                            eh_host_port_event_group_t *g,
                            eh_host_port_event_bits_t   bits_to_wait_for,
                            int                    clear_on_exit,
                            int                    wait_for_all_bits,
                            uint32_t               timeout_ms);

#endif /* EH_HOST_PORT_HAS_EVENT_GROUP */

/* Queue: bounded blocking FIFO of fixed-size items.
 * Items are byte-copied; send blocks if full, receive blocks if empty.
 * timeout_ms: 0=non-blocking, EH_HOST_PORT_WAIT_FOREVER=block, >0=bounded.
 *   - posix:   mutex + cond + ring buffer, per-item malloc.
 *   - esp_idf: thin wrap around xQueueCreate/Send/Receive.
 *
 * Lets rpc_ll skip hand-rolled queue management; just call send/receive with the appropriate
 * timeout and lets each port hit its native primitive.
 * ------------------------------------------------------------------------- */
#if EH_HOST_PORT_HAS_QUEUE

eh_host_port_queue_t *eh_host_port_queue_create(uint32_t depth, size_t item_size);
void             eh_host_port_queue_destroy(eh_host_port_queue_t *q);

/* send: copies `item_size` bytes from `item` into the queue.
 *   EH_HOST_PORT_OK            on success
 *   EH_HOST_PORT_ERR_TIMEOUT   on deadline miss (or queue full when timeout_ms==0)
 *   EH_HOST_PORT_ERR_INVAL     on NULL queue / NULL item
 *   EH_HOST_PORT_ERR_NOMEM     on internal alloc failure (posix per-item) */
eh_host_port_err_t    eh_host_port_queue_send(eh_host_port_queue_t *q,
                                    const void *item,
                                    uint32_t timeout_ms);

/* receive: copies `item_size` bytes from the queue into `out_item`.
 *   Same return contract as send (TIMEOUT on empty + deadline miss). */
eh_host_port_err_t    eh_host_port_queue_receive(eh_host_port_queue_t *q,
                                       void *out_item,
                                       uint32_t timeout_ms);

/* peek: copies `item_size` bytes from the queue head into `out_item` without
 * removing the item. Same timeout and return contract as receive. */
eh_host_port_err_t    eh_host_port_queue_peek(eh_host_port_queue_t *q,
                                    void *out_item,
                                    uint32_t timeout_ms);

/* Number of items currently queued.  Diagnostic — not for control flow.
 * (Use send/receive's TIMEOUT return for coordination.) */
uint32_t         eh_host_port_queue_count(const eh_host_port_queue_t *q);

#endif /* EH_HOST_PORT_HAS_QUEUE */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_SYNC_H_ */
