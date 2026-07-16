/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_types.h — opaque handles and shared type records
 *
 * All types callers interact with at the port boundary.  Keeps the
 * individual API headers free from structural noise.  Opaque handles
 * are forward-declared here; their concrete definitions live inside
 * port-type implementations and are never visible to callers.
 *
 * Payload / record structs that cross the port boundary (GPIO descs,
 * netif up-cfg, HCI send-cfg, event-handler cfg, etc.) are grouped
 * here when they're shared across API groups; single-group records
 * stay inside their group's header.
 */

#ifndef EH_HOST_PORT_TYPES_H_
#define EH_HOST_PORT_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "eh_host_port_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Opaque handles — concrete definitions owned by port-type impl ─ */

typedef struct eh_host_port_task         eh_host_port_task_t;
typedef struct eh_host_port_mutex        eh_host_port_mutex_t;
typedef struct eh_host_port_cond         eh_host_port_cond_t;
typedef struct eh_host_port_sem          eh_host_port_sem_t;
typedef struct eh_host_port_event_group  eh_host_port_event_group_t;
typedef struct eh_host_port_queue        eh_host_port_queue_t;

/* Event-group bit-vector.  uint32_t mirrors FreeRTOS EventBits_t on
 * 32-bit-tick builds and the upstream RainMaker os_event_bits_t. */
typedef uint32_t eh_host_port_event_bits_t;

#if 0
/* ── Shared GPIO descriptor (used by both gpio.h and gpio_events.h) ── */

typedef enum {
    EH_HOST_PORT_GPIO_LEVEL_LOW  = 0,
    EH_HOST_PORT_GPIO_LEVEL_HIGH = 1,
} eh_host_port_gpio_level_t;

typedef enum {
    EH_HOST_PORT_GPIO_DIR_INPUT        = 0,
    EH_HOST_PORT_GPIO_DIR_OUTPUT       = 1,
    EH_HOST_PORT_GPIO_DIR_INPUT_OUTPUT = 2,
} eh_host_port_gpio_dir_t;

typedef enum {
    EH_HOST_PORT_GPIO_PULL_NONE = 0,
    EH_HOST_PORT_GPIO_PULL_UP   = 1,
    EH_HOST_PORT_GPIO_PULL_DOWN = 2,
} eh_host_port_gpio_pull_t;

typedef enum {
    EH_HOST_PORT_GPIO_INTR_NONE      = 0,
    EH_HOST_PORT_GPIO_INTR_POSEDGE   = 1,
    EH_HOST_PORT_GPIO_INTR_NEGEDGE   = 2,
    EH_HOST_PORT_GPIO_INTR_ANYEDGE   = 3,
    EH_HOST_PORT_GPIO_INTR_LOW_LEVEL = 4,
    EH_HOST_PORT_GPIO_INTR_HIGH_LEVEL= 5,
} eh_host_port_gpio_intr_mode_t;

/* Output-drive strength — abstract, implementation picks nearest match.
 * Four-level ladder mirrors ESP GPIO_DRIVE_CAP_{0..3}; platforms with
 * fewer levels (STM32) collapse to their own nearest bucket. */
typedef enum {
    EH_HOST_PORT_GPIO_DRIVE_DEFAULT = 0,  /* platform default, e.g. CAP_2 */
    EH_HOST_PORT_GPIO_DRIVE_WEAK    = 1,
    EH_HOST_PORT_GPIO_DRIVE_MEDIUM  = 2,
    EH_HOST_PORT_GPIO_DRIVE_STRONG  = 3,
} eh_host_port_gpio_drive_t;

/*
 * eh_host_port_gpio_desc_t — lightweight handle for a single pin.
 * `port` is an implementation-specific bank / cluster identifier
 * (e.g. UART0 vs UART1 on a multi-bank MCU).  Implementations that have
 * only one port set `port = 0` and ignore the field.
 */
typedef struct {
    uint32_t port;
    uint32_t pin;
} eh_host_port_gpio_desc_t;
#endif

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_TYPES_H_ */
