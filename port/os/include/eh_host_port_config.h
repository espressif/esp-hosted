/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_HOST_PORT_CONFIG_H_
#define EH_HOST_PORT_CONFIG_H_

/* ── Step 1 — integrator's master override ─────────────────────────── */
#include "eh_host_port_master_config.h"

/* ── Step 2 — pull Kconfig CONFIG_* if present (IDF build) ─────────── */
#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"
#  endif
#endif

#include "eh_host_port_types.h"

/* ════════════════════════════════════════════════════════════════════
 * OS PRIMITIVE GROUPS — tasks, synchronisation, timing, heap, log
 * ════════════════════════════════════════════════════════════════════ */

#ifndef EH_HOST_PORT_HAS_TASK
#  ifdef CONFIG_EH_HOST_PORT_HAS_TASK
#    define EH_HOST_PORT_HAS_TASK                 CONFIG_EH_HOST_PORT_HAS_TASK
#  else
#    define EH_HOST_PORT_HAS_TASK                 1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_MUTEX
#  ifdef CONFIG_EH_HOST_PORT_HAS_MUTEX
#    define EH_HOST_PORT_HAS_MUTEX                CONFIG_EH_HOST_PORT_HAS_MUTEX
#  else
#    define EH_HOST_PORT_HAS_MUTEX                1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_COND
#  ifdef CONFIG_EH_HOST_PORT_HAS_COND
#    define EH_HOST_PORT_HAS_COND                 CONFIG_EH_HOST_PORT_HAS_COND
#  else
#    define EH_HOST_PORT_HAS_COND                 1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_SEM
#  ifdef CONFIG_EH_HOST_PORT_HAS_SEM
#    define EH_HOST_PORT_HAS_SEM                  CONFIG_EH_HOST_PORT_HAS_SEM
#  else
#    define EH_HOST_PORT_HAS_SEM                  1
#  endif
#endif

/* ISR-safe sem_post variant — default off on non-FreeRTOS targets. */
#ifndef EH_HOST_PORT_HAS_SEM_POST_FROM_ISR
#  ifdef CONFIG_EH_HOST_PORT_HAS_SEM_POST_FROM_ISR
#    define EH_HOST_PORT_HAS_SEM_POST_FROM_ISR    CONFIG_EH_HOST_PORT_HAS_SEM_POST_FROM_ISR
#  elif defined(ESP_PLATFORM)
#    define EH_HOST_PORT_HAS_SEM_POST_FROM_ISR    1
#  else
#    define EH_HOST_PORT_HAS_SEM_POST_FROM_ISR    0
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_EVENT_GROUP
#  ifdef CONFIG_EH_HOST_PORT_HAS_EVENT_GROUP
#    define EH_HOST_PORT_HAS_EVENT_GROUP          CONFIG_EH_HOST_PORT_HAS_EVENT_GROUP
#  else
#    define EH_HOST_PORT_HAS_EVENT_GROUP          1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_QUEUE
#  ifdef CONFIG_EH_HOST_PORT_HAS_QUEUE
#    define EH_HOST_PORT_HAS_QUEUE                CONFIG_EH_HOST_PORT_HAS_QUEUE
#  else
#    define EH_HOST_PORT_HAS_QUEUE                1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_TIMER
#  ifdef CONFIG_EH_HOST_PORT_HAS_TIMER
#    define EH_HOST_PORT_HAS_TIMER                CONFIG_EH_HOST_PORT_HAS_TIMER
#  else
#    define EH_HOST_PORT_HAS_TIMER                1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_TIME
#  ifdef CONFIG_EH_HOST_PORT_HAS_TIME
#    define EH_HOST_PORT_HAS_TIME                 CONFIG_EH_HOST_PORT_HAS_TIME
#  else 
#    define EH_HOST_PORT_HAS_TIME                 1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_MEM
#  ifdef CONFIG_EH_HOST_PORT_HAS_MEM
#    define EH_HOST_PORT_HAS_MEM                  CONFIG_EH_HOST_PORT_HAS_MEM
#  else
#    define EH_HOST_PORT_HAS_MEM                  1
#  endif
#endif

/* DMA-capable heap. */
#ifndef EH_HOST_PORT_HAS_DMA
#  ifdef CONFIG_EH_HOST_PORT_HAS_DMA
#    define EH_HOST_PORT_HAS_DMA                  CONFIG_EH_HOST_PORT_HAS_DMA
#  else
#    define EH_HOST_PORT_HAS_DMA                  1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_LOG
#  ifdef CONFIG_EH_HOST_PORT_HAS_LOG
#    define EH_HOST_PORT_HAS_LOG                  CONFIG_EH_HOST_PORT_HAS_LOG
#  else
#    define EH_HOST_PORT_HAS_LOG                  1
#  endif
#endif

/* ════════════════════════════════════════════════════════════════════
 * HAL GROUPS — GPIO, power / reset / sleep
 * ════════════════════════════════════════════════════════════════════ */

#ifndef EH_HOST_PORT_HAS_GPIO
#  ifdef CONFIG_EH_HOST_PORT_HAS_GPIO
#    define EH_HOST_PORT_HAS_GPIO                 CONFIG_EH_HOST_PORT_HAS_GPIO
#  else
#    define EH_HOST_PORT_HAS_GPIO                 1
#  endif
#endif

/* Separate from HAS_GPIO so a target without edge-interrupt capability
 * can still expose read/write without the interrupt-install surface. */
#ifndef EH_HOST_PORT_HAS_GPIO_INTR
#  ifdef CONFIG_EH_HOST_PORT_HAS_GPIO_INTR
#    define EH_HOST_PORT_HAS_GPIO_INTR            CONFIG_EH_HOST_PORT_HAS_GPIO_INTR
#  else
#    define EH_HOST_PORT_HAS_GPIO_INTR            EH_HOST_PORT_HAS_GPIO_INTR
#  endif
#endif

/* Sleep-hold pin state (ESP specialty; other targets may lack). */
#ifndef EH_HOST_PORT_HAS_GPIO_HOLD
#  ifdef CONFIG_EH_HOST_PORT_HAS_GPIO_HOLD
#    define EH_HOST_PORT_HAS_GPIO_HOLD            CONFIG_EH_HOST_PORT_HAS_GPIO_HOLD
#  elif defined(ESP_PLATFORM)
#    define EH_HOST_PORT_HAS_GPIO_HOLD            1
#  else
#    define EH_HOST_PORT_HAS_GPIO_HOLD            0
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_POWER
#  ifdef CONFIG_EH_HOST_PORT_HAS_POWER
#    define EH_HOST_PORT_HAS_POWER                CONFIG_EH_HOST_PORT_HAS_POWER
#  else
#    define EH_HOST_PORT_HAS_POWER                1
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_POWER_SAVE
#  ifdef CONFIG_EH_HOST_PORT_HAS_POWER_SAVE
#    define EH_HOST_PORT_HAS_POWER_SAVE           CONFIG_EH_HOST_PORT_HAS_POWER_SAVE
#  else
#    define EH_HOST_PORT_HAS_POWER_SAVE           EH_HOST_PORT_HAS_POWER
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_WAKEUP_REASON
#  ifdef CONFIG_EH_HOST_PORT_HAS_WAKEUP_REASON
#    define EH_HOST_PORT_HAS_WAKEUP_REASON        CONFIG_EH_HOST_PORT_HAS_WAKEUP_REASON
#  else
#    define EH_HOST_PORT_HAS_WAKEUP_REASON        EH_HOST_PORT_HAS_POWER_SAVE
#  endif
#endif

/* ════════════════════════════════════════════════════════════════════
 * PROVIDER GROUPS — netif, HCI
 * ════════════════════════════════════════════════════════════════════ */

#ifndef EH_HOST_PORT_HAS_NETIF
#  ifdef CONFIG_EH_HOST_PORT_HAS_NETIF
#    define EH_HOST_PORT_HAS_NETIF                CONFIG_EH_HOST_PORT_HAS_NETIF
#  else
#    define EH_HOST_PORT_HAS_NETIF                1
#  endif
#endif

/* ════════════════════════════════════════════════════════════════════
 * EVENT BUS + DOMAIN EVENT FAMILIES
 * The event core is the pivot; each domain follows its consumer.
 * ════════════════════════════════════════════════════════════════════ */

#ifndef EH_HOST_PORT_HAS_EVENT_BUS
#  ifdef CONFIG_EH_HOST_PORT_HAS_EVENT_BUS
#    define EH_HOST_PORT_HAS_EVENT_BUS            CONFIG_EH_HOST_PORT_HAS_EVENT_BUS
#  else
#    define EH_HOST_PORT_HAS_EVENT_BUS            1
#  endif
#endif

/* Domain event headers — each is independently gated so integrators
 * can drop unused families. */
#ifndef EH_HOST_PORT_HAS_TRANSPORT_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_TRANSPORT_EVENTS
#    define EH_HOST_PORT_HAS_TRANSPORT_EVENTS     CONFIG_EH_HOST_PORT_HAS_TRANSPORT_EVENTS
#  else
#    define EH_HOST_PORT_HAS_TRANSPORT_EVENTS     EH_HOST_PORT_HAS_EVENT_BUS
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_SYSTEM_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_SYSTEM_EVENTS
#    define EH_HOST_PORT_HAS_SYSTEM_EVENTS        CONFIG_EH_HOST_PORT_HAS_SYSTEM_EVENTS
#  else
#    define EH_HOST_PORT_HAS_SYSTEM_EVENTS        EH_HOST_PORT_HAS_EVENT_BUS
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_WIFI_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_WIFI_EVENTS
#    define EH_HOST_PORT_HAS_WIFI_EVENTS          CONFIG_EH_HOST_PORT_HAS_WIFI_EVENTS
#  else
#    define EH_HOST_PORT_HAS_WIFI_EVENTS          EH_HOST_PORT_HAS_EVENT_BUS
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_NETIF_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_NETIF_EVENTS
#    define EH_HOST_PORT_HAS_NETIF_EVENTS         CONFIG_EH_HOST_PORT_HAS_NETIF_EVENTS
#  else
#    define EH_HOST_PORT_HAS_NETIF_EVENTS         (EH_HOST_PORT_HAS_EVENT_BUS && EH_HOST_PORT_HAS_NETIF)
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_PEER_DATA_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_PEER_DATA_EVENTS
#    define EH_HOST_PORT_HAS_PEER_DATA_EVENTS     CONFIG_EH_HOST_PORT_HAS_PEER_DATA_EVENTS
#  else
#    define EH_HOST_PORT_HAS_PEER_DATA_EVENTS     EH_HOST_PORT_HAS_EVENT_BUS
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_OTA_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_OTA_EVENTS
#    define EH_HOST_PORT_HAS_OTA_EVENTS           CONFIG_EH_HOST_PORT_HAS_OTA_EVENTS
#  else
#    define EH_HOST_PORT_HAS_OTA_EVENTS           EH_HOST_PORT_HAS_EVENT_BUS
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_POWER_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_POWER_EVENTS
#    define EH_HOST_PORT_HAS_POWER_EVENTS         CONFIG_EH_HOST_PORT_HAS_POWER_EVENTS
#  else
#    define EH_HOST_PORT_HAS_POWER_EVENTS         (EH_HOST_PORT_HAS_EVENT_BUS && EH_HOST_PORT_HAS_POWER)
#  endif
#endif

#ifndef EH_HOST_PORT_HAS_GPIO_EVENTS
#  ifdef CONFIG_EH_HOST_PORT_HAS_GPIO_EVENTS
#    define EH_HOST_PORT_HAS_GPIO_EVENTS          CONFIG_EH_HOST_PORT_HAS_GPIO_EVENTS
#  else
#    define EH_HOST_PORT_HAS_GPIO_EVENTS          (EH_HOST_PORT_HAS_EVENT_BUS && EH_HOST_PORT_HAS_GPIO_INTR)
#  endif
#endif

#endif /* EH_HOST_PORT_CONFIG_H_ */
