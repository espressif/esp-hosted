/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_tags.h — compiler-attribute helpers
 *
 * Portable wrappers for GCC / Clang attributes that port-layer headers
 * (and consumers) apply to declarations.  Every macro resolves to the
 * intended attribute under GCC or Clang, and collapses to empty under
 * other toolchains.  Callers can always use these macros; a weaker
 * toolchain will simply not emit the attribute, never a compile error.
 *
 * Scope:
 *   Keep this file minimal.  One entry per attribute we actually use
 *   in a public header or consumer.  Non-portable attributes (IDF's
 *   IRAM_ATTR, target-specific section names) belong in port-type
 *   implementation sources, not here.
 */

#ifndef EH_HOST_PORT_TAGS_H_
#define EH_HOST_PORT_TAGS_H_

#if defined(__GNUC__) || defined(__clang__)

#define EH_ATTR_NONNULL(...)     __attribute__((nonnull(__VA_ARGS__)))
#define EH_ATTR_FORMAT(archetype, fmt_idx, first_arg) \
    __attribute__((format(archetype, fmt_idx, first_arg)))
#define EH_ATTR_UNUSED           __attribute__((unused))
#define EH_ATTR_WEAK             __attribute__((weak))
#define EH_ATTR_DEPRECATED(msg)  __attribute__((deprecated(msg)))
#define EH_ATTR_NORETURN         __attribute__((noreturn))
#define EH_ATTR_PACKED           __attribute__((packed))
#define EH_ATTR_ALIGNED(n)       __attribute__((aligned(n)))
#define EH_ATTR_SECTION(name)    __attribute__((section(name)))
#define EH_ATTR_WARN_UNUSED      __attribute__((warn_unused_result))

/* IRAM residency — ESP-specific; maps to IRAM_ATTR on ESP_PLATFORM
 * (so ISR functions called with flash cache disabled don't trap),
 * empty on other targets. */
#if defined(ESP_PLATFORM)
#  include "esp_attr.h"
#  define EH_HOST_PORT_IRAM_ATTR  IRAM_ATTR
/* RTC-memory residency so that there is no deep-sleep RAM loss. */
#  define EH_HOST_PORT_RETAIN_ATTR  RTC_DATA_ATTR
#else
#  define EH_HOST_PORT_IRAM_ATTR
#  define EH_HOST_PORT_RETAIN_ATTR
#endif

#else /* Other toolchains — drop attributes silently. */

#define EH_ATTR_NONNULL(...)
#define EH_ATTR_FORMAT(archetype, fmt_idx, first_arg)
#define EH_ATTR_UNUSED
#define EH_ATTR_WEAK
#define EH_ATTR_DEPRECATED(msg)
#define EH_ATTR_NORETURN
#define EH_ATTR_PACKED
#define EH_ATTR_ALIGNED(n)
#define EH_ATTR_SECTION(name)
#define EH_ATTR_WARN_UNUSED

#endif

/* Defined above (inside the GCC/Clang branch) or here as fallback. */
#ifndef EH_HOST_PORT_IRAM_ATTR
#  define EH_HOST_PORT_IRAM_ATTR
#endif
#ifndef EH_HOST_PORT_RETAIN_ATTR
#  define EH_HOST_PORT_RETAIN_ATTR
#endif

#endif /* EH_HOST_PORT_TAGS_H_ */
