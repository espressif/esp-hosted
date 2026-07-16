/* SPDX-License-Identifier: Apache-2.0 */
/* Compile-time guard: Classic BT (BR/EDR) only exists on ESP32. */

#ifndef CLASSIC_BT_PRIV_H_
#define CLASSIC_BT_PRIV_H_

#include "sdkconfig.h"

#if !defined(CONFIG_SLAVE_IDF_TARGET_ESP32)
#  error \
"\n\n" \
"================================================================\n" \
"  classic_bt_discovery requires an ESP32 coprocessor.\n" \
"  Classic BT (BR/EDR) is not supported on C2/C3/C5/C6/H2/H4/P4.\n" \
"\n" \
"  Fix: pin the CP target in sdkconfig.defaults:\n" \
"      CONFIG_SLAVE_IDF_TARGET_ESP32=y\n" \
"  (CONFIG_ESP_HOSTED_CP_TARGET_ESP32 follows by cascade.)\n" \
"================================================================\n"
#endif

#if !defined(CONFIG_BT_CLASSIC_ENABLED)
#  error \
"\n\n" \
"================================================================\n" \
"  classic_bt_discovery needs Bluedroid Classic BT enabled.\n" \
"\n" \
"  Fix: in sdkconfig.defaults:\n" \
"      CONFIG_BT_ENABLED=y\n" \
"      CONFIG_BT_BLUEDROID_ENABLED=y\n" \
"      CONFIG_BT_CLASSIC_ENABLED=y\n" \
"================================================================\n"
#endif

#endif
