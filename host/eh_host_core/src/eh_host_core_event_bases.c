/* SPDX-License-Identifier: Apache-2.0 */
/* Link-stable address for EH_HOST_EVENT; ESP_HOSTED_EVENT is a strong alias. */

#include "esp_event_base.h"

ESP_EVENT_DEFINE_BASE(EH_HOST_EVENT);

extern const char *const ESP_HOSTED_EVENT
    __attribute__((alias("EH_HOST_EVENT")));
