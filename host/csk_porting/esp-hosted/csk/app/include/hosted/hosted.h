#pragma once

#include "ctrl_api.h"
#include "platform_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOSTED_LINK_EVT_ACTIVE                  = BIT(0),
    HOSTED_WIFI_EVT_AP_STACONNECTED         = BIT(1),
    HOSTED_WIFI_EVT_AP_STADISCONNECTED      = BIT(2),
    HOSTED_WIFI_EVT_STA_DISCONNECTED        = BIT(3),
} hosted_event_t;

typedef void(*hosted_event_cb_t) (hosted_event_t event, void *arg);

typedef struct hosted_event_callback {
    sys_snode_t node;
    hosted_event_cb_t handler;
    uint32_t events;
    void *arg;
} hosted_event_callback_t;

int hosted_init();
bool hosted_is_inited(void);
int hosted_add_callback(struct hosted_event_callback *hosted_event_cb);
bool hosted_event_is_set(uint32_t events);

#ifdef __cplusplus
}
#endif
