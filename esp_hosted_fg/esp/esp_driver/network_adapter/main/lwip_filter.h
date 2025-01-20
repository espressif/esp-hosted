#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_hosted_lwip_src_port_hook.h"

typedef enum {
	SLAVE_LWIP_BRIDGE,
	HOST_LWIP_BRIDGE,
	BOTH_LWIP_BRIDGE,
	INVALID_BRIDGE,
} hosted_l2_bridge;

hosted_l2_bridge filter_and_route_packet(void *frame_data, uint16_t frame_length);
