#ifndef __ESP_HOSTED_RPC_H__
#define __ESP_HOSTED_RPC_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* This file contains the custom RPC message IDs shared between the ESP Firmware and the Host side Protobuf Application.

 * These IDs are just demonstration purpose. You can customize them as per your application requirements.
 */


enum custom_rpc_req_id {
	CUSTOM_RPC_REQ_ID__ONLY_ACK                          = 1,
	CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE                = 2,
	CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT                = 3,
	/* Add more request IDs as needed */
};

enum custom_rpc_event_id {
	/* Slave use this event to send back data received in CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT */
	CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST          = 100,
	CUSTOM_RPC_EVENT_ID__DEMO_SIMPLE_EVENT               = 101,
	/* Add more event IDs as needed */
};

#endif /* __ESP_HOSTED_RPC_H__ */
