//#include "stdint.h"
#include "cmsis_os.h"
#include "common.h"
#include "stdio.h"
#include "adapter.h"
#include "netdev_if.h"
#include "platform_wrapper.h"
#include "trace.h"

void process_capabilities(uint8_t cap);
void transport_init(void(*transport_evt_handler)(uint8_t));

void process_event(uint8_t *evt_buf, uint16_t len);
void process_priv_communication(struct pbuf *pbuf);
void print_capabilities(uint32_t cap);
int process_init_event(uint8_t *evt_buf, uint8_t len);

stm_ret_t send_to_slave(uint8_t iface_type, uint8_t iface_num,
		uint8_t * wbuffer, uint16_t wlen);
