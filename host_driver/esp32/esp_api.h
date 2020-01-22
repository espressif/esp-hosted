#ifndef _esp_api__h_
#define _esp_api__h_

#include "esp.h"

int add_card(struct esp_adapter *adapter);
int remove_card(struct esp_adapter *adapter);
void process_new_packet_intr(struct esp_adapter *adapter);
struct esp_adapter * get_adapter(void);
struct sk_buff * esp32_alloc_skb(u32 len);
int esp32_send_packet(struct esp_adapter *adapter, u8 *buf, u32 size);

#endif
