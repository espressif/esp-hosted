#ifndef _ESP_SERIAL_H_
#define _ESP_SERIAL_H_

int esp_serial_init(void * priv);
void esp_serial_cleanup(void);

int esp_serial_data_received(int dev_index, const char *data, size_t len);
#endif
