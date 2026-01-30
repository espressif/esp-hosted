/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef __EXAMPLE_MQTT_CLIENT_H__
#define __EXAMPLE_MQTT_CLIENT_H__

#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_MQTT
esp_err_t example_mqtt_init(void);
esp_err_t example_mqtt_resume(void);
esp_err_t example_mqtt_pause(void);
#endif
#endif

