/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "sdkconfig.h"

#ifdef CONFIG_ESP_HOSTED_COPROCESSOR_EXAMPLE_MQTT

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "esp_log.h"
#include "mqtt_client.h"
#include "host_power_save.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

static const char *TAG = "mqtt_example";
static esp_mqtt_client_handle_t client;
static uint8_t client_started;

#define WAKEUP_HOST_STRING "wakeup-host"

static void log_error_if_nonzero(const char *message, int error_code)
{
	if (error_code != 0) {
		ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
	}
}

static bool host_mqtt_wakeup_triggered(const void *payload, uint16_t payload_length)
{
	/* Check if payload contains "wakeup-host" string */
	if (payload_length >= strlen(WAKEUP_HOST_STRING) &&
			memcmp(payload, WAKEUP_HOST_STRING, strlen(WAKEUP_HOST_STRING)) == 0) {
		return true;
	}
	return false;
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
	esp_mqtt_event_handle_t event = event_data;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	switch ((esp_mqtt_event_id_t)event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
			msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
			ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
			ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			break;

		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
			ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);

			if (host_mqtt_wakeup_triggered(event->data, event->data_len)) {
				wakeup_host(portMAX_DELAY);
			}
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
			if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
				log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
				log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
				log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
				ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

			}
			break;
		default:
			ESP_LOGI(TAG, "Other event id:%d", event->event_id);
			break;
	}
}

#if 0
static int check_ip_reachability(const char *ip, int port)
{
	int sock;
	struct sockaddr_in server;

	// Create socket
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == -1) {
		printf("Could not create socket\n");
		return ESP_FAIL;
	}

	// Configure the server address
	server.sin_addr.s_addr = inet_addr(ip);
	server.sin_family = AF_INET;
	server.sin_port = htons(port);

	// Set a timeout for the connection attempt
	struct timeval timeout;
	timeout.tv_sec = 5;  // 5 seconds timeout
	timeout.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

	// Try to connect to the server
	if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
		printf("Connection to %s on port %d failed\n", ip, port);
		close(sock);
		return ESP_FAIL;
	}

	printf("Successfully connected to %s on port %d\n", ip, port);
	close(sock);
	return ESP_OK;
}
#endif

static esp_mqtt_client_handle_t example_mqtt_init_internal(void)
{
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = CONFIG_BROKER_URL,
	};
#if CONFIG_BROKER_URL_FROM_STDIN
	char line[128];

	if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
		int count = 0;
		printf("Please enter url of mqtt broker\n");
		while (count < 128) {
			int c = fgetc(stdin);
			if (c == '\n') {
				line[count] = '\0';
				break;
			} else if (c > 0 && c < 127) {
				line[count] = c;
				++count;
			}
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		mqtt_cfg.broker.address.uri = line;
		printf("Broker url: %s\n", line);
	} else {
		ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
		abort();
	}
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

	client = esp_mqtt_client_init(&mqtt_cfg);
	/* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

	return client;
}

esp_err_t example_mqtt_init(void)
{
#if 0
	esp_err_t err;
	err = check_ip_reachability("8.8.8.8", 53);
	if (err) {
		ESP_LOGI(TAG, "Failed to start the mqtt");
		return NULL;
	}
#endif

	ESP_LOGI(TAG, "[APP] Startup..");
	ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
	esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
	esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
	esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
	esp_log_level_set("transport", ESP_LOG_VERBOSE);
	esp_log_level_set("outbox", ESP_LOG_VERBOSE);

	example_mqtt_init_internal();

	if (client)
		return ESP_OK;
	else
		return ESP_FAIL;
}

esp_err_t example_mqtt_resume(void)
{
	if (!client) {
		example_mqtt_init();
	}

	if (client && !client_started) {
		client_started = 1;
		ESP_LOGV(TAG, "mqtt client started");
		return esp_mqtt_client_start(client);
	}
	ESP_LOGV(TAG, "%s : client[%p] client_started[%u]",
			__func__, client, client_started);

	return ESP_OK;
}

esp_err_t example_mqtt_pause(void)
{
	if (client && client_started) {
		client_started = 0;
		ESP_LOGV(TAG, "mqtt client stopped");
		return esp_mqtt_client_stop(client);
#if 0
		if (ESP_OK == esp_mqtt_client_destroy(client)) {
			client = NULL;
		}
#endif
	}
	ESP_LOGV(TAG, "%s : client[%p] client_started[%u]",
			__func__, client, client_started);

	return ESP_OK;
}


#endif
