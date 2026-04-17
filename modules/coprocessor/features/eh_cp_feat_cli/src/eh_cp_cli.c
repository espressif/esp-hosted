/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "eh_cp_cli.h"
#include "eh_cp_master_config.h"

#if EH_CP_FEAT_CLI_READY
#include "eh_cp_core.h"   /* EH_CP_FEAT_REGISTER */
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_console.h>
#include <esp_heap_caps.h>
#ifdef CONFIG_HEAP_TRACING
#include <esp_heap_trace.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_wifi.h>
#include "lwip/sockets.h"
#if EH_CP_FEAT_HOST_PS_READY
#include "eh_cp_feat_host_ps_apis.h"
#endif

static const char *TAG = "esp_cli";

/* D3 — auto-init descriptor. start CLI after core is up. */
static esp_err_t eh_cp_feat_cli_init(void)
{
	eh_cp_feat_cli_register_commands();
	return eh_cp_feat_cli_start("coprocessor> ");
}

#if EH_CP_FEAT_CLI_AUTO_INIT
EH_CP_FEAT_REGISTER(eh_cp_feat_cli_init,
                   NULL,
                   "cli",
                   tskNO_AFFINITY,
                   80);
#endif

static int task_dump_cli_handler(int argc, char *argv[])
{
	/* Just to go to the next line */
	printf("\n");
#ifndef CONFIG_FREERTOS_USE_TRACE_FACILITY
	printf("%s: To use this utility enable: Component config > FreeRTOS > Kernel > configUSE_TRACEFACILITY\n", TAG);
#else
	int num_of_tasks = uxTaskGetNumberOfTasks();
	TaskStatus_t *task_array = calloc(1, num_of_tasks * sizeof(TaskStatus_t));
	if (!task_array) {
		ESP_LOGE(TAG, "Memory not allocated for task list.");
		return 0;
	}
	num_of_tasks = uxTaskGetSystemState(task_array, num_of_tasks, NULL);
	printf("\tName\tNumber\tPriority\tStackWaterMark\n");
	for (int i = 0; i < num_of_tasks; i++) {
		printf("%16s\t%u\t%u\t%u\n",
			   task_array[i].pcTaskName,
			   (unsigned) task_array[i].xTaskNumber,
			   (unsigned) task_array[i].uxCurrentPriority,
			   (unsigned) task_array[i].usStackHighWaterMark);
	}
	free(task_array);
#endif
	return 0;
}

static int cpu_dump_cli_handler(int argc, char *argv[])
{
	/* Just to go to the next line */
	printf("\n");
#ifndef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
	printf("%s: To use this utility enable: Component config > FreeRTOS > Kernel > configGENERATE_RUN_TIME_STATS\n", TAG);
#else
	char *buf = calloc(1, 2 * 1024);
	vTaskGetRunTimeStats(buf);
	printf("%s: Run Time Stats:\n%s\n", TAG, buf);
	free(buf);
#endif
	return 0;
}

static void print_heap_stats()
{
	uint32_t freeSize = esp_get_free_heap_size();
	printf("The available total size of heap:%" PRIu32 "\n", freeSize);

	printf("\tDescription\tInternal\tSPIRAM\n");
	printf("Current Free Memory\t%d\t\t%d\n",
		   heap_caps_get_free_size(MALLOC_CAP_8BIT) - heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
		   heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
	printf("Largest Free Block\t%d\t\t%d\n",
		   heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
		   heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
	printf("Min. Ever Free Size\t%d\t\t%d\n",
		   heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
		   heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
}

static int mem_dump_cli_handler(int argc, char *argv[])
{
	/* Just to go to the next line */
	printf("\n");
	print_heap_stats();
	return 0;
}

#ifdef CONFIG_HEAP_TRACING
static int heap_trace_records;
static heap_trace_record_t *heap_trace_records_buf;
static void cli_heap_trace_start()
{
	/* Just to go to the next line */
	printf("\n");
	if (!heap_trace_records_buf) {
		heap_trace_records_buf = malloc(heap_trace_records * sizeof(heap_trace_record_t));
		if (!heap_trace_records_buf) {
			printf("%s: Failed to allocate records buffer\n", TAG);
			return;
		}
		if (heap_trace_init_standalone(heap_trace_records_buf, heap_trace_records) != ESP_OK) {
			printf("%s: Failed to initialise tracing\n", TAG);
			goto error1;
		}
	}
	if (heap_trace_start(HEAP_TRACE_LEAKS) != ESP_OK) {
		printf("%s: Failed to start heap trace\n", TAG);
		goto error2;
	}
	return;
error2:
	heap_trace_init_standalone(NULL, 0);
error1:
	free(heap_trace_records_buf);
	heap_trace_records_buf = NULL;
	return;
}

static void cli_heap_trace_stop()
{
	/* Just to go to the next line */
	printf("\n");
	if (!heap_trace_records_buf) {
		printf("%s: Tracing not started?\n", TAG);
		return;
	}
	heap_trace_stop();
	heap_trace_dump();
	heap_trace_init_standalone(NULL, 0);
	free(heap_trace_records_buf);
	heap_trace_records_buf = NULL;
}
#endif

static int heap_trace_cli_handler(int argc, char *argv[])
{
	/* Just to go to the next line */
	printf("\n");
#ifndef CONFIG_HEAP_TRACING
	printf("%s: To use this utility enable: Component config --> Heap memory debugging --> Enable heap tracing\n", TAG);
#else
	if (argc < 2) {
		printf("%s: Incorrect arguments\n", TAG);
		return 0;
	}
	if (strcmp(argv[1], "start") == 0) {
#define DEFAULT_HEAP_TRACE_RECORDS 200
		if (argc != 3) {
			heap_trace_records = DEFAULT_HEAP_TRACE_RECORDS;
		} else {
			heap_trace_records = atoi(argv[2]);
		}
		printf("%s: Using a buffer to trace %d records\n", TAG, heap_trace_records);
		cli_heap_trace_start();
	} else if (strcmp(argv[1], "stop") == 0) {
		cli_heap_trace_stop();
	} else {
		printf("%s: Invalid argument:%s:\n", TAG, argv[1]);
	}
#endif
	return 0;
}

static int reboot_cli_handler(int argc, char *argv[])
{
	/* Just to go to the next line */
	printf("\n");
	esp_restart();
	return 0;
}

static int crash_device_handler(int argc, char** argv)
{
	printf("Crashing the device now...\n");

	// Writing at invalid address
	*(int *) (0x0) = 0;
	return ESP_OK;
}

static int sock_dump_cli_handler(int argc, char *argv[])
{
	printf("\n");
	int i, ret, used_sockets = 0;

	struct sockaddr_in local_sock, peer_sock;
	socklen_t local_sock_len = sizeof(struct sockaddr_in), peer_sock_len = sizeof(struct sockaddr_in);
	char local_ip_addr[16], peer_ip_addr[16];
	unsigned int local_port, peer_port;

	int sock_type;
	socklen_t sock_type_len;

#define TOTAL_NUM_SOCKETS MEMP_NUM_NETCONN
	printf("sock_fd\tprotocol\tlocal_addr\t\tpeer_addr\n");
	for (i = LWIP_SOCKET_OFFSET; i < LWIP_SOCKET_OFFSET + TOTAL_NUM_SOCKETS; i++) {
		memset(&local_sock, 0, sizeof(struct sockaddr_in));
		memset(&peer_sock, 0, sizeof(struct sockaddr_in));
		local_sock_len = sizeof(struct sockaddr);
		peer_sock_len = sizeof(struct sockaddr);
		memset(local_ip_addr, 0, sizeof(local_ip_addr));
		memset(peer_ip_addr, 0, sizeof(peer_ip_addr));
		local_port = 0;
		peer_port = 0;
		sock_type = 0;
		sock_type_len = sizeof(int);

		ret = getsockname(i, (struct sockaddr *)&local_sock, &local_sock_len);
		if (ret >= 0) {
			used_sockets++;
			inet_ntop(AF_INET, &local_sock.sin_addr, local_ip_addr, sizeof(local_ip_addr));
			local_port = ntohs(local_sock.sin_port);
			getsockopt(i, SOL_SOCKET, SO_TYPE, &sock_type, &sock_type_len);
			printf("%d\t%d:%s\t%16s:%d", i, sock_type, sock_type == SOCK_STREAM ? "tcp" : sock_type == SOCK_DGRAM ? "udp" : "raw", local_ip_addr, local_port);

			ret = getpeername(i, (struct sockaddr *)&peer_sock, &peer_sock_len);
			if (ret >= 0) {
				inet_ntop(AF_INET, &peer_sock.sin_addr, peer_ip_addr, sizeof(peer_ip_addr));
				peer_port = ntohs(peer_sock.sin_port);
				printf("\t%16s:%d", peer_ip_addr, peer_port);
			}
			printf("\n");
		}
	}
	printf("Remaining sockets: %d\n", TOTAL_NUM_SOCKETS - used_sockets);
	return 0;
}

#if EH_CP_FEAT_HOST_PS_READY
static int wakeup_cli_handler(int argc, char *argv[])
{
	ESP_LOGI(TAG, "Asking P4 to wake-up...");
	eh_cp_feat_host_ps_wakeup_host(1000);
	return 0;
}

static int register_host_power_save_commands(void)
{
	esp_console_cmd_t cmd = {
		.command = "wake-up",
		.help = "Wake-up P4 from deep sleep",
		.func = wakeup_cli_handler,
	};
	esp_console_cmd_register(&cmd);

	return 0;
}
#endif

static esp_console_cmd_t diag_cmds[] = {
	{
		.command = "crash",
		.help = "Crash the device writing at invalid address",
		.func = &crash_device_handler,
	},
	{
		.command = "reboot",
		.help = "Reboot the device",
		.func = reboot_cli_handler,
	},
	{
		.command = "mem-dump",
		.help = "Prints memory stats",
		.func = mem_dump_cli_handler,
	},
	{
		.command = "task-dump",
		.help = "Print task snapshots",
		.func = task_dump_cli_handler,
	},
	{
		.command = "cpu-dump",
		.help = "Print CPU consumption data at the moment",
		.func = cpu_dump_cli_handler,
	},
	{
		.command = "heap-trace",
		.help = "<start|stop> [trace-buf-size]",
		.func = heap_trace_cli_handler,
	},
	{
		.command = "sock-dump",
		.help = "",
		.func = sock_dump_cli_handler,
	},


//#if H_HOST_PS_DEEP_SLEEP_ALLOWED

// #ifdef CONFIG_USE_ESP_HOSTED
// 	{
// 		.command = "wake-up",
// 		.help = "Wake-up P4 from deep sleep",
// 		.func = wakeup_cli_handler,
// 	},
// #endif
//#endif
};



int eh_cp_feat_cli_register_commands(void)
{
	int cmds_num = sizeof(diag_cmds) / sizeof(esp_console_cmd_t);
	int i;
	const char* remove_cmd = "deep_sleep";

	ESP_LOGI(TAG, "Remove any existing deep_sleep cmd in cli");
	esp_console_cmd_deregister(remove_cmd);

	for (i = 0; i < cmds_num; i++) {
		ESP_LOGI(TAG, "Registering command: %s", diag_cmds[i].command);
		esp_console_cmd_register(&diag_cmds[i]);
	}
#if EH_CP_FEAT_HOST_PS_READY
	register_host_power_save_commands();
#endif

	return 0;
}


esp_err_t eh_cp_feat_cli_start(const char* custom_prompt)
{
	static int cli_started;
	esp_err_t ret = 0;

	if (cli_started) {
		ESP_LOGW(TAG, "CLI already started");
		return ESP_OK;
	}

	ESP_LOGI(TAG, "Starting CLI console");
	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

	/* Use custom prompt if provided, otherwise use default */
	if (custom_prompt) {
		repl_config.prompt = custom_prompt;
	} else {
		repl_config.prompt = "coprocessor> ";
	}

	esp_console_register_help_command();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
	esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
	ret = esp_console_new_repl_uart(&hw_config, &repl_config, &repl);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create UART REPL: %s", esp_err_to_name(ret));
		return ret;
	}

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
	esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
	ret = esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create USB CDC REPL: %s", esp_err_to_name(ret));
		return ret;
	}

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
	esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	ret = esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create USB Serial JTAG REPL: %s", esp_err_to_name(ret));
		return ret;
	}

#else
	ESP_LOGE(TAG, "No console type configured");
	return ESP_ERR_NOT_SUPPORTED;
#endif

	ret = esp_console_start_repl(repl);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to start REPL: %s", esp_err_to_name(ret));
		return ret;
	}

	cli_started = 1;
	ESP_LOGI(TAG, "CLI started successfully");
	return ESP_OK;
}

#endif /*ESP_HOSTED_CP_FEAT_CLI*/
