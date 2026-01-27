// SPDX-License-Identifier: GPL-2.0-only
/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 */


#include "test.h"
#include "serial_if.h"
#include <signal.h>
#include "ctrl_api.h"
#include <syslog.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include "nw_helper_func.h"


#define STA_INTERFACE                                     "ethsta0"


#define YES 1
#define NO  0
#define MIN_TIMESTAMP_STR_SIZE 30

#define MAC_ADDR_LENGTH 18


#define LOG_MSG(level, fmt, ...) \
    do { \
        if (!run_in_foreground) { \
            syslog(level, fmt, ##__VA_ARGS__); \
        } else { \
            printf(fmt "\n", ##__VA_ARGS__); \
        } \
    } while(0)

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

#define CONFIG_FILE "/etc/esp-hosted/config.conf"

#define LOCK_FILE "/var/run/hosted_daemon.lock"
static int lock_fd = -1;


static network_info_t sta_network;

static uint8_t local_network_up = 0;

static int run_in_foreground = 0;
static const char *config_file = CONFIG_FILE;

static inline ctrl_cmd_t * CTRL_CMD_DEFAULT_REQ()
{
	ctrl_cmd_t *req = (ctrl_cmd_t *)calloc(1, sizeof(ctrl_cmd_t));
	req->msg_type = CTRL_REQ;
	req->ctrl_resp_cb = NULL;
	req->cmd_timeout_sec = DEFAULT_CTRL_RESP_TIMEOUT /*5 sec*/;
	return req;
}

static char * get_timestamp(char *str, uint16_t str_size)
{
	if (str && str_size>=MIN_TIMESTAMP_STR_SIZE) {
		time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		sprintf(str, "%d-%02d-%02d %02d:%02d:%02d > ", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		return str;
	}
	return NULL;
}

static int event_cb(ctrl_cmd_t * app_event)
{
	/* Note: event_cb is callback run by rx-thread
	 * Do not trigger any blocking or time consuming operations
	 * */
	char ts[MIN_TIMESTAMP_STR_SIZE] = {'\0'};

	if (test_validate_ctrl_event(app_event)) {
		CLEANUP_CTRL_MSG(app_event);
		return FAILURE;
	}

	switch(app_event->msg_id) {

		case CTRL_EVENT_ESP_INIT: {
			LOG_MSG(LOG_INFO, "%s App EVENT: ESP INIT",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE));
			break;
		} case CTRL_EVENT_DHCP_DNS_STATUS: {
			dhcp_dns_status_t *p_e = &app_event->u.dhcp_dns_status;
			LOG_MSG(LOG_INFO, "%s App EVENT: DHCP DNS status: iface[%d] dhcp_up[%d] dns_up[%d]",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), p_e->iface, p_e->dhcp_up, p_e->dns_up);
			if (p_e->dhcp_up) {
				strncpy(sta_network.ip_addr, (const char *)p_e->dhcp_ip, MAC_ADDR_LENGTH);
				strncpy(sta_network.netmask, (const char *)p_e->dhcp_nm, MAC_ADDR_LENGTH);
				strncpy(sta_network.gateway, (const char *)p_e->dhcp_gw, MAC_ADDR_LENGTH);
				strncpy(sta_network.default_route, (const char *)p_e->dhcp_gw, MAC_ADDR_LENGTH);
				sta_network.ip_valid = 1;
			} else {
				local_network_up = 0;
				sta_network.ip_valid = 0;
			}
			if (p_e->dns_up) {
				strncpy(sta_network.dns_addr, (const char *)p_e->dns_ip, MAC_ADDR_LENGTH);
				sta_network.dns_valid = 1;
			} else {
				sta_network.dns_valid = 0;
			}

			if (sta_network.dns_valid && sta_network.ip_valid) {
				LOG_MSG(LOG_INFO, "Network identified as up");
				up_sta_netdev__with_static_ip_dns_route(&sta_network);
				add_dns(sta_network.dns_addr);
				local_network_up = 1;
			} else {
				LOG_MSG(LOG_INFO, "Network identified as down");
				down_sta_netdev(&sta_network);
				remove_dns(sta_network.dns_addr);
				local_network_up = 0;
			}



			break;
		} default: {
			LOG_MSG(LOG_ERR, "%s Invalid event[%u] to parse",
				get_timestamp(ts, MIN_TIMESTAMP_STR_SIZE), app_event->msg_id);
			break;
		}
	}
	CLEANUP_CTRL_MSG(app_event);
	return SUCCESS;
}

int resp_cb(ctrl_cmd_t * app_resp)
{

	if (test_validate_ctrl_resp(app_resp)) {
		CLEANUP_CTRL_MSG(app_resp);
		return FAILURE;
	}

	switch(app_resp->msg_id) {

		case CTRL_RESP_GET_MAC_ADDR: {
			LOG_MSG(LOG_INFO, "mac address is %s", app_resp->u.wifi_mac.mac);
			break;

		} case CTRL_RESP_GET_DHCP_DNS_STATUS: {
			dhcp_dns_status_t *p = &app_resp->u.dhcp_dns_status;
			if (p->dhcp_up) {
				LOG_MSG(LOG_INFO, "DHCP IP: %s", p->dhcp_ip);
				LOG_MSG(LOG_INFO, "DHCP NM: %s", p->dhcp_nm);
				LOG_MSG(LOG_INFO, "DHCP GW: %s", p->dhcp_gw);
				strncpy(sta_network.ip_addr, (const char *)p->dhcp_ip, MAC_ADDR_LENGTH);
				strncpy(sta_network.netmask, (const char *)p->dhcp_nm, MAC_ADDR_LENGTH);
				strncpy(sta_network.gateway, (const char *)p->dhcp_gw, MAC_ADDR_LENGTH);
				sta_network.ip_valid = 1;
			} else {
				LOG_MSG(LOG_INFO, "DHCP is not up");
				sta_network.ip_valid = 0;
			}
			if (p->dns_up) {
				LOG_MSG(LOG_INFO, "DNS IP: %s", p->dns_ip);
				strncpy(sta_network.dns_addr, (const char *)p->dns_ip, MAC_ADDR_LENGTH);
				sta_network.dns_valid = 1;
			} else {
				LOG_MSG(LOG_INFO, "DNS is not up");
				sta_network.dns_valid = 0;
			}

			break;
		} default: {
			LOG_MSG(LOG_ERR, "Unsupported Response[%u] to parse", app_resp->msg_id);
			break;
		}
	}

	CLEANUP_CTRL_MSG(app_resp);
	return SUCCESS;
}



static int subscribe_events(void)
{
	/* Please note, the event_cb would be called asynchronously by underlying rx-thread
	 * 1. Do not do any blocking operation in event_cb.
	 * 2. Do not trigger any 'sync' rpc control request, as they are blocking in nature.
	 * 3. 'async' rpc control requests are allowed.
	 * 4. Be restrictive of code  or execution time in event_cb,
	 *    as it would block rx-thread for any next incoming messages
	 * */
	int ret = SUCCESS;
	int evt = 0;

	typedef struct {
		int event;
		ctrl_resp_cb_t fun;
	} event_callback_table_t;

	event_callback_table_t events[] = {
		{ CTRL_EVENT_ESP_INIT,                           event_cb },
		{ CTRL_EVENT_DHCP_DNS_STATUS,                    event_cb },
	};

	for (evt=0; evt<sizeof(events)/sizeof(event_callback_table_t); evt++) {
		if (CALLBACK_SET_SUCCESS != set_event_callback(events[evt].event, events[evt].fun) ) {
			LOG_MSG(LOG_ERR, "event callback register failed for event[%u]", events[evt].event);
			ret = FAILURE;
			break;
		}
	}
	return ret;
}


static int unsubscribe_events(void)
{
	int ret = SUCCESS;
	int evt = 0;
	for (evt=CTRL_EVENT_BASE+1; evt<CTRL_EVENT_MAX; evt++) {
		if (CALLBACK_SET_SUCCESS != reset_event_callback(evt) ) {
			LOG_MSG(LOG_ERR, "reset event callback failed for event[%u]", evt);
			ret = FAILURE;
		}
	}
	return ret;
}

static int fetch_mac_addr_from_slave(void)
{
	/* implemented synchronous */
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	ctrl_cmd_t *resp = NULL;

	req->u.wifi_mac.mode = WIFI_MODE_STA;
	resp = wifi_get_mac(req);

	if (resp && resp->resp_event_status == SUCCESS) {
		strncpy(sta_network.mac_addr, resp->u.wifi_mac.mac, MAC_ADDR_LENGTH);
	}
	free(req);

	return resp_cb(resp);
}

static int fetch_ip_addr_from_slave(void)
{
	ctrl_cmd_t *resp = NULL;
	ctrl_cmd_t *req = CTRL_CMD_DEFAULT_REQ();
	req->cmd_timeout_sec = 1;

	resp = get_dhcp_dns_status(req);
	free(req);
	return resp_cb(resp);
}

static uint8_t app_init_done = 0;
static int init_app(void)
{
	if (app_init_done) {
		return SUCCESS;
	}

	if (init_hosted_control_lib()) {
		//printf("init hosted control lib failed\n");
		return FAILURE;
	}

	test_is_network_split_on();

	subscribe_events();

	app_init_done = 1;
	LOG_MSG(LOG_INFO, "RPC at host is ready");
	return 0;
}

static void cleanup_app(void)
{
	if (!app_init_done) {
		return;
	}

	LOG_MSG(LOG_INFO, "Cleaning up application");

	unsubscribe_events();

	control_path_platform_deinit();
	deinit_hosted_control_lib();
	app_init_done = 0;

	/* Release lock file */
	if (lock_fd >= 0) {
		close(lock_fd);
		unlink(LOCK_FILE);
	}
}

static void reload_config(void)
{
	/* Add any configuration reload logic here */
	LOG_MSG(LOG_INFO, "Configuration reloaded");
}

static void sig_handler(int signum)
{
	switch (signum) {
		case SIGHUP:
			reload_config();
			break;
		case SIGUSR1:
			LOG_MSG(LOG_INFO, "SIGUSR1 received, cleaning up application");
			cleanup_app();
			break;
		case SIGTERM:
		case SIGINT:
			LOG_MSG(LOG_INFO, "Clean-up and exit");
			cleanup_app();
			if (!run_in_foreground) {
				closelog();
			}
			unlink("/var/run/hosted_daemon.pid");
			exit(0);
			break;
	}
}

static void daemonize(void)
{
	pid_t pid;

	/* Fork off the parent process */
	pid = fork();
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	/* Create new session */
	if (setsid() < 0) {
		exit(EXIT_FAILURE);
	}

	/* Fork off for the second time */
	pid = fork();
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	/* Set new file permissions */
	umask(0);

	/* Change the working directory */
	chdir("/");

	/* Close all open file descriptors */
	int x;
	for (x = sysconf(_SC_OPEN_MAX); x>=0; x--) {
		close(x);
	}

	/* Open the log file */
	openlog("hosted_daemon", LOG_PID, LOG_DAEMON);

	/* Write PID file */
	FILE *pid_fp = fopen("/var/run/hosted_daemon.pid", "w");
	if (pid_fp) {
		fprintf(pid_fp, "%d\n", getpid());
		fclose(pid_fp);
	}
}

static void print_usage(void) {
	printf("Usage: hosted_daemon [-f] [-c config_file]\n");
	printf("  -f            Run in foreground (don't daemonize)\n");
	printf("  -c <file>    Use alternate config file\n");
}

static int ensure_single_instance(void)
{
	struct flock fl = {
		.l_type = F_WRLCK,
		.l_whence = SEEK_SET,
		.l_start = 0,
		.l_len = 0
	};

	lock_fd = open(LOCK_FILE, O_RDWR | O_CREAT, 0640);
	if (lock_fd < 0) {
		LOG_MSG(LOG_ERR, "Cannot open lock file: %s", strerror(errno));
		return FAILURE;
	}

	if (fcntl(lock_fd, F_SETLK, &fl) < 0) {
		if (errno == EACCES || errno == EAGAIN) {
			LOG_MSG(LOG_ERR, "Another instance is already running");
		} else {
			LOG_MSG(LOG_ERR, "Cannot lock file: %s", strerror(errno));
		}
		close(lock_fd);
		return FAILURE;
	}

	return SUCCESS;
}

int main(int argc, char *argv[])
{
	if(getuid()) {
		printf("Please re-run program with superuser access. Exiting\n");
		return FAILURE;
	}

	/* Parse arguments first */
	if (argc > 1) {
		if (strcmp(argv[1], "-f") == 0) {
			run_in_foreground = 1;
		} else if (strcmp(argv[1], "-c") == 0 && argc > 2) {
			config_file = argv[2];
		} else {
			print_usage();
			return FAILURE;
		}
	}

	/* Initialize logging before any LOG_MSG calls */
	if (!run_in_foreground) {
		openlog("hosted_daemon", LOG_PID, LOG_DAEMON);
	}

	/* Now we can use LOG_MSG */
	if(getuid()) {
		LOG_MSG(LOG_ERR, "Please re-run program with superuser access");
		if (!run_in_foreground) {
			closelog();
		}
		return FAILURE;
	}

	/* Check for another instance before daemonizing */
	if (ensure_single_instance() != SUCCESS) {
		if (!run_in_foreground) {
			closelog();
		}
		return FAILURE;
	}

	if (!run_in_foreground) {
		/* Daemonize the process */
		daemonize();
		/* Note: daemonize() will reopen syslog */
	}

	/* Register Sig handler */
	signal(SIGHUP, sig_handler);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	signal(SIGUSR1, sig_handler);

	LOG_MSG(LOG_INFO, "Waiting for slave RPC");

	update_host_network_port_range(49152, 61439);

	while (1) {

		if (init_app()) {
			sleep(1);
			continue;
		}

		if (!local_network_up) {

			/* fetch MAC address */
			fetch_mac_addr_from_slave();

			if (strlen(sta_network.mac_addr)==0) {
				LOG_MSG(LOG_ERR, "Failed to retrieve the MAC address");
				sleep(1);
				continue;
			}

			fetch_ip_addr_from_slave();

			if (sta_network.dns_valid && sta_network.ip_valid) {
				LOG_MSG(LOG_INFO, "Network identified as up");
				up_sta_netdev__with_static_ip_dns_route(&sta_network);
				add_dns(sta_network.dns_addr);
				local_network_up = 1;
			} else {
				LOG_MSG(LOG_INFO, "Network identified as down");
				down_sta_netdev(&sta_network);
				remove_dns(sta_network.dns_addr);
				local_network_up = 0;
			}
		}

		sleep(1);
	}

	cleanup_app();
	LOG_MSG(LOG_INFO, "Exiting..");
	if (!run_in_foreground) {
		closelog();
	}
	return 0;
}
