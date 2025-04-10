/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* esp_check.h was added in idf v4.4 */
#include "iperf_esp_check.h"
#include "esp_log.h"
#include "iperf.h"
#include "esp_attr.h"

#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
#include "esp_rom_sys.h"
#endif


#ifdef CONFIG_FREERTOS_NUMBER_OF_CORES
/* new in idf v5.3 */
#define NUMBER_OF_CORES CONFIG_FREERTOS_NUMBER_OF_CORES
#else
#define NUMBER_OF_CORES portNUM_PROCESSORS
#endif

#define TAG "iperf"

typedef struct {
    iperf_cfg_t cfg;
    bool finish;
    uint32_t actual_len;
    uint32_t buffer_len;
    uint8_t *buffer;
    uint32_t sockfd;
} iperf_ctrl_t;

bool g_iperf_is_running = false;
DRAM_ATTR static iperf_ctrl_t s_iperf_ctrl;
iperf_hook_func_t iperf_hook_func = NULL;

inline static bool iperf_is_udp_client(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_CLIENT) && (s_iperf_ctrl.cfg.flag & IPERF_FLAG_UDP));
}

inline static bool iperf_is_udp_server(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_SERVER) && (s_iperf_ctrl.cfg.flag & IPERF_FLAG_UDP));
}

inline static bool iperf_is_tcp_client(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_CLIENT) && (s_iperf_ctrl.cfg.flag & IPERF_FLAG_TCP));
}

inline static bool iperf_is_tcp_server(void)
{
    return ((s_iperf_ctrl.cfg.flag & IPERF_FLAG_SERVER) && (s_iperf_ctrl.cfg.flag & IPERF_FLAG_TCP));
}

inline static int iperf_get_socket_error_code(int sockfd)
{
    return errno;
}

static int iperf_show_socket_error_reason(const char *str, int sockfd)
{
    int err = errno;
    if (err != 0) {
        ESP_LOGW(TAG, "%s error, error code: %d, reason: %s", str, err, strerror(err));
    }

    return err;
}

static void iperf_report_task(void *arg)
{
    uint32_t interval = s_iperf_ctrl.cfg.interval;
    uint32_t time = s_iperf_ctrl.cfg.time;
    TickType_t delay_interval = (interval * 1000) / portTICK_PERIOD_MS;
    uint32_t cur = 0;
    double average = 0;
    double actual_bandwidth = 0;
    int k = 1;
    char format_ch = 'M';

    /* NOTE: Output is not totally same with linux iperf */
    printf("\nInterval       Bandwidth\n");
    while (!s_iperf_ctrl.finish) {
        vTaskDelay(delay_interval);
        switch (s_iperf_ctrl.cfg.format) {
        case KBITS_PER_SEC:
            actual_bandwidth = (s_iperf_ctrl.actual_len / 1024.0 * 8) / interval;
            format_ch = 'K';
            break;
        case MBITS_PER_SEC:
            /* pass through */
        default:
            actual_bandwidth = (s_iperf_ctrl.actual_len / 1024.0 / 1024.0 * 8) / interval;
            break;
        }
        printf("%2d.0-%2d.0 sec  %.2f %cbits/sec\n", cur, cur + interval, actual_bandwidth, format_ch);
        cur += interval;
        average = ((average * (k - 1) / k) + (actual_bandwidth / k));
        k++;
        s_iperf_ctrl.actual_len = 0;
        if (cur >= time) {
            printf("%2d.0-%2d.0 sec  %.2f %cbits/sec\n", 0, time, average, format_ch);
            break;
        }
    }

    s_iperf_ctrl.finish = true;
    vTaskDelete(NULL);
}

static esp_err_t iperf_start_report(void)
{
    int ret;

    ret = xTaskCreatePinnedToCore(iperf_report_task, IPERF_REPORT_TASK_NAME, IPERF_REPORT_TASK_STACK, NULL, IPERF_REPORT_TASK_PRIORITY, NULL, NUMBER_OF_CORES - 1);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", IPERF_REPORT_TASK_NAME);
        return ESP_FAIL;
    }

    return ESP_OK;
}

IRAM_ATTR static void socket_recv(int recv_socket, struct sockaddr_storage listen_addr, uint8_t type)
{
    bool iperf_recv_start = true;
    uint8_t *buffer;
    int want_recv = 0;
    int actual_recv = 0;

#if IPERF_IPV6_ENABLED && IPERF_IPV4_ENABLED
    socklen_t socklen = (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
#elif IPERF_IPV6_ENABLED
    socklen_t socklen = sizeof(struct sockaddr_in6);
#else
    socklen_t socklen = sizeof(struct sockaddr_in);
#endif
    const char *error_log = (type == IPERF_TRANS_TYPE_TCP) ? "tcp server recv" : "udp server recv";

    buffer = s_iperf_ctrl.buffer;
    want_recv = s_iperf_ctrl.buffer_len;
    while (!s_iperf_ctrl.finish) {
        actual_recv = recvfrom(recv_socket, buffer, want_recv, 0, (struct sockaddr *)&listen_addr, &socklen);
        if (actual_recv < 0) {
            iperf_show_socket_error_reason(error_log, recv_socket);
            s_iperf_ctrl.finish = true;
            break;
        } else {
            if (iperf_recv_start) {
                iperf_start_report();
                iperf_recv_start = false;
            }
            s_iperf_ctrl.actual_len += actual_recv;
        }
    }
}

IRAM_ATTR static void socket_send(int send_socket, struct sockaddr_storage dest_addr, uint8_t type, int bw_lim)
{
    uint8_t *buffer;
    uint32_t *pkt_id_p;
    uint32_t pkt_cnt = 0;
    int actual_send = 0;
    int want_send = 0;
    int period_us = -1;
    int delay_us = 0;
    int64_t prev_time = 0;
    int64_t send_time = 0;
    int err = 0;
    struct timeval ts_now;

#if IPERF_IPV6_ENABLED && IPERF_IPV4_ENABLED
    const socklen_t socklen = (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) ? sizeof(struct sockaddr_in6) : sizeof(struct sockaddr_in);
#elif IPERF_IPV6_ENABLED
    const socklen_t socklen = sizeof(struct sockaddr_in6);
#else
    const socklen_t socklen = sizeof(struct sockaddr_in);
#endif
    const char *error_log = (type == IPERF_TRANS_TYPE_TCP) ? "tcp client send" : "udp client send";

    buffer = s_iperf_ctrl.buffer;
    pkt_id_p = (uint32_t *)s_iperf_ctrl.buffer;
    want_send = s_iperf_ctrl.buffer_len;
    iperf_start_report();

    if (bw_lim > 0) {
        period_us = want_send * 8 / bw_lim;
    }

    while (!s_iperf_ctrl.finish) {
        if (period_us > 0) {
            gettimeofday(&ts_now, NULL);
            send_time = ts_now.tv_sec * 1000000L + ts_now.tv_usec;
            if (actual_send > 0) {
                // Last packet "send" was successful, check how much off the previous loop duration was to the ideal send period. Result will adjust the
                // next send delay.
                delay_us += period_us + (int32_t)(prev_time - send_time);
            } else {
                // Last packet "send" was not successful. Ideally we should try to catch up the whole previous loop duration (e.g. prev_time - send_time).
                // However, that's not possible since the most probable reason why the send was unsuccessful is the HW was not able to process the packet.
                // Hence, we cannot queue more packets with shorter (or no) delay to catch up since we are already at the performance edge. The best we
                // can do is to reset the send delay (which is probably big negative number) and start all over again.
                delay_us = 0;
            }
            prev_time = send_time;
        }
        *pkt_id_p = htonl(pkt_cnt++); // datagrams need to be sequentially numbered
        actual_send = sendto(send_socket, buffer, want_send, 0, (struct sockaddr *)&dest_addr, socklen);
        if (actual_send != want_send) {
            if (type == IPERF_TRANS_TYPE_UDP) {
                err = iperf_get_socket_error_code(send_socket);
                // ENOMEM & ENOBUFS is expected under heavy load => do not print it
                if ((err != ENOMEM) && (err != ENOBUFS)) {
                    iperf_show_socket_error_reason(error_log, send_socket);
                } else if (err == EAGAIN) {
                    vTaskDelay(1);
                }
            } else if (type == IPERF_TRANS_TYPE_TCP) {
                iperf_show_socket_error_reason(error_log, send_socket);
                break;
            }
        } else {
            s_iperf_ctrl.actual_len += actual_send;
        }
        // The send delay may be negative, it indicates we are trying to catch up and hence to not delay the loop at all.
        if (delay_us > 0) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
            esp_rom_delay_us(delay_us);
        static int send_count = 0;
        if (++send_count % 10 == 0) {
            vTaskDelay(1);
        }
#else
            ets_delay_us(delay_us);
#endif
        }
    }
}

static int tcp_listen_socket = -1;
static esp_err_t iperf_run_tcp_server(void)
{
    int client_socket = -1;
    int opt = 1;
    int err = 0;
    esp_err_t ret = ESP_OK;
    struct timeval timeout = { 0 };
    socklen_t addr_len = sizeof(struct sockaddr);
    struct sockaddr_storage listen_addr = { 0 };
#if IPERF_IPV4_ENABLED
    struct sockaddr_in listen_addr4 = { 0 };
    struct sockaddr_in remote_addr = { 0 };
#endif
#if IPERF_IPV6_ENABLED
    struct sockaddr_in6 listen_addr6 = { 0 };
    struct sockaddr_in6 remote_addr6 = { 0 };
#endif
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
#if IPERF_IPV6_ENABLED
        // The TCP server listen at the address "::", which means all addresses can be listened to.
        inet6_aton("::", &listen_addr6.sin6_addr);
        listen_addr6.sin6_family = AF_INET6;
        listen_addr6.sin6_port = htons(s_iperf_ctrl.cfg.sport);

        tcp_listen_socket = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
        ESP_GOTO_ON_FALSE((tcp_listen_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);

        setsockopt(tcp_listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(tcp_listen_socket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));

        ESP_LOGI(TAG, "Socket created");

        err = bind(tcp_listen_socket, (struct sockaddr *)&listen_addr6, sizeof(listen_addr6));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d, IPPROTO: %d", errno, AF_INET6);
        err = listen(tcp_listen_socket, 1);
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Error occurred during listen: errno %d", errno);

        memcpy(&listen_addr, &listen_addr6, sizeof(listen_addr6));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4) {
#if IPERF_IPV4_ENABLED
        listen_addr4.sin_family = AF_INET;
        listen_addr4.sin_port = htons(s_iperf_ctrl.cfg.sport);
        listen_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.source_ip4;

        tcp_listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        ESP_GOTO_ON_FALSE((tcp_listen_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);

        setsockopt(tcp_listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        ESP_LOGI(TAG, "Socket created");

        err = bind(tcp_listen_socket, (struct sockaddr *)&listen_addr4, sizeof(listen_addr4));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d, IPPROTO: %d", errno, AF_INET);

        err = listen(tcp_listen_socket, 5);
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Error occurred during listen: errno %d", errno);
        memcpy(&listen_addr, &listen_addr4, sizeof(listen_addr4));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    }
    timeout.tv_sec = IPERF_SOCKET_RX_TIMEOUT;
    setsockopt(tcp_listen_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
#if IPERF_IPV6_ENABLED
        client_socket = accept(tcp_listen_socket, (struct sockaddr *)&remote_addr6, &addr_len);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to accept connection: errno %d", errno);
        ESP_LOGI(TAG, "accept: %s,%d", inet6_ntoa(remote_addr6.sin6_addr), htons(remote_addr6.sin6_port));
#endif
    } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4) {
#if IPERF_IPV4_ENABLED
        client_socket = accept(tcp_listen_socket, (struct sockaddr *)&remote_addr, &addr_len);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to accept connection: errno %d", errno);
        ESP_LOGI(TAG, "accept: %s,%d", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));
#endif
    }

    timeout.tv_sec = IPERF_SOCKET_RX_TIMEOUT;
    setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (iperf_hook_func) {
        iperf_hook_func(IPERF_TCP_SERVER, IPERF_STARTED);
    }
    socket_recv(client_socket, listen_addr, IPERF_TRANS_TYPE_TCP);

exit:
    if (client_socket != -1) {
        close(client_socket);
        ESP_LOGI(TAG, "TCP Socket server is closed.");
    }

    if (tcp_listen_socket != -1) {
        shutdown(tcp_listen_socket, 0);
        close(tcp_listen_socket);
        ESP_LOGD(TAG, "TCP listen socket is closed.");
    }
    if (iperf_hook_func) {
        iperf_hook_func(IPERF_TCP_SERVER, IPERF_STOPPED);
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static esp_err_t iperf_run_tcp_client(void)
{
    int client_socket = -1;
    int err = 0;
    esp_err_t ret = ESP_OK;
    struct timeval timeout = { 0 };
    struct sockaddr_storage dest_addr = { 0 };
#if IPERF_IPV4_ENABLED
    struct sockaddr_in dest_addr4 = { 0 };
#endif
#if IPERF_IPV6_ENABLED
    struct sockaddr_in6 dest_addr6 = { 0 };
#endif
    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
#if IPERF_IPV6_ENABLED
        client_socket = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);

        inet6_aton(s_iperf_ctrl.cfg.destination_ip6, &dest_addr6.sin6_addr);
        dest_addr6.sin6_family = AF_INET6;
        dest_addr6.sin6_port = htons(s_iperf_ctrl.cfg.dport);

        err = connect(client_socket, (struct sockaddr *)&dest_addr6, sizeof(struct sockaddr_in6));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to connect: errno %d", errno);
        ESP_LOGI(TAG, "Successfully connected");
        memcpy(&dest_addr, &dest_addr6, sizeof(dest_addr6));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4) {
#if IPERF_IPV4_ENABLED
        client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);

        dest_addr4.sin_family = AF_INET;
        dest_addr4.sin_port = htons(s_iperf_ctrl.cfg.dport);
        dest_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.destination_ip4;
        err = connect(client_socket, (struct sockaddr *)&dest_addr4, sizeof(struct sockaddr_in));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to connect: errno %d", errno);
        ESP_LOGI(TAG, "Successfully connected");
        memcpy(&dest_addr, &dest_addr4, sizeof(dest_addr4));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    }
    timeout.tv_sec = IPERF_SOCKET_TCP_TX_TIMEOUT;
    setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (iperf_hook_func) {
        iperf_hook_func(IPERF_TCP_CLIENT, IPERF_STARTED);
    }
    socket_send(client_socket, dest_addr, IPERF_TRANS_TYPE_TCP, s_iperf_ctrl.cfg.bw_lim);

exit:
    if (client_socket != -1) {
        shutdown(client_socket, 0);
        close(client_socket);
        ESP_LOGI(TAG, "TCP Socket client is closed.");
    }
    if (iperf_hook_func) {
        iperf_hook_func(IPERF_TCP_CLIENT, IPERF_STOPPED);
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static esp_err_t iperf_run_udp_server(void)
{
    int listen_socket = -1;
    int opt = 1;
    int err = 0;
    esp_err_t ret = ESP_OK;
    struct timeval timeout = { 0 };
    struct sockaddr_storage listen_addr = { 0 };
#if IPERF_IPV4_ENABLED
    struct sockaddr_in listen_addr4 = { 0 };
#endif
#if IPERF_IPV6_ENABLED
    struct sockaddr_in6 listen_addr6 = { 0 };
#endif

    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
#if IPERF_IPV6_ENABLED
        // The UDP server listen at the address "::", which means all addresses can be listened to.
        inet6_aton("::", &listen_addr6.sin6_addr);
        listen_addr6.sin6_family = AF_INET6;
        listen_addr6.sin6_port = htons(s_iperf_ctrl.cfg.sport);

        listen_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
        ESP_GOTO_ON_FALSE((listen_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);
        ESP_LOGI(TAG, "Socket created");

        setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        err = bind(listen_socket, (struct sockaddr *)&listen_addr6, sizeof(struct sockaddr_in6));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGI(TAG, "Socket bound, port %" PRIu16, listen_addr6.sin6_port);

        memcpy(&listen_addr, &listen_addr6, sizeof(listen_addr6));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4) {
#if IPERF_IPV4_ENABLED
        listen_addr4.sin_family = AF_INET;
        listen_addr4.sin_port = htons(s_iperf_ctrl.cfg.sport);
        listen_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.source_ip4;

        listen_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        ESP_GOTO_ON_FALSE((listen_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);
        ESP_LOGI(TAG, "Socket created");

        setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        err = bind(listen_socket, (struct sockaddr *)&listen_addr4, sizeof(struct sockaddr_in));
        ESP_GOTO_ON_FALSE((err == 0), ESP_FAIL, exit, TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGI(TAG, "Socket bound, port %d", listen_addr4.sin_port);
        memcpy(&listen_addr, &listen_addr4, sizeof(listen_addr4));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    }

    timeout.tv_sec = IPERF_SOCKET_RX_TIMEOUT;
    setsockopt(listen_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (iperf_hook_func) {
        iperf_hook_func(IPERF_UDP_SERVER, IPERF_STARTED);
    }
    socket_recv(listen_socket, listen_addr, IPERF_TRANS_TYPE_UDP);

exit:
    if (listen_socket != -1) {
        shutdown(listen_socket, 0);
        close(listen_socket);
    }
    ESP_LOGI(TAG, "Udp socket server is closed.");
    if (iperf_hook_func) {
        iperf_hook_func(IPERF_UDP_SERVER, IPERF_STOPPED);
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static esp_err_t iperf_run_udp_client(void)
{
    int client_socket = -1;
    int opt = 1;
    esp_err_t ret = ESP_OK;
    struct sockaddr_storage dest_addr = { 0 };
#if IPERF_IPV4_ENABLED
    struct sockaddr_in dest_addr4 = { 0 };
#endif
#if IPERF_IPV6_ENABLED
    struct sockaddr_in6 dest_addr6 = { 0 };
#endif

    if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
#if IPERF_IPV6_ENABLED
        inet6_aton(s_iperf_ctrl.cfg.destination_ip6, &dest_addr6.sin6_addr);
        dest_addr6.sin6_family = AF_INET6;
        dest_addr6.sin6_port = htons(s_iperf_ctrl.cfg.dport);

        client_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_IPV6);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", s_iperf_ctrl.cfg.destination_ip6, s_iperf_ctrl.cfg.dport);

        setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        memcpy(&dest_addr, &dest_addr6, sizeof(dest_addr6));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV4) {
#if IPERF_IPV4_ENABLED
        dest_addr4.sin_family = AF_INET;
        dest_addr4.sin_port = htons(s_iperf_ctrl.cfg.dport);
        dest_addr4.sin_addr.s_addr = s_iperf_ctrl.cfg.destination_ip4;

        client_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        ESP_GOTO_ON_FALSE((client_socket >= 0), ESP_FAIL, exit, TAG, "Unable to create socket: errno %d", errno);
        ESP_LOGI(TAG, "Socket created, sending to %d:%d", s_iperf_ctrl.cfg.destination_ip4, s_iperf_ctrl.cfg.dport);

        setsockopt(client_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        memcpy(&dest_addr, &dest_addr4, sizeof(dest_addr4));
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, exit, TAG, "Invalid iperf address type!");
#endif
    }

    if (iperf_hook_func) {
        iperf_hook_func(IPERF_UDP_CLIENT, IPERF_STARTED);
    }
    socket_send(client_socket, dest_addr, IPERF_TRANS_TYPE_UDP, s_iperf_ctrl.cfg.bw_lim);

exit:
    if (client_socket != -1) {
        shutdown(client_socket, 0);
        close(client_socket);
    }
    ESP_LOGI(TAG, "UDP Socket client is closed");
    if (iperf_hook_func) {
        iperf_hook_func(IPERF_UDP_CLIENT, IPERF_STOPPED);
    }
    s_iperf_ctrl.finish = true;
    return ret;
}

static void iperf_task_traffic(void *arg)
{
    if (iperf_is_udp_client()) {
        iperf_run_udp_client();
    } else if (iperf_is_udp_server()) {
        iperf_run_udp_server();
    } else if (iperf_is_tcp_client()) {
        iperf_run_tcp_client();
    } else {
        iperf_run_tcp_server();
    }

    if (s_iperf_ctrl.buffer) {
        free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
    }
    ESP_LOGI(TAG, "iperf exit");
    g_iperf_is_running = false;
    vTaskDelete(NULL);
}

static uint32_t iperf_get_buffer_len(void)
{
    if (iperf_is_udp_client()) {
#if IPERF_IPV6_ENABLED
        if (s_iperf_ctrl.cfg.len_send_buf) {
            return s_iperf_ctrl.cfg.len_send_buf;
        } else if (s_iperf_ctrl.cfg.type == IPERF_IP_TYPE_IPV6) {
            return IPERF_DEFAULT_IPV6_UDP_TX_LEN;
        } else {
            return IPERF_DEFAULT_IPV4_UDP_TX_LEN;
        }
#else
        return (s_iperf_ctrl.cfg.len_send_buf == 0 ? IPERF_DEFAULT_IPV4_UDP_TX_LEN : s_iperf_ctrl.cfg.len_send_buf);
#endif
    } else if (iperf_is_udp_server()) {
        return IPERF_DEFAULT_UDP_RX_LEN;
    } else if (iperf_is_tcp_client()) {
        return (s_iperf_ctrl.cfg.len_send_buf == 0 ? IPERF_DEFAULT_TCP_TX_LEN : s_iperf_ctrl.cfg.len_send_buf);
    } else {
        return IPERF_DEFAULT_TCP_RX_LEN;
    }
    return 0;

}

esp_err_t iperf_start(iperf_cfg_t *cfg)
{
    BaseType_t ret;

    if (!cfg) {
        return ESP_FAIL;
    }

    if (g_iperf_is_running) {
        ESP_LOGW(TAG, "iperf is running");
        return ESP_FAIL;
    }

    memset(&s_iperf_ctrl, 0, sizeof(s_iperf_ctrl));
    memcpy(&s_iperf_ctrl.cfg, cfg, sizeof(*cfg));
    g_iperf_is_running = true;
    s_iperf_ctrl.finish = false;
    s_iperf_ctrl.buffer_len = iperf_get_buffer_len();
    s_iperf_ctrl.buffer = (uint8_t *)malloc(s_iperf_ctrl.buffer_len);
    if (!s_iperf_ctrl.buffer) {
        ESP_LOGE(TAG, "create buffer: not enough memory");
        return ESP_FAIL;
    }
    memset(s_iperf_ctrl.buffer, 0, s_iperf_ctrl.buffer_len);
    ret = xTaskCreatePinnedToCore(iperf_task_traffic, IPERF_TRAFFIC_TASK_NAME, IPERF_TRAFFIC_TASK_STACK, NULL, IPERF_TRAFFIC_TASK_PRIORITY, NULL, NUMBER_OF_CORES - 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "create task %s failed", IPERF_TRAFFIC_TASK_NAME);
        free(s_iperf_ctrl.buffer);
        s_iperf_ctrl.buffer = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t iperf_stop(void)
{
    /* close listen socket to stop tcp server */
    if (tcp_listen_socket != -1) {
        shutdown(tcp_listen_socket, 0);
        close(tcp_listen_socket);
        ESP_LOGD(TAG, "TCP listen socket is closed.");
    }
    if (g_iperf_is_running) {
        s_iperf_ctrl.finish = true;
    }

    for (int i = 0; i < 10; i ++) {
        if (!g_iperf_is_running) {
            break;
        }
        ESP_LOGI(TAG, "wait current iperf to stop ...");
        vTaskDelay(400 / portTICK_PERIOD_MS);
    }

    if (g_iperf_is_running) {
        ESP_LOGE(TAG, "DONE.IPERF_STOP,FAIL.");
    } else {
        ESP_LOGI(TAG, "DONE.IPERF_STOP,OK.");
    }
    return ESP_OK;
}

void iperf_register_hook_func(iperf_hook_func_t func)
{
    iperf_hook_func = func;
}
