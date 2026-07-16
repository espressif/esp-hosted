/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal TCP echo server for the network-split routing demo/tests. Bound on the
 * HOST stack at a host port-range port; a client reaching it proves the 802.3
 * split delivered that port class to the host (CP-local ports go to the CP).
 * Disabled by default (CONFIG_EXAMPLE_NW_SPLIT_ECHO_SERVER).
 */
#include "eh_echo_server.h"

#include <errno.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char *TAG = "eh_echo_server";

static void echo_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;
    int lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (lsock < 0) {
        ESP_LOGE(TAG, "socket failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int on = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(port),
    };
    if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) != 0
            || listen(lsock, 2) != 0) {
        ESP_LOGE(TAG, "bind/listen port %u failed: errno %d", port, errno);
        close(lsock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "echo server listening on host port %u", port);
    for (;;) {
        int c = accept(lsock, NULL, NULL);
        if (c < 0) {
            continue;
        }
        ESP_LOGI(TAG, "echo client accepted");
        char buf[256];
        int n;
        while ((n = recv(c, buf, sizeof(buf), 0)) > 0) {
            int off = 0;
            while (off < n) {
                int w = send(c, buf + off, n - off, 0);
                if (w <= 0) {
                    break;
                }
                off += w;
            }
        }
        close(c);
    }
}

void eh_echo_server_start(uint16_t port)
{
    xTaskCreate(echo_task, "eh_echo", 4096, (void *)(uintptr_t)port, 5, NULL);
}
