/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Sample program to send a packet over UDP, TCP */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <errno.h>

/* ── UDP ──────────────────────────────────────────────────────────────────── */
static int send_udp(const char *ip, int port)
{
    const char *msg = "Hello, UDP!";

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return EXIT_FAILURE; }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(port),
    };
    inet_pton(AF_INET, ip, &addr.sin_addr);

    ssize_t sent = sendto(sock, msg, strlen(msg), 0,
                          (struct sockaddr *)&addr, sizeof(addr));
    if (sent < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN)
            fprintf(stderr, "sendto would block, try again later\n");
        else
            perror("sendto");
        close(sock);
        return EXIT_FAILURE;
    }
    printf("Sent %zd bytes to %s:%d (UDP)\n", sent, ip, port);
    close(sock);
    return EXIT_SUCCESS;
}

/* ── TCP ──────────────────────────────────────────────────────────────────── */
static int send_tcp(const char *ip, int port)
{
    const char *msg = "Hello, TCP!";

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("socket"); return EXIT_FAILURE; }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(port),
    };
    inet_pton(AF_INET, ip, &addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return EXIT_FAILURE;
    }

    ssize_t sent = send(sock, msg, strlen(msg), 0);
    if (sent < 0) {
        perror("send");
        close(sock);
        return EXIT_FAILURE;
    }
    printf("Sent %zd bytes to %s:%d (TCP)\n", sent, ip, port);
    close(sock);
    return EXIT_SUCCESS;
}

/* ── main ─────────────────────────────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <ip> <port> <udp|tcp>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *ip       = argv[1];
    int         port     = atoi(argv[2]);
    const char *protocol = argv[3];

    if (strcmp(protocol, "udp") == 0)
        return send_udp(ip, port);

    if (strcmp(protocol, "tcp") == 0)
        return send_tcp(ip, port);

    fprintf(stderr, "Unknown protocol '%s'. Use udp or tcp.\n", protocol);
    return EXIT_FAILURE;
}
