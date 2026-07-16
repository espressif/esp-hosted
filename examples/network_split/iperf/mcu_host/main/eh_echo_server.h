/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <stdint.h>

/* Start a TCP echo server task bound to `port` on the host stack. Used to verify
 * network-split port routing: a client to a host-range port must reach this
 * server (host stack), while a CP-local port is handled by the coprocessor. */
void eh_echo_server_start(uint16_t port);
