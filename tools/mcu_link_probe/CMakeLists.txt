# SPDX-License-Identifier: Apache-2.0
#
# system/mcu_host — minimal MCU-host end-to-end link probe.
#
# First IDF example with CONFIG_ESP_HOSTED_HOST=y.  Purpose: prove the
# host stack (port-type impl + feature layer + RPC + transport) all
# compile and link under ESP-IDF on a real target.  No hardware CP
# expected — this is a link-only gate; runtime init returns an error
# on the transport step without a slave, which the example logs and
# continues.
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(eh_host_mcu_system)
