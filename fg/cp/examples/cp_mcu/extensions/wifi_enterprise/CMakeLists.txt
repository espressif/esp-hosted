# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS
	"$ENV{IDF_PATH}/examples/common_components/protocol_examples_common"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(eh_cp_mcu_wifi_enterprise)
