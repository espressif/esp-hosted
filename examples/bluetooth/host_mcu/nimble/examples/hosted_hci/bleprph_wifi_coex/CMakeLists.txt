cmake_minimum_required(VERSION 3.16)

# Pick up the shared esp_hosted_hci_nimble bridge from ../../components.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../../components"
    "${CMAKE_CURRENT_LIST_DIR}/../../../../../../common_components/esp_hosted_examples_common"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bleprph_wifi_coex)
