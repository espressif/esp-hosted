# SPDX-License-Identifier: Apache-2.0
# Strong-body implementor for the esp_wifi_remote_* hook contract.
# Archive gated on FEAT_WIFI_READY at host/CMakeLists.txt.

add_library(esp_wifi_remote_glue STATIC
    src/esp_wifi_remote_glue.c
    src/esp_wifi_remote_extended.c
    src/esp_wifi_he_with_hosted.c
)

if(CONFIG_ESP_HOSTED_HOST_TYPE_MCU)
    target_link_libraries(esp_wifi_remote_glue PUBLIC eh_host_mcu_transport)
endif()

# EAP/DPP bridge — compile when at least one underlying feature is enabled.
if(CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_ENT_READY OR
   CONFIG_ESP_HOSTED_HOST_FEAT_WIFI_EXT_DPP_READY)
    target_sources(esp_wifi_remote_glue PRIVATE src/esp_eap_dpp_with_hosted.c)
endif()

target_link_libraries(esp_wifi_remote_glue
    PUBLIC
        eh_host_feat_system
        eh_host_feat_wifi
    PRIVATE
        eh_host_config
)

target_link_options(esp_wifi_remote_glue INTERFACE
    "LINKER:--undefined=eh_wifi_remote_glue_anchor"
    "LINKER:--undefined=eh_wifi_remote_extended_anchor"
    "LINKER:--undefined=eh_wifi_he_with_hosted_anchor"
)

foreach(_feat wifi_ext_itwt wifi_ext_ent wifi_ext_dpp)
    if(TARGET eh_host_feat_${_feat})
        target_link_libraries(esp_wifi_remote_glue PUBLIC eh_host_feat_${_feat})
    endif()
endforeach()

# Non-IDF: link the vendored esp_wifi_remote so its include propagates.
# Forward reference resolves at generate-time.
if(NOT ESP_PLATFORM)
    target_link_libraries(esp_wifi_remote_glue PRIVATE esp_wifi_remote)
endif()
