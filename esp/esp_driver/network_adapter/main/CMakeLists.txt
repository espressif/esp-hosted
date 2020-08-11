set(COMPONENT_SRCS "sdio_slave_api.c" "slave_commands.c" "esp_hosted_config.pb-c.c" "protocomm_pserial.c" "app_main.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "../../../../common/include")

register_component()
