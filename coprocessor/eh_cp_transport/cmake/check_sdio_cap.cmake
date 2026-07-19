# Build-phase guard for SDIO SW_AGGR: fail if the resolved ESP-IDF still caps
# each SDIO slave send at 4092 bytes (SW aggregation sends larger aggregates).
# Invoked via `cmake -P` from a custom target so configure completes and the
# failure surfaces at build time. IDF_PATH / EH_PY are passed in with -D.
set(_sdio_slave_c "${IDF_PATH}/components/esp_driver_sdio/src/sdio_slave.c")
if(NOT EXISTS "${_sdio_slave_c}")
    return()
endif()

file(STRINGS "${_sdio_slave_c}" _cap_4092 REGEX "len <= 4092")
if(NOT _cap_4092)
    return()
endif()

string(ASCII 27 _esc)
set(_red   "${_esc}[1;31m")
set(_green "${_esc}[1;32m")
set(_reset "${_esc}[0m")

message(NOTICE "${_red}This ESP-IDF still has the 4092-byte SDIO send cap:${_reset}")
message(NOTICE "  ${_sdio_slave_c}")
message(NOTICE "SDIO co-processor builds use SW aggregation (larger sends) for higher throughput.")
message(NOTICE "Use ESP-IDF latest master, or patch your current ESP-IDF:")
message(NOTICE "${_green}  python ${EH_PY} patch-idf --idf-path ${IDF_PATH}${_reset}")
message(NOTICE "(or ./install.sh)")
message(FATAL_ERROR "SDIO SW_AGGR: this ESP-IDF lacks the send-cap fix (details above).")
