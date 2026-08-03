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

cmake_policy(SET CMP0007 NEW)  # keep empty list elements exact

# Message body (plain text; no ';' so it survives as a CMake list). The box
# width follows the longest line, so per-machine paths never overflow.
set(_lines
    "This ESP-IDF still has the 4092-byte SDIO send cap:"
    "  ${_sdio_slave_c}"
    "SDIO co-processor builds use SW aggregation (larger sends) for higher throughput."
    "Use ESP-IDF latest master, or patch your current ESP-IDF:"
    "  python ${EH_PY} patch-idf --idf-path ${IDF_PATH}"
    "(or ./install.sh)"
)

set(_w 0)
foreach(_l IN LISTS _lines)
    string(LENGTH "${_l}" _len)
    if(_len GREATER _w)
        set(_w ${_len})
    endif()
endforeach()

math(EXPR _bar_w "${_w} + 2")
string(REPEAT "─" ${_bar_w} _bar)
message(NOTICE "┌${_bar}┐")

list(LENGTH _lines _n)
math(EXPR _last "${_n} - 1")
foreach(_i RANGE ${_last})
    list(GET _lines ${_i} _l)
    string(LENGTH "${_l}" _len)
    math(EXPR _pad "${_w} - ${_len}")
    set(_sp "")
    if(_pad GREATER 0)
        string(REPEAT " " ${_pad} _sp)
    endif()
    # ANSI codes contain ';', so they can't live in a list — pick colour by index.
    if(_i EQUAL 0)
        message(NOTICE "│ ${_red}${_l}${_reset}${_sp} │")
    elseif(_i EQUAL 4)
        message(NOTICE "│ ${_green}${_l}${_reset}${_sp} │")
    else()
        message(NOTICE "│ ${_l}${_sp} │")
    endif()
endforeach()

message(NOTICE "└${_bar}┘")
message(FATAL_ERROR "SDIO SW_AGGR: this ESP-IDF lacks the send-cap fix (details above).")
