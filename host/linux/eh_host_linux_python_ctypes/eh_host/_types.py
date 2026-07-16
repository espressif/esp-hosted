"""Common ctypes types + esp_err_t mapping.

The C surface uses `esp_err_t` (a signed int32 typedef) for
return codes — `0 == ESP_OK`, anything else is an error.
"""

from ctypes import c_int32

# Aliases — keeps per-feature modules readable.
EspErr = c_int32

ESP_OK = 0
