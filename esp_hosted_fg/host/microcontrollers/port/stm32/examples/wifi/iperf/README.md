| Supported Slaves | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-S2 | ESP32-S3 | ESP32-C6 |
| ---------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# iperf example on STM32

This example connects in station mode and allows to run iperf in between host and your laptop (any station connected to same AP that host is connected to).

## Configuration file
`main/station_example_main.c`

## Station mode configuration

* Change `CONFIG_ESP_WIFI_SSID` and `CONFIG_ESP_WIFI_PASSWORD` in `main/station_example_main.c` as per your AP/Wi-Fi router's SSID and Password.
* `ESP_WIFI_AUTH_MODE_THRESHOLD` is by default set to `WPA_WPA2_PSK`. Change as per your AP's configuration if need be.

## Iperf configuration

This Iperf config is for iperf instance running on the host.

* `IPERF_TEST_MODE` is default set to server mode i.e. `IPERF_FLAG_SERVER`. For acting as client, you may need to change it to `IPERF_FLAG_CLIENT`
* `IPERF_TEST_PROTOCOL` is default set to TCP i.e. `IPERF_TEST_MODE`. For UDP you can change to `IPERF_FLAG_UDP`
* If you setup iperf in client mode, please change `IPERF_TEST_CLIENT_DEST_IP_STR` to the iperf server's IP address.
* `main/iperf.h` can be referred to find possible values any configuration could take.
* interval, duration, port etc is configurable. please check all `IPERF_TEST_*` macros in `main/station_example_main.c` file

# Run the test
* Build and run the STM32 as C/C++ application on STM32CubeIDE
* In case you are using iperf as client, please make sure iperf server is already running and on the same network
* If the example doesn't seem to be running, please cross check with ping command from laptop, if the network is reachable
* If using in server mode, socket will timeout if no client connected. So make sure that client on laptop triggers request immediately once server starts.

# Optimise the SPI clock
* By default to maintain compatibility across all ESP32 chipsets, the SPI clock is set to 10Mbps. Following is the SPI slave clock for different ESP chipsets

| ESP chipset | Max SPI slave clock freq supported (Mhz) |
| :---------: | :--------------------------------------: |
| ESP32 | 10Mhz |
| ESP32-S2/S3/C2/C3 | 40MHz |
| Others | 26MHz |

To change the clock, open the ioc file in the project using STM32CubeIDE and go to 'Clock Configuration' tab to adjust APB2 clock prescaler.
Lower the prescaler, higher the frequency. Once prescaler changed, you can verify the frequency changed in 'Pinout & Configuration' Tab -> 'A->Z' -> 'SPI1' or 'SPI3' -> 'Parameter Settings' -> 'Baud rate'.
Try to match this baud rate as per ESP chipset's max frequency.
