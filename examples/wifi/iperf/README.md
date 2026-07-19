# Wi-Fi iperf (`wifi/iperf`)

Run the stock IDF `wifi/iperf` host example against an ESP-Hosted Wi-Fi coprocessor.

This example is kept intentionally close to upstream:
- `mcu_host/main/iperf_example_main.c` is copied from `~/esp-idf/examples/wifi/iperf/main/iperf_example_main.c`
- the host project keeps the upstream `wifi_cmd` / `iperf_cmd` console model
- the hosted-specific layer is limited to dependency wiring and host sdkconfig defaults

For the coprocessor side, pair this host example with [`../sta/cp`](../sta/cp), which provides the hosted Wi-Fi CP firmware. Select the SDIO transport and its datapath mode in menuconfig on the CP:

```bash
cd ../sta/cp
idf.py set-target esp32c5
# ESP-Hosted → Transport → SDIO, then "SDIO datapath mode":
#   MCU host   → STREAM    (CONFIG_EH_TRANSPORT_CP_SDIO_MODE_STREAM=y)
#   Linux host → SW_AGGR   (CONFIG_EH_TRANSPORT_CP_SDIO_MODE_SW_AGGR=y, the default)
idf.py menuconfig
idf.py flash monitor
```
