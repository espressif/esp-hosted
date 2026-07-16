# Wi-Fi iperf (`wifi/iperf`)

Run the stock IDF `wifi/iperf` host example against an ESP-Hosted Wi-Fi coprocessor.

This example is kept intentionally close to upstream:
- `mcu_host/main/iperf_example_main.c` is copied from `~/esp-idf/examples/wifi/iperf/main/iperf_example_main.c`
- the host project keeps the upstream `wifi_cmd` / `iperf_cmd` console model
- the hosted-specific layer is limited to dependency wiring and host sdkconfig defaults

For the coprocessor side, pair this host example with [`../sta/cp`](../sta/cp), which already provides the hosted Wi-Fi CP firmware for SDIO.

For MCU-host SDIO runs, build that CP example with the `STREAM` overlay:

```bash
cd ../sta/cp
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c5;sdkconfig.ci.sdio_mcu_stream" flash monitor
```

For Linux-host SDIO runs, use the Linux SW_AGGR overlay instead:

```bash
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c5;sdkconfig.ci.sdio_linux_sw_aggr" flash monitor
```
