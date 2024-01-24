# Raw Throughput testing

- This feature provides functionality to test throughput over required transport layer (SPI or SDIO).
- When triggered by host, this will bypass Wi-Fi traffic and push the dummy data traffic on transport layer directly to check maximum throughput on the current transport.
- This feature will also help you to check if transport layer is properly configured or not.

## Steps to test Raw TP

- Raw throughput feature is enabled by default for ESP firmware
- On Host side:
    1. While setting up the host, pass `rawtp_host_to_esp` or `rawtp_esp_to_host` as a argument to `rpi_init.sh`.
        - e.g if you are setting Wi-Fi over SDIO and you want to test raw TP from ESP to host over SDIO interface, then compile and load host driver as below:
        ```sh
        $ cd esp_hosted_ng/host/
        $ ./rpi_init.sh sdio rawtp_esp_to_host
        ```
    2. RAW throught is enabled by default for host, to disable it set the value of `TEST_RAW_TP` to 0 in `esp_hosted/esp_hosted_ng/host/include/stats.h`


