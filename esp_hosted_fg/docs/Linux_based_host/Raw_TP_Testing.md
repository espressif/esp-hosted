# Raw Throughput testing

- This is the optional feature to test throughput over required transport layer (SPI or SDIO).
- When ENABLED, this will bypass Wi-Fi traffic and push the dummy data traffic on transport layer directly to check maximum throughput on the current transport.
- This feature will also help you to check if transport layer is properly configured or not.
- This requires ESP firmware to be manually flashed with below steps.

## Steps to test Raw TP

- On Host side:
    1. While setting up the host, pass `rawtp` argument to `rpi_init.sh`.
        - e.g if you are setting Wi-Fi over SDIO and you want to test raw TP over SDIO interface, then compile and load host driver as below:
        ```sh
        $ cd esp_hosted_fg/host/linux/host_control/
        $ ./rpi_init.sh sdio rawtp
        ```

- On ESP side:
	1. Go to `esp_hosted_fg/esp/esp_driver/network_adapter/main/stats.h`
	2. Enable raw throughput tetsing feature by making `TEST_RAW_TP` value to `1`.
	3. There are two directions to test raw throughput and at a time, throughput can be tested only in one direction (simplex).
	    - ESP to Host : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 1
	    - Host to ESP : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 0
	4. Build and flash ESP firmware again.

**Note**
Please revert these configurations once raw throughput testing is done
