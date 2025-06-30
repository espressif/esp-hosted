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
        $ ./rpi_init.sh wifi=sdio rawtp
        ```

- On ESP side:
	1. Go to `esp_hosted_fg/esp/esp_driver/network_adapter/main/stats.h`
	2. Enable raw throughput tetsing feature by making `TEST_RAW_TP` value to `1`.
	3. There are two directions to test raw throughput and at a time, throughput can be tested only in one direction (simplex).
	    - ESP to Host : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 1
	    - Host to ESP : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 0
	4. Build and flash ESP firmware again.

> [!Note]
> Please revert these configurations once raw throughput testing is done

## Raw Throughput Benchmarks

### For SDIO with ESP32-C6 or C5

Before starting the test, check your SDMMC CLK frequency and bus width. On a Raspberry Pi, this is done via `/sys/kernel/debug/mmc1/ios`:

```sh
$ sudo cat /sys/kernel/debug/mmc1/ios
clock            50000000 Hz
actual clock:    41666667 Hz
[...]
bus width:       2 (4 bits)
```

This shows the SDMMC clock is running at 41 MHz and SDIO is using 4 data lines. For other linux based SOCs, check your documentation on how to display similar SDMMC info.

To change the clock frequency, add the `clockspeed` parameter when running `rpi_init.sh`:

```sh
$ ./rpi_init.sh wifi=sdio clockspeed=50 rawtp
```

With the SDMMC clock at 41 MHz, here are the raw throughput numbers:

| Data Transfer | Throughput (Mbits/s) |
| ------------- | -------------------: |
| C6/C5 to RPi  | 40 |
| RPi to C6/C5  | 60 |
