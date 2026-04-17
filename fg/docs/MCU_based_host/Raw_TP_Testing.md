# Raw Throughput testing

- This is the optional feature to test throughput over required transport layer (SPI or SDIO).
- When ENABLED, this will bypass Wi-Fi traffic and push the dummy data traffic on transport layer directly to check maximum throughput on the current transport.
- This feature will also help you to check if transport layer is properly configured or not.
- This requires ESP firmware to be manually flashed with below steps.

## Steps to test Raw TP

- On Host side:
    1. Go to `esp_hosted_fg/host/stm32/common/stats.h`
	2. Enable raw throughput tetsing feature by making `TEST_RAW_TP` value to `1`.
	3. There are two directions to test raw throughput and at a time, throughput can be tested only in one direction (simplex).
	    - ESP to Host : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 1
	    - Host to ESP : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 0
	4. Build and flash ESP firmware again.

- On ESP side:
	1. Go to `esp_hosted_fg/esp/esp_driver/network_adapter/main/stats.h`
	2. Enable raw throughput tetsing feature by making `TEST_RAW_TP` value to `1`.
	3. There are two directions to test raw throughput and at a time, throughput can be tested only in one direction (simplex).
	    - ESP to Host : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 1
	    - Host to ESP : For this, make `TEST_RAW_TP__ESP_TO_HOST` value to 0
	4. Build and flash ESP firmware again.

**Note**
Please revert these configurations once raw throughput testing is done

## Raw Throughput Numbers

| ESP32 Board | STM32 Board  | Transport  | TX case | RX case |
|:---------:|:------:|:----------:|:---:|:--:|
| ESP32 | STM32F412ZGT6-Nucleo 144  | SDIO | 5.22 Mbps | 5.32 Mbps |
| ESP32/ESP32S2/ESP32S3/ESP32C2/ESP32C3 | STM32F469I-Discovery  | SPI | 2.88 Mbps | 2.28 Mbps |