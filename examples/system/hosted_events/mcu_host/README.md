| Supported Hosts | ESP32 | ESP32-P Series | ESP32-H Series | ESP32-C Series | ESP32-S Series | Any other MCU hosts |
| --------------- | ----- | -------------- | -------------- | -------------- | -------------- | ------------------- |

| Supported Co-Processors | ESP32 | ESP32-C Series | ESP32-S Series |
| ----------------------- | ----- | -------------- | -------------- |

# ESP-Hosted Events Example

This host example uses ESP-Hosted events to detect that the co-processor has reset or hanged. It then takes action to reinitialised ESP-Hosted to restore connectivity with the co-processor.

This has been tested with the ESP32-C6 co-processor, flashed with the standard [ESP-Hosted slave example](https://components.espressif.com/components/espressif/esp_hosted/examples/slave) from the ESP Component Registry, or built from the `slave` directory from ESP-Hosted Github.

It makes use of ESP-Hosted events, with event base of `ESP_HOSTED_EVENT`:

- `ESP_HOSTED_EVENT_CP_INIT`: this event is sent by the co-processor after it has started and ESP-Hosted transport has been setup. Receiving this event for a second time indicates that the co-process has restarted. An `esp_reset_reason_t` value is provided so the host can discover why the co-processor sent the event
- `ESP_HOSTED_EVENT_CP_HEARTBEAT`: when configured by the host, the co-processor will periodically send a heartbeat event at the required interval (in seconds). If this event is not received after the interval, it indicates the the co-processor may have hanged and is unable to send the event.
- `ESP_HOSTED_EVENT_TRANSPORT_FAILURE`: indicates a ESP-Hosted transport failure

This example sets up the detection of ESP-Hosted events, then connects to a station. After connecting, if the co-processor reboots (INIT event received) or hangs (timeout getting HEARTBEAT), it will tell NETIF that the connection is lost, then proceed to reinitialise the ESP-Hosted transport.

## How to Use This Example on ESP Hosts

Before project configuration and build, be sure to set the correct
chip target using:

```bash
eh.py set-target <ESP Host chip_name>
```

### Configure the project

Open the project configuration menu:

```bash
eh.py menuconfig
```

In the `Example Configuration` menu:

* set `Heartbeat Interval in Seconds`. Default is 5 seconds
* toggle `Enable heartbeat timeout detection` to enable heartbeat timeout detection
* if heartbeat timeout detection is enabled, you can set `Heartbeat Timeout in Seconds`. This value should be larger than `Heartbeat Interval in Seconds`. A value should be set based on app requirements (how long to wait after a timeout before triggering a timeout alert).

Configure the Station parameters at needed:

- Set WiFi SSID.
- Set WiFi Password.

Optional: If you need, change the other Station options according to your requirements.

### Build and Flash

Run `eh.py -p HOST_PORT flash monitor` to build, flash and monitor the
project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full
steps to configure and use ESP-IDF to build projects.

## Example Output

> [!NOTE]
> To simulate a co-processor reset or hang, a ESP-Prog is connected to the ESP32-C6 on the ESP32-P4 Dev Board. The `RST` and `BOOT` buttons on the ESP-Prog is then used to reset or put the C6 into boot mode (to simulate a hang).

### Example Output using SDIO interface

```
I (6327) esp_netif_handlers: sta ip: 192.168.50.217, mask: 255.255.255.0, gw: 192.168.50.1
I (6327) wifi station: got ip:192.168.50.217
I (6327) wifi station: connected to ap SSID:xxx password:yyy
I (7377) hosted_events: *** Heartbeat 0***
W (13377) hosted_events: *** HEARTBEAT timeout ***
I (14327) hosted_events: heartbeat timeout, transport failure or got INIT event: reinit Hosted
E (14327) sdmmc_io: sdmmc_io_rw_extended: sdmmc_send_cmd returned 0x107
E (14327) H_SDIO_DRV: sdio_write_task: 0: Failed to send data: 263 54 54
E (14327) sdmmc_io: sdmmc_io_rw_extended: sdmmc_send_cmd returned 0x107
E (14337) H_SDIO_DRV: sdio_write_task: 1: Failed to send data: 263 54 54
E (14347) H_SDIO_DRV: Unrecoverable host sdio state
I (14347) wifi station: do not reconnect
I (14347) hosted_events: *** Transport Failure ***
I (14527) H_API: ESP-Hosted deinit

I (14527) serial_ll: Clean-up serial queue
W (14527) H_SDIO_DRV: free buffer[0] 0x4ff3d8c0
W (14527) H_SDIO_DRV: free buffer[1] 0x4ff3e1c0
I (14527) transport: TRANSPORT_INACTIVE
I (14527) hosted_events: restarting ESP-Hosted
I (14537) hosted_events: init ESP-Hosted
I (14537) H_API: ESP-Hosted starting. Hosted_Tasks: prio:23, stack: 5120 RPC_task_stack: 5120
W (14547) H_API: Transport already initialized, skipping initialization
I (14557) H_API: ** add_esp_wifi_remote_channels **
I (14557) H_SDIO_DRV: sdio_data_to_rx_buf_task started
I (14567) H_API: ESP-Hosted Try to communicate with ESP-Hosted slave

I (14567) transport: Attempt connection with slave: retry[0]
W (14577) H_SDIO_DRV: Reset slave using GPIO[54]
I (14577) os_wrapper_esp: GPIO [54] configured
I (16107) sdio_wrapper: SDIO master: Slot 1, Data-Lines: 4-bit Freq(KHz)[40000 KHz]
I (16107) sdio_wrapper: GPIOs: CLK[18] CMD[19] D0[14] D1[15] D2[16] D3[17] Slave_Reset[54]
I (16107) sdio_wrapper: Queues: Tx[20] Rx[20] SDIO-Rx-Mode[1]
I (16147) sdio_wrapper: Function 0 Blocksize: 512
I (16147) sdio_wrapper: Function 1 Blocksize: 512
I (16247) H_SDIO_DRV: Card init success, TRANSPORT_RX_ACTIVE
I (16247) transport: set_transport_state: 1
I (16247) transport: Waiting for esp_hosted slave to be ready
I (16267) H_SDIO_DRV: SDIO Host operating in STREAMING MODE
I (16267) H_SDIO_DRV: Open data path at slave
I (16267) H_SDIO_DRV: Starting SDIO process rx task
I (16287) H_SDIO_DRV: Received ESP_PRIV_IF type message
I (16287) transport: Received INIT event from ESP32 peripheral
I (16287) transport: EVENT: 12
I (16287) transport: Identified slave [esp32c6]
I (16297) transport: EVENT: 11
I (16297) transport: capabilities: 0xd
I (16307) transport: Features supported are:
I (16307) transport:     * WLAN
I (16307) transport:       - HCI over SDIO
I (16317) transport:       - BLE only
I (16317) transport: EVENT: 13
I (16317) transport: ESP board type is : 13

I (16327) transport: Base transport is set-up, TRANSPORT_TX_ACTIVE
I (16327) H_API: Transport active
I (16327) transport: Slave chip Id[12]
I (16337) transport: raw_tp_dir[-], flow_ctrl: low[60] high[80]
I (16337) transport: transport_delayed_init
I (16347) esp_cli: Remove any existing deep_sleep cmd in cli
I (16347) esp_cli: Registering command: crash
I (16357) esp_cli: Registering command: reboot
I (16357) esp_cli: Registering command: mem-dump
I (16367) esp_cli: Registering command: task-dump
I (16367) esp_cli: Registering command: cpu-dump
I (16367) esp_cli: Registering command: heap-trace
I (16377) esp_cli: Registering command: sock-dump
I (16377) esp_cli: Registering command: host-power-save
I (16387) hci_stub_drv: Host BT Support: Disabled
I (16387) H_SDIO_DRV: Received INIT event
I (16397) H_SDIO_DRV: Event type: 0x22
I (16397) H_SDIO_DRV: Write thread started
I (16447) hosted_events: getting fw version
I (16567) hosted_events: *** got INIT event from co-processor ***
I (16567) hosted_events: *** Co-processor Reset Reason 1***
I (16567) hosted_events: Expected INIT event
I (16567) hosted_events: FW Version: 2.8.1
I (16577) hosted_events: heartbeat timer started
I (16577) transport: Attempt connection with slave: retry[0]
I (16577) transport: Transport is already up
I (16877) wifi station: wifi_init_sta finished.
I (16877) RPC_WRAP: ESP Event: wifi station started
I (16887) RPC_WRAP: ESP Event: wifi station started
I (19327) RPC_WRAP: ESP Event: Station mode: Disconnected
I (19327) H_API: esp_wifi_remote_connect
I (19347) wifi station: retry to connect to the AP
I (21567) hosted_events: *** Heartbeat 0***
I (21757) RPC_WRAP: ESP Event: Station mode: Disconnected
I (21757) H_API: esp_wifi_remote_connect
I (21767) wifi station: retry to connect to the AP
I (24377) RPC_WRAP: ESP Event: Station mode: Connected
I (24377) esp_wifi_remote: esp_wifi_internal_reg_rxcb: sta: 0x400477f6
--- 0x400477f6: wifi_sta_receive at /home/kysoh/esp/gitlab_esp-idf/components/esp_wifi/src/wifi_netif.c:38
I (25387) esp_netif_handlers: sta ip: 192.168.50.217, mask: 255.255.255.0, gw: 192.168.50.1
I (25387) wifi station: got ip:192.168.50.217
I (25387) wifi station: connected to ap SSID:xxx password:yyy
I (26577) hosted_events: *** Heartbeat 1***
```

### Example Output using SPI-FD interface

```
I (5323) esp_netif_handlers: sta ip: 192.168.50.82, mask: 255.255.255.0, gw: 192.168.50.1
I (5323) wifi station: got ip:192.168.50.82
I (5323) wifi station: connected to ap SSID:xxx password:yyy
I (6383) hosted_events: *** Heartbeat 0***
I (11383) hosted_events: *** Heartbeat 1***
I (16383) hosted_events: *** Heartbeat 2***
I (18523) spi: rx packet ignored: len [65535], rcvd_offset[65535], exp_offset[12]

I (19173) transport: Received INIT event from ESP32 peripheral
I (19173) transport: EVENT: 12
I (19173) transport: Identified slave [esp32c6]
I (19173) transport: EVENT: 11
I (19183) transport: capabilities: 0xe8
I (19183) transport: Features supported are:
I (19183) transport:       - HCI over SPI
I (19193) transport:       - BLE only
I (19193) transport: EVENT: 13
I (19193) transport: ESP board type is : 13

I (19203) transport: Base transport is set-up, TRANSPORT_TX_ACTIVE
I (19203) H_API: Transport active
I (19213) transport: Slave chip Id[12]
I (19213) transport: raw_tp_dir[-], flow_ctrl: low[60] high[90]
I (19223) transport: transport_delayed_init
I (19223) esp_cli: Remove any existing deep_sleep cmd in cli
I (19233) esp_cli: Registering command: crash
I (19233) esp_cli: Registering command: reboot
I (19233) esp_cli: Registering command: mem-dump
I (19243) esp_cli: Registering command: task-dump
I (19243) esp_cli: Registering command: cpu-dump
I (19253) esp_cli: Registering command: heap-trace
I (19253) esp_cli: Registering command: sock-dump
I (19263) esp_cli: Registering command: host-power-save
I (19263) hci_stub_drv: Host BT Support: Disabled
I (19263) spi: Received INIT event
I (19273) hosted_events: *** got INIT event from co-processor ***
I (19273) hosted_events: *** Co-processor Reset Reason 1***
W (19283) hosted_events: *** Unexpected INIT event
I (19323) hosted_events: heartbeat timeout, transport failure or got INIT event: reinit Hosted
I (19323) wifi station: do not reconnect
I (19523) H_API: ESP-Hosted deinit

I (19523) serial_ll: Clean-up serial queue
I (19523) spi: Deinitializing SPI bus
I (19523) spi_wrapper: SPI deinitialized
I (19523) spi: SPI bus deinitialized
I (19523) transport: TRANSPORT_INACTIVE
I (19523) hosted_events: restarting ESP-Hosted
I (19533) hosted_events: init ESP-Hosted
I (19533) H_API: ESP-Hosted starting. Hosted_Tasks: prio:23, stack: 5120 RPC_task_stack: 5120
W (19543) H_API: Transport already initialized, skipping initialization
I (19553) H_API: ** add_esp_wifi_remote_channels **
I (19553) spi_wrapper: Transport: SPI, Mode:3 Freq:40MHz TxQ:20 RxQ:20
 GPIOs: CLK:18 MOSI:14 MISO:15 CS:19 HS:16 DR:17 SlaveReset:54
I (19563) spi: Staring SPI task
I (19573) os_wrapper_esp: GPIO [16] configuring as Interrupt
I (19573) os_wrapper_esp: GPIO [17] configuring as Interrupt
I (19583) H_API: ESP-Hosted Try to communicate with ESP-Hosted slave

I (19583) transport: Attempt connection with slave: retry[0]
I (19593) spi: Resetting slave
I (19593) spi: Resetting slave on SPI bus with pin 54
I (19603) os_wrapper_esp: GPIO [54] configured
I (20123) transport: Waiting for esp_hosted slave to be ready
I (20183) transport: Received INIT event from ESP32 peripheral
I (20183) transport: EVENT: 12
I (20183) transport: Identified slave [esp32c6]
I (20183) transport: EVENT: 11
I (20183) transport: capabilities: 0xe8
I (20183) transport: Features supported are:
I (20193) transport:       - HCI over SPI
I (20193) transport:       - BLE only
I (20193) transport: EVENT: 13
I (20203) transport: ESP board type is : 13

I (20203) transport: Base transport is set-up, TRANSPORT_TX_ACTIVE
I (20213) H_API: Transport active
I (20213) transport: Slave chip Id[12]
I (20213) transport: raw_tp_dir[-], flow_ctrl: low[60] high[90]
I (20223) transport: transport_delayed_init
I (20223) esp_cli: Remove any existing deep_sleep cmd in cli
I (20233) esp_cli: Registering command: crash
I (20233) esp_cli: Registering command: reboot
I (20243) esp_cli: Registering command: mem-dump
I (20243) esp_cli: Registering command: task-dump
I (20253) esp_cli: Registering command: cpu-dump
I (20253) esp_cli: Registering command: heap-trace
I (20253) esp_cli: Registering command: sock-dump
I (20263) esp_cli: Registering command: host-power-save
I (20263) hci_stub_drv: Host BT Support: Disabled
I (20273) spi: Received INIT event
I (20323) hosted_events: getting fw version
I (20583) hosted_events: *** got INIT event from co-processor ***
I (20583) hosted_events: *** Co-processor Reset Reason 1***
I (20583) hosted_events: Expected INIT event
I (20583) hosted_events: FW Version: 2.7.4
I (20593) transport: Attempt connection with slave: retry[0]
I (20593) transport: Transport is already up
I (20893) wifi station: wifi_init_sta finished.
I (20893) RPC_WRAP: ESP Event: wifi station started
I (20903) RPC_WRAP: ESP Event: wifi station started
I (23343) RPC_WRAP: ESP Event: Station mode: Disconnected
I (23343) H_API: esp_wifi_remote_connect
I (23373) wifi station: retry to connect to the AP
I (25593) hosted_events: *** Heartbeat 0***
I (25783) RPC_WRAP: ESP Event: Station mode: Disconnected
I (25783) H_API: esp_wifi_remote_connect
I (25793) wifi station: retry to connect to the AP
I (28403) RPC_WRAP: ESP Event: Station mode: Connected
I (28403) esp_wifi_remote: esp_wifi_internal_reg_rxcb: sta: 0x400408f2
--- 0x400408f2: wifi_sta_receive at /home/kysoh/esp/gitlab_esp-idf/components/esp_wifi/src/wifi_netif.c:38
I (29423) esp_netif_handlers: sta ip: 192.168.50.82, mask: 255.255.255.0, gw: 192.168.50.1
I (29423) wifi station: got ip:192.168.50.82
I (29423) wifi station: connected to ap SSID:xxx password:yyy
I (30593) hosted_events: *** Heartbeat 1***
I (32213) spi: rx packet ignored: len [65535], rcvd_offset[65535], exp_offset[12]
```
