# Demo App in C

### Introduction
- This demo application showcases a way to configure control path.
- App is available under directory [c_demo_app/](../../host/linux/user_space/c_demo_app)
- App interacts with [`esp_hosted_rpc_lib` (Hosted Control Library)](../../host/components/esp_hosted_rpc_lib) using [control path APIs](./ctrl_apis.md).
- API header could be found at [ctrl_api.h](../../host/components/esp_hosted_rpc_lib/include/ctrl_api.h)

## Sample Applications

The c_demo_app directory contains several applications to interface with ESP devices:

- 1. `test.out` \
Uses **test.c**: Simplistic demo application to test control RPC APIs

- 2. `stress.out` \
Uses **stress.c**: Internal stress testing for control path APIs

- 3. `hosted_shell.out` \
**hosted_shell.c**: Interactive shell interface extended over test

- 4. `hosted_daemon.out` \
**hosted_daemon.c**: Background daemon for network management

These apps work over [esp_hosted_rpc_lib (Hosted Control Library)](../../host/components/esp_hosted_rpc_lib/) as base.\
These apps also use **app_custom_rpc.c**, which provides simple Custom RPC implementation for application-specific communication

## 1. Demo App (test.c)

### Source files
- Source files are [test.c](../../host/linux/user_space/c_demo_app/test.c) and [test_utils.c](../../host/linux/user_space/c_demo_app/test_utils.c)
- Config file : [ctrl_config.h](../../host/linux/user_space/c_demo_app/ctrl_config.h)

### List of control path commands
- Below is a list of control path commands showcased in demo app

| Command line argument | Operation |
|:----|:----|
| get_sta_mac_addr | Get Mac address of Wi-Fi in station mode |
| get_softap_mac_addr | Get Mac address of Wi-Fi in softAP mode |
| set_sta_mac_addr | Set Mac address of Wi-Fi in station mode |
| set_softap_mac_addr | Set Mac address of Wi-Fi in softAP mode |
|||
| get_wifi_mode | Get current Wi-Fi mode |
| set_wifi_mode | Set Wi-Fi mode |
|||
| get_ap_scan_list | Get neighboring AP (Wi-Fi routers or Hotspot) list |
| sta_connect | Connect ESP station to external AP(Wi-Fi router or hotspot), assign MAC address of ESP station to `ethsta0` and up `ethsta0` interface |
| get_sta_config | Get details of connected AP in station mode |
| sta_disconnect | Disconnect ESP station from external AP and down `ethsta0` interface |
|||
| set_softap_vendor_ie | Set vendor information element for ESP softAP |
| reset_softap_vendor_ie | Reset vendor information element for ESP softAP |
| softap_start | Start ESP softAP, assign MAC address of ESP softAP to `ethap0` and up `ethap0` interface |
| get_softap_config | Get softAP configuration |
| softap_connected_sta_list | Get connected station info of softAP |
| softap_stop | Stop ESP softAP stop and down `ethap0` interface |
|||
| set_wifi_powersave_mode | Set Wi-Fi Power save mode |
| get_wifi_powersave_mode | Get Wi-Fi Power save mode |
|||
| set_wifi_max_tx_power | Sets Wi-Fi maximum transmitting power |
| get_wifi_curr_tx_power | Get Wi-Fi current transmitting power |
|||
| ota </path/to/ota_image.bin> | performs OTA operation using local OTA binary file |
|||
| get_fw_version | Get firmware version |
| enable_wifi | Enable Wi-Fi driver |
| disable_wifi | Disable Wi-Fi driver |
| enable_bt | Enable Bluetooth driver |
| disable_bt | Disable Bluetooth driver |
|||
| get_fw_version | Get Firmware Version |
|||
| get_dhcp_dns_status | Get DHCP and DNS status (Works if Network Split is enabled) |
|||
| set_country_code | Set Country Code with IEEE802.11d disabled |
| set_country_code_enabled | Set Country Code with IEEE802.11d enabled |
| get_country_code | Get the current Country Code|
|||
| send_packed_data__only_ack | Custom RPC demo 1 - send data with acknowledgement only |
| send_packed_data__echo_back_as_response | Custom RPC demo 2 - send data with echo back as response |
| send_packed_data__echo_back_as_event | Custom RPC demo 3 - send data with echo back as event |



> [!NOTE]
>
> 1. With IEEE802.11d enabled, the country info of the AP to which the station is connected is used. E.g. if the configured country is US and the country info of the AP to which the station is connected is JP then the country info that will be used is JP. If the station disconnected from the AP the country info is set back to the country info of the station automatically, US in the example.
> 2. With IEEE802.11d disabled, then the configured country info is used always.
> 3. When the country info is changed because of configuration or because the station connects to a different external AP, the country IE in probe response/beacon of the soft-AP is also changed.

### How to run
### Building the C Demo Applications

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first (see Getting_started.md for details)


```bash
cd esp_hosted_fg/host/linux/scripts/
# First build and load the kernel module as per Getting_started.md
# example,
bash ./esp_kmod_up.sh wifi=sdio
```

**Building Steps:**


```bash
cd esp_hosted_fg/host/linux/scripts/
./c_app_build.sh

# Binaries will be created in: esp_hosted_fg/host/linux/scripts/bin/
```
> [!NOTE]
> 
> For `text.out`, some configuration sudh as SSId, passsowrd etc needs to changed, as per user's test environment. File to adjust: ../user_space/c_demo_app/ctrl_config.h. Rebuild again if changed.
> For other apps, this is not needed.

the easiest to evaluate is using hosted shell example, where you have CLI support, to configure on the fly.

Building above, should create:
- `bin/test.out` - Basic demo application
- `bin/hosted_daemon.out` - Network management daemon
- `bin/hosted_shell.out` - Interactive shell (requires replxx library)




## 1. Interactive Shell Application (hosted_shell.c)

[hosted_shell.c](../../host/linux/user_space/c_demo_app/hosted_shell.c) provides an interactive shell interface for controlling the ESP device. It offers a more user-friendly way to interact with the device through a command-line shell with features like command auto-completion, and hints.


### Features
- Interactive command-line interface
- Tab completion for commands and arguments
- Command hints
- Help documentation for all commands

### How to build and run

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first (see Getting_started.md for details)
```
cd esp_hosted_fg/host/linux/scripts/
./esp_kmod_up.sh wifi=sdio bt=sdio
```

**Installing replxx dependency:**
`replxx` is used as command line processor which is shell handler. You are free to change to your preferred shell handler.

```bash
# Install replxx library
git clone https://github.com/AmokHuginnsson/replxx.git
cd replxx
make
sudo make install
```

**Building and Running:**
```bash
# Method 1: Using build script
cd esp_hosted_fg/host/linux/scripts/
./c_app_build.sh
sudo ./bin/hosted_shell.out

# Method 2: Manual build
cd esp_hosted_fg/host/linux/user_space/c_demo_app/
make hosted_shell
sudo ./hosted_shell.out
```

In the shell, you can type double tab to see all available commands


## 2. Minimalistic example, test.out

Config file: [ctrl_config.h](../../host/linux/user_space/c_demo_app/ctrl_config.h)

Once changed this config, please rebuild and execute `sudo ./bin/./test.out`

Below are some details on `test.out`.
```sh
$ make
$ sudo ./test.out \
	[ get_sta_mac_addr      || get_softap_mac_addr     || set_sta_mac_addr          || \
	  set_softap_mac_addr   || get_wifi_mode           || set_wifi_mode             || \
	  get_ap_scan_list      || sta_connect             || get_sta_config            || \
	  sta_disconnect        || set_softap_vendor_ie    || reset_softap_vendor_ie    || \
	  softap_start          || get_softap_config       || softap_connected_sta_list || \
	  softap_stop           || set_wifi_powersave_mode || get_wifi_powersave_mode   || \
	  set_wifi_max_tx_power || get_wifi_curr_tx_power  || \
	  ota </path/to/esp_firmware_network_adapter.bin> || \
	  enable_wifi || disable_wifi || enable_bt || disable_bt || get_fw_version || \
	  set_country_code || set_country_code_enabled || get_country_code || \
	  get_dhcp_dns_status || send_packed_data__only_ack || \
	  send_packed_data__echo_back_as_response || send_packed_data__echo_back_as_event
	]
```
For example,
```sh
$ sudo ./test.out get_wifi_mode
```

### Some points to note
- Connect to AP in station mode
  - After `sta_connect`, User needs to run the DHCP client to obtain an IP address from an external AP. Then network data path will be open for higher applications to use `ethsta0` interface for data communication. For an example as below.

  ```sh
  $ sudo killall dhclient      # kill earlier dhclient processes
  $ sudo dhclient ethsta0 -v   # Run DHCP client to get IP from AP/Router
  ```

- Disconnect AP in station mode
  - After disconnect, user can remove DHCP lease. For example,

  ```sh
  $ sudo dhclient ethsta0 -r     # clean-up earlier assigned DHCP IP
  and
    $ sudo killall dhclient      # kill earlier dhclient processes

  ```

- For softAP vendor specific IE
  - `set_softap_vendor_ie` will be in effect only if it is done before starting of ESP softAP
  - Once vendor IE set, consecutive `set_softap_vendor_ie` will fail unless vendor IE is reset using `reset_softap_vendor_ie` or ESP reboot
- SoftAP start
  - After `softap_start` to start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`). For an example as below:

  ```sh
  $ sudo dnsmasq --no-daemon --no-resolv --no-poll --dhcp-script=/system/bin/dhcp_announce --dhcp-range=192.168.4.1,192.168.4.20,1h

  $ sudo ifconfig ethap0 192.168.4.5
  ```

- SoftAP stop
  - After stopping softAP, dnsmasq could be down/killed as per user expectation

- OTA
  - The OTA update using C currently assumes the complete binary is downloaded locally
  - OTA update using HTTP URL is only supported in [python demo app](python_demo.md#ota-update)
  - In case HTTP based OTA update is desired, user can do the same using third party HTTP client library

  ```sh
  ex.
  ./test.out ota </path/to/ota_image.bin>
  ```

- Set Wi-Fi max transmit power
  - This is just a request to Wi-Fi driver. The actual power set may slightly differ from exact requested power.


## 3. Network Management Daemon (hosted_daemon.c)

[hosted_daemon.c](../../host/linux/user_space/c_demo_app/hosted_daemon.c) implements a background daemon that manages network interfaces for ESP device. It handles network events and automatically configures interfaces based on events from the ESP device.

### Features
- Runs as a daemon in the background
- Automatically manages network interfaces
- Handles DHCP and DNS configuration
- Responds to network state changes from ESP device

### How to build and run

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first (see Getting_started.md for details)
```
cd esp_hosted_fg/host/linux/scripts/
./esp_kmod_up.sh wifi=sdio bt=sdio
```

**Building and Running:**
```bash
# Method 1: Using build script
cd esp_hosted_fg/host/linux/scripts/
./c_app_build.sh
sudo ./bin/hosted_daemon.out

# Method 2: Manual build
cd esp_hosted_fg/host/linux/user_space/c_demo_app/
make hosted_daemon
sudo ./hosted_daemon.out
```

To run in foreground mode (for debugging):
```bash
sudo ./hosted_daemon.out -f
```

## 4. Stress Testing Application (stress.c)

[stress.c](../../host/linux/user_space/c_demo_app/stress.c) is used for internal stress testing of control path APIs. It is similar to the demo app but additionally allows multiple iteration testing.

### Source files
- Source files are [stress.c](../../host/linux/user_space/c_demo_app/stress.c) and [test_utils.c](../../host/linux/user_space/c_demo_app/test_utils.c)
- Config file : [ctrl_config.h](../../host/linux/user_space/c_demo_app/ctrl_config.h)

### How to run

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first (see Getting_started.md for details)
```
cd esp_hosted_fg/host/linux/scripts/
./esp_kmod_up.sh wifi=sdio bt=sdio
```

**Building and Running:**
1. Change expected configuration from config file [ctrl_config.h](../../user_space/c_demo_app/ctrl_config.h)
2. Build the stress application:
```bash
# Method 1: Using build script
cd esp_hosted_fg/host/linux/scripts/
./c_app_build.sh

# Method 2: Manual build
cd esp_hosted_fg/host/linux/user_space/c_demo_app/
make stress
```
3. Execute `stress.out` as below:

```sh
$ sudo ./stress.out <Number of test iterations> [get_sta_mac_addr] [get_softap_mac_addr] \
		[set_sta_mac_addr] [set_softap_mac_addr] [get_wifi_mode] [set_wifi_mode] \
		[get_ap_scan_list] [sta_connect] [get_sta_config] [sta_disconnect] \
		[set_softap_vendor_ie] [reset_softap_vendor_ie] [softap_start] \
		[get_softap_config] [softap_connected_sta_list] [softap_stop] \
		[set_wifi_powersave_mode] [get_wifi_powersave_mode] [set_wifi_max_tx_power] \
		[get_wifi_curr_tx_power] [set_country_code] [set_country_code_enabled] [get_country_code] \
		[ota </path/to/esp_firmware_network_adaptor.bin>]

For example:
$ sudo ./stress.out 10 scan sta_connect sta_disconnect ap_start sta_list ap_stop wifi_tx_power
```


# Custom RPC Communication (app_custom_rpc.c)

[app_custom_rpc.c](../../host/linux/user_space/c_demo_app/app_custom_rpc.c) demonstrates how to use the Custom Remote Procedure Call (RPC) functionality of ESP Hosted. This allows application-specific communication between the host and ESP device.

### Features
- Demonstrates custom communication between host and ESP device
- Allows sending events and requests with custom data
- Shows how to handle custom responses and events

> [!NOTE]
>
> 1. Provides APIs for sending packed data between host and ESP device.
> 2. User is responsible for serializing data before sending and deserializing after receiving.

### Example Demo Functions
- `custom_rpc_demo1_request_only_ack`: Sends a request with only acknowledgement (uses `CUSTOM_RPC_REQ_ID__ONLY_ACK`)
- `custom_rpc_demo2_request_echo_back_as_response`: Sends data and receives an echo back as response (uses `CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE`)
- `custom_rpc_demo3_request_echo_back_as_event`: Sends data and receives echo back as an event (uses `CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT` and `CUSTOM_RPC_EVENT_ID__DEMO_ECHO_BACK_REQUEST`)

These demos are integrated in applications like `test.out` and `hosted_shell.out` and demonstrate the complete flow of custom RPC communication between host and ESP device.
It uses underlying control path API for reliable communication.

> [!NOTE]
>
> Current APIs discussed can carry your own 'packed' data from host to slave or vice versa. If you need pure serialised message handling, you can add new message in esp_hosted_config.proto and handle similar to other existing RPC protobuf messages

### Adding New Application-Specific RPC

This needs tiny tweaks at both coprocessor and host as per your requirement. You can register your own custom message types at both slave and host. This way, you can add mutiple RPC requests and events per use case basis.

#### 2. Implementation Steps

**Host Side:**
- Add your custom `requests` or `events` in [app_custom_rpc.c](../../host/linux/user_space/c_demo_app/app_custom_rpc.c) and [app_custom_rpc.h](../../host/linux/user_space/c_demo_app/app_custom_rpc.h)
- Use the provided APIs in `esp_hosted_custom_rpc.h` to send and receive custom RPC data
- Optionally add new Request or Event IDs in [esp_hosted_custom_rpc.h](../../common/include/esp_hosted_custom_rpc.h):

```c
typedef enum {
    CUSTOM_RPC_REQ_ID__ONLY_ACK = 1,
    CUSTOM_RPC_REQ_ID__ECHO_BACK_RESPONSE,
    CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT,
    /* Add your custom request IDs here */
} custom_rpc_req_id_e;
```

**ESP Coprocessor Side:**
- Use the **coprocessor extension framework** for custom RPC handling
- See the complete example at: [coprocessor/examples/extensions/custom_rpc_msg/](../../coprocessor/examples/extensions/custom_rpc_msg/)
- Key APIs for ESP coprocessor:
  ```c
  # part of example code
  /* Initialize custom RPC module (includes handler registration) */
  esp_err_t esp_hosted_cp_ext_user_defined_rpc_init(void);
  
  /* Send custom events to host */
  esp_err_t esp_hosted_cp_ext_user_defined_rpc_send_event(uint32_t custom_event_id, const void *data, size_t data_len);
  
  /* Register extension with core component */
  esp_err_t esp_hosted_cp_register_extension(esp_hosted_cp_ext_type_t ext_type, esp_hosted_cp_ext_t *ext);
  ```

#### 3. Example Usage
Refer to the complete working example: [custom_rpc_msg example](../../coprocessor/examples/extensions/custom_rpc_msg/README.md)

This example demonstrates:
- Registering custom RPC handlers using the extension framework
- Handling echo requests and converting requests to events  
- Sending periodic events with data
- Clean separation between core networking and application logic

> [!NOTE] Custom RPC events
  >
  > 1. The handlers in these requests should be as concise as possible.
  > 2. Although it is not interrupt context, try not to have blocking 
  calls in the response handler, as this function is called as callback.
  > 3. Refrain sending any synchronous RPC request as this would make it 
  dead block (RPC Request `without` a response function callback 
  registered - > `Sync RPC Req`)
  > 4. Sending any 'asynchronous' RPC request is supported (RPC Request 
  `with` a response function callback registered - > `Async RPC Req`)
  - You can also trigger events instead of immediate responses as 
  demonstrated in `CUSTOM_RPC_REQ_ID__ECHO_BACK_AS_EVENT`