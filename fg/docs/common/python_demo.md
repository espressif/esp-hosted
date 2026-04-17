# Demo App in python

- [test.py](../../host/linux/user_space/python_demo_app/test.py) is a demo application to test control path interface
- App is available under directory [python_demo_app/](../../host/linux/user_space/python_demo_app)
- App interacts with `esp_hosted_rpc_lib` (Hosted Control Library) by doing python to C translation
- This app is developed using [Google's Python Fire](https://github.com/google/python-fire), [Prompt toolkit](https://github.com/prompt-toolkit/python-prompt-toolkit) libraries and [ctypes](https://docs.python.org/3/library/ctypes.html) library

### Python to C translation
- [commands_map_py_to_c.py](../../host/linux/user_space/python_demo_app/commands_map_py_to_c.py) uses `ctypes` library to translate python into [ctrl_api.h](../../host/components/esp_hosted_rpc_lib/include/ctrl_api.h) APIs
- This way, both python and C codes are unified and both use the same [control path APIs](./ctrl_apis.md).
- Using this unification, python app becomes more maintainable and extensible

### Source files
- This demo app works over [esp_hosted_rpc_lib (Hosted Control Library)](../../host/components/esp_hosted_rpc_lib/)
- Config file [hosted_py_header.py](../../host/linux/user_space/python_demo_app/hosted_py_header.py) contains enum and basic structures, need not be modified
- [test.py](../../host/linux/user_space/python_demo_app/test.py) is the main demo app file. This works in shell mode or CLI mode.
- [py_parse/cmds.py](../../host/linux/user_space/python_demo_app/py_parse/cmds.py) is used with prompt toolkit library to add new command in auto complete
- [py_parse/process.py](../../host/linux/user_space/python_demo_app/py_parse/process.py) is wrapper to command handling functions in `commands_lib.py`
- [commands_lib.py](../../host/linux/user_space/python_demo_app/commands_lib.py) is a supporting file for Demo APP, where methods are implemented. User can adapt as per their need.
- [commands_map_py_to_c.py](../../host/linux/user_space/python_demo_app/commands_map_py_to_c.py) is mapper file for `ctypes` library
### Building the Python Demo Applications

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first
- Python3 and dependency software installed. You can make use of [setup_python.sh](../../host/linux/user_space/python_demo_app/setup_python.sh). Both local or virtual environment of python is supported.

**Building Steps:**

1. **Build the kernel module:**
```bash
cd esp_hosted_fg/host/linux/scripts/
./kmod_build_and_load.sh wifi=spi resetpin=6    # Adjust resetpin for your setup
```

2. **Build Python applications and RPC library:**
```bash
# Use the Python build script (builds required RPC library)
./py_app_build.sh
```

3. **Install Python dependencies:**
```bash
# Method 1: System-wide installation
sudo python3 -m pip install prompt_toolkit fire argparse docstring_parser requests

# Method 2: Virtual environment (recommended if you get externally-managed-environment error)
python3 -m venv my-venv
my-venv/bin/pip install prompt_toolkit fire argparse docstring_parser requests
```

4. **Run Python applications:**
```bash
# Using system Python with proper library path
sudo LD_LIBRARY_PATH=build_py/control_lib_build python3 ../user_space/python_demo_app/test.py

OR 

# Using virtual environment
sudo LD_LIBRARY_PATH=build_py/control_lib_build my-venv/bin/python test.py
```

### Modes supported
- Demo apps use `sudo` permission as interface up and down are executed as network IOCTL for command like `connect_ap`, `start_softap`
- There are two modes supported in demo application, `Shell mode` and `CLI mode`
- Gif shows the use of both modes
![alt text](ctrl_path_python.gif "Control Path Setup using python")

##### **CLI mode**
- User when pass **no arguments** to `test.py`, the app runs in CLI mode
- In CLI mode, command input from the user is taken, and then the input string is mapped to control request and injected into `Hosted Control Lib`
- CLI mode generally uses synchronous procedure as user intends to wait for the response
- Help
  - when string 'help' or '--help' is run, all available commands and their use are popped.
    - For example,

	```
	$ make clean; make
	$ sudo python3 ./test.py

	hosted > help
	```

  - command specific help is also available which describes supported mandatory and optional options
    - For example,

	```
	hosted > connect_ap --help    OR
	hosted > connect_ap help
	```

- Exit from CLI mode
  - commands `exit`, `quit` and `q` will exit the demo app

##### **Shell mode**
- Shell mode could be used to just run one command to completion and exit
- In command line shell, when command with options are passed, it will treat as shell mode
- Help for available commands

  ```
  $ make clean; make
  $ sudo python3 ./test.py --help
  ```

  - Command specific help is also available

  ```
  $ sudo python3 ./test.py start_softap --help
  ```

- Example of command execution:

  ```
  $ sudo python3 ./test.py connect_ap --ssid SaveEarth --pwd PlantMoreTrees123
  ```

### Supported commands
- Commands are picked from functions in file, [py_parse/cmds.py](../../host/linux/user_space/python_demo_app/py_parse/cmds.py)
- These commands will internally map to control request and send it to `esp_hosted_rpc_lib` (Hosted Control Library)
- These commands finally mapped to one of Hosted [control path API](./ctrl_apis.md). Please find all acceptable input values and possible outputs from API documentation.
- List of commands
  - `wifi_get_mode`
    - Get current Wi-Fi mode
  - `wifi_set_mode`
    - Set Wi-Fi mode
  - `wifi_get_mac`
    - Get Mac address of Wi-Fi mode passed
  - `wifi_set_mac`
    - Set Mac address of Wi-Fi mode passed
  - `get_available_ap`
    - Scan and list neighbouring APs (Wi-Fi routers or Hotspot)
  - `connect_ap`
    - Connect ESP station to external AP(Wi-Fi router or hotspot), assign MAC address of ESP station to `ethsta0` and up `ethsta0` interface
  - `get_connected_ap_info`
    - Get details of connected AP in station mode
  - `disconnect_ap`
    - Disconnect ESP station from external AP and down `ethsta0` interface
  - `softap_vendor_ie`
    - Enable or disable softap vendor specific IE
  - `start_softap`
    - Start ESP softAP, assign MAC address of ESP softAP to `ethap0` and up `ethap0` interface
  - `get_softap_info`
    - Get softAP configuration
  - `softap_connected_clients_info`
    - Get stations info which are connected to ESP softAP
  - `stop_softap`
    - Stop ESP softAP
  - `set_wifi_power_save`
    - Set Wi-Fi power save
  - `get_wifi_power_save`
    - Get current Wi-Fi power save mode
  - `set_wifi_max_tx_power`
    - Request firmware to set maximum possible from input power. Actual power set may slightly vary depending upon external factors at Wi-Fi driver.
  - `get_wifi_curr_tx_power`
    - Get current Wi-Fi TX power
  - `ota_update`
    - Over The Air (OTA) update from HTTP server using passed URL of ESP firmware binary
  - `heartbeat`
    - Disable or Enable heartbeat with expected duration
  - `subscribe_event`
    - Subscribe event to get notifications
  - `unsubscribe_event`
    - Unsubscribe event to get notifications
  - `disable_wifi`
    - Deinit Wi-Fi driver in slave
  - `enable_wifi`
    - Init and start Wi-Fi driver in slave
  - `disable_bt`
    - Deinit Bluetooth driver in slave
  - `enable_bt`
    - Init and start Bluetooth driver in slave
  - `get_fw_version`
    - Fetch firmware version at slave
  - `set_country_code`
    - Set the Wi-Fi Country Code
  - `get_country_code`
    - Get the current Wi-Fi Country Code

- Note:
  - Positional arguments are supported in case of `Shell mode`
  - In both, `CLI mode` and `Shell mode`, a single command is supported at a time.


# Stress Application

- [stress.py](../../host/linux/user_space/python_demo_app/stress.py) use for stress testing of control path APIs
- User may change config from top of file
- Stress test repeats every type of command sequence for `STRESS_TEST_COUNT` iterations

### How to run

**Prerequisites:**
- ESP-Hosted kernel module must be built and loaded first

**Building and Running:**
```bash
cd esp_hosted/esp_hosted_fg/host/linux/scripts
bash py_app_build.sh
sudo LD_LIBRARY_PATH=build_py/control_lib_build python3 ../user_space/python_demo_app/stress.py
```

# OTA update

- As explained above, OTA is run as

```sh
$ cd esp_hosted_fg/host/linux/user_space/python_demo_app/
$ sudo python3 ota_update <URL of ESP firmware binary>
```

- This script assumes the station is connected to an AP, an IP is assigned to `ethsta0` and HTTP URL is accessible.
- It downloads a **chunk** of OTA image data using HTTP client over `ethsta0` interface and writes on ESP flash
- Once all chunks are successfully written, it restarts ESP after 5 sec.
- Suggestion of steps for OTA with python
  - Start a HTTP server on a remote machine which contains OTA image.
  - The following python command can be used to start the HTTP server if it's not running already.

  ```
  $ python3 -m http.server <port_number>
  ```

  - Example:

  ```
  python3 -m http.server 9999
  ```

  - Pass OTA image URL as command line argument to ota_update.py.

  ```
  $ sudo python3 test.py ota_update "http://<IP_address>:<port_number>/ota_image.bin"
  ```

  - Example:

  ```
  $ sudo python3 test.py ota_update "http://192.168.0.106:9999/network_adapter.bin"
  ```

  - It will perform the following operations.
    - Erase OTA flash partition of ESP
    - Download chunk from URL and write that chunk into flash, one by one, till whole binary is written
    - Validate the complete written binary in flash
    - Sets newly written OTA partition as boot partition
    - Reboot the ESP after 5 second
