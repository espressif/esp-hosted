# System Architecture: Linux Based Host
Below diagram depicts building blocks of Linux based ESP-Hosted solution.

![ESP-Hosted linux based design](./linux_hosted_design.png)


## ESP Host Software

This implements ESP-Hosted solution part that runs on Linux host. It mainly consists of following.
* ESP Host Driver
* Control/command interface
* Python convenience scripts
  
---

### **ESP Host Driver**

ESP Host driver implements following.  

* **SDIO Host driver**  
This implements data path over SDIO interface. Communication protocol is explained in further section.

* **SPI Host driver**  
This implements data path over SPI interface. Communication protocol is explained in further section.

* **Virtual Serial interface driver**  
This implements virtual serial interface over SDIO/SPI interface. This virtual serial interface is used as a control interface to configure Wi-Fi of ESP peripheral

* **802.3 network interface**  
This registers two network interfaces with Linux kernel: ethsta0 and ethap0. This allows exchange of 802.3 frames between Linux kernel and ESP firmware.

* **HCI interface**  
This registers HCI interface with Linux kernel. This interface is implemented over SDIO/SPI.

  
---

### **Control/Command Interface**

* This implements custom control commands that are based on protobuf.
* These commands are used to control and configure Wi-Fi on ESP peripheral.
* Control interface makes use of virtual serial interface provided by ESP Host driver.
* There are 2 flavors of control interface implementation:
	* Python based implementation
	* C based implementation
* API's are described in subsequent section
  
---

### **Python Convenience Scripts**
Following are few ready to use convenience script provided in the repository. These scripts make use of 'python implementation' of control commands interface as mentioned in above section.

* **Scan external access points**  

	`ap_scan_list.py` is a python script which gives a scanned list of available APs. The list contains SSID, channel number, RSSI, MAC address and authentication mode of AP.

	```
	python ap_scan_list.py
	```


* **Connect to external access point**  

	`station_connect.py` is a python script which configures ESP peripheral in station mode and connects to an external AP with user-provided credentials. Also it enables the station interface and runs DHCP client. The script accepts arguments such as SSID, password, optionally MAC address, wpa3 support and listen interval (AP beacon intervals). For example:

	```
	python station_connect.py 'xyz' 'xyz123456' --bssid='e5:6c:67:3c:cf:65' --is_wpa3_supported=True --listen_interval=3
	```

	You can check that `ethsta0` interface is up (enabled) using `ifconfig`. WPA3 option is only applicable if target AP supports WPA3.

	To know status of station, use wifi_get_ap_config() function. In case station is connected with AP, it returns ssid, bssid(MAC address), channel, rssi, encryption mode of AP. and In case of not connected with AP returns `failure` with `not_connected` print.


* **Disconnect from external access point**  

	`station_disconnect.py` is a python script to disconnect ESP peripheral station from AP.

	```
	python station_disconnect.py
	```

	You can check that `ethsta0` interface is down (disabled) using `ifconfig`.


* **Setup and start SoftAP**  

	`softap_config.py` is a python script for configuring ESP peripheral to work in softAP mode. Following parameters should be provided:

		- SSID
		- password, should be 8 ~ 64 bytes ASCII
		- channel ID, 1 ~ 11
		- encryption method (0: `OPEN`, 2: `WPA_PSK`, 3: `WPA2_PSK`, 4: `WPA_WPA2_PSK`)
		- maximum number of stations, in range of 1 ~ 10.
		- whether SSID is hidden (True if the softAP shouldn't broadcast its SSID, else False)
		- bandwidth (1: `WIFI_BW_HT20` (20MHz), 2: `WIFI_BW_HT40` (40MHz))

	The maximum number of connections, "SSID hidden", and bandwidth parameters are optional.

	For example:
	```
	python softap_config.py 'xyz' 'xyz123456' 1 3 --max_conn=4 --ssid_hidden=False --bw=1
	```

	You can check that `ethap0` interface is up (enabled) using `ifconfig`.
	`Note: Currently WEP, WPA2_ENTERPRISE, WPA3 support is not present for softAP mode.`

	To start data connection, set up a DHCP server on the Raspberry Pi, or configure a static IP address for AP interface (`ethap0`).


* **Stop SoftAP**  

	`softap_stop.py` python script disables wifi softAP mode on ESP peripheral. This script will change wifi mode to `null` if only softAP is running, or to `station` mode if softAP and station both are on.

	```
	python softap_stop.py
	```

	You can check that `ethap0` interface is down (disabled) using `ifconfig`.


* **List external stations connected to SoftAP**  

	`connected_stations_list.py` is a python script that returns a list of MAC addresses of stations connected to softAP.

	```
	python connected_stations_list.py
	```

