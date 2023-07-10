# Host Sleep for ESP-Hosted-NG
- [1. Introduction](#1-introduction)
  * [2. Additional GPIO](#2-additional-gpio)
  * [3. ESP side setup](#3-esp-side-setup)
  * [4. Host side setup](#4-host-side-setup)
    + [4.1 Device Tree setup](#41-device-tree-setup)
    + [4.2 Host software setup](#42-host-software-setup)
  * [5. Logs to verify](#5-logs-to-verify)
    + [5.1 ESP Log](#51-esp-log)
    + [5.2 Host log](#52-host-log)


# 1. Introduction

The crux of this feature is to optimize the host power by putting the host into deep sleep. ESP chipset would wake up the sleeping host if need be.

Possible ways to wake-up host:
1. Data packet is destined for Host IP (ESP wakes up Host by triggering host GPIO)
2. Some GPIO triggered (Host waken up because of peripheral)
3. Touchsceen is pressed (Host waken up because of peripheral)

This feature focuses on point (1) only, as point (2) & (3) would be handled at host level and requires no additional setting.
This feature is tested on imx8 mini board (imx8mm-lpddr4-evk). Other Linux board also should be able to port easily.

- Points to note:
  - Not all the Linux systems support this feature
  - Many Linux distributions may have different commands to put host into sleep mode

## 2. Additional GPIO
- IO05 is connected to pin 38 on J1003 GPIO header on imx8 mini board
- This GPIO is customizable

## 3. ESP side setup
- Extra GPIO is to be connected using which the ESP wakes up host when needed.
- It is possible to change this GPIO using `idf menuconfig`

## 4. Host side setup

### 4.1 Device Tree setup

SoC used is imx8mm-lpddr4-evk. Please keep in mind the device tree is very specific to the SoC. For other Linux hosts, you can relate and modify the device tree accordingly.

- Change `imx8mm-evk.dtsi` similar to below:

```
/dts-v1/;

#include <dt-bindings/usb/pd.h>
#include "imx8mm.dtsi"

/ {

...
...

+  gpio-keys {
+    compatible = "gpio-keys";
+    pinctrl-names = "default";
+    pinctrl-0 = <&pinctrl_gpio_esp_to_host_wakeup>;
+
+    gpio_esp_to_host_wakeup {
+      label = "gpio-esp-to-host-wakeup";
+      linux,code = <KEY_POWER>;
+      gpios = <&gpio3 21 GPIO_ACTIVE_HIGH>;
+      wakeup-source;
+      debounce-interval = <50>;
+    };
+  };
+
+
+
+
 };

...
...
...

 &iomuxc {
     ...
     ...
+
+
+
+	pinctrl_gpio_esp_to_host_wakeup: pinctrl_gpio_esp_to_host_wakeup_grp {
+		fsl,pins = <
+			MX8MM_IOMUXC_SAI5_RXD0_GPIO3_IO21 0x19
+		>;
+	};
+
+

...
...

};
```


- Verify if the GPIO is configured
```sh
cat /proc/interrupts  | grep esp
162:          0  gpio-mxc  21 Edge      gpio-esp-to-host-wakeup
```

Please note:
1. `wakeup-source` is the configuration what makes this GPIO as wakeup compatible GPIO
2. imx8nm boards also should be compitible with similar configuration
3. For other recent Linux boards like imx8qxp, [pad gpio](https://community.nxp.com/t5/i-MX-Processors-Knowledge-Base/How-to-add-iMX8QXP-PAD-GPIO-Wakeup/ta-p/1120559?attachment-id=32640) may be required

References: \
a. More info on [wakeup-source](https://www.kernel.org/doc/Documentation/devicetree/bindings/power/wakeup-source.txt) \
b. GPIO wakeup-source for [imx8mm example](https://community.nxp.com/t5/i-MX-Processors/GPIO-based-wake-from-deep-sleep-for-I-mx8-mini/td-p/1324317)

### 4.2 Host software setup
1. Once the esp32 kernel driver is loaded, validate if network device is detected and driver is active. \
Following command should show 'espsta0' network interface
```sh
$ ifconfig -a
```
2. Using procedure [Connect the Wi-Fi](https://github.com/espressif/esp-hosted/tree/master/esp_hosted_ng#321-wi-fi) establish the station mode connection
3. Execute following command to verify connection with Access point
```sh
$ sudo iwconfig espsta0
espsta0   IEEE 802.11  ESSID:"<SSID>"
		  Mode:Managed  Frequency:2.462 GHz  Access Point: C4:70:0B:CF:F9:AF
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Encryption key:off
          Power Management:on
```
4. Assign appropriate IP address (using dhclient or manually a static IP) to espsta0 interface and ping access point to ensure data path is working fine
5. To test host suspend/resume, execute following command to enable host wake up

```sh
$ sudo iw phy0 wowlan enable magic-packet
```
6. To verify above step, execute following
```sh
$ sudo iw phy0 wowlan show
	WoWLAN is enabled:
		 * wake up on magic packet
```
7. Execute following command to put IMX in sleep
```sh
$ sudo echo s2idle > /sys/power/mem_sleep
$ sudo echo mem > /sys/power/state
```
8. Once host is awake, ping access point to ensure that data path is working fine


## 5. Logs to verify

### 5.1 ESP Log
```
D (132978) wifi:Send sta connected event
D (132982) event: running post WIFI_EVENT:4 with handler 0x400d82a4 and context 0x3ffb7f1c on loop 0x3ffb7e84
D (132982) wifi:connect status 1 -> 5I (132990) FW_CMD: Wifi Station Connected event!!


D (132996) wifi:obss scan is disabled
D (133000) wifi:start obss scan: obss scan is stopped
I (133004) wifi:AP's beacon interval = 102400 us, DTIM period = 1
D (133008) wifi:set max rate: from <rate=130, phy=3, sig=0> to <rate=144, phy=3 sig=0>
D (133014) wifi:sig_b=0, sig_g=0, sig_n=0, max_b=22, max_g=108, max_n=144
D (133020) wifi:update trc
I (147170) FW_MAIN: Set IP Address

D (147174) SDIO_HAL: restart new send: 0x3ffe1df0->0x3ffe1df0, pkt_len: 3546
I (147190) FW_MAIN: Set multicast mac address list

D (147192) SDIO_HAL: restart new send: 0x3ffe1c60->0x3ffe1c60, pkt_len: 3566
I (147200) FW_MAIN: Set multicast mac address list

D (147202) SDIO_HAL: restart new send: 0x3ffe1c74->0x3ffe1c74, pkt_len: 3586
D (148970) SDIO_HAL: restart new send: 0x3ffe1c88->0x3ffe1c88, pkt_len: 4010
D (148970) SDIO_HAL: restart new send: 0x3ffe1c9c->0x3ffe1c9c, pkt_len: 4506
D (148972) SDIO_HAL: restart new send: 0x3ffe1cb0->0x3ffe1cb0, pkt_len: 4998
D (148978) SDIO_HAL: restart new send: 0x3ffe1cc4->0x3ffe1cc4, pkt_len: 5502
D (148984) SDIO_HAL: restart new send: 0x3ffe1cd8->0x3ffe1cd8, pkt_len: 5988
D (148988) SDIO_HAL: restart new send: 0x3ffe1cec->0x3ffe1cec, pkt_len: 6476
D (148994) SDIO_HAL: restart new send: 0x3ffe1d00->0x3ffe1d00, pkt_len: 6800
D (149000) SDIO_HAL: restart new send: 0x3ffe1d14->0x3ffe1d14, pkt_len: 7188
D (149006) SDIO_HAL: restart new send: 0x3ffe1d28->0x3ffe1d28, pkt_len: 7586
D (157818) SDIO_HAL: restart new send: 0x3ffe1d3c->0x3ffe1d3c, pkt_len: 7640
D (157834) SDIO_HAL: restart new send: 0x3ffe1d50->0x3ffe1d50, pkt_len: 7750
D (158638) SDIO_HAL: restart new send: 0x3ffe1d64->0x3ffe1d64, pkt_len: 7860
D (159458) SDIO_HAL: restart new send: 0x3ffe1d78->0x3ffe1d78, pkt_len: 7970
D (162082) SDIO_HAL: restart new send: 0x3ffe1d8c->0x3ffe1d8c, pkt_len: 8024
I (170236) FW_MAIN: Host Sleep
E (1413238) FW_SDIO_SLAVE: WAKE UP Host!!!!!

I (1413384) FW_MAIN: Host Awake
D (1414058) SDIO_HAL: restart new send: 0x3ffe1c60->0x3ffe1c60, pkt_len: 54
```

Where,
1. `I (147170) FW_MAIN: Set IP Address` is printed when the IP address is assigned to espsta0
2. `I (170236) FW_MAIN: Host Sleep` is printed when Host is put to sleep
3. `E (1413238) FW_SDIO_SLAVE: WAKE UP Host!!!!!` is printed when we have pinged to espsta0 IP adress (e.g. from PC connected to AP, where ESP is connected to)
4. `WAKE UP Host` is printed when ESP triggers high output on `gpio_esp_to_host_wakeup`.
5. If it is printed **repeatedly** without host getting wakeup, please check your GPIO is connected correctly from ESP to host & Device tree is correctly configured on host.


### 5.2 Host log

```sh
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~# cat set_sleepable.sh
iw phy0 wowlan enable magic-packet
iw phy0 wowlan show
root@imx8mm-lpddr4-evk:~# cat sleep_now.sh
echo s2idle > /sys/power/mem_sleep
echo mem > /sys/power/state
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~# ifconfig espsta0 192.168.1.88
[  179.047223] esp_inetaddr_event: NETDEV_UP interface espsta0 ip changed to  192.168.001.088
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~#
root@imx8mm-lpddr4-evk:~# bash set_sleepable.sh
WoWLAN is enabled:
 * wake up on magic packet
root@imx8mm-lpddr4-evk:~# bash sleep_now.sh
[  206.808749] PM: suspend entry (s2idle)
[  206.817536] Filesystems sync: 0.004 seconds
[  206.829318] Freezing user space processes ... (elapsed 0.001 seconds) done.
[  206.837972] OOM killer disabled.
[  206.841212] Freezing remaining freezable tasks ... (elapsed 0.001 seconds) done.
[  206.849779] printk: Suspending console(s) (use no_console_suspend to debug)
[ 1622.790390] ----> Host Suspend
[ 1623.834392] fec 30be0000.ethernet eth0: Link is Down
[ 1623.843126] PM: suspend devices took 1.052 seconds
[ 1623.919015] caam 30900000.crypto: registering rng-caam
[ 1623.919169] -----> Host Awake
[ 1623.919253] Rx Pre ====== 268591
[ 1623.919258] Rx Pos ======  0
[ 1623.919306] Tx Pre ======  24
[ 1623.919309] Tx Pos ======  10
[ 1624.028441] PM: resume devices took 0.184 seconds
[ 1624.066254] OOM killer enabled.
[ 1624.069403] Restarting tasks ... done.
[ 1624.094026] PM: suspend exit
root@imx8mm-lpddr4-evk:~# [ 1626.980516] fec 30be0000.ethernet eth0: Link is Up - 1Gbps/Full - flow control rx/tx
```

Where,
1. `set_sleepable.sh` is script which enables host sleep configuration
2. `sleep_now.sh` is script which when fired, puts host, `imx8mm-lpddr-evk` into deep sleep.
3. Commands in `sleep_now.sh`, demonstrates way to sleep imx8mm-lpddr-evk. It may vary for other Linux hosts depending upon SoC.
4. `ifconfig espsta0 192.168.1.88` assigns static IP address to espsta0 network interface
5. `[  206.857408] ----> Host Suspend`  shows host went to deep sleep. But the log would only be printed after waking up (As Linux went to deep sleep already by the time)
6. Host will stay in suspend state unless waken up by one of way discussed earlier (GPIO awake from ESP OR Hosts self GPIO OR screen touch etc)
7. `[  207.979186] -----> Host Awake` is printed because ESP sensed input data packet on espsta0 and triggered pin 38 on J1003 using `gpio_esp_to_host_wakeup` GPIO
8. Normal operation on host resumed after that.
