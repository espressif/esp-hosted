# Network Split with Host Deep Sleep Example

This example shows how to use two powerful ESP-Hosted features together: **Network Split** and **Host Deep Sleep**. These features allow a host MCU to save power by sleeping while the slave device keeps the network connection alive and only wakes the host when necessary.

## What This Example Does

**Network Split** - Smart traffic routing:

- Host handles important traffic (ports 49152-61439)  
- Slave handles background traffic (ports 61440-65535)
- No unnecessary host wake-ups

**Host Deep Sleep** - Ultra-low power mode:

- Host can sleep while slave stays connected to WiFi
- Slave wakes host only for important traffic
- Wake-up can also be triggered by commands or MQTT messages

## Supported Platforms and Transports

### Supported Co-processors

| Co-Processors Supported | ESP32-C5 | ESP32-C6/C61 | ESP32 | ESP32-S2 | ESP32-S3 |
| :---------------------- | :------- | :----------- | :---- | :------- | :------- |

### Supported Host MCUs

| Hosts Supported | ESP32-P4 | ESP32-H2 | Any other MCU hosts |
| :-------------- | :------- | :------- | :------------------ |

### Supported Transport Interfaces

| Transport Interface | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :------------------ | :--- | :-------------- | :-------------- | :--- |

## Example Hardware Used

This example is designed for the **ESP32-P4-Function-EV-Board** with its built-in ESP32-C6.

**Default setup:**

- **Host**: ESP32-P4 (handles important network traffic)
- **Slave**: ESP32-C6 (maintains WiFi and handles background traffic)  
- **Connection**: SDIO (pre-wired on the board)
- **Wake-up wire**: Already connected between GPIO2 (slave) and GPIO6 (host)

All listed items in default setup are customizable to suit specific use cases and requirements.

## Flash the Slave (ESP32-C6)

The slave needs ESP-Hosted firmware with Network Split and Host Power Save enabled.

With below additional configuration, Flash the slave using setup instructions, see: the slave ESP-Hosted firmware build guide

#### Slave Side Configuration:

Using `eh.py menuconfig`, Enable:

```
# Minimal slave config
Example Configuration
├── [*] Enable Network Split
└── [*] Allow host to power save
```

**Advanced slave settings** (optional):

```
Example Configuration
└── Network Split Configuration
    ├── Host Static Port Forwarding
    │   ├── TCP dst: 22,80,443,8080,8554
    │   └── UDP dst: 53,123
    ├── Port Ranges
    │   ├── Host: 49152–61439
    │   └── Slave: 61440–65535
    └── Host power save config
        ├── Allow host to enter deep sleep
        ├── Slave out: Host wakeup GPIO (2)
        └── Host Wakeup GPIO Level (High)
```

Continue to build and flash slave using the slave ESP-Hosted firmware build guide

## Flash the Host (ESP32-P4)

The host needs ESP-Hosted firmware with Network Split and Host Power Save configured.

**Essential host configuration:**

1. Set target: `eh.py set-target esp32p4`

2. Network split and Host power save is pre-configured in this example.
(Optionally) customise the configuration using `eh.py menuconfig`

```
Component config
└── ESP-Hosted config
    ├── [*] Enable Network Split
    │    └── Network Split Configuration
    │        └── Port Ranges
    │            ├── Host: 49152–61439
    │            └── Slave: 61440–65535
    └── [*] Enable Host Power Save
         └── Host Power Save Configuration
            [*] Allow host to enter deep sleep.
             └── Deep Sleep Configuration
                 ├── Host Wakeup GPIO (6)
                 └── Host wakeup GPIO active low
```

3. Build and flash: `eh.py build && eh.py -p <HOST_PORT> flash monitor`

## Testing the Example

### Basic Power Save Test

1. Wait for both devices to connect to WiFi
2. Connect to AP using command, `sta_connect <SSID> <Password>`
3. On host console: type `host-power-save` 
4. Host goes to sleep, slave stays connected
5. On slave console: type `wake-up-host` to wake the host

### Smart Wake-up Test

1. Put host to sleep using `host-power-save`
2. Send dummy TCP/UDP packet to the device IP in host ports (49152–61439)  → host wakes automatically
3. Send dummy TCP/UDP packet to the device IP in slave ports (61440-65535) → slave handles without waking host

#### Network-Based Wakeup

To trigger host wakeup via network, send a UDP or TCP packet to any host-reserved port (49152–61439). A demo utility script is provided for testing:

```bash
# Navigate to the utility directory
cd host_wakeup_demo_using_udp_packet

# Send UDP packet to host port (default 123) - will wake up host
./host_wakeup_demo_using_test_packet.sh <device_ip_address>

# Send UDP packet to specific host port - will wake up host
./host_wakeup_demo_using_test_packet.sh <device_ip_address> 123

# Send UDP packet to slave port - will NOT wake up host
./host_wakeup_demo_using_test_packet.sh <device_ip_address> 62000
```

> [!TIP]
>
> You can design your packet routing & filtering as per application application use-case and deploy in slave firmware.
> Refer to the https://github.com/espressif/esp-hosted-mcu/blob/663d6631af6e7a6735755e2247ab60363fda87c8/slave/main/nw_split_router.c#L349

### Performance Test

1. Run iPerf server: `iperf -s -p 5001`
2. From another device: `iperf -c <device_ip> -p 5001`
3. Watch logs to see which device handles the traffic
For more details on iperf tests, refer [iperf test](README_iperf.md)

## Customizing for Other Hardware

#### Different Transport (e.g., SPI instead of SDIO)

**Slave side:**

```
Example Configuration
└── Bus Config in between Host and Co-processor
    └── Transport layer
        └── (X) SPI Full-duplex
```

**Host side:**

```
Component config
└── ESP-Hosted config
    └── Transport layer
        └── (X) SPI Full-duplex
```

Configure the specific GPIO pins for the chosen transport bus and Host wake up GPIO pin

#### Different MCU Combination

1. Change `eh.py set-target` to your MCU
2. Update GPIO pins in menuconfig to match your wiring
3. Make sure wake-up GPIO on host supports deep sleep wake-up

## Troubleshooting

**Host won't wake up:**

- Check GPIO wire connection between slave GPIO2 and host GPIO6
- Verify host GPIO6 supports RTC wake-up
- Check GPIO level settings match on both sides

**WiFi connection fails:**

- Double-check WiFi credentials in both devices
- Make sure both devices use the same WiFi settings

**Traffic not splitting correctly:**

- Check port range settings in slave menuconfig
- Monitor logs to see packet routing decisions
- Verify Network Split is enabled on slave

## Light Sleep Integration

For enhanced power savings, you can configure the slave device to enter **Light Sleep** when the host enters power save mode. This significantly reduces slave power consumption while maintaining fast wake-up times.

See the **[Light Sleep Integration Guide](README_light_sleep.md)** for detailed configuration instructions, dependency requirements, and troubleshooting.

**Quick Summary:**

- Light sleep significantly reduces slave power consumption compared to active mode
- Requires Power Management, FreeRTOS Tickless Idle, and optional peripheral powerdown
- Integrates automatically with host power save callbacks
- UART console may be unavailable during sleep if peripheral powerdown is enabled

## References

- [Light Sleep Integration Guide](README_light_sleep.md)
- [iperf test setup](README_iperf.md)

