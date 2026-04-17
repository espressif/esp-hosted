| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# ESP-Hosted Network Split iPerf Example

This example demonstrates the network split functionality with iPerf testing capabilities. Network split allows both the host and the ESP32 to share the same IP address but handle different port ranges.

## What is Network Split?

Network split is a feature that allows the ESP32 (slave) and the host to share the same IP address but handle different port ranges:

- **Host ports**: 49152-61439
- **Slave ports**: 61440-65535

This enables applications to be distributed between the host and the ESP32 while appearing as a single device on the network.

## Features

- **iPerf Server**: Run iPerf server on ESP32 to test network performance
- **Echo Server**: Simple TCP echo server running on port 61500
- **HTTP Server**: Basic HTTP server running on port 61600
- **Port-based Routing**: Packets are automatically routed to the appropriate device based on port numbers
- **Shared IP Address**: Both devices share the same IP address

## Hardware Requirements

- ESP32 development board
- Host device (Linux/Raspberry Pi) connected to ESP32 via SPI or SDIO

## Building the Example

1. Set up ESP-IDF environment
2. Navigate to the example directory:
   ```
   cd esp_hosted_fg/coprocessor/examples/extensions/network_split/iperf
   ```
3. Configure the project:
   ```
   idf.py menuconfig
   ```
   - Ensure "ESP-Hosted Network Split iPerf Example Configuration" → "Enable Network Split Mode" is enabled
   - Configure your WiFi credentials under "ESP-Hosted Network Split iPerf Example Configuration"
   - Configure SPI pins if using SPI interface

4. Build and flash:
   ```
   idf.py build flash monitor
   ```

## Using the Example

After flashing and booting, the ESP32 will:

1. Initialize the network split functionality
2. Connect to the configured WiFi network
3. Start the ESP-Hosted coprocessor service
4. Start the demo servers (echo and HTTP)
5. Launch the console interface

### Console Commands

The example provides a command-line interface with the following commands:

- `help`: Show available commands
- `wifi connect <ssid> <password>`: Connect to a WiFi network
- `iperf -s`: Start iPerf server
- `demo-servers start`: Start demo servers (echo and HTTP)
- `demo-servers stop`: Stop demo servers
- `demo-servers status`: Check demo server status
- `network-split-info`: Show network split configuration

### Testing Network Split

1. Connect the ESP32 to WiFi using the console or pre-configured settings
2. Note the IP address assigned to the ESP32
3. Test the slave services:
   - iPerf: `iperf3 -c <ip> -p 5001`
   - Echo server: `telnet <ip> 61500`
   - HTTP server: `curl http://<ip>:61600`
4. Test the host services (if host is running appropriate services):
   - SSH: `ssh user@<ip>` (port 22 is forwarded to host)
   - HTTP: `curl http://<ip>:80` (port 80 is forwarded to host)

## Port Forwarding Rules

The example configures the following port forwarding rules:

- TCP source ports forwarded to host: 22, 8554
- TCP destination ports forwarded to host: 22, 80, 443, 8080, 8554
- UDP destination ports forwarded to host: 53, 123

All other traffic is routed based on the port ranges defined above.

## Troubleshooting

### Connection Issues

- Ensure the WiFi credentials are correct
- Check that the ESP32 has obtained an IP address
- Verify that the host device is properly connected to the ESP32

### Network Split Issues

- Ensure CONFIG_ESP_HOSTED_CP_NETWORK_SPLIT_ENABLED is set to 'y' in sdkconfig
- Check that the port ranges are correctly configured
- Verify that the lwIP hooks are properly installed

### Performance Issues

- Run `iperf -s` on the ESP32 and test with `iperf3 -c <ip> -p 5001` from a client
- Check WiFi signal strength and connection quality
- Adjust buffer sizes if necessary

## Technical Details

The network split functionality works by:

1. Intercepting incoming packets at the Ethernet layer
2. Examining TCP/UDP headers to determine destination ports
3. Routing packets to either the host or slave based on port ranges
4. Using a shared IP address for both devices

This implementation uses ESP-IDF's lwIP hooks to filter and route packets appropriately.

## License

This example is licensed under the Apache License 2.0.

## Overview

Network Split is a sophisticated feature that enables:
- **Shared IP Address**: Both host and slave appear as a single device to the network
- **Port-based Traffic Routing**: Traffic is intelligently routed based on destination ports
- **Independent Processing**: Each system handles its assigned traffic independently
- **Transparent Operation**: Remote endpoints see only one IP address

### Port Range Configuration

| System | Port Range | Purpose |
|--------|------------|---------|
| **Host** | 49152-61439 | Host applications and services |
| **Slave** | 61440-65535 | ESP slave applications and services |

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Remote Client │    │ Network Split    │    │   Host System   │
│                 │    │ ESP Slave        │    │   (Linux)       │
│                 │    │                  │    │                 │
├─────────────────┤    ├──────────────────┤    ├─────────────────┤
│ Single IP View  │───▶│ Port Filter      │───▶│ Host Services   │
│                 │    │                  │    │ (49152-61439)   │
│ iperf3 -c IP    │    │ ┌──────────────┐ │    │                 │
│ telnet IP 61500 │    │ │ Slave LWIP   │ │    │                 │
│ curl IP:61600   │    │ │ (61440-65535)│ │    │                 │
│                 │    │ └──────────────┘ │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## Features Demonstrated

### 1. iPerf Performance Testing
- **Port 5001**: iPerf server running on slave LWIP stack
- **High Throughput**: Demonstrates network performance with split architecture
- **Statistics**: Real-time WiFi statistics during testing

### 2. Demo Services (Slave Ports)
- **TCP Echo Server (61500)**: Simple echo service for connectivity testing
- **HTTP Server (61600)**: Web interface showing network split status
- **iPerf Server (5001)**: Performance testing endpoint

### 3. Interactive CLI
- **WiFi Management**: Connect to networks, scan, configure
- **Demo Control**: Start/stop demo servers
- **Network Information**: View port configurations and routing
- **System Diagnostics**: Memory, tasks, and performance monitoring

## Quick Start

### 1. Build and Flash
```bash
cd esp_hosted_fg/coprocessor/examples/extensions/network_split/iperf
idf.py build
idf.py flash monitor
```

### 2. Configure Network Split
The example includes default network split configuration. To customize:
```bash
idf.py menuconfig
# Navigate to: Component config → ESP-Hosted Configuration → Network Split
```

### 3. Connect to WiFi
```
network-split-iperf> wifi connect MyNetwork MyPassword
```

### 4. Start Demo Services
```
network-split-iperf> demo-servers start
network-split-iperf> iperf -s
```

### 5. Test from External Client
```bash
# Test iPerf (slave)
iperf3 -c <esp_ip> -p 5001

# Test echo server (slave)
telnet <esp_ip> 61500

# Test HTTP server (slave)
curl http://<esp_ip>:61600

# Test host services (would go to host if available)
nc <esp_ip> 50000
```

## CLI Commands

### WiFi Commands
```bash
wifi scan                           # Scan for networks
wifi connect <ssid> <password>      # Connect to network
wifi disconnect                     # Disconnect from network
wifi status                         # Show connection status
```

### iPerf Commands
```bash
iperf -s                            # Start iPerf server
iperf -c <ip>                       # Start iPerf client
iperf -a                            # Abort iPerf
```

### Network Split Demo Commands
```bash
demo-servers start                  # Start demo servers
demo-servers stop                   # Stop demo servers  
demo-servers status                 # Show server status
network-split-info                  # Show configuration info
```

### System Commands
```bash
help                                # Show all commands
mem-dump                            # Memory statistics
task-dump                           # Task information
reboot                              # Restart device
```

## Testing Network Split

### 1. Verify Port Routing
Test that traffic to different ports goes to the correct system:

```bash
# These should be handled by slave (ESP)
iperf3 -c <ip> -p 5001             # iPerf
telnet <ip> 61500                   # Echo server
curl http://<ip>:61600              # HTTP server
nc -u <ip> 61700                    # Any slave port

# These would be handled by host (if host is running)
nc <ip> 50000                       # Any host port
```

### 2. Performance Testing
```bash
# TCP throughput test
iperf3 -c <ip> -p 5001 -t 30

# UDP throughput test  
iperf3 -c <ip> -p 5001 -u -b 100M -t 30

# Bidirectional test
iperf3 -c <ip> -p 5001 --bidir -t 30
```

### 3. Concurrent Connections
Test multiple simultaneous connections to verify proper routing:
```bash
# Terminal 1: iPerf test
iperf3 -c <ip> -p 5001 -t 60

# Terminal 2: HTTP requests
while true; do curl http://<ip>:61600; sleep 1; done

# Terminal 3: Echo server
telnet <ip> 61500
```

## Configuration Options

### Network Split Settings
```c
// Port ranges (configurable in menuconfig)
CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START=61440    // Slave start
CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END=65535      // Slave end
CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START=49152   // Host start  
CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END=61439     // Host end

// Enable network split
CONFIG_ESP_HOSTED_CP_NETWORK_SPLIT_ENABLED=y
```

### Default Routing
Configure what happens to unfiltered packets:
- **Send to Slave**: Default routing to slave LWIP
- **Send to Host**: Default routing to host LWIP  
- **Send to Both**: Duplicate to both systems

## Troubleshooting

### Common Issues

1. **Network Split Not Working**
   - Verify `CONFIG_ESP_HOSTED_CP_NETWORK_SPLIT_ENABLED=y`
   - Check port range configuration
   - Ensure proper WiFi connection

2. **iPerf Connection Failed**
   - Confirm iPerf server is running: `iperf -s`
   - Check firewall settings on client
   - Verify IP address and port 5001

3. **Demo Servers Not Accessible**
   - Start servers: `demo-servers start`
   - Check server status: `demo-servers status`
   - Verify ports 61500, 61600 are not blocked

4. **Poor Performance**
   - Disable power saving: Already done in example
   - Check WiFi signal strength
   - Monitor with `mem-dump` and `task-dump`

### Debug Commands
```bash
# Check memory usage
mem-dump

# Monitor tasks
task-dump

# Network information
network-split-info

# WiFi statistics (during iPerf)
# Automatically shown with iPerf hook
```

## Advanced Usage

### Custom Port Forwarding
The example can be extended to support custom port forwarding rules for specific applications.

### Integration with Host Services
When running with a Linux host, the host can run complementary services on its port range (49152-61439).

### Performance Optimization
- Adjust task priorities for high-throughput applications
- Configure LWIP buffer sizes for your use case
- Optimize WiFi settings for your environment

## Benefits of Network Split

1. **Resource Efficiency**: Slave handles lightweight tasks, host handles complex processing
2. **Power Management**: Host can sleep while slave maintains network presence  
3. **Service Distribution**: Different services can run optimally on each system
4. **Transparent Operation**: Single IP address simplifies network configuration
5. **Scalability**: Easy to add new services without IP conflicts

This example provides a comprehensive demonstration of ESP-Hosted Network Split capabilities, showing how to effectively utilize this powerful feature for distributed network applications.

## Note about iperf version
The iperf example doesn't support all features in standard iperf. It's compatible with iperf version 2.x.

- Refer to the ESP Component Registry iperf-cmd page for more information: https://components.espressif.com/components/espressif/iperf-cmd

## Note about 80MHz flash frequency (ESP32)
The iperf can get better throughput if the SPI flash frequency is set to 80MHz, but the system may crash in 80MHz mode for ESP-WROVER-KIT.
Removing R140\~R145 from the board can fix this issue. Currently the default SPI frequency is set to 40MHz, if you want to change the SPI flash
frequency to 80MHz, please make sure R140\~R145 are removed from ESP-WROVER-KIT or use ESP32 DevKitC.

## Introduction
This example implements the protocol used by the common performance measurement tool [iPerf](https://iperf.fr/).
Performance can be measured between two ESP targets running this example, or between a single ESP target and a computer running the iPerf tool

Demo steps to test station TCP Tx performance:

- Configure in `menuconfig` which serial output you are using. Execute `idf.py menuconfig` and go to `Component config/ESP System Settings/Channel for console output`, then select the appropriate interface. By default the UART0 interface is used, this means that for example in the ESP32-S3-DevKitC-1 or ESP32-C6-DevKitC-1 you should connect to the micro-usb connector labeled as UART and not to the one labeled as USB. To use the one labeled as USB you should change the aforementioned setting to `USB Serial/JTAG Controller`.

- Build and flash the iperf example with `sdkconfig.defaults`, which contains performance test specific configurations
  - Use `help` for detailed command usage information.

- Run the demo as station mode and join the target AP
  - `sta_connect <ssid> <password>`
  - NOTE: the dut is started in station mode by default. If you want to use the dut as softap, please set wifi mode first:
    - `wifi_mode ap`
    - `ap_set <dut_ap_ssid> <dut_ap_password>`

- Run iperf as server on AP side
  - `iperf -s -i 3`

- Run iperf as client on ESP side
  - `iperf -c 192.168.10.42 -i 3 -t 60`

The console output, which is printed by station TCP RX throughput test, looks like:

  ```
  iperf> sta_connect testap-11 ********
  I (36836) WIFI: Connecting to testap-11...
  I (36839) WIFI: DONE.WIFI_CONNECT_START,OK.
  iperf> I (39248) WIFI: WIFI_EVENT_STA_DISCONNECTED! reason: 201
  I (39249) WIFI: trying to reconnect...
  I (41811) wifi:new:<11,2>, old:<1,0>, ap:<255,255>, sta:<11,2>, prof:1, snd_ch_cfg:0x0
  I (41813) wifi:state: init -> auth (0xb0)
  I (41816) wifi:state: auth -> assoc (0x0)
  I (41840) wifi:state: assoc -> run (0x10)
  I (41847) wifi:<ba-add>idx:0 (ifx:0, 30:5a:3a:74:90:f0), tid:0, ssn:0, winSize:64
  I (41914) wifi:connected with testap-11, aid = 1, channel 11, 40D, bssid = 30:5a:3a:74:90:f0
  I (41915) wifi:security: WPA2-PSK, phy: bgn, rssi: -34
  I (41926) wifi:pm start, type: 0
  
  I (41927) wifi:dp: 1, bi: 102400, li: 3, scale listen interval from 307200 us to 307200 us
  I (41929) WIFI: WIFI_EVENT_STA_CONNECTED!
  I (41983) wifi:AP's beacon interval = 102400 us, DTIM period = 3
  I (42929) esp_netif_handlers: sta ip: 192.168.1.79, mask: 255.255.255.0, gw: 192.168.1.1
  I (42930) WIFI: IP_EVENT_STA_GOT_IP: Interface "sta" address: 192.168.1.79
  I (42942) WIFI: - IPv4 address: 192.168.1.79,
  iperf> 
  iperf> iperf -s -i 2
  I (84810) IPERF: mode=tcp-server sip=0.0.0.0:5001,             dip=0.0.0.0:5001,               interval=2, time=30
  I (84812) iperf: Socket created
  iperf> I (87967) iperf: accept: 192.168.1.2,43726
  
  Interval       Bandwidth
  0.0- 2.0 sec  24.36 Mbits/sec
  2.0- 4.0 sec  23.38 Mbits/sec
  4.0- 6.0 sec  24.02 Mbits/sec
  6.0- 8.0 sec  25.27 Mbits/sec
  8.0-10.0 sec  23.84 Mbits/sec
  ```

Steps to test station/soft-AP TCP/UDP RX/TX throughput are similar as test steps in station TCP TX.

See the README.md file in the upper level 'examples' directory for more information about examples.

