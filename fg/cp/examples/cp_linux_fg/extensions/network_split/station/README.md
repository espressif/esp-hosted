| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# WiFi Station with Network Split Example

This example demonstrates how to use the ESP-Hosted network split functionality to route network traffic between the ESP32 (slave) and the host system.

## Overview

The network split functionality allows packets received by the ESP32's WiFi station interface to be selectively routed to either:

1. The ESP32's local LWIP stack (for local processing)
2. The host system's LWIP stack (for processing by the host)
3. Both the ESP32 and host LWIP stacks

This enables applications to run on both the ESP32 and the host, with each handling specific network traffic.

## How it Works

The example implements:

1. A custom WLAN station RX callback that intercepts incoming WiFi packets
2. A packet filter that decides where to route each packet based on:
   - Protocol (TCP, UDP, ICMP, ARP)
   - Port numbers (for TCP and UDP)
   - Configured rules

## Configuration

The example can be configured through `sdkconfig.defaults` or menuconfig:

### Network Split Configuration

- `CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_COPROCESSOR`: Route unfiltered packets to ESP32 LWIP
- `CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_HOST`: Route unfiltered packets to host LWIP
- `CONFIG_ESP_HOSTED_DEFAULT_NETWORK_STACK_BOTH`: Route unfiltered packets to both ESP32 and host LWIP

### Host Port Reservation

- `CONFIG_ESP_HOSTED_HOST_TCP_SRC_PORTS`: TCP source ports to forward to host (comma-separated)
- `CONFIG_ESP_HOSTED_HOST_TCP_DST_PORTS`: TCP destination ports to forward to host (comma-separated)
- `CONFIG_ESP_HOSTED_HOST_UDP_SRC_PORTS`: UDP source ports to forward to host (comma-separated)
- `CONFIG_ESP_HOSTED_HOST_UDP_DST_PORTS`: UDP destination ports to forward to host (comma-separated)

### WiFi Configuration

- `CONFIG_EXAMPLE_WIFI_SSID`: WiFi SSID
- `CONFIG_EXAMPLE_WIFI_PASSWORD`: WiFi password
- `CONFIG_EXAMPLE_WIFI_CONN_MAX_RETRY`: Maximum connection retry attempts

### LWIP Port Range Configuration

- `CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_START`: Start of TCP local port range
- `CONFIG_LWIP_TCP_LOCAL_PORT_RANGE_END`: End of TCP local port range
- `CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_START`: Start of TCP remote port range
- `CONFIG_LWIP_TCP_REMOTE_PORT_RANGE_END`: End of TCP remote port range
- `CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_START`: Start of UDP local port range
- `CONFIG_LWIP_UDP_LOCAL_PORT_RANGE_END`: End of UDP local port range
- `CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_START`: Start of UDP remote port range
- `CONFIG_LWIP_UDP_REMOTE_PORT_RANGE_END`: End of UDP remote port range

## Building and Running

1. Set up the ESP-IDF development environment
2. Configure the example:
   ```
   idf.py menuconfig
   ```
3. Build and flash:
   ```
   idf.py build flash monitor
   ```

## Expected Output

The example will:
1. Initialize the network coprocessor
2. Initialize the network split functionality
3. Connect to the configured WiFi network
4. Start routing packets according to the configured rules

You should see logs indicating:
- WiFi connection status
- Network split initialization
- Packet routing decisions (in verbose logging mode)

## Customization

To customize the packet filtering logic, modify the `app_nw_split_filter_packet` function in `app_nw_split_lwip_filter.c`.

## Troubleshooting

* If you see `Failed to connect to SSID:...`, check that your WiFi credentials are correct.
* If you see `Failed to initialize ESP-Hosted coprocessor`, check your hardware connections.
* If you see `Packet filtering failed, drop packet`, there might be an issue with the network split configuration.

## Further Customization

### Custom Port Forwarding Rules

You can define custom port forwarding rules by modifying the `CONFIG_ESP_HOSTED_HOST_*` options in the `sdkconfig.defaults` file. For example:

```
CONFIG_ESP_HOSTED_HOST_TCP_SRC_PORTS="22,80,443,8080"
CONFIG_ESP_HOSTED_HOST_TCP_DST_PORTS="22,80,443,8080"
```

This will forward TCP traffic with source or destination ports 22, 80, 443, or 8080 to the host.

### DHCP Configuration

You can configure whether DHCP is handled by the slave (ESP32) or the host by selecting the appropriate option in menuconfig under "ESP-Hosted FG Coprocessor - Extensions" → "Network Split configuration" → "Default DHCP client location".
