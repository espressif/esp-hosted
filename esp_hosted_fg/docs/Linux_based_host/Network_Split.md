
# Network Split Feature

<details>
<summary>Index</summary>

1.  [Introduction](#1-introduction)
2.  [Network Architecture](#2-network-architecture)
      - [2.1. High-level Overview](#21-high-level-overview)
          - [Packet routing entry](#packet-routing-entry)
          - [TCP Packets](#tcp-packets)
          - [UDP Packets](#udp-packets)
          - [ICMP & ARP Packets](#icmp--arp-packets)
      - [2.2. Packet Routing Summary](#22-packet-routing-summary)
3.  [Enable Network Split](#3-enable-network-split)
      - [3.1. Configure the ESP Firmware](#31-configure-the-esp-firmware-for-linux-host)
      - [3.2. Configure the Linux Host System](#32-configure-the-linux-host-system)
4.  [Troubleshooting](#4-troubleshooting)

</details>

## 1. Introduction

**Network Split** allows a **Linux Host** and an **ESP32 Slave** to share a single IP address and intelligently split network traffic between them. This means the ESP can continue handling selected network activity (like MQTT or DNS) even when the host is in a low-power "sleep" state, conserving energy without interrupting essential connectivity.

**Highlights:**

  * Port-based traffic splitting between the host and slave.
  * Shared IP address with intelligent packet routing.
  * Deep Packet Inspection (DPI) for custom handling and routing of packets.

##### Co-Processor Supported

| Supported Slave Targets | ESP32-C5 | ESP32-C6/C61 | ESP32-S2 | ESP32-S3 | ESP32 |
| :---------------------- | :------: | :----------: | :------: | :------: | :---: |

##### Communication Bus Supported

| Transports Supported | SPI | SDIO |
| :-------------------: | :-: | :--: |

-----

## 2. Network Architecture

The Network Split architecture divides incoming network traffic based on destination port ranges. By default:

  * The **Host** handles ports **49152-61439**.
  * The **Slave** handles ports **61440-65535**.

This port-based routing is transparent to remote endpoints, making it appear as a single device with one IP address.

### 2.1. High-level Overview

The core packet routing logic resides in `lwip_filter.c` and `lwip_filter.h` on the slave side, managing all packet routing decisions. The packet routing logic implementation:

https://github.com/espressif/esp-hosted/blob/ba39e6392f4f1c2e9ace3c5fa463bab34a42cb31/esp_hosted_fg/esp/esp_driver/network_adapter/main/lwip_filter.c#L347-L520

The routing logic includes special handling for:

  * **Priority ports** (e.g., SSH, HTTP, RTSP, DNS, NTP).
  * **MQTT messages** containing "wakeup-host" content.
  * **iPerf** performance testing traffic.
  * **DHCP client** traffic.
  * **ARP** and **ICMP** protocols.

The diagrams below illustrate the packet classification and routing logic:

#### Packet routing entry

```mermaid
%% Main Packet Classification
flowchart TD
    A[New Packet] --> B{MAC Broadcast?}
    B -->|Yes| C[Send to Slave Network]
    B -->|No| D{IP Packet?}
    D -->|Yes| E[Protocol Processing]
    D -->|No| F[ARP Processing]

    E --> G{TCP?}
    E --> H{UDP?}
    E --> I{ICMP?}

    classDef slaveNode fill:#d9f7be,stroke:#389e0d
    classDef hostNode fill:#d6e4ff,stroke:#1d39c4
    classDef invalidNode fill:#ffccc7,stroke:#cf1322
    classDef bothNode fill:#fff1b8,stroke:#d4b106
    classDef defaultNode fill:#fff,stroke:#333

    class C slaveNode
```

#### TCP Packets

```mermaid
%% TCP Processing
flowchart TD
    G[TCP Packet] --> G1{Priority Port?<br/>SSH, RTSP, etc.}
    G1 -->|Yes| K[Send to Host Network]

    G1 -->|No| G2{Dest Port == iPerf<br/>5001?}
    G2 -->|Yes| G3{Local TCP Port<br/>Open?}
    G3 -->|Yes| C[Send to Slave Network]
    G3 -->|No| G4{Host Sleeping?}
    G4 -->|No| K
    G4 -->|Yes| C

    G2 -->|No| G5{Is Remote TCP Port?}
    G5 -->|Yes| G6{Host Sleeping?}
    G6 -->|Yes| G7{Source Port == MQTT<br/>1883?}
    G7 -->|Yes| G8{"Contains<br/>'wakeup-host'?"}
    G8 -->|Yes| K
    G8 -->|No| L[Drop Packet]
    G7 -->|No| L
    G6 -->|No| K

    G5 -->|No| G9{Is Local TCP Port?}
    G9 -->|Yes| C
    G9 -->|No| D[Slave's Internal LWIP Stack]

    classDef slaveNode fill:#d9f7be,stroke:#389e0d
    classDef hostNode fill:#d6e4ff,stroke:#1d39c4
    classDef invalidNode fill:#ffccc7,stroke:#cf1322
    classDef defaultNode fill:#fff,stroke:#333

    class K hostNode
    class C slaveNode
    class L invalidNode
    class D defaultNode
```

#### UDP Packets

```mermaid
%% UDP Processing
flowchart TD
    H[UDP Packet] --> H1{Priority Port?<br/>DNS, NTP, etc.}
    H1 -->|Yes| K[Send to Host Network]

    H1 -->|No| H2{Dest Port == iPerf<br/>5001?}
    H2 -->|Yes| H3{Local UDP Port<br/>Open?}
    H3 -->|Yes| C[Send to Slave Network]
    H3 -->|No| H4{Host Sleeping?}
    H4 -->|No| K
    H4 -->|Yes| C

    H2 -->|No| H5{DHCP Client Port?}
    H5 -->|Yes| H6["Send to DHCP Handler<br/>(Host or Slave)"]

    H5 -->|No| H7{Is Remote UDP Port?}
    H7 -->|Yes| H8{Host Sleeping?}
    H8 -->|Yes| L[Drop Packet]
    H8 -->|No| K

    H7 -->|No| H9{Is Local UDP Port?}
    H9 -->|Yes| C
    H9 -->|No| D[Slave's Internal LWIP Stack]

    classDef slaveNode fill:#d9f7be,stroke:#389e0d
    classDef hostNode fill:#d6e4ff,stroke:#1d39c4
    classDef invalidNode fill:#ffccc7,stroke:#cf1322
    classDef defaultNode fill:#fff,stroke:#333

    class K hostNode
    class C slaveNode
    class L invalidNode
    class D defaultNode
    class H6 defaultNode
```

#### ICMP & ARP Packets

```mermaid
%% ICMP & ARP Processing
flowchart TD
    subgraph ICMP
    I[ICMP Packet] --> I1{Ping Request?}
    I1 -->|Yes| C[Send to Slave Network]
    I1 -->|No| I2{Ping Response?}
    I2 -->|Yes| I3{Host Sleeping?}
    I3 -->|Yes| C
    I3 -->|No| N[Send to Both Networks]
    end

    subgraph ARP
    F[ARP Packet] --> F1{ARP Request?}
    F1 -->|Yes| C
    F1 -->|No| F2{Host Sleeping?}
    F2 -->|Yes| C
    F2 -->|No| N
    end

    classDef slaveNode fill:#d9f7be,stroke:#389e0d
    classDef bothNode fill:#fff1b8,stroke:#d4b106

    class C slaveNode
    class N bothNode
```

The slave determines if the host is "sleeping" by monitoring the host's activity or a dedicated control signal, ensuring packets are routed appropriately to wake the host when necessary or handled by the slave directly.

### 2.2. Packet Routing Summary

All incoming packets are evaluated by `lwip_filter.c`. The slave routes packets based on these rules:

| Packet Type                           | Destination Port Condition                 | Routed To                          |
| :------------------------------------ | :----------------------------------------- | :--------------------------------- |
| Broadcast, ARP Request, ICMP Request  | N/A                                        | Slave Network Stack                |
| TCP/UDP                               | Listed in Static Port Forwarding           | Host Network Stack                 |
| TCP/UDP                               | Within Host Port Range (49152-61439)       | Host Network Stack                 |
| TCP/UDP                               | Within Slave Port Range (61440-65535)      | Slave Network Stack                |
| Others                                | Not matched by any rule                    | Default Destination (as configured)|
| iperf (-p 5001 default port)          | If 5001 is open at slave                   | Slave Network Stack. Else Host Network Stack |

-----

## 3. Enable Network Split

### 3.1. Configure the ESP Firmware (for Linux Host)

The firmware source is located at: [esp\_hosted\_fg/esp/esp\_driver/network\_adapter](https://github.com/espressif/esp-hosted/tree/master/esp_hosted_fg/esp/esp_driver/network_adapter)

##### SDK Setup

  * **Linux / Mac users:**
    ```bash
    bash setup-idf.sh
    ```
  * **Windows users:**
    Refer to [setup\_windows11.md](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/esp/esp_driver/setup_windows11.md).

##### Configure Network Split Mode

Run `idf.py menuconfig` from your firmware directory.

Enable the feature:

```
Example Configuration
└── [*] Allow Network Split using packet port number
```

(Optional) Customize under `Network Split Configuration`:

```
├── Extra port forwarding to host (static)
│   ├── TCP dst: 22,8554  (e.g., SSH, RTSP)
│   └── UDP dst: 53,123   (e.g., DNS, NTP)
├── Port Ranges
│   ├── Host: 49152–61439
│   └── Slave: 61440–65535
└── Default Destination: slave / host / both
```

##### Build and Flash ESP Firmware

```bash
idf.py -p /dev/ttyUSBx build flash monitor
```

(Replace `/dev/ttyUSBx` with your ESP device’s serial port.)

-----

### 3.2. Configure the Linux Host System

The Linux host code is located at: [esp\_hosted\_fg/host/linux](https://www.google.com/search?q=https://github.com/espressif/esp-hosted/tree/master/esp_hosted_fg/host/linux)

##### Sysctl Configuration

Edit `/etc/sysctl.conf`:

```bash
net.ipv4.ip_local_port_range = 49152 61439
```

Apply the changes:

```bash
sudo sysctl -p
```

> [!TIP]
> The port ranges configured on the host and slave **must match exactly** to avoid routing issues.

##### Setup Linux Host Driver and Application

  * **Kernel Module:**
    Build using the [Makefile](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/host/linux/host_driver/esp32/Makefile) or the [helper script](https://github.com/espressif/esp-hosted/blob/master/esp_hosted_fg/host/linux/host_control/rpi_init.sh) (`rpi_init.sh`) for Raspberry Pi.

  * **User Application:**
    Build:

    ```bash
    cd esp_hosted_fg/host/linux/host_control
    make hosted_shell
    ```

    Run:

    ```bash
    sudo ./hosted_shell.out
    ```

    Connect the ESP to Wi-Fi:

    ```bash
    connect_ap <SSID> <Password>
    ```

    This command creates the `ethsta0` network interface, which is then ready for standard socket-based tools like `iperf`.

-----

## 4. Troubleshooting

**Debug Logging:**
To enable verbose logging for the packet routing logic on the slave:

```c
esp_log_level_set("lwip_filter", ESP_LOG_VERBOSE);
```

**Network Monitoring:**
Use `tcpdump` to monitor traffic on the host interface:

```bash
sudo tcpdump -i ethsta0 port 80
```

Check the configured local port range on the host:

```bash
cat /proc/sys/net/ipv4/ip_local_port_range
```

**Common Issues:**

  * **Packets not getting delivered to host or slave:**
      * Check if the socket being opened is using reserved port ranges.
      * If so, you can either handle the packet within the existing packet routing logic or add the destination port to the `Extra port forwarding to host` configuration in `menuconfig`.
  * **DHCP not completing:**
      * Verify where the DHCP packet is destined by checking the `lwip_filter.c` logic related to DHCP (refer to lines [38-42 here](https://github.com/espressif/esp-hosted/blob/ba39e6392f4f1c2e9ace3c5fa463bab34a42cb31/esp_hosted_fg/esp/esp_driver/network_adapter/main/lwip_filter.c#L38-L42)).
  * **Same port needing to be used by both slave and host:**
      * This is already supported for iperf on port 5001. You can amend the packet routing logic in `lwip_filter.c` to handle other specific ports similarly if needed.
