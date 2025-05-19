# Auto Network Setup

Auto IP Restore ensures seamless collaboration between your ESP device and host system by automating network configuration — ideal for both development and deployment scenarios.

<details>
<summary>Index</summary>

1. [Overview](#1-overview)  
   - [1.1. What is Auto IP Restore?](#11-what-is-auto-ip-restore)  
   - [1.2. How It Works](#12-how-it-works)  

2. [Getting Started](#2-getting-started)  
   - [2.1. ESP Wi-Fi Setup](#21-esp-wi-fi-setup)  
   - [2.2. Running the Host Application](#22-running-the-host-application)  

3. [System Components](#3-system-components)  
   - [3.1. Host Kernel Module](#31-host-kernel-module)  
   - [3.2. User-Space Applications](#32-user-space-applications)  
   - [3.3. Host System Configuration](#33-host-system-configuration)  

4. [Implementation Details](#4-implementation-details)  

5. [Future Enhancements](#5-future-enhancements)

</details>

---

## 1. Overview

### 1.1 What is Auto IP Restore?

Auto IP Restore allows the host system to retrieve and apply network configuration from a connected ESP device. It simplifies Wi-Fi restoration after reboot or sleep cycles.

#### ✨ Key Benefits

- ✅ No manual IP setup required  
- ✅ Blazingly Fast connectivity on boot or wake-up from power saving
- ✅ Background daemon supported  
- ✅ Extremely easy for user  

---

### 1.2 How It Works

Auto IP Restore involves tight integration between the host (Linux) and the ESP device:

```mermaid
sequenceDiagram
    participant Host
    participant KMod as Kernel Module
    participant Daemon
    participant ESP

    participant Router as WiFi Router
    Note over Host,Router: System Bootup
    Note over ESP: ESP Bootup
    ESP->>Router: Auto-connect to Wi-Fi
    Router-->>ESP: DHCP IP assigned

    Note over Host,Daemon: Host Bootup
    Host->>KMod: Load Kernel Module
    KMod->>ESP: Set-up ESP-Hosted transport
    ESP-->>KMod: Transport Up

    Note over Host,Router: IP Recovery
    Note over Daemon: Start User Space App
    Daemon->>ESP: Query MAC Address
    ESP-->>Daemon: Return MAC
    Daemon->>ESP: Query IP Config
    ESP-->>Daemon: Return IP/DNS Info
    Daemon->>Host: Configure ethsta0 interface
    Daemon->>Host: Update resolv.conf
    Daemon->>Host: Update default route

    Note over Host,Router: Network Ready
    Host<<->>Router: Traffic begins

    Daemon->>Daemon: Subscribe network change event
    Router-->>ESP: Change in DHCP or Wi-Fi
    ESP-->>Daemon: Send network change Event
    Daemon->>Host: Update IP / Router / DNS / Network accordingly

````

The ESP fetches IP configuration from the router and stores it in flash. On host boot, the daemon queries the ESP and sets the host interface (`ethsta0`) accordingly.

---

## 2. Getting Started

### 2.1 ESP Wi-Fi Setup

You can configure the ESP via:

* 📦 **Stored Config**: Automatically connects using credentials in flash
* 🛠 **CLI**:

  ```sh
  coprocessor> sta <ssid> <password>
  ```
* ⚙️ **Kconfig**: Default Wi-Fi set in [Kconfig.projbuild](../../../esp_hosted_fg/esp/esp_driver/network_adapter/main/Kconfig.projbuild)
* 💻 **Host App**: Configure via `hosted_shell.out` or other RPC-capable tools

Once configured, the ESP uses its DHCP client to get IP/gateway/DNS from the router.

---

### 2.2 Running the Host Application

Two main user-space options exist:

#### Option A: Development (Interactive Shell)

```bash
sudo ./hosted_shell.out
```

Provides interactive CLI with tab completion and command help.

#### Option B: Production (Daemon)

1. Install daemon:

   ```bash
   sudo cp hosted_daemon.out /usr/local/bin/
   ```

2. Create a systemd service:

   ```bash
   sudo nano /etc/systemd/system/hosted-daemon.service
   ```

   ```ini
   [Unit]
   Description=ESP Hosted Auto IP Restore Daemon
   After=network.target

   [Service]
   Type=forking
   ExecStart=/usr/local/bin/hosted_daemon.out
   Restart=always
   StandardOutput=syslog
   StandardError=syslog
   SyslogIdentifier=hosted-daemon

   [Install]
   WantedBy=multi-user.target
   ```

3. Enable and start the service:

   ```bash
   sudo systemctl enable hosted-daemon
   sudo systemctl start hosted-daemon
   ```

4. Monitor logs:

   ```bash
   journalctl -u hosted-daemon -f
   ```

---

## 3. System Components

### 3.1 Host Kernel Module

**Path**: [esp_hosted_fg/host/linux/host_driver/esp32](../../../esp_hosted_fg/host/linux/host_driver/esp32)

* Initializes ESP transport (SPI or SDIO)
* Creates the virtual `ethsta0` interface
* Triggers ESP power-up sequence
* Kernel mode cannot/should not configure IP directly (done via user-space)

---

### 3.2 User-Space Applications

**Path**: [esp_hosted_fg/host/linux/host_control/c_support/](../../../esp_hosted_fg/host/linux/host_control/c_support/)

#### A. Hosted Daemon (`hosted_daemon.out`)

* Background service for IP sync
* Key files:
  - [hosted_daemon.c](../../../esp_hosted_fg/host/linux/host_control/c_support/hosted_daemon.c)
  - [Makefile](../../../esp_hosted_fg/host/linux/host_control/c_support/Makefile)

#### B. Hosted CLI (`hosted_shell.out`)

* Interactive CLI
* Uses replxx for tab completion
* Good for testing or scripting
* Key files:
  - Install replxx library from https://github.com/AmokHuginnsson/replxx
  - [hosted_shell.c](../../../esp_hosted_fg/host/linux/host_control/c_support/hosted_shell.c)
  - [Makefile](../../../esp_hosted_fg/host/linux/host_control/c_support/Makefile)

---

### 3.3 Host System Configuration

Configuration changes made by user-space apps include:

* `/etc/resolv.conf` — sets DNS from ESP
* Routing table — sets default gateway
* `ethsta0` IP — applies ESP-provided IP

These use helper functions from:
  - [nw_helper_func.c](../../../esp_hosted_fg/host/linux/host_control/c_support/nw_helper_func.c)

---

## 4. Implementation Details

Daemon on boot-up, calls function to fetch IP from ESP:

```c
static int fetch_ip_addr_from_slave(void) {
    ctrl_cmd_t *resp = get_dhcp_dns_status(req);
    if (resp->resp_event_status == SUCCESS) {
        strncpy(sta_ip_str, resp->u.dhcp_dns_status.dhcp_ip, MAC_ADDR_LENGTH);
        // Set IP, DNS, and route on ethsta0
    }
    return resp_cb(resp);
}
```

Also subscribes to DHCP-DNS events, when received, automatically brings network up and down
Key Points:

* ESP sends `CTRL_EVENT_DHCP_DNS_STATUS` when IP changes
* Daemon auto-applies new settings
* Host DHCP client optional (can override daemon config)

> ℹ️ Make sure a user-space app (daemon or shell) is running to handle events.

---

## 5. Future Enhancements

* 🔋 Power management integration
* 🌐 IPv6 support
