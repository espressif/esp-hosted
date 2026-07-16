# Peer Data Transfer Example

## Why This Example

This example demonstrates how to send and receive **raw binary data** between a host device and a coprocessor. Unlike standard communication which uses predefined message formats, this approach lets you send any data directly—ideal for peer-to-peer applications or when you need maximum control over your data format.

## APIs in Focus

```c
// Send data
esp_hosted_send_custom_data(data, length);

// Receive data
esp_hosted_register_rx_callback_custom_data(my_callback);
```

The API names are identical on both the coprocessor and host sides.

## Supported Platforms and Transports

### Supported Coprocessors

| Coprocessor | ESP32 | ESP32-C Series | ESP32-S Series |
| :----------: | :---: | :------------: | :------------: |
| Support     | Yes   | Yes            | Yes            |

### Supported Host Devices

| Host Device | ESP32-P4 | ESP32-H2 | Other MCUs |
| :---------: | :------: | :------: | :--------: |
| Support     | Yes      | Yes      | Yes        |

### Supported Connection Types

| Connection | SDIO | SPI Full-Duplex | SPI Half-Duplex | UART |
| :--------: | :--- | :-------------- | :-------------- | :--- |
| Support    | Yes  | Yes             | Yes             | Yes  |

## Coprocessor Setup

### 1. Enable Example in Menuconfig

Navigate to your coprocessor project directory and open menuconfig:

```bash
cd <project_path>/slave
eh.py menuconfig
```

Enable the peer data transfer example:

```
Example Configuration → Additional higher layer examples to run → Select Examples to run → [*] Peer Data Transfer Example
```

**Note:** Peer data transfer feature is enabled by default in ESP-Hosted. This example demonstrates how to use it by registering handlers and echoing data back to the host.

### 2. Flash the Coprocessor

```bash
eh.py -p <slave_port> build flash monitor
```

Replace `<slave_port>` with your coprocessor's serial port (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).

## Host Setup

### 1. Configure Host Project

Navigate to your host project directory:

```bash
cd <project_path>/host
eh.py menuconfig
```

Configure the connection interface (SDIO, SPI, or UART) and host MCU settings as per your hardware setup.

### 2. Flash the Host

```bash
eh.py -p <host_port> build flash monitor
```

Replace `<host_port>` with your host device's serial port

## How It Works

This example sends data packets of increasing size from the host to the coprocessor. Packet sizes range from 1 byte up to 8166 bytes. The coprocessor receives each packet, verifies it, and sends it back. Both devices check that the data arrived correctly and display results.

**Expected Output:**

```
========================================
Peer Data Transfer Test (max: 8166 bytes)
========================================
copro <-- host : 1 byte stream, sent Y
host --> copro : 1 byte stream received, verification: Y
copro --> host : 1 byte stream received, verification: Y

copro <-- host : 512 byte stream, sent Y
host --> copro : 512 byte stream received, verification: Y
copro --> host : 512 byte stream received, verification: Y

copro <-- host : 4096 byte stream, sent Y
host --> copro : 4096 byte stream received, verification: Y
copro --> host : 4096 byte stream received, verification: Y

copro <-- host : 8166 byte stream, sent Y
host --> copro : 8166 byte stream received, verification: Y
copro --> host : 8166 byte stream received, verification: Y

copro <-- host : 8200 byte stream (exceeds limit - skipped)

========================================
COPROCESSOR-SIDE VERIFICATION
========================================
Packets received: 10
Bytes received:   13643
Data validation:  Y ALL PASSED
========================================

========================================
TEST SUMMARY
========================================
Result:           Y PASS
========================================
```

## Limitations

- **Maximum payload size**: 8166 bytes per packet. For larger data, break it into smaller chunks in your application.
- **No automatic formatting**: You must prepare and interpret the raw data yourself.
- **Struct alignment**: Use `__attribute__((packed))` to ensure data layout is consistent.

## See Also

- Coprocessor example: `slave/main/example_peer_data_transfer.c`
- Coprocessor API: `slave/main/esp_hosted_peer_data.h`
