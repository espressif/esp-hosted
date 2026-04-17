# Setup for building ESP-Hosted-FG Coprocessor on Windows 11

Follow these steps to setup ESP-IDF and compile ESP-Hosted-FG coprocessor examples using the ESP-IDF configured Windows PowerShell Command Line tool.

## Prerequisites

1. **Install ESP-IDF**: Follow the [Standard Setup of Toolchain for Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html) guide.

2. **Install Git**: Ensure Git is installed and available in PATH.

## Setup Steps

### 1. Setup ESP-IDF for ESP-Hosted

Use the ESP-IDF [PowerShell Command Prompt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#using-the-command-prompt) to execute the setup script:

```powershell
cd esp_hosted_fg\coprocessor\sdk_esp_idf_setup
.\setup-idf.ps1
```

**Options:**
- Run `.\setup-idf.ps1 -h` for supported options
- To remove the ESP-IDF installation: `.\remove-idf.ps1`

### 2. Setup ESP-IDF Environment

```powershell
cd esp-idf
.\export.ps1
```

### 3. Build a Coprocessor Example

#### For WiFi-only coprocessor:
```powershell
cd ..\examples\minimal\wifi
idf.py set-target <chip_name>
idf.py build
```
<chip_name> is coprocessor target name. For example, esp32 , esp32c2, esp32c3, esp32c5, esp32c6, esp32s2, esp32s3

#### For Bluetooth-only coprocessor:
```powershell
cd ..\examples\minimal\bt
idf.py set-target <chip_name>
idf.py build
```

#### Other examples:
Please refer ..\examples\README.md for other supported examples

### 4. Flash and Monitor

```powershell
# Flash the firmware
idf.py flash

# Monitor serial output
idf.py monitor

# Or combine both steps
idf.py flash monitor
```

## Supported Target Chips

| Example | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 |
|---------|-------|----------|----------|----------|----------|----------|----------|
| WiFi Minimal | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| BT Minimal | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Custom RPC | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ |
| Network Split | ✅ | ✅ | ✅ | ❌ | ❌ | ✅ | ✅ |

## Configuration

### Optional: Configure the Project

```powershell
idf.py menuconfig
```

Key configuration options:
- **ESP-Hosted config**: Transport interface (SPI/SDIO), GPIO pins
- **WiFi config**: Country code, power save mode
- **Bluetooth config**: Controller mode, TX power

### Debug Options

Enable debug logging in menuconfig:
- **Component config** → **Log output** → **Default log verbosity** → **Debug**
- **ESP-Hosted config** → **Debug Options**

## Next Steps

1. **Choose your example**: Start with `minimal/wifi` or `minimal/bt`
2. **Read the example README**: Each example has detailed instructions
3. **Setup the host**: Configure Linux host or MCU host
4. **Test connectivity**: Use control path to verify functionality

## Directory Structure

```
esp_hosted_fg/
├── coprocessor/
│   ├── examples/
│   │   ├── minimal/
│   │   │   ├── wifi/          # WiFi-only coprocessor
│   │   │   └── bt/            # Bluetooth-only coprocessor
│   │   └── extensions/
│   │       ├── custom_rpc_msg/    # Custom RPC example
│   │       └── network_split/     # Network split examples
│   └── sdk_esp_idf_setup/     # ESP-IDF setup scripts (you are here)
```

## Additional Resources

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP-Hosted Documentation](../../docs/)
- [Example READMEs](../examples/) for detailed build instructions
