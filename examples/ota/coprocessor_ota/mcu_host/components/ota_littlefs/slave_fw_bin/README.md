# Slave Firmware Binaries for LittleFS OTA

This directory should contain ESP32 slave firmware `.bin` files for LittleFS-based OTA updates.

## Usage

1. Copy your slave firmware binary (e.g., `network_adapter.bin`) to this directory
2. Build the project with `CONFIG_OTA_METHOD_LITTLEFS=y`
3. The build system will automatically:
   - Find the newest `.bin` file in this directory
   - Create a LittleFS partition image containing the firmware
   - Flash it to the `storage` partition

## Important Notes

- Only `.bin` files are recognized
- If multiple files exist, the newest one (by timestamp) is used
- The firmware must be a valid ESP32 binary with proper image header
- This directory is only used during build time, not runtime

## Example

```bash
# Copy your slave firmware here
cp /path/to/your/slave/build/network_adapter.bin ./slave_fw_bin/

# Then build the project
eh.py -p <host_port> build flash monitor
```
