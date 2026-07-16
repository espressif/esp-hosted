# Slave Firmware Binaries for Partition OTA

This directory should contain ESP32 slave firmware `.bin` files for partition-based OTA updates.

## Usage

1. Copy your slave firmware binary (e.g., `network_adapter.bin`) to this directory
2. Ensure your `partitions.csv` has a partition labeled `slave_fw` (or as configured in Kconfig)
3. Build the project with `CONFIG_OTA_METHOD_PARTITION=y`
4. The build system will automatically:
   - Find the newest `.bin` file in this directory
   - Flash it directly to the dedicated partition during `eh.py flash`

## Important Notes

- Only `.bin` files are recognized
- If multiple files exist, the newest one (by timestamp) is used
- The firmware must be a valid ESP32 binary with proper image header
- This directory is only used during build time for flashing, not runtime
- The partition must be defined in your partition table

## Example

```bash
# Copy your slave firmware here
cp /path/to/your/slave/build/network_adapter.bin ./slave_fw_bin/

# Ensure partitions.csv has something like:
# slave_fw, data, 0x40,  0x5F0000, 0x200000,

# Then build and flash the project
eh.py fullclean
eh.py -p <host_port> build flash monitor
```
