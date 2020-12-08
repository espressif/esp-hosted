### Directory structure for Linux based host

```
├── linux
│   ├── host_control            (Contains necessary files for installation of control path)
│   │   ├── c_support           (Contains files test basic control path commands)
│   │   ├── python_support      (Contains python scripts for Wi-Fi functionality, also
│   │   │                           `test.py` to test basic control path commands )
│   │   │   └── host_commands   (Contains control path commands implementation using
│   │   │                           python protobuf generated files)
│   │   │      └── transport    (Handles read/write operation of control path commands on
│   │   │                           ESP-Hosted character driver)
│   │   ├── rpi_init.sh         (Installation sequence for ESP-Hosted driver)
│   │   └── spidev_disabler.dts (dts file for SPI transport)
│   └── host_driver             (Contains ESP-Hosted kernel module files)
│       └── esp32               (ESP-hosted kernel module files )
│           ├── sdio            (Contains SDIO transport files used by kernel module to communicate
│           │                       with slave)
│           └── spi             (Contains SPI transport files used by kernel module to communicate
│                                   with slave)
```
