### Directory structure for Linux based host

```
├── esp_hosted_fg
│   │
│   ├── host                            (host directory is common for MCU and MPU)
│   │   ├── LICENSES
│   │   │   ├── Apache-2.0
│   │   │   └── GPL-2.0
│   │   ├── components                  (common utilities like queue etc)
│   │   ├── control_lib                 (Hosted control path library)
│   │   ├── linux
│   │   │   ├── host_control            (Contain necessary files for installation of control path)
│   │   │   │   ├── c_support           (Contain files to test basic control path commands and stress.c for stress testing)
│   │   │   │   ├── python_support      (Contain python scripts for Wi-Fi functionality,
│   │   │   │   │                           `test.py` to test basic control path commands, stress.py for stress testing, also
│   │   │   │   │                           `commands_lib.py` control path commands implementation using
│   │   │   │   │                           `ctypes` module converts command requests from python to c and maps command responses from c to python)
│   │   │   │   ├── rpi_init.sh         (Installation sequence for ESP-Hosted-FG driver)
│   │   │   │   └── spidev_disabler.dts (dts file for SPI transport)
│   │   │   ├── host_driver             (Contain ESP-Hosted-FG kernel module files)
│   │   │   │   └── esp32               (ESP-Hosted kernel module files)
│   │   │   │       ├── sdio            (Contain SDIO transport files used by kernel module to communicate
│   │   │   │       │                       with ESP peripheral)
│   │   │   │       └── spi             (Contain SPI transport files used by kernel module to communicate
│   │   │   │                               with ESP peripheral)
│   │   │   └── port                    (Porting files for OS abstraction, contains wrappers for thread creation, semaphore operations, memory functions,
│   │   │                                   serial driver operations etc)
│   │   └── virtual_serial_if           (Virtual serial interface, platform agnostic layer which provides abstraction to setup control path
│   │                                       over serial driver)
```
