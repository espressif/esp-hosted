### Directory structure for ESP-Hosted-FG

```
├── esp_hosted_fg
│   │
│   ├── esp                             (ESP coprocessor/slave side)
│   │   ├── esp_driver                  (ESP-IDF project for coprocessor firmware)
│   │   │   ├── main                    (Main application code)
│   │   │   ├── components              (ESP-IDF components)
│   │   │   │   ├── esp_hosted_rpc_lib  (RPC library for ESP side)
│   │   │   │   └── esp_hosted_transport (Transport layer implementation)
│   │   │   ├── network_adapter         (Network adapter implementation)
│   │   │   └── sdkconfig.defaults      (Default ESP-IDF configuration)
│   │   └── esp_driver_ng               (Next generation ESP driver - experimental)
│   │
│   ├── host                            (Host side implementation)
│   │   ├── components                  (Host-side components)
│   │   │   └── esp_hosted_rpc_lib      (RPC library for host side)
│   │   │       ├── include             (Header files)
│   │   │       │   └── ctrl_api.h      (Control API definitions)
│   │   │       └── src                 (Source files)
│   │   │           ├── ctrl_api.c      (Control API implementation)
│   │   │           └── ctrl_core.c     (Core control functionality)
│   │   ├── linux                       (Linux host implementation)
│   │   │   ├── kmod                    (Kernel module)
│   │   │   │   ├── sdio                (SDIO transport)
│   │   │   │   └── spi                 (SPI transport)
│   │   │   ├── scripts                 (Build and setup scripts)
│   │   │   │   ├── esp_kmod_up.sh  (Kernel module build script)
│   │   │   │   ├── c_app_build.sh      (C application build script)
│   │   │   │   ├── py_app_build.sh     (Python application build script)
│   │   │   │   └── spidev_disabler.dts (Device tree overlay for SPI)
│   │   │   └── user_space              (User space applications)
│   │   │       ├── c_demo_app          (C demo applications)
│   │   │       │   ├── test.out        (Basic test application)
│   │   │       │   ├── stress.out      (Stress test application)
│   │   │       │   ├── hosted_daemon.out (Network daemon)
│   │   │       │   └── hosted_shell.out (Interactive shell)
│   │   │       └── python_demo_app     (Python demo applications)
│   │   │           ├── test.py         (Basic test script)
│   │   │           ├── stress.py       (Stress test script)
│   │   │           └── setup_python.sh (Python setup script)
│   │   └── port                        (OS abstraction layer)
│   │
│   ├── docs                            (Documentation)
│   │   ├── common                      (Common documentation)
│   │   ├── Linux_based_host            (Linux host documentation)
│   │   └── icons                       (Documentation assets)
│   │
│   └── common                          (Shared definitions and utilities)
       ├── include                     (Common header files)
       └── protobuf                    (Protocol buffer definitions)
```

## Key Components

### ESP Side (Coprocessor)
- **`esp/esp_driver/`**: Main ESP-IDF project for coprocessor firmware
- **`esp/esp_driver/components/esp_hosted_rpc_lib/`**: RPC library for ESP side communication
- **`esp/esp_driver/network_adapter/`**: Network adapter implementation for WiFi/Bluetooth

### Host Side
- **`host/components/esp_hosted_rpc_lib/`**: Host-side RPC library with control APIs
- **`host/linux/kmod/`**: Linux kernel module for transport layer
- **`host/linux/scripts/`**: Build and setup scripts for easy deployment
- **`host/linux/user_space/`**: Demo applications for testing and development

### Build Scripts
- **`esp_kmod_up.sh`**: Builds and loads kernel module with transport configuration
- **`c_app_build.sh`**: Builds C demo applications
- **`py_app_build.sh`**: Builds Python demo applications and RPC library

## Getting Started

### For ESP Firmware Development
1. **Start here**: [`esp/esp_driver/`](../../esp/esp_driver/README.md)
2. **Build firmware**: Use ESP-IDF build system in `esp/esp_driver/`

### For Host Development
1. **Linux hosts**: Use `host/linux/scripts/esp_kmod_up.sh`
2. **Control path**: Test with `host/linux/user_space/python_demo_app/test.py`
3. **Integration**: Use APIs from `host/components/esp_hosted_rpc_lib/include/ctrl_api.h`
