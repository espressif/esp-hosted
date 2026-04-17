### Directory structure for MCU based host

```
├── esp_hosted_fg
│   │ 
│   ├── host                          (host directory is common for MCU and MPU)
│   │   ├── LICENSES
│   │   │   ├── Apache-2.0
│   │   │   └── GPL-2.0
│   │   ├── components                (common utilities like queue etc)
│   │   ├── control_lib               (Hosted control path library)
│   │   ├── stm32
│   │   │   ├── app                   (Contain Initialization sequence of STM32 firmware, control
│   │   │   │   │                         and data path modules)
│   │   │   │   ├── control           (Contain files for control path, used for Wi-Fi connectivity)
│   │   │   │   └── data              (Contain files for demo APP, ARP server. The application
│   │   │   │                             would need to be hooked here.)
│   │   │   ├── common                (Contain common constructs used across app, driver)
│   │   │   ├── driver                (Contain modules of networking, serial interface and SPI transport)
│   │   │   │   ├── netif             (API for network device operation (netdev))
│   │   │   │   ├── network           (Contain netdev API implementation and network stub. Network stub
│   │   │   │   │                         should be replace by actual network stub like lwip by user)
│   │   │   │   ├── serial            (Contain Virtual Serial Interface implemented over SPI transport)
│   │   │   │   └── transport
│   │   │   │       ├── SDIO          (Contain driver for SDIO transport)
│   │   │   │       └── SPI           (Contain driver for SPI transport)
│   │   │   ├── proj                  (Contain windows 'batch'(.bat) file and sh scripts for project setup. Also contain
│   │   │   │                             `.ioc` file which is HAL layer for STM32, used in project installation in STM32CubeIDE)
│   │   │   └── port                  (Porting files for OS abstraction, contains wrappers
│   │   │                                 for thread creation, semaphore operations, memory functions,
│   │   │                                 serial driver operations etc)
│   │   └── virtual_serial_if         (Virtual serial interface, platform agnostic layer,
│   │                                      which provides abstraction to setup control path
│   │                                      over serial driver)
```
