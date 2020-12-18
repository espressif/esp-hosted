### Directory structure for MCU based host

```
├── stm32
│   ├── app                             (Contains Initialization sequence of firmware, control
│   │   │                                    and data path modules)
│   │   ├── control                     (Contains files for control path)
│   │   └── data                        (Contains files for ARP server testing)
│   ├── common                          (Contains basic utilities such as hex_dump, delay, convert
│   │                                        mac address into bytes etc.)
│   ├── driver                          (Contains modules of networking and transport)
│   │   ├── netif                       (Header file for network interface)
│   │   ├── network                     (Contains file of Network Interface)
│   │   ├── serial                      (Contains Virtual Serial Interface files)
│   │   └── spi                         (Contains file for SPI transport)
│   └── proj                            (Contains `bat and sh` scripts for project setup,also
│                                           `.ioc` file for project installation in STM32CubeIDE)
```
