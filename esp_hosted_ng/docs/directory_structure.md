### Directory structure for Linux based host

```
├── esp_hosted_ng
│   │
│   ├── host                            (host directory for Linux based host)
│   │   ├── rpi_init.sh                 (Installation sequence for ESP-Hosted-NG driver)
│   │   │   └── spidev_disabler.dts     (dts file for SPI transport)
│   │   ├── sdio                        (Contain SDIO transport files used by kernel module to communicate
│   │   │                                   with ESP peripheral)
│   │   └── spi                         (Contain SPI transport files used by kernel module to communicate
│   │                                       with ESP peripheral)
```
