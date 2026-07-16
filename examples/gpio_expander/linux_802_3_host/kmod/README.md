# gpio_expander · linux_802_3_host/kmod

Thin wrapper scripts for the canonical Linux kernel-module transport flow.

## Use

```sh
./build.sh --bus spi
./load.sh --bus spi
./status.sh
./unload.sh
./clean.sh
```

These wrappers forward to:

- `host/linux/eh_host_linux_kmod/scripts/build.sh`
- `host/linux/eh_host_linux_kmod/scripts/load.sh`
- `host/linux/eh_host_linux_kmod/scripts/unload.sh`

Use `--bus spi` or `--bus sdio` depending on your wiring.

Linux host support in this repo is:

- `SDIO`
- `SPI full-duplex`

Linux host does not use:

- `SPI half-duplex`
- `UART`

After the kmod side is up, return to `../c_app/` or `../py_app/`.
