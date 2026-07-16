# ESP-Hosted Transport Configuration Example

Demonstrates how to configure ESP-Hosted transport interfaces (SDIO, SPI, UART) before initialization.

## Supported Transports

| Transport | Speed | Pins | Use Case |
|-----------|-------|------|----------|
| **SDIO 1-bit** | High | 5 pins | Good performance, fewer pins |
| **SDIO 4-bit** | Highest | 7 pins | Best performance |
| **SPI Full-Duplex** | Medium | 6 pins | Standard interface |
| **SPI HD Dual** | Medium | 4 pins | Fewer pins needed |
| **SPI HD Quad** | Medium | 6 pins | Better throughput |
| **UART** | Low | 3 pins | Simple connection |

## Pin Connections

### SDIO 1-bit

| Host <-> Slave (5 pins) | CLK | CMD | DAT0 | DAT1 (interrupt) | RESET |
|---|---|---|---|---|---|

### SDIO 4-bit (Default)

| Host <-> Slave (7 pins) | CLK | CMD | DAT0 | DAT1 | DAT2 | DAT3 | RESET |
|---|---|---|---|---|---|---|---|

### SPI Full-Duplex

| Host <-> Slave (6 pins) | MOSI | MISO | SCLK | CS | HANDSHAKE | DATA_READY |
|---|---|---|---|---|---|---|

### SPI Half-Duplex Dual

| Host <-> Slave (4 pins) | CLK | CS | D0 | D1 (+ DATA_READY) |
|---|---|---|---|---|

### SPI Half-Duplex Quad

| Host <-> Slave (6 pins) | CLK | CS | D0 | D1 | D2 | D3 (+ DATA_READY) |
|---|---|---|---|---|---|---|

### UART

| Host <-> Slave (3 pins) | TX | RX | RESET |
|---|---|---|---|

## What This Example Shows

- Get default transport configuration
- Customize parameters (clock, pins, buffers)
- Apply configuration before `esp_hosted_init()`

## Key Code Pattern

```c
// 1. Get default config
struct esp_hosted_sdio_config config = INIT_DEFAULT_HOST_SDIO_CONFIG();

// 2. Customize settings
config.clock_freq_khz = 25000;
config.bus_width = 4;

// 3. Apply config
esp_hosted_sdio_set_config(&config);

// 4. Initialize ESP-Hosted
esp_hosted_init();
```

## Customization

> [!TIP]
>
> This example was added to showcase (Option 2: Programmatic Configuration) discussed below.
> (Option 1: Using menuconfig) is anyway available for any host example.

### Pin Configuration Options

**Option 1: Using menuconfig**

First select your transport:

```bash
eh.py menuconfig
# (Top) → Component config → ESP-Hosted config → Transport layer
```

Then configure pins:

- **SDIO**: `(Top) → Component config → ESP-Hosted config → Hosted SDIO Configuration`
- **SPI Full-Duplex**: `(Top) → Component config → ESP-Hosted config → SPI Configuration`
- **SPI Half-Duplex**: `(Top) → Component config → ESP-Hosted config → SPI Half-duplex Configuration`
- **UART**: `(Top) → Component config → ESP-Hosted config → UART Configuration`

**Option 2: Programmatic Configuration**
Edit the configuration functions in `main.c` for users who prefer code-based configuration or have limitations with menuconfig:

```c
// Example: Custom SPI pin assignment
spi_config.pin_mosi.pin = YOUR_MOSI_PIN;
spi_config.pin_miso.pin = YOUR_MISO_PIN;
spi_config.pin_sclk.pin = YOUR_SCLK_PIN;
spi_config.pin_cs.pin = YOUR_CS_PIN;
```

### Other Customizations

- **Clock speeds**: Adjust for stability vs performance
- **Buffer sizes**: Tune for your throughput needs
- **Data lines**: Configure GPIOs to match your connections
- **Transport config**: Any other transport specific config
