/**
 * @file esp_hosted_transport_config_example.c
 * @brief ESP-Hosted Transport Configuration Example
 *
 * This example demonstrates comprehensive transport configuration for ESP-Hosted,
 * including SDIO, SPI Full-Duplex, SPI Half-Duplex, and UART interfaces.
 *
 * Features:
 * - Default configuration initialization
 * - Custom parameter configuration
 * - Configuration validation
 * - Error handling and recovery
 * - Runtime configuration inspection
 *
 * @version 1.0.0
 * @date 2024-2025
 *
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

 #include <string.h>
 #include <stdint.h>
 #include <stdbool.h>

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "esp_event.h"
 #include "esp_err.h"
 #include "nvs_flash.h"
 #include "esp_console.h"
// #include "cmd_system.h"

 #include "esp_hosted.h"
 #include "esp_hosted_transport_config.h"
#include "esp_check.h"

 /*==============================================================================
  * CONSTANTS AND DEFINITIONS
  *============================================================================*/

 static const char *TAG = "esp_hosted_transport_config";

 /** @brief Default task delay for main loop (ms) */
 #define MAIN_LOOP_DELAY_MS              10000

 /*==============================================================================
  * TYPE DEFINITIONS
  *============================================================================*/

 /**
  * @brief Transport configuration status enumeration
  */
 typedef enum {
     TRANSPORT_CONFIG_STATUS_UNINITIALIZED = 0,
     TRANSPORT_CONFIG_STATUS_CONFIGURED,
     TRANSPORT_CONFIG_STATUS_ACTIVE,
     TRANSPORT_CONFIG_STATUS_ERROR
 } transport_config_status_t;

 /**
  * @brief Transport configuration context structure
  */
 typedef struct {
     transport_config_status_t status;
     esp_hosted_transport_err_t last_error;
 } transport_config_context_t;

 /*==============================================================================
  * GLOBAL VARIABLES
  *============================================================================*/

 static transport_config_context_t g_transport_context = {
     .status = TRANSPORT_CONFIG_STATUS_UNINITIALIZED,
     .last_error = ESP_TRANSPORT_OK,
 };

 /*==============================================================================
  * FORWARD DECLARATIONS
  *============================================================================*/

 static esp_err_t initialize_system_components(void);
 static esp_err_t configure_transport_interface(void);
 static esp_err_t validate_transport_configuration(void);
 static void display_transport_configuration(void);

 /*==============================================================================
  * SDIO TRANSPORT CONFIGURATION
  *============================================================================*/

 #if EH_HOST_TRANSPORT_BUS_SDIO
 /**
  * @brief Configure SDIO transport with optimized settings
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t configure_sdio_transport(void)
 {
     ESP_LOGI(TAG, "Configuring SDIO transport interface");

     // Initialize default SDIO configuration
     struct esp_hosted_sdio_config sdio_config = INIT_DEFAULT_HOST_SDIO_CONFIG();

     // Apply optimized settings
     sdio_config.clock_freq_khz = 25000;     // 25MHz for stable operation
     sdio_config.bus_width = 4;              // 4-bit bus for better throughput
     sdio_config.tx_queue_size = 20;         // Increased queue sizes
     sdio_config.rx_queue_size = 20;

     // Optional: Enable IOMUX for better performance (hardware dependent)
     // sdio_config.iomux_enable = true;

     // Apply configuration.  If the transport was already brought up by
     // the auto-init constructor (which fires before app_main on MCU
     // builds), set_config returns ESP_TRANSPORT_ERR_ALREADY_SET — that
     // is the *expected* path for a default build; treat it as
     // informational and fall through to display the active config below.
     esp_hosted_transport_err_t result = esp_hosted_sdio_set_config(&sdio_config);
     if (result == ESP_TRANSPORT_ERR_ALREADY_SET) {
         ESP_LOGW(TAG, "SDIO transport already initialised by constructor — "
                       "preferred config not applied (override happens at "
                       "build time via Kconfig).  Showing active config.");
         g_transport_context.last_error = result;
         return ESP_OK;  /* Continue: caller will GET + display live config. */
     }
     if (result != ESP_TRANSPORT_OK) {
         ESP_LOGE(TAG, "SDIO configuration failed: error code %d", result);
         g_transport_context.last_error = result;
         return ESP_FAIL;
     }

     ESP_LOGI(TAG, "SDIO preferred config applied (pre-init):");
     ESP_LOGI(TAG, "  Clock frequency: %lu kHz", sdio_config.clock_freq_khz);
     ESP_LOGI(TAG, "  Bus width: %d bits", sdio_config.bus_width);
     ESP_LOGI(TAG, "  Queue sizes - TX: %d, RX: %d",
              sdio_config.tx_queue_size, sdio_config.rx_queue_size);

     return ESP_OK;
 }
 #endif /* EH_HOST_TRANSPORT_BUS_SDIO */

 /*==============================================================================
  * SPI FULL-DUPLEX TRANSPORT CONFIGURATION
  *============================================================================*/

 #if EH_HOST_TRANSPORT_BUS_SPI
 /**
  * @brief Configure SPI Full-Duplex transport with custom pin mapping
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t configure_spi_transport(void)
 {
     ESP_LOGI(TAG, "Configuring SPI Full-Duplex transport interface");

     // Initialize default SPI configuration
     struct esp_hosted_spi_config spi_config = INIT_DEFAULT_HOST_SPI_CONFIG();

     // Configure GPIO pin assignments
     spi_config.pin_mosi.pin = 23;           // Master Out Slave In
     spi_config.pin_miso.pin = 19;           // Master In Slave Out
     spi_config.pin_sclk.pin = 18;           // Serial Clock
     spi_config.pin_cs.pin = 5;              // Chip Select
     spi_config.pin_handshake.pin = 2;       // Handshake signal
     spi_config.pin_data_ready.pin = 4;      // Data Ready indication

     // Configure SPI parameters
     spi_config.clk_mhz = 10;                // 10MHz clock frequency
     spi_config.mode = 0;                    // SPI mode 0 (CPOL=0, CPHA=0)
     spi_config.tx_queue_size = 15;          // Transmission queue size
     spi_config.rx_queue_size = 15;          // Reception queue size

     // Apply configuration; ALREADY_SET = auto-init beat us, fine.
     esp_hosted_transport_err_t result = esp_hosted_spi_set_config(&spi_config);
     if (result == ESP_TRANSPORT_ERR_ALREADY_SET) {
         ESP_LOGW(TAG, "SPI transport already initialised by constructor — "
                       "showing active config below.");
         g_transport_context.last_error = result;
         return ESP_OK;
     }
     if (result != ESP_TRANSPORT_OK) {
         ESP_LOGE(TAG, "SPI configuration failed: error code %d", result);
         g_transport_context.last_error = result;
         return ESP_FAIL;
     }

     ESP_LOGI(TAG, "SPI Full-Duplex preferred config applied (pre-init):");
     ESP_LOGI(TAG, "  Clock frequency: %lu MHz", spi_config.clk_mhz);
     ESP_LOGI(TAG, "  Pin mapping - MOSI: %d, MISO: %d, SCLK: %d, CS: %d",
              spi_config.pin_mosi.pin, spi_config.pin_miso.pin,
              spi_config.pin_sclk.pin, spi_config.pin_cs.pin);
     ESP_LOGI(TAG, "  Control pins - Handshake: %d, Data Ready: %d",
              spi_config.pin_handshake.pin, spi_config.pin_data_ready.pin);

     return ESP_OK;
 }
 #endif /* EH_HOST_TRANSPORT_BUS_SPI */

 /*==============================================================================
  * SPI HALF-DUPLEX TRANSPORT CONFIGURATION
  *============================================================================*/

 #if EH_HOST_TRANSPORT_BUS_SPI_HD
 /**
  * @brief Configure SPI Half-Duplex transport with enhanced features
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t configure_spi_hd_transport(void)
 {
     ESP_LOGI(TAG, "Configuring SPI Half-Duplex transport interface");

     // Initialize default SPI HD configuration
     struct esp_hosted_spi_hd_config spi_hd_config = INIT_DEFAULT_HOST_SPI_HD_CONFIG();

     // Configure enhanced parameters
     spi_hd_config.num_data_lines = 4;       // Quad SPI mode
     spi_hd_config.clk_mhz = 20;             // 20MHz clock frequency

     // Configure GPIO pin assignments
     spi_hd_config.pin_cs.pin = 15;          // Chip Select
     spi_hd_config.pin_clk.pin = 14;         // Clock line
     spi_hd_config.pin_data_ready.pin = 4;   // Data Ready indication

     // Apply configuration; ALREADY_SET = auto-init beat us, fine.
     esp_hosted_transport_err_t result = esp_hosted_spi_hd_set_config(&spi_hd_config);
     if (result == ESP_TRANSPORT_ERR_ALREADY_SET) {
         ESP_LOGW(TAG, "SPI-HD transport already initialised by constructor — "
                       "showing active config below.");
         g_transport_context.last_error = result;
         return ESP_OK;
     }
     if (result != ESP_TRANSPORT_OK) {
         ESP_LOGE(TAG, "SPI Half-Duplex configuration failed: error code %d", result);
         g_transport_context.last_error = result;
         return ESP_FAIL;
     }

     ESP_LOGI(TAG, "SPI Half-Duplex preferred config applied (pre-init):");
     ESP_LOGI(TAG, "  Clock frequency: %lu MHz", spi_hd_config.clk_mhz);
     ESP_LOGI(TAG, "  Data lines: %d (Quad SPI)", spi_hd_config.num_data_lines);
     ESP_LOGI(TAG, "  Pin mapping - CS: %d, CLK: %d, Data Ready: %d",
              spi_hd_config.pin_cs.pin, spi_hd_config.pin_clk.pin,
              spi_hd_config.pin_data_ready.pin);

     return ESP_OK;
 }
 #endif /* EH_HOST_TRANSPORT_BUS_SPI_HD */

 /*==============================================================================
  * UART TRANSPORT CONFIGURATION
  *============================================================================*/

 #if EH_HOST_TRANSPORT_BUS_UART
 /**
  * @brief Configure UART transport with high-speed settings
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t configure_uart_transport(void)
 {
     ESP_LOGI(TAG, "Configuring UART transport interface");

     // Initialize default UART configuration
     struct esp_hosted_uart_config uart_config = INIT_DEFAULT_HOST_UART_CONFIG();

     // Configure high-speed UART parameters
     uart_config.baud_rate = 921600;         // High baud rate for performance
     uart_config.num_data_bits = 8;          // 8 data bits
     uart_config.parity = 0;                 // No parity bit
     uart_config.stop_bits = 1;              // 1 stop bit

     // Configure GPIO pin assignments
     uart_config.pin_tx.pin = 17;            // Transmit pin
     uart_config.pin_rx.pin = 16;            // Receive pin

     // Configure buffer sizes
     uart_config.tx_queue_size = 100;        // Transmission buffer
     uart_config.rx_queue_size = 100;        // Reception buffer

     // Apply configuration; ALREADY_SET = auto-init beat us, fine.
     esp_hosted_transport_err_t result = esp_hosted_uart_set_config(&uart_config);
     if (result == ESP_TRANSPORT_ERR_ALREADY_SET) {
         ESP_LOGW(TAG, "UART transport already initialised by constructor — "
                       "showing active config below.");
         g_transport_context.last_error = result;
         return ESP_OK;
     }
     if (result != ESP_TRANSPORT_OK) {
         ESP_LOGE(TAG, "UART configuration failed: error code %d", result);
         g_transport_context.last_error = result;
         return ESP_FAIL;
     }

     ESP_LOGI(TAG, "UART preferred config applied (pre-init):");
     ESP_LOGI(TAG, "  Baud rate: %lu bps", uart_config.baud_rate);
     ESP_LOGI(TAG, "  Parameters: %d%c%d", uart_config.num_data_bits,
              uart_config.parity ? 'P' : 'N', uart_config.stop_bits);
     ESP_LOGI(TAG, "  Pin mapping - TX: %d, RX: %d",
              uart_config.pin_tx.pin, uart_config.pin_rx.pin);
     ESP_LOGI(TAG, "  Buffer sizes - TX: %d, RX: %d",
              uart_config.tx_queue_size, uart_config.rx_queue_size);

     return ESP_OK;
 }
 #endif /* EH_HOST_TRANSPORT_BUS_UART */

 /*==============================================================================
  * CONFIGURATION VALIDATION AND INSPECTION
  *============================================================================*/

 /**
  * @brief Display current transport configuration
  */
 static void display_transport_configuration(void)
 {
     ESP_LOGI(TAG, "Retrieving current transport configuration...");

     struct esp_hosted_transport_config *config;
     esp_hosted_transport_err_t result = esp_hosted_transport_get_config(&config);

     if (result != ESP_TRANSPORT_OK || !config) {
         ESP_LOGE(TAG, "Failed to retrieve transport configuration: error %d", result);
         return;
     }

     ESP_LOGI(TAG, "Current transport configuration:");
     ESP_LOGI(TAG, "  Transport type: %d", config->transport_in_use);

     switch (config->transport_in_use) {
         case 1: // SDIO
             ESP_LOGI(TAG, "  SDIO Configuration:");
             ESP_LOGI(TAG, "    Clock frequency: %lu kHz", (unsigned long)config->u.sdio.clock_freq_khz);
             ESP_LOGI(TAG, "    Bus width:       %d bits", config->u.sdio.bus_width);
             ESP_LOGI(TAG, "    Slot:            %d", config->u.sdio.slot);
             ESP_LOGI(TAG, "    GPIOs - CLK[%d] CMD[%d] D0[%d] D1[%d] D2[%d] D3[%d] RESET[%d]",
                      config->u.sdio.pin_clk.pin, config->u.sdio.pin_cmd.pin,
                      config->u.sdio.pin_d0.pin, config->u.sdio.pin_d1.pin,
                      config->u.sdio.pin_d2.pin, config->u.sdio.pin_d3.pin,
                      config->u.sdio.pin_reset.pin);
             ESP_LOGI(TAG, "    RX mode:         %d", config->u.sdio.rx_mode);
             ESP_LOGI(TAG, "    Block mode:      %s", config->u.sdio.block_mode ? "yes" : "no");
             ESP_LOGI(TAG, "    IOMUX enable:    %s", config->u.sdio.iomux_enable ? "yes" : "no");
             ESP_LOGI(TAG, "    Queue sizes - TX[%d] RX[%d]",
                      config->u.sdio.tx_queue_size, config->u.sdio.rx_queue_size);
             break;

         case 2: // SPI Half-Duplex
             ESP_LOGI(TAG, "  SPI Half-Duplex Configuration:");
             ESP_LOGI(TAG, "    Clock frequency: %lu MHz", config->u.spi_hd.clk_mhz);
             ESP_LOGI(TAG, "    Data lines: %d", config->u.spi_hd.num_data_lines);
             break;

         case 3: // SPI Full-Duplex
             ESP_LOGI(TAG, "  SPI Full-Duplex Configuration:");
             ESP_LOGI(TAG, "    Clock frequency: %lu MHz", config->u.spi.clk_mhz);
             ESP_LOGI(TAG, "    SPI mode: %d", config->u.spi.mode);
             ESP_LOGI(TAG, "    Queue sizes - TX: %d, RX: %d",
                      config->u.spi.tx_queue_size, config->u.spi.rx_queue_size);
             break;

         case 4: // UART
             ESP_LOGI(TAG, "  UART Configuration:");
             ESP_LOGI(TAG, "    Baud rate: %lu bps", config->u.uart.baud_rate);
             ESP_LOGI(TAG, "    UART port: %d", config->u.uart.port);
             break;

         default:
             ESP_LOGW(TAG, "  Unknown transport type: %d", config->transport_in_use);
             break;
     }
 }

 /**
  * @brief Validate transport configuration
  *
  * @return ESP_OK if configuration is valid, ESP_FAIL otherwise
  */
 static esp_err_t validate_transport_configuration(void)
 {
     ESP_LOGI(TAG, "Validating transport configuration...");

     struct esp_hosted_transport_config *config;
     esp_hosted_transport_err_t result = esp_hosted_transport_get_config(&config);

     if (result != ESP_TRANSPORT_OK || !config) {
         ESP_LOGE(TAG, "Configuration validation failed: cannot retrieve config");
         return ESP_FAIL;
     }

     // Basic validation checks
     if (config->transport_in_use < 1 || config->transport_in_use > 4) {
         ESP_LOGE(TAG, "Invalid transport type: %d", config->transport_in_use);
         return ESP_FAIL;
     }

     ESP_LOGI(TAG, "Transport configuration validation passed");
     return ESP_OK;
 }

 /*==============================================================================
  * SYSTEM INITIALIZATION
  *============================================================================*/

 /**
  * @brief Initialize system components (NVS, event loop)
  *
  * @return ESP_OK on success, error code on failure
  */
 static esp_err_t initialize_system_components(void)
 {
     ESP_LOGI(TAG, "Initializing system components...");

     // Initialize NVS flash
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_LOGW(TAG, "NVS flash needs to be erased, reinitializing...");
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "NVS flash initialization failed: %s", esp_err_to_name(ret));
         return ret;
     }

     // Initialize default event loop
     ret = esp_event_loop_create_default();
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Event loop initialization failed: %s", esp_err_to_name(ret));
         return ret;
     }

     ESP_LOGI(TAG, "System components initialized successfully");
     return ESP_OK;
 }

 /**
  * @brief Configure transport interface based on menuconfig selection
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t configure_transport_interface(void)
 {
     esp_err_t result = ESP_FAIL;

     ESP_LOGI(TAG, "Configuring transport interface...");

 #if EH_HOST_TRANSPORT_BUS_SDIO
     ESP_LOGI(TAG, "SDIO interface selected in menuconfig");
     result = configure_sdio_transport();
 #elif EH_HOST_TRANSPORT_BUS_SPI
     ESP_LOGI(TAG, "SPI Full-Duplex interface selected in menuconfig");
     result = configure_spi_transport();
 #elif EH_HOST_TRANSPORT_BUS_SPI_HD
     ESP_LOGI(TAG, "SPI Half-Duplex interface selected in menuconfig");
     result = configure_spi_hd_transport();
 #elif EH_HOST_TRANSPORT_BUS_UART
     ESP_LOGI(TAG, "UART interface selected in menuconfig");
     result = configure_uart_transport();
 #else
     ESP_LOGW(TAG, "No transport interface selected in menuconfig");
     ESP_LOGI(TAG, "Please configure transport interface using 'idf.py menuconfig'");
     return ESP_FAIL;
 #endif

     if (result == ESP_OK) {
         g_transport_context.status = TRANSPORT_CONFIG_STATUS_CONFIGURED;
         ESP_LOGI(TAG, "Transport interface configured successfully");
     } else {
         g_transport_context.status = TRANSPORT_CONFIG_STATUS_ERROR;
         ESP_LOGE(TAG, "Transport interface configuration failed");
     }

     return result;
 }

 /**
  * @brief Initialize ESP-Hosted (simple version)
  *
  * @return ESP_OK on success, ESP_FAIL on error
  */
 static esp_err_t initialize_esp_hosted_simple(void)
 {
     ESP_LOGI(TAG, "Initializing ESP-Hosted...");

     esp_err_t result = esp_hosted_init();
     if (result != ESP_OK) {
         ESP_LOGE(TAG, "ESP-Hosted initialization failed: %s", esp_err_to_name(result));
         ESP_LOGE(TAG, "Check transport configuration and slave device connection");
         return result;
     }

     result = esp_hosted_connect_to_slave();
     if (result != ESP_OK) {
         ESP_LOGE(TAG, "esp_hosted_connect_to_slave failed: %s", esp_err_to_name(result));
         g_transport_context.status = TRANSPORT_CONFIG_STATUS_ERROR;
         return result;
     }

     ESP_LOGI(TAG, "ESP-Hosted initialized successfully");
     return ESP_OK;
 }

 /*==============================================================================
  * MAIN APPLICATION
  *============================================================================*/

 /**
  * @brief Main application entry point
  */
 void app_main(void)
 {
     ESP_LOGI(TAG, "ESP-Hosted Transport Configuration Example v1.0.0");
     ESP_LOGI(TAG, "========================================");

     // Initialize system components
     if (initialize_system_components() != ESP_OK) {
         ESP_LOGE(TAG, "System initialization failed, stopping application");
         return;
     }

     // Configure transport interface
     if (configure_transport_interface() != ESP_OK) {
         ESP_LOGE(TAG, "Transport configuration failed, stopping application");
         return;
     }

     // Validate configuration
     if (validate_transport_configuration() != ESP_OK) {
         ESP_LOGE(TAG, "Transport configuration validation failed");
         return;
     }

     // Display current configuration
     display_transport_configuration();

     ESP_LOGI(TAG, "========================================");
     ESP_LOGI(TAG, "Initializing ESP-Hosted...");

     // Initialize ESP-Hosted with configured transport
     if (initialize_esp_hosted_simple() == ESP_OK) {
         ESP_LOGI(TAG, "ESP-Hosted initialization successful!");
         ESP_LOGI(TAG, "WiFi and Bluetooth functionality is now available");

         // Display final configuration
         display_transport_configuration();

         ESP_LOGI(TAG, "========================================");
     } else {
         ESP_LOGE(TAG, "ESP-Hosted initialization failed");
     }

	 ESP_LOGI(TAG, "Application running - Transport status: %s",
			 g_transport_context.status == TRANSPORT_CONFIG_STATUS_ACTIVE ?
			 "Active" : "Error");
 }
