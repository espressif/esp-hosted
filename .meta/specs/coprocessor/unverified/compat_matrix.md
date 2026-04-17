---
type: spec
status: unverified
last_updated: 2026-03-31
---

# Legacy Compatibility Matrix

## Curated Rename Map (Legacy → New)

| Legacy Kconfig | Closest New Kconfig | Notes |
|---|---|---|
| `ESP_HOSTED_NETWORK_SPLIT_ENABLED` | `ESP_HOSTED_CP_FEAT_NW_SPLIT` + `ESP_HOSTED_CP_FEAT_NW_SPLIT_READY` | New split extension gate + readiness. |
| `ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER` | `ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER` + `ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY` | New peer-data extension gate + readiness. |
| `ESP_HOSTED_HOST_POWER_SAVE_ENABLED` | `ESP_HOSTED_CP_FEAT_HOST_PS` + `ESP_HOSTED_CP_FEAT_HOST_PS_READY` | Host power-save extension gate + readiness. |
| `ESP_HOSTED_HOST_DEEP_SLEEP_ALLOWED` | `ESP_HOSTED_CP_FEAT_HOST_PS_DEEP_SLEEP` | Host deep sleep option in extension. |
| `ESP_HOSTED_HOST_WAKEUP_GPIO` | `ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO` | Host wakeup GPIO location. |
| `ESP_HOSTED_HOST_WAKEUP_GPIO_LEVEL` | `ESP_HOSTED_CP_FEAT_HOST_PS_HOST_WAKEUP_GPIO_LEVEL` | Host wakeup GPIO level. |
| `ESP_HOSTED_ENABLE_BT_BLUEDROID` | `CONFIG_BT_ENABLED` + `CONFIG_ESP_HOSTED_CP_BT_ENABLED` | New split: IDF BT enable + hosted CP BT enable. |
| `ESP_HOSTED_ENABLE_BT_NIMBLE` | `CONFIG_BT_ENABLED` + `CONFIG_ESP_HOSTED_CP_BT_ENABLED` | New split: IDF BT enable + hosted CP BT enable. |
| `ESP_HOSTED_ENABLE_DPP` | `H_DPP_SUPPORT` (from IDF WiFi DPP support) | Derived from IDF capability macros. |
| `ESP_HOSTED_ENABLE_ITWT` | `CONFIG_SOC_WIFI_HE_SUPPORT` | ITWT depends on 11ax support. |
| `ESP_HOSTED_MEM_MONITOR` | — | Missing; no new equivalent yet. |
| `ESP_HOSTED_ENABLE_GPIO_EXPANDER` | — | Missing; no new equivalent yet. |
| `ESP_HOSTED_CP_EXT_COEX` | — | Missing; no new equivalent yet. |

## Curated RPC Placement Map

| Legacy RPC | New Placement | Notes |
|---|---|---|
| `RPC_ID__Req_SetDhcpDnsStatus` | `eh_cp_feat_rpc_ext_mcu_handler_req_nw_split.c` | Requires `ESP_HOSTED_CP_FEAT_NW_SPLIT_READY`. |
| `RPC_ID__Req_GetDhcpDnsStatus` | `eh_cp_feat_rpc_ext_mcu_handler_req_nw_split.c` | Requires `ESP_HOSTED_CP_FEAT_NW_SPLIT_READY`. |
| `RPC_ID__Req_CustomRpc` | `eh_cp_feat_rpc_ext_mcu_handler_req_peer_data.c` | Requires `ESP_HOSTED_CP_FEAT_PEER_DATA_TRANSFER_READY`. |
| `RPC_ID__Req_OTA*` | `eh_cp_feat_rpc_ext_mcu_handler_req_system.c` | Requires `ESP_HOSTED_CP_FEAT_SYSTEM_READY`. |
| `RPC_ID__Req_FeatureControl` | `eh_cp_feat_rpc_ext_mcu_handler_req_system.c` | Requires `ESP_HOSTED_CP_FEAT_SYSTEM_READY`. |

## Kconfig Exact-Match Matrix

| Legacy Kconfig | Present In esp_hosted.new | Notes |
|---|---|---|
| `ESP_HOSTED_BLUEDROID_HCI_VHCI` | No | |
| `ESP_HOSTED_CLI_ENABLED` | No | |
| `ESP_HOSTED_CLI_NEW_INSTANCE` | No | |
| `ESP_HOSTED_CP_EXT_COEX` | No | |
| `ESP_HOSTED_CP_EXT_COEX_ADVANCE` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32C2` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32C3` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32C5` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32C6` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32C61` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32H2` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32H4` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32S2` | No | |
| `ESP_HOSTED_CP_TARGET_ESP32S3` | No | |
| `ESP_HOSTED_DECODE_WIFI_RESERVED_FIELD` | No | |
| `ESP_HOSTED_DFLT_TASK_FROM_SPIRAM` | No | |
| `ESP_HOSTED_DFLT_TASK_STACK` | No | |
| `ESP_HOSTED_DPP_URI_LEN_MAX` | No | |
| `ESP_HOSTED_DR_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_DR_ACTIVE_LOW` | No | |
| `ESP_HOSTED_ENABLED` | No | |
| `ESP_HOSTED_ENABLE_BT_BLUEDROID` | No | |
| `ESP_HOSTED_ENABLE_BT_NIMBLE` | No | |
| `ESP_HOSTED_ENABLE_DPP` | No | |
| `ESP_HOSTED_ENABLE_GPIO_EXPANDER` | No | |
| `ESP_HOSTED_ENABLE_ITWT` | No | |
| `ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER` | No | |
| `ESP_HOSTED_FW_VERSION_MISMATCH_WARNING_SUPPRESS` | No | |
| `ESP_HOSTED_GPIO_SLAVE_RESET_SLAVE` | No | |
| `ESP_HOSTED_HOST_DEEP_SLEEP_ALLOWED` | No | |
| `ESP_HOSTED_HOST_POWER_SAVE_ENABLED` | No | |
| `ESP_HOSTED_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE` | No | |
| `ESP_HOSTED_HOST_RESTART_NO_COMMUNICATION_WITH_SLAVE_TIMEOUT` | No | |
| `ESP_HOSTED_HOST_TO_ESP_WIFI_DATA_THROTTLE` | No | |
| `ESP_HOSTED_HOST_WAKEUP_GPIO` | No | |
| `ESP_HOSTED_HOST_WAKEUP_GPIO_LEVEL` | No | |
| `ESP_HOSTED_HOST_WAKEUP_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_HOST_WAKEUP_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_HS_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_HS_ACTIVE_LOW` | No | |
| `ESP_HOSTED_IDF_SLAVE_TARGET` | No | |
| `ESP_HOSTED_MAX_CUSTOM_MSG_HANDLERS` | No | |
| `ESP_HOSTED_MAX_SIMULTANEOUS_ASYNC_RPC_REQUESTS` | No | |
| `ESP_HOSTED_MAX_SIMULTANEOUS_SYNC_RPC_REQUESTS` | No | |
| `ESP_HOSTED_MEM_MONITOR` | No | |
| `ESP_HOSTED_NETWORK_SPLIT_ENABLED` | No | |
| `ESP_HOSTED_NIMBLE_HCI_VHCI` | No | |
| `ESP_HOSTED_P4_C5_CORE_BOARD` | No | |
| `ESP_HOSTED_P4_C61_CORE_BOARD` | No | |
| `ESP_HOSTED_P4_C6_CORE_BOARD` | No | |
| `ESP_HOSTED_P4_DEV_BOARD_FUNC_BOARD` | No | |
| `ESP_HOSTED_P4_DEV_BOARD_NONE` | No | |
| `ESP_HOSTED_PKT_STATS` | No | |
| `ESP_HOSTED_PKT_STATS_INTERVAL_SEC` | No | |
| `ESP_HOSTED_PRIV_ENABLE_WIFI_OPTIONS` | No | |
| `ESP_HOSTED_PRIV_SDIO_OPTION` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_CMD_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_CMD_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D0_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D0_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D1_1BIT_BUS_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D1_1BIT_BUS_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D1_4BIT_BUS_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D1_4BIT_BUS_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D2_4BIT_BUS_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D2_4BIT_BUS_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D3_4BIT_BUS_SLOT_0` | No | |
| `ESP_HOSTED_PRIV_SDIO_PIN_D3_4BIT_BUS_SLOT_1` | No | |
| `ESP_HOSTED_PRIV_SPI_HD_OPTION` | No | |
| `ESP_HOSTED_PRIV_WIFI_TX_SDIO_HIGH_THRESHOLD` | No | |
| `ESP_HOSTED_PRIV_WIFI_TX_SPI_HD_HIGH_THRESHOLD` | No | |
| `ESP_HOSTED_PRIV_WIFI_TX_SPI_HIGH_THRESHOLD` | No | |
| `ESP_HOSTED_PRIV_WIFI_TX_UART_HIGH_THRESHOLD` | No | |
| `ESP_HOSTED_RAW_THROUGHPUT_BIDIRECTIONAL` | No | |
| `ESP_HOSTED_RAW_THROUGHPUT_RX_FROM_SLAVE` | No | |
| `ESP_HOSTED_RAW_THROUGHPUT_TRANSPORT` | No | |
| `ESP_HOSTED_RAW_THROUGHPUT_TX_TO_SLAVE` | No | |
| `ESP_HOSTED_RAW_TP_HOST_TO_ESP_PKT_LEN` | No | |
| `ESP_HOSTED_RAW_TP_REPORT_INTERVAL` | No | |
| `ESP_HOSTED_RESET_GPIO_ACTIVE_LOW` | No | |
| `ESP_HOSTED_RPC_TASK_STACK` | No | |
| `ESP_HOSTED_SDIO_1_BIT_BUS` | No | |
| `ESP_HOSTED_SDIO_4_BIT_BUS` | No | |
| `ESP_HOSTED_SDIO_BUS_WIDTH` | No | |
| `ESP_HOSTED_SDIO_CHECKSUM` | No | |
| `ESP_HOSTED_SDIO_CLK_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_CLK_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ` | No | |
| `ESP_HOSTED_SDIO_CMD_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_CMD_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_D0_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_D0_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_D1_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_D1_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_D2_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_D2_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_D3_GPIO_RANGE_MAX` | No | |
| `ESP_HOSTED_SDIO_D3_GPIO_RANGE_MIN` | No | |
| `ESP_HOSTED_SDIO_GPIO_RESET_SLAVE` | No | |
| `ESP_HOSTED_SDIO_HOST_INTERFACE` | No | |
| `ESP_HOSTED_SDIO_OPTIMIZATION_RX_MAX_SIZE` | No | |
| `ESP_HOSTED_SDIO_OPTIMIZATION_RX_NONE` | No | |
| `ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE` | No | |
| `ESP_HOSTED_SDIO_PIN_CLK` | No | |
| `ESP_HOSTED_SDIO_PIN_CMD` | No | |
| `ESP_HOSTED_SDIO_PIN_D0` | No | |
| `ESP_HOSTED_SDIO_PIN_D1` | No | |
| `ESP_HOSTED_SDIO_PIN_D2` | No | |
| `ESP_HOSTED_SDIO_PIN_D3` | No | |
| `ESP_HOSTED_SDIO_PRIV_PIN_D1_1BIT_BUS` | No | |
| `ESP_HOSTED_SDIO_PRIV_PIN_D1_4BIT_BUS` | No | |
| `ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_SDIO_RESET_ACTIVE_LOW` | No | |
| `ESP_HOSTED_SDIO_RESET_DELAY_MS` | No | |
| `ESP_HOSTED_SDIO_RESET_SLAVE_GPIO_MAX` | No | |
| `ESP_HOSTED_SDIO_RESET_SLAVE_GPIO_MIN` | No | |
| `ESP_HOSTED_SDIO_RX_Q_SIZE` | No | |
| `ESP_HOSTED_SDIO_SLOT` | No | |
| `ESP_HOSTED_SDIO_SLOT_0` | No | |
| `ESP_HOSTED_SDIO_SLOT_1` | No | |
| `ESP_HOSTED_SDIO_TX_Q_SIZE` | No | |
| `ESP_HOSTED_SD_PWR_CTRL_LDO_INTERNAL_IO` | No | |
| `ESP_HOSTED_SD_PWR_CTRL_LDO_IO_ID` | No | |
| `ESP_HOSTED_SLAVE_RESET_ONLY_IF_NECESSARY` | No | |
| `ESP_HOSTED_SLAVE_RESET_ON_EVERY_HOST_BOOTUP` | No | |
| `ESP_HOSTED_SPI_CLK_FREQ` | No | |
| `ESP_HOSTED_SPI_CLK_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_CLK_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_CONTROLLER` | No | |
| `ESP_HOSTED_SPI_CS_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_CS_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_DATA_READY_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_DATA_READY_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_FREQ_ESP32` | No | |
| `ESP_HOSTED_SPI_FREQ_ESP32C6` | No | |
| `ESP_HOSTED_SPI_FREQ_ESP32H2` | No | |
| `ESP_HOSTED_SPI_FREQ_ESP32XX` | No | |
| `ESP_HOSTED_SPI_GPIO_CLK` | No | |
| `ESP_HOSTED_SPI_GPIO_CS` | No | |
| `ESP_HOSTED_SPI_GPIO_DATA_READY` | No | |
| `ESP_HOSTED_SPI_GPIO_HANDSHAKE` | No | |
| `ESP_HOSTED_SPI_GPIO_MISO` | No | |
| `ESP_HOSTED_SPI_GPIO_MOSI` | No | |
| `ESP_HOSTED_SPI_GPIO_RESET_SLAVE` | No | |
| `ESP_HOSTED_SPI_HANDSHAKE_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HANDSHAKE_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_CHECKSUM` | No | |
| `ESP_HOSTED_SPI_HD_CLK_FREQ` | No | |
| `ESP_HOSTED_SPI_HD_CLK_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_CLK_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_CS_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_CS_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_D0_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_D0_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_D1_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_D1_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_D2_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_D2_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_D3_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_D3_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_DATA_READY_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_DATA_READY_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_DR_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_SPI_HD_DR_ACTIVE_LOW` | No | |
| `ESP_HOSTED_SPI_HD_FREQ_ESP32C6` | No | |
| `ESP_HOSTED_SPI_HD_FREQ_ESP32XX` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_CLK` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_CS` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_D0` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_D1` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_D2` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_D3` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_DATA_READY` | No | |
| `ESP_HOSTED_SPI_HD_GPIO_RESET_SLAVE` | No | |
| `ESP_HOSTED_SPI_HD_HOST_INTERFACE` | No | |
| `ESP_HOSTED_SPI_HD_INTERFACE_NUM_DATA_LINES` | No | |
| `ESP_HOSTED_SPI_HD_MODE` | No | |
| `ESP_HOSTED_SPI_HD_PRIV_INTERFACE_2_DATA_LINES` | No | |
| `ESP_HOSTED_SPI_HD_PRIV_INTERFACE_4_DATA_LINES` | No | |
| `ESP_HOSTED_SPI_HD_RESET_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_SPI_HD_RESET_ACTIVE_LOW` | No | |
| `ESP_HOSTED_SPI_HD_RESET_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_HD_RESET_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_HD_RX_Q_SIZE` | No | |
| `ESP_HOSTED_SPI_HD_TX_Q_SIZE` | No | |
| `ESP_HOSTED_SPI_HOST_INTERFACE` | No | |
| `ESP_HOSTED_SPI_HSPI` | No | |
| `ESP_HOSTED_SPI_HSPI_GPIO_CLK` | No | |
| `ESP_HOSTED_SPI_HSPI_GPIO_CS` | No | |
| `ESP_HOSTED_SPI_HSPI_GPIO_MISO` | No | |
| `ESP_HOSTED_SPI_HSPI_GPIO_MOSI` | No | |
| `ESP_HOSTED_SPI_MISO_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_MISO_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_MODE` | No | |
| `ESP_HOSTED_SPI_MOSI_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_MOSI_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_0_ESP32` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_0_ESP32XX` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_1_ESP32` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_1_ESP32XX` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_2_ESP32` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_2_ESP32XX` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_3_ESP32` | No | |
| `ESP_HOSTED_SPI_PRIV_MODE_3_ESP32XX` | No | |
| `ESP_HOSTED_SPI_RESET_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_SPI_RESET_ACTIVE_LOW` | No | |
| `ESP_HOSTED_SPI_RESET_RANGE_MAX` | No | |
| `ESP_HOSTED_SPI_RESET_RANGE_MIN` | No | |
| `ESP_HOSTED_SPI_RX_Q_SIZE` | No | |
| `ESP_HOSTED_SPI_TX_Q_SIZE` | No | |
| `ESP_HOSTED_SPI_VSPI` | No | |
| `ESP_HOSTED_SPI_VSPI_GPIO_CLK` | No | |
| `ESP_HOSTED_SPI_VSPI_GPIO_CS` | No | |
| `ESP_HOSTED_SPI_VSPI_GPIO_MISO` | No | |
| `ESP_HOSTED_SPI_VSPI_GPIO_MOSI` | No | |
| `ESP_HOSTED_TO_WIFI_DATA_THROTTLE_HIGH_THRESHOLD` | No | |
| `ESP_HOSTED_TO_WIFI_DATA_THROTTLE_LOW_THRESHOLD` | No | |
| `ESP_HOSTED_TRANSPORT_RESTART_ON_FAILURE` | No | |
| `ESP_HOSTED_UART_BAUDRATE` | No | |
| `ESP_HOSTED_UART_CHECKSUM` | No | |
| `ESP_HOSTED_UART_GPIO_RESET_SLAVE` | No | |
| `ESP_HOSTED_UART_HOST_INTERFACE` | No | |
| `ESP_HOSTED_UART_NUM_DATA_BITS` | No | |
| `ESP_HOSTED_UART_PARITY` | No | |
| `ESP_HOSTED_UART_PIN_RX` | No | |
| `ESP_HOSTED_UART_PIN_TX` | No | |
| `ESP_HOSTED_UART_PORT` | No | |
| `ESP_HOSTED_UART_PRIV_PARITY_EVEN` | No | |
| `ESP_HOSTED_UART_PRIV_PARITY_NONE` | No | |
| `ESP_HOSTED_UART_PRIV_PARITY_ODD` | No | |
| `ESP_HOSTED_UART_PRIV_STOP_BITS_1` | No | |
| `ESP_HOSTED_UART_PRIV_STOP_BITS_1_5` | No | |
| `ESP_HOSTED_UART_PRIV_STOP_BITS_2` | No | |
| `ESP_HOSTED_UART_RESET_ACTIVE_HIGH` | No | |
| `ESP_HOSTED_UART_RESET_ACTIVE_LOW` | No | |
| `ESP_HOSTED_UART_RX_Q_SIZE` | No | |
| `ESP_HOSTED_UART_RX_RANGE_MAX` | No | |
| `ESP_HOSTED_UART_RX_RANGE_MIN` | No | |
| `ESP_HOSTED_UART_STOP_BITS` | No | |
| `ESP_HOSTED_UART_TX_Q_SIZE` | No | |
| `ESP_HOSTED_UART_TX_RANGE_MAX` | No | |
| `ESP_HOSTED_UART_TX_RANGE_MIN` | No | |
| `ESP_HOSTED_USE_MEMPOOL` | No | |
| `ESP_HOSTED_WIFI_AUTO_CONNECT_ON_STA_START` | No | |

## RPC Request Matrix (MCU)

| Legacy RPC Req ID | Present In esp_hosted.new | Notes |
|---|---|---|
| `RPC_ID__Req_AppGetDesc` | Yes |  |
| `RPC_ID__Req_Base` | Yes |  |
| `RPC_ID__Req_ConfigHeartbeat` | Yes |  |
| `RPC_ID__Req_CustomRpc` | Yes |  |
| `RPC_ID__Req_EapClearCaCert` | Yes |  |
| `RPC_ID__Req_EapClearCertificateAndKey` | Yes |  |
| `RPC_ID__Req_EapClearIdentity` | Yes |  |
| `RPC_ID__Req_EapClearNewPassword` | Yes |  |
| `RPC_ID__Req_EapClearPassword` | Yes |  |
| `RPC_ID__Req_EapClearUsername` | Yes |  |
| `RPC_ID__Req_EapGetDisableTimeCheck` | Yes |  |
| `RPC_ID__Req_EapSetCaCert` | Yes |  |
| `RPC_ID__Req_EapSetCertificateAndKey` | Yes |  |
| `RPC_ID__Req_EapSetDisableTimeCheck` | Yes |  |
| `RPC_ID__Req_EapSetDomainName` | Yes |  |
| `RPC_ID__Req_EapSetEapMethods` | Yes |  |
| `RPC_ID__Req_EapSetFastParams` | Yes |  |
| `RPC_ID__Req_EapSetIdentity` | Yes |  |
| `RPC_ID__Req_EapSetNewPassword` | Yes |  |
| `RPC_ID__Req_EapSetPacFile` | Yes |  |
| `RPC_ID__Req_EapSetPassword` | Yes |  |
| `RPC_ID__Req_EapSetSuitebCertification` | Yes |  |
| `RPC_ID__Req_EapSetTtlsPhase2Method` | Yes |  |
| `RPC_ID__Req_EapSetUsername` | Yes |  |
| `RPC_ID__Req_EapUseDefaultCertBundle` | Yes |  |
| `RPC_ID__Req_ExtCoex` | No | External coex not yet ported |
| `RPC_ID__Req_FeatureControl` | Yes |  |
| `RPC_ID__Req_GetCoprocessorFwVersion` | Yes |  |
| `RPC_ID__Req_GetDhcpDnsStatus` | Yes |  |
| `RPC_ID__Req_GetMACAddress` | Yes |  |
| `RPC_ID__Req_GetWifiMode` | Yes |  |
| `RPC_ID__Req_GpioConfig` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioGetLevel` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioInputEnable` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioResetPin` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioSetDirection` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioSetLevel` | No | GPIO expander not yet ported |
| `RPC_ID__Req_GpioSetPullMode` | No | GPIO expander not yet ported |
| `RPC_ID__Req_IfaceMacAddrLenGet` | Yes |  |
| `RPC_ID__Req_IfaceMacAddrSetGet` | Yes |  |
| `RPC_ID__Req_Max` | Yes |  |
| `RPC_ID__Req_MemMonitor` | No | Mem monitor not yet ported |
| `RPC_ID__Req_OTAActivate` | Yes |  |
| `RPC_ID__Req_OTABegin` | Yes |  |
| `RPC_ID__Req_OTAEnd` | Yes |  |
| `RPC_ID__Req_OTAWrite` | Yes |  |
| `RPC_ID__Req_SetDhcpDnsStatus` | Yes |  |
| `RPC_ID__Req_SetMacAddress` | Yes |  |
| `RPC_ID__Req_SetWifiMode` | Yes |  |
| `RPC_ID__Req_SuppDppBootstrapGen` | Yes |  |
| `RPC_ID__Req_SuppDppDeinit` | Yes |  |
| `RPC_ID__Req_SuppDppInit` | Yes |  |
| `RPC_ID__Req_SuppDppStartListen` | Yes |  |
| `RPC_ID__Req_SuppDppStopListen` | Yes |  |
| `RPC_ID__Req_WifiApGetStaAid` | Yes |  |
| `RPC_ID__Req_WifiApGetStaList` | Yes |  |
| `RPC_ID__Req_WifiClearApList` | Yes |  |
| `RPC_ID__Req_WifiClearFastConnect` | Yes |  |
| `RPC_ID__Req_WifiConnect` | Yes |  |
| `RPC_ID__Req_WifiDeauthSta` | Yes |  |
| `RPC_ID__Req_WifiDeinit` | Yes |  |
| `RPC_ID__Req_WifiDisconnect` | Yes |  |
| `RPC_ID__Req_WifiGetBand` | Yes |  |
| `RPC_ID__Req_WifiGetBandMode` | Yes |  |
| `RPC_ID__Req_WifiGetBandwidth` | Yes |  |
| `RPC_ID__Req_WifiGetBandwidths` | Yes |  |
| `RPC_ID__Req_WifiGetChannel` | Yes |  |
| `RPC_ID__Req_WifiGetConfig` | Yes |  |
| `RPC_ID__Req_WifiGetCountry` | Yes |  |
| `RPC_ID__Req_WifiGetCountryCode` | Yes |  |
| `RPC_ID__Req_WifiGetInactiveTime` | Yes |  |
| `RPC_ID__Req_WifiGetMaxTxPower` | Yes |  |
| `RPC_ID__Req_WifiGetProtocol` | Yes |  |
| `RPC_ID__Req_WifiGetProtocols` | Yes |  |
| `RPC_ID__Req_WifiGetPs` | Yes |  |
| `RPC_ID__Req_WifiInit` | Yes |  |
| `RPC_ID__Req_WifiRestore` | Yes |  |
| `RPC_ID__Req_WifiScanGetApNum` | Yes |  |
| `RPC_ID__Req_WifiScanGetApRecord` | Yes |  |
| `RPC_ID__Req_WifiScanGetApRecords` | Yes |  |
| `RPC_ID__Req_WifiScanParams` | Yes |  |
| `RPC_ID__Req_WifiScanStart` | Yes |  |
| `RPC_ID__Req_WifiScanStop` | Yes |  |
| `RPC_ID__Req_WifiSetBand` | Yes |  |
| `RPC_ID__Req_WifiSetBandMode` | Yes |  |
| `RPC_ID__Req_WifiSetBandwidth` | Yes |  |
| `RPC_ID__Req_WifiSetBandwidths` | Yes |  |
| `RPC_ID__Req_WifiSetChannel` | Yes |  |
| `RPC_ID__Req_WifiSetConfig` | Yes |  |
| `RPC_ID__Req_WifiSetCountry` | Yes |  |
| `RPC_ID__Req_WifiSetCountryCode` | Yes |  |
| `RPC_ID__Req_WifiSetInactiveTime` | Yes |  |
| `RPC_ID__Req_WifiSetMaxTxPower` | Yes |  |
| `RPC_ID__Req_WifiSetOkcSupport` | Yes |  |
| `RPC_ID__Req_WifiSetProtocol` | Yes |  |
| `RPC_ID__Req_WifiSetProtocols` | Yes |  |
| `RPC_ID__Req_WifiSetPs` | Yes |  |
| `RPC_ID__Req_WifiSetStorage` | Yes |  |
| `RPC_ID__Req_WifiStaEnterpriseDisable` | Yes |  |
| `RPC_ID__Req_WifiStaEnterpriseEnable` | Yes |  |
| `RPC_ID__Req_WifiStaGetAid` | Yes |  |
| `RPC_ID__Req_WifiStaGetApInfo` | Yes |  |
| `RPC_ID__Req_WifiStaGetNegotiatedPhymode` | Yes |  |
| `RPC_ID__Req_WifiStaGetRssi` | Yes |  |
| `RPC_ID__Req_WifiStaItwtGetFlowIdStatus` | Yes |  |
| `RPC_ID__Req_WifiStaItwtSendProbeReq` | Yes |  |
| `RPC_ID__Req_WifiStaItwtSetTargetWakeTimeOffset` | Yes |  |
| `RPC_ID__Req_WifiStaItwtSetup` | Yes |  |
| `RPC_ID__Req_WifiStaItwtSuspend` | Yes |  |
| `RPC_ID__Req_WifiStaItwtTeardown` | Yes |  |
| `RPC_ID__Req_WifiStaTwtConfig` | Yes |  |
| `RPC_ID__Req_WifiStart` | Yes |  |
| `RPC_ID__Req_WifiStop` | Yes |  |
