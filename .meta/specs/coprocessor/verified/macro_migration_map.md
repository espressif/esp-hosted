<!-- %% sp.co.ve-mm.o %% - context -->
---
type: spec
last_verified: 2026-04-02
---

# Coprocessor Macro Migration Map

Legacy (`esp_hosted_mcu`) → New (`eh_hosted`). This is the single source of truth for macro naming.

<!-- %% sp.co.ve-mm.idf.o %% - context -->
## IDF Compat (centralize in `eh_cp_idf_compat.h`)

These are compound IDF-version-dependent checks. Must use `EH_CP_*` prefix.
Currently `H_*` in `eh_cp_feat_rpc_ext_mcu_wifi_features.h` — migrate here.

| Legacy H_* | New EH_CP_* | Condition | Status |
|------------|-------------|-----------|--------|
| H_WIFI_HE_GREATER_THAN_ESP_IDF_5_3 | EH_CP_WIFI_HE_GT_IDF_5_3 | SOC_WIFI_HE_SUPPORT && IDF > 5.3.0 | ✅ |
| H_PRESENT_IN_ESP_IDF_5_5_0 | EH_CP_IDF_GE_5_5 | IDF >= 5.5.0 | ✅ |
| H_PRESENT_IN_ESP_IDF_5_4_0 | EH_CP_IDF_GE_5_4 | IDF >= 5.4.0 | ✅ |
| H_WIFI_NEW_RESERVED_FIELD_NAMES | EH_CP_WIFI_NEW_RESERVED_FIELDS | IDF >= 5.5.0 | ✅ |
| H_GOT_TWT_ENABLE_KEEP_ALIVE | EH_CP_WIFI_GOT_TWT_KEEP_ALIVE | IDF > 5.3.1 | ✅ |
| H_GOT_AP_CONFIG_PARAM_TRANSITION_DISABLE | EH_CP_WIFI_GOT_AP_TRANSITION_DISABLE | IDF >= 5.3.3 or >= 5.4.1 | ✅ |
| H_OTA_CHECK_IMAGE_VALIDITY | EH_CP_OTA_CHECK_IMAGE_VALIDITY | IDF < 6.0.0 | ✅ |
| H_WIFI_ENTERPRISE_SUPPORT | EH_CP_WIFI_ENTERPRISE | CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT | ✅ |
| H_GOT_SET_EAP_METHODS_API | EH_CP_WIFI_GOT_SET_EAP_METHODS | compound IDF version | ✅ |
| H_GOT_EAP_SET_DOMAIN_NAME | EH_CP_WIFI_GOT_EAP_SET_DOMAIN_NAME | compound IDF version | ✅ |
| H_GOT_EAP_OKC_SUPPORT | EH_CP_WIFI_GOT_EAP_OKC | compound IDF version | ✅ |
| H_SUPP_DPP_SUPPORT | EH_CP_WIFI_SUPP_DPP | DPP_SUPPORT && IDF < 6.0 | ✅ |
| H_WIFI_DPP_SUPPORT | EH_CP_WIFI_DPP | DPP_SUPPORT && IDF >= 5.5 | ✅ |
| H_DPP_SUPPORT | EH_CP_WIFI_DPP_ANY | SUPP_DPP or WIFI_DPP | ✅ |
| H_IF_AVAILABLE_SPI_SLAVE_ENABLE_DISABLE | EH_CP_IDF_SPI_SLAVE_EN_DIS | IDF >= 5.5.0 | ✅ |
| (new) CONFIG_SOC_WIFI_HE_SUPPORT | EH_CP_SOC_WIFI_HE | CONFIG_SOC_WIFI_HE_SUPPORT | ✅ |
<!-- %% sp.co.ve-mm.idf.c %% -->

<!-- %% sp.co.ve-mm.sys.o %% - context -->
## System Compat (centralize in `eh_cp_idf_compat.h`)

| Legacy H_* | New EH_CP_* | Condition | Status |
|------------|-------------|-----------|--------|
| H_ALLOW_FULL_APP_DESC | EH_CP_ALLOW_FULL_APP_DESC | CONFIG_ESP_HOSTED_ALLOW_FULL_APP_DESC | ✅ |
| H_GOT_EFUSE_BLK_REV_FULL_APP_DESC | EH_CP_GOT_EFUSE_BLK_REV_APP_DESC | FULL_APP_DESC && IDF > 5.3.1 | ✅ |
| H_GOT_MMU_PAGE_SIZE_FULL_APP_DESC | EH_CP_GOT_MMU_PAGE_SIZE_APP_DESC | FULL_APP_DESC && IDF >= 5.4.0 | ✅ |
| H_USE_MEMPOOL | EH_CP_USE_MEMPOOL | CONFIG_ESP_HOSTED_USE_MEMPOOL | ✅ |
<!-- %% sp.co.ve-mm.sys.c %% -->

<!-- %% sp.co.ve-mm.bt.o %% - context -->
## Bluetooth Derived Macros (in `eh_cp_feat_bt_core.h`)

These are chip-specific derived macros from IDF BT Kconfig. Stay inside BT extension.

| Legacy | New EH_CP_BT_* | Derivation | Status |
|--------|----------------|------------|--------|
| BT_OVER_C3_S3 | EH_CP_BT_OVER_C3_S3 | CONFIG_IDF_TARGET_ESP32C3 or S3 | ✅ |
| BLUETOOTH_BLE | EH_CP_BT_MODE_BLE | CONFIG_BTDM_* / CONFIG_BT_CTRL_* / CONFIG_BT_LE_* | ✅ |
| BLUETOOTH_BT | EH_CP_BT_MODE_BT | CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY | ✅ |
| BLUETOOTH_BT_BLE | EH_CP_BT_MODE_DUAL | CONFIG_BTDM_CONTROLLER_MODE_BTDM | ✅ |
| BLUETOOTH_HCI | EH_CP_BT_HCI | CONFIG_*_HCI_MODE_VHCI / CONFIG_*_USE_RAM | ✅ |
| BLUETOOTH_UART | EH_CP_BT_UART | CONFIG_BT_HCI_UART_NO / CONFIG_*_UART_H4 | ✅ |

Note: These have complex chip-conditional logic. The derivation stays in the BT extension
header, but the macro names must use `EH_CP_BT_*` prefix.
<!-- %% sp.co.ve-mm.bt.c %% -->

<!-- %% sp.co.ve-mm.feat.o %% - context -->
## Feature Extension Guards

| Legacy | New EH_CP_* | Location | Status |
|--------|-------------|----------|--------|
| CONFIG_ESP_HOSTED_CP_BT | EH_CP_FEAT_BT_READY | master_config.h | ✅ |
| CONFIG_ESP_HOSTED_CP_WIFI | EH_CP_FEAT_WIFI_READY | master_config.h | ✅ |
| CONFIG_ESP_HOSTED_ENABLE_GPIO_EXPANDER / H_GPIO_EXPANDER_SUPPORT | EH_CP_FEAT_GPIO_EXP_READY | 🔲 new extension | ✅ |
| CONFIG_ESP_HOSTED_MEM_MONITOR | EH_CP_FEAT_MEM_MONITOR_READY | 🔲 new extension | ✅ |
| CONFIG_ESP_HOSTED_CP_EXT_COEX / H_EXT_COEX_SUPPORT | EH_CP_FEAT_EXT_COEX_READY | 🔲 new extension | ✅ |
| CONFIG_ESP_HOSTED_LIGHT_SLEEP_ENABLE | EH_CP_FEAT_LIGHT_SLEEP_READY | 🔲 new extension | ✅ |
| CONFIG_ESP_HOSTED_NETWORK_SPLIT_ENABLED | EH_CP_FEAT_NW_SPLIT_READY | master_config.h | ✅ |
| CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER | EH_CP_FEAT_PEER_DATA_TRANSFER_READY | master_config.h | ✅ |
| CONFIG_ESP_HOSTED_HOST_POWER_SAVE_ENABLED / H_HOST_PS_ALLOWED | EH_CP_FEAT_HOST_PS_READY | master_config.h | ✅ |
<!-- %% sp.co.ve-mm.feat.c %% -->

<!-- %% sp.co.ve-mm.power.o %% - context -->
## Power Management (Host PS extension)

| Legacy H_* | New EH_CP_* | Status |
|------------|-------------|--------|
| H_HOST_PS_ALLOWED | EH_CP_FEAT_HOST_PS_READY | ✅ (mapped in master_config) |
| H_HOST_PS_DEEP_SLEEP_ALLOWED | EH_CP_HOST_PS_DEEP_SLEEP | 🔲 map in master_config or host_ps cfg |
| H_HOST_WAKE_UP_GPIO | EH_CP_HOST_PS_WAKEUP_GPIO | ✅ |
| H_HOST_WAKEUP_GPIO_LEVEL | EH_CP_HOST_PS_WAKEUP_GPIO_LEVEL | ✅ |
| H_PS_UNLOAD_BUS_WHILE_PS | EH_CP_HOST_PS_UNLOAD_BUS | ✅ |
<!-- %% sp.co.ve-mm.power.c %% -->

<!-- %% sp.co.ve-mm.transport.o %% - context -->
## Transport (already mapped in master_config.h)

| Legacy CONFIG_* | New EH_CP_* | Status |
|-----------------|-------------|--------|
| CONFIG_ESP_SPI_HOST_INTERFACE | EH_CP_TRANSPORT_SPI | ✅ |
| CONFIG_ESP_SDIO_HOST_INTERFACE | EH_CP_TRANSPORT_SDIO | ✅ |
| CONFIG_ESP_SPI_HD_HOST_INTERFACE | EH_CP_TRANSPORT_SPI_HD | ✅ |
| CONFIG_ESP_UART_HOST_INTERFACE | EH_CP_TRANSPORT_UART | ✅ |
<!-- %% sp.co.ve-mm.transport.c %% -->

<!-- %% sp.co.ve-mm.c %% -->
