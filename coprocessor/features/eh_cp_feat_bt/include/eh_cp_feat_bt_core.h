// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//
#ifndef EH_CP_FEAT_BT_CORE_H
#define EH_CP_FEAT_BT_CORE_H

#include "esp_err.h"
#include "esp_idf_version.h"
#include "priv/eh_cp_feat_bt_cfg.h"

#if defined(CONFIG_BT_ENABLED) && !defined(CONFIG_BT_CONTROLLER_ONLY) && !EH_CP_BT_STACK_ENABLED
#error "BT Host is enabled. This consumes memory and is not needed by ESP-Hosted. Only BT Controller is sufficient."
#error "============================================================================="
#error "Option 1) Disable 'BT stack' by selecting:"
#error "         idf.py menuconfig -> Component config -> Bluetooth -> Host -> Disabled"
#error "============================================================================="
#error "Option 2) Enable stack if you wish to handle bluetooth manually at slave:"
#error "         idf.py menuconfig -> ESP-Hosted FG Slave Config -> Enable BT stack:"
#error "         Advanced option: Handle bluetooth on your own at slave"
#error "============================================================================="
#endif


#if EH_CP_FEAT_BT_READY


#include "esp_bt.h"
#ifdef CONFIG_BT_HCI_UART_NO
#include "driver/uart.h"
#endif

// include only if BT component enabled and soc supports BT
#include "esp_bt.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
  #include "driver/periph_ctrl.h"
  #define DISABLE_INTR_ON_GPIO GPIO_PIN_INTR_DISABLE
#else
  #include "esp_private/periph_ctrl.h"
  #define DISABLE_INTR_ON_GPIO GPIO_INTR_DISABLE
#endif

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || \
	 defined(CONFIG_IDF_TARGET_ESP32S3))
  #define EH_CP_BT_OVER_C3_S3 1
#endif

#if CONFIG_IDF_TARGET_ESP32

  #if defined(CONFIG_BTDM_CONTROLLER_MODE_BLE_ONLY)
    #define EH_CP_BT_MODE_BLE    1
  #elif defined(CONFIG_BTDM_CONTROLLER_MODE_BR_EDR_ONLY)
    #define EH_CP_BT_MODE_BT     2
  #elif defined(CONFIG_BTDM_CONTROLLER_MODE_BTDM)
    #define EH_CP_BT_MODE_DUAL 3
  #endif

  #if defined(CONFIG_BTDM_CONTROLLER_HCI_MODE_VHCI)
    #define EH_CP_BT_HCI    4
  #elif CONFIG_BT_HCI_UART_NO
    #define EH_CP_BT_UART   CONFIG_BT_HCI_UART_NO
  #elif CONFIG_BTDM_CTRL_HCI_UART_NO
    #define EH_CP_BT_UART   CONFIG_BTDM_CTRL_HCI_UART_NO
  #endif

#elif EH_CP_BT_OVER_C3_S3

  #define EH_CP_BT_MODE_BLE      1

  #if defined(CONFIG_BT_CTRL_HCI_MODE_VHCI)
    #define EH_CP_BT_HCI    4
  #elif CONFIG_BT_CTRL_HCI_MODE_UART_H4
    #define EH_CP_BT_UART   EH_CP_BT_UART_PORT_C3_S3
  #endif

#else
  /* only BLE for chipsets other than ESP32 */

  #define EH_CP_BT_MODE_BLE      1

  #if defined(CONFIG_BT_LE_HCI_INTERFACE_USE_RAM)
    #define EH_CP_BT_HCI    4
  #elif defined(CONFIG_BT_LE_HCI_INTERFACE_USE_UART)
    #define EH_CP_BT_UART   CONFIG_BT_LE_HCI_UART_PORT
  #endif

#endif

#ifdef EH_CP_BT_UART

  #include "driver/uart.h"

  #if defined(CONFIG_IDF_TARGET_ESP32)

    // GPIO pins are fixed
    #define BT_TX_PIN	       EH_CP_BT_UART_TX_PIN_ESP32
    #define BT_RX_PIN	       EH_CP_BT_UART_RX_PIN_ESP32

    #if defined(CONFIG_BTDM_CTRL_HCI_UART_FLOW_CTRL_EN)
    #define BT_RTS_PIN         EH_CP_BT_UART_RTS_PIN_ESP32
    #define BT_CTS_PIN         EH_CP_BT_UART_CTS_PIN_ESP32
    #else
    #define BT_RTS_PIN         -1
    #define BT_CTS_PIN         -1
    #endif

  #elif EH_CP_BT_OVER_C3_S3

    #define BT_BAUDRATE         EH_CP_BT_UART_BAUDRATE_C3_S3

    #if defined(CONFIG_IDF_TARGET_ESP32C3)

      #define BT_TX_PIN         EH_CP_BT_UART_TX_PIN_C3_S3
      #define BT_RX_PIN         EH_CP_BT_UART_RX_PIN_C3_S3

      #define BT_FLOWCTRL       EH_CP_BT_UART_FLOWCTRL_C3_S3

      #if EH_CP_BT_UART_FLOWCTRL_C3_S3
      #define BT_RTS_PIN        EH_CP_BT_UART_RTS_PIN_C3_S3
      #define BT_CTS_PIN        EH_CP_BT_UART_CTS_PIN_C3_S3
      #else
      #define BT_RTS_PIN        -1
      #define BT_CTS_PIN        -1
      #endif

    #elif defined(CONFIG_IDF_TARGET_ESP32S3)

      #define BT_TX_PIN         EH_CP_BT_UART_TX_PIN_C3_S3
      #define BT_RX_PIN         EH_CP_BT_UART_RX_PIN_C3_S3

      #define BT_FLOWCTRL       EH_CP_BT_UART_FLOWCTRL_C3_S3

      #if EH_CP_BT_UART_FLOWCTRL_C3_S3
      #define BT_RTS_PIN        EH_CP_BT_UART_RTS_PIN_C3_S3
      #define BT_CTS_PIN        EH_CP_BT_UART_CTS_PIN_C3_S3
      #else
      #define BT_RTS_PIN        -1
      #define BT_CTS_PIN        -1
      #endif

    #endif

    #define UART_RX_THRS       (120)

    #if defined(EH_CP_BT_UART_FLOWCTRL_ENABLED)
    #define GPIO_OUTPUT_PIN_SEL  ((1ULL<<BT_TX_PIN) | (1ULL<<BT_RTS_PIN))
    #define GPIO_INPUT_PIN_SEL   ((1ULL<<BT_RX_PIN) | (1ULL<<BT_CTS_PIN))
    #else
    #define GPIO_OUTPUT_PIN_SEL  (1ULL<<BT_TX_PIN)
    #define GPIO_INPUT_PIN_SEL   (1ULL<<BT_RX_PIN)
    #endif

  #else

    #define BT_TX_PIN         CONFIG_BT_LE_HCI_UART_TX_PIN
    #define BT_RX_PIN         CONFIG_BT_LE_HCI_UART_RX_PIN

    #if defined(CONFIG_BT_LE_HCI_UART_FLOWCTRL)
    #define BT_RTS_PIN        CONFIG_BT_LE_HCI_UART_RTS_PIN
    #define BT_CTS_PIN        CONFIG_BT_LE_HCI_UART_CTS_PIN
    #else
    #define BT_RTS_PIN        -1
    #define BT_CTS_PIN        -1
    #endif

  #endif

#elif EH_CP_BT_HCI
  void process_hci_rx_pkt(uint8_t *payload, uint16_t payload_len);
#endif

esp_err_t eh_cp_bt_init(void);
esp_err_t eh_cp_bt_enable(void);
esp_err_t eh_cp_bt_disable(void);
esp_err_t eh_cp_bt_deinit(bool mem_release);

uint8_t eh_cp_bt_get_capabilities(void);
uint32_t eh_cp_bt_get_ext_capabilities(void);

#endif /* EH_CP_FEAT_BT_READY */
#endif /* EH_CP_FEAT_BT_CORE_H */
