// Copyright 2025 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0-only OR Apache-2.0 */

/* Bitmask definitions used in RPC data */

/* TODO: Add CI to check if the file here and host are exactly same for matching versions */
#ifndef __ESP_HOSTED_CP_RPC_MCU_BITMASKS__H
#define __ESP_HOSTED_CP_RPC_MCU_BITMASKS__H

#include "esp_idf_version.h"

#define H_SET_BIT(pos, val)                       (val|=(1<<pos))

#define H_GET_BIT(pos, val)                       (val&(1<<pos)? 1: 0)

enum {
	WIFI_SCAN_AP_REC_phy_11b_BIT       = 0,
	WIFI_SCAN_AP_REC_phy_11g_BIT       = 1,
	WIFI_SCAN_AP_REC_phy_11n_BIT       = 2,
	WIFI_SCAN_AP_REC_phy_lr_BIT        = 3,
	WIFI_SCAN_AP_REC_phy_11ax_BIT      = 4,
	WIFI_SCAN_AP_REC_wps_BIT           = 5,
	WIFI_SCAN_AP_REC_ftm_responder_BIT = 6,
	WIFI_SCAN_AP_REC_ftm_initiator_BIT = 7,
	WIFI_SCAN_AP_REC_phy_11a_BIT       = 8,
	WIFI_SCAN_AP_REC_phy_11ac_BIT      = 9,
	WIFI_SCAN_AP_REC_MAX_USED_BIT      = 10,
};

#define WIFI_SCAN_AP_RESERVED_BITMASK             0xFC00

#define WIFI_SCAN_AP_GET_RESERVED_VAL(num)                                      \
    ((num & WIFI_SCAN_AP_RESERVED_BITMASK) >> WIFI_SCAN_AP_REC_MAX_USED_BIT)

#define WIFI_SCAN_AP_SET_RESERVED_VAL(reserved_in,num_out)                      \
    (num_out |= (reserved_in << WIFI_SCAN_AP_REC_MAX_USED_BIT));

enum {
	WIFI_STA_INFO_phy_11b_BIT       = 0,
	WIFI_STA_INFO_phy_11g_BIT       = 1,
	WIFI_STA_INFO_phy_11n_BIT       = 2,
	WIFI_STA_INFO_phy_lr_BIT        = 3,
	WIFI_STA_INFO_phy_11ax_BIT      = 4,
	WIFI_STA_INFO_is_mesh_child_BIT = 5,
	WIFI_STA_INFO_MAX_USED_BIT      = 6,
};

#define WIFI_STA_INFO_RESERVED_BITMASK             0xFFC0

#define WIFI_STA_INFO_GET_RESERVED_VAL(num)                                      \
    ((num & WIFI_STA_INFO_RESERVED_BITMASK) >> WIFI_STA_INFO_MAX_USED_BIT)

#define WIFI_STA_INFO_SET_RESERVED_VAL(reserved_in,num_out)                      \
    (num_out |= (reserved_in << WIFI_STA_INFO_MAX_USED_BIT));

/* WIFI HE AP Info bitmasks */
enum {
	// WIFI_HE_AP_INFO_BSS_COLOR is six bits wide
	WIFI_HE_AP_INFO_partial_bss_color_BIT  = 6,
	WIFI_HE_AP_INFO_bss_color_disabled_BIT = 7,
	WIFI_HE_AP_INFO_MAX_USED_BIT           = 8,
};

#define WIFI_HE_AP_INFO_BSS_COLOR_BITS 0x3F

/*** There are currently two set of bitfields in wifi_sta_config_t */

/* WIFI Station Config Bitfield 1 bitmasks */
enum {
	WIFI_STA_CONFIG_1_rm_enabled = 0,
	WIFI_STA_CONFIG_1_btm_enabled = 1,
	WIFI_STA_CONFIG_1_mbo_enabled = 2,
	WIFI_STA_CONFIG_1_ft_enabled = 3,
	WIFI_STA_CONFIG_1_owe_enabled = 4,
	WIFI_STA_CONFIG_1_transition_disable = 5,
	WIFI_STA_CONFIG_1_MAX_USED_BIT = 6,
};

#define WIFI_STA_CONFIG_1_RESERVED_BITMASK 0xFFFFFFC0

#define WIFI_STA_CONFIG_1_GET_RESERVED_VAL(num)                                   \
    ((num & WIFI_STA_CONFIG_1_RESERVED_BITMASK) >> WIFI_STA_CONFIG_1_MAX_USED_BIT)

#define WIFI_STA_CONFIG_1_SET_RESERVED_VAL(reserved_in, num_out)                  \
    (num_out |= (reserved_in << WIFI_STA_CONFIG_1_MAX_USED_BIT));

/* WIFI Station Config Bitfield 2 bitmasks */
enum {
	WIFI_STA_CONFIG_2_he_dcm_set_BIT                                     = 0,
	// WIFI_STA_CONFIG_he_dcm_max_constellation_tx is two bits wide
	WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS                   = 1,
	// WIFI_STA_CONFIG_he_dcm_max_constellation_rx is two bits wide
	WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS                   = 3,
	WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT                                = 5,
	WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT                      = 6,
	WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT         = 7,
	WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT = 8,
	WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT                  = 9,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
	WIFI_STA_CONFIG_2_MAX_USED_BIT                                       = 10,
#else
	WIFI_STA_CONFIG_2_vht_su_beamformee_disabled                         = 10,
	WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled                         = 11,
	WIFI_STA_CONFIG_2_vht_mcs8_enabled                                   = 12,
	WIFI_STA_CONFIG_2_MAX_USED_BIT                                       = 13,
#endif
};

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
#define WIFI_STA_CONFIG_2_RESERVED_BITMASK 0xFFFFFC00
#else
#define WIFI_STA_CONFIG_2_RESERVED_BITMASK 0xFFFFE000
#endif

#define WIFI_STA_CONFIG_2_GET_RESERVED_VAL(num)                                    \
    ((num & WIFI_STA_CONFIG_2_RESERVED_BITMASK) >> WIFI_STA_CONFIG_2_MAX_USED_BIT)

#define WIFI_STA_CONFIG_2_SET_RESERVED_VAL(reserved_in,num_out)                    \
    (num_out |= (reserved_in << WIFI_STA_CONFIG_2_MAX_USED_BIT));

#endif

/* WIFI ITWT Setup Config bitmasks */

enum {
	WIFI_ITWT_CONFIG_1_trigger_BIT            = 0,
	WIFI_ITWT_CONFIG_1_flow_type_BIT          = 1,
	// WIFI_ITWT_CONFIG_1_flow_id_BIT is three bits wide
	WIFI_ITWT_CONFIG_1_flow_id_BIT            = 2,
	// WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT is five bits wide
	WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT     = 5,
	WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT = 10,
	WIFI_ITWT_CONFIG_1_MAX_USED_BIT           = 11,
};

#define WIFI_ITWT_CONFIG_1_RESERVED_BITMASK 0xFFFFF800

#define WIFI_ITWT_CONFIG_1_GET_RESERVED_VAL(num)                                    \
    ((num & WIFI_ITWT_CONFIG_1_RESERVED_BITMASK) >> WIFI_ITWT_CONFIG_1_MAX_USED_BIT)

#define WIFI_ITWT_CONFIG_1_SET_RESERVED_VAL(reserved_in,num_out)                    \
    (num_out |= (reserved_in << WIFI_ITWT_CONFIG_1_MAX_USED_BIT));
