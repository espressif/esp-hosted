/* SPDX-License-Identifier: Apache-2.0 */
/* eh_rpc_bitmasks.h — RPC sub-message bit positions; must match CP bit-for-bit. */

#ifndef EH_RPC_BITMASKS_H_
#define EH_RPC_BITMASKS_H_

#if __has_include("esp_idf_version.h")
#  include "esp_idf_version.h"
#endif

#ifndef ESP_IDF_VERSION
/* Non-IDF: treat as newer-than-everything so 5.5+ branches compile. */
#  ifndef ESP_IDF_VERSION_VAL
#    define ESP_IDF_VERSION_VAL(major, minor, patch) \
        ((major) * 1000000 + (minor) * 1000 + (patch))
#  endif
#  define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(99, 0, 0)
#endif

#define EH_HOST_RPC_SET_BIT(pos, val)             ((val) |= (1u << (pos)))

#define EH_HOST_RPC_GET_BIT(pos, val)             (((val) & (1u << (pos))) ? 1 : 0)

enum {
    EH_HOST_WIFI_SCAN_AP_REC_phy_11b_BIT       = 0,
    EH_HOST_WIFI_SCAN_AP_REC_phy_11g_BIT       = 1,
    EH_HOST_WIFI_SCAN_AP_REC_phy_11n_BIT       = 2,
    EH_HOST_WIFI_SCAN_AP_REC_phy_lr_BIT        = 3,
    EH_HOST_WIFI_SCAN_AP_REC_phy_11ax_BIT      = 4,
    EH_HOST_WIFI_SCAN_AP_REC_wps_BIT           = 5,
    EH_HOST_WIFI_SCAN_AP_REC_ftm_responder_BIT = 6,
    EH_HOST_WIFI_SCAN_AP_REC_ftm_initiator_BIT = 7,
    EH_HOST_WIFI_SCAN_AP_REC_phy_11a_BIT       = 8,
    EH_HOST_WIFI_SCAN_AP_REC_phy_11ac_BIT      = 9,
    EH_HOST_WIFI_SCAN_AP_REC_MAX_USED_BIT      = 10,
};

#define EH_HOST_WIFI_SCAN_AP_RESERVED_BITMASK             0xFC00u

#define EH_HOST_WIFI_SCAN_AP_GET_RESERVED_VAL(num)                              \
    (((num) & EH_HOST_WIFI_SCAN_AP_RESERVED_BITMASK)                            \
        >> EH_HOST_WIFI_SCAN_AP_REC_MAX_USED_BIT)

#define EH_HOST_WIFI_SCAN_AP_SET_RESERVED_VAL(reserved_in, num_out)             \
    ((num_out) |= ((reserved_in) << EH_HOST_WIFI_SCAN_AP_REC_MAX_USED_BIT))

enum {
    EH_HOST_WIFI_STA_INFO_phy_11b_BIT       = 0,
    EH_HOST_WIFI_STA_INFO_phy_11g_BIT       = 1,
    EH_HOST_WIFI_STA_INFO_phy_11n_BIT       = 2,
    EH_HOST_WIFI_STA_INFO_phy_lr_BIT        = 3,
    EH_HOST_WIFI_STA_INFO_phy_11ax_BIT      = 4,
    EH_HOST_WIFI_STA_INFO_is_mesh_child_BIT = 5,
    EH_HOST_WIFI_STA_INFO_MAX_USED_BIT      = 6,
};

#define EH_HOST_WIFI_STA_INFO_RESERVED_BITMASK             0xFFC0u

#define EH_HOST_WIFI_STA_INFO_GET_RESERVED_VAL(num)                              \
    (((num) & EH_HOST_WIFI_STA_INFO_RESERVED_BITMASK)                            \
        >> EH_HOST_WIFI_STA_INFO_MAX_USED_BIT)

#define EH_HOST_WIFI_STA_INFO_SET_RESERVED_VAL(reserved_in, num_out)             \
    ((num_out) |= ((reserved_in) << EH_HOST_WIFI_STA_INFO_MAX_USED_BIT))

/* WIFI HE AP Info bitmasks */
enum {
    /* BSS_COLOR is six bits wide (0..5). */
    EH_HOST_WIFI_HE_AP_INFO_partial_bss_color_BIT  = 6,
    EH_HOST_WIFI_HE_AP_INFO_bss_color_disabled_BIT = 7,
    EH_HOST_WIFI_HE_AP_INFO_MAX_USED_BIT           = 8,
};

#define EH_HOST_WIFI_HE_AP_INFO_BSS_COLOR_BITS 0x3Fu

/*** Two sets of bitfields in wifi_sta_config_t */

/* WIFI Station Config Bitfield 1 bitmasks */
enum {
    EH_HOST_WIFI_STA_CONFIG_1_rm_enabled         = 0,
    EH_HOST_WIFI_STA_CONFIG_1_btm_enabled        = 1,
    EH_HOST_WIFI_STA_CONFIG_1_mbo_enabled        = 2,
    EH_HOST_WIFI_STA_CONFIG_1_ft_enabled         = 3,
    EH_HOST_WIFI_STA_CONFIG_1_owe_enabled        = 4,
    EH_HOST_WIFI_STA_CONFIG_1_transition_disable = 5,
    EH_HOST_WIFI_STA_CONFIG_1_MAX_USED_BIT       = 6,
};

#define EH_HOST_WIFI_STA_CONFIG_1_RESERVED_BITMASK 0xFFFFFFC0u

#define EH_HOST_WIFI_STA_CONFIG_1_GET_RESERVED_VAL(num)                          \
    (((num) & EH_HOST_WIFI_STA_CONFIG_1_RESERVED_BITMASK)                        \
        >> EH_HOST_WIFI_STA_CONFIG_1_MAX_USED_BIT)

#define EH_HOST_WIFI_STA_CONFIG_1_SET_RESERVED_VAL(reserved_in, num_out)         \
    ((num_out) |= ((reserved_in) << EH_HOST_WIFI_STA_CONFIG_1_MAX_USED_BIT))

/* WIFI Station Config Bitfield 2 bitmasks */
enum {
    EH_HOST_WIFI_STA_CONFIG_2_he_dcm_set_BIT                                     = 0,
    /* he_dcm_max_constellation_tx is two bits wide (1..2). */
    EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_tx_BITS                   = 1,
    /* he_dcm_max_constellation_rx is two bits wide (3..4). */
    EH_HOST_WIFI_STA_CONFIG_2_he_dcm_max_constellation_rx_BITS                   = 3,
    EH_HOST_WIFI_STA_CONFIG_2_he_mcs9_enabled_BIT                                = 5,
    EH_HOST_WIFI_STA_CONFIG_2_he_su_beamformee_disabled_BIT                      = 6,
    EH_HOST_WIFI_STA_CONFIG_2_he_trig_su_bmforming_feedback_disabled_BIT         = 7,
    EH_HOST_WIFI_STA_CONFIG_2_he_trig_mu_bmforming_partial_feedback_disabled_BIT = 8,
    EH_HOST_WIFI_STA_CONFIG_2_he_trig_cqi_feedback_disabled_BIT                  = 9,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
    EH_HOST_WIFI_STA_CONFIG_2_MAX_USED_BIT                                       = 10,
#else
    EH_HOST_WIFI_STA_CONFIG_2_vht_su_beamformee_disabled                         = 10,
    EH_HOST_WIFI_STA_CONFIG_2_vht_mu_beamformee_disabled                         = 11,
    EH_HOST_WIFI_STA_CONFIG_2_vht_mcs8_enabled                                   = 12,
    EH_HOST_WIFI_STA_CONFIG_2_MAX_USED_BIT                                       = 13,
#endif
};

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
#  define EH_HOST_WIFI_STA_CONFIG_2_RESERVED_BITMASK 0xFFFFFC00u
#else
#  define EH_HOST_WIFI_STA_CONFIG_2_RESERVED_BITMASK 0xFFFFE000u
#endif

#define EH_HOST_WIFI_STA_CONFIG_2_GET_RESERVED_VAL(num)                          \
    (((num) & EH_HOST_WIFI_STA_CONFIG_2_RESERVED_BITMASK)                        \
        >> EH_HOST_WIFI_STA_CONFIG_2_MAX_USED_BIT)

#define EH_HOST_WIFI_STA_CONFIG_2_SET_RESERVED_VAL(reserved_in, num_out)         \
    ((num_out) |= ((reserved_in) << EH_HOST_WIFI_STA_CONFIG_2_MAX_USED_BIT))

/* WIFI ITWT Setup Config bitmasks */
enum {
    EH_HOST_WIFI_ITWT_CONFIG_1_trigger_BIT            = 0,
    EH_HOST_WIFI_ITWT_CONFIG_1_flow_type_BIT          = 1,
    /* flow_id_BIT is three bits wide (2..4). */
    EH_HOST_WIFI_ITWT_CONFIG_1_flow_id_BIT            = 2,
    /* wake_invl_expn_BIT is five bits wide (5..9). */
    EH_HOST_WIFI_ITWT_CONFIG_1_wake_invl_expn_BIT     = 5,
    EH_HOST_WIFI_ITWT_CONFIG_1_wake_duration_unit_BIT = 10,
    EH_HOST_WIFI_ITWT_CONFIG_1_MAX_USED_BIT           = 11,
};

#define EH_HOST_WIFI_ITWT_CONFIG_1_RESERVED_BITMASK 0xFFFFF800u

#define EH_HOST_WIFI_ITWT_CONFIG_1_GET_RESERVED_VAL(num)                         \
    (((num) & EH_HOST_WIFI_ITWT_CONFIG_1_RESERVED_BITMASK)                       \
        >> EH_HOST_WIFI_ITWT_CONFIG_1_MAX_USED_BIT)

#define EH_HOST_WIFI_ITWT_CONFIG_1_SET_RESERVED_VAL(reserved_in, num_out)        \
    ((num_out) |= ((reserved_in) << EH_HOST_WIFI_ITWT_CONFIG_1_MAX_USED_BIT))

#endif /* EH_RPC_BITMASKS_H_ */
