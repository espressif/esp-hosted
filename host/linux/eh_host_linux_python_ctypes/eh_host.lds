/* SPDX-License-Identifier: Apache-2.0
 *
 * eh_host.lds — version script for libeh_host.so.
 *
 * CANONICAL hand-picked export surface.  Every symbol exposed by
 * libeh_host.so must be listed below; everything else stays local.
 *
 * Inclusion rule:
 *   - eh_host_*    : public C API consumed by external code.
 *   - esp_hosted_* : upstream-MCU back-compat orchestration APIs.
 *   - esp_event_*  : event-API forwarders (handler registration +
 *                    posting against the same loop the .so uses).
 *   - esp_netif_*  : IDF netif forwarders (IP + interface plumbing).
 *   - Event-base data globals (WIFI_EVENT, IP_EVENT, …).
 *
 * Exclusion rule (kept local):
 *   - eh_host_port_*  : OS abstraction — consumers own their OS.
 *   - eh_host_transport_*, eh_host_auto_*, eh_host_core_lifecycle_*
 *   - eh_host_feat_*_decode / _pack / _proto_ops, *_reset_subscribers
 *
 * Authority: this file is the single source of truth for the public
 * ABI of the built libeh_host.so.
 */

EH_HOST_1.0 {
    global:
        /* ── Core lifecycle ──────────────────────────────────── */
        eh_host_init;
        eh_host_init_linux_default;
        eh_host_deinit;
        eh_host_iface_mac_addr_get;
        eh_host_iface_mac_addr_len_get;
        eh_host_iface_mac_addr_set;
        eh_host_heartbeat_configure;
        eh_host_set_mem_monitor;

        /* ── System ──────────────────────────────────────────── */
        eh_host_sys_get_cp_app_desc;
        eh_host_sys_get_cp_fw_version;
        eh_host_sys_get_cp_info;
        eh_host_sys_get_mac;
        eh_host_sys_register_event_handlers;
        eh_host_sys_set_mac;
        eh_host_sys_unregister_event_handlers;

        /* ── Bluetooth ───────────────────────────────────────── */
        eh_host_bt_controller_deinit;
        eh_host_bt_controller_disable;
        eh_host_bt_controller_enable;
        eh_host_bt_controller_init;
        eh_host_bt_hci_send;
        eh_host_bt_set_hci_rx_cb;

        /* ── External coexistence ────────────────────────────── */
        eh_host_cp_ext_coex_disable;
        eh_host_cp_ext_coex_set_gpio_pin;
        eh_host_cp_ext_coex_set_grant_delay;
        eh_host_cp_ext_coex_set_validate_high;
        eh_host_cp_ext_coex_set_work_mode;

        /* ── GPIO expander ───────────────────────────────────── */
        eh_host_cp_gpio_config;
        eh_host_cp_gpio_get_level;
        eh_host_cp_gpio_input_enable;
        eh_host_cp_gpio_reset_pin;
        eh_host_cp_gpio_set_direction;
        eh_host_cp_gpio_set_level;
        eh_host_cp_gpio_set_pull_mode;

        /* ── Memory monitor ──────────────────────────────────── */
        eh_host_mem_monitor_register_event_handlers;
        eh_host_mem_monitor_unregister_event_handlers;

        /* ── Network split ───────────────────────────────────── */
        eh_host_nw_split_bind_mac;
        eh_host_nw_split_get_status;
        eh_host_nw_split_register_event_handlers;
        eh_host_nw_split_set_status;
        eh_host_nw_split_unregister_event_handlers;

        /* ── OTA ─────────────────────────────────────────────── */
        eh_host_cp_ota_activate;
        eh_host_cp_ota_begin;
        eh_host_cp_ota_end;
        eh_host_cp_ota_write;

        /* ── Peer data transfer ──────────────────────────────── */
        eh_host_peer_data_register;
        eh_host_peer_data_register_event_handlers;
        eh_host_peer_data_send;
        eh_host_peer_data_unregister_event_handlers;

        /* ── Linux vserial transport ─────────────────────────── */
        eh_host_linux_vserial_deinit;
        eh_host_linux_vserial_get_ops;
        eh_host_linux_vserial_init;

        /* ── Feature init/deinit ─────────────────────────────── */
        eh_host_feat_bt_deinit;
        eh_host_feat_bt_init;
        eh_host_feat_cp_ext_coex_deinit;
        eh_host_feat_cp_ext_coex_init;
        eh_host_feat_gpio_exp_deinit;
        eh_host_feat_gpio_exp_init;
        eh_host_feat_heartbeat_deinit;
        eh_host_feat_heartbeat_init;
        eh_host_feat_mem_monitor_deinit;
        eh_host_feat_mem_monitor_init;
        eh_host_feat_nw_split_deinit;
        eh_host_feat_nw_split_init;
        eh_host_feat_peer_data_deinit;
        eh_host_feat_peer_data_init;
        eh_host_feat_system_deinit;
        eh_host_feat_system_init;
        eh_host_feat_wifi_deinit;
        eh_host_feat_wifi_ext_dpp_deinit;
        eh_host_feat_wifi_ext_dpp_init;
        eh_host_feat_wifi_ext_ent_deinit;
        eh_host_feat_wifi_ext_ent_init;
        eh_host_feat_wifi_ext_itwt_deinit;
        eh_host_feat_wifi_ext_itwt_init;
        eh_host_feat_wifi_init;

        /* ── RPC: ext_v1_mcu wire codec public hooks ─────────── */
        eh_host_feat_rpc_ext_v2_decode;
        eh_host_feat_rpc_ext_v2_id_ranges;
        eh_host_feat_rpc_ext_v2_pack;
        eh_host_feat_rpc_ext_v2_proto_ops;
        eh_host_feat_rpc_ext_v2_register_wifi_event_handlers;
        eh_host_feat_rpc_ext_v2_unregister_wifi_event_handlers;

        /* ── RPC: low-level public API ───────────────────────── */
        eh_host_feat_rpc_deinit;
        eh_host_feat_rpc_init;
        eh_host_feat_rpc_is_running;
        eh_host_feat_rpc_register_event;
        eh_host_feat_rpc_request;
        eh_host_feat_rpc_request_sync;
        eh_host_feat_rpc_send;
        eh_host_feat_rpc_start;
        eh_host_feat_rpc_stop;
        eh_host_feat_rpc_unregister_event;

        /* ── Wi-Fi: lifecycle ────────────────────────────────── */
        eh_host_wifi_init;
        eh_host_wifi_deinit;
        eh_host_wifi_start;
        eh_host_wifi_stop;
        eh_host_wifi_restore;
        eh_host_wifi_connect;
        eh_host_wifi_disconnect;

        /* ── Wi-Fi: AP-side ──────────────────────────────────── */
        eh_host_wifi_ap_get_sta_aid;
        eh_host_wifi_ap_get_sta_list;
        eh_host_wifi_deauth_sta;

        /* ── Wi-Fi: STA-side ─────────────────────────────────── */
        eh_host_wifi_sta_get_aid;
        eh_host_wifi_sta_get_ap_info;
        eh_host_wifi_sta_get_negotiated_phymode;
        eh_host_wifi_sta_get_rssi;
        eh_host_wifi_sta_enterprise_disable;
        eh_host_wifi_sta_enterprise_enable;

        /* ── Wi-Fi: scanning + AP list ───────────────────────── */
        eh_host_wifi_clear_ap_list;
        eh_host_wifi_clear_fast_connect;
        eh_host_wifi_scan_get_ap_num;
        eh_host_wifi_scan_get_ap_record;
        eh_host_wifi_scan_get_ap_records;
        eh_host_wifi_scan_start;
        eh_host_wifi_scan_stop;
        eh_host_wifi_get_scan_parameters;
        eh_host_wifi_set_scan_parameters;

        /* ── Wi-Fi: config getters ───────────────────────────── */
        eh_host_wifi_get_band;
        eh_host_wifi_get_band_mode;
        eh_host_wifi_get_bandwidth;
        eh_host_wifi_get_bandwidths;
        eh_host_wifi_get_channel;
        eh_host_wifi_get_config;
        eh_host_wifi_get_country;
        eh_host_wifi_get_country_code;
        eh_host_wifi_get_inactive_time;
        eh_host_wifi_get_max_tx_power;
        eh_host_wifi_get_mode;
        eh_host_wifi_get_protocol;
        eh_host_wifi_get_protocols;
        eh_host_wifi_get_ps;

        /* ── Wi-Fi: config setters ───────────────────────────── */
        eh_host_wifi_set_band;
        eh_host_wifi_set_band_mode;
        eh_host_wifi_set_bandwidth;
        eh_host_wifi_set_bandwidths;
        eh_host_wifi_set_channel;
        eh_host_wifi_set_config;
        eh_host_wifi_set_country;
        eh_host_wifi_set_country_code;
        eh_host_wifi_set_inactive_time;
        eh_host_wifi_set_max_tx_power;
        eh_host_wifi_set_mode;
        eh_host_wifi_set_okc_support;
        eh_host_wifi_set_protocol;
        eh_host_wifi_set_protocols;
        eh_host_wifi_set_ps;
        eh_host_wifi_set_storage;

        /* ── Wi-Fi: DPP (Device Provisioning Protocol) ───────── */
        eh_host_wifi_dpp_bootstrap_gen;
        eh_host_wifi_dpp_deinit;
        eh_host_wifi_dpp_init;
        eh_host_wifi_dpp_on_cfg_recvd;
        eh_host_wifi_dpp_on_fail;
        eh_host_wifi_dpp_on_uri_ready;
        eh_host_wifi_dpp_start_listen;
        eh_host_wifi_dpp_stop_listen;
        eh_host_wifi_dpp_subscribe_cfg_recvd;
        eh_host_wifi_dpp_subscribe_fail;
        eh_host_wifi_dpp_subscribe_uri_ready;
        eh_host_wifi_dpp_unsubscribe_cfg_recvd;
        eh_host_wifi_dpp_unsubscribe_fail;
        eh_host_wifi_dpp_unsubscribe_uri_ready;

        /* ── Wi-Fi: Enterprise (AP-side) ─────────────────────── */
        eh_host_wifi_ent_clear_ca_cert;
        eh_host_wifi_ent_clear_certificate_and_key;
        eh_host_wifi_ent_clear_identity;
        eh_host_wifi_ent_clear_new_password;
        eh_host_wifi_ent_clear_password;
        eh_host_wifi_ent_clear_username;
        eh_host_wifi_ent_enterprise_disable;
        eh_host_wifi_ent_enterprise_enable;
        eh_host_wifi_ent_get_disable_time_check;
        eh_host_wifi_ent_set_ca_cert;
        eh_host_wifi_ent_set_certificate_and_key;
        eh_host_wifi_ent_set_disable_time_check;
        eh_host_wifi_ent_set_domain_name;
        eh_host_wifi_ent_set_eap_methods;
        eh_host_wifi_ent_set_fast_params;
        eh_host_wifi_ent_set_identity;
        eh_host_wifi_ent_set_new_password;
        eh_host_wifi_ent_set_pac_file;
        eh_host_wifi_ent_set_password;
        eh_host_wifi_ent_set_suiteb_192bit_certification;
        eh_host_wifi_ent_set_ttls_phase2_method;
        eh_host_wifi_ent_set_username;
        eh_host_wifi_ent_use_default_cert_bundle;

        /* ── Wi-Fi: STA Enterprise (EAP) ─────────────────────── */
        eh_host_wifi_sta_eap_clear_ca_cert;
        eh_host_wifi_sta_eap_clear_certificate_and_key;
        eh_host_wifi_sta_eap_clear_identity;
        eh_host_wifi_sta_eap_clear_new_password;
        eh_host_wifi_sta_eap_clear_password;
        eh_host_wifi_sta_eap_clear_username;
        eh_host_wifi_sta_eap_get_disable_time_check;
        eh_host_wifi_sta_eap_set_ca_cert;
        eh_host_wifi_sta_eap_set_certificate_and_key;
        eh_host_wifi_sta_eap_set_disable_time_check;
        eh_host_wifi_sta_eap_set_domain_name;
        eh_host_wifi_sta_eap_set_eap_methods;
        eh_host_wifi_sta_eap_set_fast_params;
        eh_host_wifi_sta_eap_set_identity;
        eh_host_wifi_sta_eap_set_new_password;
        eh_host_wifi_sta_eap_set_pac_file;
        eh_host_wifi_sta_eap_set_password;
        eh_host_wifi_sta_eap_set_suiteb_192bit_certification;
        eh_host_wifi_sta_eap_set_ttls_phase2_method;
        eh_host_wifi_sta_eap_set_username;
        eh_host_wifi_sta_eap_use_default_cert_bundle;

        /* ── Wi-Fi: iTWT (individual Target Wake Time) ───────── */
        eh_host_wifi_itwt_get_flow_id_status;
        eh_host_wifi_itwt_send_probe_req;
        eh_host_wifi_itwt_set_target_wake_time_offset;
        eh_host_wifi_itwt_setup;
        eh_host_wifi_itwt_subscribe_probe;
        eh_host_wifi_itwt_subscribe_setup;
        eh_host_wifi_itwt_subscribe_suspend;
        eh_host_wifi_itwt_subscribe_teardown;
        eh_host_wifi_itwt_suspend;
        eh_host_wifi_itwt_teardown;
        eh_host_wifi_itwt_unsubscribe_probe;
        eh_host_wifi_itwt_unsubscribe_setup;
        eh_host_wifi_itwt_unsubscribe_suspend;
        eh_host_wifi_itwt_unsubscribe_teardown;

        /* ── Wi-Fi: STA-side iTWT ────────────────────────────── */
        eh_host_wifi_sta_itwt_get_flow_id_status;
        eh_host_wifi_sta_itwt_on_probe;
        eh_host_wifi_sta_itwt_on_setup;
        eh_host_wifi_sta_itwt_on_suspend;
        eh_host_wifi_sta_itwt_on_teardown;
        eh_host_wifi_sta_itwt_send_probe_req;
        eh_host_wifi_sta_itwt_set_target_wake_time_offset;
        eh_host_wifi_sta_itwt_setup;
        eh_host_wifi_sta_itwt_suspend;
        eh_host_wifi_sta_itwt_teardown;
        eh_host_wifi_sta_twt_config;

        /* ── esp_hosted_* (upstream-MCU back-compat) ─────────── */
        esp_hosted_connect_to_slave;
        esp_hosted_deinit;
        esp_hosted_get_coprocessor_fwversion;
        esp_hosted_get_cp_info;
        esp_hosted_init;

        /* ── IDF event API — handler registration + posting ──── */
        esp_event_handler_instance_register;
        esp_event_handler_instance_unregister;
        esp_event_handler_register;
        esp_event_handler_unregister;
        esp_event_loop_create_default;
        esp_event_loop_delete_default;
        esp_event_post;

        /* ── Event-base data globals ─────────────────────────── */
        EH_HOST_EVENT;
        ESP_HOSTED_EVENT;  /* alias of EH_HOST_EVENT (back-compat) */
        ETH_EVENT;
        IP_EVENT;
        WIFI_EVENT;

        /* ── IDF netif API — IP + interface plumbing ─────────── */
        esp_netif_action_connected;
        esp_netif_action_disconnected;
        esp_netif_action_got_ip;
        esp_netif_action_start;
        esp_netif_action_stop;
        esp_netif_attach;
        esp_netif_attach_wifi_station;
        esp_netif_create_default_wifi_ap;
        esp_netif_create_default_wifi_sta;
        esp_netif_create_wifi;
        esp_netif_create_wifi_remote;
        esp_netif_destroy;
        esp_netif_destroy_default_wifi;
        esp_netif_destroy_wifi_remote;
        esp_netif_dhcpc_get_status;
        esp_netif_dhcpc_start;
        esp_netif_dhcpc_stop;
        esp_netif_find_if;
        esp_netif_get_desc;
        esp_netif_get_dns_info;
        esp_netif_get_handle_from_ifkey;
        esp_netif_get_ifkey;
        esp_netif_get_io_driver;
        esp_netif_get_ip_info;
        esp_netif_get_netif_impl;
        esp_netif_get_netif_impl_index;
        esp_netif_get_netif_impl_name;
        esp_netif_htonl;
        esp_netif_init;
        esp_netif_ip_addr_copy;
        esp_netif_is_netif_up;
        esp_netif_napt_enable;
        esp_netif_new;
        esp_netif_next_unsafe;
        esp_netif_ppp_get_params;
        esp_netif_ppp_set_params;
        esp_netif_receive;
        esp_netif_set_dns_info;
        esp_netif_set_driver_config;
        esp_netif_set_ip_info;
        esp_netif_set_mac;
        esp_netif_set_static_ip;
        esp_netif_tcpip_exec;
        esp_netif_transmit;

    local:
        *;
};
