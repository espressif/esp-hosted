/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>

#include "sdkconfig.h"
#include "eh_tlv_tags.h"
#include "eh_common_caps.h"
#include "eh_common_sdio_cfg.h"
#include "eh_frame.h"
#include "eh_common_fw_version.h"
#include "eh_host_port_master_config.h"
#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
#include "eh_host_port_dma.h"
#include "eh_mempool.h"   /* HOSTED_MEM_ALIGNMENT_64 */
#endif
#include "eh_host_mcu_transport_init_event.h"
#include "esp_log.h"
#include "eh_host_port_tags.h"   /* EH_HOST_PORT_RETAIN_ATTR */
#include "eh_host_raw_tp_stats.h"

/* Host always speaks RPC V2 (msg_id dispatch via rpc_v2.proto). */
#define EH_HOST_RPC_WIRE_VERSION  ESP_HOSTED_RPC_VERSION_V2

__attribute__((weak)) void eh_host_raw_tp_process_test_capabilities(uint8_t cap)
{
    (void)cap;
}

#define TAG "eh_init_evt"

static volatile uint8_t  s_chip_id      = EH_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
static volatile uint8_t  s_capabilities = 0;
static volatile uint32_t s_ext_caps     = 0;
static volatile uint32_t s_fw_version   = 0;
static volatile uint8_t  s_slave_sdio_streaming_mode = 0; /* 0=packet, 1=streaming */
static volatile bool     s_peer_advertised_rpc_version = false;
/* Retained across a host deep-sleep */
static EH_HOST_PORT_RETAIN_ATTR struct eh_priv_sdio_buf_config s_sdio_buf_cfg;   /* valid iff s_sdio_buf_cfg_valid */
static EH_HOST_PORT_RETAIN_ATTR volatile bool                  s_sdio_buf_cfg_valid = false;

uint8_t  eh_host_mcu_transport_get_chip_id(void)         { return s_chip_id; }
uint32_t eh_host_mcu_transport_get_fw_version(void)      { return s_fw_version; }
uint8_t  eh_host_mcu_transport_get_capabilities(void)    { return s_capabilities; }
uint32_t eh_host_mcu_transport_get_ext_capabilities(void){ return s_ext_caps; }
bool     eh_host_mcu_transport_peer_advertised_rpc_version(void) { return s_peer_advertised_rpc_version; }

const struct eh_priv_sdio_buf_config *eh_host_mcu_transport_sdio_buf_config(void)
{
    return s_sdio_buf_cfg_valid ? &s_sdio_buf_cfg : NULL;
}

bool eh_host_mcu_transport_sdio_sw_aggr_negotiated(void)
{
    return s_sdio_buf_cfg_valid &&
           s_sdio_buf_cfg.e2h_mode == EH_SDIO_CFG_TXMODE_SW_AGGR &&
           s_sdio_buf_cfg.h2e_mode == EH_SDIO_CFG_TXMODE_SW_AGGR;
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0]        |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16)|
           ((uint32_t)p[3] << 24);
}

static void print_capabilities(uint8_t cap)
{
    ESP_LOGI(TAG, "Features supported are:");
    if (cap & ESP_WLAN_SDIO_SUPPORT)   ESP_LOGI(TAG, "\t * WLAN over SDIO");
    if (cap & ESP_WLAN_SPI_SUPPORT)    ESP_LOGI(TAG, "\t * WLAN over SPI");
    if (cap & ESP_BT_UART_SUPPORT)     ESP_LOGI(TAG, "\t   - HCI over UART");
    if (cap & ESP_BT_SDIO_SUPPORT)     ESP_LOGI(TAG, "\t   - HCI over SDIO");
    if (cap & ESP_BT_SPI_SUPPORT)      ESP_LOGI(TAG, "\t   - HCI over SPI");
    if ((cap & ESP_BLE_ONLY_SUPPORT) && (cap & ESP_BR_EDR_ONLY_SUPPORT))
        ESP_LOGI(TAG, "\t   - BT/BLE dual mode");
    else if (cap & ESP_BLE_ONLY_SUPPORT)
        ESP_LOGI(TAG, "\t   - BLE only");
    else if (cap & ESP_BR_EDR_ONLY_SUPPORT)
        ESP_LOGI(TAG, "\t   - BR/EDR only");
    if (cap & ESP_CHECKSUM_ENABLED)    ESP_LOGI(TAG, "\t   - Wire-format checksum");
}

static void print_ext_capabilities(uint32_t ext)
{
    if (ext == 0)
        return;
    ESP_LOGI(TAG, "Extended Features supported:");
#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SPI_HD
    if (ext & EH_TRANSPORT_CP_SPI_HD_2_DATA_LINES) ESP_LOGI(TAG, "\t * SPI-HD 2 data lines");
    if (ext & EH_TRANSPORT_CP_SPI_HD_4_DATA_LINES) ESP_LOGI(TAG, "\t * SPI-HD 4 data lines");
#endif
    if (ext & ESP_WLAN_SUPPORT)        ESP_LOGI(TAG, "\t * WLAN");
    if (ext & ESP_WLAN_UART_SUPPORT)   ESP_LOGI(TAG, "\t * WLAN over UART");
    if (ext & ESP_EXT_CAP_WIFI_ENT)    ESP_LOGI(TAG, "\t * Wi-Fi Enterprise");
    if (ext & ESP_EXT_CAP_WIFI_DPP)    ESP_LOGI(TAG, "\t * Wi-Fi DPP");
    if (ext & ESP_EXT_CAP_HOST_PS)     ESP_LOGI(TAG, "\t * Host power-save");
    if (ext & ESP_EXT_CAP_NW_SPLIT)    ESP_LOGI(TAG, "\t * Network split");
    if (ext & ESP_EXT_CAP_CUSTOM_RPC)  ESP_LOGI(TAG, "\t * Custom RPC (peer data)");
    if (ext & ESP_EXT_CAP_OT)          ESP_LOGI(TAG, "\t * OpenThread");
}

static const char *chip_id_name(uint8_t id)
{
    switch (id) {
    case EH_PRIV_FIRMWARE_CHIP_ESP32:    return "esp32";
    case EH_PRIV_FIRMWARE_CHIP_ESP32S2:  return "esp32s2";
    case EH_PRIV_FIRMWARE_CHIP_ESP32C3:  return "esp32c3";
    case EH_PRIV_FIRMWARE_CHIP_ESP32S3:  return "esp32s3";
    case EH_PRIV_FIRMWARE_CHIP_ESP32C2:  return "esp32c2";
    case EH_PRIV_FIRMWARE_CHIP_ESP32C6:  return "esp32c6";
    case EH_PRIV_FIRMWARE_CHIP_ESP32H2:  return "esp32h2";
    case EH_PRIV_FIRMWARE_CHIP_ESP32C61: return "esp32c61";
    case EH_PRIV_FIRMWARE_CHIP_ESP32C5:  return "esp32c5";
    case EH_PRIV_FIRMWARE_CHIP_ESP32H4:  return "esp32h4";
    default:                             return "unsupported";
    }
}

static int chip_id_is_known(uint8_t id)
{
    switch (id) {
    case EH_PRIV_FIRMWARE_CHIP_ESP32:
    case EH_PRIV_FIRMWARE_CHIP_ESP32S2:
    case EH_PRIV_FIRMWARE_CHIP_ESP32S3:
    case EH_PRIV_FIRMWARE_CHIP_ESP32C2:
    case EH_PRIV_FIRMWARE_CHIP_ESP32C3:
    case EH_PRIV_FIRMWARE_CHIP_ESP32C6:
    case EH_PRIV_FIRMWARE_CHIP_ESP32C5:
    case EH_PRIV_FIRMWARE_CHIP_ESP32C61:
    case EH_PRIV_FIRMWARE_CHIP_ESP32H2:
    case EH_PRIV_FIRMWARE_CHIP_ESP32H4:
        return 1;
    default:
        return 0;
    }
}

#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
/* HARD ASSERT on slave=streaming + host=packet (slave's oversized frames are
 * undecodable). Reverse asymmetry is non-fatal — host can drain. */
static int sdio_mode_check(uint8_t slave_streaming)
{
# ifdef CONFIG_ESP_HOSTED_HOST_SDIO_RX_STREAMING_MODE
    uint8_t host_streaming = 1;
# else
    uint8_t host_streaming = 0;
# endif
    ESP_LOGI(TAG, "SDIO mode: slave=%s host=%s",
             slave_streaming ? "streaming" : "packet",
             host_streaming  ? "streaming" : "packet");

    if (slave_streaming && !host_streaming) {
        ESP_LOGE(TAG, "SDIO mode mismatch: slave is in streaming mode, "
                      "but host is in packet mode.  Aborting.");
        assert(0);
    }
    if (!slave_streaming && host_streaming) {
        ESP_LOGW(TAG, "SDIO mode asymmetric: slave packet, host streaming "
                      "— continuing (host will drain smaller frames).");
    }
    return 0;
}
#endif

/* Diagnose the EH_PRIV_RPC_VERSION TLV (0x1A) value from the CP init event.
 *
 * Updates s_peer_advertised_rpc_version so eh_host_mcu_transport_send_caps
 * knows whether it is safe to echo 0x1A back (older CP parsers may not
 * tolerate unknown tags from the host).
 *
 * Never aborts.  The host MUST stay up so that the OTA channel remains
 * available even on version mismatch — re-flashing the CP is the only
 * recovery, and the host must be reachable for that to happen.
 */
static void eh_host_mcu_transport_diagnose_peer_rpc_version(uint8_t peer_rpc_version)
{
    if (peer_rpc_version == 0) {
        s_peer_advertised_rpc_version = false;
        ESP_LOGI(TAG, "Default RPC to V%u", (unsigned)EH_HOST_RPC_WIRE_VERSION);
        return;
    }

    s_peer_advertised_rpc_version = true;

    if (peer_rpc_version == EH_HOST_RPC_WIRE_VERSION) {
        ESP_LOGI(TAG, "RPC version negotiated: V%u", (unsigned)peer_rpc_version);
        return;
    }

    /* Version mismatch — do NOT abort.  Keep transport up so the OTA channel
     * can rescue the CP.  Wire-level RPC commands may fail organically; the
     * recovery procedure below restores compatibility. */
    ESP_LOGE(TAG,
        "RPC version mismatch: peer=0x%02x, ours=0x%02x.",
        (unsigned)peer_rpc_version, (unsigned)EH_HOST_RPC_WIRE_VERSION);
    ESP_LOGE(TAG,
        "  RPC commands will likely fail.  Recovery procedure:");
    ESP_LOGE(TAG,
        "    1. From this source tree, build the CP firmware:");
    ESP_LOGE(TAG,
        "         cd examples/system/get_cp_fw_version/cp");
    ESP_LOGE(TAG,
        "         idf.py set-target <cp_chip> && idf.py build");
    ESP_LOGE(TAG,
        "       Save build/eh_cp_*.bin outside the repo.");
    ESP_LOGE(TAG,
        "    2. Identify the commit your CP firmware was built from");
    ESP_LOGE(TAG,
        "       (CP boot log / release notes).  If you cannot identify");
    ESP_LOGE(TAG,
        "       a mapped commit for your CP firmware, contact your CP");
    ESP_LOGE(TAG,
        "       firmware provider.  git checkout <that_commit>.");
    ESP_LOGE(TAG,
        "    3. Build the host driver + app from that older commit, load");
    ESP_LOGE(TAG,
        "       it, and OTA the CP with the binary saved in step 1.");
    ESP_LOGE(TAG,
        "    4. git checkout <this_branch>; rebuild the host; reconnect.");
}

int eh_host_mcu_transport_process_init_event(const uint8_t *evt_buf, uint16_t len)
{
    if (!evt_buf) return -1;
    if (len > 64) {
        ESP_LOGE(TAG, "init event too long: %u — possible bus mode mismatch",
                 (unsigned)len);
        return -1;
    }

    const uint8_t *pos    = evt_buf;
    uint16_t       remain = len;
#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
    int            saw_sdio_mode_tlv = 0;
#endif
    uint8_t        peer_rpc_version  = 0;

    while (remain >= 2) {
        uint8_t tag = pos[0], tlen = pos[1];
        if ((uint16_t)tlen + 2u > remain) {
            ESP_LOGE(TAG, "TLV truncated tag=0x%02x", tag);
            return -1;
        }
        const uint8_t *val = pos + 2;

        switch (tag) {
        case EH_PRIV_RPC_VERSION:
            if (tlen >= 1) peer_rpc_version = val[0];
            break;
        case EH_PRIV_CAPABILITY:
            if (tlen >= 1) {
                s_capabilities = val[0];
                ESP_LOGI(TAG, "capabilities: 0x%02x", (unsigned)s_capabilities);
                print_capabilities(s_capabilities);
                eh_frame_set_checksum_enabled(s_capabilities & ESP_CHECKSUM_ENABLED);
            }
            break;
        case EH_PRIV_CAP_EXT:
            if (tlen >= 4) {
                s_ext_caps = le32(val);
                ESP_LOGI(TAG, "extended capabilities: 0x%08x", (unsigned)s_ext_caps);
                print_ext_capabilities(s_ext_caps);
            }
            break;
        case EH_PRIV_FIRMWARE_CHIP_ID:
            if (tlen >= 1) {
                s_chip_id = val[0];
                ESP_LOGI(TAG, "slave chip id: 0x%02x (%s)",
                         (unsigned)s_chip_id, chip_id_name(s_chip_id));
            }
            break;
        case EH_PRIV_FIRMWARE_VERSION:
            if (tlen >= 4) {
                s_fw_version = le32(val);
                ESP_LOGD(TAG, "coprocessor firmware: 0x%08x", (unsigned)s_fw_version);
            }
            break;
        case EH_PRIV_TRANS_SDIO_MODE:
            if (tlen >= 1) {
                s_slave_sdio_streaming_mode = val[0];
#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
                saw_sdio_mode_tlv = 1;
#endif
            }
            break;
        case EH_PRIV_SDIO_BUF_CONFIG: {
            /* Malformed = real fault (we own the emitter) — fail init, no
             * silent fallback. Absent stays graceful (older CP). */
            struct eh_priv_sdio_buf_config cfg;
            if (eh_sdio_cfg_parse(val, tlen, &cfg) != EH_SDIO_CFG_OK) {
                ESP_LOGE(TAG, "malformed SDIO buf-config TLV (len=%u) — aborting init",
                         (unsigned)tlen);
                return -1;
            }
            s_sdio_buf_cfg       = cfg;
            s_sdio_buf_cfg_valid = true;
            ESP_LOGI(TAG, "SDIO buf-config: e2h mode=%u bufsz=%uB, h2e mode=%u bufsz=%uB",
                     (unsigned)cfg.e2h_mode,
                     (unsigned)cfg.e2h_bufsz_512B * EH_SDIO_CFG_BUF_BLOCK,
                     (unsigned)cfg.h2e_mode,
                     (unsigned)cfg.h2e_bufsz_512B * EH_SDIO_CFG_BUF_BLOCK);
            break;
        }
        case EH_PRIV_RX_Q_SIZE:
            if (tlen >= 1) ESP_LOGD(TAG, "slave rx queue size: %u", (unsigned)val[0]);
            break;
        case EH_PRIV_TX_Q_SIZE:
            if (tlen >= 1) ESP_LOGD(TAG, "slave tx queue size: %u", (unsigned)val[0]);
            break;
        case EH_PRIV_TEST_RAW_TP:
            if (tlen >= 1) {
                eh_host_raw_tp_process_test_capabilities(val[0]);
            }
            break;
        case EH_PRIV_FEAT_CAPS:
            /* Feature layer reads its own copy. */
            break;
        default:
            ESP_LOGW(TAG, "unknown PRIV TLV 0x%02x", tag);
            break;
        }
        pos    += 2 + tlen;
        remain -= 2 + tlen;
    }

    eh_host_mcu_transport_diagnose_peer_rpc_version(peer_rpc_version);

    eh_host_mcu_transport_verify_fw_compat(s_fw_version);

    if (!chip_id_is_known(s_chip_id)) {
        ESP_LOGE(TAG, "unrecognised slave chip id 0x%02x — refusing",
                 (unsigned)s_chip_id);
        s_chip_id = EH_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
        return -1;
    }

    if (eh_host_mcu_transport_verify_chip_match(s_chip_id) != 0) {
        return -1;
    }

    /* Older CP firmware lacks the SDIO_MODE TLV; warn but don't abort. */
#if CONFIG_ESP_HOSTED_HOST_TRANSPORT_BUS_SDIO
    if (eh_host_mcu_transport_sdio_sw_aggr_negotiated()) {
        /* Fail-fast on unallocatable advertised sizes (decision: assert, no
         * silent degrade) — probe the larger direction once. */
        uint32_t e2h = (uint32_t)s_sdio_buf_cfg.e2h_bufsz_512B * EH_SDIO_CFG_BUF_BLOCK;
        uint32_t h2e = (uint32_t)s_sdio_buf_cfg.h2e_bufsz_512B * EH_SDIO_CFG_BUF_BLOCK;
        uint32_t probe_sz = e2h > h2e ? e2h : h2e;
        void *probe = eh_host_port_dma_alloc_aligned(probe_sz, HOSTED_MEM_ALIGNMENT_64);
        if (!probe) {
            ESP_LOGE(TAG, "SDIO SW_AGGR advertised %u B buffers — host cannot "
                          "allocate; failing init (no silent degrade)",
                     (unsigned)probe_sz);
            assert(probe);
            return -1;
        }
        eh_host_port_dma_free(probe);
        ESP_LOGI(TAG, "SDIO SW_AGGR negotiated (e2h=%uB h2e=%uB)", (unsigned)e2h, (unsigned)h2e);
    } else {
		ESP_LOGI(TAG, "CP without SDIO SW_AGGR; compatible streaming mode enabled");
    }
    if (saw_sdio_mode_tlv) {
        sdio_mode_check(s_slave_sdio_streaming_mode);
    } else {
		ESP_LOGI(TAG, "Missing SDIO_MODE TLV; Continue anyway.");
    }
#endif

    ESP_LOGI(TAG, "chip=0x%02x (%s) fw=0x%08x caps=0x%02x ext=0x%08x",
             (unsigned)s_chip_id, chip_id_name(s_chip_id),
             (unsigned)s_fw_version,
             (unsigned)s_capabilities, (unsigned)s_ext_caps);
    return 0;
}

static uint8_t expected_chip_id(void)
{
#if EH_HOST_CP_TARGET_ESP32
    return EH_PRIV_FIRMWARE_CHIP_ESP32;
#elif EH_HOST_CP_TARGET_ESP32S2
    return EH_PRIV_FIRMWARE_CHIP_ESP32S2;
#elif EH_HOST_CP_TARGET_ESP32S3
    return EH_PRIV_FIRMWARE_CHIP_ESP32S3;
#elif EH_HOST_CP_TARGET_ESP32C2
    return EH_PRIV_FIRMWARE_CHIP_ESP32C2;
#elif EH_HOST_CP_TARGET_ESP32C3
    return EH_PRIV_FIRMWARE_CHIP_ESP32C3;
#elif EH_HOST_CP_TARGET_ESP32C5
    return EH_PRIV_FIRMWARE_CHIP_ESP32C5;
#elif EH_HOST_CP_TARGET_ESP32C6
    return EH_PRIV_FIRMWARE_CHIP_ESP32C6;
#else
    return EH_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
#endif
}

int eh_host_mcu_transport_verify_chip_match(uint8_t slave_chip_id)
{
    uint8_t expected = expected_chip_id();
    if (expected == EH_PRIV_FIRMWARE_CHIP_UNRECOGNIZED) return 0;
    if (slave_chip_id == expected) return 0;
    ESP_LOGE(TAG, "chip mismatch: expect=0x%02x got=0x%02x",
             (unsigned)expected, (unsigned)slave_chip_id);
    return -1;
}

int eh_host_mcu_transport_verify_fw_compat(uint32_t slave_fw_version)
{
    uint32_t host_fw = EH_VERSION_VAL(PROJECT_VERSION_MAJOR_1,
                                      PROJECT_VERSION_MINOR_1,
                                      PROJECT_VERSION_PATCH_1);
    uint32_t cp_fw = slave_fw_version;

    /* Both versions, once. Verdict lines below carry only severity + action. */
    ESP_LOGI(TAG, "esp-hosted fw versions: host=" EH_VERSION_PRINTF_FMT " coprocessor="
                  EH_VERSION_PRINTF_FMT "%s",
             EH_VERSION_PRINTF_ARGS(host_fw), EH_VERSION_PRINTF_ARGS(cp_fw),
             host_fw == cp_fw ? " (match)" : "");

    if (host_fw == cp_fw)
        return 0;

    /* Full-version compare is monotonic → tells us who is behind. */
    const char *rec = host_fw > cp_fw ? "OTA coprocessor from host"
                                      : "update host firmware";
    int dir = host_fw > cp_fw ? 1 : -1;

    /* Severity by most-significant differing field. */
    if (EH_VERSION_MAJOR(host_fw) != EH_VERSION_MAJOR(cp_fw)) {
        ESP_LOGE(TAG, "major version mismatch — %s", rec);
        return dir;
    }
    if (EH_VERSION_MINOR(host_fw) != EH_VERSION_MINOR(cp_fw)) {
        ESP_LOGW(TAG, "minor version mismatch — %s", rec);
        return dir;
    }
    ESP_LOGI(TAG, "patch version differs (compatible)");
    return 0;
}
