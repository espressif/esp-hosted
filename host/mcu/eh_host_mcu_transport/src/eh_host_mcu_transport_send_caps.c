/* SPDX-License-Identifier: Apache-2.0 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "eh_tlv_tags.h"
#include "eh_host_mcu_transport_init_event.h"
#include "eh_host_mcu_transport_send_caps.h"

#define EH_HOST_PRIV_EVENT_INIT  0x22u

/* Each TLV is 3 bytes (tag + len + 1-byte value). */
#define EH_HOST_TLV_SIZE         3u

/* Five host->slave caps TLVs (0x44..0x48) are always emitted. */
#define EH_HOST_BASE_TLV_COUNT   5u

/* RPC_VERSION (0x1A) is emitted only when the CP advertised it first;
 * older CP parsers don't tolerate unknown tags in the reverse direction. */
#define EH_HOST_OPT_TLV_COUNT    1u

#define EH_HOST_BASE_PKT_SIZE \
    (2u /* evt hdr */ + EH_HOST_BASE_TLV_COUNT * EH_HOST_TLV_SIZE)

#define EH_HOST_MAX_PKT_SIZE \
    (EH_HOST_BASE_PKT_SIZE + EH_HOST_OPT_TLV_COUNT * EH_HOST_TLV_SIZE)

int eh_host_transport_build_host_caps_pkt(uint8_t *out, size_t out_size,
                                          uint8_t host_cap,
                                          uint8_t firmware_chip_id,
                                          uint8_t raw_tp_direction,
                                          uint8_t low_threshold,
                                          uint8_t high_threshold)
{
    if (!out || out_size < EH_HOST_MAX_PKT_SIZE) return -1;

    uint8_t *p = out;
    uint8_t  evt_len = 0;

    *p++ = EH_HOST_PRIV_EVENT_INIT;
    uint8_t *evt_len_ptr = p++;

    *p++ = EH_HOST_PRIV_HOST_CAPABILITIES;         evt_len++;
    *p++ = 1;                                       evt_len++;
    *p++ = host_cap;                                evt_len++;

    *p++ = EH_HOST_PRIV_RCVD_ESP_FIRMWARE_CHIP_ID; evt_len++;
    *p++ = 1;                                       evt_len++;
    *p++ = firmware_chip_id;                        evt_len++;

    *p++ = EH_HOST_PRIV_SLV_CONFIG_TEST_RAW_TP;    evt_len++;
    *p++ = 1;                                       evt_len++;
    *p++ = raw_tp_direction;                        evt_len++;

    *p++ = EH_HOST_PRIV_SLV_CONFIG_THROTTLE_HIGH;  evt_len++;
    *p++ = 1;                                       evt_len++;
    *p++ = high_threshold;                          evt_len++;

    *p++ = EH_HOST_PRIV_SLV_CONFIG_THROTTLE_LOW;   evt_len++;
    *p++ = 1;                                       evt_len++;
    *p++ = low_threshold;                           evt_len++;

    /* Echo RPC_VERSION (0x1A) only when the CP advertised it on the
     * inbound init event.  Older upstream CP firmware (esp-hosted /
     * esp-hosted-mcu pre-rc1) does not parse this tag in the host->slave
     * direction and may not silently skip it. */
    if (eh_host_mcu_transport_peer_advertised_rpc_version()) {
        *p++ = EH_PRIV_RPC_VERSION;                evt_len++;
        *p++ = 1;                                   evt_len++;
        *p++ = ESP_HOSTED_RPC_VERSION_V2;           evt_len++;
    }

    *evt_len_ptr = evt_len;
    return (int)(p - out);   /* total bytes written, including evt hdr */
}
