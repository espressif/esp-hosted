/* SPDX-License-Identifier: Apache-2.0 */
/* MCU byte-stream channel adapter. No envelope policy, no framing.
 * Transport invokes our RX thunk; we forward to base RPC. */

#include "eh_host_mcu_vserial.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "eh_host_port.h"

#include "eh_host_mcu_transport.h"
#include "eh_host_mcu_transport_channels.h"
#include "eh_common_header.h"
#include "eh_common_interface.h"

#include <stdlib.h>
#include <string.h>

/* protocomm-pserial TLV envelope:
 *   [0x01][ep_len:2 LE]["RPCRsp"][0x02][data_len:2 LE][protobuf] */
#define EH_VSM_TLV_T_EPNAME   0x01
#define EH_VSM_TLV_T_DATA     0x02
#define EH_VSM_RPC_EP_RSP     "RPCRsp"

typedef void (*eh_vsm_transport_rx_cb_t)(const uint8_t *buf,
                                         size_t len,
                                         void *ctx);

#define EH_VSM_LOG(fmt, ...) \
    ESP_LOGI("eh_vserial_mcu", fmt, ##__VA_ARGS__)

typedef int (*eh_vsm_rx_cb_t)(const uint8_t *, size_t, void *);

typedef struct {
    eh_host_port_mutex_t *cb_mu;
    eh_vsm_rx_cb_t rx_cb;
    void *rx_cb_ctx;

    /* Serialises start/stop. */
    eh_host_port_mutex_t *lifecycle_mu;
    int started;

    eh_host_rpc_io_ops_t ops;
} eh_vsm_state_t;

static eh_vsm_state_t *g_state;

/* Strip protocomm-pserial TLV envelope: returns 0 on success, -1 on bad shape. */
static int eh_vsm_strip_tlv(const uint8_t *buf, size_t len,
                            const uint8_t **out_data, uint16_t *out_len)
{
    size_t p = 0;

    if (len < 3) return -1;
    if (buf[p++] != EH_VSM_TLV_T_EPNAME) return -1;
    uint16_t ep_len = (uint16_t)(buf[p] | (buf[p + 1] << 8));
    p += 2;
    if (ep_len + p > len) return -1;
    p += ep_len;  /* ep_name not validated — registry routes by msg_id. */

    if (p + 3 > len) return -1;
    if (buf[p++] != EH_VSM_TLV_T_DATA) return -1;
    uint16_t data_len = (uint16_t)(buf[p] | (buf[p + 1] << 8));
    p += 2;
    if (p + data_len > len) return -1;

    *out_data = &buf[p];
    *out_len  = data_len;
    return 0;
}

static void eh_vsm_transport_rx_thunk(const uint8_t *buf,
                                      size_t len,
                                      void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    eh_vsm_rx_cb_t cb;
    void *cb_ctx;

    if (!s || !buf || len == 0) return;

    /* Strip TLV envelope; if shape doesn't match (legacy cp), fall through. */
    const uint8_t *payload = buf;
    uint16_t payload_len = (len > UINT16_MAX) ? UINT16_MAX : (uint16_t)len;
    {
        const uint8_t *unwrapped = NULL;
        uint16_t unwrapped_len = 0;
        if (eh_vsm_strip_tlv(buf, len, &unwrapped, &unwrapped_len) == 0) {
            payload     = unwrapped;
            payload_len = unwrapped_len;
        }
    }

    eh_host_port_mutex_lock(s->cb_mu);
    cb = s->rx_cb;
    cb_ctx = s->rx_cb_ctx;
    eh_host_port_mutex_unlock(s->cb_mu);

    if (cb) cb(payload, payload_len, cb_ctx);
    /* No registered cb => bytes are dropped. */
}

static int eh_vsm_send_tlv_frames(const uint8_t *buf, size_t len,
                                  uint16_t max_payload_len)
{
    if (!buf || len == 0) return -EINVAL;
    if (len > UINT16_MAX) return -EINVAL;

    /* Wrap protobuf payload in TLV envelope expected by cp. */
    const uint16_t ep_len   = (uint16_t)(sizeof(EH_VSM_RPC_EP_RSP) - 1);
    const uint16_t data_len = (uint16_t)len;
    const size_t   tlv_total = 1u + 2u + ep_len + 1u + 2u + data_len;
    if (tlv_total > UINT16_MAX) return -EINVAL;

    uint8_t *tlv = (uint8_t *)malloc(tlv_total);
    if (!tlv) return -ENOMEM;

    size_t p = 0;
    tlv[p++] = EH_VSM_TLV_T_EPNAME;
    tlv[p++] = (uint8_t)(ep_len & 0xFF);
    tlv[p++] = (uint8_t)((ep_len >> 8) & 0xFF);
    memcpy(&tlv[p], EH_VSM_RPC_EP_RSP, ep_len);
    p += ep_len;

    tlv[p++] = EH_VSM_TLV_T_DATA;
    tlv[p++] = (uint8_t)(data_len & 0xFF);
    tlv[p++] = (uint8_t)((data_len >> 8) & 0xFF);
    memcpy(&tlv[p], buf, data_len);
    p += data_len;

    int rc = 0;
    if (!max_payload_len || p <= max_payload_len) {
        rc = eh_host_transport_tx(ESP_SERIAL_IF, 0, tlv, (uint16_t)p, 0);
        free(tlv);
        return (rc == 0) ? (int)len : -1;
    }

    size_t offset = 0;
    while (offset < p) {
        size_t frag_len = p - offset;
        if (frag_len > max_payload_len) frag_len = max_payload_len;
        uint8_t flags = ((offset + frag_len) < p) ? MORE_FRAGMENT : 0;
        rc = eh_host_transport_tx(ESP_SERIAL_IF, 0,
                                  tlv + offset, (uint16_t)frag_len, flags);
        if (rc != 0) {
            free(tlv);
            return -1;
        }
        offset += frag_len;
    }

    free(tlv);
    return (int)len;
}

static int eh_vsm_op_tx_bytes(const uint8_t *buf, size_t len, void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    if (!s || !buf || len == 0) return -EINVAL;
    return eh_vsm_send_tlv_frames(buf, len, 0);
}

static int eh_vsm_op_tx_bytes_chunked(const uint8_t *buf, size_t len,
                                      uint16_t max_payload_len, void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    if (!s || !buf || len == 0) return -EINVAL;
    return eh_vsm_send_tlv_frames(buf, len, max_payload_len);
}

static int eh_vsm_op_register_rx_cb(int (*cb)(const uint8_t*, size_t, void*),
                                    void *cb_ctx, void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    if (!s) return -EINVAL;
    eh_host_port_mutex_lock(s->cb_mu);
    s->rx_cb = cb;
    s->rx_cb_ctx = cb_ctx;
    eh_host_port_mutex_unlock(s->cb_mu);
    return 0;
}

static int eh_vsm_op_start(void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    int ret = 0;
    if (!s) return -EINVAL;

    eh_host_port_mutex_lock(s->lifecycle_mu);
    if (s->started) {
        eh_host_port_mutex_unlock(s->lifecycle_mu);
        return 0;
    }

    ret = eh_host_mcu_transport_init();
    if (ret != 0) {
        EH_VSM_LOG("transport_mcu_init failed: %d", ret);
        goto out;
    }

    ret = eh_host_mcu_transport_register_rx(eh_vsm_transport_rx_thunk, s);
    if (ret != 0) {
        EH_VSM_LOG("transport_mcu_register_rx failed: %d", ret);
        eh_host_mcu_transport_deinit();
        goto out;
    }

    s->started = 1;

out:
    eh_host_port_mutex_unlock(s->lifecycle_mu);
    return ret;
}

static int eh_vsm_op_stop(void *ctx)
{
    eh_vsm_state_t *s = (eh_vsm_state_t *)ctx;
    if (!s) return -EINVAL;

    eh_host_port_mutex_lock(s->lifecycle_mu);
    if (!s->started) {
        eh_host_port_mutex_unlock(s->lifecycle_mu);
        return 0;
    }

    /* Clear cb before teardown to avoid racing upcalls. */
    eh_host_mcu_transport_register_rx(NULL, NULL);
    eh_host_mcu_transport_deinit();

    s->started = 0;
    eh_host_port_mutex_unlock(s->lifecycle_mu);
    return 0;
}

int eh_host_mcu_vserial_init(const eh_host_mcu_vserial_cfg_t *cfg)
{
    (void)cfg;

    if (g_state) return 0;

    eh_vsm_state_t *s = (eh_vsm_state_t *)calloc(1, sizeof(*s));
    if (!s) return -ENOMEM;

    s->cb_mu        = eh_host_port_mutex_create();
    s->lifecycle_mu = eh_host_port_mutex_create();
    if (!s->cb_mu || !s->lifecycle_mu) {
        if (s->cb_mu)        eh_host_port_mutex_destroy(s->cb_mu);
        if (s->lifecycle_mu) eh_host_port_mutex_destroy(s->lifecycle_mu);
        free(s);
        return -ENOMEM;
    }

    s->ops.tx_bytes       = eh_vsm_op_tx_bytes;
    s->ops.tx_bytes_chunked = eh_vsm_op_tx_bytes_chunked;
    s->ops.max_payload_len = EH_HOST_RPC_SERIAL_MAX_PAYLOAD_BYTES;
    s->ops.register_rx_cb = eh_vsm_op_register_rx_cb;
    s->ops.start          = eh_vsm_op_start;
    s->ops.stop           = eh_vsm_op_stop;
    s->ops.ctx            = s;

    g_state = s;
    return 0;
}

int eh_host_mcu_vserial_deinit(void)
{
    eh_vsm_state_t *s = g_state;
    if (!s) return 0;

    eh_vsm_op_stop(s);

    eh_host_port_mutex_lock(s->cb_mu);
    s->rx_cb = NULL;
    s->rx_cb_ctx = NULL;
    eh_host_port_mutex_unlock(s->cb_mu);

    eh_host_port_mutex_destroy(s->cb_mu);
    eh_host_port_mutex_destroy(s->lifecycle_mu);
    free(s);
    g_state = NULL;
    return 0;
}

const eh_host_rpc_io_ops_t *eh_host_mcu_vserial_get_ops(void)
{
    return g_state ? &g_state->ops : NULL;
}
