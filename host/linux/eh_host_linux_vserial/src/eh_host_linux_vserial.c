/* SPDX-License-Identifier: Apache-2.0 */
/* Linux /dev/esps0 framed-serial adapter. Exact-length reads keep framing
 * synced across partial / coalesced reads. */

#define _POSIX_C_SOURCE 200809L

#include "eh_host_feat_rpc_io_ops.h"
#include "eh_host_linux_vserial.h"
#include <stdlib.h>
#include <string.h>
#include "esp_event.h"
#include "eh_host_event.h"
#include "eh_host_port_master_config.h"
#include "eh_host_port_power.h"
#include "eh_host_port_sync.h"
#include "eh_host_port_task.h"

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define EH_VSL_DEFAULT_DEV   "/dev/esps0"
#define EH_VSL_RX_INITIAL    1024u
#define EH_VSL_EPNAME_MAX    16u
#define EH_VSL_DATA_MAX      16384u

#define EH_VSL_TLV_T_EPNAME  0x01
#define EH_VSL_TLV_T_DATA    0x02
#define EH_VSL_EP_RPCRSP     "RPCRsp"
#define EH_VSL_EP_RPCEVT     "RPCEvt"

#define EH_VSL_LOG(fmt, ...) \
    fprintf(stderr, "eh_vserial_linux: " fmt "\n", ##__VA_ARGS__)

typedef int (*eh_vsl_rx_cb_t)(const uint8_t *, size_t, void *);

typedef struct {
    char *device_path;
    size_t rx_buf_initial;
    size_t rx_buf_size;          /* current allocation; 0 when rx_buf == NULL */

    int fd;
    uint8_t *rx_buf;

    eh_host_port_task_t *rx_task;
    int rx_thread_running;
    atomic_int stop_requested;
    atomic_int rx_thread_exited; /* set by worker on exit, lets restart reap zombie */

    eh_host_port_mutex_t *cb_mu;
    eh_vsl_rx_cb_t rx_cb;
    void *rx_cb_ctx;

    eh_host_port_mutex_t *lifecycle_mu;

    eh_host_rpc_io_ops_t ops;
} eh_vsl_state_t;

static eh_vsl_state_t *g_state;

static int eh_vsl_set_nonblocking(int fd, int enable)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) return -1;
    flags = enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK);
    return (fcntl(fd, F_SETFL, flags) < 0) ? -1 : 0;
}

/* Drain pre-session leftovers so framing doesn't sync mid-message. */
static int eh_vsl_flush_stale(int fd)
{
    uint8_t scratch[128];
    if (eh_vsl_set_nonblocking(fd, 1) < 0) return -1;
    for (;;) {
        ssize_t n = read(fd, scratch, sizeof(scratch));
        if (n > 0) continue;
        if (n == 0) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        if (errno == EINTR) continue;
        eh_vsl_set_nonblocking(fd, 0);
        return -1;
    }
    return eh_vsl_set_nonblocking(fd, 0);
}

static int read_exact(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    size_t off = 0;
    while (off < n) {
        ssize_t r = read(fd, p + off, n - off);
        if (r > 0) { off += (size_t)r; continue; }
        if (r == 0)         return -EPIPE;
        if (errno == EINTR) {
            if (g_state && atomic_load(&g_state->stop_requested)) return -EINTR;
            continue;
        }
        return (errno > 0) ? -errno : -EIO;
    }
    return 0;
}

static int read_tlv_hdr(int fd, uint8_t expect_type, uint16_t *out_len)
{
    uint8_t h[3];
    int rc = read_exact(fd, h, sizeof(h));
    if (rc < 0) return rc;
    if (h[0] != expect_type) {
        EH_VSL_LOG("framing: TLV type 0x%02x, expected 0x%02x", h[0], expect_type);
        return -EBADMSG;
    }
    *out_len = (uint16_t)(h[1] | ((uint16_t)h[2] << 8));
    return 0;
}

static int ep_is_supported(const uint8_t *ep, uint16_t ep_len)
{
    if (ep_len != 6) return 0;
    return (memcmp(ep, EH_VSL_EP_RPCRSP, 6) == 0) ||
           (memcmp(ep, EH_VSL_EP_RPCEVT, 6) == 0);
}

static int eh_vsl_ensure_buf(eh_vsl_state_t *s, size_t need)
{
    if (need <= s->rx_buf_size) return 0;
    uint8_t *nb = (uint8_t *)malloc(need);
    if (!nb) {
        EH_VSL_LOG("rx_buf grow to %zu: out of memory", need);
        return -ENOMEM;
    }
    free(s->rx_buf);
    s->rx_buf = nb;
    s->rx_buf_size = need;
    return 0;
}

static void eh_vsl_dispatch(eh_vsl_state_t *s, const uint8_t *buf, uint16_t len)
{
    eh_vsl_rx_cb_t cb;
    void *cb_ctx;
    eh_host_port_mutex_lock(s->cb_mu);
    cb = s->rx_cb;
    cb_ctx = s->rx_cb_ctx;
    eh_host_port_mutex_unlock(s->cb_mu);
    if (cb) cb(buf, len, cb_ctx);
}

static int eh_vsl_rx_one(eh_vsl_state_t *s)
{
    uint8_t ep_name[EH_VSL_EPNAME_MAX];
    uint16_t ep_len = 0, data_len = 0;
    int rc;

    rc = read_tlv_hdr(s->fd, EH_VSL_TLV_T_EPNAME, &ep_len);
    if (rc < 0) return rc;
    if (ep_len == 0 || ep_len > sizeof(ep_name)) {
        EH_VSL_LOG("framing: ep_len=%u out of bounds", ep_len);
        return -EBADMSG;
    }
    rc = read_exact(s->fd, ep_name, ep_len);
    if (rc < 0) return rc;
    if (!ep_is_supported(ep_name, ep_len)) {
        EH_VSL_LOG("framing: unknown endpoint '%.*s'", (int)ep_len, (const char *)ep_name);
        return -EBADMSG;
    }

    rc = read_tlv_hdr(s->fd, EH_VSL_TLV_T_DATA, &data_len);
    if (rc < 0) return rc;
    if (data_len == 0 || data_len > EH_VSL_DATA_MAX) {
        EH_VSL_LOG("framing: data_len=%u out of bounds", data_len);
        return -EBADMSG;
    }

    rc = eh_vsl_ensure_buf(s, data_len);
    if (rc < 0) return rc;

    rc = read_exact(s->fd, s->rx_buf, data_len);
    if (rc < 0) return rc;

    eh_vsl_dispatch(s, s->rx_buf, data_len);
    return 0;
}

static void eh_vsl_rx_thread_fn(void *arg)
{
    eh_vsl_state_t *s = (eh_vsl_state_t *)arg;
    int fatal = 0;

    while (!atomic_load(&s->stop_requested)) {
        int rc = eh_vsl_rx_one(s);
        if (rc == 0) continue;
        if (atomic_load(&s->stop_requested)) break;
        EH_VSL_LOG("rx worker exiting: rc=%d", rc);
        fatal = 1;
        break;
    }
    atomic_store(&s->rx_thread_exited, 1);
    if (fatal) {
        esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
        eh_host_port_restart_host();
#endif
    }
}

/* Caller must hold lifecycle_mu. No-op for live or never-started workers. */
static void eh_vsl_reap_zombie(eh_vsl_state_t *s)
{
    if (!s->rx_thread_running) return;
    if (!atomic_load(&s->rx_thread_exited)) return;

    eh_host_port_task_join(s->rx_task);
    eh_host_port_task_destroy(s->rx_task);
    s->rx_task = NULL;
    s->rx_thread_running = 0;
    atomic_store(&s->rx_thread_exited, 0);
}

static int eh_vsl_op_tx_bytes(const uint8_t *buf, size_t len, void *ctx)
{
    eh_vsl_state_t *s = (eh_vsl_state_t *)ctx;
    uint16_t ep_len, data_len;
    size_t tlv_len, p, written;
    uint8_t *tlv;

    if (!s || !buf || len == 0) return -EINVAL;
    if (s->fd < 0) return -ENOTCONN;
    if (len > UINT16_MAX) return -EINVAL;

    ep_len = (uint16_t)(sizeof(EH_VSL_EP_RPCRSP) - 1);
    data_len = (uint16_t)len;
    tlv_len = 1u + 2u + ep_len + 1u + 2u + data_len;
    if (tlv_len > UINT16_MAX) return -EINVAL;

    tlv = (uint8_t *)malloc(tlv_len);
    if (!tlv) return -ENOMEM;

    p = 0;
    tlv[p++] = EH_VSL_TLV_T_EPNAME;
    tlv[p++] = (uint8_t)(ep_len & 0xFF);
    tlv[p++] = (uint8_t)((ep_len >> 8) & 0xFF);
    memcpy(&tlv[p], EH_VSL_EP_RPCRSP, ep_len);
    p += ep_len;
    tlv[p++] = EH_VSL_TLV_T_DATA;
    tlv[p++] = (uint8_t)(data_len & 0xFF);
    tlv[p++] = (uint8_t)((data_len >> 8) & 0xFF);
    memcpy(&tlv[p], buf, data_len);

    written = 0;
    while (written < tlv_len) {
        ssize_t n = write(s->fd, tlv + written, tlv_len - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            EH_VSL_LOG("write failed: %s", strerror(errno));
            free(tlv);
            return -errno;
        }
        if (n == 0) {
            /* kmod rejected (e.g. oversize); avoid tight-loop. */
            EH_VSL_LOG("write rejected by kmod at %zu/%zu (oversize?)",
                       written, tlv_len);
            free(tlv);
            return -EMSGSIZE;
        }
        written += (size_t)n;
    }
    free(tlv);
    return (int)len;
}

static int eh_vsl_op_tx_bytes_chunked(const uint8_t *buf, size_t len,
                                      uint16_t max_payload_len, void *ctx)
{
    (void)max_payload_len;
    return eh_vsl_op_tx_bytes(buf, len, ctx);
}

static int eh_vsl_op_register_rx_cb(int (*cb)(const uint8_t*, size_t, void*),
                                    void *cb_ctx, void *ctx)
{
    eh_vsl_state_t *s = (eh_vsl_state_t *)ctx;
    if (!s) return -EINVAL;
    eh_host_port_mutex_lock(s->cb_mu);
    s->rx_cb = cb;
    s->rx_cb_ctx = cb_ctx;
    eh_host_port_mutex_unlock(s->cb_mu);
    return 0;
}

static int eh_vsl_op_start(void *ctx)
{
    eh_vsl_state_t *s = (eh_vsl_state_t *)ctx;
    int ret = 0;
    if (!s) return -EINVAL;

    eh_host_port_mutex_lock(s->lifecycle_mu);
    eh_vsl_reap_zombie(s);
    if (s->rx_thread_running) {
        eh_host_port_mutex_unlock(s->lifecycle_mu);
        return 0;
    }

    if (s->fd < 0) {
        /* O_CLOEXEC: keep the transport fd out of DHCP-client/server children
         * we fork+exec (dhclient/dnsmasq/...) — otherwise they inherit it and
         * hold /dev/esps0 open past our exit, EBUSY-ing the next run. */
        s->fd = open(s->device_path, O_RDWR | O_CLOEXEC);
        if (s->fd < 0) {
            ret = -errno;
            EH_VSL_LOG("open(%s) failed: %s", s->device_path, strerror(errno));
            esp_event_post(EH_HOST_EVENT, (int32_t)EH_HOST_EVENT_TRANSPORT_FAILURE, NULL, 0, 0);
#if EH_HOST_TRANSPORT_RESTART_ON_FAILURE
            eh_host_port_restart_host();
#endif
            goto out;
        }
        if (eh_vsl_flush_stale(s->fd) < 0) {
            EH_VSL_LOG("stale flush failed, continuing");
        }
    }

    if (!s->rx_buf) {
        s->rx_buf = (uint8_t *)malloc(s->rx_buf_initial);
        if (!s->rx_buf) {
            ret = -ENOMEM;
            close(s->fd);
            s->fd = -1;
            goto out;
        }
        s->rx_buf_size = s->rx_buf_initial;
    }

    atomic_store(&s->stop_requested, 0);
    atomic_store(&s->rx_thread_exited, 0);
    eh_host_port_task_create_cfg_t tcfg = {
        .fn = eh_vsl_rx_thread_fn, .arg = s,
        .stack_bytes = 0, .priority = 0, .name = "eh_vsl_rx",
    };
    if (eh_host_port_task_create(&tcfg, &s->rx_task) != EH_HOST_PORT_OK) {
        ret = -ENOMEM;
        EH_VSL_LOG("rx task create failed");
        free(s->rx_buf);
        s->rx_buf = NULL;
        s->rx_buf_size = 0;
        close(s->fd);
        s->fd = -1;
        goto out;
    }
    s->rx_thread_running = 1;

out:
    eh_host_port_mutex_unlock(s->lifecycle_mu);
    return ret;
}

static int eh_vsl_op_stop(void *ctx)
{
    eh_vsl_state_t *s = (eh_vsl_state_t *)ctx;
    int fd_to_close = -1;
    eh_host_port_task_t *task_to_join = NULL;
    if (!s) return -EINVAL;

    eh_host_port_mutex_lock(s->lifecycle_mu);
    eh_vsl_reap_zombie(s);
    if (!s->rx_thread_running) {
        if (s->fd >= 0) { close(s->fd); s->fd = -1; }
        if (s->rx_buf) {
            free(s->rx_buf);
            s->rx_buf = NULL;
            s->rx_buf_size = 0;
        }
        eh_host_port_mutex_unlock(s->lifecycle_mu);
        return 0;
    }
    atomic_store(&s->stop_requested, 1);
    task_to_join = s->rx_task;
    eh_host_port_mutex_unlock(s->lifecycle_mu);

    eh_host_port_task_wake(task_to_join);
    eh_host_port_task_join(task_to_join);
    eh_host_port_task_destroy(task_to_join);

    eh_host_port_mutex_lock(s->lifecycle_mu);
    fd_to_close = s->fd;
    s->fd = -1;
    s->rx_task = NULL;
    s->rx_thread_running = 0;
    atomic_store(&s->rx_thread_exited, 0);
    if (s->rx_buf) {
        free(s->rx_buf);
        s->rx_buf = NULL;
        s->rx_buf_size = 0;
    }
    eh_host_port_mutex_unlock(s->lifecycle_mu);

    if (fd_to_close >= 0) close(fd_to_close);
    return 0;
}

int eh_host_linux_vserial_init(const eh_host_linux_vserial_cfg_t *cfg)
{
    if (g_state) return 0;

    eh_vsl_state_t *s = (eh_vsl_state_t *)calloc(1, sizeof(*s));
    if (!s) return -ENOMEM;

    /* Precedence: explicit cfg > EH_ESPS_DEV env (lets a test point the host at
     * a PTY without a /dev/esps0 symlink) > compiled-in default. */
    const char *env_dev = getenv("EH_ESPS_DEV");
    const char *path = (cfg && cfg->device_path) ? cfg->device_path
                     : (env_dev && env_dev[0])   ? env_dev
                                                 : EH_VSL_DEFAULT_DEV;
    size_t path_len = strlen(path) + 1;
    s->device_path = (char *)malloc(path_len);
    if (!s->device_path) {
        free(s);
        return -ENOMEM;
    }
    memcpy(s->device_path, path, path_len);
    s->rx_buf_initial = (cfg && cfg->rx_buf_size) ? cfg->rx_buf_size
                                                  : EH_VSL_RX_INITIAL;
    s->rx_buf_size = 0;
    s->fd = -1;
    atomic_init(&s->stop_requested, 0);
    atomic_init(&s->rx_thread_exited, 0);
    s->cb_mu = eh_host_port_mutex_create();
    s->lifecycle_mu = eh_host_port_mutex_create();
    if (!s->cb_mu || !s->lifecycle_mu) {
        if (s->cb_mu) eh_host_port_mutex_destroy(s->cb_mu);
        if (s->lifecycle_mu) eh_host_port_mutex_destroy(s->lifecycle_mu);
        free(s->device_path);
        free(s);
        return -ENOMEM;
    }

    s->ops.tx_bytes        = eh_vsl_op_tx_bytes;
    s->ops.tx_bytes_chunked = eh_vsl_op_tx_bytes_chunked;
    s->ops.max_payload_len = EH_HOST_RPC_SERIAL_MAX_PAYLOAD_BYTES;
    s->ops.register_rx_cb  = eh_vsl_op_register_rx_cb;
    s->ops.start           = eh_vsl_op_start;
    s->ops.stop            = eh_vsl_op_stop;
    s->ops.ctx             = s;

    g_state = s;
    return 0;
}

int eh_host_linux_vserial_deinit(void)
{
    eh_vsl_state_t *s = g_state;
    if (!s) return 0;

    eh_vsl_op_stop(s);

    if (s->cb_mu) eh_host_port_mutex_destroy(s->cb_mu);
    if (s->lifecycle_mu) eh_host_port_mutex_destroy(s->lifecycle_mu);
    free(s->device_path);
    free(s);
    g_state = NULL;
    return 0;
}

const eh_host_rpc_io_ops_t *eh_host_linux_vserial_get_ops(void)
{
    return g_state ? &g_state->ops : NULL;
}
