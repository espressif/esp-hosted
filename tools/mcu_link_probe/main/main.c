/* SPDX-License-Identifier: Apache-2.0 */
/*
 * system/mcu_host — MCU-host link-probe example.
 *
 * Minimal IDF app that exercises the full host stack on an ESP target:
 *   esp_idf port-type → feature layer → base RPC → MCU-V1 extension →
 *   transport dispatcher → SPI bus backend.
 *
 * Purpose: prove the stack compiles + links on IDF when
 * CONFIG_ESP_HOSTED_HOST=y.  No CP is expected on the other end; the
 * first transport access will fail and the example logs + exits the
 * init chain, then sleeps forever.  Hardware bring-up uses this same
 * app wired to a real CP.
 *
 * The example stops short of calling eh_host_feat_rpc_start() because
 * that would spawn RX/TX worker tasks waiting on the transport; on a
 * link probe with no CP that just spins heap + cycles.  The init chain
 * through feature-level init is sufficient to prove linkage.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eh_port.h"
#include "eh_host_feat_rpc.h"
#include "eh_host_auto_init.h"
#include "eh_host_feat_system.h"

static const char *TAG = "mcu_host";

/* TX placeholder: sink that drops frames + logs so the base's io_ops
 * contract is satisfied without a real transport.  Replace with the
 * actual transport-dispatcher tx when wiring to hardware. */
static int tx_sink(const uint8_t *buf, size_t len, void *ctx)
{
    (void)buf; (void)ctx;
    ESP_LOGD(TAG, "tx_sink dropped %u bytes", (unsigned)len);
    return (int)len;  /* pretend accepted */
}

static int rx_register_sink(
    int (*cb)(const uint8_t *, size_t, void *),
    void *cb_ctx, void *ctx)
{
    (void)cb; (void)cb_ctx; (void)ctx;
    ESP_LOGD(TAG, "rx_register_sink installed (no-op)");
    return 0;
}

static int start_sink(void *ctx) { (void)ctx; return 0; }
static int stop_sink(void *ctx)  { (void)ctx; return 0; }

void app_main(void)
{
    ESP_LOGI(TAG, "eh_host MCU link probe starting");
    ESP_LOGI(TAG, "port-type auto-init features: %u",
             eh_host_auto_init_count());

    /* Initialise the event bus so any future event producer can post. */
    esp_err_t ev = esp_event_loop_create_default();
    ESP_LOGI(TAG, "esp_event_loop_create_default rc=%d", (int)ev);

    /* Build a no-op I/O ops that satisfies the base's contract without
     * requiring a real transport slave on the other end. */
    static const eh_host_rpc_io_ops_t io_ops = {
        .tx_bytes        = tx_sink,
        .register_rx_cb  = rx_register_sink,
        .start           = start_sink,
        .stop            = stop_sink,
        .ctx             = NULL,
    };
    /* Pull the configured MCU V1 proto_ops — the ext defines this
     * symbol and the base wires up through it. */
    extern const eh_host_rpc_proto_ops_t *eh_host_feat_rpc_ext_v2_proto_ops(void);
    extern eh_host_id_ranges_t        eh_host_feat_rpc_ext_v2_id_ranges(void);

    eh_host_feat_rpc_cfg_t cfg = {
        .io_ops    = &io_ops,
        .proto_ops = eh_host_feat_rpc_ext_v2_proto_ops(),
        .id_ranges = eh_host_feat_rpc_ext_v2_id_ranges(),
    };

    int rc = eh_host_feat_rpc_init(&cfg);
    ESP_LOGI(TAG, "eh_host_feat_rpc_init rc=%d", rc);

    /* Kick the linker-section auto-init so registered features (system
     * in this build) wire their event thunks into the base. */
    rc = eh_host_auto_init_features();
    ESP_LOGI(TAG, "eh_host_auto_init_features rc=%d", rc);

    ESP_LOGI(TAG, "host stack linked + initialised OK; idling");

    /* Idle forever so flash+boot keep working; real apps would
     * eh_host_feat_rpc_start() and drive requests here. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
