/* SPDX-License-Identifier: Apache-2.0 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "eh_host_port.h"
#include "eh_host_auto_init.h"

#define EH_AUTO_INIT_LOG(fmt, ...) \
    ESP_LOGI("eh_auto_init", fmt, ##__VA_ARGS__)

size_t eh_host_auto_init_count(void)
{
    return (size_t)(__stop_eh_host_feat_descs - __start_eh_host_feat_descs);
}

static uint8_t *g_inited_flags;
static size_t   g_inited_flags_n;

static int ensure_flags(size_t n)
{
    if (g_inited_flags && g_inited_flags_n == n) {
        return 0;
    }
    if (g_inited_flags) {
        /* Descriptor count cannot change at runtime; resize defensively. */
        free(g_inited_flags);
        g_inited_flags = NULL;
        g_inited_flags_n = 0;
    }
    g_inited_flags = (uint8_t *)calloc(n, sizeof(*g_inited_flags));
    if (!g_inited_flags) {
        return -1;
    }
    g_inited_flags_n = n;
    return 0;
}

static size_t desc_index(const eh_host_feat_desc_t *d)
{
    return (size_t)(d - __start_eh_host_feat_descs);
}

/* Stable insertion sort, ascending by priority; ties keep linker order. */
static void sort_by_priority(const eh_host_feat_desc_t **out, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        out[i] = &__start_eh_host_feat_descs[i];
    }
    for (size_t i = 1; i < n; ++i) {
        const eh_host_feat_desc_t *key = out[i];
        size_t j = i;
        while (j > 0 && out[j - 1]->priority > key->priority) {
            out[j] = out[j - 1];
            --j;
        }
        out[j] = key;
    }
}

int eh_host_auto_init_features(void)
{
    size_t n = eh_host_auto_init_count();
    if (n == 0) {
        EH_AUTO_INIT_LOG("auto_init: no registered features");
        return 0;
    }
    EH_AUTO_INIT_LOG("auto_init: found %u descriptor(s)", (unsigned)n);

    if (ensure_flags(n) != 0) {
        EH_AUTO_INIT_LOG("auto_init: calloc for inited_flags failed");
        return -1;
    }

    /* Stack-alloc sort buffer; falls back to heap if features exceed cap. */
    enum { STACK_CAP = 32 };
    const eh_host_feat_desc_t *stack_buf[STACK_CAP];
    const eh_host_feat_desc_t **sorted = stack_buf;
    const eh_host_feat_desc_t **heap_buf = NULL;
    if (n > STACK_CAP) {
        heap_buf = calloc(n, sizeof(*heap_buf));
        if (!heap_buf) {
            EH_AUTO_INIT_LOG("auto_init: calloc for sort buffer failed");
            return -1;
        }
        sorted = heap_buf;
    }
    sort_by_priority(sorted, n);

    int first_err = 0;
    for (size_t i = 0; i < n; ++i) {
        const eh_host_feat_desc_t *d = sorted[i];
        size_t di = desc_index(d);

        if (g_inited_flags[di]) {
            continue;
        }
        if (!d->init_fn) {
            EH_AUTO_INIT_LOG("auto_init: descriptor '%s' has NULL init_fn, skipping",
                       d->name ? d->name : "?");
            continue;
        }
        EH_AUTO_INIT_LOG("auto_init: initialising '%s' (priority %u)",
                   d->name ? d->name : "?", (unsigned)d->priority);
        int rc = d->init_fn();
        if (rc == 0) {
            g_inited_flags[di] = 1;
        } else if (first_err == 0) {
            /* One failure doesn't cascade; flag stays 0 so deinit skips it. */
            first_err = rc;
            EH_AUTO_INIT_LOG("auto_init: '%s' init_fn failed rc=%d (continuing)",
                       d->name ? d->name : "?", rc);
        }
    }

    if (heap_buf) {
        free(heap_buf);
    }
    return first_err;
}

int eh_host_auto_deinit_features(void)
{
    size_t n = eh_host_auto_init_count();
    if (n == 0) {
        return 0;
    }

    enum { STACK_CAP = 32 };
    const eh_host_feat_desc_t *stack_buf[STACK_CAP];
    const eh_host_feat_desc_t **sorted = stack_buf;
    const eh_host_feat_desc_t **heap_buf = NULL;
    if (n > STACK_CAP) {
        heap_buf = calloc(n, sizeof(*heap_buf));
        if (!heap_buf) {
            return -1;
        }
        sorted = heap_buf;
    }
    sort_by_priority(sorted, n);

    /* Reverse-order teardown; skip descriptors whose init never ran/failed. */
    for (size_t i = n; i > 0; --i) {
        const eh_host_feat_desc_t *d = sorted[i - 1];
        size_t di = desc_index(d);
        if (g_inited_flags && di < g_inited_flags_n &&
            g_inited_flags[di] && d->deinit_fn) {
            EH_AUTO_INIT_LOG("auto_deinit: deinitialising '%s' (priority %u)",
                       d->name ? d->name : "?", (unsigned)d->priority);
            (void)d->deinit_fn();
        }
        if (g_inited_flags && di < g_inited_flags_n) {
            g_inited_flags[di] = 0;
        }
    }

    if (heap_buf) {
        free(heap_buf);
    }
    if (g_inited_flags) {
        free(g_inited_flags);
        g_inited_flags = NULL;
        g_inited_flags_n = 0;
    }
    return 0;
}
