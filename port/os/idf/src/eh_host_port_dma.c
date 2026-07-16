/* SPDX-License-Identifier: Apache-2.0 */
/* IDF DMA-alloc via heap_caps_(aligned_)malloc + MALLOC_CAP_DMA. */

#include "eh_host_port_config.h"
#include "eh_host_port_dma.h"

#if EH_HOST_PORT_HAS_DMA

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"

void *eh_host_port_dma_alloc(size_t n)
{
    return heap_caps_malloc(n, MALLOC_CAP_DMA);
}

void *eh_host_port_dma_alloc_aligned(size_t n, size_t alignment)
{
    if (alignment == 0 || (alignment & (alignment - 1u)) != 0) return NULL;
    void *ptr = NULL;
#if CONFIG_EH_HOST_PORT_DMA_PREFER_SPIRAM
    /* P4-class: prefer DMA-capable SPIRAM to preserve internal RAM */
    ptr = heap_caps_aligned_alloc(alignment, n,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
#endif
    if (!ptr) {
        ptr = heap_caps_aligned_alloc(alignment, n,
                MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    }
    return ptr;
}

void eh_host_port_dma_free(void *p)
{
    heap_caps_free(p);
}
#endif /* ESP_PLATFORM */

#endif /* EH_HOST_PORT_HAS_DMA */
