/* SPDX-License-Identifier: Apache-2.0 */
/* Linux-user DMA-alloc: aligned_alloc, no real DMA. */

#include "eh_host_port_config.h"
#include "eh_host_port_dma.h"

#if EH_HOST_PORT_HAS_DMA

#include <stdlib.h>

static size_t round_up(size_t n, size_t a)
{
    return (n + a - 1u) & ~(a - 1u);
}

void *eh_host_port_dma_alloc(size_t n)
{
    return eh_host_port_dma_alloc_aligned(n, 4);
}

void *eh_host_port_dma_alloc_aligned(size_t n, size_t alignment)
{
    if (alignment == 0 || (alignment & (alignment - 1u)) != 0) return NULL;
    if (alignment < sizeof(void *)) alignment = sizeof(void *);
    /* aligned_alloc requires size to be a multiple of alignment. */
    size_t rounded = round_up(n, alignment);
    return aligned_alloc(alignment, rounded);
}

void eh_host_port_dma_free(void *p) { free(p); }

#endif /* EH_HOST_PORT_HAS_DMA */
