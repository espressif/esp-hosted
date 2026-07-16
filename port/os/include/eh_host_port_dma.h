/* SPDX-License-Identifier: Apache-2.0 */
/*
 * DMA-capable allocation.  On ESP: MALLOC_CAP_DMA-backed.  On Linux:
 * aligned_alloc.  `_alloc_aligned` lets the caller request a stricter
 * boundary (e.g. 64 B for ESP32-P4 GDMA).  NOT ISR-safe.
 */

#ifndef EH_HOST_PORT_DMA_H_
#define EH_HOST_PORT_DMA_H_

#include "eh_host_port_config.h"
#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_HOST_PORT_HAS_DMA

/* Allocate at least `n` bytes of DMA-capable memory.  Word-aligned
 * (minimum 4-byte boundary).  Returns NULL on failure. */
void *eh_host_port_dma_alloc(size_t n);

/* As above but caller controls alignment.  `alignment` must be a
 * power of two and a multiple of sizeof(void *).  Returns NULL on
 * failure or invalid alignment. */
void *eh_host_port_dma_alloc_aligned(size_t n, size_t alignment);

/* Free a buffer returned by eh_host_port_dma_alloc{,_aligned}.  Passing
 * NULL is a no-op, matching libc free(). */
void  eh_host_port_dma_free(void *p);

#endif /* EH_HOST_PORT_HAS_DMA */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_DMA_H_ */
