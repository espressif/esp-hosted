// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD

#ifndef __EH_TRANSPORT_MEMPOOL_H__
#define __EH_TRANSPORT_MEMPOOL_H__

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#ifdef CONFIG_ESP_CACHE_MALLOC
#include "eh_mempool_ll.h"
struct hosted_mempool {
	struct eh_mp_mempool *pool;
	uint8_t *heap;
	uint8_t static_heap;
	size_t num_blocks;
	size_t block_size;
};
#endif

#define MEM_DUMP(s) \
    printf("%s free:%lu min-free:%lu lfb-dma:%u lfb-def:%u lfb-8bit:%u\n", s, \
                  esp_get_free_heap_size(), esp_get_minimum_free_heap_size(), \
                  heap_caps_get_largest_free_block(MALLOC_CAP_DMA),\
                  heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),\
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT))

#define MEMPOOL_OK                       0
#define MEMPOOL_FAIL                     -1

#define CALLOC(x,y)                      calloc(x,y)
#define MALLOC(x)                        malloc(x)

#ifdef CONFIG_ESP_HOSTED_CP
#include "esp_heap_caps.h"
#define MEM_ALLOC(x)                     heap_caps_malloc(x, MALLOC_CAP_DMA)
#else
#include "eh_host_port_dma.h"
#define MEM_ALLOC(x)                     eh_host_port_dma_alloc(x)
#endif

#define FREE(x) do {                     \
	if (x) {                             \
		free(x);                         \
		x = NULL;                        \
	}                                    \
} while(0);

#define MEMPOOL_NAME_STR_SIZE            32

#define MEMPOOL_ALIGNMENT_BYTES          4
#define MEMPOOL_ALIGNMENT_MASK           (MEMPOOL_ALIGNMENT_BYTES-1)
#define IS_MEMPOOL_ALIGNED(VAL)          (!((VAL)& MEMPOOL_ALIGNMENT_MASK))
#define MEMPOOL_ALIGNED(VAL)             ((VAL) + MEMPOOL_ALIGNMENT_BYTES - \
                                             ((VAL)& MEMPOOL_ALIGNMENT_MASK))

#define MEMSET_REQUIRED                  1
#define MEMSET_NOT_REQUIRED              0

#define HOSTED_MEM_ALIGNMENT_4      4
#define HOSTED_MEM_ALIGNMENT_32     32
#define HOSTED_MEM_ALIGNMENT_64     64

struct hosted_mempool * hosted_mempool_create(void *pre_allocated_mem,
		size_t pre_allocated_mem_size, size_t num_blocks, size_t block_size);
void hosted_mempool_destroy(struct hosted_mempool *mempool);
void * hosted_mempool_alloc(struct hosted_mempool *mempool,
		size_t nbytes, uint8_t need_memset);
int hosted_mempool_free(struct hosted_mempool *mempool, void *mem);

#endif
