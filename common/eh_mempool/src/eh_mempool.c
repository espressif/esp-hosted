// SPDX-License-Identifier: Apache-2.0
// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD

#include "eh_mempool.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "HS_MP";

/* Pass pre_allocated_mem to use static memory; NULL allocates from heap. */
struct hosted_mempool * hosted_mempool_create(void *pre_allocated_mem,
		size_t pre_allocated_mem_size, size_t num_blocks, size_t block_size)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	struct hosted_mempool *new = NULL;
	struct eh_mp_mempool *pool = NULL;
	uint8_t *heap = NULL;
	char str[MEMPOOL_NAME_STR_SIZE] = {0};

	if (!pre_allocated_mem) {
		heap = (uint8_t *)MEM_ALLOC( MEMPOOL_ALIGNED(EH_MP_MEMPOOL_BYTES(
						num_blocks,block_size)));
		if (!heap) {
			ESP_LOGE(TAG, "mempool create failed, no mem\n");
			return NULL;
		}
	} else {
		heap = pre_allocated_mem;
		if (pre_allocated_mem_size < num_blocks*block_size) {
			ESP_LOGE(TAG, "mempool create failed, insufficient mem\n");
			return NULL;
		}

		if (!IS_MEMPOOL_ALIGNED((unsigned long)pre_allocated_mem)) {
			ESP_LOGE(TAG, "mempool create failed, mempool start addr unaligned\n");
			return NULL;
		}
	}

	new = (struct hosted_mempool*)CALLOC(1, sizeof(struct hosted_mempool));
	pool = (struct eh_mp_mempool *)CALLOC(1, sizeof(struct eh_mp_mempool));

	if(!new || !pool) {
		goto free_buffs;
	}

	snprintf(str, MEMPOOL_NAME_STR_SIZE, "hosted_%p", (void *)pool);

	if (eh_mp_mempool_init(pool, num_blocks, block_size, heap, str)) {
		ESP_LOGE(TAG, "eh_mp_mempool_init failed\n");
		goto free_buffs;
	}

	if (pre_allocated_mem)
		new->static_heap = 1;

	new->heap = heap;
	new->pool = pool;
	new->num_blocks = num_blocks;
	new->block_size = block_size;

#if MEMPOOL_DEBUG
	ESP_LOGI(TAG, "Create mempool %p with num_blk[%u] blk_size:[%u]", new->pool, (unsigned int)new->num_blocks, (unsigned int)new->block_size);
#endif

	return new;

free_buffs:
	FREE(new);
	FREE(pool);
	if (!pre_allocated_mem)
		FREE(heap);
	return NULL;
#else
	return NULL;
#endif
}

void hosted_mempool_destroy(struct hosted_mempool *mempool)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	if (!mempool)
		return;
#if MEMPOOL_DEBUG
	ESP_LOGI(MEM_TAG, "Destroy mempool %p num_blk[%lu] blk_size:[%lu]", mempool->pool, mempool->num_blocks, mempool->block_size);
#endif

	eh_mp_mempool_unregister(mempool->pool);
	FREE(mempool->pool);

	if (!mempool->static_heap)
		FREE(mempool->heap);

	FREE(mempool);
#endif
}

void * hosted_mempool_alloc(struct hosted_mempool *mempool,
		size_t nbytes, uint8_t need_memset)
{
	void *mem = NULL;

#ifdef CONFIG_ESP_CACHE_MALLOC
	if (!mempool) {
		ESP_LOGE(TAG, "mempool %p is NULL", mempool);
		return NULL;
	}

#if MYNEWT_VAL(EH_MP_MEMPOOL_CHECK)
	assert(mempool->heap);
	assert(mempool->pool);
#endif
	if(nbytes > mempool->block_size) {
		ESP_LOGE(TAG, "Exp alloc bytes[%u] > mempool block size[%u]\n",
				(unsigned int)nbytes, (unsigned int)mempool->block_size);
		return NULL;
	}

	mem = eh_mp_memblock_get(mempool->pool);
#else
	mem = MEM_ALLOC(MEMPOOL_ALIGNED(nbytes));
#endif
	if (mem && need_memset)
		memset(mem, 0, nbytes);

	if (!mem) {
		ESP_LOGE(TAG, "mempool %p alloc failed nbytes[%u]", mempool, nbytes);
	}
	return mem;
}

int hosted_mempool_free(struct hosted_mempool *mempool, void *mem)
{
	if (!mem) {
		return 0;
	}
#ifdef CONFIG_ESP_CACHE_MALLOC
	if (!mempool) {
		ESP_LOGE(TAG, "%s: mempool %p is NULL", __func__, mempool);
		return MEMPOOL_FAIL;
	}

#if MYNEWT_VAL(EH_MP_MEMPOOL_CHECK)
	assert(mempool->heap);
	assert(mempool->pool);
#endif

	return eh_mp_memblock_put(mempool->pool, mem);
#else
	FREE(mem);
	return 0;
#endif
}
