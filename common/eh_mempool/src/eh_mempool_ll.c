// SPDX-License-Identifier: Apache-2.0
/* eh_mempool_ll.c — mynewt os_mempool implementation (vendored). */

#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include "sdkconfig.h"
#if CONFIG_ESP_HOSTED_HOST
#include "eh_mempool_ll_host_priv.h"
#else
#include "eh_mempool_ll_cp_priv.h"
#endif
#include "eh_mempool_ll.h"

#define OS_MEM_TRUE_BLOCK_SIZE(bsize)   OS_ALIGN(bsize, OS_ALIGNMENT)
#define OS_MEMPOOL_TRUE_BLOCK_SIZE(mp) OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size)

STAILQ_HEAD(, os_mempool) g_os_hosted_mempool_list =
    STAILQ_HEAD_INITIALIZER(g_os_hosted_mempool_list);

#if MYNEWT_VAL(OS_MEMPOOL_POISON)
static uint32_t os_mem_poison = 0xde7ec7ed;

static void
os_mempool_poison(void *start, int sz)
{
	int i;
	char *p = start;

	for (i = sizeof(struct os_memblock); i < sz;
			i = i + sizeof(os_mem_poison)) {
		memcpy(p + i, &os_mem_poison, min(sizeof(os_mem_poison), sz - i));
	}
}

static void
os_mempool_poison_check(void *start, int sz)
{
	int i;
	char *p = start;

	for (i = sizeof(struct os_memblock); i < sz;
			i = i + sizeof(os_mem_poison)) {
		assert(!memcmp(p + i, &os_mem_poison,
					min(sizeof(os_mem_poison), sz - i)));
	}
}
#else
#define os_mempool_poison(start, sz)
#define os_mempool_poison_check(start, sz)
#endif

os_error_t
os_mempool_init(struct os_mempool *mp, uint16_t blocks, uint32_t block_size,
                void *membuf, const char *name)
{
	int true_block_size;
	uint8_t *block_addr;
	struct os_memblock *block_ptr;

	if (!mp || (block_size == 0)) {
		return OS_INVALID_PARM;
	}

	if ((!membuf) && (blocks != 0)) {
		return OS_INVALID_PARM;
	}
	OS_INIT_CRITICAL();

	if (membuf != NULL) {
		if (((uintptr_t)membuf & (OS_ALIGNMENT - 1)) != 0) {
			return OS_MEM_NOT_ALIGNED;
		}
	}
	true_block_size = OS_MEM_TRUE_BLOCK_SIZE(block_size);

	mp->mp_block_size = block_size;
	mp->mp_num_free = blocks;
	mp->mp_min_free = blocks;
	mp->mp_flags = 0;
	mp->mp_num_blocks = blocks;
	mp->mp_membuf_addr = (uintptr_t)membuf;
	mp->name = name;
	os_mempool_poison(membuf, true_block_size);
	SLIST_FIRST(mp) = membuf;

	block_addr = (uint8_t *)membuf;
	block_ptr = (struct os_memblock *)block_addr;
	while (blocks > 1) {
		block_addr += true_block_size;
		os_mempool_poison(block_addr, true_block_size);
		SLIST_NEXT(block_ptr, mb_next) = (struct os_memblock *)block_addr;
		block_ptr = (struct os_memblock *)block_addr;
		--blocks;
	}

	SLIST_NEXT(block_ptr, mb_next) = NULL;

	STAILQ_INSERT_TAIL(&g_os_hosted_mempool_list, mp, mp_list);

	return OS_OK;
}

os_error_t
os_mempool_ext_init(struct os_mempool_ext *mpe, uint16_t blocks,
                    uint32_t block_size, void *membuf, const char *name)
{
	int rc;

	rc = os_mempool_init(&mpe->mpe_mp, blocks, block_size, membuf, name);
	if (rc != 0) {
		return rc;
	}

	mpe->mpe_mp.mp_flags = OS_MEMPOOL_F_EXT;
	mpe->mpe_put_cb = NULL;
	mpe->mpe_put_arg = NULL;

	return 0;
}

os_error_t
os_mempool_clear(struct os_mempool *mp)
{
	struct os_memblock *block_ptr;
	int true_block_size;
	uint8_t *block_addr;
	uint16_t blocks;

	if (!mp) {
		return OS_INVALID_PARM;
	}

	true_block_size = OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size);

	mp->mp_num_free = mp->mp_num_blocks;
	mp->mp_min_free = mp->mp_num_blocks;
	os_mempool_poison((void *)mp->mp_membuf_addr, true_block_size);
	SLIST_FIRST(mp) = (void *)mp->mp_membuf_addr;

	block_addr = (uint8_t *)mp->mp_membuf_addr;
	block_ptr = (struct os_memblock *)block_addr;
	blocks = mp->mp_num_blocks;

	while (blocks > 1) {
		block_addr += true_block_size;
		os_mempool_poison(block_addr, true_block_size);
		SLIST_NEXT(block_ptr, mb_next) = (struct os_memblock *)block_addr;
		block_ptr = (struct os_memblock *)block_addr;
		--blocks;
	}

	SLIST_NEXT(block_ptr, mb_next) = NULL;

	return OS_OK;
}

os_error_t
os_mempool_ext_clear(struct os_mempool_ext *mpe)
{
	mpe->mpe_mp.mp_flags = 0;
	mpe->mpe_put_cb = NULL;
	mpe->mpe_put_arg = NULL;

	return os_mempool_clear(&mpe->mpe_mp);
}

void
os_mempool_unregister(struct os_mempool *mp)
{
	STAILQ_REMOVE(&g_os_hosted_mempool_list, mp, os_mempool, mp_list);
}

bool
os_mempool_is_sane(const struct os_mempool *mp)
{
	struct os_memblock *block;

	SLIST_FOREACH(block, mp, mb_next) {
		if (!os_memblock_from(mp, block)) {
			return false;
		}
		os_mempool_poison_check(block, OS_MEMPOOL_TRUE_BLOCK_SIZE(mp));
	}

	return true;
}

int
os_memblock_from(const struct os_mempool *mp, const void *block_addr)
{
	uintptr_t true_block_size;
	uintptr_t baddr_ptr;
	uintptr_t end;

	_Static_assert(sizeof block_addr == sizeof baddr_ptr,
			"Pointer to void must be native word size.");

	baddr_ptr = (uintptr_t)block_addr;
	true_block_size = OS_MEMPOOL_TRUE_BLOCK_SIZE(mp);
	end = mp->mp_membuf_addr + (mp->mp_num_blocks * true_block_size);

	if ((baddr_ptr < mp->mp_membuf_addr) || (baddr_ptr >= end)) {
		return 0;
	}

	if (((baddr_ptr - mp->mp_membuf_addr) % true_block_size) != 0) {
		return 0;
	}

	return 1;
}

void *
os_memblock_get(struct os_mempool *mp)
{
	struct os_memblock *block;

	block = NULL;
	if (mp) {
		OS_ENTER_CRITICAL();
		if (mp->mp_num_free) {
			block = SLIST_FIRST(mp);
			SLIST_FIRST(mp) = SLIST_NEXT(block, mb_next);

			mp->mp_num_free--;
			if (mp->mp_min_free > mp->mp_num_free) {
				mp->mp_min_free = mp->mp_num_free;
			}
		}
		OS_EXIT_CRITICAL();

		if (block) {
			os_mempool_poison_check(block, OS_MEMPOOL_TRUE_BLOCK_SIZE(mp));
		}
	}

	return (void *)block;
}

os_error_t
os_memblock_put_from_cb(struct os_mempool *mp, void *block_addr)
{
	struct os_memblock *block;

	os_mempool_poison(block_addr, OS_MEMPOOL_TRUE_BLOCK_SIZE(mp));

	block = (struct os_memblock *)block_addr;
	OS_ENTER_CRITICAL();

	SLIST_NEXT(block, mb_next) = SLIST_FIRST(mp);
	SLIST_FIRST(mp) = block;

	/* TODO(mempool-safety): assert mp_num_free <= mp_num_blocks. */
	mp->mp_num_free++;

	OS_EXIT_CRITICAL();

	return OS_OK;
}

os_error_t
os_memblock_put(struct os_mempool *mp, void *block_addr)
{
	struct os_mempool_ext *mpe;
	int rc;
#if MYNEWT_VAL(OS_MEMPOOL_CHECK)
	struct os_memblock *block;
#endif

	if ((mp == NULL) || (block_addr == NULL)) {
		return OS_INVALID_PARM;
	}

#if MYNEWT_VAL(OS_MEMPOOL_CHECK)
	assert(os_memblock_from(mp, block_addr));

	/* Duplicate-free check. */
	SLIST_FOREACH(block, mp, mb_next) {
		assert(block != (struct os_memblock *)block_addr);
	}
#endif

	/* Extended mempool with put cb: defer to callback. */
	if (mp->mp_flags & OS_MEMPOOL_F_EXT) {
		mpe = (struct os_mempool_ext *)mp;
		if (mpe->mpe_put_cb != NULL) {
			rc = mpe->mpe_put_cb(mpe, block_addr, mpe->mpe_put_arg);
			return rc;
		}
	}

	return os_memblock_put_from_cb(mp, block_addr);
}

struct os_mempool *
os_mempool_info_get_next(struct os_mempool *mp, struct os_mempool_info *omi)
{
	struct os_mempool *cur;

	if (mp == NULL) {
		cur = STAILQ_FIRST(&g_os_hosted_mempool_list);
	} else {
		cur = STAILQ_NEXT(mp, mp_list);
	}

	if (cur == NULL) {
		return (NULL);
	}

	omi->omi_block_size = cur->mp_block_size;
	omi->omi_num_blocks = cur->mp_num_blocks;
	omi->omi_num_free = cur->mp_num_free;
	omi->omi_min_free = cur->mp_min_free;
	if (cur->name) {
		strncpy(omi->omi_name, cur->name, sizeof(omi->omi_name) - 1);
		omi->omi_name[sizeof(omi->omi_name) - 1] = '\0';
	} else {
		omi->omi_name[0] = '\0';
	}

	return (cur);
}
