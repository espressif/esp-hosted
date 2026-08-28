// SPDX-License-Identifier: Apache-2.0
/* eh_mempool_ll.c — mynewt eh_mp_mempool implementation (vendored). */

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

#define EH_MP_MEM_TRUE_BLOCK_SIZE(bsize)   EH_MP_ALIGN(bsize, EH_MP_ALIGNMENT)
#define EH_MP_MEMPOOL_TRUE_BLOCK_SIZE(mp) EH_MP_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size)

STAILQ_HEAD(, eh_mp_mempool) g_os_hosted_mempool_list =
    STAILQ_HEAD_INITIALIZER(g_os_hosted_mempool_list);

#if MYNEWT_VAL(EH_MP_MEMPOOL_POISON)
static uint32_t eh_mp_mem_poison = 0xde7ec7ed;

static void
eh_mp_mempool_poison(void *start, int sz)
{
	int i;
	char *p = start;

	for (i = sizeof(struct eh_mp_memblock); i < sz;
			i = i + sizeof(eh_mp_mem_poison)) {
		memcpy(p + i, &eh_mp_mem_poison, min(sizeof(eh_mp_mem_poison), sz - i));
	}
}

static void
eh_mp_mempool_poison_check(void *start, int sz)
{
	int i;
	char *p = start;

	for (i = sizeof(struct eh_mp_memblock); i < sz;
			i = i + sizeof(eh_mp_mem_poison)) {
		assert(!memcmp(p + i, &eh_mp_mem_poison,
					min(sizeof(eh_mp_mem_poison), sz - i)));
	}
}
#else
#define eh_mp_mempool_poison(start, sz)
#define eh_mp_mempool_poison_check(start, sz)
#endif

eh_mp_error_t
eh_mp_mempool_init(struct eh_mp_mempool *mp, uint16_t blocks, uint32_t block_size,
                void *membuf, const char *name)
{
	int true_block_size;
	uint8_t *block_addr;
	struct eh_mp_memblock *block_ptr;

	if (!mp || (block_size == 0)) {
		return EH_MP_INVALID_PARM;
	}

	if ((!membuf) && (blocks != 0)) {
		return EH_MP_INVALID_PARM;
	}
	EH_MP_INIT_CRITICAL();

	if (membuf != NULL) {
		if (((uintptr_t)membuf & (EH_MP_ALIGNMENT - 1)) != 0) {
			return EH_MP_MEM_NOT_ALIGNED;
		}
	}
	true_block_size = EH_MP_MEM_TRUE_BLOCK_SIZE(block_size);

	mp->mp_block_size = block_size;
	mp->mp_num_free = blocks;
	mp->mp_min_free = blocks;
	mp->mp_flags = 0;
	mp->mp_num_blocks = blocks;
	mp->mp_membuf_addr = (uintptr_t)membuf;
	mp->name = name;
	eh_mp_mempool_poison(membuf, true_block_size);
	SLIST_FIRST(mp) = membuf;

	block_addr = (uint8_t *)membuf;
	block_ptr = (struct eh_mp_memblock *)(void *)block_addr;
	while (blocks > 1) {
		block_addr += true_block_size;
		eh_mp_mempool_poison(block_addr, true_block_size);
		SLIST_NEXT(block_ptr, mb_next) = (struct eh_mp_memblock *)(void *)block_addr;
		block_ptr = (struct eh_mp_memblock *)(void *)block_addr;
		--blocks;
	}

	SLIST_NEXT(block_ptr, mb_next) = NULL;

	STAILQ_INSERT_TAIL(&g_os_hosted_mempool_list, mp, mp_list);

	return EH_MP_OK;
}

eh_mp_error_t
eh_mp_mempool_ext_init(struct eh_mp_mempool_ext *mpe, uint16_t blocks,
                    uint32_t block_size, void *membuf, const char *name)
{
	int rc;

	rc = eh_mp_mempool_init(&mpe->mpe_mp, blocks, block_size, membuf, name);
	if (rc != 0) {
		return rc;
	}

	mpe->mpe_mp.mp_flags = EH_MP_MEMPOOL_F_EXT;
	mpe->mpe_put_cb = NULL;
	mpe->mpe_put_arg = NULL;

	return 0;
}

eh_mp_error_t
eh_mp_mempool_clear(struct eh_mp_mempool *mp)
{
	struct eh_mp_memblock *block_ptr;
	int true_block_size;
	uint8_t *block_addr;
	uint16_t blocks;

	if (!mp) {
		return EH_MP_INVALID_PARM;
	}

	true_block_size = EH_MP_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size);

	mp->mp_num_free = mp->mp_num_blocks;
	mp->mp_min_free = mp->mp_num_blocks;
	eh_mp_mempool_poison((void *)mp->mp_membuf_addr, true_block_size);
	SLIST_FIRST(mp) = (void *)mp->mp_membuf_addr;

	block_addr = (uint8_t *)mp->mp_membuf_addr;
	block_ptr = (struct eh_mp_memblock *)(void *)block_addr;
	blocks = mp->mp_num_blocks;

	while (blocks > 1) {
		block_addr += true_block_size;
		eh_mp_mempool_poison(block_addr, true_block_size);
		SLIST_NEXT(block_ptr, mb_next) = (struct eh_mp_memblock *)(void *)block_addr;
		block_ptr = (struct eh_mp_memblock *)(void *)block_addr;
		--blocks;
	}

	SLIST_NEXT(block_ptr, mb_next) = NULL;

	return EH_MP_OK;
}

eh_mp_error_t
eh_mp_mempool_ext_clear(struct eh_mp_mempool_ext *mpe)
{
	mpe->mpe_mp.mp_flags = 0;
	mpe->mpe_put_cb = NULL;
	mpe->mpe_put_arg = NULL;

	return eh_mp_mempool_clear(&mpe->mpe_mp);
}

void
eh_mp_mempool_unregister(struct eh_mp_mempool *mp)
{
	STAILQ_REMOVE(&g_os_hosted_mempool_list, mp, eh_mp_mempool, mp_list);
}

bool
eh_mp_mempool_is_sane(const struct eh_mp_mempool *mp)
{
	struct eh_mp_memblock *block;

	SLIST_FOREACH(block, mp, mb_next) {
		if (!eh_mp_memblock_from(mp, block)) {
			return false;
		}
		eh_mp_mempool_poison_check(block, EH_MP_MEMPOOL_TRUE_BLOCK_SIZE(mp));
	}

	return true;
}

int
eh_mp_memblock_from(const struct eh_mp_mempool *mp, const void *block_addr)
{
	uintptr_t true_block_size;
	uintptr_t baddr_ptr;
	uintptr_t end;

	_Static_assert(sizeof block_addr == sizeof baddr_ptr,
			"Pointer to void must be native word size.");

	baddr_ptr = (uintptr_t)block_addr;
	true_block_size = EH_MP_MEMPOOL_TRUE_BLOCK_SIZE(mp);
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
eh_mp_memblock_get(struct eh_mp_mempool *mp)
{
	struct eh_mp_memblock *block;

	block = NULL;
	if (mp) {
		EH_MP_ENTER_CRITICAL();
		if (mp->mp_num_free) {
			block = SLIST_FIRST(mp);
			SLIST_FIRST(mp) = SLIST_NEXT(block, mb_next);

			mp->mp_num_free--;
			if (mp->mp_min_free > mp->mp_num_free) {
				mp->mp_min_free = mp->mp_num_free;
			}
		}
		EH_MP_EXIT_CRITICAL();

		if (block) {
			eh_mp_mempool_poison_check(block, EH_MP_MEMPOOL_TRUE_BLOCK_SIZE(mp));
		}
	}

	return (void *)block;
}

eh_mp_error_t
eh_mp_memblock_put_from_cb(struct eh_mp_mempool *mp, void *block_addr)
{
	struct eh_mp_memblock *block;

	eh_mp_mempool_poison(block_addr, EH_MP_MEMPOOL_TRUE_BLOCK_SIZE(mp));

	block = (struct eh_mp_memblock *)(void *)block_addr;
	EH_MP_ENTER_CRITICAL();

	SLIST_NEXT(block, mb_next) = SLIST_FIRST(mp);
	SLIST_FIRST(mp) = block;

	/* TODO(mempool-safety): assert mp_num_free <= mp_num_blocks. */
	mp->mp_num_free++;

	EH_MP_EXIT_CRITICAL();

	return EH_MP_OK;
}

eh_mp_error_t
eh_mp_memblock_put(struct eh_mp_mempool *mp, void *block_addr)
{
	struct eh_mp_mempool_ext *mpe;
	int rc;
#if MYNEWT_VAL(EH_MP_MEMPOOL_CHECK)
	struct eh_mp_memblock *block;
#endif

	if ((mp == NULL) || (block_addr == NULL)) {
		return EH_MP_INVALID_PARM;
	}

#if MYNEWT_VAL(EH_MP_MEMPOOL_CHECK)
	assert(eh_mp_memblock_from(mp, block_addr));

	/* Duplicate-free check. */
	SLIST_FOREACH(block, mp, mb_next) {
		assert(block != (struct eh_mp_memblock *)(void *)block_addr);
	}
#endif

	/* Extended mempool with put cb: defer to callback. */
	if (mp->mp_flags & EH_MP_MEMPOOL_F_EXT) {
		mpe = (struct eh_mp_mempool_ext *)mp;
		if (mpe->mpe_put_cb != NULL) {
			rc = mpe->mpe_put_cb(mpe, block_addr, mpe->mpe_put_arg);
			return rc;
		}
	}

	return eh_mp_memblock_put_from_cb(mp, block_addr);
}

struct eh_mp_mempool *
eh_mp_mempool_info_get_next(struct eh_mp_mempool *mp, struct eh_mp_mempool_info *omi)
{
	struct eh_mp_mempool *cur;

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
