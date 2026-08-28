// SPDX-License-Identifier: Apache-2.0
/* eh_mempool_ll.h — a private, self-contained block pool, originally vendored
 * from mynewt's os_mempool.
 *
 * Everything here is namespaced eh_mp_* / EH_MP_* on purpose. It used to export
 * the mynewt names (os_mempool_init, os_memblock_get, struct os_mempool, and the
 * _OS_MEMPOOL_H_ include guard), all of which the ESP-IDF Bluetooth component
 * also defines in components/bt/porting/mem/os_mempool.c. With BT linked the two
 * implementations got mixed by link order - the pool was initialised by this
 * file and allocated from by NimBLE's IRAM copy - which left a garbage free-list
 * head and faulted on the next allocation. Keep this namespace private. */

#ifndef _EH_MEMPOOL_LL_H_
#define _EH_MEMPOOL_LL_H_

#include <stdbool.h>
#include <stdint.h>
#include "sys/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MYNEWT_VAL_OS_MEMPOOL_POISON (0)
#define MYNEWT_VAL_OS_MEMPOOL_CHECK (0)

#define MYNEWT_VAL(_name)                       MYNEWT_VAL_ ## _name
#define MYNEWT_VAL_CHOICE(_name, _val)          MYNEWT_VAL_ ## _name ## __ ## _val


#ifndef min
#define min(a, b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#define max(a, b) ((a)>(b)?(a):(b))
#endif

#define EH_MP_ALIGN(__n, __a) (                             \
        (((__n) & ((__a) - 1)) == 0)                   ? \
            (__n)                                      : \
            ((__n) + ((__a) - ((__n) & ((__a) - 1))))    \
        )
#define EH_MP_ALIGNMENT 4

typedef uint32_t eh_mp_sr_t;

/* Critical-section hooks — backend provided by eh_mempool_ll_{host,cp}_priv.h. */
#define EH_MP_INIT_CRITICAL() EH_MP_LOCK_INIT()
#define EH_MP_ENTER_CRITICAL() EH_MP_LOCK()
#define EH_MP_EXIT_CRITICAL() EH_MP_UNLOCK()

enum eh_mp_error {
    EH_MP_OK = 0,
    EH_MP_ENOMEM = 1,
    EH_MP_EINVAL = 2,
    EH_MP_INVALID_PARM = 3,
    EH_MP_MEM_NOT_ALIGNED = 4,
    EH_MP_BAD_MUTEX = 5,
    EH_MP_TIMEOUT = 6,
    EH_MP_ERR_IN_ISR = 7,      /* Function cannot be called from ISR */
    EH_MP_ERR_PRIV = 8,        /* Privileged access error */
    EH_MP_NOT_STARTED = 9,     /* OS must be started to call this function, but isn't */
    EH_MP_ENOENT = 10,         /* No such thing */
    EH_MP_EBUSY = 11,          /* Resource busy */
    EH_MP_ERROR = 12,          /* Generic Error */
};

typedef enum eh_mp_error eh_mp_error_t;

/* TODO(mempool-layout): store first pool address in struct.
 * TODO(mempool-debug): optional debug metadata + helpers.
 * TODO(mempool-api): rename to SLIST_HEAD(, eh_mp_memblock) mp_head. */

struct eh_mp_memblock {
    SLIST_ENTRY(eh_mp_memblock) mb_next;
};

struct eh_mp_mempool {
    uint32_t mp_block_size;
    uint16_t mp_num_blocks;
    uint16_t mp_num_free;
    uint16_t mp_min_free;
    uint8_t mp_flags;
    uintptr_t mp_membuf_addr;
    STAILQ_ENTRY(eh_mp_mempool) mp_list;
    SLIST_HEAD(,eh_mp_memblock);
    const char *name;
};

/* Extended mempool — address safely castable to (struct eh_mp_mempool_ext *). */
#define EH_MP_MEMPOOL_F_EXT        0x01

struct eh_mp_mempool_ext;

/* Put callback: runs on free instead of the default. Must call
 * eh_mp_memblock_put_from_cb() to actually release the block (else recursion). */
typedef eh_mp_error_t eh_mp_mempool_put_fn(struct eh_mp_mempool_ext *ome, void *data,
                                     void *arg);

struct eh_mp_mempool_ext {
    struct eh_mp_mempool mpe_mp;
    eh_mp_mempool_put_fn *mpe_put_cb;
    void *mpe_put_arg;
};

#define EH_MP_MEMPOOL_INFO_NAME_LEN (32)

struct eh_mp_mempool_info {
    int omi_block_size;
    int omi_num_blocks;
    int omi_num_free;
    int omi_min_free;
    char omi_name[EH_MP_MEMPOOL_INFO_NAME_LEN];
};

/* Iterate pools: pass NULL to start; returns next pool or NULL when done. */
struct eh_mp_mempool *eh_mp_mempool_info_get_next(struct eh_mp_mempool *,
        struct eh_mp_mempool_info *);

/* Pool buffer size — in eh_mp_membuf_t units, NOT bytes. */
#if (EH_MP_ALIGNMENT == 4)
#define EH_MP_MEMPOOL_SIZE(n,blksize)      ((((blksize) + 3) / 4) * (n))
typedef uint32_t eh_mp_membuf_t;
#else
#define EH_MP_MEMPOOL_SIZE(n,blksize)      ((((blksize) + 7) / 8) * (n))
typedef uint64_t eh_mp_membuf_t;
#endif

#define EH_MP_MEMPOOL_BYTES(n,blksize)     \
    (sizeof (eh_mp_membuf_t) * EH_MP_MEMPOOL_SIZE((n), (blksize)))

eh_mp_error_t eh_mp_mempool_init(struct eh_mp_mempool *mp, uint16_t blocks,
                           uint32_t block_size, void *membuf, const char *name);

/* Callbacks must be assigned after init. */
eh_mp_error_t eh_mp_mempool_ext_init(struct eh_mp_mempool_ext *mpe, uint16_t blocks,
                               uint32_t block_size, void *membuf, const char *name);

eh_mp_error_t eh_mp_mempool_clear(struct eh_mp_mempool *mp);

void eh_mp_mempool_unregister(struct eh_mp_mempool *mp);

eh_mp_error_t eh_mp_mempool_ext_clear(struct eh_mp_mempool_ext *mpe);

/* Integrity check; true if sane. */
bool eh_mp_mempool_is_sane(const struct eh_mp_mempool *mp);

/* 1 if block belongs to mp, else 0. */
int eh_mp_memblock_from(const struct eh_mp_mempool *mp, const void *block_addr);

void *eh_mp_memblock_get(struct eh_mp_mempool *mp);

/* Free without firing put callback — only call from inside a put callback. */
eh_mp_error_t eh_mp_memblock_put_from_cb(struct eh_mp_mempool *mp, void *block_addr);

eh_mp_error_t eh_mp_memblock_put(struct eh_mp_mempool *mp, void *block_addr);

#ifdef __cplusplus
}
#endif

#endif  /* _EH_MEMPOOL_LL_H_ */
