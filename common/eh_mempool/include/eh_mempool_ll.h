// SPDX-License-Identifier: Apache-2.0
/* eh_mempool_ll.h — mynewt os_mempool API (vendored). */

#ifndef _OS_MEMPOOL_H_
#define _OS_MEMPOOL_H_

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

#define OS_ALIGN(__n, __a) (                             \
        (((__n) & ((__a) - 1)) == 0)                   ? \
            (__n)                                      : \
            ((__n) + ((__a) - ((__n) & ((__a) - 1))))    \
        )
#define OS_ALIGNMENT 4

typedef uint32_t os_sr_t;

/* Critical-section hooks — backend provided by eh_mempool_ll_{host,cp}_priv.h. */
#define OS_INIT_CRITICAL() EH_MP_LOCK_INIT()
#define OS_ENTER_CRITICAL() EH_MP_LOCK()
#define OS_EXIT_CRITICAL() EH_MP_UNLOCK()

enum os_error {
    OS_OK = 0,
    OS_ENOMEM = 1,
    OS_EINVAL = 2,
    OS_INVALID_PARM = 3,
    OS_MEM_NOT_ALIGNED = 4,
    OS_BAD_MUTEX = 5,
    OS_TIMEOUT = 6,
    OS_ERR_IN_ISR = 7,      /* Function cannot be called from ISR */
    OS_ERR_PRIV = 8,        /* Privileged access error */
    OS_NOT_STARTED = 9,     /* OS must be started to call this function, but isn't */
    OS_ENOENT = 10,         /* No such thing */
    OS_EBUSY = 11,          /* Resource busy */
    OS_ERROR = 12,          /* Generic Error */
};

typedef enum os_error os_error_t;

/* TODO(mempool-layout): store first pool address in struct.
 * TODO(mempool-debug): optional debug metadata + helpers.
 * TODO(mempool-api): rename to SLIST_HEAD(, os_memblock) mp_head. */

struct os_memblock {
    SLIST_ENTRY(os_memblock) mb_next;
};

struct os_mempool {
    uint32_t mp_block_size;
    uint16_t mp_num_blocks;
    uint16_t mp_num_free;
    uint16_t mp_min_free;
    uint8_t mp_flags;
    uintptr_t mp_membuf_addr;
    STAILQ_ENTRY(os_mempool) mp_list;
    SLIST_HEAD(,os_memblock);
    const char *name;
};

/* Extended mempool — address safely castable to (struct os_mempool_ext *). */
#define OS_MEMPOOL_F_EXT        0x01

struct os_mempool_ext;

/* Put callback: runs on free instead of the default. Must call
 * os_memblock_put_from_cb() to actually release the block (else recursion). */
typedef os_error_t os_mempool_put_fn(struct os_mempool_ext *ome, void *data,
                                     void *arg);

struct os_mempool_ext {
    struct os_mempool mpe_mp;
    os_mempool_put_fn *mpe_put_cb;
    void *mpe_put_arg;
};

#define OS_MEMPOOL_INFO_NAME_LEN (32)

struct os_mempool_info {
    int omi_block_size;
    int omi_num_blocks;
    int omi_num_free;
    int omi_min_free;
    char omi_name[OS_MEMPOOL_INFO_NAME_LEN];
};

/* Iterate pools: pass NULL to start; returns next pool or NULL when done. */
struct os_mempool *os_mempool_info_get_next(struct os_mempool *,
        struct os_mempool_info *);

/* Pool buffer size — in os_membuf_t units, NOT bytes. */
#if (OS_CFG_ALIGNMENT == OS_CFG_ALIGN_4)
#define OS_MEMPOOL_SIZE(n,blksize)      ((((blksize) + 3) / 4) * (n))
typedef uint32_t os_membuf_t;
#else
#define OS_MEMPOOL_SIZE(n,blksize)      ((((blksize) + 7) / 8) * (n))
typedef uint64_t os_membuf_t;
#endif

#define OS_MEMPOOL_BYTES(n,blksize)     \
    (sizeof (os_membuf_t) * OS_MEMPOOL_SIZE((n), (blksize)))

os_error_t os_mempool_init(struct os_mempool *mp, uint16_t blocks,
                           uint32_t block_size, void *membuf, const char *name);

/* Callbacks must be assigned after init. */
os_error_t os_mempool_ext_init(struct os_mempool_ext *mpe, uint16_t blocks,
                               uint32_t block_size, void *membuf, const char *name);

os_error_t os_mempool_clear(struct os_mempool *mp);

void os_mempool_unregister(struct os_mempool *mp);

os_error_t os_mempool_ext_clear(struct os_mempool_ext *mpe);

/* Integrity check; true if sane. */
bool os_mempool_is_sane(const struct os_mempool *mp);

/* 1 if block belongs to mp, else 0. */
int os_memblock_from(const struct os_mempool *mp, const void *block_addr);

void *os_memblock_get(struct os_mempool *mp);

/* Free without firing put callback — only call from inside a put callback. */
os_error_t os_memblock_put_from_cb(struct os_mempool *mp, void *block_addr);

os_error_t os_memblock_put(struct os_mempool *mp, void *block_addr);

#ifdef __cplusplus
}
#endif

#endif  /* _OS_MEMPOOL_H_ */
