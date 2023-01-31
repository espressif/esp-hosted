// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#ifndef __MEMPOOL_H__
#define __MEMPOOL_H__

#include <string.h>
#include <stdio.h>
#include <sys/queue.h>
#include "os_wrapper.h"

#define CONFIG_ESP_CACHE_MALLOC 1


#define MEMPOOL_OK                       0
#define MEMPOOL_FAIL                     -1


#define LOG                              printf

#define MEMPOOL_NAME_STR_SIZE            32

#define MEMPOOL_ALIGNMENT_BYTES          4
#define MEMPOOL_ALIGNMENT_MASK           (MEMPOOL_ALIGNMENT_BYTES-1)
#define IS_MEMPOOL_ALIGNED(VAL)          (!((VAL)& MEMPOOL_ALIGNMENT_MASK))
#define MEMPOOL_ALIGNED(VAL)             ((VAL) + MEMPOOL_ALIGNMENT_BYTES - \
                                             ((VAL)& MEMPOOL_ALIGNMENT_MASK))

#define MEMSET_REQUIRED                  1
#define MEMSET_NOT_REQUIRED              0


#ifdef CONFIG_ESP_CACHE_MALLOC
struct mempool_entry {
	SLIST_ENTRY(mempool_entry) entries;
};

typedef SLIST_HEAD(slisthead, mempool_entry) mempool_t;

struct mempool {
	mempool_t head;
	void * spinlock;
	uint32_t block_size;
};
#endif

struct mempool * mempool_create(uint32_t block_size);
void mempool_destroy(struct mempool* mp);
void * mempool_alloc(struct mempool* mp, int nbytes, int need_memset);
void mempool_free(struct mempool* mp, void *mem);
#endif
