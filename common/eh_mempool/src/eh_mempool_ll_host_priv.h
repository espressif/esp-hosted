// SPDX-License-Identifier: Apache-2.0

#ifndef EH_MEMPOOL_LL_HOST_PRIV_H_
#define EH_MEMPOOL_LL_HOST_PRIV_H_

#include "eh_host_port_sync.h"

static eh_host_port_mutex_t *s_mp_lock;

#define EH_MP_LOCK_INIT()                                                   \
	do {                                                                    \
		if (!s_mp_lock) {                                                   \
			s_mp_lock = eh_host_port_mutex_create();                        \
		}                                                                   \
	} while (0)

#define EH_MP_LOCK()                                                        \
	do {                                                                    \
		EH_MP_LOCK_INIT();                                                  \
		eh_host_port_mutex_lock(s_mp_lock);                                 \
	} while (0)

#define EH_MP_UNLOCK()                                                      \
	do {                                                                    \
		if (s_mp_lock) {                                                    \
			eh_host_port_mutex_unlock(s_mp_lock);                           \
		}                                                                   \
	} while (0)

#endif /* EH_MEMPOOL_LL_HOST_PRIV_H_ */

