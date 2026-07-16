// SPDX-License-Identifier: Apache-2.0

#ifndef EH_MEMPOOL_LL_CP_PRIV_H_
#define EH_MEMPOOL_LL_CP_PRIV_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_mp_lock;

#define EH_MP_LOCK_INIT()                                                   \
	do {                                                                    \
		if (!s_mp_lock) {                                                   \
			s_mp_lock = xSemaphoreCreateMutex();                            \
		}                                                                   \
	} while (0)

#define EH_MP_LOCK()                                                        \
	do {                                                                    \
		EH_MP_LOCK_INIT();                                                  \
		xSemaphoreTake(s_mp_lock, portMAX_DELAY);                           \
	} while (0)

#define EH_MP_UNLOCK()                                                      \
	do {                                                                    \
		if (s_mp_lock) {                                                    \
			xSemaphoreGive(s_mp_lock);                                      \
		}                                                                   \
	} while (0)

#endif /* EH_MEMPOOL_LL_CP_PRIV_H_ */

