/* SPDX-License-Identifier: Apache-2.0 */
/*
 * lwip/err.h — type-only mirror for non-IDF hosts.
 * Real lwIP runs on the CP, not the host.
 */

#ifndef EH_COMPAT_LWIP_ERR_H_
#define EH_COMPAT_LWIP_ERR_H_

#include <stdint.h>

typedef int8_t err_t;

#define ERR_OK          0
#define ERR_MEM        -1
#define ERR_BUF        -2
#define ERR_TIMEOUT    -3
#define ERR_RTE        -4
#define ERR_INPROGRESS -5
#define ERR_VAL        -6
#define ERR_WOULDBLOCK -7
#define ERR_USE        -8
#define ERR_ALREADY    -9
#define ERR_ISCONN     -10

#endif /* EH_COMPAT_LWIP_ERR_H_ */
