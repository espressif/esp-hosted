/* SPDX-License-Identifier: Apache-2.0 */

#ifndef EH_HOST_API_TYPES_H_
#define EH_HOST_API_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t major1;
    uint32_t minor1;
    uint32_t patch1;
    int32_t  revision;
    int32_t  prerelease;
    int32_t  build;
} eh_host_coprocessor_fwver_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_API_TYPES_H_ */
