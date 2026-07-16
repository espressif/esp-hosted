/* SPDX-License-Identifier: Apache-2.0 */
/* glibc feature-test macros, force-included via CMakeLists. Guarded so caller -D wins. */
#pragma once

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
