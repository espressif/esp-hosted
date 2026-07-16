/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_err.h — error codes for the port layer
 *
 * Every `int`-returning port API uses these values.  Pointer-returning
 * constructors signal failure with NULL; int-returning constructors
 * (e.g. eh_host_port_task_create) write an opaque handle via out-param and
 * return one of these codes.
 *
 * Port-type implementations MUST translate their platform-native errors
 * (esp_err_t, errno) into these codes before returning to callers.
 * Callers MUST NOT compare an `int` return against esp_err_t / errno
 * values — that would leak the implementation's native error space
 * across the port boundary.
 */

#ifndef EH_HOST_PORT_ERR_H_
#define EH_HOST_PORT_ERR_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EH_HOST_PORT_OK              =  0,
    EH_HOST_PORT_ERR             = -1,  /* generic / unclassified failure */
    EH_HOST_PORT_ERR_TIMEOUT     = -2,  /* returned by *_wait_ms on deadline miss */
    EH_HOST_PORT_ERR_INVAL       = -3,  /* bad argument (null cfg, out-of-range, …) */
    EH_HOST_PORT_ERR_NOMEM       = -4,  /* allocation failed */
    EH_HOST_PORT_ERR_NOSYS       = -5,  /* capability not built in / not supported */
    EH_HOST_PORT_ERR_BUSY        = -6,  /* resource contended / try again later */
    /* Capability is built-in but the integrator didn't supply the
     * board-level configuration (RST pin = -1, no I2C bus address, …).
     * Distinct from NOSYS — the code path exists, but it has nothing
     * to act on.  Callers usually treat this as "skip silently" rather
     * than "fail the call". */
    EH_HOST_PORT_ERR_NOT_CONFIGURED = -7,
} eh_host_port_err_t;

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_ERR_H_ */
