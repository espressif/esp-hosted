/* SPDX-License-Identifier: Apache-2.0 */
/*
 * eh_host_port_gpio.h — general-purpose I/O
 *
 * Thin abstraction over a single pin.  Bulk-pin config, bus-specific
 * pin mux (SPI_D0, SDMMC_CLK, …) sit under their respective provider
 * / transport layers — the port boundary owns per-pin primitives only.
 *
 * Pin identity is the `eh_host_port_gpio_desc_t` record (port + pin) defined
 * in eh_host_port_master_config.h, reused here so the event-domain header can
 * cross-reference the same type.
 *
 * Interrupt mode + callback install is split to a separate sub-group
 * gate (EH_HOST_PORT_HAS_GPIO_INTR) so targets that lack MCU-level edge
 * interrupts can still expose read/write.
 */

#ifndef EH_HOST_PORT_GPIO_H_
#define EH_HOST_PORT_GPIO_H_

#include "eh_host_port_config.h"
#include "eh_host_port_err.h"
#include "eh_host_port_master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if EH_HOST_PORT_HAS_GPIO

typedef struct {
    uint32_t port;
    uint32_t pin;
} eh_host_port_gpio_desc_t;

typedef enum {
    EH_HOST_PORT_GPIO_LEVEL_LOW  = 0,
    EH_HOST_PORT_GPIO_LEVEL_HIGH = 1,
} eh_host_port_gpio_level_t;

typedef enum {
    EH_HOST_PORT_GPIO_DIR_INPUT        = 0,
    EH_HOST_PORT_GPIO_DIR_OUTPUT       = 1,
    EH_HOST_PORT_GPIO_DIR_INPUT_OUTPUT = 2,
} eh_host_port_gpio_dir_t;

typedef enum {
    EH_HOST_PORT_GPIO_PULL_NONE = 0,
    EH_HOST_PORT_GPIO_PULL_UP   = 1,
    EH_HOST_PORT_GPIO_PULL_DOWN = 2,
} eh_host_port_gpio_pull_t;

typedef enum {
    EH_HOST_PORT_GPIO_INTR_NONE      = 0,
    EH_HOST_PORT_GPIO_INTR_POSEDGE   = 1,
    EH_HOST_PORT_GPIO_INTR_NEGEDGE   = 2,
    EH_HOST_PORT_GPIO_INTR_ANYEDGE   = 3,
    EH_HOST_PORT_GPIO_INTR_LOW_LEVEL = 4,
    EH_HOST_PORT_GPIO_INTR_HIGH_LEVEL= 5,
} eh_host_port_gpio_intr_mode_t;

/* Output-drive strength — abstract, implementation picks nearest match.
 * Four-level ladder mirrors ESP GPIO_DRIVE_CAP_{0..3}; platforms with
 * fewer levels (STM32) collapse to their own nearest bucket. */
typedef enum {
    EH_HOST_PORT_GPIO_DRIVE_DEFAULT = 0,  /* platform default, e.g. CAP_2 */
    EH_HOST_PORT_GPIO_DRIVE_WEAK    = 1,
    EH_HOST_PORT_GPIO_DRIVE_MEDIUM  = 2,
    EH_HOST_PORT_GPIO_DRIVE_STRONG  = 3,
} eh_host_port_gpio_drive_t;


typedef struct {
    eh_host_port_gpio_desc_t      gpio;
    eh_host_port_gpio_dir_t       dir;
    eh_host_port_gpio_pull_t      pull;
    uint32_t                 flags;      /* reserved, must be 0 */
} eh_host_port_gpio_config_cfg_t;

eh_host_port_err_t eh_host_port_gpio_config(const eh_host_port_gpio_config_cfg_t *cfg);

eh_host_port_err_t eh_host_port_gpio_set(const eh_host_port_gpio_desc_t *gpio,
                                eh_host_port_gpio_level_t level);

/* Returns the current level as 0/1, or negative eh_host_port_err_t on failure. */
int           eh_host_port_gpio_get(const eh_host_port_gpio_desc_t *gpio);

/* Output drive strength override.  Useful for bus-peripheral pins
 * (SPI CLK / CS, I2C SDA pull) where the integrator needs more than
 * the default capability.  Implementations without per-pin drive
 * control return EH_HOST_PORT_ERR_NOSYS. */
eh_host_port_err_t eh_host_port_gpio_set_drive(const eh_host_port_gpio_desc_t *gpio,
                                      eh_host_port_gpio_drive_t drive);

#if EH_HOST_PORT_HAS_GPIO_INTR

/* ISR callback for an enabled pin.  Context is implementation-defined
 * (ISR on bare metal, tasklet on some Linux paths).  Callbacks MUST be
 * short and route through eh_host_port_sem_post_from_isr (or equivalent) to
 * notify a task for the real work. */
typedef void (*eh_host_port_gpio_isr_cb_t)(const eh_host_port_gpio_desc_t *gpio,
                                       void *ctx);

typedef struct {
    eh_host_port_gpio_desc_t       gpio;
    eh_host_port_gpio_intr_mode_t  mode;
    eh_host_port_gpio_isr_cb_t     cb;
    void                     *ctx;
    uint32_t                  flags;    /* reserved, must be 0 */
} eh_host_port_gpio_intr_enable_cfg_t;

eh_host_port_err_t eh_host_port_gpio_intr_enable(const eh_host_port_gpio_intr_enable_cfg_t *cfg);
eh_host_port_err_t eh_host_port_gpio_intr_disable(const eh_host_port_gpio_desc_t *gpio);

#endif /* EH_HOST_PORT_HAS_GPIO_INTR */

#if EH_HOST_PORT_HAS_GPIO_HOLD
/* Pin-state latching across sleep/deep-sleep.  hold=true freezes the
 * current output level; hold=false releases.  On targets without this
 * capability the gate defaults off and the symbol is not declared. */
eh_host_port_err_t eh_host_port_gpio_hold(const eh_host_port_gpio_desc_t *gpio, bool hold);
#endif

#endif /* EH_HOST_PORT_HAS_GPIO */

#ifdef __cplusplus
}
#endif

#endif /* EH_HOST_PORT_GPIO_H_ */
