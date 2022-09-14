#pragma once

#include "csk_os_common/csk_os_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Timer type definition
 *     - one shot timer: Timer will be in the dormant state after it expires.
 *     - periodic timer: Timer will auto-reload after it expires.
 */
typedef enum {
    CSK_OS_TIMER_ONESHOT       = 0, /* one shot timer */
    CSK_OS_TIMER_PERIODIC   = 1  /* periodic timer */
} csk_os_timer_type_t;

/** @brief Timer expire callback function definition */
typedef void (*csk_os_timer_handler_t)(void *arg);

typedef void *csk_os_timer_t;

/**
 * @brief Create and initialize a timer object
 *
 * @note Creating a timer does not start the timer running. The csk_os_timer_start()
 *       and XR_OS_TimerChangePeriod() API functions can all be used to start the
 *       timer running.
 *
 * @param[in] timer Pointer to the timer object
 * @param[in] type Timer type
 * @param[in] cb Timer expire callback function
 * @param[in] arg Argument of Timer expire callback function
 * @param[in] period_ms Timer period in milliseconds
 * @retval int, XR_OS_OK on success
 */
int csk_os_timer_create(csk_os_timer_t *timer, csk_os_timer_type_t type,
                         csk_os_timer_handler_t cb, void *arg, uint32_t period_ms);

/**
 * @brief Delete the timer object
 * @param[in] timer Pointer to the timer object
 * @retval int, XR_OS_OK on success
 */
int csk_os_timer_delete(csk_os_timer_t timer);

/**
 * @brief Start a timer running.
 * @note If the timer is already running, this function will re-start the timer.
 * @param[in] timer Pointer to the timer object
 * @retval int, XR_OS_OK on success
 */
int csk_os_timer_start(csk_os_timer_t timer);

/**
 * @brief Change the period of a timer
 *
 * If OS_TimerChangePeriod() is used to change the period of a timer that is
 * already running, then the timer will use the new period value to recalculate
 * its expiry time. The recalculated expiry time will then be relative to when
 * OS_TimerChangePeriod() was called, and not relative to when the timer was
 * originally started.

 * If OS_TimerChangePeriod() is used to change the period of a timer that is
 * not already running, then the timer will use the new period value to
 * calculate an expiry time, and the timer will start running.
 *
 * @param[in] timer Pointer to the timer object
 * @retval OS_Status, OS_OK on success
 */
// int XR_OS_TimerChangePeriod(csk_os_timer_t *timer, XR_OS_Time_t period_ms);

/**
 * @brief Stop a timer running.
 * @param[in] timer Pointer to the timer object
 * @retval int, XR_OS_OK on success
 */
int csk_os_timer_stop(csk_os_timer_t timer);

int csk_os_timer_get_status(csk_os_timer_t timer);

#ifdef __cplusplus
}
#endif
