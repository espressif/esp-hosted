#include <zephyr.h>
#include <stdbool.h>
#include <stdio.h>

#include "csk_os_common/csk_os_memory.h"
#include "csk_os_common/csk_os_timer.h"

static struct k_work_q xr_ostimer_workq;
K_THREAD_STACK_DEFINE(xr_ostimer_workq_stack_area, 2048);

typedef struct {
    struct k_timer timer;   /* Timer handle */
    struct k_work work;
	struct k_work_sync work_sync;
    k_timeout_t period;
    k_timeout_t duration;
	csk_os_timer_type_t timer_type;
    csk_os_timer_handler_t  callback; /* Timer expire callback function */
    void *argument; /* Argument of timer expire callback function */
} z_timer_t;

void timer_entry(struct k_timer *timer)
{
	z_timer_t *timer_handle = k_timer_user_data_get(timer);
	k_work_submit_to_queue(&xr_ostimer_workq, &timer_handle->work);
}

static void timer_work_handler(struct k_work *work)
{
	z_timer_t *timer_handle = CONTAINER_OF(work, z_timer_t, work);
	timer_handle->callback(timer_handle->argument);
}

/**
 * @brief Create and initialize a timer object
 *
 * @note Creating a timer does not start the timer running. The OS_TimerStart()
 *       and OS_TimerChangePeriod() API functions can all be used to start the
 *       timer running.
 *
 * @param[in] timer Pointer to the timer object
 * @param[in] type Timer type
 * @param[in] cb Timer expire callback function
 * @param[in] arg Argument of Timer expire callback function
 * @param[in] period_ms Timer period in milliseconds
 * @retval OS_Status, OS_OK on success
 */
int csk_os_timer_create(csk_os_timer_t *timer, csk_os_timer_type_t type,
                         csk_os_timer_handler_t cb, void *arg, uint32_t period_ms)
{
	static bool is_init = false;
    if (!is_init) {
        k_work_queue_start(&xr_ostimer_workq,
                            xr_ostimer_workq_stack_area,
                            K_THREAD_STACK_SIZEOF(xr_ostimer_workq_stack_area),
                            2,
                            NULL);
        k_thread_name_set(&xr_ostimer_workq.thread, "xr_ostimer_workq");
        is_init = true;
    }

	z_timer_t *z_timer = csk_os_malloc(sizeof(z_timer_t));
	if (z_timer == NULL) {
		return -ENOMEM;
	}

	z_timer->timer_type = type;
	z_timer->callback = cb;
	z_timer->argument = arg;
	if (z_timer->timer_type == CSK_OS_TIMER_ONESHOT) {
		k_timeout_t wait = (period_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER : K_MSEC(period_ms);
		z_timer->duration = wait;
		z_timer->period = K_NO_WAIT;
	} else {
		k_timeout_t wait = (period_ms == CSK_OS_WAIT_FOREVER) ? K_FOREVER : K_MSEC(period_ms);
    	z_timer->duration = wait;
		z_timer->period = wait;
	}

	k_timer_init(&z_timer->timer, &timer_entry, NULL);
	k_timer_user_data_set(&z_timer->timer, z_timer);
	k_work_init(&z_timer->work, timer_work_handler);

	*timer = z_timer;
	return 0;
}

int csk_os_timer_start(csk_os_timer_t timer)
{
    z_timer_t *z_timer = timer;
    k_timer_start(&z_timer->timer, z_timer->duration, z_timer->period);
	return 0;
}

int csk_os_timer_delete(csk_os_timer_t timer)
{
	z_timer_t *z_timer = timer;
	k_timer_stop(&z_timer->timer);
	k_work_cancel(&z_timer->work);
	k_work_cancel_sync(&z_timer->work, &z_timer->work_sync);

	csk_os_free(z_timer);
	return 0;
}

// int XR_OS_TimerChangePeriod(csk_os_timer_t *timer, XR_OS_Time_t period_ms)
// {
//     z_timer_t *timer_handle = timer->handle;

// 	if (timer_handle->timer_type == CSK_OS_TIMER_ONESHOT) {
// 		k_timeout_t wait = (period_ms == XR_OS_WAIT_FOREVER) ? K_FOREVER : K_MSEC(period_ms);
// 		timer_handle->duration = wait;
// 		timer_handle->period = K_NO_WAIT;
// 	} else {
// 		k_timeout_t wait = (period_ms == XR_OS_WAIT_FOREVER) ? K_FOREVER : K_MSEC(period_ms);
//     	timer_handle->duration = wait;
// 		timer_handle->period = wait;
// 	}

//     k_timer_start(timer_handle->timer, timer_handle->duration, timer_handle->period);

// 	return XR_OS_OK;
// }

int csk_os_timer_stop(csk_os_timer_t timer)
{
    z_timer_t *z_timer = timer;
	k_work_cancel(&z_timer->work);
    k_timer_stop(&z_timer->timer);
	return 0;
}

int csk_os_timer_get_status(csk_os_timer_t timer)
{
    z_timer_t *z_timer = timer;
    return k_timer_remaining_ticks(&z_timer->timer) > 0;
}
