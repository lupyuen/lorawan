//  Timer Functions ported from Mynewt to NimBLE Porting Layer
#ifndef __LORA_TIMER_H__
#define __LORA_TIMER_H__

#include "nimble_npl.h"      //  For NimBLE Porting Layer (timer functions)

/// Initialise a timer. Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_init
void os_cputime_timer_init(
    struct ble_npl_callout *timer,   //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f,             //  The timer callback function. Cannot be NULL.
    void *arg);                      //  Pointer to data object to pass to timer.

/// Stops a timer from running.  Can be called even if timer is not running.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_stop
void os_cputime_timer_stop(
    struct ble_npl_callout *timer);  //  Pointer to timer to stop. Cannot be NULL.

/// Sets a timer that will expire ‘usecs’ microseconds from the current time.
/// NOTE: This must be called when the timer is stopped.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_relative
void os_cputime_timer_relative(
    struct ble_npl_callout *timer,   //  Pointer to timer. Cannot be NULL.
    uint32_t microsecs);             //  The number of microseconds from now at which the timer will expire.

/// Get the current OS time in ticks.
ble_npl_time_t os_time_get(void);

/// Converts OS ticks to milliseconds. Return 0 for success.
int os_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms);

/// Get the current time in microseconds
uint32_t timer_get_current_time(void);

/// Init the HAL Timer
void lora_bsp_enable_mac_timer(void);

/**
 * Un-initialize a HW timer.
 *
 * @param timer_num The number of the HW timer to un-initialize
 */
int hal_timer_deinit(int timer_num);

/**
 * Config a HW timer at the given frequency and start it. If the exact
 * frequency is not obtainable the closest obtainable frequency is set.
 *
 * @param timer_num The number of the HW timer to configure
 * @param freq_hz   The frequency in Hz to configure the timer at
 *                  (Must be microseconds)
 *
 * @return 0 on success, non-zero error code on failure
 */
int hal_timer_config(int timer_num, uint32_t freq_hz);

/**
 * Returns the HW timer current microsecond value
 *
 * @param timer_num The HW timer to read the tick value from
 *
 * @return The current microsecond value
 */
uint32_t hal_timer_read(int timer_num);

/**
 * Set the timer structure prior to use. Should not be called if the timer
 * is running. Must be called at least once prior to using timer.
 *
 * @param timer_num The number of the HW timer to configure the callback on
 * @param tmr       The timer structure to use for this timer
 * @param cb_func   The timer callback to call when the timer fires
 * @param arg       An opaque argument to provide the timer callback
 *
 * @return 0  on success, non-zero error code on failure.
 */
int hal_timer_set_cb(int timer_num, struct ble_npl_callout *tmr, ble_npl_event_fn cb_func,
                     void *arg);

/**
 * Start a timer that will expire when the timer reaches 'microsec' microseconds. If it
 * has already passed the timer callback will be called "immediately" (at
 * application context).
 *
 * @param tmr  The timer to start
 * @param microsec The absolute microsecond value to fire the timer at
 *
 * @return 0 on success, non-zero error code on failure.
 */
int hal_timer_start_at(struct ble_npl_callout *tmr, uint32_t microsec);

/**
 * Stop a currently running timer; associated callback will NOT be called
 *
 * @param tmr The timer to stop
 */
int hal_timer_stop(struct ble_npl_callout *tmr);

#endif  //  __LORA_TIMER_H__
