//  Timer Functions ported from Mynewt to NimBLE Porting Layer
#include <assert.h>
#include "nimble_npl.h"      //  For NimBLE Porting Layer (timer functions)
#include "node/lora_timer.h"

//  Event Queue containing Events to be processed, defined in demo.c.  TODO: Move to header file.
extern struct ble_npl_eventq event_queue;

/// Initialise a timer. Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_init
void os_cputime_timer_init(
    struct ble_npl_callout *timer,  //  The timer to initialize. Cannot be NULL.
    ble_npl_event_fn *f,            //  The timer callback function. Cannot be NULL.
    void *arg)                      //  Pointer to data object to pass to timer.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);
    assert(f != NULL);

    //  Init the Callout Timer with the Callback Function
    ble_npl_callout_init(
        timer,         //  Callout Timer
        &event_queue,  //  Event Queue that will handle the Callout upon timeout
        f,             //  Callback Function
        arg            //  Argument to be passed to Callback Function
    );
}

/// Stops a timer from running.  Can be called even if timer is not running.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_stop
void os_cputime_timer_stop(
    struct ble_npl_callout *timer)  //  Pointer to timer to stop. Cannot be NULL.
{
    //  Implement with Callout Functions from NimBLE Porting Layer
    assert(timer != NULL);

    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(timer)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(timer);
    }
}

/// Sets a timer that will expire ‘usecs’ microseconds from the current time.
/// NOTE: This must be called when the timer is stopped.
/// Based on https://mynewt.apache.org/latest/os/core_os/cputime/os_cputime.html#c.os_cputime_timer_relative
void os_cputime_timer_relative(
    struct ble_npl_callout *timer,  //  Pointer to timer. Cannot be NULL.
    uint32_t microsecs)             //  The number of microseconds from now at which the timer will expire.
{
    //  Implement with Callout Functions from NimBLE Porting Layer.
    //  Assume that Callout Timer has been stopped.
    assert(timer != NULL);

    //  Convert microseconds to ticks
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        microsecs / 1000  //  Duration in milliseconds
    );

    //  Wait at least 1 tick
    if (ticks == 0) { ticks = 1; }

    //  Trigger the Callout Timer after the elapsed ticks
    ble_npl_error_t rc = ble_npl_callout_reset(
        timer,  //  Callout Timer
        ticks   //  Number of ticks
    );
    assert(rc == 0);
}

/// Get the current OS time in ticks
ble_npl_time_t os_time_get(void) {
    //  Get current time in system ticks
    ble_npl_time_t ticks = ble_npl_time_get();

    return ticks;
}

/// Converts OS ticks to milliseconds. Return 0 for success.
int os_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms) {
    assert(out_ms != NULL);
    
    //  Convert ticks to milliseconds
    uint32_t millisec = ble_npl_time_ticks_to_ms32(ticks);
    *out_ms = millisec;
    return 0;
}

/// Get the current time in microseconds
uint32_t timer_get_current_time(void)
{
    //  Get current time in system ticks
    ble_npl_time_t ticks = ble_npl_time_get();

    //  Convert ticks to microseconds
    uint32_t millisec = ble_npl_time_ticks_to_ms32(ticks);
    return millisec * 1000;
}

///////////////////////////////////////////////////////////////////////////////
//  HAL Timer ported from Mynewt. We simulate with NimBLE Porting Layer.

/// Init the HAL Timer
void lora_bsp_enable_mac_timer(void) {
    //  Nothing to init
}

/**
 * Un-initialize a HW timer.
 *
 * @param timer_num The number of the HW timer to un-initialize
 */
int hal_timer_deinit(int timer_num) {
    assert(timer_num == 0);
    return 0;
}

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
int hal_timer_config(int timer_num, uint32_t freq_hz) {
    assert(timer_num == 0);
    assert(freq_hz == 1000000);
    return 0;
}

/**
 * Returns the HW timer current microsecond value
 *
 * @param timer_num The HW timer to read the tick value from
 *
 * @return The current microsecond value
 */
uint32_t hal_timer_read(int timer_num) {
    assert(timer_num == 0);

    //  Get current time in system ticks
    ble_npl_time_t ticks = ble_npl_time_get();

    //  Convert ticks to microseconds
    uint32_t millisec = ble_npl_time_ticks_to_ms32(ticks);
    return millisec * 1000;
}

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
    void *arg) {
    assert(timer_num == 0);
    assert(tmr != NULL);

    //  Init the Callout Timer with the Callback Function
    ble_npl_callout_init(
        tmr,           //  Callout Timer
        &event_queue,  //  Event Queue that will handle the Callout upon timeout
        cb_func,       //  Callback Function
        arg            //  Argument to be passed to Callback Function
    );
    return 0;
}

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
int hal_timer_start_at(struct ble_npl_callout *tmr, uint32_t microsec) {
    assert(tmr != NULL);

    //  Convert absolute microseconds to ticks
    ble_npl_time_t ticks = ble_npl_time_ms_to_ticks32(
        microsec / 1000  //  Duration in milliseconds
    );

    //  Get current time in ticks
    ble_npl_time_t current_ticks = ble_npl_time_get();

    //   Get relative ticks to wait
    ble_npl_time_t ticks_to_wait = (ticks > current_ticks)
        ? (ticks - current_ticks)
        : 1;  //  Wait at least 1 tick

    //  Trigger the Callout Timer after the elapsed ticks
    ble_npl_error_t rc = ble_npl_callout_reset(
        tmr,           //  Callout Timer
        ticks_to_wait  //  Number of ticks
    );
    assert(rc == 0);
    return 0;
}

/**
 * Stop a currently running timer; associated callback will NOT be called
 *
 * @param tmr The timer to stop
 */
int hal_timer_stop(struct ble_npl_callout *tmr) {
    assert(tmr != NULL);

    //  If Callout Timer is still running...
    if (ble_npl_callout_is_active(tmr)) {
        //  Stop the Callout Timer
        ble_npl_callout_stop(tmr);
    }
    return 0;
}
