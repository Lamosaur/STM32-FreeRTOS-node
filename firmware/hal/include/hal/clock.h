/* clock.h — a monotonic microsecond timestamp, for the measurements the plan asks for.
 *
 * Why this interface exists at all, since adding a HAL interface is the kind of thing
 * that should be argued for rather than assumed: NFR-1.8 requires control-loop period
 * jitter and NFR-1.9 requires ISR-to-task latency, both "measured and reported, not
 * assumed", and app/timing.h records the series for both. The application layer is
 * target-independent (FR-0.6) and may not include a device header, so a DWT cycle count
 * read in the control task is not available to it, and the RTOS interface's millisecond
 * clock is too coarse: a 1 ms timestamp turns a jitter figure that lives in the tens of
 * microseconds into a value that is 0 or 1000 and nothing else. The remaining option —
 * measuring through the RTOS's clock and calling the result jitter — would produce a
 * number that describes the tick rather than the loop.
 *
 * One entry point, deliberately. A clock that also exposed a rate, an epoch, or a way to
 * convert its own units would invite arithmetic in the middle of the measurement, and the
 * only operations the callers need are "read it now" and "subtract two of them".
 *
 * The implementation is the DWT cycle counter divided down by the core clock
 * (hal_stm32f1.c). Two consequences, both documented there and both harmless here:
 *
 *   - Resolution is one core cycle (13.9 ns at 72 MHz) rounded down to a microsecond, and
 *     the round-down is per read rather than accumulated, so two reads 100 ms apart are
 *     within a microsecond of 100000.
 *   - The counter wraps, at 2^32 us (about 71.6 minutes). Every consumer compares two
 *     timestamps by unsigned subtraction, which is exact across a wrap as long as the two
 *     are less than one wrap apart — a control period of 1 s against a 71 minute wrap has
 *     three orders of magnitude of margin, and the shortest measurement here is the
 *     minimum on-time of 50 ms, which has six.
 *
 * It is optional in the sense that matters: nothing in the application layer fails
 * without it, and NodeHardware treats a missing clock as "the timing series stay empty"
 * rather than as an error. What is lost is the NFR-1.8/NFR-1.9 figures, which is why the
 * target always supplies one.
 */
#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_clock_iface {
  void *ctx;

  /* Microseconds since an arbitrary, fixed origin (the implementation's counter start).
   *
   * Monotonic and free-running: it must not be reset by anything the application does, and
   * it must be safe to call from an interrupt as well as from a task. Unsigned subtraction
   * of two readings gives the elapsed interval.
   */
  uint32_t (*now_us)(void *ctx);
} hal_clock_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_CLOCK_H */
