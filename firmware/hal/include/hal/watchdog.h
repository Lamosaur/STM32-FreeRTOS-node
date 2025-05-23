/* watchdog.h — the independent watchdog interface.
 *
 * FR-0.4 requires that an independent *task* feeds the watchdog and that the same task
 * detects missed heartbeats. That requirement is why this interface is only "start"
 * and "feed": the entire decision of whether feeding is permitted lives in the
 * supervisor (firmware/app/src/supervisor.cpp), where it is host-testable, and never
 * in a timer callback or interrupt.
 *
 * "Independent" means the watchdog's timebase is not the one the tasks are scheduled
 * from: on the F103 that is the IWDG, clocked from the independent LSI oscillator, so
 * a failed HSE/PLL or a wedged tick interrupt cannot keep the watchdog quiet.
 */
#ifndef HAL_WATCHDOG_H
#define HAL_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_watchdog_iface {
  void *ctx;

  /* Start the watchdog with a timeout of `timeout_ms`. Once started it cannot be
   * stopped (IWDG semantics). Returns false if the timeout is outside what the
   * hardware can produce, or the watchdog is already running.
   */
  bool (*start)(void *ctx, uint32_t timeout_ms);

  /* Reload the counter. If the supervisor stops calling this, the MCU resets. */
  void (*feed)(void *ctx);
} hal_watchdog_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HAL_WATCHDOG_H */
