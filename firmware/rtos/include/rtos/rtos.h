/* rtos.h — the RTOS abstraction the application layer is written against.
 *
 * The application core (firmware/app) never includes a FreeRTOS header. It sees only
 * this interface, which is why the application layer compiles and runs on the host
 * under a test double (FR-0.6) while the target build binds the same interface to
 * FreeRTOS. See firmware/rtos/freertos/rtos_freertos.c and tests/support/rtos_host.h.
 *
 * Every function takes the iface's `ctx` as its first argument so that an
 * implementation can be a plain struct with no global state.
 */
#ifndef RTOS_RTOS_H
#define RTOS_RTOS_H

#include <stdbool.h>
#include <stdint.h>

/* Opaque handles. The application allocates none of these and never dereferences one.
 *
 * These typedefs sit outside the extern "C" block below on purpose. A handle's
 * definition belongs to whichever implementation is linked: FreeRTOS's own types on
 * target, and a host double with C++ members under tests. Declaring the tag inside
 * extern "C" would give that class C language linkage and make a C++ definition of it
 * ill-formed, which would rule out the host double this design depends on.
 */
typedef struct rtos_mutex rtos_mutex_t;
typedef struct rtos_task rtos_task_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  RTOS_OK = 0,
  RTOS_TIMEOUT, /* the resource was not available before the timeout elapsed */
  RTOS_ERROR,   /* the resource could not be created or the call is invalid */
} rtos_status_t;

/* Task body. `arg` is whatever was passed to task_create. Must not return on target. */
typedef void (*rtos_task_fn)(void *arg);

typedef struct rtos_iface {
  void *ctx;

  /* --- mutual exclusion -------------------------------------------------------
   * Create a mutex with priority inheritance (FR-1.7 depends on this: the I2C
   * arbiter's mutex is on the critical path of every sensor and display cycle, so a
   * binary semaphore would let a low-priority holder be preempted indefinitely).
   * Returns NULL on failure. Created once at start-up, never freed: a 72 h
   * unattended run must not fragment the heap (NFR-0.7).
   */
  rtos_mutex_t *(*mutex_create)(void *ctx);

  /* Acquire `m`, blocking at most timeout_ms. On RTOS_OK, `waited` (optional,
   * may be NULL) is set to true if the caller had to block before acquiring,
   * which is how bus contention is counted for FR-0.2.
   */
  rtos_status_t (*mutex_lock)(void *ctx, rtos_mutex_t *m, uint32_t timeout_ms, bool *waited);

  /* Release `m`. Must be called by the acquiring task. */
  void (*mutex_unlock)(void *ctx, rtos_mutex_t *m);

  /* Priority of the task currently holding `m`, or 0 if unheld. Used only by the
   * FR-1.7 priority-inheritance instrumentation.
   */
  uint32_t (*mutex_holder_priority)(void *ctx, rtos_mutex_t *m);

  /* --- time -------------------------------------------------------------------
   * Monotonic milliseconds since scheduler start. Wraps at 2^32 ms (~49.7 days);
   * all comparisons must therefore be done as signed differences.
   */
  uint32_t (*now_ms)(void *ctx);

  /* Sleep the calling task. */
  void (*delay_ms)(void *ctx, uint32_t ms);

  /* --- tasks ------------------------------------------------------------------
   * `stack_words` is in words (4 bytes), as FreeRTOS counts it. Returns NULL on
   * failure. `priority` is a raw FreeRTOS priority (higher number = higher priority).
   */
  rtos_task_t *(*task_create)(void *ctx, const char *name, rtos_task_fn fn, void *arg,
                              uint32_t stack_words, uint32_t priority);

  /* Minimum free stack the task has ever had, in words. Basis of NFR-1.11. */
  uint32_t (*task_stack_high_water)(void *ctx, rtos_task_t *t);

  /* Voluntarily give up the CPU. */
  void (*yield)(void *ctx);
} rtos_iface_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RTOS_RTOS_H */
