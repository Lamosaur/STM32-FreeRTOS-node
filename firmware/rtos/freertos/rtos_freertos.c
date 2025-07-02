/* rtos_freertos.c — the target binding of rtos_iface_t to FreeRTOS (V11.3.1).
 *
 * The application layer never sees a FreeRTOS header: it sees rtos/rtos.h, which this
 * file implements. The host build binds the same interface to a std::timed_mutex-backed
 * double (tests/support/fake_rtos.h), which is what makes FR-0.6 possible.
 *
 * Two ownership rules shape the code below:
 *
 *   - The wrapper objects (struct rtos_mutex / struct rtos_task) come from fixed static
 *     pools, so the *abstraction* never allocates. The FreeRTOS objects themselves (the
 *     mutex's queue, a task's TCB and stack) still come from heap_4, which is exactly
 *     what "created once at start-up, never freed" (rtos/rtos.h) is asking for: no
 *     run-time churn, so nothing to fragment over a 72 h unattended run (NFR-0.7).
 *
 *   - Handles are valid forever. Nothing here frees a mutex or deletes a task, and a
 *     pool slot is never recycled, which means a handle cannot go stale underneath the
 *     application.
 */

#include "rtos/rtos.h"

#include <stddef.h> /* size_t: the pools below are indexed by count, not by handle */

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "stm32f1xx.h" /* DWT (run-time stats time base), CoreDebug (assert handler) */

/* ---------------------------------------------------------------------------
 * Fixed pools
 * ---------------------------------------------------------------------------
 * Eight of each is generous for five tasks (app/board.h TaskId) plus the I2C arbiter
 * mutex and the UART log mutex. Exhaustion returns NULL/false rather than allocating,
 * which is the failure the caller can see and handle.
 */
#define NODE_RTOS_MUTEX_POOL 8U
#define NODE_RTOS_TASK_POOL 8U

struct rtos_mutex {
  SemaphoreHandle_t handle;
};

struct rtos_task {
  TaskHandle_t handle;
};

static struct rtos_mutex s_mutex_pool[NODE_RTOS_MUTEX_POOL];
static size_t s_mutex_used = 0U;
static struct rtos_task s_task_pool[NODE_RTOS_TASK_POOL];
static size_t s_task_used = 0U;

/* ---------------------------------------------------------------------------
 * Tick conversion
 * ---------------------------------------------------------------------------
 * pdMS_TO_TICKS() multiplies before it divides, so on a 32-bit tick it overflows for
 * millisecond values above roughly 4.29e6/configTICK_RATE_HZ. The two helpers below
 * clamp instead of letting that wrap into a near-zero timeout, and they differ because
 * the clamped value means something different to each caller:
 *
 *   - a lock wait that would exceed ten minutes is treated as "forever"
 *     (portMAX_DELAY), because a mutex held that long is a wedged system, not a slow
 *     one;
 *   - a *delay* must never become portMAX_DELAY, which means "suspend indefinitely",
 *     so an absurdly long delay is clamped just short of it.
 */
#define NODE_RTOS_LOCK_FOREVER_MS (10U * 60U * 1000U)
#define NODE_RTOS_MAX_DELAY_MS (60U * 60U * 1000U)

static TickType_t lock_timeout_ticks(uint32_t timeout_ms) {
  if (timeout_ms == 0U) {
    return 0;
  }
  if (timeout_ms >= NODE_RTOS_LOCK_FOREVER_MS) {
    return portMAX_DELAY;
  }
  return pdMS_TO_TICKS(timeout_ms);
}

/* ---------------------------------------------------------------------------
 * Mutual exclusion
 * ---------------------------------------------------------------------------
 */

static rtos_mutex_t *mutex_create_cb(void *ctx) {
  (void)ctx;

  if (s_mutex_used >= NODE_RTOS_MUTEX_POOL) {
    return NULL;
  }

  /* xSemaphoreCreateMutex() and not xSemaphoreCreateBinary(): it creates a queue of
   * type queueQUEUE_IS_MUTEX, which is what makes queue.c call xTaskPriorityInherit()
   * on the holder when a higher-priority task blocks on it, and
   * xTaskPriorityDisinherit() on release. FR-1.7 is a claim about that property on the
   * I2C arbiter's mutex, so a binary semaphore here would quietly invalidate the
   * measurement it exists to produce.
   */
  struct rtos_mutex *m = &s_mutex_pool[s_mutex_used];
  m->handle = xSemaphoreCreateMutex();
  if (m->handle == NULL) {
    return NULL;
  }

  ++s_mutex_used;
  return m;
}

static rtos_status_t mutex_lock_cb(void *ctx, rtos_mutex_t *m, uint32_t timeout_ms,
                                   bool *waited) {
  (void)ctx;

  if (m == NULL || m->handle == NULL) {
    return RTOS_ERROR;
  }
  if (waited != NULL) {
    *waited = false;
  }

  /* "Did the caller have to block" is answered by *trying* the mutex first rather than
   * by comparing xTaskGetTickCount() before and after. At a 1 ms tick a tick
   * comparison cannot see a wait shorter than a tick, and the interesting wait here —
   * behind another task's I2C transfer — is often exactly that short. Since FR-0.2's
   * contention counter and its stress run are the point of the flag, under-reporting
   * would be worse than the extra call.
   *
   * The try is side-effect free with respect to FR-1.7: xQueueSemaphoreTake() applies
   * xTaskPriorityInherit() only in the branch where the task actually enters the
   * Blocked state, and a zero-tick wait returns errQUEUE_EMPTY before reaching it. So a
   * failed try cannot perturb the holder's priority.
   *
   * The one imprecision is a release that lands between the try and the blocking call:
   * that acquisition is counted as contended, which is the conservative direction.
   */
  if (xSemaphoreTake(m->handle, 0) == pdTRUE) {
    return RTOS_OK;
  }

  /* Blocking before the scheduler runs is undefined in FreeRTOS (nothing can block on
   * a tick that is not ticking), so the pre-scheduler case degrades to the try above
   * and reports a timeout rather than deadlocking the bring-up path. */
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return RTOS_TIMEOUT;
  }

  if (xSemaphoreTake(m->handle, lock_timeout_ticks(timeout_ms)) != pdTRUE) {
    return RTOS_TIMEOUT;
  }

  /* Reaching here means the blocking call was needed: the first take failed, so the
   * mutex was held by someone else at that instant. */
  if (waited != NULL) {
    *waited = true;
  }
  return RTOS_OK;
}

static void mutex_unlock_cb(void *ctx, rtos_mutex_t *m) {
  (void)ctx;

  if (m == NULL || m->handle == NULL) {
    return;
  }
  (void)xSemaphoreGive(m->handle);
}

static uint32_t mutex_holder_priority_cb(void *ctx, rtos_mutex_t *m) {
  (void)ctx;

  if (m == NULL || m->handle == NULL) {
    return 0U;
  }

  /* The value the FR-1.7 instrumentation compares: while a low-priority task holds the
   * arbiter and a high-priority task is blocked on it, this returns the *raised*
   * priority, which is the observable evidence that inheritance happened. 0 means
   * unheld (or held by an ISR-less context), as documented in rtos/rtos.h. */
  TaskHandle_t holder = xSemaphoreGetMutexHolder(m->handle);
  if (holder == NULL) {
    return 0U;
  }
  return (uint32_t)uxTaskPriorityGet(holder);
}

/* ---------------------------------------------------------------------------
 * Time
 * ---------------------------------------------------------------------------
 */

static uint32_t now_ms_cb(void *ctx) {
  (void)ctx;

  /* Monotonic since scheduler start; 0 before it. The tick counter is 32-bit and wraps
   * every ~49.7 days, which is why rtos/rtos.h requires comparisons to be signed
   * differences — the same rule FreeRTOS itself applies to tick values. */
  return (uint32_t)xTaskGetTickCount();
}

static void delay_ms_cb(void *ctx, uint32_t ms) {
  (void)ctx;

  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
    return;
  }
  if (ms >= NODE_RTOS_MAX_DELAY_MS) {
    vTaskDelay(portMAX_DELAY - 1U);
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(ms));
}

static void yield_cb(void *ctx) {
  (void)ctx;
  taskYIELD();
}

/* ---------------------------------------------------------------------------
 * Tasks
 * ---------------------------------------------------------------------------
 */

static rtos_task_t *task_create_cb(void *ctx, const char *name, rtos_task_fn fn, void *arg,
                                   uint32_t stack_words, uint32_t priority) {
  (void)ctx;

  if (fn == NULL || s_task_used >= NODE_RTOS_TASK_POOL) {
    return NULL;
  }
  if (priority == 0U || priority >= (uint32_t)configMAX_PRIORITIES) {
    /* Priority 0 belongs to the idle task, and a priority above configMAX_PRIORITIES
     * would be silently wrapped by the scheduler rather than refused. Refused is the
     * honest behaviour and is what rtos/rtos.h's "NULL on failure" contract is for. */
    return NULL;
  }

  struct rtos_task *t = &s_task_pool[s_task_used];

  /* Stack size is counted in words, as rtos/rtos.h states, which is also how FreeRTOS
   * counts it (configSTACK_DEPTH_TYPE defaults to StackType_t). */
  if (xTaskCreate(fn, name, (configSTACK_DEPTH_TYPE)stack_words, arg, (UBaseType_t)priority,
                  &t->handle) != pdPASS) {
    return NULL;
  }

  ++s_task_used;
  return t;
}

static uint32_t task_stack_high_water_cb(void *ctx, rtos_task_t *t) {
  (void)ctx;

  if (t == NULL || t->handle == NULL) {
    return 0U;
  }

  /* Minimum free stack in words since the task started. NFR-1.11's 25% headroom is
   * this number against the size the task was created with. */
  return (uint32_t)uxTaskGetStackHighWaterMark(t->handle);
}

/* ---------------------------------------------------------------------------
 * Interface instance and helpers used by the rest of the target build
 * ---------------------------------------------------------------------------
 */

static rtos_iface_t s_iface = {
    .ctx = NULL,
    .mutex_create = mutex_create_cb,
    .mutex_lock = mutex_lock_cb,
    .mutex_unlock = mutex_unlock_cb,
    .mutex_holder_priority = mutex_holder_priority_cb,
    .now_ms = now_ms_cb,
    .delay_ms = delay_ms_cb,
    .task_create = task_create_cb,
    .task_stack_high_water = task_stack_high_water_cb,
    .yield = yield_cb,
};

/* rtos/rtos.h declares the interface but no factory, because the host double exposes
 * its instance differently (FakeRtos::iface()). The target image therefore declares
 * this accessor itself; see the note in the Phase 0 report. */
rtos_iface_t *rtos_freertos_iface(void) {
  return &s_iface;
}

/* Run-time stats time base, named by portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() in
 * FreeRTOSConfig.h. Called from vTaskStartScheduler(), i.e. before any task runs.
 *
 * The DWT cycle counter is used instead of a spare TIM because it costs no peripheral
 * (TIM1/TIM2/TIM3 stay free for Phase 1's ADC trigger and PWM) and because 72 MHz
 * resolution attributes short task bursts that a ~1 kHz timer would smear. See the
 * comment in FreeRTOSConfig.h for why its 59.6 s wrap is not a problem.
 */
void nodeRunTimeStatsInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* configASSERT() target. Deliberately not a printf: this can fire with interrupts
 * masked, from an ISR, or before the UART exists.
 *
 * With a debugger attached it stops at a breakpoint, which is where the assert wants to
 * be looked at. Without one, __BKPT would escalate to a HardFault and a reset with no
 * evidence at all, so instead the CPU latches with interrupts disabled and lets the
 * independent watchdog (FR-0.4) reset the node: an unattended node (NFR-0.7) that trips
 * an assert has to come back by itself.
 */
void node_rtos_config_assert_failed(const char *file, int line) {
  (void)file;
  (void)line;

  portDISABLE_INTERRUPTS();

  if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
    __BKPT(0);
  }

  for (;;) {
    /* Latch. The IWDG stops being fed and resets the node. */
  }
}
