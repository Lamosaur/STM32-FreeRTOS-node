/* FreeRTOSConfig.h — kernel configuration for the Phase 0 greenhouse node.
 *
 * Target: STM32F103C8T6, Cortex-M3, 72 MHz SYSCLK, 64 KB flash, 20 KB RAM, no FPU.
 *
 * Every number here is either fixed by the hardware, fixed by design-spec.md, or
 * derived from a RAM budget that is written out below. Anything that is a measurement
 * rather than a decision says so and names the requirement that will replace it.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* The device header is pulled in the way the upstream ARM_CM3 demos do it, so that
 * configCPU_CLOCK_HZ reads the *actual* SystemCoreClock (set by hal_stm32f1_init()
 * from the real RCC configuration) instead of a repeated literal, and so that
 * configPRIO_BITS comes from __NVIC_PRIO_BITS rather than from a fourth hand-copied
 * "4". Getting that constant wrong is how a priority-inheritance system silently
 * stops masking interrupts correctly.
 *
 * The guard keeps this header usable by a compiler that has no __GNUC__ (IAR/Keil).
 */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  #include "stm32f1xx.h"
#endif

/* ---------------------------------------------------------------------------
 * Scheduler
 * ---------------------------------------------------------------------------
 * Preemption with an equal-priority round-robin (time slicing) is what makes the
 * scheduler/task architecture of design-spec.md §5.1 measurable rather than
 * aspirational: the sensor and app tasks share priority 3, so time slicing decides
 * between them, and the display task at priority 1 gets whatever is left.
 */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                       ((TickType_t)1000)

/* 8 entries for an application that uses 1..4 (app/board.h task_priority). Headroom is
 * deliberate: configMAX_PRIORITIES costs only a pointer per level in .bss, and this
 * file is shared with the node application that supersedes the bring-up image. */
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 8
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1

/* The kernel carries only what this node uses. FreeRTOS code that is not referenced is
 * dropped by --gc-sections anyway, so these settings are about keeping the API surface
 * honest rather than about flash size alone. Event groups stay on because the
 * supervisor's heartbeat aggregation is the obvious use for them; stream buffers and
 * co-routines are off because nothing in Phase 0 has a use for them. */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_EVENT_GROUPS                  1
#define configUSE_STREAM_BUFFERS                0
#define configUSE_CO_ROUTINES                   0
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               0

/* Every FreeRTOS API this firmware uses has to be enabled here, because the INCLUDE_
 * default is 0 and the function is then compiled out of the kernel entirely. The
 * consequence of forgetting one is an undefined reference at link time, which says
 * nothing about which option is missing.
 *
 *   xSemaphoreGetMutexHolder : FR-1.7 priority-inheritance instrumentation, read by
 *                              rtos_freertos.c mutex_holder_priority
 *   uxTaskGetStackHighWaterMark : NFR-1.11 stack headroom
 *   xTaskGetSchedulerState   : lets mutex_lock behave before the scheduler starts
 *   uxTaskPriorityGet        : the other half of the FR-1.7 reading
 */
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelete                     1

/* ---------------------------------------------------------------------------
 * Instrumentation required by the measurement plan
 * ---------------------------------------------------------------------------
 * configUSE_TRACE_FACILITY          -> uxTaskGetStackHighWaterMark() / vTaskList(),
 *                                      the basis of NFR-1.11 (25% stack headroom).
 * configGENERATE_RUN_TIME_STATS     -> vTaskGetRunTimeStats(), NFR-1.10 (<60% CPU).
 * configUSE_STATS_FORMATTING_FUNCTIONS -> the sprintf-based reporters themselves.
 *                                      Referenced or not, --gc-sections drops them,
 *                                      so enabling this costs nothing until used.
 */
#define configUSE_TRACE_FACILITY                1
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
#define configRUN_TIME_COUNTER_TYPE             uint32_t

/* The run-time counter is the DWT cycle counter, not a peripheral timer.
 *
 * Two reasons. First, resolution: 72 MHz is far better than the ~1 kHz a spare timer
 * would give, so short control-task bursts are actually attributable. Second, it
 * leaves TIM1/TIM2/TIM3 unclaimed for Phase 1's timer-triggered ADC and PWM output.
 * The counter wraps every 2^32/72e6 = 59.6 s, which is harmless because FreeRTOS
 * accumulates per-task time as an unsigned *difference* at each context switch
 * (tasks.c: pxCurrentTCB->ulRunTimeCounter += ulTotalRunTime - ulTaskSwitchedInTime),
 * so a wrap inside a slice is accounted correctly as long as no single slice lasts
 * longer than 59.6 s. It is also what hal_i2c_stm32f1.c's delay_us() counts.
 */
void nodeRunTimeStatsInit(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  nodeRunTimeStatsInit()
#define portGET_RUN_TIME_COUNTER_VALUE()          (DWT->CYCCNT)

/* ---------------------------------------------------------------------------
 * Timers: off
 * ---------------------------------------------------------------------------
 * configUSE_TIMERS 0 means no daemon task, no timer command queue, and no timer
 * service callbacks — about 1 KB of heap and one stack back.
 *
 * This is a deliberate fit for the design rather than a saving: FR-0.4 requires the
 * *supervisor task* to feed the watchdog and to own the heartbeat decision
 * (design-spec.md §5.3 decision #4 explicitly rejects a timer callback for it), and
 * everything else Phase 0 schedules is periodic work already expressed as a task.
 * Turning timers on later means adding configTIMER_TASK_PRIORITY,
 * configTIMER_QUEUE_LENGTH and configTIMER_TASK_STACK_DEPTH, which FreeRTOS only
 * demands when configUSE_TIMERS is 1.
 */
#define configUSE_TIMERS                        0

/* ---------------------------------------------------------------------------
 * Memory
 * ---------------------------------------------------------------------------
 * heap_4 over 10 KB, and this is the number that has to be argued because RAM is the
 * scarce resource (20 KB total):
 *
 *     five task stacks (app/board.h task_stack_words, words)
 *         192 + 320 + 320 + 384 + 384 = 1600 words = 6400 B
 *     five TCBs (TCB_t ~ 100 B on this port)                ~  500 B
 *     idle task stack (configMINIMAL_STACK_SIZE) + TCB     ~  650 B
 *     the arbiter mutex and a queue-less binary semaphore    ~  200 B
 *                                                          ---------
 *                                                          ~ 7750 B
 *
 * 10240 B leaves roughly 2.4 KB of slack, which is the room the stack right-sizing in
 * NFR-1.11 is allowed to grow into. Static allocation is off (heap_4 is the design's
 * choice); dynamic allocation must be on because configUSE_STATS_FORMATTING_FUNCTIONS
 * is only legal with it. Objects are created once at start-up and never deleted, so
 * fragmentation is bounded by construction (NFR-0.7).
 */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(10 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP        0

/* ---------------------------------------------------------------------------
 * Failure detection and hooks
 * ---------------------------------------------------------------------------
 * configCHECK_FOR_STACK_OVERFLOW 2 is the mark-and-check version: it writes a known
 * pattern at the low end of every task stack and validates it at each switch, so it
 * catches a stack that was written past, not merely one that happened to dip. That
 * plus uxTaskGetStackHighWaterMark() is what NFR-1.11 is measured with.
 *
 * The three hooks are implemented in app/src/main_target.cpp, not here and not in the
 * port layer, because what they must do is application policy: the idle hook toggles
 * the board's status LED through the same GPIO interface the rest of the application
 * uses, and the two failure hooks latch with interrupts off so that the *independent*
 * watchdog (FR-0.4) resets the node. An unattended node (NFR-0.7) that has run out of
 * heap should come back, not sit in a debugger-less infinite loop.
 */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_IDLE_HOOK                     1
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TICK_HOOK                     0

/* ---------------------------------------------------------------------------
 * Newlib
 * ---------------------------------------------------------------------------
 * configUSE_NEWLIB_REENTRANT stays 0: a full struct _reent per task would cost
 * hundreds of bytes each out of the same 10 KB heap, which this node cannot afford.
 * The consequence is that newlib's static reentrancy structure is shared, so all
 * stdio-style formatting is serialised by the application's own log mutex (see
 * UartLog in app/src/main_target.cpp). That is the correct trade for a node whose
 * only output is one UART link.
 */
#define configUSE_NEWLIB_REENTRANT              0

/* ---------------------------------------------------------------------------
 * Cortex-M3 interrupt priorities and the vector-table bindings
 * ---------------------------------------------------------------------------
 * Cortex-M priorities are inverted: a *numerically smaller* value is more urgent.
 * FreeRTOS masks interrupts at or below (numerically >=) the syscall priority, so any
 * ISR that calls a ...FromISR() API must be configured at 5..15. Interrupts that never
 * call FreeRTOS stay free to use anything.
 *
 * 5 is the conventional choice and the highest value that still leaves the kernel's
 * own PendSV/SysTick (lowest priority, 15) below every application interrupt.
 * configLIBRARY_* are the human-readable values; the shifted forms are what actually
 * goes into the NVIC/BASEPRI register on a 4-priority-bit part.
 */
#define configPRIO_BITS                          __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY          15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* The startup file's vector table is weak, so the port's handlers take its slots by
 * name. Without these three the image still links — the weak defaults are an infinite
 * loop — and simply never schedules. */
#define xPortPendSVHandler                      PendSV_Handler
#define vPortSVCHandler                         SVC_Handler
#define xPortSysTickHandler                     SysTick_Handler

/* ---------------------------------------------------------------------------
 * Asserts
 * ---------------------------------------------------------------------------
 * configASSERT() is live in a release image on purpose: it is the only check that
 * catches "a ...FromISR() API called from an interrupt at the wrong priority" and
 * "xTaskCreate() before the scheduler started" at the moment they happen instead of
 * three days into an unattended run. The handler (rtos_freertos.c) breaks into an
 * attached debugger if there is one and otherwise latches with interrupts disabled so
 * the independent watchdog resets the node.
 */
void node_rtos_config_assert_failed(const char *file, int line);
#define configASSERT(x)                                                     \
    do {                                                                    \
        if ((x) == 0) {                                                     \
            node_rtos_config_assert_failed(__FILE__, __LINE__);             \
        }                                                                   \
    } while (0)

#endif /* FREERTOS_CONFIG_H */
