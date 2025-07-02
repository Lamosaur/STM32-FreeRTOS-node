/* freertos_hooks.c — the application hooks FreeRTOS calls from inside the kernel.
 *
 * These live in the RTOS layer rather than in hal/ because they *are* FreeRTOS
 * callbacks: they are reached from the kernel's own code paths, not from anything this
 * project calls. Keeping them here also keeps the layering honest, since the failure
 * hooks must not depend on the I2C bus or the display being usable at the moment they
 * fire — that is rather the point of them.
 *
 * configUSE_IDLE_HOOK, configUSE_MALLOC_FAILED_HOOK and configCHECK_FOR_STACK_OVERFLOW
 * are all enabled in FreeRTOSConfig.h, and each one obliges the application to provide
 * the matching function below. Missing any of them is a link error, which is how these
 * came to exist.
 */
#include "FreeRTOS.h"
#include "task.h"

/* Called from the idle task, which runs only when nothing else can.
 *
 * Deliberately empty. The obvious temptation is to blink the status LED here, but the
 * supervisor task already owns that signal, and moving it would silently weaken what the
 * LED means: "the idle task was scheduled" is true on a node whose every real task has
 * stopped, whereas the supervisor's toggle only happens if the supervisor itself is
 * running.
 *
 * Kept as a real definition rather than disabled, because it is the hook a future
 * tickless-idle mode would enter through.
 */
void vApplicationIdleHook(void) {}

/* The node cannot continue: a task's stack overflowed the mark the kernel planted, or
 * the heap is exhausted. Both are unrecoverable in place, and both are exactly the case
 * the independent watchdog exists for.
 *
 * So the response is to stop doing anything that could mask the fault — interrupts off,
 * spin — and let the IWDG reset the board. The next boot reports `reset watchdog` and
 * the boot count, which is how an unattended run (NFR-0.7) records that this happened
 * rather than pretending it did not.
 *
 * Task names are accepted and ignored: the UART cannot be trusted here (the hook may
 * have been called with the port's own mutex held, or from an ISR-adjacent context), and
 * the reset cause plus the boot counter carry the information that survives.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
  (void)task;
  (void)task_name;

  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
