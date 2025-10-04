/* jump.c — the FR-2.3 jump into an application slot.
 *
 * The jump is three facts, and each has the failure mode it prevents:
 *
 *   - The application's initial stack pointer comes from the application's own vector
 *     table, never from anything the bootloader assumes. The image's first word.
 *   - The vector table must be relocated with SCB->VTOR before the branch. Without it,
 *     an interrupt would dispatch through the bootloader's table (the alias at
 *     0x00000000 still maps here) and the application would run correctly only until
 *     the first interrupt.
 *   - The jump target is the reset PC with the Thumb bit: the bit is a mode marker in
 *     the vector, not an address bit, and the branch must end with it set.
 *
 * The order inside the function: interrupts off (nothing of ours may fire after this),
 * then VTOR (still executing on the bootloader's stack, so a fault on this code would
 * dispatch from a table we just moved — acceptable, but the sequence is documented),
 * then MSP from the application, then the branch. After MSP switches there is no
 * returning: the application's reset handler re-initialises .data/.bss and runs.
 */
#include "jump.h"

#include "boot/layout.h"
#include "stm32f1xx.h"

void boot_jump(uint32_t slot) {
  const uint32_t base = boot_slot_base(slot);
  if (base == 0u) {
    for (;;) { /* not a slot: freeze rather than jump nowhere */
    }
  }

  const uint32_t app_sp = *(const volatile uint32_t *)base;
  const uint32_t app_pc = *(const volatile uint32_t *)(base + 4u);

  __disable_irq();
  SCB->VTOR = base;
  __set_MSP(app_sp);

  typedef void (*reset_fn)(void);
  reset_fn reset = (reset_fn)(app_pc | 1u);
  reset();
  for (;;) { /* the application's reset handler never returns */
  }
}