/* boot_main.c — the bootloader program: decide, then jump or accept an update.
 *
 * The whole program in one paragraph. On every reset the part fetches this image's
 * vector table from 0x08000000 (the boot-0 alias you met during bring-up) and lands
 * here with nothing initialised. The bootloader reads the two metadata slips and the
 * update marker from the backup domain, runs the decision ladder (decision.h) with a
 * verifier that hashes the candidate image right out of flash — the slip is a promise,
 * the flash is the evidence, from Lesson 2 — and then either jumps into the application
 * or stays. The stay path is the update loop; the framed protocol is the next lesson,
 * so this revision of the stay path reports and waits.
 *
 * The jump (FR-2.3) is the part that makes an application at an offset work, and it is
 * three facts: the application's initial stack pointer comes from the application's own
 * vector table, not from anything the bootloader knows; the vector table itself must be
 * relocated with SCB->VTOR, or the first interrupt the application sees dispatching from
 * the bootloader's table; and the jump target is the reset PC with the Thumb bit set.
 */
#include "stm32f1xx.h"

#include "boot/crc.h"
#include "boot/decision.h"
#include "boot/layout.h"
#include "boot/metadata.h"
#include "boot/update.h"
#include "boot_uart.h"

#define BOOT_VERSION_STRING "0.1.0"

/* The verifier handed to boot_decide: hash the slot's live flash contents and compare
 * with what the slip promises. This is the "flash is the evidence" step, and the only
 * input to the decision that reads hardware. */
static bool verify_image(const boot_metadata_t *slip, void *ctx) {
  (void)ctx;
  const uint32_t base = boot_slot_base(slip->active_slot);
  if (base == 0u) {
    return false;
  }
  return boot_crc32((const uint8_t *)base, slip->image_size) == slip->image_crc32;
}

static bool update_requested(void) {
  /* Reading a backup register needs the BKP clock on the APB1 bus; writing also needs
   * the write-protect lift (PWR_CR.DBP), but the bootloader only reads. Reading a
   * clock-gated register on this part returns zeros, so the enable is not optional. */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  /* Let the bus come up; the application's own backup-domain code does the equivalent. */
  (void)RCC->APB1ENR;
  return BKP->DR3 == BOOT_UPDATE_MARKER;
}

static void jump_to_application(uint32_t slot) {
  const uint32_t base = boot_slot_base(slot);
  /* The application's vector table is its image's first words: [0] the initial SP,
   * [1] the reset PC. Both are read from the slot, never assumed. */
  const uint32_t app_sp = *(const volatile uint32_t *)base;
  const uint32_t app_pc = *(const volatile uint32_t *)(base + 4u);

  /* Nothing in the bootloader must fire after this point. */
  __disable_irq();

  /* FR-2.3: relocate the vector table so the application's interrupts dispatch from
   * ITS table. Without this, an interrupt would dispatch through the bootloader's table
   * (the alias at 0x00000000 still maps here) and the application would run correctly
   * only until the first interrupt.
   *
   * The order matters: VTOR is set before the stack is switched, because the code still
   * executing (this function) is the bootloader's and may fault its own way out. */
  SCB->VTOR = base;

  /* The application's stack pointer, loaded from the application, and never returned
   * from: from here the machine belongs to the app. */
  __set_MSP(app_sp);

  /* The reset PC has the Thumb bit set in flash; the cast drops it and the call sets it
   * again, which is how the hardware knows the target is Thumb (it always is here). */
  typedef void (*reset_fn)(void);
  reset_fn reset = (reset_fn)(app_pc & ~1u);
  reset();
  for (;;) { /* not reached: the application's reset handler never returns */
  }
}

static const char *reason_text(boot_decision_reason_t reason) {
  switch (reason) {
    case BOOT_REASON_UPDATE_REQUESTED: return "update requested";
    case BOOT_REASON_NEWEST_SLOT:      return "newest slot verified";
    case BOOT_REASON_ROLLBACK_SLOT:    return "rollback slot verified";
    case BOOT_REASON_NO_VALID_SLIP:    return "no valid metadata";
    case BOOT_REASON_NO_VALID_IMAGE:   return "no image verified";
  }
  return "?";
}

int main(void) {
  boot_uart_init();
  boot_uart_puts("boot " BOOT_VERSION_STRING "\r\n");

  /* The two slips, read whole from flash. Struct assignment rather than memcpy keeps
   * the image free of newlib; flash reads as ordinary memory on this part. */
  const boot_metadata_t slip_a = *(const boot_metadata_t *)BOOT_METADATA_A_BASE;
  const boot_metadata_t slip_b = *(const boot_metadata_t *)BOOT_METADATA_B_BASE;
  const bool update = update_requested();

  /* The one decision, pure (Lesson 2). The update window starts open and nothing here
   * services the UART yet, so window_expired stays false until the update loop exists. */
  const boot_outcome_t outcome = boot_decide(&slip_a, &slip_b, update, false,
                                             verify_image, NULL);

  boot_uart_puts("boot reason: ");
  boot_uart_puts(reason_text(outcome.reason));
  boot_uart_puts("\r\n");

  if (outcome.decision == BOOT_DECISION_RUN_SLOT_A ||
      outcome.decision == BOOT_DECISION_RUN_SLOT_B) {
    const char *slot_name =
        (outcome.decision == BOOT_DECISION_RUN_SLOT_A) ? "A" : "B";
    boot_uart_puts("boot: running slot ");
    boot_uart_puts(slot_name);
    boot_uart_puts("\r\n");
    jump_to_application((outcome.decision == BOOT_DECISION_RUN_SLOT_A) ? BOOT_SLOT_A
                                                                       : BOOT_SLOT_B);
  }

  /* Stay in the bootloader. The framed update protocol is Lesson 4; until it exists the
   * stay path is a visible stub so a board in this state says where it is. */
  boot_uart_puts("boot: update mode (protocol to follow)\r\n");
  for (;;) {
  }
}