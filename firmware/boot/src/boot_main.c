/* boot_main.c — the bootloader program: decide, then jump or accept an update.
 *
 * The whole program in one paragraph. On every reset the part fetches this image's
 * vector table from 0x08000000 (the boot-0 alias you met during bring-up) and lands
 * here with nothing initialised. The bootloader reads the two metadata slips and the
 * update marker from the backup domain, runs the decision ladder (decision.h) with the
 * image verifier (boot_update.h — the slip is a promise, the flash is the evidence),
 * and then either jumps into the application (jump.h, FR-2.3) or hands the machine to
 * the update loop (boot_update.h), which is where a field update actually happens.
 *
 * This file is deliberately thin: the decision is pure and host-tested, the jump is
 * three documented facts, the update loop is its own file. What remains here is the
 * choreography and the banner, which is exactly what a reader of this file expects to
 * see.
 */
#include "boot/decision.h"
#include "boot/layout.h"
#include "boot/metadata.h"
#include "boot_loop.h"
#include "boot_uart.h"
#include "jump.h"

#define BOOT_VERSION_STRING "0.1.0"

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
  const bool update = boot_update_marker_read();

  /* The one decision, pure (Lesson 2). The update window starts open and the loop that
   * follows measures it; at this instant it has not expired. */
  const boot_outcome_t outcome = boot_decide(&slip_a, &slip_b, update, false,
                                             boot_verify_image, NULL);

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
    boot_jump(outcome.decision == BOOT_DECISION_RUN_SLOT_A ? BOOT_SLOT_A : BOOT_SLOT_B);
  }

  /* Stay in the bootloader: the update loop. It answers the framed protocol, measures
   * the idle window, and jumps to a valid app if the update never arrives — the
   * recovery state a field box should sit in. */
  boot_uart_puts("boot: update mode\r\n");
  boot_update_loop();
  for (;;) { /* not reached: the loop never returns on its own */
  }
}