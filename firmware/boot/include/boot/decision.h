/* decision.h — the boot decision (FR-2.2), as a pure function over gathered inputs.
 *
 * The bootloader's only real job is to answer "what do I run?", and the answer has to be
 * auditable, so it lives here as a plain function over the inputs the bootloader's main()
 * gathers: the two metadata slips read from flash, whether an update was requested (the
 * application's backup-register marker), whether the update UART has been silent long
 * enough to give up on it, and the result of verifying an image's CRC-32 over the flash
 * itself. Everything else — UART, flash, timing — is someone else's job; this module
 * only decides, which is what makes it host-testable.
 *
 * The preference order, which IS the fallback ladder:
 *
 *   1. Update requested and the update window still open? Stay in the bootloader and
 *      accept frames: the application asked for an update (marker), so the host is about
 *      to talk, and "run the stale app now" would defeat the purpose.
 *   2. Otherwise, the newest valid slip names the slot to run — but the name is only a
 *      name until the image's CRC-32 over live flash agrees with the slip. Agreeing, run
 *      that slot.
 *   3. The newest slot's image failed verification? Try the slot the LOSER slip names —
 *      the previously active one. Flash bit-rot or a torn update can take down the newest
 *      image while an older one is still whole, and the node is better off running last
 *      week's firmware than nothing. This is a rollback, and the reason field says so.
 *   4. Nothing validates anywhere — stay in the bootloader (FR-2.2's letter: a failed
 *      image leaves the node in the bootloader, and the bootloader itself is the bootable
 *      image, the one that can still accept an update).
 *
 * The marker + timeout pair is the anti-brick. The marker is set by the application's
 * `update` command and a host that never finishes would otherwise leave the node stuck in
 * update mode forever; so when the idle window expires, the marker is treated as stale and
 * the ladder decides. The window length is measured from the last received frame by the
 * UART loop, not from power-on, so a host that takes thirty seconds to launch still fits.
 */
#ifndef BOOT_DECISION_H
#define BOOT_DECISION_H

#include <stdbool.h>
#include <stdint.h>

#include "boot/layout.h"
#include "boot/metadata.h"

#ifdef __cplusplus
extern "C" {
#endif

/* How long the bootloader keeps the update window open after the last received frame, in
 * milliseconds. Tuned to be generous to a human launching the flasher, yet short enough
 * that a stale marker delays a normal boot by no more than a blink of the status LED. */
#define BOOT_UPDATE_IDLE_TIMEOUT_MS 30000u

typedef enum {
  BOOT_DECISION_RUN_SLOT_A = 1,
  BOOT_DECISION_RUN_SLOT_B = 2,
  BOOT_DECISION_STAY_BOOTLOADER = 3, /* accept an update; nothing to run or update in
                                        progress */
} boot_decision_t;

/* Why the decision is what it is. The bootloader's banner prints this, so a board that
 * boots strangely says which rung of the ladder it took instead of only that it took one. */
typedef enum {
  BOOT_REASON_UPDATE_REQUESTED = 1, /* stayed: the app asked and the window is open */
  BOOT_REASON_NEWEST_SLOT = 2,      /* ran: the newest slip's image verified */
  BOOT_REASON_ROLLBACK_SLOT = 3,    /* ran: newest failed, the previous slot verified */
  BOOT_REASON_NO_VALID_SLIP = 4,    /* stayed: neither slip validates */
  BOOT_REASON_NO_VALID_IMAGE = 5,   /* stayed: slips valid but no image verifies */
} boot_decision_reason_t;

typedef struct {
  boot_decision_t decision;
  boot_decision_reason_t reason;
} boot_outcome_t;

/* Decide. `verify_image` verifies, on whatever medium the caller reads, that the image
 * the given slip points at hashes to the slip's recorded CRC-32, returning false makes
 * that slot the rollback candidate or, ultimately, the reason nothing runs. `ctx` is
 * passed through verbatim (NULL for the bootloader, which needs no context; a test table
 * for the host suite). May be NULL, which is treated as "verification always fails" — the
 * conservative reading, correct for a bootloader that has no image checker at all: it
 * must not run images it cannot check.
 */
boot_outcome_t boot_decide(const boot_metadata_t *slip_a, const boot_metadata_t *slip_b,
                           bool update_requested, bool update_window_expired,
                           bool (*verify_image)(const boot_metadata_t *slip, void *ctx),
                           void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_DECISION_H */