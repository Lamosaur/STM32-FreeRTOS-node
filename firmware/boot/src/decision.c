/* decision.c — the fallback ladder documented in boot/decision.h.
 *
 * Pure: no flash, no UART, no clock. The bootloader's main() supplies the two slips and
 * the flag/callback results; the host test suite supplies the same inputs from tables.
 */
#include "boot/decision.h"

static boot_decision_t slot_to_decision(uint32_t slot) {
  return (slot == BOOT_SLOT_A) ? BOOT_DECISION_RUN_SLOT_A : BOOT_DECISION_RUN_SLOT_B;
}

boot_outcome_t boot_decide(const boot_metadata_t *slip_a, const boot_metadata_t *slip_b,
                           bool update_requested, bool update_window_expired,
                           bool (*verify_image)(const boot_metadata_t *slip, void *ctx),
                           void *ctx) {
  boot_outcome_t out;
  out.decision = BOOT_DECISION_STAY_BOOTLOADER;
  out.reason = BOOT_REASON_NO_VALID_SLIP;

  /* Rung 1: an in-flight update wins over everything, including a perfectly good app —
   * the app itself asked for this. The window expiring demotes the marker to a stale
   * relic and falls through to the ladder, which is the anti-brick behaviour. */
  if (update_requested && !update_window_expired) {
    out.reason = BOOT_REASON_UPDATE_REQUESTED;
    return out;
  }

  /* boot_metadata_newer returns a validated slip, or NULL when neither is valid. */
  const boot_metadata_t *newest = boot_metadata_newer(slip_a, slip_b);
  if (newest == NULL) {
    out.reason = BOOT_REASON_NO_VALID_SLIP; /* already set, but state it explicitly */
    return out;
  }

  /* Rung 2: run the newest slot only if its image actually hashes to the slip's CRC.
   * With verify_image NULL, no image is ever "verified", which is a bootloader defect,
   * and the conservative answer is to run nothing. */
  if (verify_image != NULL && verify_image(newest, ctx)) {
    out.decision = slot_to_decision(newest->active_slot);
    out.reason = BOOT_REASON_NEWEST_SLOT;
    return out;
  }

  /* Rung 3: rollback — the loser slip names the previously active slot. It is checked
   * with validate() first because newer() only validated the winner, and a garbage
   * image_size must never reach the image reader. */
  const boot_metadata_t *loser = (newest == slip_a) ? slip_b : slip_a;
  if (loser != NULL && boot_metadata_validate(loser) && verify_image != NULL &&
      verify_image(loser, ctx)) {
    out.decision = slot_to_decision(loser->active_slot);
    out.reason = BOOT_REASON_ROLLBACK_SLOT;
    return out;
  }

  /* Rung 4: slips exist but no image verifies. Stay; this is where an update recovers. */
  out.reason = BOOT_REASON_NO_VALID_IMAGE;
  return out;
}