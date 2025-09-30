/* metadata.h — the record of which slot is active, and the rules for trusting it.
 *
 * One 32-byte struct lives at the start of each metadata slip (layout.h). It is the only
 * thing the bootloader believes without checking the flash for itself, so every field and
 * every rule is documented against the failure it defends against:
 *
 *   magic        — a fixed constant. An erased page reads 0xFF, and a half-written slip
 *                  reads some mix of 0xFF and data; both fail this check. It is what
 *                  makes "the page was properly programmed" decidable in one comparison.
 *   sequence     — monotonically increasing across commits. The reader trusts the slip
 *                  with the higher sequence; the comparison is wraparound-safe (see
 *                  boot_metadata_newer), because 2^32 commits is not a number this part
 *                  will ever see but it costs nothing to get right.
 *   active_slot  — BOOT_SLOT_A or BOOT_SLOT_B. The bootloader runs the image here; the
 *                  update writes the *other* slot and then commits a new slip pointing at
 *                  it.
 *   image_size   — the byte length of the image in the slot. A size that exceeds the
 *                  slot, or is not a multiple of 4 (the flash half-word/word program
 *                  unit), fails validation before the CRC runs.
 *   image_crc32  — CRC-32 over exactly image_size bytes starting at the slot's base.
 *                  Computed by the bootloader at every boot over the *current* slot, so
 *                  a torn flash page or a slot that never finished programming cannot
 *                  hide behind a stale-but-pristine slip.
 *   version      — the app's build number (FR-2.6: the host reads it without booting the
 *                  app). Semantics live with the app; the bootloader only stores it.
 *   crc16        — CRC-16 over everything in the struct before this field (28 bytes
 *                  today), so no single field can be corrupted into validity: a corrupt
 *                  magic fails on its own, and a corrupted sequence, slot, size or CRC
 *                  also breaks this checksum. The reach of the checksum is derived by
 *                  offsetof, so it follows the struct as it evolves.
 *
 * The struct is deliberately a fixed 32 bytes with the checksum last, so "recompute and
 * compare" is one call over a compile-time-known length regardless of which fields exist
 * in a future revision — extend the struct by inserting before crc16 and the rule still
 * holds; its length changes and the constant below must too (the host test pins it).
 */
#ifndef BOOT_METADATA_H
#define BOOT_METADATA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "boot/layout.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOOT_METADATA_MAGIC 0xB007B007u

typedef struct {
  uint32_t magic;        /* BOOT_METADATA_MAGIC */
  uint32_t sequence;     /* higher wins; wraps at 2^32 */
  uint32_t active_slot;  /* BOOT_SLOT_A or BOOT_SLOT_B */
  uint32_t image_size;   /* bytes of the image; <= BOOT_SLOT_SIZE, % 4 == 0 */
  uint32_t image_crc32;  /* CRC-32 over image_size bytes from the slot base */
  uint32_t version;      /* app build number, reported to the host (FR-2.6) */
  uint32_t reserved;
  uint16_t crc16;        /* CRC-16 over the bytes before this field (see below) */
  uint16_t reserved16;
} boot_metadata_t;

/* The checksum covers every byte up to the crc16 field, whatever that offset is. Derived
 * with offsetof rather than a literal so that adding or removing a field above crc16
 * automatically extends the checksum's reach when the struct changes — a literal here
 * would leave new fields outside the checksum until someone noticed. The host test pins
 * the value (28 bytes today) so a layout change is a deliberate, visible event. */
#define BOOT_METADATA_CHECKSUM_OFFSET (offsetof(boot_metadata_t, crc16))

/* True when the slip passes its own checks: magic correct, checksum correct, and the
 * payload within the geometry of the part. The image CRC is NOT checked here — that is
 * done over the flash itself, at boot, by the bootloader; a slip with a stale CRC is
 * still used to reach the slot, whose content then has to pass the real check. */
bool boot_metadata_validate(const boot_metadata_t *m);

/* The newer of two slips, or NULL if neither is valid. The comparison must be safe
 * against a wrapping sequence: `a` is newer than `b` when (a.sequence - b.sequence)
 * mod 2^32 is in [1, 2^31). Exactly 2^31 apart is ambiguous and treated as "b is
 * newer" — the two slips can never be that far apart unless every commit in between
 * was lost, which no recovery can fix. */
const boot_metadata_t *boot_metadata_newer(const boot_metadata_t *a,
                                           const boot_metadata_t *b);

/* Fill in the checksum after the caller has set every other field. The bootloader writes
 * a slip by: erase the slip's page, program the struct with crc16 filled by this call. */
void boot_metadata_finalize(boot_metadata_t *m);

/* Build the slip for the next commit and say which metadata page it must be written to.
 * `written_slot` is the slot the update just filled; `image_size`/`image_crc32`/`version`
 * are what the host supplied and what verification has already agreed with the flash. The
 * new slip's sequence is the winning slip's plus one, and it goes to the metadata page
 * that is NOT winning — the whole two-slip argument: a power cut during the commit's own
 * erase/program leaves the winning slip intact, so the previous app still boots. With no
 * winning slip (a fresh part) slip A is the natural first home.
 *
 * Returns the page base to program, or 0 if the inputs cannot be a valid slip (bad slot,
 * zero or oversized or misaligned image size). On 0 the caller must refuse the commit. */
uint32_t boot_metadata_next(const boot_metadata_t *slip_a, const boot_metadata_t *slip_b,
                            uint32_t written_slot, uint32_t image_size, uint32_t image_crc32,
                            uint32_t version, boot_metadata_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_METADATA_H */