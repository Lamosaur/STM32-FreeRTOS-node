/* metadata.c — see boot/metadata.h for what each rule is protecting.
 *
 * This file is deliberately pure: no flash, no registers, nothing that ties it to the
 * target. The bootloader's flash driver calls metadata_finalize() before programming a
 * slip, and the boot decision calls validate()/newer() on slips it has already read; the
 * same three functions run in the host test suite, which is how the rules are pinned.
 */
#include "boot/metadata.h"

#include "boot/crc.h"

bool boot_metadata_validate(const boot_metadata_t *m) {
  if (m == NULL) {
    return false;
  }
  if (m->magic != BOOT_METADATA_MAGIC) {
    return false;
  }
  if (m->active_slot != BOOT_SLOT_A && m->active_slot != BOOT_SLOT_B) {
    return false;
  }
  /* A slot sized image that would overrun its region, or that no program unit could
   * have written, is refused before anybody reads that many bytes. */
  if (m->image_size == 0u || m->image_size > BOOT_SLOT_SIZE) {
    return false;
  }
  if ((m->image_size % 4u) != 0u) {
    return false;
  }
  const uint16_t expected =
      boot_crc16((const uint8_t *)m, BOOT_METADATA_CHECKSUM_OFFSET);
  return m->crc16 == expected;
}

const boot_metadata_t *boot_metadata_newer(const boot_metadata_t *a,
                                           const boot_metadata_t *b) {
  const bool a_ok = boot_metadata_validate(a);
  const bool b_ok = boot_metadata_validate(b);
  if (!a_ok && !b_ok) {
    return NULL;
  }
  if (!a_ok) {
    return b;
  }
  if (!b_ok) {
    return a;
  }
  /* Both valid: the higher sequence wins, wraparound-safe. Equal sequences cannot
   * legitimately occur (each commit writes old+1 into the other slip), and if they do
   * either answer is as good as the other. */
  const uint32_t delta = a->sequence - b->sequence;
  if (delta == 0u) {
    return a;
  }
  return (delta < 0x80000000u) ? a : b;
}

void boot_metadata_finalize(boot_metadata_t *m) {
  if (m == NULL) {
    return;
  }
  m->crc16 = boot_crc16((const uint8_t *)m, BOOT_METADATA_CHECKSUM_OFFSET);
}

uint32_t boot_metadata_next(const boot_metadata_t *slip_a, const boot_metadata_t *slip_b,
                            uint32_t written_slot, uint32_t image_size, uint32_t image_crc32,
                            uint32_t version, boot_metadata_t *out) {
  if (out == NULL) {
    return 0u;
  }
  if (written_slot != BOOT_SLOT_A && written_slot != BOOT_SLOT_B) {
    return 0u;
  }
  /* The same geometry rules the validator enforces on a stored slip: a commit that
   * would write a slip the next boot refuses is a commit that must not happen. */
  if (image_size == 0u || image_size > BOOT_SLOT_SIZE || (image_size & 3u) != 0u) {
    return 0u;
  }

  const boot_metadata_t *winner = boot_metadata_newer(slip_a, slip_b);
  uint32_t write_page;
  if (winner == slip_b) {
    write_page = BOOT_METADATA_A_BASE;
  } else if (winner == slip_a) {
    write_page = BOOT_METADATA_B_BASE;
  } else {
    write_page = BOOT_METADATA_A_BASE; /* nothing valid anywhere: A is the first home */
  }

  out->magic = BOOT_METADATA_MAGIC;
  out->sequence = (winner != NULL ? winner->sequence : 0u) + 1u;
  out->active_slot = written_slot;
  out->image_size = image_size;
  out->image_crc32 = image_crc32;
  out->version = version;
  out->reserved = 0u;
  out->reserved16 = 0u;
  boot_metadata_finalize(out);
  return write_page;
}