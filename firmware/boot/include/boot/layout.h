/* layout.h — the Phase 2 flash map, in one place (NFR-2.9).
 *
 * Every address in this phase that depends on the part's flash geometry lives here and
 * nowhere else. Retargeting to the Phase 3 MCU is changing constants in this file (and in
 * the two linker scripts, which cannot include a C header — they cross-check against this
 * file in a host test instead).
 *
 * The map, on the 128 KB part measured during bring-up (1 KB pages):
 *
 *   0x08000000  bootloader, 8 KB     the program that runs at every reset
 *   0x08002000  slot A, 56 KB        an application image
 *   0x08010000  slot B, 56 KB        the other application image
 *   0x0801E000  metadata A, 1 KB     the record of which slot is active
 *   0x0801E400  metadata B, 1 KB     its twin (see below)
 *   0x0801E800  spare, 6 KB          for growth
 *
 * Why two slots and why two metadata slips, one paragraph each, because both are the same
 * argument at different scales:
 *
 *   - The boot-0 rule (see the bootloader's main.c) is that something is ALWAYS runnable.
 *     A single app region would have to erase the running image to make room for the new
 *     one, so a power cut mid-update would erase the only app. Two slots fix that: the
 *     update writes the slot that is NOT running, and metadata only flips once the new
 *     image is verified (FR-2.5's "previous valid metadata intact").
 *   - The metadata page itself is written the same non-atomic way (erase, then program),
 *     so a power cut between those two steps would erase the only record of which slot is
 *     valid — violating the rule again. Two metadata slips, written alternately with a
 *     monotonic sequence number, guarantee one is always fully written: the reader takes
 *     the slip with the higher sequence.
 *
 * Every region is page-aligned (pages are 1 KB), so no erase or program operation ever
 * spans two regions, and a half-finished operation can only ever damage one region.
 */
#ifndef BOOT_LAYOUT_H
#define BOOT_LAYOUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The geometry of the part itself. These are the numbers that change for the Phase 3
 * MCU; everything below is derived from them and must not be edited separately. */
#define BOOT_FLASH_BASE      0x08000000u   /* where flash begins in the memory map */
#define BOOT_FLASH_PAGE_SIZE 0x400u        /* 1 KB per erase unit */
#define BOOT_FLASH_SIZE      0x20000u      /* 128 KB, measured on the bench part */

/* The four regions. Sizes are page multiples on purpose (see the header comment). */
#define BOOT_BOOTLOADER_BASE 0x08000000u
#define BOOT_BOOTLOADER_SIZE 0x2000u       /* 8 KB — the bootloader image itself */

#define BOOT_SLOT_A_BASE     0x08002000u
#define BOOT_SLOT_B_BASE     0x08010000u
#define BOOT_SLOT_SIZE       0xE000u       /* 56 KB per slot: room for a 42 KB app plus
                                              ~14 KB of Phase 3 growth */

#define BOOT_METADATA_A_BASE 0x0801E000u
#define BOOT_METADATA_B_BASE 0x0801E400u
#define BOOT_METADATA_SIZE   0x400u        /* 1 KB per slip; a slip's struct is ~32 bytes */

/* Unallocated growth room. The host test asserts the regions tile exactly to the end of
 * flash, so this number is what changes when a future phase needs more room. */
#define BOOT_SPARE_BASE      0x0801E800u
#define BOOT_SPARE_SIZE      0x1800u       /* 6 KB */

#define BOOT_FLASH_END       (BOOT_FLASH_BASE + BOOT_FLASH_SIZE) /* 0x08020000 */

/* The slots are identified by these values in metadata, never by their addresses in
 * application code: the decision "which slot" is a datum, and the address of that slot is
 * a lookup this header owns. This is what lets the map change without touching the logic.
 */
#define BOOT_SLOT_NONE       0u
#define BOOT_SLOT_A          1u
#define BOOT_SLOT_B          2u

/* The base address of a slot's image, from its slot ID. Returns 0 for an unknown ID. */
static inline uint32_t boot_slot_base(uint32_t slot) {
  if (slot == BOOT_SLOT_A) {
    return BOOT_SLOT_A_BASE;
  }
  if (slot == BOOT_SLOT_B) {
    return BOOT_SLOT_B_BASE;
  }
  return 0u;
}

/* The other slot — "where the update goes" is always "not where we are running from". */
static inline uint32_t boot_other_slot(uint32_t slot) {
  return (slot == BOOT_SLOT_A) ? BOOT_SLOT_B : BOOT_SLOT_A;
}

#ifdef __cplusplus
}
#endif

#endif /* BOOT_LAYOUT_H */