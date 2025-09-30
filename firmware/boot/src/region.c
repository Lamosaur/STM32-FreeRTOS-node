/* region.c — see boot/region.h for the rule being enforced. */
#include "boot/region.h"

#include "boot/layout.h"

bool boot_region_writable(uint32_t address, uint32_t length) {
  if (length == 0u) {
    return false;
  }
  /* The bounds are checked before either difference, so a hostile address cannot wrap
   * the arithmetic: `address < end` is true before `end - address` is taken. */
  const uint32_t slot_a_end = BOOT_SLOT_A_BASE + BOOT_SLOT_SIZE;
  if (address >= BOOT_SLOT_A_BASE && address < slot_a_end &&
      length <= slot_a_end - address) {
    return true;
  }
  const uint32_t slot_b_end = BOOT_SLOT_B_BASE + BOOT_SLOT_SIZE;
  if (address >= BOOT_SLOT_B_BASE && address < slot_b_end &&
      length <= slot_b_end - address) {
    return true;
  }
  return false;
}