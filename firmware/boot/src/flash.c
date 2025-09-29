/* flash.c — the F103 flash controller, unfiltered.
 *
 * The register story, because a programmer is where the map and the silicon meet:
 *
 *   - FLASH->KEYR. The controller is locked after reset; every operation begins with the
 *     two key writes and ends with re-locking. A stray bus pattern can never reprogram
 *     anything without the keys in sequence.
 *   - FLASH->CR selects the operation. PG = program half-word now, PER = prepare a page
 *     erase, and for a page erase the address goes into FLASH->AR before STRT fires it.
 *   - The controller is busy (SR.BSY) for the duration. The F1 stalls instruction fetch
 *     while the controller is busy — a quirk, not a bug: the busy-wait loop after an
 *     operation simply does not run until the controller is idle again, so a synchronous
 *     driver is correct, it just cannot do anything else during an operation. That is
 *     fine here; the bootloader has nothing else to do.
 *   - Errors latch in SR (WRPRTERR, PGERR) and must be cleared by writing 1s. The driver
 *     reports them so the updater can refuse to commit a metadata slip over a page that
 *     did not actually program.
 *
 * This code lives in the bootloader region (flash pages 0..7). It never programs or
 * erases its own pages — the updater only touches the slots and metadata — so the fetch
 * stall never races the code that waits.
 */
#include "flash.h"

#include <stddef.h>

#include "boot/layout.h"
#include "stm32f1xx.h"

static void flash_unlock(void) {
  FLASH->KEYR = 0x45670123u;
  FLASH->KEYR = 0xCDEF89ABu;
}

static void flash_lock(void) {
  FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_idle(void) {
  while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
  }
}

/* Any latched error, cleared. Returns non-zero if an error was latched. */
static int flash_errors_clear(void) {
  const int had_error = (FLASH->SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)) != 0u;
  FLASH->SR = FLASH_SR_WRPRTERR | FLASH_SR_PGERR; /* write 1 to clear */
  return had_error;
}

static void flash_clear_eop(void) {
  FLASH->SR = FLASH_SR_EOP; /* write 1 to clear */
}

int boot_flash_erase_page(uint32_t page_address) {
  if ((page_address & (BOOT_FLASH_PAGE_SIZE - 1u)) != 0u) {
    return -1; /* the controller expects the page's first byte */
  }
  flash_unlock();
  flash_wait_idle();
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = page_address;
  FLASH->CR |= FLASH_CR_STRT;
  flash_wait_idle();
  FLASH->CR &= ~FLASH_CR_PER;
  flash_clear_eop();
  const int err = flash_errors_clear();
  flash_lock();
  return err ? -1 : 0;
}

int boot_flash_program(uint32_t address, const uint8_t *data, uint32_t length) {
  /* The controller's unit is a half-word at an even address; the updater's unit is a
   * page, and this driver does not span pages because erase is page-granular. */
  if (data == NULL || (address & 1u) != 0u || (length & 1u) != 0u || length == 0u) {
    return -1;
  }
  if ((address & (BOOT_FLASH_PAGE_SIZE - 1u)) + length > BOOT_FLASH_PAGE_SIZE) {
    return -1;
  }
  flash_unlock();
  flash_wait_idle();
  FLASH->CR |= FLASH_CR_PG;
  for (uint32_t off = 0; off < length; off += 2u) {
    const uint16_t half = (uint16_t)((uint16_t)data[off] | ((uint16_t)data[off + 1u] << 8));
    /* Writing the flash address starts the program; the next instruction fetch stalls
     * until it completes (see the header), so the wait below is belt-and-braces for the
     * controller's BSY on the last half-word. */
    *(volatile uint16_t *)(address + off) = half;
    flash_wait_idle();
  }
  FLASH->CR &= ~FLASH_CR_PG;
  flash_clear_eop();
  const int err = flash_errors_clear();
  flash_lock();
  return err ? -1 : 0;
}