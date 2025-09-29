/* flash.h — the bootloader's flash programmer (F103 controller, raw registers).
 *
 * Two operations, both page-scoped: erase one 1 KB page, then program half-words into an
 * erased page. The updater never spans a page boundary; the whole-image CRC at the end of
 * an update is what catches a driver bug, so the driver itself stays dumb.
 */
#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Erase the page containing page_address. Returns 0 on success, -1 on a latched flash
 * error (e.g. a write-protected or disabled page). The address must be the page's base. */
int boot_flash_erase_page(uint32_t page_address);

/* Program `length` bytes at even `address`, inside a single erased page. `length` must
 * be even (the F1 programs half-words) and non-zero. Returns 0 on success, -1 otherwise. */
int boot_flash_program(uint32_t address, const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_FLASH_H */