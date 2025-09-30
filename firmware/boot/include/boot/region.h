/* region.h — which flash regions an update may touch (the boot-0 rule's teeth).
 *
 * The bootloader's ERASE and WRITE handlers accept addresses from the wire, and the one
 * thing a host must never be able to do with them is reach the program answering the
 * wire. Only the two application slots are writable by a frame; the metadata pages are
 * written exclusively by COMMIT's slip writer, which constructs the record it programs.
 */
#ifndef BOOT_REGION_H
#define BOOT_REGION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* True when address..address+length lies entirely within one of the two application
 * slots. Zero length, a length crossing a slot boundary, the bootloader's own pages,
 * the metadata pages, the spare and anything outside the map are all refused — the
 * arithmetic is wrap-safe because the bounds are checked before the differences. */
bool boot_region_writable(uint32_t address, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_REGION_H */