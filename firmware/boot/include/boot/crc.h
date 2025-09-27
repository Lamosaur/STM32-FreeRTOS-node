/* crc.h — the two checksums the update path uses, as pure functions.
 *
 * Why two different ones. The wire protocol checks every frame with a CRC-16 (FR-2.4:
 * "length-prefixed, CRC-16, ACK/retry"): a 16-bit value costs less than half the per-frame
 * overhead of a 32-bit one and catches any single-bit or short-burst corruption of a
 * frame, which retry already defends against. The image itself is checked with a CRC-32,
 * decided in the Phase 2 record: a corrupt image is not retried per-frame, it is booted by
 * the flash reader for however long the node runs, so a 1-in-65,536 miss rate (CRC-16)
 * is not acceptable for a controller whose failure mode is watering wrongly.
 *
 * Neither function allocates, and neither needs a context: the bootloader calls them from
 * wherever it happens to be, including while the flash programmer runs.
 */
#ifndef BOOT_CRC_H
#define BOOT_CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection, no final
 * XOR. The "false" qualifier matters: the reflected CCITT variant (init 0x0000, used by
 * XMODEM and Bluetooth) gives different values for the same bytes, and the two are
 * indistinguishable to someone who has not named the variant. The check value on the
 * ASCII string "123456789" is 0x29B1, which the host test pins. */
uint16_t boot_crc16(const uint8_t *data, size_t len);

/* CRC-32, the IEEE 802.3 / zlib polynomial: 0xEDB88320 reflected, init 0xFFFFFFFF,
 * final XOR 0xFFFFFFFF. Check value on "123456789" is 0xCBF43926. */
uint32_t boot_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_CRC_H */