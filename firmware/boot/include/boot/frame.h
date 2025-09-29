/* boot_frame.h — the update wire protocol (FR-2.4): one framed format, both ends.
 *
 * The frame, byte for byte:
 *
 *     b0           = 0xAA, the sync/magic byte. Nothing else can start a frame.
 *     b1           = the command (host -> boot) or the status (boot -> host).
 *     b2           = payload length L, 0..BOOT_FRAME_MAX_LEN.
 *     b3..b2+L     = the payload.
 *     last two     = CRC-16/CCITT-FALSE over b1..b2+L, little-endian.
 *
 * One frame in flight at a time (stop-and-wait): the host sends a frame and waits for the
 * reply; the reply is the same shape with the status byte in the command position — 0
 * (BOOT_STATUS_OK) is the ACK, any other value is the NAK with the reason attached. On a
 * NAK the host retries the whole frame, which is what makes the protocol robust against
 * a line glitch: there is no partial-frame recovery to get right, only "say it again".
 * The same framing is implemented in host/flasher.py with this header as the shared
 * reference, so "one framed protocol" means one format, pinned on both sides by tests.
 *
 * Why 0xAA and why a length byte: the sync byte gives the receiver a resync point (any
 * byte that is not 0xAA is discarded, so mid-garbage recovery is "wait for 0xAA"), and
 * the length byte tells the receiver how many payload bytes to consume before the CRC, so
 * a 0xAA inside a payload is data, never a false frame start.
 *
 * Commands:
 *   HELLO     - reply ACK, payload = the bootloader version string (FR-2.6's "who am I
 *               talking to", before any image logic).
 *   GET_META  - reply ACK, payload = the 32 bytes of the winning metadata slip, so a host
 *               reads the app's version, slot and CRC-32 without booting the app (FR-2.6).
 *   ERASE     - payload = 4-byte little-endian page address. Erases that page.
 *   WRITE     - payload = 4-byte LE address + data bytes. Programs half-words into the
 *               (erased) page up to its end; payload length bounds the write.
 *   COMMIT    - payload = 4-byte LE image size + 4-byte LE CRC-32 + 4-byte LE version.
 *               The bootloader re-verifies the just-written image's CRC-32 over flash,
 *               and only then writes the new metadata slip (sequence + 1, active = the
 *               slot that was written) and clears the update marker: the boot-0 rule's
 *               commit moment, atomic across one slip.
 *   RESET     - reply ACK, then reset the part: the fresh image boots, or the bootloader
 *               decides otherwise if the commit failed.
 *
 * The status reasons are small, numbered and stable: a host prints them verbatim, and a
 * bootloader never invents a new one without extending this header and the tests.
 */
#ifndef BOOT_FRAME_H
#define BOOT_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOOT_FRAME_SYNC 0xAAu
#define BOOT_FRAME_MAX_LEN 240u /* 240 payload bytes: 42 KB / 240 = 180 frames an update */

typedef enum {
  BOOT_CMD_HELLO = 1,
  BOOT_CMD_GET_META = 2,
  BOOT_CMD_ERASE = 3,
  BOOT_CMD_WRITE = 4,
  BOOT_CMD_COMMIT = 5,
  BOOT_CMD_RESET = 6,
} boot_cmd_t;

typedef enum {
  BOOT_STATUS_OK = 0,        /* the ACK */
  BOOT_STATUS_BAD_CRC = 1,   /* frame arrived with a bad checksum: say it again */
  BOOT_STATUS_BAD_CMD = 2,   /* unknown command byte */
  BOOT_STATUS_BAD_LEN = 3,   /* payload length over the wire limit */
  BOOT_STATUS_BAD_ADDR = 4,  /* address not in the map / not the requested page / odd */
  BOOT_STATUS_BAD_STATE = 5, /* e.g. WRITE before ERASE, or COMMIT with nothing written */
  BOOT_STATUS_VERIFY_FAIL = 6, /* COMMIT's re-verified image CRC did not match */
  BOOT_STATUS_FLASH_ERROR = 7, /* the flash controller latched an error */
} boot_status_t;

/* One frame's payload space for the receiver. Almost all of the 20 KB RAM is the
 * application's; less than 256 bytes here is the bootloader's largest single allocation.
 */
#define BOOT_FRAME_DATA_BYTES (BOOT_FRAME_MAX_LEN + 2u) /* cmd + len + payload */

typedef enum {
  BOOT_FRAME_MORE = 0,      /* consume more bytes; nothing complete yet */
  BOOT_FRAME_COMPLETE = 1,  /* a whole frame, CRC verified; fields are valid */
  BOOT_FRAME_BAD_CRC = 2,   /* a whole frame arrived, its CRC failed; resync already done */
  BOOT_FRAME_BAD_LEN = 3,   /* the length byte exceeded the wire limit; resync done */
} boot_frame_result_t;

typedef struct {
  uint8_t buf[BOOT_FRAME_DATA_BYTES];
  uint8_t phase;       /* 0 sync, 1 cmd, 2 len, 3 payload, 4 crc-low, 5 crc-high */
  uint8_t fill;        /* bytes stored in buf so far for the current frame */
  uint8_t expected;    /* payload bytes remaining in the payload phase */
  uint8_t crc_low;     /* first CRC byte, held until the second arrives */
} boot_frame_rx_t;

void boot_frame_rx_init(boot_frame_rx_t *rx);

/* Feed one byte. On BOOT_FRAME_COMPLETE, out_cmd/out_len/out_payload describe the frame
 * (payload points INTO rx, valid until the next byte is fed). On BAD_CRC or BAD_LEN the
 * receiver has already resynced (the next frame can begin immediately) and the caller
 * replies with the matching status. Never blocks, never allocates. */
boot_frame_result_t boot_frame_rx_byte(boot_frame_rx_t *rx, uint8_t byte,
                                       uint8_t *out_cmd, uint8_t *out_len,
                                       const uint8_t **out_payload);

/* Build a reply frame (the same shape, status in the command position) into `out`.
 * Returns the byte count, or 0 if the payload does not fit. `status` is BOOT_STATUS_OK
 * for the ACK; the NAK attaches the reason. CRC computed over command and payload. */
uint16_t boot_frame_encode(uint8_t *out, uint16_t capacity, uint8_t status,
                           const uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_FRAME_H */