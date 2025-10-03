/* frame.c — the receiver state machine and reply encoder of boot/frame.h.
 *
 * Both are byte-level on purpose. The receiver is driven one byte at a time by the
 * update loop's polled UART, so it must never assume a whole frame is ever available at
 * once; the encoder exists so every reply — ACK or NAK, with or without payload — is
 * built by one function and cannot drift from the wire format.
 */
#include "boot/frame.h"

#include "boot/crc.h"

void boot_frame_rx_init(boot_frame_rx_t *rx) {
  rx->phase = 0;
  rx->fill = 0;
  rx->expected = 0;
}

boot_frame_result_t boot_frame_rx_byte(boot_frame_rx_t *rx, uint8_t byte,
                                       uint8_t *out_cmd, uint8_t *out_len,
                                       const uint8_t **out_payload) {
  switch (rx->phase) {
    case 0: /* sync: nothing before 0xAA counts, which is the resync rule */
      if (byte == BOOT_FRAME_SYNC) {
        rx->phase = 1;
        rx->fill = 0;
      }
      return BOOT_FRAME_MORE;

    case 1: /* command */
      rx->buf[rx->fill++] = byte;
      rx->phase = 2;
      return BOOT_FRAME_MORE;

    case 2: /* length: governs how many bytes are payload, so a 0xAA inside a payload
               is data, never a new frame */
      if (byte > BOOT_FRAME_MAX_LEN) {
        /* Refuse on the wire: the frame is over the limit; the caller replies NAK and
         * the receiver waits for the next sync byte. */
        rx->phase = 0;
        return BOOT_FRAME_BAD_LEN;
      }
      rx->buf[rx->fill++] = byte;
      rx->expected = byte;
      rx->phase = (byte == 0u) ? 4u : 3u;
      return BOOT_FRAME_MORE;

    case 3: /* payload */
      rx->buf[rx->fill++] = byte;
      --rx->expected;
      if (rx->expected == 0u) {
        rx->phase = 4;
      }
      return BOOT_FRAME_MORE;

    case 4: /* CRC low byte */
      rx->crc_low = byte;
      rx->phase = 5;
      return BOOT_FRAME_MORE;

    case 5: /* CRC high byte; the frame is complete */
      rx->phase = 0; /* resync for the next frame regardless of the verdict below */
      {
        const uint16_t expected_crc =
            (uint16_t)(((uint16_t)byte << 8) | (uint16_t)rx->crc_low);
        /* The checksum covers command, length and payload — everything from buf[0] to
         * the end of the payload, which is fill bytes at this point. */
        const uint16_t actual_crc = boot_crc16(rx->buf, rx->fill);
        if (actual_crc != expected_crc) {
          return BOOT_FRAME_BAD_CRC;
        }
      }
      if (out_cmd != NULL) {
        *out_cmd = rx->buf[0];
      }
      if (out_len != NULL) {
        *out_len = rx->buf[1];
      }
      if (out_payload != NULL) {
        *out_payload = &rx->buf[2];
      }
      return BOOT_FRAME_COMPLETE;

    default:
      rx->phase = 0;
      return BOOT_FRAME_MORE;
  }
}

uint16_t boot_frame_encode(uint8_t *out, uint16_t capacity, uint8_t status,
                           const uint8_t *payload, uint8_t len) {
  if (out == NULL || (len > BOOT_FRAME_MAX_LEN)) {
    return 0u;
  }
  /* sync(1) + status(1) + len(1) + payload(len) + crc(2) */
  const uint16_t total = (uint16_t)(5u + (uint16_t)len);
  if (capacity < total) {
    return 0u;
  }
  out[0] = BOOT_FRAME_SYNC;
  out[1] = status;
  out[2] = len;
  if (len > 0u && payload != NULL) {
    for (uint8_t i = 0; i < len; ++i) {
      out[3u + i] = payload[i];
    }
  }
  const uint16_t crc = boot_crc16(&out[1], 2u + (uint16_t)len); /* status..payload */
  out[3u + (uint16_t)len] = (uint8_t)(crc & 0xFFu);
  out[4u + (uint16_t)len] = (uint8_t)(crc >> 8);
  return total;
}