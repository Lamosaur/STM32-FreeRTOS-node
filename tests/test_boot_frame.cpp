/* Tests for the Phase 2 wire protocol (boot/frame.h): one framing, both ends.
 *
 * The receiver is tested the way the update loop will run it — a byte at a time — and
 * the way want it to behave in the field: a valid frame decodes whole; a frame with a
 * bad CRC is refused without wedging the receiver; garbage before the sync byte is
 * skipped; an over-limit length is refused; a 0xAA inside a payload is data, never a
 * frame start; and back-to-back frames decode without state between them. The encoder is
 * the other half pinned here: anything a bootloader replies with must decode through the
 * same FSM that parses host frames, which is the "one framing" promise.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "boot/crc.h"
#include "boot/frame.h"

namespace node {
namespace {

// Build a host->boot frame: sync, cmd, len, payload, little-endian CRC-16 over cmd..payload.
size_t make_frame(uint8_t *out, uint8_t cmd, const uint8_t *payload, uint8_t len) {
  out[0] = BOOT_FRAME_SYNC;
  out[1] = cmd;
  out[2] = len;
  if (len > 0u) {
    std::memcpy(out + 3, payload, len);
  }
  const uint16_t crc = boot_crc16(out + 1, 2u + len);
  out[3u + len] = static_cast<uint8_t>(crc & 0xFFu);
  out[4u + len] = static_cast<uint8_t>(crc >> 8);
  return 5u + len;
}

// Feed a whole frame byte by byte; returns the last non-MORE result.
boot_frame_result_t feed(boot_frame_rx_t *rx, const uint8_t *frame, size_t n,
                         uint8_t *cmd, uint8_t *len, const uint8_t **payload) {
  boot_frame_result_t r = BOOT_FRAME_MORE;
  for (size_t i = 0; i < n; ++i) {
    r = boot_frame_rx_byte(rx, frame[i], cmd, len, payload);
  }
  return r;
}

TEST(BootFrameTest, AWholeFrameDecodes) {
  const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
  uint8_t frame[16];
  const size_t n = make_frame(frame, BOOT_CMD_HELLO, payload, sizeof(payload));

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload_out = nullptr;
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload_out), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(cmd, BOOT_CMD_HELLO);
  EXPECT_EQ(len, sizeof(payload));
  ASSERT_NE(payload_out, nullptr);
  EXPECT_EQ(std::memcmp(payload_out, payload, sizeof(payload)), 0);
}

TEST(BootFrameTest, AZeroLengthFrameDecodes) {
  uint8_t frame[8];
  const size_t n = make_frame(frame, BOOT_CMD_RESET, nullptr, 0);

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(cmd, BOOT_CMD_RESET);
  EXPECT_EQ(len, 0u);
}

TEST(BootFrameTest, ABadCrcIsRefusedAndTheReceiverKeepsWorking) {
  uint8_t frame[16];
  const size_t n = make_frame(frame, BOOT_CMD_ERASE, nullptr, 0);
  frame[n - 1] ^= 0xFFu; /* corrupt the CRC high byte */

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload), BOOT_FRAME_BAD_CRC);

  // The very next frame must decode cleanly: a NAK retry is a whole new frame.
  const size_t m = make_frame(frame, BOOT_CMD_HELLO, nullptr, 0);
  EXPECT_EQ(feed(&rx, frame, m, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
}

TEST(BootFrameTest, GarbageBeforeTheSyncByteIsSkipped) {
  // Any byte that is not 0xAA is discarded, so junk that genuinely cannot be a frame
  // start is skipped and the real frame decodes. (A literal 0xAA in the garbage WOULD
  // be taken as a sync — that is the protocol's resync rule, and the worst it can do is
  // start a frame whose CRC fails, which the receiver refuses and the host retries.)
  const uint8_t junk[] = {0x00, 0xFF, 0x55, 0x00};
  uint8_t frame[16];
  const size_t n = make_frame(frame, BOOT_CMD_HELLO, nullptr, 0);

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  for (size_t i = 0; i < sizeof(junk); ++i) {
    ASSERT_EQ(boot_frame_rx_byte(&rx, junk[i], nullptr, nullptr, nullptr), BOOT_FRAME_MORE);
  }
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(cmd, BOOT_CMD_HELLO);
}

TEST(BootFrameTest, AnOverLimitLengthIsRefusedAndTheReceiverResyncs) {
  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  // sync, cmd, then a length beyond the wire limit; the third byte already ends it.
  EXPECT_EQ(boot_frame_rx_byte(&rx, BOOT_FRAME_SYNC, &cmd, &len, &payload), BOOT_FRAME_MORE);
  EXPECT_EQ(boot_frame_rx_byte(&rx, BOOT_CMD_WRITE, &cmd, &len, &payload), BOOT_FRAME_MORE);
  EXPECT_EQ(boot_frame_rx_byte(&rx, BOOT_FRAME_MAX_LEN + 1u, &cmd, &len, &payload),
            BOOT_FRAME_BAD_LEN);

  uint8_t frame[16];
  const size_t n = make_frame(frame, BOOT_CMD_HELLO, nullptr, 0);
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
}

TEST(BootFrameTest, ASyncByteInsideThePayloadIsDataNotAFrameStart) {
  const uint8_t payload[] = {0x00, BOOT_FRAME_SYNC, 0x01, 0xAA, 0x02};
  uint8_t frame[16];
  const size_t n = make_frame(frame, BOOT_CMD_WRITE, payload, sizeof(payload));

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload_out = nullptr;
  EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload_out), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(len, sizeof(payload));
  EXPECT_EQ(std::memcmp(payload_out, payload, sizeof(payload)), 0);
}

TEST(BootFrameTest, BackToBackFramesDecodeWithoutStateBetweenThem) {
  uint8_t frame[16];
  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);

  for (uint8_t i = 0; i < 4; ++i) {
    const size_t n = make_frame(frame, static_cast<uint8_t>(BOOT_CMD_HELLO + i), nullptr, 0);
    uint8_t cmd = 0, len = 0;
    const uint8_t *payload = nullptr;
    EXPECT_EQ(feed(&rx, frame, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
    EXPECT_EQ(cmd, static_cast<uint8_t>(BOOT_CMD_HELLO + i));
  }
}

TEST(BootFrameTest, AnEncodedReplyDecodesThroughTheSameMachine) {
  // "One framing" is the promise: a reply built by the encoder must parse through the
  // same FSM as a host frame, with the status byte in the command position.
  const uint8_t info[] = "boot 0.1.0";
  uint8_t reply[32];
  const uint16_t n = boot_frame_encode(reply, sizeof(reply), BOOT_STATUS_OK, info,
                                       static_cast<uint8_t>(sizeof(info) - 1u));
  ASSERT_GT(n, 0u);

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  EXPECT_EQ(feed(&rx, reply, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(cmd, BOOT_STATUS_OK);
  EXPECT_EQ(len, static_cast<uint8_t>(sizeof(info) - 1u));
  EXPECT_EQ(std::memcmp(payload, info, sizeof(info) - 1u), 0);
}

TEST(BootFrameTest, EncoderRefusesWhatCannotBeSent) {
  uint8_t out[4];
  EXPECT_EQ(boot_frame_encode(out, sizeof(out), BOOT_STATUS_OK, nullptr, 0), 0u); /* too
                                                                                      small */
  EXPECT_EQ(boot_frame_encode(out, sizeof(out) + 8u, BOOT_STATUS_OK, nullptr,
                              BOOT_FRAME_MAX_LEN + 1u), 0u); /* over the wire limit */
}

TEST(BootFrameTest, NakCarriesTheReasonInTheStatusByte) {
  uint8_t reply[8];
  const uint16_t n = boot_frame_encode(reply, sizeof(reply), BOOT_STATUS_BAD_ADDR, nullptr,
                                       0u);
  ASSERT_GT(n, 0u);

  boot_frame_rx_t rx;
  boot_frame_rx_init(&rx);
  uint8_t cmd = 0, len = 0;
  const uint8_t *payload = nullptr;
  EXPECT_EQ(feed(&rx, reply, n, &cmd, &len, &payload), BOOT_FRAME_COMPLETE);
  EXPECT_EQ(cmd, BOOT_STATUS_BAD_ADDR); /* the reason is the status */
}

} // namespace
} // namespace node