/* Tests for boot_crc16 and boot_crc32 — the two checksums of the Phase 2 update path.
 *
 * The canonical check values ("123456789" gives 0x29B1 for CRC-16/CCITT-FALSE and
 * 0xCBF43926 for CRC-32) pin the *algorithm*, which is what a table typo would break:
 * both values are independently defined by the CRC standards, not by this implementation.
 * The extra vectors ("hello") are computed independently of the code under test and guard
 * against the narrower mistake of a table that happens to agree with the check string
 * while being wrong elsewhere. The empty-input behaviour is pinned because the bootloader
 * computes CRCs over live flash reads, and "no bytes were read" must be distinguishable
 * from a zero-length image, not crash.
 */
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "boot/crc.h"

namespace {

TEST(BootCrcTest, Crc16MatchesTheCcittFalseCheckValue) {
  const uint8_t data[] = "123456789";
  EXPECT_EQ(boot_crc16(data, sizeof(data) - 1u), 0x29B1u);
}

TEST(BootCrcTest, Crc32MatchesTheIeeeCheckValue) {
  const uint8_t data[] = "123456789";
  EXPECT_EQ(boot_crc32(data, sizeof(data) - 1u), 0xCBF43926u);
}

TEST(BootCrcTest, ExtraVectorGuardsAgainstATableThatOnlyMatchesTheCheckValue) {
  const uint8_t data[] = "hello";
  EXPECT_EQ(boot_crc16(data, sizeof(data) - 1u), 0xD26Eu);
  EXPECT_EQ(boot_crc32(data, sizeof(data) - 1u), 0x3610A686u);
}

TEST(BootCrcTest, EmptyInputsAreWellDefinedAndDistinct) {
  // CRC-16/CCITT-FALSE of no bytes is the initial value, 0xFFFF.
  EXPECT_EQ(boot_crc16(nullptr, 0), 0xFFFFu);
  // CRC-32 of no bytes is init 0xFFFFFFFF XOR final 0xFFFFFFFF = 0.
  EXPECT_EQ(boot_crc32(nullptr, 0), 0u);
  EXPECT_NE(boot_crc16(nullptr, 0), boot_crc32(nullptr, 0));
}

TEST(BootCrcTest, NullPointerWithZeroLengthIsSafe) {
  // The bootloader may compute a CRC over a flash region that is present in the map but
  // empty in practice; the loop must not dereference anything when len == 0.
  EXPECT_EQ(boot_crc32(nullptr, 0), 0u);
  EXPECT_EQ(boot_crc16(nullptr, 0), 0xFFFFu);
}

TEST(BootCrcTest, TheTwoCrCsDisagreeOnNonTrivialInput) {
  // A guard against copy-paste between the two implementations.
  const uint8_t data[] = "not the same input";
  const size_t len = sizeof(data) - 1u;
  EXPECT_NE(static_cast<uint32_t>(boot_crc16(data, len)), boot_crc32(data, len));
}

} // namespace