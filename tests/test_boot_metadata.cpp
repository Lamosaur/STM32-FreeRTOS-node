/* Tests for boot/metadata.h — the record that decides which slot runs.
 *
 * The rules this file pins are the ones the bootloader's one job depends on: an erased
 * or half-written slip must be refused, a fresh slip must be preferred over a stale one,
 * and a single corrupted field must never make a slip look valid. The wraparound case
 * matters because the sequence is a plain 32-bit counter and this node can run for years.
 */
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <stddef.h>

#include "boot/crc.h"
#include "boot/layout.h"
#include "boot/metadata.h"

namespace {

// A slip with sane values, checksummed, ready for a test to mutate.
boot_metadata_t make_slip(uint32_t sequence, uint32_t slot = BOOT_SLOT_A,
                          uint32_t size = 0x400u, uint32_t crc = 0x12345678u,
                          uint32_t version = 7u) {
  boot_metadata_t m;
  std::memset(&m, 0, sizeof(m));
  m.magic = BOOT_METADATA_MAGIC;
  m.sequence = sequence;
  m.active_slot = slot;
  m.image_size = size;
  m.image_crc32 = crc;
  m.version = version;
  boot_metadata_finalize(&m);
  return m;
}

TEST(BootMetadataTest, StructGeometryIsStableAndChecksummedUpToTheChecksum) {
  // The bootloader's flash writer programs this struct into a 1 KB page; the exact size
  // is a promise the rest of the system relies on, and the checksum must cover exactly
  // the bytes before the crc16 field — not the field itself, not a byte of padding.
  EXPECT_EQ(sizeof(boot_metadata_t), 32u);
  EXPECT_EQ(offsetof(boot_metadata_t, crc16), 28u);
  EXPECT_EQ(BOOT_METADATA_CHECKSUM_OFFSET, 28u);
}

TEST(BootMetadataTest, AnErasedSlipIsRefused) {
  // 0xFF everywhere is what an erased page reads back as, and a partially programmed
  // slip contains runs of 0xFF too. The magic check is the first line of defence.
  boot_metadata_t erased;
  std::memset(&erased, 0xFF, sizeof(erased));
  EXPECT_FALSE(boot_metadata_validate(&erased));
}

TEST(BootMetadataTest, AValidSlipPassesAndAMutatedFieldBreaksIt) {
  const boot_metadata_t slip = make_slip(1u);
  EXPECT_TRUE(boot_metadata_validate(&slip));

  // Corrupt one byte of image_size after the checksum was computed: the slip must fail,
  // because this is exactly a torn or half-written slip.
  boot_metadata_t mutated = slip;
  mutated.image_size += 1u;
  EXPECT_FALSE(boot_metadata_validate(&mutated));

  // Same for the sequence — a checksum that did not cover it would let a forged newer
  // slip win.
  mutated = slip;
  mutated.sequence += 1u;
  EXPECT_FALSE(boot_metadata_validate(&mutated));
}

TEST(BootMetadataTest, AValidSlipWithGeometryViolationsIsRefused) {
  boot_metadata_t bad_slot = make_slip(1u, 3u);          // not a slot
  EXPECT_FALSE(boot_metadata_validate(&bad_slot));

  boot_metadata_t too_big = make_slip(1u, BOOT_SLOT_A, BOOT_SLOT_SIZE + 0x400u);
  EXPECT_FALSE(boot_metadata_validate(&too_big));

  boot_metadata_t zero_size = make_slip(1u, BOOT_SLOT_A, 0u);
  EXPECT_FALSE(boot_metadata_validate(&zero_size));

  boot_metadata_t misaligned = make_slip(1u, BOOT_SLOT_A, 0x402u);
  EXPECT_FALSE(boot_metadata_validate(&misaligned));
}

TEST(BootMetadataTest, NewerPrefersTheHigherSequence) {
  const boot_metadata_t old_slip = make_slip(1u);
  const boot_metadata_t new_slip = make_slip(2u);
  EXPECT_EQ(boot_metadata_newer(&old_slip, &new_slip), &new_slip);
  EXPECT_EQ(boot_metadata_newer(&new_slip, &old_slip), &new_slip);

  // Equal sequences cannot legitimately happen (each commit writes old+1), and if they
  // do either answer is as good as the other — but it must be one of the two, not NULL.
  const boot_metadata_t twin = make_slip(9u);
  const boot_metadata_t other_twin = make_slip(9u);
  const boot_metadata_t *winner = boot_metadata_newer(&twin, &other_twin);
  EXPECT_TRUE(winner == &twin || winner == &other_twin);
}

TEST(BootMetadataTest, NewerIsWraparoundSafe) {
  // The classic unsigned-counter comparison: after 2^32 - 10 commits the sequence wraps,
  // and "5" is still newer than 0xFFFFFFF0 because the *difference* is small when read
  // modulo 2^32. A signed comparison would get this wrong.
  const boot_metadata_t wrapped_new = make_slip(5u);
  const boot_metadata_t wrapped_old = make_slip(0xFFFFFFF0u);
  EXPECT_EQ(boot_metadata_newer(&wrapped_old, &wrapped_new), &wrapped_new);
}

TEST(BootMetadataTest, NewerSkipsInvalidSlips) {
  boot_metadata_t valid = make_slip(1u);
  boot_metadata_t invalid;
  std::memset(&invalid, 0, sizeof(invalid)); // magic is 0: refused

  EXPECT_EQ(boot_metadata_newer(&invalid, &valid), &valid);
  EXPECT_EQ(boot_metadata_newer(&valid, &invalid), &valid);

  boot_metadata_t invalid2;
  std::memset(&invalid2, 0xFF, sizeof(invalid2));
  EXPECT_EQ(boot_metadata_newer(&invalid, &invalid2), nullptr);
}

TEST(BootMetadataTest, FinalizeFollowsFieldChanges) {
  // The bootloader's writer sets fields, calls finalize, then programs the slip. A
  // change after finalize must invalidate the checksum — which is exactly the hook the
  // writer relies on to notice programmer errors in tests.
  boot_metadata_t slip = make_slip(1u);
  EXPECT_TRUE(boot_metadata_validate(&slip));
  slip.version += 1u;
  EXPECT_FALSE(boot_metadata_validate(&slip));
  boot_metadata_finalize(&slip);
  EXPECT_TRUE(boot_metadata_validate(&slip));
}

// ---------------------------------------------------------------------------
// boot_metadata_next — the commit's slip construction (Lesson 4b).
// ---------------------------------------------------------------------------

TEST(BootMetadataTest, NextWritesToTheLoserPageAndIncrementsTheSequence) {
  const boot_metadata_t slip_a = make_slip(10u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(11u, BOOT_SLOT_B);

  boot_metadata_t next;
  const uint32_t page =
      boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_A, 0x1000u, 0xDEADBEEFu, 42u, &next);
  EXPECT_EQ(page, BOOT_METADATA_A_BASE); /* B is winning, so the new slip goes to A */
  EXPECT_EQ(next.sequence, 12u);         /* winner(11) + 1 */
  EXPECT_EQ(next.active_slot, BOOT_SLOT_A);
  EXPECT_EQ(next.image_size, 0x1000u);
  EXPECT_EQ(next.image_crc32, 0xDEADBEEFu);
  EXPECT_EQ(next.version, 42u);
  EXPECT_TRUE(boot_metadata_validate(&next));
}

TEST(BootMetadataTest, NextAlternatesToTheOtherPageWhenAIsWinning) {
  const boot_metadata_t slip_a = make_slip(10u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(9u, BOOT_SLOT_B);

  boot_metadata_t next;
  const uint32_t page =
      boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_B, 0x1000u, 1u, 2u, &next);
  EXPECT_EQ(page, BOOT_METADATA_B_BASE); /* A is winning, so the new slip goes to B */
  EXPECT_EQ(next.sequence, 11u);
  EXPECT_EQ(next.active_slot, BOOT_SLOT_B);
}

TEST(BootMetadataTest, FirstEverCommitGoesToPageAWithSequenceOne) {
  boot_metadata_t erased_a, erased_b;
  std::memset(&erased_a, 0xFF, sizeof(erased_a));
  std::memset(&erased_b, 0xFF, sizeof(erased_b));

  boot_metadata_t next;
  const uint32_t page =
      boot_metadata_next(&erased_a, &erased_b, BOOT_SLOT_B, 0x800u, 7u, 1u, &next);
  EXPECT_EQ(page, BOOT_METADATA_A_BASE);
  EXPECT_EQ(next.sequence, 1u);
  EXPECT_EQ(next.active_slot, BOOT_SLOT_B);
  EXPECT_TRUE(boot_metadata_validate(&next));
}

TEST(BootMetadataTest, NextRefusesAnInvalidCommit) {
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  boot_metadata_t next;

  EXPECT_EQ(boot_metadata_next(&slip_a, &slip_b, 0u, 0x1000u, 1u, 1u, &next), 0u);
  EXPECT_EQ(boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_A, 0u, 1u, 1u, &next), 0u);
  EXPECT_EQ(boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_A, BOOT_SLOT_SIZE + 1u, 1u, 1u,
                               &next), 0u);
  EXPECT_EQ(boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_A, 0x402u, 1u, 1u, &next), 0u);
  EXPECT_EQ(boot_metadata_next(&slip_a, &slip_b, BOOT_SLOT_A, 0x1000u, 1u, 1u, nullptr), 0u);
}

TEST(BootMetadataTest, NextSurvivesSequenceWrap) {
  // A sequence at the top of the 32-bit range must still produce a valid, higher-by-
  // wraparound sequence the validator accepts.
  const boot_metadata_t slip_a = make_slip(0xFFFFFFFEu, BOOT_SLOT_A);
  boot_metadata_t next;
  const uint32_t page =
      boot_metadata_next(&slip_a, &slip_a, BOOT_SLOT_B, 0x1000u, 1u, 1u, &next);
  EXPECT_NE(page, 0u);
  EXPECT_EQ(next.sequence, 0xFFFFFFFFu);
  EXPECT_TRUE(boot_metadata_validate(&next));
}

// ---------------------------------------------------------------------------
// Layout invariants (NFR-2.9): the map must tile exactly and stay page-aligned.
// Editing layout.h into nonsense fails here rather than on a bricked board.
// ---------------------------------------------------------------------------
static_assert(BOOT_FLASH_PAGE_SIZE == 0x400u, "page size is part of the geometry");
static_assert(BOOT_BOOTLOADER_SIZE % BOOT_FLASH_PAGE_SIZE == 0u, "regions are page-aligned");
static_assert(BOOT_SLOT_SIZE % BOOT_FLASH_PAGE_SIZE == 0u, "regions are page-aligned");
static_assert(BOOT_METADATA_SIZE % BOOT_FLASH_PAGE_SIZE == 0u, "regions are page-aligned");

static_assert(BOOT_SLOT_A_BASE == BOOT_BOOTLOADER_BASE + BOOT_BOOTLOADER_SIZE,
              "slot A follows the bootloader without a gap or overlap");
static_assert(BOOT_SLOT_B_BASE == BOOT_SLOT_A_BASE + BOOT_SLOT_SIZE,
              "slot B follows slot A without a gap or overlap");
static_assert(BOOT_METADATA_A_BASE == BOOT_SLOT_B_BASE + BOOT_SLOT_SIZE,
              "metadata A follows slot B without a gap or overlap");
static_assert(BOOT_METADATA_B_BASE == BOOT_METADATA_A_BASE + BOOT_METADATA_SIZE,
              "metadata B follows metadata A without a gap or overlap");
static_assert(BOOT_SPARE_BASE == BOOT_METADATA_B_BASE + BOOT_METADATA_SIZE,
              "spare follows metadata B without a gap or overlap");
static_assert(BOOT_SPARE_BASE + BOOT_SPARE_SIZE == BOOT_FLASH_END,
              "the regions tile exactly to the end of flash");
static_assert(BOOT_SLOT_SIZE * 2u + BOOT_BOOTLOADER_SIZE + 2u * BOOT_METADATA_SIZE +
                      BOOT_SPARE_SIZE == BOOT_FLASH_SIZE,
              "the regions tile exactly to the end of flash (size view)");

} // namespace