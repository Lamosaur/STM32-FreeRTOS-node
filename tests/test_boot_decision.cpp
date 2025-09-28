/* Tests for boot/decision.h — the fallback ladder that is the bootloader's whole job.
 *
 * Every rung is driven here: the update marker winning while its window is open, the
 * marker being ignored once the window expires, the newest slip's image being preferred,
 * the rollback to the loser slot when the newest image fails to verify, and the final
 * "stay in the bootloader" states that FR-2.2's letter demands. Verification is stubbed
 * per slot via the context pointer, so each test controls exactly which image "hashes".
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>

#include "boot/decision.h"
#include "boot/layout.h"
#include "boot/metadata.h"

namespace {

boot_metadata_t make_slip(uint32_t sequence, uint32_t slot) {
  boot_metadata_t m;
  std::memset(&m, 0, sizeof(m));
  m.magic = BOOT_METADATA_MAGIC;
  m.sequence = sequence;
  m.active_slot = slot;
  m.image_size = 0x400u;
  m.image_crc32 = 0x12345678u;
  boot_metadata_finalize(&m);
  return m;
}

// The verification stub: "does the slot this slip names hash correctly?" answered from
// a per-slot table carried through the context pointer.
struct VerifyTable {
  bool slot_a_ok;
  bool slot_b_ok;
};

bool verify_stub(const boot_metadata_t *slip, void *ctx) {
  const VerifyTable &table = *static_cast<const VerifyTable *>(ctx);
  if (slip->active_slot == BOOT_SLOT_A) {
    return table.slot_a_ok;
  }
  return table.slot_b_ok;
}

struct Outcome {
  boot_decision_t decision;
  boot_decision_reason_t reason;
};

TEST(BootDecisionTest, AnOpenUpdateWindowAlwaysWins) {
  // The app asked for an update and the window has not expired: stay and accept frames,
  // even though a perfectly good image is present. That is the point of the marker.
  boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  VerifyTable table{true, true};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, /*update_requested=*/true,
                  /*update_window_expired=*/false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_STAY_BOOTLOADER);
  EXPECT_EQ(out.reason, BOOT_REASON_UPDATE_REQUESTED);
}

TEST(BootDecisionTest, AnExpiredUpdateWindowIsIgnored) {
  // The anti-brick: the marker is stale (a host that never finished), so the ladder
  // decides normally and the verified app runs.
  boot_metadata_t slip_a = make_slip(2u, BOOT_SLOT_A);
  boot_metadata_t slip_b = make_slip(1u, BOOT_SLOT_B);
  VerifyTable table{true, false};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, /*update_requested=*/true,
                  /*update_window_expired=*/true, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_RUN_SLOT_A);
  EXPECT_EQ(out.reason, BOOT_REASON_NEWEST_SLOT);
}

TEST(BootDecisionTest, TheNewestSlipRunsWhenItsImageVerifies) {
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  VerifyTable table{false, true};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, false, false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_RUN_SLOT_B);
  EXPECT_EQ(out.reason, BOOT_REASON_NEWEST_SLOT);
}

TEST(BootDecisionTest, AFailedNewestImageRollsBackToTheLoserSlot) {
  // The newest slip (sequence 2, slot B) passes validation but its image does not hash.
  // The ladder falls to the loser slip (sequence 1, slot A) whose image does — the node
  // runs last week's firmware rather than nothing.
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  VerifyTable table{true, false};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, false, false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_RUN_SLOT_A);
  EXPECT_EQ(out.reason, BOOT_REASON_ROLLBACK_SLOT);
}

TEST(BootDecisionTest, RollbackOnlyConsidersTheWinnerWhenSelectingTheNewest) {
  // The loser must not win just because its image verifies while the newest one fails —
  // the ladder tries the newest first, and only the LOSER (which is the other slip) is
  // the rollback candidate. A test that confuses "newest" and "loser" ordering fails
  // here even though both images verify.
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  VerifyTable table{true, true};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, false, false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_RUN_SLOT_B); // newest, not the rollback
  EXPECT_EQ(out.reason, BOOT_REASON_NEWEST_SLOT);
}

TEST(BootDecisionTest, BothImagesFailingStaysInTheBootloader) {
  // FR-2.2's letter: a failed image leaves the node in the bootloader.
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  VerifyTable table{false, false};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, false, false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_STAY_BOOTLOADER);
  EXPECT_EQ(out.reason, BOOT_REASON_NO_VALID_IMAGE);
}

TEST(BootDecisionTest, NoValidSlipsStaysInTheBootloader) {
  boot_metadata_t erased;
  std::memset(&erased, 0xFF, sizeof(erased));
  VerifyTable table{true, true};

  const boot_outcome_t out =
      boot_decide(&erased, &erased, false, false, verify_stub, &table);
  EXPECT_EQ(out.decision, BOOT_DECISION_STAY_BOOTLOADER);
  EXPECT_EQ(out.reason, BOOT_REASON_NO_VALID_SLIP);
}

TEST(BootDecisionTest, AMissingVerifierRunsNothing) {
  // Conservative by contract: a bootloader with no image checker must not run images it
  // cannot check.
  const boot_metadata_t slip_a = make_slip(1u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  const boot_outcome_t out = boot_decide(&slip_a, &slip_b, false, false, nullptr, nullptr);
  EXPECT_EQ(out.decision, BOOT_DECISION_STAY_BOOTLOADER);
  EXPECT_EQ(out.reason, BOOT_REASON_NO_VALID_IMAGE);
}

TEST(BootDecisionTest, AnInvalidLoserCannotBeARollbackTarget) {
  // Two slips, the newest valid but failing verification, the loser a garbage slip that
  // does not validate: no rollback is possible, so the node stays. The garbage slip's
  // image_size must never reach the verifier.
  const boot_metadata_t slip_b = make_slip(2u, BOOT_SLOT_B);
  boot_metadata_t garbage;
  std::memset(&garbage, 0, sizeof(garbage)); // magic 0: invalid

  const boot_outcome_t out =
      boot_decide(&garbage, &slip_b, false, false, verify_stub, &garbage);
  EXPECT_EQ(out.decision, BOOT_DECISION_STAY_BOOTLOADER);
  EXPECT_EQ(out.reason, BOOT_REASON_NO_VALID_IMAGE);
}

TEST(BootDecisionTest, EqualSequencesDecideDeterministicallyOneWayOrTheOther) {
  // Both slips are valid with equal sequence (impossible from the writer, but a torn
  // slip could fake it): the decision must be a run of one of them, never a stay that
  // looks like "nothing valid".
  const boot_metadata_t slip_a = make_slip(9u, BOOT_SLOT_A);
  const boot_metadata_t slip_b = make_slip(9u, BOOT_SLOT_B);
  VerifyTable table{true, true};

  const boot_outcome_t out =
      boot_decide(&slip_a, &slip_b, false, false, verify_stub, &table);
  EXPECT_TRUE(out.decision == BOOT_DECISION_RUN_SLOT_A ||
              out.decision == BOOT_DECISION_RUN_SLOT_B);
}

} // namespace