/* Tests for the I2C bus release sequence (FR-0.3).
 *
 * FR-0.3 is verified on hardware by holding the bus and watching the recovery on a
 * scope. That capture only tells you the electrical behaviour is right; it does not
 * tell you the *sequence* is right, and a scope is a slow way to iterate on an edge
 * order. These tests pin the sequence down first, so the scope run confirms a design
 * that is already known to be correct.
 */
#include <gtest/gtest.h>

#include <vector>

#include "app/board.h"
#include "app/i2c_recovery.h"
#include "support/fake_i2c.h"

namespace node {
namespace {

using test::BusLogEntry;
using test::BusOp;
using test::FakeI2c;
using test::kNeverRelease;

// First index in the log at which `op` appears, or -1.
int first_index(const std::vector<BusLogEntry> &log, BusOp op) {
  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i].op == op) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// Rising SCL edges emitted before the STOP starts. The STOP is the first moment the
// master pulls SDA low, so everything before that is the release clocking.
int release_pulses(const std::vector<BusLogEntry> &log) {
  int pulses = 0;
  for (const BusLogEntry &e : log) {
    if (e.op == BusOp::SdaWrite && !e.level) {
      break;
    }
    if (e.op == BusOp::SclWrite && e.level) {
      ++pulses;
    }
  }
  return pulses;
}

// Index of the first SDA write at `level`, or -1.
int first_sda_write(const std::vector<BusLogEntry> &log, bool level) {
  for (size_t i = 0; i < log.size(); ++i) {
    if (log[i].op == BusOp::SdaWrite && log[i].level == level) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

TEST(I2cRecovery, IdleBusIsLeftAlone) {
  FakeI2c bus;
  I2cRecovery recovery(bus.iface());

  EXPECT_EQ(recovery.run(), RecoveryOutcome::NotNeeded);

  // Nothing may be driven on a bus that is already idle: stray edges on a healthy bus
  // are a real fault, because a listening slave can interpret them as a start.
  EXPECT_EQ(bus.count(BusOp::RecoverBegin), 0);
  EXPECT_EQ(bus.count(BusOp::SclWrite), 0);
  EXPECT_EQ(bus.count(BusOp::SdaWrite), 0);
  EXPECT_EQ(recovery.stats().not_needed, 1u);
  EXPECT_EQ(recovery.stats().attempts, 1u);
}

TEST(I2cRecovery, EmitsNineClocksThenAStop) {
  FakeI2c bus;
  bus.hold_sda_low_for_clocks(4); // the slave lets go partway through
  I2cRecovery recovery(bus.iface());

  ASSERT_EQ(recovery.run(), RecoveryOutcome::Recovered);

  const std::vector<BusLogEntry> log = bus.log();

  // Exactly nine release pulses, all of them, even though SDA came up after four.
  // Emitting the full nine keeps the sequence deterministic for the scope capture.
  EXPECT_EQ(release_pulses(log), kRecoveryClocks);
  EXPECT_EQ(release_pulses(log), 9);
  EXPECT_EQ(recovery.last_clocks_emitted(), 9);

  // A STOP, not just nine clocks: a slave that is released without a STOP can still be
  // left mid-transaction and will hold the bus again. Exactly one, because the sequence
  // parks SCL low before releasing SDA — releasing SDA while SCL is high would itself
  // be a STOP, and then the edge sequence would depend on the slave's prior state.
  EXPECT_EQ(bus.stop_conditions(), 1);

  // SCL edges: one park-low, nine pulses at two edges each, and the STOP's raise.
  EXPECT_EQ(bus.count(BusOp::SclWrite), 1 + 9 * 2 + 1);

  // The peripheral must be handed over before any bit is driven and restored after.
  const int begin = first_index(log, BusOp::RecoverBegin);
  const int first_scl = first_index(log, BusOp::SclWrite);
  const int end = first_index(log, BusOp::RecoverEnd);
  ASSERT_GE(begin, 0);
  ASSERT_GE(first_scl, 0);
  ASSERT_GE(end, 0);
  EXPECT_LT(begin, first_scl);
  EXPECT_LT(first_scl, end);
  EXPECT_GT(end, first_sda_write(log, false));

  // SCL is parked low before SDA is released, which is what keeps the STOP count at
  // one. Clocking while the master still held SDA low would drive against the slave.
  ASSERT_LT(first_scl, static_cast<int>(log.size()));
  EXPECT_FALSE(log[static_cast<size_t>(first_scl)].level);
  EXPECT_LT(first_scl, first_sda_write(log, true));
  EXPECT_TRUE(bus.master_released_sda());

  EXPECT_EQ(recovery.stats().recovered, 1u);
  EXPECT_TRUE(recovery.last_sda_released());
}

TEST(I2cRecovery, EveryEdgeIsSeparatedByTheBusDelay) {
  FakeI2c bus;
  bus.hold_sda_low_for_clocks(2);
  I2cRecovery recovery(bus.iface(), kRecoveryClocks, kRecoveryHalfPeriodUs);

  ASSERT_EQ(recovery.run(), RecoveryOutcome::Recovered);

  // SCL was parked low, SDA was released, then 9 pulses x 2 edges, then the STOP's
  // three edges: every one separated by the configured half period.
  EXPECT_EQ(bus.count(BusOp::DelayUs), 2 + 9 * 2 + 3);

  for (const BusLogEntry &e : bus.log()) {
    if (e.op == BusOp::DelayUs) {
      EXPECT_EQ(e.arg, kRecoveryHalfPeriodUs);
    }
  }
}

TEST(I2cRecovery, SlaveHoldingSdaForeverIsReported) {
  FakeI2c bus;
  bus.hold_sda_low_for_clocks(kNeverRelease);
  I2cRecovery recovery(bus.iface());

  EXPECT_EQ(recovery.run(), RecoveryOutcome::SdaStuckLow);

  // The attempt still ran to completion and the peripheral was restored, because a
  // slave that needs its own reset must not also leave the peripheral detached.
  EXPECT_EQ(bus.count(BusOp::RecoverBegin), 1);
  EXPECT_EQ(bus.count(BusOp::RecoverEnd), 1);
  EXPECT_EQ(release_pulses(bus.log()), 9);
  EXPECT_FALSE(recovery.last_sda_released());
  EXPECT_EQ(recovery.stats().sda_stuck_low, 1u);

  BusLines lines{};
  ASSERT_TRUE(recovery.lines(&lines));
  EXPECT_FALSE(lines.sda_high);
  EXPECT_FALSE(lines.idle());
}

TEST(I2cRecovery, PeripheralTakeoverFailureIsReported) {
  FakeI2c bus;
  bus.hold_sda_low_for_clocks(kNeverRelease);
  bus.set_recover_begin_ok(false);
  I2cRecovery recovery(bus.iface());

  EXPECT_EQ(recovery.run(), RecoveryOutcome::PeripheralFail);
  EXPECT_EQ(bus.count(BusOp::SclWrite), 0); // never drove a bit it did not own
  EXPECT_EQ(bus.count(BusOp::RecoverEnd), 0);
  EXPECT_EQ(recovery.stats().peripheral_fail, 1u);
}

TEST(I2cRecovery, PeripheralRestoreFailureIsReported) {
  FakeI2c bus;
  bus.hold_sda_low_for_clocks(3);
  bus.set_recover_end_ok(false);
  I2cRecovery recovery(bus.iface());

  // Even though the lines came back idle, the peripheral is not usable, so this is a
  // failure rather than a recovery: reporting success here would leave the arbiter
  // issuing transfers on a detached peripheral.
  EXPECT_EQ(recovery.run(), RecoveryOutcome::PeripheralFail);
  EXPECT_EQ(recovery.stats().peripheral_fail, 1u);
  EXPECT_EQ(recovery.stats().recovered, 0u);
}

TEST(I2cRecovery, StatsAccumulateAcrossAttempts) {
  FakeI2c bus;
  I2cRecovery recovery(bus.iface());

  EXPECT_EQ(recovery.run(), RecoveryOutcome::NotNeeded);

  bus.hold_sda_low_for_clocks(1);
  EXPECT_EQ(recovery.run(), RecoveryOutcome::Recovered);

  bus.hold_sda_low_for_clocks(kNeverRelease);
  EXPECT_EQ(recovery.run(), RecoveryOutcome::SdaStuckLow);

  EXPECT_EQ(recovery.stats().attempts, 3u);
  EXPECT_EQ(recovery.stats().not_needed, 1u);
  EXPECT_EQ(recovery.stats().recovered, 1u);
  EXPECT_EQ(recovery.stats().sda_stuck_low, 1u);
  EXPECT_EQ(recovery.stats().peripheral_fail, 0u);
}

TEST(I2cRecovery, OutcomeNamesAreStable) {
  // These strings go into the telemetry stream and the bring-up log, so they are part
  // of the interface rather than a debug detail.
  EXPECT_STREQ(to_string(RecoveryOutcome::NotNeeded), "not-needed");
  EXPECT_STREQ(to_string(RecoveryOutcome::Recovered), "recovered");
  EXPECT_STREQ(to_string(RecoveryOutcome::SdaStuckLow), "sda-stuck-low");
  EXPECT_STREQ(to_string(RecoveryOutcome::PeripheralFail), "peripheral-fail");
}

} // namespace
} // namespace node
