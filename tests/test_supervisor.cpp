/* Tests for the supervisor (FR-0.4).
 *
 * The requirement has two halves that must be verified together: a task feeds the
 * watchdog, and the same logic notices when a monitored task has stopped. Testing only
 * the first half would be satisfied by a loop that feeds unconditionally, which is the
 * failure mode FR-0.4 exists to prevent — a node that looks alive because something is
 * still running while a subsystem is dead.
 *
 * So every test here asserts on the *withholding* of feeds as well as the feeding.
 *
 * Timing discipline: the virtual clock is set to a non-zero base *before* a task is
 * watched, because registration seeds that task's heartbeat from the current clock.
 * Registering at t=0 and then jumping the clock forward would blow the deadline
 * immediately, which is a mistake in the test rather than in the supervisor.
 */
#include <gtest/gtest.h>

#include "app/board.h"
#include "app/supervisor.h"
#include "support/fake_rtos.h"
#include "support/fake_watchdog.h"

namespace node {
namespace {

using test::FakeRtos;
using test::FakeWatchdog;

// A start-of-run base time, so that "watch at 0" is never accidental.
constexpr uint32_t kBase = 1000;

class SupervisorTest : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_TRUE(watchdog.start(kWatchdogTimeoutMs)); }

  FakeRtos rtos;
  FakeWatchdog watchdog;
  Supervisor supervisor{rtos.iface(), watchdog.iface()};
};

TEST_F(SupervisorTest, FeedsTheWatchdogWhileEveryWatchedTaskIsHealthy) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));
  ASSERT_TRUE(supervisor.watch(TaskId::Display, 200));

  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 1);

  rtos.set_now_ms(kBase + 100);
  supervisor.heartbeat(TaskId::Sensor);
  supervisor.heartbeat(TaskId::Display);
  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 2);

  EXPECT_EQ(supervisor.stats().misses, 0u);
  EXPECT_EQ(supervisor.stats().feeds_withheld, 0u);
}

TEST_F(SupervisorTest, RegistrationDoesNotTripTheWatchdogAtStartUp) {
  // The window between start-up and the first time each task runs must not look like a
  // fault, or every boot would reset the node before it finished initialising.
  rtos.set_now_ms(0);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100)); // deadline 300 ms
  ASSERT_TRUE(supervisor.watch(TaskId::Comms, 50));   // deadline 150 ms

  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 1);

  rtos.set_now_ms(149); // inside both deadlines
  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 2);

  // The tighter task is now past its deadline even though the looser one is fine:
  // the deadline is per task, not global.
  rtos.set_now_ms(151);
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 2);
  EXPECT_EQ(supervisor.trip().task, TaskId::Comms);
}

TEST_F(SupervisorTest, WithholdsTheFeedWhenATaskMissesItsDeadline) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100)); // deadline 300 ms

  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 1);

  rtos.set_now_ms(kBase + 300); // exactly on the deadline: still alive
  EXPECT_TRUE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 2);

  rtos.set_now_ms(kBase + 301); // one millisecond past it
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 2); // no further feed

  EXPECT_EQ(supervisor.stats().misses, 1u);
  EXPECT_EQ(supervisor.stats().feeds_withheld, 1u);
  EXPECT_TRUE(supervisor.trip().tripped);
  EXPECT_EQ(supervisor.trip().task, TaskId::Sensor);
  EXPECT_EQ(supervisor.trip().lateness_ms, 1u);
  EXPECT_EQ(supervisor.trip().at_ms, kBase + 301);
}

TEST_F(SupervisorTest, AHeartbeatPushesTheDeadlineOut) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));

  EXPECT_TRUE(supervisor.run());

  rtos.set_now_ms(kBase + 250);
  supervisor.heartbeat(TaskId::Sensor); // the task is alive again
  EXPECT_TRUE(supervisor.run());

  rtos.set_now_ms(kBase + 549); // age 299
  EXPECT_TRUE(supervisor.run());

  rtos.set_now_ms(kBase + 551); // age 301
  EXPECT_FALSE(supervisor.run());

  EXPECT_EQ(supervisor.stats().feeds, 3u);
}

TEST_F(SupervisorTest, HeartbeatsAreCounted) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Comms, 50));

  for (int i = 0; i < 7; ++i) {
    supervisor.heartbeat(TaskId::Comms);
  }

  const TaskHealth *health = supervisor.task(TaskId::Comms);
  ASSERT_NE(health, nullptr);
  EXPECT_EQ(health->beats, 7u);
  EXPECT_TRUE(health->ever_beat);
  EXPECT_EQ(health->deadline_ms, 150u);
}

TEST_F(SupervisorTest, OneDeadTaskStopsTheNodeEvenWhileTheOthersBeat) {
  // The whole point of FR-0.4: one dead subsystem must not be masked by healthy ones.
  rtos.set_now_ms(0);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));  // deadline 300
  ASSERT_TRUE(supervisor.watch(TaskId::Display, 100)); // deadline 300
  ASSERT_TRUE(supervisor.watch(TaskId::Comms, 100));   // deadline 300

  EXPECT_TRUE(supervisor.run());
  uint32_t feeds = 1;

  // Sensor and comms keep beating; display never does.
  for (uint32_t t = 100; t <= 300; t += 100) {
    rtos.set_now_ms(t);
    supervisor.heartbeat(TaskId::Sensor);
    supervisor.heartbeat(TaskId::Comms);
    EXPECT_TRUE(supervisor.run()) << "t=" << t << ", display age " << t;
    ++feeds;
  }

  rtos.set_now_ms(400); // display age 400, past its 300 ms deadline
  supervisor.heartbeat(TaskId::Sensor);
  supervisor.heartbeat(TaskId::Comms);
  EXPECT_FALSE(supervisor.run());

  EXPECT_EQ(watchdog.feeds(), static_cast<int>(feeds));
  EXPECT_EQ(supervisor.trip().task, TaskId::Display);
}

TEST_F(SupervisorTest, TripLatchesTheFirstOffender) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));
  ASSERT_TRUE(supervisor.watch(TaskId::Display, 100));

  EXPECT_TRUE(supervisor.run());

  // The sensor goes missing first; the display is still beating.
  rtos.set_now_ms(kBase + 400);
  supervisor.heartbeat(TaskId::Display);
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(supervisor.trip().task, TaskId::Sensor);
  EXPECT_EQ(supervisor.trip().at_ms, kBase + 400);

  // Much later the display is by far the more overdue task, but the original cause is
  // what gets reported, because that is the fault worth explaining after a reset.
  rtos.set_now_ms(kBase + 4000);
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(supervisor.trip().task, TaskId::Sensor);
  EXPECT_EQ(supervisor.trip().at_ms, kBase + 400);
}

TEST_F(SupervisorTest, TracksWorstLateness) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));

  EXPECT_TRUE(supervisor.run());

  rtos.set_now_ms(kBase + 500); // 200 ms late
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(supervisor.stats().worst_lateness_ms, 200u);

  rtos.set_now_ms(kBase + 1000); // 700 ms late
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(supervisor.stats().worst_lateness_ms, 700u);

  // A task that comes back and then misses by less must not lower the recorded worst
  // case: the worst case is the evidence, not the latest sample.
  rtos.set_now_ms(kBase + 1000);
  supervisor.heartbeat(TaskId::Sensor);
  rtos.set_now_ms(kBase + 1400); // 100 ms late
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(supervisor.stats().worst_lateness_ms, 700u);
}

TEST_F(SupervisorTest, HealthStatesDistinguishLateFromMissingFromUnwatched) {
  rtos.set_now_ms(0);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100)); // deadline 300

  EXPECT_EQ(supervisor.health(TaskId::Sensor), HealthState::Ok);
  EXPECT_EQ(supervisor.health(TaskId::Display), HealthState::Never); // never watched

  rtos.set_now_ms(150);
  EXPECT_EQ(supervisor.health(TaskId::Sensor), HealthState::Late);

  rtos.set_now_ms(200);
  supervisor.heartbeat(TaskId::Sensor);
  EXPECT_EQ(supervisor.health(TaskId::Sensor), HealthState::Ok);

  rtos.set_now_ms(501);
  EXPECT_EQ(supervisor.health(TaskId::Sensor), HealthState::Missing);
}

TEST_F(SupervisorTest, ResetClearsTheTripLatchAndCounters) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));
  ASSERT_TRUE(supervisor.run());
  rtos.set_now_ms(kBase + 400);
  ASSERT_FALSE(supervisor.run());
  ASSERT_TRUE(supervisor.trip().tripped);

  supervisor.reset();

  EXPECT_FALSE(supervisor.trip().tripped);
  EXPECT_EQ(supervisor.stats().polls, 0u);
  EXPECT_EQ(supervisor.task(TaskId::Sensor), nullptr); // no longer watched
}

TEST_F(SupervisorTest, RejectsAnInvalidWatch) {
  EXPECT_FALSE(supervisor.watch(TaskId::Count, 100));
  EXPECT_EQ(supervisor.task(TaskId::Count), nullptr);
}

TEST_F(SupervisorTest, WithholdsTheFeedWhenHealthCannotBeRead) {
  rtos.set_now_ms(kBase);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100));

  // The registry could not be locked, so the supervisor cannot establish that the node
  // is healthy. It must not feed on an assumption.
  rtos.fail_next_lock();
  EXPECT_FALSE(supervisor.run());
  EXPECT_EQ(watchdog.feeds(), 0);
  EXPECT_EQ(supervisor.stats().lock_failures, 1u);
  EXPECT_EQ(supervisor.stats().feeds_withheld, 1u);
}

TEST_F(SupervisorTest, HeartbeatSurvivesTheMillisecondClockWrapping) {
  // NFR-0.7 is a 72 h unattended run and the 32-bit millisecond clock wraps at about
  // 49.7 days, so the deadline arithmetic has to be wrap-safe rather than merely
  // correct on a freshly booted node. Unsigned subtraction is what makes it so: the
  // heartbeat lands before the wrap and the check happens after it.
  constexpr uint32_t kAlmostWrap = 0xFFFFFF00u;

  rtos.set_now_ms(kAlmostWrap);
  ASSERT_TRUE(supervisor.watch(TaskId::Sensor, 100)); // deadline 300

  rtos.set_now_ms(kAlmostWrap + 100); // age 100, before the wrap
  supervisor.heartbeat(TaskId::Sensor);
  ASSERT_TRUE(supervisor.run());

  // 0x100000000 - 0xFFFFFF64 == 156, so any time after the wrap gives age = now + 156.
  rtos.set_now_ms(50); // age 206, inside the 300 ms deadline
  EXPECT_TRUE(supervisor.run()) << "age 206 ms must still be inside the deadline";

  rtos.set_now_ms(200); // age 356, past it
  EXPECT_FALSE(supervisor.run()) << "age 356 ms must be past the deadline";
  EXPECT_EQ(supervisor.trip().lateness_ms, 56u);
  EXPECT_EQ(supervisor.trip().at_ms, 200u);
}

TEST_F(SupervisorTest, SupervisorWithoutAWatchdogNeverClaimsToHaveFed) {
  Supervisor broken(rtos.iface(), nullptr);
  EXPECT_FALSE(broken.run());
  EXPECT_EQ(broken.stats().feeds, 0u);
  EXPECT_EQ(watchdog.feeds(), 0);
}

TEST_F(SupervisorTest, NamesAreStable) {
  EXPECT_STREQ(to_string(TaskId::Sensor), "sensor");
  EXPECT_STREQ(to_string(TaskId::Display), "display");
  EXPECT_STREQ(to_string(TaskId::Supervisor), "supervisor");
  EXPECT_STREQ(to_string(HealthState::Ok), "ok");
  EXPECT_STREQ(to_string(HealthState::Missing), "missing");
}

} // namespace
} // namespace node
