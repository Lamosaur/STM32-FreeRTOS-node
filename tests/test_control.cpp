/* Tests for the Phase 1 closed loop (FR-1.3, FR-1.4, and the decision record in
 * design-spec.md §6).
 *
 * The single most important thing about this file is where the plant lives: here, in the
 * test, and not in the firmware. A control law tested against a model that ships with it is
 * tested against its own assumptions; a control law tested against a model that only the
 * test can see is tested against something that can be wrong in a way the firmware is not.
 * The model is first-order-plus-dead-time, because that is what a pot of soil is: water
 * applied now shows up at the probe a few seconds later, and the response then creeps
 * toward its new equilibrium rather than arriving at it.
 *
 * ---------------------------------------------------------------------------
 * The plant, and why these numbers
 * ---------------------------------------------------------------------------
 *   air-dry baseline  300 permille    the moisture a bare pot decays to with no water
 *   gain              0.600 permille of moisture per permille of duty
 *   time constant     60 windows      = 60 s at the 1000 ms control period
 *   dead time         3 windows       = 3 s: water has to reach the probe
 *
 * The gain is chosen so that holding the setpoint (450 permille) needs 150 / 0.6 = 250
 * permille of duty — a pump running a quarter of each window, which is the operating point
 * a soil-moisture loop actually lives at and, importantly, one where the integrator has to
 * hold a real number. That last part is not decoration: it is the constraint that decides
 * how the integral clamp has to be read (see the clamp test below and board.h), and a
 * plant needing a trivially small duty would hide the difference.
 *
 * These figures are stated as numbers rather than as ranges because the loop being tested is
 * pure integer arithmetic and the plant below is too: the whole simulation is bit-exact and
 * reproducible, so the tests can assert what it produces rather than that it produced
 * something plausible.
 *
 * The windup test uses a different plant: bone dry (0 permille) and a 30 s dead time. That
 * is the flooding scenario — the probe cannot see any of the water being applied for half a
 * minute, which is exactly the condition that winds an integrator up.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cstdint>
#include <thread>
#include <vector>

#include "app/control.h"

namespace node {
namespace {

// A reading as AcquisitionService would publish it. `failures` > 0 makes it stale while
// still carrying a plausible number, which is the case that matters: a stopped DMA channel
// leaves the last good value in the snapshot with `valid` still true.
AnalogSnapshot reading(int32_t permille, uint32_t failures = 0) {
  AnalogSnapshot snapshot{};
  snapshot.raw_counts = 2000;
  snapshot.moisture_permille = permille;
  snapshot.valid = true;
  snapshot.taken_at_ms = 0;
  snapshot.sequence = 1;
  snapshot.consecutive_failures = failures;
  return snapshot;
}

// Tuning built field by field rather than by position: an aggregate initialiser with six
// int32/uint32 members is exactly the kind of call site where a swapped pair of gains
// compiles and behaves.
ControlTuning tuning_with(int32_t setpoint, int32_t kp, int32_t ki, int32_t kd = 0,
                          uint32_t min_on_ms = kControlMinOnTimeMs,
                          int32_t max_duty = kControlMaxDutyPermille) {
  ControlTuning tuning;
  tuning.setpoint_permille = setpoint;
  tuning.kp_milli = kp;
  tuning.ki_milli = ki;
  tuning.kd_milli = kd;
  tuning.min_on_time_ms = min_on_ms;
  tuning.max_duty_permille = max_duty;
  return tuning;
}

// A mixture of two published sets, spelled out field by field: the first three fields from one
// publication and the last three from another. This is not a hypothetical value — it is what a
// two-slot publication scheme returns whenever a second publication lands while the reader is
// copying the slot the first one left live, and the whole point of it is that it is neither of
// the two sets: every field is individually plausible and the combination is one nobody sent.
ControlTuning mixed(const ControlTuning &first, const ControlTuning &second) {
  ControlTuning mixture = first;
  mixture.kd_milli = second.kd_milli;
  mixture.min_on_time_ms = second.min_on_time_ms;
  mixture.max_duty_permille = second.max_duty_permille;
  return mixture;
}

// ControlTuning has no operator== and should not grow one — it is a wire-facing aggregate whose
// fields carry different units, and a whole-struct equality is exactly the check that hides
// which field was wrong. The store tests therefore compare it field by field.
void expect_same_tuning(const ControlTuning &actual, const ControlTuning &expected) {
  EXPECT_EQ(actual.setpoint_permille, expected.setpoint_permille);
  EXPECT_EQ(actual.kp_milli, expected.kp_milli);
  EXPECT_EQ(actual.ki_milli, expected.ki_milli);
  EXPECT_EQ(actual.kd_milli, expected.kd_milli);
  EXPECT_EQ(actual.min_on_time_ms, expected.min_on_time_ms);
  EXPECT_EQ(actual.max_duty_permille, expected.max_duty_permille);
}

// Rounds to nearest, halves away from zero. The plant's state is kept in milli-permille
// (a thousandth of a permille) so that a first-order lag with a 60-window time constant
// does not lose its fractional part to truncation — which would bias the equilibrium the
// convergence assertions are about.
int64_t round_div(int64_t numerator, int64_t denominator) {
  const int64_t half = denominator / 2;
  if (numerator >= 0) {
    return (numerator + half) / denominator;
  }
  return -((-numerator + half) / denominator);
}

// A pot of soil: first-order lag plus dead time, in integer arithmetic.
//
//   dy/dt = (gain * u(t - dead) - (y - dry)) / tau
//
// `u` is the duty the loop's window actually delivered, in permille. The state is in
// milli-permille; measurement_permille() drops the fraction, because that is what a 12-bit
// conversion over a calibrated span produces — the loop never sees more than a permille,
// and a model that handed it a fraction would be testing a sensor this node does not have.
class SoilPlant {
 public:
  SoilPlant(int32_t dry_milli_permille, int32_t gain_milli, uint32_t tau_windows,
            uint32_t dead_windows)
      : dry_milli_permille_(dry_milli_permille),
        gain_milli_(gain_milli),
        tau_windows_(static_cast<int64_t>(tau_windows)),
        delay_line_(dead_windows == 0 ? 1U : dead_windows, 0),
        moisture_milli_permille_(dry_milli_permille) {}

  // Start somewhere other than the dry baseline — bone dry, for the windup scenario.
  void start_at(int32_t milli_permille) { moisture_milli_permille_ = milli_permille; }

  int32_t measurement_permille() const {
    return static_cast<int32_t>(moisture_milli_permille_ / 1000);
  }

  int64_t moisture_milli_permille() const { return moisture_milli_permille_; }

  // Deliver one window's duty and advance one control period. The delay line is written
  // before it is read, so a dead time of N windows means the duty commanded now reaches the
  // soil N windows later; a dead time of 0 means it arrives in the same window.
  void step(uint32_t duty_permille) {
    delay_line_[delay_index_] = static_cast<int32_t>(duty_permille);
    delay_index_ = (delay_index_ + 1) % delay_line_.size();
    const int64_t applied = static_cast<int64_t>(delay_line_[delay_index_]);
    moisture_milli_permille_ += round_div(gain_milli_ * applied -
                                              (moisture_milli_permille_ - dry_milli_permille_),
                                          tau_windows_);
  }

 private:
  int32_t dry_milli_permille_;
  int32_t gain_milli_;
  int64_t tau_windows_;
  std::vector<int32_t> delay_line_;
  size_t delay_index_ = 0;
  int64_t moisture_milli_permille_;
};

// The plant the convergence and steady-state assertions use.
constexpr int32_t kDryMilliPermille = 300'000;
constexpr int32_t kGainMilli = 600; // 0.600 permille of moisture per permille of duty
constexpr uint32_t kTauWindows = 60;
constexpr uint32_t kDeadWindows = 3;

// What one closed-loop run recorded, so the assertions can be about the whole response
// rather than about its last value.
struct SimulationRun {
  std::vector<int32_t> measured;   // what the loop saw each window
  std::vector<int32_t> duty;       // what each window delivered
  std::vector<int32_t> integral;   // the integral state after each window
  std::vector<int64_t> moisture;   // the plant's true state, milli-permille
  ControlStats stats{};
  int32_t peak_duty = 0;
};

SimulationRun run_closed_loop(ControlLoop &loop, SoilPlant &plant, uint32_t windows) {
  SimulationRun run;
  run.measured.reserve(windows);
  run.duty.reserve(windows);
  run.integral.reserve(windows);
  run.moisture.reserve(windows);

  for (uint32_t window = 0; window < windows; ++window) {
    const ControlDecision decision =
        loop.update(window * kControlPeriodMs, reading(plant.measurement_permille()));
    plant.step(static_cast<uint32_t>(decision.duty_permille));
    run.measured.push_back(decision.measured_permille);
    run.duty.push_back(decision.duty_permille);
    run.integral.push_back(decision.terms.integral_permille_seconds);
    run.moisture.push_back(plant.moisture_milli_permille());
    run.peak_duty = std::max(run.peak_duty, decision.duty_permille);
  }
  run.stats = loop.stats();
  return run;
}

int64_t mean_of(const std::vector<int32_t> &values, size_t tail) {
  const size_t count = std::min(tail, values.size());
  int64_t sum = 0;
  for (size_t i = values.size() - count; i < values.size(); ++i) {
    sum += values[i];
  }
  return sum / static_cast<int64_t>(count);
}

// The last window at which the recorded value was further than `band` from the setpoint:
// i.e. the loop had settled by the following window, and stayed settled.
uint32_t settle_window(const std::vector<int32_t> &measured, int32_t setpoint, int32_t band) {
  uint32_t last = 0;
  for (uint32_t i = 0; i < measured.size(); ++i) {
    const int32_t difference = measured[i] - setpoint;
    if (difference > band || difference < -band) {
      last = i;
    }
  }
  return last;
}

int32_t peak_of(const std::vector<int32_t> &values) {
  return values.empty() ? 0 : *std::max_element(values.begin(), values.end());
}

// ---------------------------------------------------------------------------
// Tuning: the decision record's values, and refusing what is out of range
// ---------------------------------------------------------------------------

TEST(ControlTuningTest, DefaultsAreTheDecisionRecordsNumbers) {
  const ControlTuning tuning;
  EXPECT_EQ(tuning.setpoint_permille, 450);
  EXPECT_EQ(tuning.kp_milli, 2000);
  EXPECT_EQ(tuning.ki_milli, 50);
  EXPECT_EQ(tuning.kd_milli, 0); // decision 7: the derivative term is off by default
  EXPECT_EQ(tuning.min_on_time_ms, 50u);
  EXPECT_EQ(tuning.max_duty_permille, 500);
  EXPECT_EQ(kControlPeriodMs, 1000u); // decision 3, and deliberately not 1 kHz
  EXPECT_TRUE(tuning_valid(tuning));
}

TEST(ControlTuningTest, RefusesEachOutOfRangeFieldAndNamesIt) {
  // The point of validation is that the host link can tell a person which number it
  // rejected. A silent clamp would leave the link reporting success for a loop nobody
  // asked for.
  ControlTuning tuning;
  tuning.setpoint_permille = 1001;
  EXPECT_EQ(tuning_error(tuning), TuningField::Setpoint);
  tuning.setpoint_permille = -1;
  EXPECT_EQ(tuning_error(tuning), TuningField::Setpoint);

  tuning = ControlTuning{};
  tuning.kp_milli = kControlMaxGainMilli + 1;
  EXPECT_EQ(tuning_error(tuning), TuningField::Kp);
  // A negative gain is not a gain: it is positive feedback, and the sign error is
  // invisible in the code and obvious only in a flooded pot.
  tuning.kp_milli = -1;
  EXPECT_EQ(tuning_error(tuning), TuningField::Kp);

  tuning = ControlTuning{};
  tuning.ki_milli = -1;
  EXPECT_EQ(tuning_error(tuning), TuningField::Ki);

  tuning = ControlTuning{};
  tuning.kd_milli = kControlMaxGainMilli + 1;
  EXPECT_EQ(tuning_error(tuning), TuningField::Kd);

  tuning = ControlTuning{};
  tuning.min_on_time_ms = kControlPeriodMs + 1;
  EXPECT_EQ(tuning_error(tuning), TuningField::MinOnTime);

  tuning = ControlTuning{};
  tuning.max_duty_permille = 1001;
  EXPECT_EQ(tuning_error(tuning), TuningField::MaxDuty);
  tuning.max_duty_permille = 0;
  EXPECT_EQ(tuning_error(tuning), TuningField::MaxDuty);

  // The boundaries themselves are accepted: a bound that refused its own limit would be an
  // off-by-one that only shows up on the day someone sets it exactly.
  EXPECT_TRUE(tuning_valid(tuning_with(0, 0, 0, 0, 0, 1000)));
  EXPECT_TRUE(tuning_valid(tuning_with(1000, kControlMaxGainMilli, kControlMaxGainMilli,
                                       kControlMaxGainMilli, kControlPeriodMs, 1000)));
}

TEST(ControlTuningTest, RefusesACapThatCannotDeliverAMinimumOnTime) {
  // A duty cap of 40 permille inside a 1000 ms window is a 40 ms maximum on-time, i.e. a
  // pump that never runs: every window fails the minimum on-time rule. The loop would
  // report a healthy nonzero duty while the soil dried out, which reads as a hardware
  // fault. It is almost always a mistyped cap, so the refusal names the cap.
  ControlTuning tuning = tuning_with(450, 2000, 50, 0, /*min_on_ms=*/50, /*max_duty=*/40);
  EXPECT_EQ(tuning_error(tuning), TuningField::MaxDuty);

  // Exactly at the limit is fine: a 50 permille cap is a 50 ms on-time, which is deliverable.
  tuning.max_duty_permille = 50;
  EXPECT_TRUE(tuning_valid(tuning));
}

// ---------------------------------------------------------------------------
// Publication without a lock (decision 9)
// ---------------------------------------------------------------------------

TEST(ControlTuningStoreTest, PublishesAndLoadsCompleteTunings) {
  ControlTuningStore store;
  EXPECT_TRUE(tuning_valid(store.load())); // the defaults are published from construction

  EXPECT_TRUE(store.publish(tuning_with(500, 3000, 40)));
  EXPECT_EQ(store.load().setpoint_permille, 500);
  EXPECT_EQ(store.load().kp_milli, 3000);

  // Publication alternates slots; the reader is never told which one it is reading, which
  // is the whole point.
  EXPECT_TRUE(store.publish(tuning_with(600, 3000, 40)));
  EXPECT_EQ(store.load().setpoint_permille, 600);

  // The reader that keeps a value of its own — the control loop's form of the read — sees the
  // same publications, and never keeps a stale one by accident: with no writer running, every
  // read is a fresh one.
  ControlTuning held = store.load_or(ControlTuning{});
  EXPECT_EQ(held.setpoint_permille, 600);
  EXPECT_TRUE(store.publish(tuning_with(700, 3000, 40)));
  held = store.load_or(held);
  EXPECT_EQ(held.setpoint_permille, 700);
  EXPECT_EQ(held.kp_milli, 3000);
}

TEST(ControlTuningStoreTest, RefusesAnInvalidPublishAndKeepsThePreviousTuning) {
  ControlTuningStore store;
  ASSERT_TRUE(store.publish(tuning_with(500, 2000, 50)));

  EXPECT_FALSE(store.publish(tuning_with(5000, 2000, 50)));

  // Refused means refused: the published tuning is still the one in force, not a partially
  // applied set with a new setpoint and the old gains. Both forms of the read agree, which
  // also says that a refused publication does not leave the sequence odd — a refusal that
  // consumed a sequence step without publishing would make every later read collide.
  const ControlTuning current = store.load();
  EXPECT_EQ(current.setpoint_permille, 500);
  EXPECT_EQ(current.kp_milli, 2000);
  EXPECT_EQ(current.ki_milli, 50);
  expect_same_tuning(store.load_or(ControlTuning{}), current);
}

// ---------------------------------------------------------------------------
// The reader's rule, deterministically
// ---------------------------------------------------------------------------
//
// The two tests below are the ones the racy test cannot be. A publication landing underneath a
// copy needs a writer running concurrently with the reader, so a test that can only drive
// load_or() catches a broken rule on the runs where the interleaving happens to occur — which
// is how the defect this replaced survived its own test suite. reader_value() is the function
// load_or() decides with, so handing it the two sequence observations directly pins the rule
// itself, in every case, with no threads and no luck.

TEST(ControlTuningStoreTest, AReaderThatMeetsAPublicationInFlightKeepsTheValueItHad) {
  // The reader's value, the set the writer is publishing, and the copy a reader takes when the
  // writer gets from the first to the second while that copy is in progress.
  const ControlTuning held = tuning_with(500, 2000, 50, /*kd=*/7, /*min_on_ms=*/60,
                                        /*max_duty=*/600);
  const ControlTuning fresh = tuning_with(600, 3000, 60, /*kd=*/9, /*min_on_ms=*/80,
                                          /*max_duty=*/700);
  const ControlTuning torn = mixed(held, fresh);

  // The premise, so that a test that passes cannot be passing because the two sets happen to be
  // the same: the mixture differs from each of them in a field, and matches neither.
  EXPECT_NE(torn.kd_milli, held.kd_milli);
  EXPECT_NE(torn.min_on_time_ms, held.min_on_time_ms);
  EXPECT_NE(torn.setpoint_permille, fresh.setpoint_permille);
  EXPECT_NE(torn.kp_milli, fresh.kp_milli);

  // Sequence 5 is odd: a publication is in flight, so the reader has no published set to name
  // and the copy it took — of the previous publication's slot — is not one it may use.
  expect_same_tuning(ControlTuningStore::reader_value(5, 5, torn, held), held);

  // Sequence 6 → 10: two publications landed while the copy was being taken — the writer's
  // `set sp=`/`set kp=`/`set ki=` burst — and the second of them writes the very slot the copy
  // came from. (Publication m goes to slot m % 2 and the reader's slot at sequence 2n is
  // n % 2, so at 6 the reader is on publication 3's slot, which publication 5 rewrites.) This is
  // the exact case the scheme this replaces returned as though it were a set, and the reader
  // must return `held` instead of the mixture.
  expect_same_tuning(ControlTuningStore::reader_value(6, 10, torn, held), held);

  // Sequence 6 → 8: one publication landed, and it wrote the *other* slot, so a copy taken in
  // this window is in fact whole — and the rule refuses it anyway. That is the deliberate
  // conservative direction the store documents: two observations cannot tell this window from
  // the one above, and the price of not asking a third time is one control period.
  expect_same_tuning(ControlTuningStore::reader_value(6, 8, fresh, held), held);

  // Sequence 6 → 7: a publication began under the copy. Refused for the same reason, and it is
  // the same comparison — `after` odd is not a case of its own, because `before` is already
  // known to be even by the time the rule is applied.
  expect_same_tuning(ControlTuningStore::reader_value(6, 7, torn, held), held);

  // The one accepted case: the sequence was even and unchanged across the copy, so the copy is
  // a whole publication and there is nothing to keep.
  expect_same_tuning(ControlTuningStore::reader_value(6, 6, fresh, held), fresh);

  // A 32-bit sequence wraps, and the rule does not care: left where it started is a whole
  // publication, odd is a write in flight. There is no 2^32-publication wrap inside one copy to
  // worry about — that would be four billion host commands in six instructions — and the wrap
  // itself is defined for an unsigned counter rather than being a case to handle.
  expect_same_tuning(ControlTuningStore::reader_value(0, 0, fresh, held), fresh);
  expect_same_tuning(
      ControlTuningStore::reader_value(UINT32_MAX, UINT32_MAX, torn, held), held);
}

TEST(ControlTuningStoreTest, ACollidingReadDoesNotLoseTheNextPublication) {
  // What a collided read costs is one control period, not the host's command. The value it
  // keeps is a value, not a latch: the publication the read missed has to be visible to the
  // next read, in full — which is also what rules out the tempting "merge the two" fix, where
  // the fields the collision kept would survive into the next set.
  ControlTuningStore store;
  ASSERT_TRUE(store.publish(tuning_with(500, 2000, 50, /*kd=*/7, /*min_on_ms=*/60,
                                        /*max_duty=*/600)));

  ControlTuning held = store.load_or(ControlTuning{});
  ASSERT_EQ(held.setpoint_permille, 500);

  // The collided read, driven through the real rule: sequence 3 is odd, so a publication is in
  // flight and the copy it left is refused.
  const ControlTuning fresh = tuning_with(600, 3000, 60, /*kd=*/9, /*min_on_ms=*/80,
                                          /*max_duty=*/700);
  const ControlTuning after_collision =
      ControlTuningStore::reader_value(3, 3, mixed(held, fresh), held);
  expect_same_tuning(after_collision, held);
  held = after_collision;

  // The writer finishes the set the collided read missed. The next read sees all of it.
  ASSERT_TRUE(store.publish(fresh));
  held = store.load_or(held);
  expect_same_tuning(held, fresh);
}

TEST(ControlTuningStoreTest, AConcurrentReaderNeverObservesAHalfWrittenTuning) {
  // Every published tuning has all six fields set to the same number, so a reader that ever
  // saw a mixture of two publications would fail this assertion while every individual
  // field still looked plausible — which is exactly the failure a torn read is.
  //
  // The reader here is the real one, not a copy of the ideal: it holds the value it last
  // accepted and hands it to load_or(), which is what ControlLoop does with last_tuning_. That
  // is the contract the store offers — a whole set or the value you had, never a mixture — and
  // this test is the end-to-end statement of it under a writer publishing back to back, which
  // is what a host sending `set sp=`, `set kp=`, `set ki=` in one burst does.
  //
  // Versions run 1..999: the setpoint, both limits and the minimum on-time all accept that
  // range, so every version is a publishable tuning and no field is silently at its bound.
  constexpr int32_t kVersions = 999;

  // What the reader must read before the writer may stop. One pass of 999 publications can
  // finish before a second thread has started at all — that has been observed here, as a run
  // whose reader reported zero reads — and a run in which the two never overlap tests nothing
  // about a race. `reads` is the reader's own counter, so the handover waits for the reader's
  // progress rather than for a sleep, and the overlap is a property of the test rather than of
  // the scheduler.
  constexpr int kMinReads = 20000;

  ControlTuningStore store;

  // Version 1 is published, and the reader's own value seeded to it, before either thread
  // starts. That closes the one window in which the compiled-in defaults are the published set:
  // they are a complete set and the store is constructed holding them, but they are not one of
  // this test's versions and the check below could not tell them from a mixture. From here on
  // every even sequence value names a slot holding a version — sequence 0 (slot 0, the
  // defaults) is never observed again, and the second publication overwrites slot 0.
  ControlTuning held = tuning_with(1, 1, 1, 1, 1, 1);
  ASSERT_TRUE(store.publish(held));

  std::atomic<bool> stop{false};
  std::atomic<int> mismatches{0};
  std::atomic<int> reads{0};

  std::thread writer([&] {
    // Cycles the versions and stops once the reader has clearly been running alongside. Every
    // version is a whole set on its own, so a cycle boundary is not a case the reader can see.
    int32_t version = 1;
    while (reads.load(std::memory_order_relaxed) < kMinReads) {
      store.publish(tuning_with(version, version, version, version,
                                static_cast<uint32_t>(version), version));
      version = version == kVersions ? 1 : version + 1;
    }
    stop.store(true, std::memory_order_relaxed);
  });

  std::thread reader([&] {
    while (!stop.load(std::memory_order_relaxed)) {
      held = store.load_or(held);
      const ControlTuning &tuning = held;
      const bool consistent = tuning.setpoint_permille == tuning.kp_milli &&
                              tuning.kp_milli == tuning.ki_milli &&
                              tuning.ki_milli == tuning.kd_milli &&
                              tuning.kd_milli == static_cast<int32_t>(tuning.min_on_time_ms) &&
                              tuning.min_on_time_ms == static_cast<uint32_t>(tuning.max_duty_permille);
      if (!consistent) {
        mismatches.fetch_add(1);
      }
      reads.fetch_add(1);
    }
  });

  writer.join();
  reader.join();

  EXPECT_GT(reads.load(), kMinReads);
  EXPECT_EQ(mismatches.load(), 0);

  // And the staleness is not a latch: with the writer gone, the next publication is visible to
  // the next read, in full. A reader that kept its previous value and never let go would pass
  // every assertion above and be useless. This one is published from the test thread, so it
  // cannot collide and the assertion is exact rather than probabilistic.
  const ControlTuning last = tuning_with(kVersions, kVersions, kVersions, kVersions,
                                        static_cast<uint32_t>(kVersions), kVersions);
  ASSERT_TRUE(store.publish(last));
  held = store.load_or(held);
  expect_same_tuning(held, last);
}

// ---------------------------------------------------------------------------
// The control law
// ---------------------------------------------------------------------------

TEST(PidControllerTest, GainUnitsMatchTheDecisionRecordsWorkedExample) {
  PidController pid;
  pid.set_tuning(tuning_with(450, 2000, 50));

  // A 150 permille error commands 300 permille of duty: kp_milli = 2000 is 2.0
  // duty-permille per error-permille.
  const PidTerms first = pid.update(300, 1000);
  EXPECT_EQ(first.error_permille, 150);
  EXPECT_EQ(first.p_duty_permille, 300);

  // Error 145 permille sustained for 20 s gives 145 permille of I: the integral state is
  // 20 x 145 = 2900 permille-seconds, and 50 x 2900 / 1000 = 145.
  PidController example;
  example.set_tuning(tuning_with(450, 2000, 50));
  PidTerms terms{};
  for (int i = 0; i < 20; ++i) {
    terms = example.update(305, 1000); // measured 305, setpoint 450 -> error 145
  }
  EXPECT_EQ(terms.error_permille, 145);
  EXPECT_EQ(terms.integral_permille_seconds, 2900);
  EXPECT_EQ(terms.i_duty_permille, 145);
  EXPECT_EQ(terms.requested_duty_permille, 435); // P = 290, I = 145
  EXPECT_FALSE(terms.integral_clamped);
}

TEST(PidControllerTest, TheIntegralIsClampedToTheActuatorsAuthority) {
  // The failure the clamp prevents, in the maintainer's words: with a duty cap and slow
  // percolation the integrator winds up during the dead time, and the surplus is delivered
  // after the reading reaches the setpoint, which floods the pot.
  //
  // The clamp is the decided number — max_duty_permille — read as a bound on the
  // integrator's contribution. At the defaults the state limit is 500 x 1000 / 50 = 10,000
  // permille-seconds, and the I term cannot exceed 500 duty-permille.
  //
  // Read the other way — as a bound of 500 permille-seconds on the state — the I term would
  // cap at 50 x 500 / 1000 = 25 duty-permille, which is half of kControlMinOnTimeMs. The
  // assertions below are the arithmetic that decided the reading: the integral has to be
  // able to ask for at least a minimum on-time, and at 25 permille it never could, so the
  // loop could not hold any duty the actuator would deliver and would carry a permanent
  // offset of at least (50 - 25) x 1000 / 2000 = 12.5 permille.
  EXPECT_EQ(control_integral_state_limit(kControlMaxDutyPermille, kControlDefaultKiMilli),
            10'000);
  // And the same gains read the other way: the integral's contribution would be capped at
  // 25 duty-permille, which is below the 50 ms minimum on-time. That is the arithmetic the
  // reading was decided on, kept as an assertion so that a later change to either number has
  // to re-read the argument rather than quietly invalidating it.
  const int32_t literal_reading_authority =
      (kControlDefaultKiMilli * kControlMaxDutyPermille) / 1000;
  EXPECT_EQ(literal_reading_authority, 25);
  EXPECT_LT(literal_reading_authority, static_cast<int32_t>(kControlMinOnTimeMs));

  PidController pid;
  pid.set_tuning(tuning_with(450, 0, 50)); // P off, so the I term stands alone

  // A 450 permille error for 60 windows would accumulate 27,000 permille-seconds without a
  // clamp. It stops at 10,000, and the I term stops at the actuator's own cap.
  PidTerms terms{};
  for (int i = 0; i < 60; ++i) {
    terms = pid.update(0, 1000);
  }
  EXPECT_EQ(terms.integral_permille_seconds, 10'000);
  EXPECT_TRUE(terms.integral_clamped);
  EXPECT_EQ(terms.i_duty_permille, kControlMaxDutyPermille);

  // Negative errors clamp symmetrically, so the loop cannot bank a negative integral and
  // spend the next hour ignoring a dry reading.
  for (int i = 0; i < 120; ++i) {
    terms = pid.update(900, 1000); // error -450
  }
  EXPECT_EQ(terms.integral_permille_seconds, -10'000);
  EXPECT_EQ(terms.i_duty_permille, -kControlMaxDutyPermille);
}

TEST(PidControllerTest, TheOutputIsClampedToTheDecidedDutyCap) {
  PidController pid;
  pid.set_tuning(tuning_with(450, 2000, 0));

  // A bone-dry reading asks for 900 permille of duty; the cap is 500.
  const PidTerms terms = pid.update(0, 1000);
  EXPECT_EQ(terms.requested_duty_permille, 900);
  EXPECT_EQ(terms.duty_permille, 500);
  EXPECT_TRUE(terms.output_clamped);

  // The lower end: this actuator can only add water, so a reading above the setpoint
  // commands exactly zero rather than a negative number something downstream might
  // reinterpret.
  const PidTerms wet = pid.update(900, 1000);
  EXPECT_EQ(wet.requested_duty_permille, -900);
  EXPECT_EQ(wet.duty_permille, 0);
  EXPECT_TRUE(wet.output_clamped);
}

TEST(PidControllerTest, TheDerivativeTermIsSizedInTheDecidedUnits) {
  PidController pid;
  pid.set_tuning(tuning_with(450, 0, 0, /*kd=*/1000)); // P and I off, D alone

  pid.update(400, 1000); // error 50, first evaluation: no previous error, so D = 0
  EXPECT_EQ(pid.terms().d_duty_permille, 0);

  // error 60 now, 50 before, dt 1000 ms: 1000 x 10 x 1000 / 1000 / 1000 = 10.
  const PidTerms terms = pid.update(390, 1000);
  EXPECT_EQ(terms.d_duty_permille, 10);
}

TEST(PidControllerTest, KdZeroMakesTheDerivativeTermExactlyZero) {
  // Decision 7 keeps D off by default. "Off" has to mean exactly zero and not merely
  // small: a derivative of one permille per window is a term nobody can see in the output
  // while it is amplifying measurement noise.
  PidController pid;
  pid.set_tuning(tuning_with(450, 2000, 50, /*kd=*/0));

  pid.update(300, 1000);
  const PidTerms terms = pid.update(450, 1000); // a 150 permille jump in one window
  EXPECT_EQ(terms.d_duty_permille, 0);
  // And the integral is where it was, because a zero error integrates to nothing: this is
  // the mechanism that holds the steady-state duty, so it must not drift when the error
  // momentarily vanishes.
  EXPECT_EQ(terms.integral_permille_seconds, 150);
}

TEST(PidControllerTest, TheIntegralIsSampleTimeAware) {
  PidController pid;
  pid.set_tuning(tuning_with(450, 0, 50));

  EXPECT_EQ(pid.update(350, 500).integral_permille_seconds, 50);   // 100 x 500 / 1000
  EXPECT_EQ(pid.update(350, 2000).integral_permille_seconds, 250); // + 100 x 2000 / 1000
}

TEST(PidControllerTest, AZeroIntervalNeitherIntegratesNorDivides) {
  // A caller that evaluates twice with the same timestamp must not fault and must not
  // pretend time passed. The derivative's division by dt_ms is the trap here.
  PidController pid;
  pid.set_tuning(tuning_with(450, 2000, 50, /*kd=*/1000));

  const PidTerms first = pid.update(400, 1000); // error 50 -> state 50
  ASSERT_EQ(first.integral_permille_seconds, 50);
  const PidTerms terms = pid.update(400, 0);
  EXPECT_EQ(terms.integral_permille_seconds, 50); // unchanged: no interval, no integration
  EXPECT_EQ(terms.d_duty_permille, 0);
}

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

TEST(WindowOutputTest, IsOnForExactlyTheCommandedOnTime) {
  WindowOutput window{kControlPeriodMs};
  window.begin_window(1000, 50);

  // On for [start, start + on_time): 50 ms of a 1000 ms window, at both boundaries.
  EXPECT_TRUE(window.on(1000));
  EXPECT_TRUE(window.on(1049));
  EXPECT_FALSE(window.on(1050));
  EXPECT_FALSE(window.on(1999));

  // An on-time of 0 is off for the whole window, which is what a zero duty means.
  window.begin_window(2000, 0);
  EXPECT_FALSE(window.on(2000));
  EXPECT_FALSE(window.on(2999));

  // A full-window on-time is on until the next window begins.
  window.begin_window(3000, kControlPeriodMs);
  EXPECT_TRUE(window.on(3000));
  EXPECT_TRUE(window.on(3999));
  EXPECT_FALSE(window.on(4000));
}

TEST(WindowOutputTest, ClampsAnOnTimeLongerThanTheWindow) {
  // "On for longer than the window" is not expressible in a time-proportioned scheme, and
  // silently leaving the pump on forever is the wrong way to discover that.
  WindowOutput window{1000};
  window.begin_window(0, 5000);
  EXPECT_EQ(window.on_time_ms(), 1000u);
  EXPECT_TRUE(window.on(999));
  EXPECT_FALSE(window.on(1000));
}

TEST(WindowOutputTest, AbortTurnsTheOutputOffForTheRestOfTheWindow) {
  WindowOutput window{1000};
  window.begin_window(0, 500);
  ASSERT_TRUE(window.on(10));

  window.abort();

  EXPECT_FALSE(window.active());
  EXPECT_FALSE(window.on(10));
  EXPECT_FALSE(window.on(20));
}

TEST(WindowOutputTest, ATimestampBehindTheWindowStartReadsAsOff) {
  // The millisecond clock wraps at 2^32. A caller that passes a stale timestamp (or a clock
  // that has wrapped) must get "off" rather than a huge elapsed time that looks like an
  // expired window followed by whatever the comparison does with it.
  WindowOutput window{1000};
  window.begin_window(1000, 500);

  EXPECT_FALSE(window.on(999));
  EXPECT_FALSE(window.on(0));

  // The wrap itself: a window that starts just below the wrap and is asked about just above
  // it is still on, because the elapsed time is a subtraction.
  window.begin_window(0xFFFFFFF0U, 500);
  EXPECT_TRUE(window.on(0xFFFFFFF0U));
  EXPECT_TRUE(window.on(0x00000010U)); // 32 ms later, across the wrap
  EXPECT_TRUE(window.on(0x000001E3U)); // 499 ms later: the last on millisecond
  EXPECT_FALSE(window.on(0x000001E4U)); // 500 ms later: the window's on-time is spent
}

// ---------------------------------------------------------------------------
// The loop: fail-safe, the minimum on-time, the cap
// ---------------------------------------------------------------------------

TEST(ControlLoopTest, FailSafeForcesZeroOutputAndHoldsTheIntegral) {
  ControlLoop loop;

  // Five windows of a reading that is 50 permille dry, so the integral has something in it.
  for (uint32_t window = 0; window < 5; ++window) {
    loop.update(window * kControlPeriodMs, reading(400));
  }
  const int32_t held = loop.pid().terms().integral_permille_seconds;
  ASSERT_EQ(held, 250); // 5 windows x (450 - 400)
  ASSERT_TRUE(loop.window().active());

  // Stale: the sensor has stopped answering but the last good value is still in the
  // snapshot with valid == true. This is the case that matters, because a loop checking
  // `valid` would keep watering a pot on the strength of a number that stopped changing
  // minutes ago.
  const ControlDecision stale = loop.update(5000, reading(400, /*failures=*/1));

  EXPECT_TRUE(stale.fail_safe);
  EXPECT_EQ(stale.duty_permille, 0);
  EXPECT_EQ(stale.on_time_ms, 0u);
  EXPECT_EQ(stale.requested_duty_permille, 0);
  // The window is aborted, not merely left unrefreshed: a reading that goes stale mid-pulse
  // stops the pump now rather than at the end of the commanded on-time.
  EXPECT_FALSE(loop.window().active());
  EXPECT_FALSE(loop.window().on(5000));
  // The integral is held — not unwound and not cleared. It is the only state that removes
  // the steady-state offset, so clearing it makes every sensor hiccup cost the loop the
  // offset it spent minutes earning.
  EXPECT_EQ(loop.pid().terms().integral_permille_seconds, held);
  EXPECT_EQ(stale.terms.integral_permille_seconds, held);
  EXPECT_EQ(loop.stats().fail_safe_windows, 1u);
  // Fail-safe windows are counted as fail-safe and not also as zero-duty windows, so that
  // the counters partition the evaluations.
  EXPECT_EQ(loop.stats().zero_duty_windows, 0u);

  // Recovery: control resumes from the held integral, and the interval that just elapsed is
  // one control period rather than the whole blind gap. Integrating the gap would re-earn
  // windup with a measurement that was stale for all of it.
  const ControlDecision recovered = loop.update(6000, reading(400));
  EXPECT_FALSE(recovered.fail_safe);
  EXPECT_EQ(recovered.terms.integral_permille_seconds, held + 50); // + error x dt / 1000
}

TEST(ControlLoopTest, AnAbsentReadingIsAlsoFailSafe) {
  ControlLoop loop;

  const ControlDecision decision = loop.update(0, AnalogSnapshot{});

  EXPECT_TRUE(decision.fail_safe);
  EXPECT_EQ(decision.duty_permille, 0);
  EXPECT_EQ(decision.measured_permille, 0);
  EXPECT_FALSE(loop.window().active());
}

TEST(ControlLoopTest, MinimumOnTimeBoundaryIs49To0And50To50) {
  // Decision 5, asserted from both sides. The duty is set up to be exactly the error by
  // using kp = 1000 and no integral, so the on-time is a number the test chose rather than
  // a number it has to compute backwards.
  ControlTuningStore store;
  ASSERT_TRUE(store.publish(tuning_with(450, 1000, 0)));
  ControlLoop loop(&store);

  const ControlDecision below = loop.update(0, reading(401)); // error 49 -> duty 49
  EXPECT_EQ(below.requested_duty_permille, 49);
  EXPECT_EQ(below.on_time_ms, 0u);
  EXPECT_TRUE(below.skipped);
  EXPECT_EQ(below.duty_permille, 0);
  EXPECT_EQ(loop.stats().min_on_time_skips, 1u);
  EXPECT_EQ(loop.stats().zero_duty_windows, 1u);

  const ControlDecision at = loop.update(1000, reading(400)); // error 50 -> duty 50
  EXPECT_EQ(at.requested_duty_permille, 50);
  EXPECT_EQ(at.on_time_ms, 50u);
  EXPECT_FALSE(at.skipped);
  EXPECT_TRUE(loop.window().on(1000));
  EXPECT_FALSE(loop.window().on(1050));
  EXPECT_EQ(loop.stats().min_on_time_skips, 1u); // unchanged by the window that acted
}

TEST(ControlLoopTest, TheOutputNeverExceedsTheDecidedCap) {
  ControlLoop loop;

  const ControlDecision decision = loop.update(0, reading(0)); // 450 permille wet of dry

  // 900 from the proportional term (2 x 450) plus 22 from the integral's first window
  // (50 x 450 / 1000), against a cap of 500.
  EXPECT_EQ(decision.terms.requested_duty_permille, 922);
  EXPECT_EQ(decision.requested_duty_permille, 500);       // after the cap
  EXPECT_EQ(decision.duty_permille, 500);
  EXPECT_EQ(decision.on_time_ms, 500u);
  EXPECT_TRUE(decision.saturated);
  EXPECT_EQ(loop.stats().saturated_windows, 1u);
}

TEST(ControlLoopTest, WetSoilCommandsExactlyNothing) {
  ControlLoop loop;

  const ControlDecision decision = loop.update(0, reading(600)); // above the setpoint

  EXPECT_EQ(decision.duty_permille, 0);
  EXPECT_EQ(decision.on_time_ms, 0u);
  EXPECT_FALSE(decision.skipped); // zero was asked for; nothing was rounded away
  EXPECT_FALSE(decision.saturated);
  EXPECT_EQ(loop.stats().zero_duty_windows, 1u);
  EXPECT_EQ(loop.stats().min_on_time_skips, 0u);
}

TEST(ControlLoopTest, RuntimeTuningTakesEffectWithoutAReflash) {
  // FR-1.4: the host sets a value and the closed-loop response changes. The store is the
  // only path, and the loop reads it once per evaluation, so a set is live in one period.
  ControlTuningStore store;
  ControlLoop loop(&store);

  const ControlDecision before = loop.update(0, reading(400));
  EXPECT_EQ(before.setpoint_permille, 450);
  EXPECT_EQ(before.terms.error_permille, 50);
  EXPECT_EQ(before.terms.p_duty_permille, 100); // 2 x 50
  EXPECT_EQ(before.terms.i_duty_permille, 2);   // 50 x 50 / 1000
  EXPECT_EQ(before.duty_permille, 102);

  ASSERT_TRUE(store.publish(tuning_with(600, 2000, 50)));

  const ControlDecision after = loop.update(1000, reading(400));
  EXPECT_EQ(after.setpoint_permille, 600);
  EXPECT_EQ(after.terms.error_permille, 200);
  EXPECT_GT(after.duty_permille, before.duty_permille);

  // And a refused set leaves the loop on the tuning in force.
  ASSERT_FALSE(store.publish(tuning_with(6000, 2000, 50)));
  EXPECT_EQ(loop.update(2000, reading(400)).setpoint_permille, 600);
}

TEST(ControlLoopTest, TheCountersPartitionTheEvaluations) {
  ControlLoop loop;
  loop.update(0, reading(400));                  // acts
  loop.update(1000, reading(450, /*f=*/2));      // fail-safe
  loop.update(2000, reading(600));               // zero duty, nothing asked for
  loop.update(3000, reading(400));               // acts

  const ControlStats stats = loop.stats();
  const uint32_t acting = stats.evaluations - stats.fail_safe_windows - stats.zero_duty_windows;
  EXPECT_EQ(stats.evaluations, 4u);
  EXPECT_EQ(stats.fail_safe_windows, 1u);
  EXPECT_EQ(stats.zero_duty_windows, 1u);
  EXPECT_EQ(acting, 2u);
  EXPECT_LE(stats.min_on_time_skips, stats.zero_duty_windows);
}

// ---------------------------------------------------------------------------
// The plant: convergence, overshoot, windup
// ---------------------------------------------------------------------------

TEST(ControlLoopSimulation, ConvergesToTheSetpointWithNoSteadyStateError) {
  // The test that proves the integrator is doing its job. From a dry start the loop has to
  // arrive at 450 permille and *hold* it — a proportional-only loop cannot, because at
  // zero error a P-only loop commands zero duty and the demand is not zero.
  ControlLoop loop;
  SoilPlant plant{kDryMilliPermille, kGainMilli, kTauWindows, kDeadWindows};
  const SimulationRun run = run_closed_loop(loop, plant, 1200);

  // The demand at the setpoint is (450 - 300) / 0.6 = 250 permille of duty, so the
  // equilibrium is a real number of milliseconds per window, not a fossil of the plant's
  // arithmetic.
  const int64_t steady_error = mean_of(run.measured, 200) - kControlSetpointPermille;
  EXPECT_EQ(steady_error, 0);

  // The soil itself, not just what the sensor can resolve: within a permille of the
  // setpoint at the end, and the integral is holding the demand.
  EXPECT_LE(std::abs(run.moisture.back() - 450'000), 1000);
  // The integral state the loop settles on: 50 x 5030 / 1000 = 251 permille of duty, which
  // is the 250.6 permille the plant needs at the setpoint plus the rounding of a
  // per-window integer division.
  EXPECT_EQ(mean_of(run.integral, 200), 5030);

  RecordProperty("steady_state_error_permille", static_cast<int>(steady_error));
  RecordProperty("final_integral_permille_seconds", run.integral.back());
  RecordProperty("final_duty_permille", run.duty.back());

  // For comparison, the same plant with the integral switched off. This is the measurement
  // that makes the previous assertions mean something: without the integral the loop sits
  // 69 permille below the setpoint and stays there forever.
  ControlTuningStore p_only_store;
  ASSERT_TRUE(p_only_store.publish(tuning_with(450, 2000, /*ki=*/0)));
  ControlLoop p_only_loop(&p_only_store);
  SoilPlant p_only_plant{kDryMilliPermille, kGainMilli, kTauWindows, kDeadWindows};
  const SimulationRun p_only = run_closed_loop(p_only_loop, p_only_plant, 1200);

  const int64_t p_only_error = mean_of(p_only.measured, 200) - kControlSetpointPermille;
  EXPECT_EQ(p_only_error, -69);
  RecordProperty("p_only_steady_state_error_permille", static_cast<int>(p_only_error));
}

TEST(ControlLoopSimulation, OvershootStaysBounded) {
  // Same run, read for its transient. The figures are recorded rather than asserted as
  // prose: peak overshoot, the window the loop settled at, and the peak duty.
  ControlLoop loop;
  SoilPlant plant{kDryMilliPermille, kGainMilli, kTauWindows, kDeadWindows};
  const SimulationRun run = run_closed_loop(loop, plant, 1200);

  const int32_t peak = peak_of(run.measured);
  const uint32_t settled = settle_window(run.measured, kControlSetpointPermille, 2);
  const int64_t peak_moisture = *std::max_element(run.moisture.begin(), run.moisture.end());

  // The step is 150 permille; the measured overshoot is 5 permille (3.3 % of the step) and
  // the soil's true peak is 455.456 permille. It is not zero and is not asserted as zero:
  // the dead time means the loop is still delivering the water it commanded three seconds
  // ago when the probe first reads the setpoint.
  EXPECT_GE(peak, 450); // it does reach the setpoint
  EXPECT_LE(peak - kControlSetpointPermille, 6);
  EXPECT_LE(peak_moisture, 455'500);
  // Settled: the last window at which the measured value was more than 2 permille from the
  // setpoint is window 218, and it holds from there to window 1199. Settling is quoted
  // against a band because "exactly at the setpoint" is not a thing a 1 permille sensor
  // can report; the band is 2 permille, which is 0.2 % of the calibrated span.
  EXPECT_LE(settled, 250u);

  RecordProperty("peak_overshoot_permille", peak - kControlSetpointPermille);
  RecordProperty("settling_window_band_2_permille", static_cast<int>(settled));
  RecordProperty("peak_moisture_milli_permille", static_cast<int>(peak_moisture));
}

TEST(ControlLoopSimulation, WindupIsClampedAndThePumpDoesNotStayPinned) {
  // THIS IS THE FLOODING SCENARIO.
  //
  // Bone dry (a reading of 0 permille, the worst error the loop can see) with a 30 s dead
  // time: for thirty windows the probe sees none of the water being applied, so the error
  // stays at its maximum and the integrator has nothing to correct it with. Without a clamp
  // the integral would keep climbing for the whole dead time and beyond, and every
  // permille of it has to be unwound by a *negative* error after the reading crosses the
  // setpoint — which means delivering water for that whole unwinding period. That is the
  // flood: the pot is watered hardest exactly when it no longer needs it.
  constexpr uint32_t kLongDeadWindows = 30;

  ControlLoop loop;
  SoilPlant plant{kDryMilliPermille, kGainMilli, kTauWindows, kLongDeadWindows};
  plant.start_at(0);
  const SimulationRun run = run_closed_loop(loop, plant, 2000);

  // The clamp engaged, and at exactly the limit. The counterfactual is arithmetic: 30
  // windows of a 450 permille error accumulate 13,500 permille-seconds without a clamp,
  // against a limit of 500 x 1000 / 50 = 10,000.
  const int32_t peak_integral = *std::max_element(run.integral.begin(), run.integral.end());
  EXPECT_EQ(peak_integral, 10'000);
  EXPECT_GT(450 * static_cast<int>(kLongDeadWindows), 10'000);

  // With the error pinned at 450 the output is at its cap while the probe is blind: 99
  // windows of saturated output, which is the windup the clamp bounds rather than a number
  // the clamp prevents.
  EXPECT_GT(run.stats.saturated_windows, 50u);
  EXPECT_EQ(run.peak_duty, 500);
  RecordProperty("saturated_windows", static_cast<int>(run.stats.saturated_windows));

  // The crossing, and what happens after it. The pump must not stay pinned at the cap: the
  // P term turns negative as soon as the reading is above the setpoint, so the output falls
  // away from the cap within a window or two instead of being held there by a wound-up
  // integral. (Held there is precisely what an unclamped integrator does, and it is why
  // this assertion exists.)
  size_t crossing = 0;
  while (crossing < run.measured.size() && run.measured[crossing] < kControlSetpointPermille) {
    ++crossing;
  }
  ASSERT_LT(crossing, run.measured.size());
  ASSERT_EQ(run.duty[crossing], 500); // still pinned at the moment of crossing: the cap
  size_t unpinned = crossing;
  while (unpinned < run.duty.size() && run.duty[unpinned] >= 500) {
    ++unpinned;
  }
  ASSERT_LT(unpinned, run.duty.size());
  EXPECT_LE(unpinned - crossing, 10u);
  RecordProperty("windows_pinned_at_cap_after_crossing", static_cast<int>(unpinned - crossing));

  // The overshoot this costs, stated as a number: the water in flight during the 30 s of
  // dead time plus the integral's remaining authority. It is bounded, and the bound is what
  // the clamp buys — 250 permille is the worst case the arithmetic allows (the integral can
  // hold the cap until the error reaches -max_duty/kp x 1000), against unbounded without it.
  const int32_t peak = peak_of(run.measured);
  EXPECT_LE(peak - kControlSetpointPermille, 80);
  RecordProperty("windup_peak_overshoot_permille", peak - kControlSetpointPermille);

  // And it recovers: the loop returns to the setpoint and holds it, which is what makes the
  // clamp an anti-windup measure rather than merely a limit.
  const uint32_t settled = settle_window(run.measured, kControlSetpointPermille, 2);
  EXPECT_LE(settled, 450u);
  EXPECT_LE(std::abs(mean_of(run.measured, 200) - kControlSetpointPermille), 1);
  RecordProperty("windup_recovery_window", static_cast<int>(settled));
}

TEST(ControlLoopSimulation, TheDerivativeTermAmplifiesMeasurementNoise) {
  // THIS IS THE EVIDENCE FOR DECISION 7, which keeps kd at 0.
  //
  // The measured quantity is an averaged 12-bit reading over a calibrated span, so it
  // carries a permille or two of noise on a good day and more on a long probe lead. A
  // derivative term multiplies exactly that noise by kd/dt, so the right question is not
  // "does D help the response" but "how much more does the output move between consecutive
  // windows", because every one of those movements is a pump and a gate driver being
  // switched.
  //
  // The measurement sequence is a fixed 20 permille alternation — a square wave of noise,
  // which is the worst case and also the easiest to reason about — fed open-loop so that
  // feedback cannot filter it. The metric is the total variation of the commanded duty.
  //
  // The alternation sits on a 40 permille error rather than on the setpoint, and that is a
  // finding in itself: noise around zero error commands +-20 permille of duty, every window
  // of which is below the 50 ms minimum on-time and is therefore skipped. The pump would
  // never move at all, so a measurement at zero error would show two zeros and prove
  // nothing. At a 40 permille error the commanded duty is 100/60 permille, both windows
  // act, and the derivative's contribution to the switching is what the test can see.
  const auto total_variation = [](int32_t kd_milli) {
    ControlTuningStore store;
    EXPECT_TRUE(store.publish(tuning_with(450, 2000, 50, kd_milli)));
    ControlLoop loop(&store);

    int64_t variation = 0;
    int32_t previous = 0;
    bool have_previous = false;
    for (uint32_t window = 0; window < 40; ++window) {
      const int32_t measured = (window % 2 == 0) ? 400 : 420;
      const ControlDecision decision = loop.update(window * kControlPeriodMs, reading(measured));
      if (have_previous) {
        variation += std::abs(decision.duty_permille - previous);
      }
      previous = decision.duty_permille;
      have_previous = true;
    }
    return variation;
  };

  const int64_t with_kd_off = total_variation(0);
  const int64_t with_kd = total_variation(500);

  // D is off, and the output still moves — that movement is the proportional term doing its
  // job on the error. What the derivative adds is more movement with no information in it:
  // the same 40-window reading gives a total variation of 1558 permille with kd = 0 and
  // 2328 with kd = 500, i.e. 49 % more switching of the pump and the gate for the same
  // measurement. And 500 milli is a *small* kd — the term scales linearly with it, so the
  // gain that looks like it "sharpened the response" on a quiet bench is several times that
  // in the field, on a probe on a long lead.
  EXPECT_GT(with_kd_off, 0);
  EXPECT_GT(with_kd, with_kd_off);
  EXPECT_GE(with_kd * 10, with_kd_off * 12);

  RecordProperty("total_variation_kd_0", static_cast<int>(with_kd_off));
  RecordProperty("total_variation_kd_500", static_cast<int>(with_kd));
}

} // namespace
} // namespace node
