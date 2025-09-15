/* Integration tests for the whole node: the five-task architecture, the four Phase 0
 * behaviours layered on it, and Phase 1's analog channel and control loop.
 *
 * This is the test that makes FR-0.6 pay off. The node owns its HAL interfaces rather
 * than reaching for globals, so the same `Node::tick()` that runs under FreeRTOS on the
 * F103 runs here one iteration at a time against doubles — no board, no scheduler, and
 * no timing flakiness. Each FR below gets an assertion that would fail if the mechanism
 * were removed rather than merely degraded.
 *
 * The Phase 1 section at the end is the one that matters for the wiring: before it, the
 * acquisition service, the control loop and the actuator existed and were tested in
 * isolation while `node.cpp` referenced none of them, so the target image did not contain
 * them at all. `arm-none-eabi-nm firmware.elf | grep AcquisitionService` is the check that
 * they are linked; these tests are the check that they are *connected*, which is a
 * different claim.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "app/board.h"
#include "app/node.h"
#include "support/fake_actuator.h"
#include "support/fake_adc.h"
#include "support/fake_clock.h"
#include "support/fake_i2c.h"
#include "support/fake_peripherals.h"
#include "support/fake_rtos.h"
#include "support/fake_watchdog.h"

namespace node {
namespace {

using test::FakeActuator;
using test::FakeAdc;
using test::FakeClock;
using test::FakeI2c;
using test::FakeRtos;
using test::FakeUart;
using test::FakeWatchdog;

// A correctly checksummed AHT20 frame for the given 20-bit raw values.
//
// The sensor on the bench is an "AHT20 + BMP280" module, so the framing here is the
// AHT20's: a status byte, five data bytes packing 20 bits of humidity and 20 bits of
// temperature (their low and high nibbles sharing the third data byte), and one CRC over
// the first six. The status byte defaults to "calibrated, not busy".
std::vector<uint8_t> reply(uint32_t raw_temperature, uint32_t raw_humidity,
                           uint8_t status = 0x08) {
  std::vector<uint8_t> out(7, 0);
  out[0] = status;
  out[1] = static_cast<uint8_t>((raw_humidity >> 12) & 0xFFU);
  out[2] = static_cast<uint8_t>((raw_humidity >> 4) & 0xFFU);
  out[3] = static_cast<uint8_t>(((raw_humidity & 0x0FU) << 4) |
                                ((raw_temperature >> 16) & 0x0FU));
  out[4] = static_cast<uint8_t>((raw_temperature >> 8) & 0xFFU);
  out[5] = static_cast<uint8_t>(raw_temperature & 0xFFU);
  out[6] = Aht20::crc8(out.data(), 6);
  return out;
}

// 42.50 C and 50.00 %RH, comfortably inside the "plant is fine" band so the app
// classification has a defined answer. The raw values are the ones that land on those
// numbers exactly; see tests/test_comms.cpp for the arithmetic.
constexpr uint32_t kRawTemp42_5 = 484'967;
constexpr uint32_t kRawRh50 = 524'288;

// The same spot the acquisition tests use for "a plausible probe reading": between the
// calibration endpoints in board.h (dry 3600, wet 1400), so the moisture conversion has a
// defined answer.
constexpr uint16_t kProbeCountsMid = 2'500;

class NodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    bus.set_read_data(reply(kRawTemp42_5, kRawRh50));

    hw.i2c = bus.iface();
    hw.rtos = rtos.iface();
    hw.gpio = gpio.iface();
    hw.uart = uart.iface();
    hw.watchdog = watchdog.iface();
    hw.system = system.iface();
    hw.adc = adc.iface();
    hw.actuator = actuator.iface();
    hw.clock = clock.iface();

    node = std::make_unique<Node>(hw);
  }

  // Bring the node up the way the target does, and fail the test loudly if it cannot.
  void init_node() {
    ASSERT_TRUE(node->init());
    uart.clear_transmitted();
  }

  // Hand the acquisition one complete block, as the DMA half-transfer interrupt does.
  void deliver_blocks(int count, uint16_t value = kProbeCountsMid) {
    for (int i = 0; i < count; ++i) {
      adc.deliver_constant(value);
    }
  }

  // Tick the app task `count` times, which is how the drain cadence and the control
  // period are driven on the host: the target's task loop calls the same function on its
  // own 25 ms schedule, and the tests call it directly.
  void tick_app(int count) {
    for (int i = 0; i < count; ++i) {
      node->tick(TaskId::App);
    }
  }

  FakeI2c bus;
  FakeRtos rtos;
  test::FakeGpio gpio;
  FakeUart uart;
  FakeWatchdog watchdog;
  test::FakeSystem system;
  FakeAdc adc;
  FakeActuator actuator;
  FakeClock clock;
  NodeHardware hw;
  std::unique_ptr<Node> node;
};

// ---------------------------------------------------------------------------
// FR-0.1 — five concurrent tasks, with the designed priorities
// ---------------------------------------------------------------------------
TEST_F(NodeTest, CreatesExactlyFiveTasksWithTheDesignedPriorities) {
  init_node();
  ASSERT_TRUE(node->start());

  ASSERT_EQ(rtos.tasks_created(), 5);
  EXPECT_EQ(node->stats().tasks_created, 5u);
  EXPECT_EQ(kTaskCount, 5u);

  // The priorities are the design, not an incidental detail: the supervisor must
  // outrank everything, and the display must be lowest because it is the long-running
  // holder of the I2C mutex.
  const std::vector<rtos_task *> &tasks = rtos.tasks();
  ASSERT_EQ(tasks.size(), 5u);

  EXPECT_EQ(tasks[0]->name, "supervisor");
  EXPECT_EQ(tasks[0]->priority, 4u);
  EXPECT_EQ(tasks[1]->name, "sensor");
  EXPECT_EQ(tasks[1]->priority, 3u);
  EXPECT_EQ(tasks[2]->name, "app");
  EXPECT_EQ(tasks[2]->priority, 3u);
  EXPECT_EQ(tasks[3]->name, "comms");
  EXPECT_EQ(tasks[3]->priority, 2u);
  EXPECT_EQ(tasks[4]->name, "display");
  EXPECT_EQ(tasks[4]->priority, 1u);

  // Every task must outrank the idle task and be distinguishable from every other.
  for (const rtos_task *task : tasks) {
    EXPECT_GE(task->priority, 1u);
    EXPECT_GT(task->stack_words, 0u);
  }
  EXPECT_GT(tasks[0]->priority, tasks[4]->priority);
}

TEST_F(NodeTest, TaskStacksMatchTheConfiguredBudget) {
  init_node();
  ASSERT_TRUE(node->start());

  const std::vector<rtos_task *> &tasks = rtos.tasks();
  EXPECT_EQ(tasks[0]->stack_words, task_stack_words(TaskId::Supervisor));
  EXPECT_EQ(tasks[1]->stack_words, task_stack_words(TaskId::Sensor));
  EXPECT_EQ(tasks[2]->stack_words, task_stack_words(TaskId::App));
  EXPECT_EQ(tasks[3]->stack_words, task_stack_words(TaskId::Comms));
  EXPECT_EQ(tasks[4]->stack_words, task_stack_words(TaskId::Display));

  // The whole set has to fit the F103C8T6's 20 KB with room for the kernel and the
  // main stack, so the budget is asserted rather than assumed.
  uint32_t words = 0;
  for (const rtos_task *task : tasks) {
    words += task->stack_words;
  }
  const uint32_t bytes = words * 4U;
  EXPECT_LT(bytes, 8U * 1024U) << "task stacks alone would leave too little RAM";
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------
TEST_F(NodeTest, InitStartsTheWatchdogAndWatchesEveryTaskButItself) {
  init_node();

  EXPECT_TRUE(watchdog.started());
  EXPECT_EQ(watchdog.timeout_ms(), kWatchdogTimeoutMs);

  // The four worker tasks are watched; the supervisor is not, because it is the watcher.
  // It is covered by the watchdog through whether it manages to feed.
  EXPECT_NE(node->supervisor().task(TaskId::Sensor), nullptr);
  EXPECT_NE(node->supervisor().task(TaskId::Display), nullptr);
  EXPECT_NE(node->supervisor().task(TaskId::App), nullptr);
  EXPECT_NE(node->supervisor().task(TaskId::Comms), nullptr);
  EXPECT_EQ(node->supervisor().task(TaskId::Supervisor), nullptr);
}

TEST_F(NodeTest, InitConfiguresThePinsTheNodeDrives) {
  init_node();

  EXPECT_TRUE(gpio.configured(kPinStatusLed));
  EXPECT_TRUE(gpio.configured(kPinTraceControl));
  EXPECT_EQ(gpio.outputs_configured(), 4);
}

TEST_F(NodeTest, InitDescribesItselfOverTheHostLink) {
  system.set_reset_cause(HAL_RESET_WATCHDOG);
  system.set_boot_count(3);

  ASSERT_TRUE(node->init());

  const std::string banner = uart.transmitted();
  EXPECT_NE(banner.find(kNodeName), std::string::npos);
  EXPECT_NE(banner.find("watchdog"), std::string::npos);
  EXPECT_NE(banner.find("boots=3"), std::string::npos);
}

TEST_F(NodeTest, ADeadSensorDoesNotStopTheNodeBooting) {
  // A node that cannot boot far enough to report its own fault is less useful than one
  // that comes up and complains, so a silent I2C bus must not be fatal.
  bus.set_transfer_status(HAL_I2C_NACK);

  EXPECT_TRUE(node->init());
  EXPECT_TRUE(node->start());
  EXPECT_TRUE(node->canvas().framebuffer().bytes() != nullptr);

  node->tick(TaskId::Sensor);
  EXPECT_FALSE(node->sensor().snapshot().reading.valid);
  EXPECT_EQ(node->app_state().plant, PlantState::Unknown);
}

// ---------------------------------------------------------------------------
// FR-0.2 — all bus access goes through the arbiter
// ---------------------------------------------------------------------------
TEST_F(NodeTest, SensorAndDisplayShareTheBusWithoutEverOverlapping) {
  init_node();

  // Interleave the two bus users the way the scheduler would, for enough cycles to
  // cover many display updates and sensor conversions.
  for (int i = 0; i < 60; ++i) {
    node->tick(TaskId::Sensor);
    node->tick(TaskId::App);
    node->tick(TaskId::Display);
    node->tick(TaskId::Comms);
  }

  const ArbiterStats stats = node->arbiter().stats();

  // The FR-0.2 pass condition: no contention faults.
  EXPECT_EQ(stats.failed_transfers, 0u);
  EXPECT_EQ(stats.bus_faults, 0u);
  EXPECT_EQ(bus.overlaps(), 0);
  EXPECT_EQ(bus.max_concurrent_transfers(), 1);
  EXPECT_TRUE(rtos.locks_balanced());
  EXPECT_EQ(rtos.max_mutex_depth(), 1);

  // And the evidence that the bus really was used by both, so the test is not vacuous.
  EXPECT_GT(stats.transfers, 100u);
  EXPECT_GT(bus.dma_writes(), 0) << "the display never used the bus";
}

// ---------------------------------------------------------------------------
// FR-0.3 — recovery without a power cycle
// ---------------------------------------------------------------------------
TEST_F(NodeTest, AStuckBusIsReleasedAndTheNodeKeepsWorking) {
  init_node();

  // A slave holding SDA releases after two clocks; the first transfer attempt fails with
  // a bus error. The arbiter must recover and retry, not reset the node.
  bus.hold_sda_low_for_clocks(2);
  bus.fail_transfers(1, HAL_I2C_BUS_ERROR);

  node->tick(TaskId::Sensor);

  const ArbiterStats stats = node->arbiter().stats();
  EXPECT_EQ(stats.recoveries, 1u);
  EXPECT_EQ(stats.recovery_failures, 0u);
  EXPECT_EQ(stats.bus_faults, 0u);

  // Nine clocks and one deliberate STOP, which is exactly what the FR-0.3 scope capture
  // is meant to show.
  EXPECT_EQ(bus.recovery_clocks_in_release(), 9);
  EXPECT_EQ(bus.stop_conditions(), 1);

  // And the node carried on: the measurement after the recovery succeeded.
  EXPECT_TRUE(node->sensor().snapshot().reading.valid);
}

TEST_F(NodeTest, RecoveryCanBeTriggeredOnDemandFromTheHost) {
  init_node();
  bus.hold_sda_low_for_clocks(test::kNeverRelease);

  uart.push_rx("recover\n");
  node->tick(TaskId::Comms);

  EXPECT_EQ(bus.recoveries_run(), 1);
  EXPECT_NE(uart.transmitted().find("recover sda-stuck-low"), std::string::npos);
}

// ---------------------------------------------------------------------------
// FR-0.4 — an independent task feeds the watchdog, and withholds when a task dies
// ---------------------------------------------------------------------------
TEST_F(NodeTest, SupervisorFeedsTheWatchdogWhileEveryTaskReportsIn) {
  init_node();

  node->tick(TaskId::Supervisor);
  ASSERT_EQ(watchdog.feeds(), 1);

  for (int i = 0; i < 5; ++i) {
    node->tick(TaskId::Sensor);
    node->tick(TaskId::App);
    node->tick(TaskId::Display);
    node->tick(TaskId::Comms);
    node->tick(TaskId::Supervisor);
  }

  EXPECT_EQ(watchdog.feeds(), 6);
  EXPECT_EQ(node->supervisor().stats().misses, 0u);
}

TEST_F(NodeTest, AStalledTaskStopsTheWatchdogBeingFed) {
  init_node();

  // The display task never runs again. The others keep reporting in, so this proves the
  // watchdog is tied to *every* task rather than to the node looking busy.
  node->tick(TaskId::Supervisor);
  ASSERT_EQ(watchdog.feeds(), 1);

  // Each sensor tick advances the virtual clock by its conversion time, so this walks
  // well past the display's 200 ms x 3 deadline.
  for (int i = 0; i < 40; ++i) {
    node->tick(TaskId::Sensor);
    node->tick(TaskId::App);
    node->tick(TaskId::Comms);
  }

  node->tick(TaskId::Supervisor);

  EXPECT_EQ(watchdog.feeds(), 1) << "the watchdog was fed with a dead display task";
  EXPECT_EQ(node->supervisor().stats().misses, 1u);
  EXPECT_TRUE(node->supervisor().trip().tripped);
  EXPECT_EQ(node->supervisor().trip().task, TaskId::Display);

  // A node whose own watchdog is about to reset it says so over the link first.
  uart.push_rx("stat\n");
  node->tick(TaskId::Comms);
  EXPECT_NE(uart.transmitted().find("miss=1"), std::string::npos);
}

TEST_F(NodeTest, HeartbeatsAreRecordedForEveryWorkerTask) {
  init_node();

  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  node->tick(TaskId::Display);
  node->tick(TaskId::Comms);

  EXPECT_EQ(node->supervisor().task(TaskId::Sensor)->beats, 1u);
  EXPECT_EQ(node->supervisor().task(TaskId::App)->beats, 1u);
  EXPECT_EQ(node->supervisor().task(TaskId::Display)->beats, 1u);
  EXPECT_EQ(node->supervisor().task(TaskId::Comms)->beats, 1u);
}

// ---------------------------------------------------------------------------
// FR-0.5 — the display path is DMA-driven and only refreshes what changed
// ---------------------------------------------------------------------------
TEST_F(NodeTest, DisplayFlushesOverDmaAndSkipsWhenNothingChanged) {
  init_node();

  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  ASSERT_TRUE(node->sensor().snapshot().reading.valid);

  const int dma_before = bus.dma_writes();
  node->tick(TaskId::Display);
  const int dma_after_first = bus.dma_writes();

  EXPECT_GT(dma_after_first, dma_before) << "the display task never wrote to the panel";
  EXPECT_EQ(node->stats().display_renders, 1u);

  // Same reading, so the screen would be identical: the panel must cost nothing at all.
  node->tick(TaskId::Display);
  node->tick(TaskId::Display);
  EXPECT_EQ(bus.dma_writes(), dma_after_first) << "an unchanged screen cost bus traffic";
  EXPECT_EQ(node->stats().display_renders, 1u);
  EXPECT_EQ(node->stats().display_skips, 2u);
}

TEST_F(NodeTest, DisplayNeverUsesTheBlockingBusPath) {
  init_node();
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);

  // Any transfer the display makes must be a DMA one; the blocking path belongs to the
  // sensor, whose transfers are too short to be worth a DMA setup.
  const int blocking_before = bus.count(test::BusOp::Transfer);
  node->tick(TaskId::Display);
  EXPECT_EQ(bus.count(test::BusOp::Transfer), blocking_before);
  EXPECT_GT(bus.dma_writes(), 0);
}

TEST_F(NodeTest, DisplayReportsAStaleReadingInsteadOfFreezingTheLastNumber) {
  init_node();
  node->tick(TaskId::Sensor);
  node->tick(TaskId::Display);
  ASSERT_EQ(node->stats().display_renders, 1u);
  ASSERT_TRUE(node->sensor().snapshot().fresh());

  // The sensor stops answering. The last good reading is deliberately retained, so
  // `reading.valid` stays true and a consumer that looked only at that would show a
  // frozen number forever — indistinguishable from a working node.
  bus.fail_address(kAddrAht20, HAL_I2C_NACK);
  for (int i = 0; i < 4; ++i) {
    node->tick(TaskId::Sensor);
  }

  const SensorSnapshot snapshot = node->sensor().snapshot();
  EXPECT_TRUE(snapshot.reading.valid) << "the last good reading should be retained";
  EXPECT_FALSE(snapshot.fresh()) << "but it must not claim to be current";
  EXPECT_GE(snapshot.consecutive_failures, 2u);

  // The staleness is what makes the panel repaint, which is how a viewer learns the
  // difference between a steady greenhouse and a dead sensor.
  node->tick(TaskId::Display);
  EXPECT_EQ(node->stats().display_renders, 2u);

  // Once the sensor recovers, the numbers come back.
  bus.clear_address_failures();
  node->tick(TaskId::Sensor);
  EXPECT_TRUE(node->sensor().snapshot().fresh());
  node->tick(TaskId::Display);
  EXPECT_EQ(node->stats().display_renders, 3u);
}

// ---------------------------------------------------------------------------
// The application layer
// ---------------------------------------------------------------------------
TEST_F(NodeTest, AppClassifiesTheReadingAndOwnsTheState) {
  init_node();

  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  EXPECT_EQ(node->app_state().plant, PlantState::Ok);

  // Dry air: raw 94372 is 94372 * 100000 / 1048576 = 8999.9 -> 8999 milli-RH, below the
  // 30 %RH dry threshold.
  bus.set_read_data(reply(kRawTemp42_5, 94'372)); // 9.0 %RH
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  EXPECT_EQ(node->app_state().plant, PlantState::Dry);

  // Very humid air: raw 954204 -> 91000.0 milli-RH, above the 70 %RH wet threshold.
  bus.set_read_data(reply(kRawTemp42_5, 954'204)); // 91.0 %RH
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  EXPECT_EQ(node->app_state().plant, PlantState::Wet);
}

TEST_F(NodeTest, AppRefusesToActOnAnInvalidReading) {
  init_node();
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  ASSERT_EQ(node->app_state().plant, PlantState::Ok);

  bus.fail_address(kAddrAht20, HAL_I2C_NACK);
  node->tick(TaskId::Sensor);
  const uint32_t rejected_before = node->app_state().rejected_readings;
  node->tick(TaskId::App);

  // The state is held rather than replaced. An unreadable sensor is not evidence that
  // conditions changed, and a wrong actuator decision is worse than a stale one.
  EXPECT_EQ(node->app_state().plant, PlantState::Ok);
  EXPECT_EQ(node->app_state().rejected_readings, rejected_before + 1);
}

// ---------------------------------------------------------------------------
// The comms task
// ---------------------------------------------------------------------------
TEST_F(NodeTest, CommsAnswersCommandsFromTheHost) {
  init_node();

  uart.push_rx("ver\r\n");
  node->tick(TaskId::Comms);

  EXPECT_EQ(node->comms().stats().commands_received, 1u);
  EXPECT_NE(uart.transmitted().find("ver 0.1.0"), std::string::npos);
}

TEST_F(NodeTest, CommsReportsTelemetryOnItsOwnCadence) {
  init_node();
  node->tick(TaskId::Sensor);

  node->tick(TaskId::Comms);
  const std::string first = uart.transmitted();
  EXPECT_NE(first.find("T=42.50"), std::string::npos);

  // Inside the telemetry period the link stays quiet.
  node->tick(TaskId::Comms);
  EXPECT_EQ(uart.transmitted(), first);
}

TEST_F(NodeTest, TracingPinsAreDrivenForTheMeasurementPlan) {
  init_node();
  const int toggles_before = gpio.toggles();

  // A block arrives: the ISR marker (kPinTraceIsr) is toggled in interrupt context by the
  // node's stamping wrapper, and the task marker (kPinTraceTask) at the first app task
  // entry that observes it. That pair is NFR-1.9's capture.
  deliver_blocks(1);
  node->tick(TaskId::App);
  EXPECT_GT(gpio.toggles(), toggles_before);
  EXPECT_EQ(gpio.last_toggled_pin(), kPinTraceTask);

  // And the control evaluation's marker: NFR-1.8 pairs two of these to measure the loop's
  // period. Deliberately at the evaluation and not at task entry — the app task wakes 40
  // times a second while the loop runs once a second, so a marker at entry would measure
  // the scheduler.
  deliver_blocks(kAdcBlocksPerControlPeriod - 1);
  tick_app(kAcquisitionDrainEveryAppTicks);
  EXPECT_EQ(gpio.last_toggled_pin(), kPinTraceControl);

  node->tick(TaskId::Supervisor);
  EXPECT_GT(gpio.writes(), 0);
}

TEST_F(NodeTest, ATickForAnInvalidTaskIdIsIgnored) {
  init_node();

  node->tick(TaskId::Count);

  for (uint32_t ticks : node->stats().ticks) {
    EXPECT_EQ(ticks, 0u);
  }
}


// ---------------------------------------------------------------------------
// Phase 1 — the analog channel, the control loop and the gate
//
// The tests below are about *wiring*, which is a different claim from the ones in
// tests/test_acquisition.cpp and tests/test_control.cpp: those show that each service
// behaves, and these show that node.cpp is actually connected to them. Before this
// section existed, none of the Phase 1 code was referenced from the image at all.
// ---------------------------------------------------------------------------
TEST_F(NodeTest, TheActuatorIsOffAfterInitAndBeforeAnyReading) {
  init_node();

  EXPECT_TRUE(actuator.initialised());
  // The whole command history so far, in order: init, then off, and nothing else. init()
  // is required to leave the gate de-energised (hal/actuator.h), which is the node-level
  // half of the claim the STM32 implementation makes by driving the pin low *before* it
  // becomes an output.
  EXPECT_EQ(actuator.commands(), (std::string{FakeActuator::kInit, FakeActuator::kOff}));
  EXPECT_EQ(actuator.on_calls(), 0);
  EXPECT_FALSE(actuator.on());

  // And it stays off while nothing has been measured. Forty ticks is one and a half control
  // periods' worth of gate decisions, so a node that energised the gate on an absent
  // reading would have done it here (decision 8).
  tick_app(40);
  EXPECT_EQ(actuator.on_calls(), 0);
  EXPECT_FALSE(actuator.on());
  EXPECT_GT(node->stats().fail_safe_ticks, 0u)
      << "a tick with no usable reading must be counted as a fail-safe tick";
}

TEST_F(NodeTest, AGateThatCannotBeInitialisedIsNeverEnergised) {
  // hal/actuator.h: a failed init() means the pump is *unavailable*, not stopped, because a
  // gate driven through an unconfigured pin may sit high. The node's response is to drop
  // the handle, so nothing in this firmware writes that pin again.
  actuator.set_init_ok(false);
  init_node();

  EXPECT_FALSE(actuator.initialised());
  deliver_blocks(1, kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks * 2);

  EXPECT_EQ(actuator.on_calls(), 0);
}

TEST_F(NodeTest, TheControlLoopRecomputesOncePerTenAcquisitionBlocksNotOncePerTick) {
  init_node();
  EXPECT_EQ(node->control().stats().evaluations, 0u);

  // One block, then one drain. The published value carries block_index 0, which is the
  // first control period.
  deliver_blocks(1);
  tick_app(kAcquisitionDrainEveryAppTicks);
  EXPECT_EQ(node->control().stats().evaluations, 1u);
  EXPECT_EQ(node->stats().control_evaluations, 1u);

  // Ten more drains' worth of ticks with no new data. The loop must not re-evaluate: no new
  // control period has begun, and this is the assertion that would fail if the evaluation
  // were on a task delay rather than on the block count (design-spec.md §6). The gate is
  // still being driven on every one of those ticks — the window is a separate decision.
  const uint32_t ticks_before = node->stats().ticks[static_cast<size_t>(TaskId::App)];
  tick_app(kAcquisitionDrainEveryAppTicks * 10);
  EXPECT_EQ(node->control().stats().evaluations, 1u);
  EXPECT_EQ(node->stats().ticks[static_cast<size_t>(TaskId::App)] - ticks_before,
            kAcquisitionDrainEveryAppTicks * 10U)
      << "the app task must still run on every tick; only the loop is gated on blocks";

  // A ring's worth of blocks takes block_index past the boundary, so exactly one more
  // evaluation — with the skipped groups counted rather than evaluated.
  deliver_blocks(static_cast<int>(kAdcBlocksPerControlPeriod));
  tick_app(kAcquisitionDrainEveryAppTicks);
  EXPECT_EQ(node->control().stats().evaluations, 2u);
  EXPECT_EQ(node->stats().control_evaluations, 2u);
}

TEST_F(NodeTest, TheControlPeriodIsMeasuredWithTheMicrosecondClock) {
  init_node();

  // The first evaluation has no previous timestamp to measure against, so it records a
  // decision and no period sample. Anything else would be inventing the interval.
  clock.set_now_us(5'000'000);
  deliver_blocks(1, kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);
  ASSERT_EQ(node->control().stats().evaluations, 1u);
  EXPECT_EQ(node->timing().period().count(), 0u);

  // A second control period, a second of DWT time later.
  clock.advance_us(kControlPeriodMs * 1'000U);
  deliver_blocks(static_cast<int>(kAdcBlocksPerControlPeriod), kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);

  ASSERT_EQ(node->timing().period().count(), 1u);
  EXPECT_EQ(node->timing().period().min_us(), kControlPeriodMs * 1'000U);
  EXPECT_EQ(node->timing().period().max_us(), kControlPeriodMs * 1'000U);
  // A period that matches the nominal exactly is a zero deviation, which is the first
  // bucket of the histogram. The point of asserting it is that the series is a *deviation*
  // series: a node that fed it raw values would put this sample in a different bucket.
  EXPECT_EQ(node->timing().period().bucket(0), 1u);
  EXPECT_EQ(node->timing().nominal_period_us(), kControlPeriodMs * 1'000U);
}

TEST_F(NodeTest, TheIsrToTaskLatencyIsMeasuredFromTheInterruptStamp) {
  init_node();

  // The ISR stamps the block at 1,000,000 us; the app task sees it 3 ms later. NFR-1.9's
  // number is that difference, taken from the same two edges the logic analyzer sees on
  // kPinTraceIsr and kPinTraceTask.
  clock.set_now_us(1'000'000);
  deliver_blocks(1);
  clock.advance_us(3'000);
  node->tick(TaskId::App);

  ASSERT_EQ(node->timing().latency().count(), 1u);
  EXPECT_EQ(node->timing().latency().min_us(), 3'000u);
  EXPECT_EQ(node->timing().latency().max_us(), 3'000u);
}

TEST_F(NodeTest, TheGateFollowsTheWindowAndIsOffOutsideIt) {
  init_node();

  // Dry soil: 3600 counts is the air-dry calibration endpoint, so the moisture reading is 0
  // permille against a 450 permille setpoint. The law asks for 900 permille of duty, the cap
  // clamps it to 500, and 500 permille of a 1000 ms window is a 500 ms on-time — comfortably
  // above the 50 ms minimum, so this is a window the gate is meant to act on.
  deliver_blocks(1, kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);

  ASSERT_EQ(node->control().stats().evaluations, 1u);
  ASSERT_EQ(node->control().last().on_time_ms, 500u);
  EXPECT_TRUE(actuator.on()) << "the gate must be on inside the commanded on-time";
  EXPECT_EQ(node->stats().gate_on_edges, 1u);

  // Past the on-time, and the gate has to go off — on a 25 ms tick, so the edge is within
  // one task period of the commanded end of the pulse.
  rtos.advance_ms(node->control().last().on_time_ms + 1U);
  node->tick(TaskId::App);
  EXPECT_FALSE(actuator.on());
  EXPECT_EQ(node->stats().gate_off_edges, 1u);

  // And it stays off for the rest of the window rather than restarting.
  rtos.advance_ms(10);
  node->tick(TaskId::App);
  EXPECT_EQ(actuator.on_calls(), 1);
}

TEST_F(NodeTest, AStaleAnalogReadingDrivesTheGateOff) {
  init_node();

  deliver_blocks(1, kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);
  ASSERT_TRUE(actuator.on());

  // The producer stops. This is exactly what a dead DMA channel or a stopped trigger looks
  // like from the consumer's side: a drain that finds nothing, which AcquisitionService
  // records as a failed publish cycle, leaving the last good value in place with `valid`
  // still true and `fresh()` false.
  tick_app(kAcquisitionDrainEveryAppTicks);

  const AnalogSnapshot stale = node->acquisition().snapshot();
  EXPECT_TRUE(stale.valid) << "the last good value is deliberately kept";
  EXPECT_FALSE(stale.fresh()) << "but it must not claim to be current";
  EXPECT_FALSE(actuator.on()) << "a lost probe must stop the pump (decision 8)";
  EXPECT_GT(node->stats().fail_safe_ticks, 0u);

  // The latch holds for the rest of the window even if a reading arrives before the next
  // control period: resuming a pulse whose command came from a reading that has since been
  // replaced would deliver water on a decision the loop no longer stands behind.
  clock.advance_us(1'000);
  deliver_blocks(1, kProbeCountsDry);
  node->tick(TaskId::App);
  EXPECT_FALSE(actuator.on()) << "the fail-safe must hold, not just blink";

  // The next control period re-decides on a fresh reading, which is the only place the
  // latch is cleared.
  deliver_blocks(static_cast<int>(kAdcBlocksPerControlPeriod), kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);
  EXPECT_TRUE(actuator.on()) << "a control period with a fresh reading may run the pump";
}

TEST_F(NodeTest, ThePhaseOneStatsAccountForWhatHappened) {
  init_node();

  // Three blocks delivered, of which the drain can only publish the newest: the ring holds
  // two, so the third evicts the first. The counters are the evidence that the path ran at
  // all, and they are what `stat` reports over the link.
  deliver_blocks(3, kProbeCountsDry);
  tick_app(kAcquisitionDrainEveryAppTicks);

  const AcquisitionStats adc_stats = node->acquisition().stats();
  EXPECT_EQ(adc_stats.blocks_received, 3u);
  EXPECT_EQ(adc_stats.blocks_processed, 2u);
  EXPECT_EQ(adc_stats.blocks_dropped, 1u);
  EXPECT_EQ(adc_stats.publishes, 2u);
  EXPECT_EQ(adc_stats.hardware_blocks, 3u) << "the HAL's own count of what it delivered";

  EXPECT_EQ(node->stats().acquisition_drains, 1u);
  EXPECT_EQ(node->stats().control_evaluations, 1u);
  EXPECT_EQ(node->stats().gate_on_edges, 1u);

  // Ticks before the first drain had no reading at all, so they are fail-safe ticks — that
  // is what "absent" means for decision 8. What must NOT happen is a tick that has a fresh
  // reading counting as one of them, so the count is pinned across the next tick.
  EXPECT_GT(node->stats().fail_safe_ticks, 0u);
  const uint32_t fail_safe_after_drain = node->stats().fail_safe_ticks;
  node->tick(TaskId::App);
  EXPECT_EQ(node->stats().fail_safe_ticks, fail_safe_after_drain)
      << "a tick with a fresh reading must not be counted as a fail-safe tick";

  // What the analog channel asked for, pinned against board.h rather than against the
  // service: FR-1.1's rate and block size are a hardware decision, and the number that
  // reaches the HAL is the one that matters.
  EXPECT_EQ(adc.rate_hz(), kAdcSampleRateHz);
  EXPECT_EQ(adc.block_samples(), kAdcBlockSamples);
  EXPECT_TRUE(adc.started());
  EXPECT_EQ(adc.start_calls(), 1);

  // And the same numbers on the wire, which is how they are visible on a board with no
  // debugger attached.
  uart.push_rx("stat\n");
  node->tick(TaskId::Comms);
  EXPECT_NE(uart.transmitted().find("adc=3/2/1"), std::string::npos);
  EXPECT_NE(uart.transmitted().find("ctl=1/"), std::string::npos);
}

TEST_F(NodeTest, AnAdcThatCannotStartLeavesTheNodeRunningAndTheGateOff) {
  // hal/adc.h's false return is "this rate or configuration is impossible". The node must
  // come up anyway — the I2C sensor and the display are unaffected — and must not let the
  // control loop act on readings that will never arrive.
  adc.set_start_ok(false);
  init_node();

  EXPECT_FALSE(node->acquisition().started());
  EXPECT_EQ(node->acquisition().stats().start_failures, 1u);

  tick_app(kAcquisitionDrainEveryAppTicks * 2);
  EXPECT_EQ(actuator.on_calls(), 0);
  EXPECT_GT(node->stats().fail_safe_ticks, 0u);

  // The rest of the node still works: this is a degraded node, not a dead one.
  node->tick(TaskId::Sensor);
  EXPECT_TRUE(node->sensor().snapshot().reading.valid);
}

TEST_F(NodeTest, TheTuningStoreReachesTheControlLoopAndTheHostLink) {
  init_node();

  // FR-1.4: the setpoint is adjustable at run time without reflashing, and the path from a
  // command on the link to the value the loop reads is the wiring this asserts. A node whose
  // CommsSources had no tuning store would answer `set` with "unsupported" and keep
  // controlling to the compiled-in default — a failure that is invisible from the wire.
  uart.push_rx("set sp=60.0\n");
  node->tick(TaskId::Comms);

  EXPECT_EQ(node->tuning().load().setpoint_permille, 600);
  EXPECT_EQ(node->comms().stats().tuning_sets, 1u);

  uart.clear_transmitted();
  uart.push_rx("get\n");
  node->tick(TaskId::Comms);
  EXPECT_NE(uart.transmitted().find("sp=60.0"), std::string::npos);

  // And the loop acts on the new setpoint: a reading of 500 permille (2500 counts) is now
  // 100 permille below it rather than above it, so the pump runs.
  node->tick(TaskId::Sensor);
  deliver_blocks(1, kProbeCountsMid);
  tick_app(kAcquisitionDrainEveryAppTicks);
  EXPECT_GT(node->control().last().setpoint_permille, 0);
  EXPECT_EQ(node->control().last().setpoint_permille, 600);
}

} // namespace
} // namespace node
