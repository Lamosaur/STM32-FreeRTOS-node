/* Integration tests for the whole node: the five-task architecture and the four Phase 0
 * behaviours layered on it.
 *
 * This is the test that makes FR-0.6 pay off. The node owns its HAL interfaces rather
 * than reaching for globals, so the same `Node::tick()` that runs under FreeRTOS on the
 * F103 runs here one iteration at a time against doubles — no board, no scheduler, and
 * no timing flakiness. Each FR below gets an assertion that would fail if the mechanism
 * were removed rather than merely degraded.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "app/board.h"
#include "app/node.h"
#include "support/fake_i2c.h"
#include "support/fake_peripherals.h"
#include "support/fake_rtos.h"
#include "support/fake_watchdog.h"

namespace node {
namespace {

using test::FakeI2c;
using test::FakeRtos;
using test::FakeUart;
using test::FakeWatchdog;

// A correctly checksummed SHT31 reply for the given raw words.
std::vector<uint8_t> reply(uint16_t raw_temperature, uint16_t raw_humidity) {
  std::vector<uint8_t> out(6, 0);
  out[0] = static_cast<uint8_t>(raw_temperature >> 8);
  out[1] = static_cast<uint8_t>(raw_temperature & 0xFF);
  out[2] = Sht31::crc8(out.data(), 2);
  out[3] = static_cast<uint8_t>(raw_humidity >> 8);
  out[4] = static_cast<uint8_t>(raw_humidity & 0xFF);
  out[5] = Sht31::crc8(&out[3], 2);
  return out;
}

// Raw 0x8000 on both channels: 42.501 C and 50.000 %RH, comfortably inside the
// "plant is fine" band so the app classification has a defined answer.
constexpr uint16_t kRawMid = 32'768;

class NodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    bus.set_read_data(reply(kRawMid, kRawMid));

    hw.i2c = bus.iface();
    hw.rtos = rtos.iface();
    hw.gpio = gpio.iface();
    hw.uart = uart.iface();
    hw.watchdog = watchdog.iface();
    hw.system = system.iface();

    node = std::make_unique<Node>(hw);
  }

  // Bring the node up the way the target does, and fail the test loudly if it cannot.
  void init_node() {
    ASSERT_TRUE(node->init());
    uart.clear_transmitted();
  }

  FakeI2c bus;
  FakeRtos rtos;
  test::FakeGpio gpio;
  FakeUart uart;
  FakeWatchdog watchdog;
  test::FakeSystem system;
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
  bus.fail_address(kAddrSht31, HAL_I2C_NACK);
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

  // Dry air.
  bus.set_read_data(reply(kRawMid, 6'000)); // about 9 %RH
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  EXPECT_EQ(node->app_state().plant, PlantState::Dry);

  // Very humid air.
  bus.set_read_data(reply(kRawMid, 60'000)); // about 91 %RH
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  EXPECT_EQ(node->app_state().plant, PlantState::Wet);
}

TEST_F(NodeTest, AppRefusesToActOnAnInvalidReading) {
  init_node();
  node->tick(TaskId::Sensor);
  node->tick(TaskId::App);
  ASSERT_EQ(node->app_state().plant, PlantState::Ok);

  bus.fail_address(kAddrSht31, HAL_I2C_NACK);
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

  node->tick(TaskId::App);

  // NFR-1.8 and NFR-1.9 are verified with a logic analyzer, so the marker has to
  // actually toggle.
  EXPECT_GT(gpio.toggles(), 0);
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

} // namespace
} // namespace node
