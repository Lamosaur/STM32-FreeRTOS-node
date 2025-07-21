/* board.h — every tunable number the application core needs, in one place.
 *
 * This header settles the Phase 0 decisions listed in design-spec.md §5.3 that are
 * not hardware-determined: task count and priorities (#1), the contention-stress
 * duration (#2), the recovery sequence shape (#3), the refresh strategy (#5), and the
 * sensor set (#8). Keeping them here rather than scattered through the sources is what
 * makes them reviewable against the spec.
 *
 * Nothing here is measured yet. Task periods and stack sizes are provisional and are
 * to be replaced by the NFR-1.10/NFR-1.11 measurements; the comments say which
 * requirement will correct each number.
 */
#ifndef APP_BOARD_H
#define APP_BOARD_H

#include <cstddef>
#include <cstdint>

namespace node {

// ---------------------------------------------------------------------------
// Node identity and purpose
// ---------------------------------------------------------------------------
// A greenhouse node: soil moisture is the primary (analog) quantity, on the Phase 1
// timer-triggered ADC path. Phase 0 reads temperature and humidity over I2C, which is
// what makes the shared-bus arbitration of FR-0.2 a real problem rather than a
// contrived one: two different devices, two different tasks, one bus.
constexpr char kNodeName[] = "greenhouse-node-01";

// Short label for the OLED. Deliberately restricted to characters the panel's 5x7 font
// actually covers — see app/display.h — because an uncovered character renders as a
// filled box, and a header of boxes is worse than a short header.
constexpr char kNodeShortName[] = "NODE";

// ---------------------------------------------------------------------------
// Shared I2C bus
// ---------------------------------------------------------------------------
// All I2C peripherals sit on one bus behind the arbiter (FR-0.2).
constexpr uint32_t kI2cBusHz = 400'000;
constexpr uint32_t kI2cTransferTimeoutMs = 20;

// 7-bit device addresses.
constexpr uint16_t kAddrSht31 = 0x44;   // temperature + humidity (sensor task)
constexpr uint16_t kAddrSsd1306 = 0x3C; // 128x64 OLED (display task)

// SHT31 single-measurement conversion time, high repeatability. The datasheet's
// worst case is 15.5 ms; 20 gives margin without meaningfully changing the 1 s poll
// period. The bus is released for this whole window, not held.
constexpr uint32_t kSht31ConversionMs = 20;

// ---------------------------------------------------------------------------
// I2C bus recovery (FR-0.3)
// ---------------------------------------------------------------------------
// A slave that was mid-byte when the master reset can hold SDA low forever. The
// release is nine SCL pulses with SDA released, then a STOP, then peripheral re-init.
//
// Nine is not arbitrary: a slave that has lost bit alignment can be up to eight bits
// into a byte, so eight clocks are not always enough to walk it to the ACK bit; the
// ninth guarantees it. We always emit all nine rather than stopping early when SDA
// goes high, because a fixed, deterministic edge sequence is what makes the scope
// capture in FR-0.3 a pass/fail check instead of a judgement call.
constexpr int kRecoveryClocks = 9;
constexpr uint32_t kRecoveryHalfPeriodUs = 5; // ~100 kHz bit-banged, well under 400 kHz

// One recovery attempt, followed by one retry of the transfer.
//
// Deliberately not two: a recovery reports whether it cleared the bus, so a second
// identical attempt in the same transfer is redundant — if the bus is still not idle,
// clocking it again changes nothing. Each attempt also holds the I2C mutex (FR-0.2),
// so a longer retry chain would block the other tasks on the bus while achieving
// nothing. Failing after one retry hands the decision back to the caller.
constexpr int kRecoveryMaxAttempts = 1;

// ---------------------------------------------------------------------------
// Scheduling (FR-0.1)
// ---------------------------------------------------------------------------
// Five concurrent tasks. FreeRTOS priorities: a higher number preempts a lower one.
//
// The ordering is not cosmetic. The supervisor is highest so the watchdog is fed on
// time even when everything below is busy. Display is lowest on purpose: it is the
// cosmetic job, and it is also the long-running holder of the I2C mutex, which is
// exactly the priority-inversion case FR-1.7 instruments.
enum class TaskId : uint8_t {
  Supervisor = 0,
  Sensor = 1,
  App = 2,
  Comms = 3,
  Display = 4,
  Count = 5,
};

constexpr uint32_t kTaskCount = static_cast<uint32_t>(TaskId::Count);

// ---- The task budget ------------------------------------------------------
//
// One table rather than three switches, because these properties are reviewed together
// and separately they hide their relationships. Two of them are deliberately equal, and as
// duplicated switch branches those read exactly like a copy-paste slip — which is what a
// linter sees too (clang-tidy's bugprone-branch-clone). Stating them once, side by side,
// says "intended" instead of "suspicious", and makes the whole 20 KB budget readable at a
// glance.
//
// Priorities (FreeRTOS: a higher number preempts a lower one):
//   4 supervisor  highest, because feeding the watchdog late is the one failure that
//                 resets the node
//   3 sensor      the node's primary input; a missed sample is a missed measurement
//   3 app         owns application state, and becomes the Phase 1 control loop. Equal to
//                 the sensor on purpose: neither should preempt the other, and time
//                 slicing between two critical-path tasks is the intended relationship
//   2 comms       a host that stops reading must not stall the node
//   1 display     cosmetic, and the long-running holder of the I2C mutex, which is
//                 exactly the priority-inversion case FR-1.7 instruments
// `name` is what FreeRTOS keeps a pointer to and what a debugger shows, so it is short
// but not cryptic. It lives here rather than in a second table so that a task's whole
// specification is one row.
struct TaskSpec {
  TaskId id;
  const char *name;
  uint32_t priority;
  uint32_t stack_words;
  uint32_t period_ms;
};

// Stack sizes are in words, as FreeRTOS counts them (4 bytes). Provisional: NFR-1.11
// requires these to be right-sized from uxTaskGetStackHighWaterMark() with 25 % headroom
// kept, and the F103C8T6 has only 20 KB of RAM, so raising one is a real budget decision
// rather than a free change.
//
// Periods: Phase 0 carries no control loop (that is Phase 1), so the app task's period is
// the one that will become the control period, which keeps the scheduling measurements
// taken here comparable to Phase 1's.
constexpr TaskSpec kTaskSpecs[kTaskCount] = {
    {TaskId::Supervisor, "supervisor", 4, 192, 250},
    {TaskId::Sensor, "sensor", 3, 320, 1000},
    {TaskId::App, "app", 3, 320, 100},
    {TaskId::Comms, "comms", 2, 384, 50},
    {TaskId::Display, "display", 1, 384, 200},
};

// The table must be indexed by TaskId, because everything below looks a row up by casting
// the id. A row inserted in the wrong place would silently give a task another task's
// priority, so the invariant is checked rather than trusted.
constexpr bool task_table_is_indexed_by_id() {
  for (uint32_t row = 0; row < kTaskCount; ++row) {
    if (static_cast<uint32_t>(kTaskSpecs[row].id) != row) {
      return false;
    }
  }
  return true;
}
static_assert(task_table_is_indexed_by_id(), "kTaskSpecs must be ordered by TaskId");

// An out-of-range id resolves to the supervisor's row rather than reading past the array.
// A wrong id is a programming error, and the alternative — returning a default — would
// hide it at the one place it matters.
constexpr const TaskSpec &task_spec(TaskId id) {
  const uint32_t index = static_cast<uint32_t>(id);
  return kTaskSpecs[index < kTaskCount ? index : 0];
}

constexpr uint32_t task_priority(TaskId id) { return task_spec(id).priority; }
constexpr uint32_t task_stack_words(TaskId id) { return task_spec(id).stack_words; }
constexpr uint32_t task_period_ms(TaskId id) { return task_spec(id).period_ms; }

// ---------------------------------------------------------------------------
// Application thresholds
// ---------------------------------------------------------------------------
// Phase 0 has no actuator, so these only classify the reading for the display and the
// telemetry line. They are where the Phase 1 control loop's setpoint will live.
constexpr int32_t kPlantDryMilliRh = 30'000; // below 30 %RH
constexpr int32_t kPlantWetMilliRh = 70'000; // above 70 %RH

// How many consecutive failed polls make a reading stale enough to stop showing it as a
// number. Two, not one: a single dropped transfer is normal on a shared bus and the
// arbiter already retries it, so reacting immediately would make the display flicker
// between a value and a fault for a bus that is working correctly.
constexpr uint32_t kSensorStaleAfterFailures = 2;

// ---------------------------------------------------------------------------
// Watchdog and heartbeats (FR-0.4)
// ---------------------------------------------------------------------------
// The supervisor feeds the IWDG only while every registered task is inside its
// deadline. A heartbeat deadline is a multiple of the task's period: three missed
// periods means the task is not merely late, it is gone.
constexpr uint32_t kWatchdogTimeoutMs = 1000;
constexpr uint32_t kHeartbeatDeadlinePeriods = 3;

// ---------------------------------------------------------------------------
// Bus contention stress test (FR-0.2, decision §5.3 #2)
// ---------------------------------------------------------------------------
// FR-0.2 needs a defined test rather than an informal one. The stress run drives the
// sensor and display tasks at an artificially short period so that both contend for
// the arbiter on every cycle, for long enough to cover many watchdog and wrap
// boundaries. The pass condition is zero bus-contention faults: every contended
// acquisition is resolved by the mutex, no transfer returns BusError, and the
// contention counter is non-zero (otherwise the test proved nothing).
constexpr uint32_t kStressPeriodMs = 5;
constexpr uint32_t kStressDurationMs = 60'000;

// ---------------------------------------------------------------------------
// OLED display (FR-0.5, decision §5.3 #5)
// ---------------------------------------------------------------------------
constexpr uint16_t kDisplayWidth = 128;
constexpr uint16_t kDisplayHeight = 64;
constexpr size_t kDisplayFramebufferBytes = (kDisplayWidth * kDisplayHeight) / 8;
constexpr uint8_t kDisplayPages = kDisplayHeight / 8;

// A full 128x64 frame is 1024 bytes, which at 400 kHz is ~23 ms of bus time. FR-0.5
// requires partial refresh to beat that, so the display task flushes only the damaged
// page-aligned region and the target is well under 10 ms per update.
constexpr uint32_t kDisplayFullFrameBudgetUs = 23'000;

// ---------------------------------------------------------------------------
// Instrumentation and status pins
// ---------------------------------------------------------------------------
// hal_gpio_iface_t takes an opaque application-level pin id, and these are the ids.
// The HAL owns the mapping from id to port and bit, so the application core compiles
// on the host with no device header.
//
// The trace pins exist for the measurement plan, not for the application: NFR-1.8
// measures control-loop jitter by toggling a pin at control-task entry, and NFR-1.9
// measures worst-case ISR-to-task latency by toggling one in the ISR and another at
// task entry. They cost nothing when no logic analyzer is attached.
constexpr uint32_t kPinStatusLed = 0;  // board LED; heartbeat of last resort
constexpr uint32_t kPinTraceControl = 1; // toggled at control-task entry (NFR-1.8)
constexpr uint32_t kPinTraceIsr = 2;     // toggled in the DMA-complete ISR (NFR-1.9)
constexpr uint32_t kPinTraceTask = 3;    // toggled at the woken task's entry (NFR-1.9)
constexpr uint32_t kPinCount = 4;

// ---------------------------------------------------------------------------
// Comms link
// ---------------------------------------------------------------------------
constexpr uint32_t kUartBaud = 115'200;
constexpr uint32_t kUartWriteTimeoutMs = 50;
constexpr uint32_t kUartReadTimeoutMs = 5;
constexpr size_t kCommsRxBufferBytes = 128;

// ---------------------------------------------------------------------------
// Supervisor diagnostics
// ---------------------------------------------------------------------------
// A node that resets unattended (NFR-0.7) must say why on the next boot, so the reset
// cause and boot count are reported over the comms link.
constexpr uint32_t kTelemetryPeriodMs = 1000;

} // namespace node

#endif // APP_BOARD_H
