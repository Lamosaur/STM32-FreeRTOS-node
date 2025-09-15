/* node.h — the five-task application (FR-0.1) and everything it owns.
 *
 * FR-0.1 requires no fewer than five concurrent tasks, verified by "task count and
 * priorities in code review". Those priorities are the design, so they are worth
 * stating here rather than being read out of a table:
 *
 *   supervisor (4)  highest, because feeding the watchdog late is the one failure that
 *                   resets the node
 *   sensor (3)      the node's primary input; a missed sample is a missed measurement
 *   app (3)         owns application state and runs the Phase 1 control loop: it drains the
 *                   acquisition ring, evaluates the loop once per control period and drives
 *                   the actuator gate. Its 25 ms period exists for the gate's edge
 *                   resolution — see kAppTaskPeriodMs in board.h
 *   comms (2)       a host that stops reading must not stall the node
 *   display (1)     cosmetic, and the long-running holder of the I2C mutex, which is
 *                   exactly the priority-inversion case FR-1.7 instruments
 *
 * The node owns the HAL interfaces it was handed and never reaches for a global one.
 * That is what lets the same class run on the host against test doubles (FR-0.6) and on
 * the F103 against the real HAL, with `tick()` as the single task body in both cases.
 */
#ifndef APP_NODE_H
#define APP_NODE_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "app/acquisition.h"
#include "app/board.h"
#include "app/comms.h"
#include "app/control.h"
#include "app/display.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "app/sensor.h"
#include "app/supervisor.h"
#include "app/timing.h"
#include "hal/adc.h"
#include "hal/actuator.h"
#include "hal/clock.h"
#include "hal/gpio.h"
#include "hal/i2c.h"
#include "hal/system.h"
#include "hal/uart.h"
#include "hal/watchdog.h"
#include "rtos/rtos.h"

namespace node {

// Borrowed hardware handles. Any of the optional ones may be null, and the node
// degrades rather than crashing: a board with no display still runs.
//
// Fields are appended, never inserted. Callers that build this struct positionally (the
// host tests and main_target.cpp's make_hardware()) would silently rebind every field
// after an insertion, and a NodeHardware whose i2c pointer was actually the watchdog is
// exactly the kind of change that compiles and then misbehaves on a bench.
struct NodeHardware {
  hal_i2c_iface_t *i2c = nullptr;
  const rtos_iface_t *rtos = nullptr;
  hal_gpio_iface_t *gpio = nullptr;         // optional: status LED and trace pins
  hal_uart_iface_t *uart = nullptr;         // optional: comms task
  hal_watchdog_iface_t *watchdog = nullptr; // optional: independence is lost without it
  hal_system_iface_t *system = nullptr;     // optional: reset cause and boot count

  // Phase 1. Each is optional in the same sense: a node without an ADC still runs its I2C
  // sensor and its display, and reports the analog channel as absent rather than as zero.
  //
  //  - `adc` is the timer-triggered analog channel (FR-1.1). Without it the acquisition
  //    service never starts and the control loop's fail-safe holds the gate off, which is
  //    the correct behaviour for a node that cannot measure the thing it controls.
  //  - `actuator` is the switched output (FR-1.3). Without it the loop still evaluates and
  //    its decisions are still visible in the stats; only the pin is missing.
  //  - `clock` is the microsecond timestamp (hal/clock.h). Without it the NFR-1.8/NFR-1.9
  //    series stay empty rather than being filled with numbers of the wrong magnitude.
  hal_adc_iface_t *adc = nullptr;
  hal_actuator_iface_t *actuator = nullptr;
  hal_clock_iface_t *clock = nullptr;
};

// ---------------------------------------------------------------------------
// The ISR-side timestamp that NFR-1.9 needs, and why it is a decorator
// ---------------------------------------------------------------------------
// NFR-1.9 is "worst-case ISR-to-task latency, measured and reported", and the measurement
// needs two timestamps: one taken in the interrupt that handed over a block, and one taken
// in the task that picked it up. The task-side one is easy — the app task has a clock. The
// ISR-side one has no home: hal/adc.h is frozen and deliberately carries no timestamps
// (its callback signature is a pointer, a count and a block index), and AcquisitionService
// is the application's consumer, not a place to hang instrumentation the application does
// not use.
//
// So this wraps the ADC interface. AcquisitionService is handed `iface()` instead of the
// real ADC, and this forwards every call to the real one while stamping each block on its
// way through. Three properties make that the smallest place to put it:
//
//   - The stamp happens in the ISR, in the callback the HAL was given, so it is the same
//     edge the logic analyzer sees on kPinTraceIsr. The alternative — reading a counter in
//     the task and subtracting an assumed block boundary — measures the assumption.
//   - It costs one clock read and one GPIO toggle in interrupt context, both register
//     accesses, and neither blocks, allocates nor takes a lock (the contract in hal/adc.h).
//   - It is host-testable: a test drives the fake ADC and the fake clock and asserts the
//     latency that reaches TimingService, which is the only way any of this is verified
//     before it is on a bench.
//
// The ISR-side fields are atomic because the producer is an interrupt and the consumer is a
// task; on this core they are plain 32-bit loads and stores.
class AdcIsrStamp {
 public:
  AdcIsrStamp() {
    iface_.ctx = this;
    iface_.start = &AdcIsrStamp::start_cb;
    iface_.stop = &AdcIsrStamp::stop_cb;
    iface_.blocks = &AdcIsrStamp::blocks_cb;
  }

  AdcIsrStamp(const AdcIsrStamp &) = delete;
  AdcIsrStamp &operator=(const AdcIsrStamp &) = delete;

  // `inner` is the real ADC interface and must outlive this object. `clock` and `gpio` may
  // be null, in which case the stamp is not taken and the ISR marker is not driven; the
  // block is still forwarded, because the acquisition path must not depend on the
  // instrumentation.
  void bind(hal_adc_iface_t *inner, const hal_clock_iface_t *clock, hal_gpio_iface_t *gpio) {
    inner_ = inner;
    clock_ = clock;
    gpio_ = gpio;
  }

  // What AcquisitionService is given.
  hal_adc_iface_t *iface() { return &iface_; }

  // Blocks stamped since start(). Compared against a remembered value by the task, so it
  // only has to be monotonic.
  uint32_t stamped_blocks() const { return stamped_blocks_.load(std::memory_order_relaxed); }

  // The clock reading taken at the most recent block, in microseconds. Meaningful only
  // when the clock exists and stamped_blocks() has advanced since the caller last looked.
  uint32_t last_stamp_us() const { return stamp_us_.load(std::memory_order_relaxed); }

 private:
  static bool start_cb(void *ctx, uint32_t sample_rate_hz, size_t block_samples,
                       hal_adc_block_fn on_block, void *on_block_ctx) {
    auto *self = static_cast<AdcIsrStamp *>(ctx);
    if (self->inner_ == nullptr || self->inner_->start == nullptr) {
      return false;
    }

    // The service's callback is remembered and invoked by block_cb, which is what this
    // interface registers with the real ADC in its place.
    self->downstream_ = on_block;
    self->downstream_ctx_ = on_block_ctx;
    return self->inner_->start(self->inner_->ctx, sample_rate_hz, block_samples,
                               &AdcIsrStamp::block_cb, self);
  }

  static void stop_cb(void *ctx) {
    auto *self = static_cast<AdcIsrStamp *>(ctx);
    if (self->inner_ != nullptr && self->inner_->stop != nullptr) {
      self->inner_->stop(self->inner_->ctx);
    }
    self->downstream_ = nullptr;
    self->downstream_ctx_ = nullptr;
  }

  static uint32_t blocks_cb(void *ctx) {
    auto *self = static_cast<AdcIsrStamp *>(ctx);
    return self->inner_ != nullptr && self->inner_->blocks != nullptr
               ? self->inner_->blocks(self->inner_->ctx)
               : 0U;
  }

  // INTERRUPT CONTEXT. No blocking, no allocation, no lock: a clock read, a pin toggle and
  // a call into the consumer's callback, which is itself under the same rule.
  static void block_cb(void *ctx, const uint16_t *samples, size_t count, uint32_t index) {
    auto *self = static_cast<AdcIsrStamp *>(ctx);

    self->stamped_blocks_.fetch_add(1, std::memory_order_relaxed);
    if (self->clock_ != nullptr && self->clock_->now_us != nullptr) {
      self->stamp_us_.store(self->clock_->now_us(self->clock_->ctx), std::memory_order_relaxed);
    }
    if (self->gpio_ != nullptr && self->gpio_->toggle != nullptr) {
      // NFR-1.9's ISR edge. kPinTraceIsr is the same pin board.h documents for it, so the
      // analyzer capture and this series describe the same event.
      (void)self->gpio_->toggle(self->gpio_->ctx, kPinTraceIsr);
    }

    if (self->downstream_ != nullptr) {
      self->downstream_(self->downstream_ctx_, samples, count, index);
    }
  }

  hal_adc_iface_t iface_{};
  hal_adc_iface_t *inner_ = nullptr;
  const hal_clock_iface_t *clock_ = nullptr;
  hal_gpio_iface_t *gpio_ = nullptr;
  hal_adc_block_fn downstream_ = nullptr;
  void *downstream_ctx_ = nullptr;

  std::atomic<uint32_t> stamped_blocks_{0};
  std::atomic<uint32_t> stamp_us_{0};
};

// The application's own state.
//
// `plant` classifies the I2C humidity reading for the display and the telemetry line, and
// is deliberately NOT what the control loop acts on: decision 1 of the Phase 1 record makes
// soil moisture the controlled variable, and that lives in AcquisitionService and
// ControlLoop. The classification stays because it is what the panel shows and because it
// is the only use the humidity reading has in this phase.
enum class PlantState {
  Unknown, // no valid reading has been taken yet
  Dry,
  Ok,
  Wet,
};

const char *to_string(PlantState state);

struct AppState {
  PlantState plant = PlantState::Unknown;
  SensorSnapshot last_reading{};
  uint32_t evaluations = 0;
  uint32_t rejected_readings = 0; // invalid readings that were refused rather than used
};

struct NodeStats {
  uint32_t ticks[kTaskCount]{};
  uint32_t display_renders = 0;
  uint32_t display_skips = 0; // renders avoided because nothing changed
  uint32_t tasks_created = 0;

  // Phase 1, and the numbers that make the wiring above observable on real hardware rather
  // than only in a host test. "Image-visible" in the sense that matters: they are read by
  // the comms task and reported over the link, so a bench session can see whether the
  // acquisition is running, whether the control loop is being evaluated at the rate its
  // period claims, and whether the fail-safe is holding the gate off.
  uint32_t acquisition_drains = 0;       // calls to AcquisitionService::update()
  uint32_t control_evaluations = 0;      // control periods evaluated (one per ten blocks)
  uint32_t control_skipped_periods = 0;  // block groups that passed without an evaluation
  uint32_t gate_on_edges = 0;            // commands that energised the gate
  uint32_t gate_off_edges = 0;           // commands that de-energised it
  uint32_t fail_safe_ticks = 0;          // app ticks that found the analog reading unusable
  uint32_t timing_windows = 0;           // NFR-1.8/NFR-1.9 sample windows completed
};

class Node {
 public:
  explicit Node(const NodeHardware &hardware);

  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

  // Bring up services in dependency order. Does not start the scheduler.
  bool init();

  // Create the five tasks (FR-0.1). Call after init() and before the scheduler starts.
  bool start();

  // One iteration of one task's body.
  //
  // The target's FreeRTOS task functions loop over this with their own period; a host
  // test calls it directly. Having a single implementation for both is what makes the
  // task architecture itself testable without a board.
  void tick(TaskId id);

  I2cArbiter &arbiter() { return arbiter_; }
  Supervisor &supervisor() { return supervisor_; }
  SensorService &sensor() { return sensor_; }
  CommsService &comms() { return comms_; }
  I2cRecovery &recovery() { return recovery_; }

  // Phase 1's three services, exposed for the same reason as the Phase 0 ones: a host test
  // asserts on the mechanism (blocks processed, the loop's own statistics, the timing
  // series) rather than inferring it from the output.
  AcquisitionService &acquisition() { return acquisition_; }
  ControlLoop &control() { return control_; }
  ControlTuningStore &tuning() { return tuning_; }
  TimingService &timing() { return timing_; }

  const AppState &app_state() const { return app_; }
  const NodeStats &stats() const { return stats_; }
  Canvas &canvas() { return canvas_; }

  // Null when no display was supplied. `display_ready()` distinguishes "no panel" from
  // "panel present but init failed".
  Ssd1306 *display() { return display_ready_ ? &display_ : nullptr; }

  // FreeRTOS task entry. Wraps tick() in the task's own period loop.
  static void task_entry(void *arg);

 private:
  void tick_supervisor();
  void tick_sensor();
  void tick_display();
  void tick_app();
  void tick_comms();

  // Phase 1's app-task body, split so that each part is one idea:
  //
  //   drain_acquisition() - the block ring's consumer, at kAcquisitionDrainPeriodMs
  //   recompute_control() - one control period, when the block count crosses the boundary
  //   apply_gate()        - the gate, every tick, however the loop decided
  void drain_acquisition();
  void recompute_control(uint32_t now_ms, uint32_t now_us, const AnalogSnapshot &snapshot);
  void apply_gate(uint32_t now_ms, const AnalogSnapshot &snapshot);

  // The microsecond clock reading, or 0 when no clock was supplied. A single place that
  // answers "is there a clock?", so no caller has to ask twice.
  uint32_t now_us() const;

  // Report and re-arm the NFR-1.8/NFR-1.9 windows when one fills. See the note in node.cpp:
  // the series is bounded so its arithmetic stays exact, and nothing pulls it off the board
  // yet, so a full window is counted and reset rather than silently ignored.
  void note_timing_window();

  // Draw the status screen if anything it shows has changed. Returns true if a flush
  // was attempted.
  bool render_status();

  // Everything the panel shows, so that "has anything changed" is one comparison
  // rather than a set of ad-hoc flags.
  struct RenderedState {
    bool fresh = false;
    bool ever_valid = false;
    int32_t temperature_milli_c = 0;
    int32_t humidity_milli_rh = 0;
    uint32_t consecutive_failures = 0;
    PlantState plant = PlantState::Unknown;
    bool supervisor_tripped = false;
    uint32_t boot_count = 0;

    bool operator==(const RenderedState &other) const;
    bool operator!=(const RenderedState &other) const { return !(*this == other); }
  };

  RenderedState current_rendered_state() const;

  // Per-task context for the FreeRTOS trampoline. Stored inline rather than allocated:
  // the 72 h run (NFR-0.7) must not depend on the heap surviving that long.
  struct TaskContext {
    Node *node = nullptr;
    TaskId id = TaskId::Count;
  };

  NodeHardware hw_;
  I2cRecovery recovery_;
  I2cArbiter arbiter_;
  Supervisor supervisor_;
  SensorService sensor_;

  // Phase 1's chain, in dependency order, and each declaration is where it is for a reason:
  //
  //   adc_stamp_   wraps the real ADC interface, so it must exist before the acquisition
  //                service is handed its `iface()`.
  //   acquisition_ borrows adc_stamp_.iface() and the RTOS (for its publish mutex).
  //   tuning_      is the published setpoint/gains; the control loop reads it and the
  //                comms task writes it (FR-1.4).
  //   control_     borrows tuning_ and must be declared after it.
  //   timing_      is fed by the app task alone (NFR-1.8, NFR-1.9).
  AdcIsrStamp adc_stamp_{};
  AcquisitionService acquisition_;
  ControlTuningStore tuning_{};
  ControlLoop control_;
  TimingService timing_{};

  // The panel and its canvas are declared before the comms service because the comms
  // service reports the display's byte counters, so it holds a pointer into them.
  Canvas canvas_{};
  Ssd1306 display_{&arbiter_};
  bool display_ready_ = false;
  NodeIdentity identity_{};
  CommsService comms_;

  AppState app_{};
  RenderedState rendered_{};
  NodeStats stats_{};
  TaskContext contexts_[kTaskCount]{};
  bool status_led_on_ = false;

  // Phase 1's app-task state.
  //
  // `app_ticks_` counts this task's own entries and is what the acquisition drain is gated
  // on; `control_block_group_` is the acquisition block number divided by
  // kAdcBlocksPerControlPeriod, so a change means ten hardware-timed blocks have completed
  // and a new control period has begun.
  uint32_t app_ticks_ = 0;
  uint32_t control_block_group_ = 0;
  bool have_control_group_ = false;
  uint32_t last_control_us_ = 0;
  bool have_last_control_us_ = false;
  uint32_t last_stamped_blocks_ = 0;

  // The gate's last commanded state, so the counters count edges rather than ticks and the
  // actuator is only written when the decision changes. Tracked here rather than read back
  // from the pin, because hal/actuator.h deliberately has no readback: a pin's electrical
  // state is not the question, and asking it would invite treating a pin read as proof that
  // the pump is running.
  bool gate_on_ = false;

  // Set when a tick finds the analog reading unusable, and cleared at the next control
  // evaluation. It is a latch rather than a per-tick comparison because decision 8 means
  // "a lost probe stops the pump": resuming the remainder of a window whose command came
  // from a reading that has since been replaced would deliver water on a decision the loop
  // no longer stands behind.
  bool fail_safe_latched_ = true;
};

} // namespace node

#endif // APP_NODE_H
