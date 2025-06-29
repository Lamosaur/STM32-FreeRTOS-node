/* node.h — the five-task application (FR-0.1) and everything it owns.
 *
 * FR-0.1 requires no fewer than five concurrent tasks, verified by "task count and
 * priorities in code review". Those priorities are the design, so they are worth
 * stating here rather than being read out of a table:
 *
 *   supervisor (4)  highest, because feeding the watchdog late is the one failure that
 *                   resets the node
 *   sensor (3)      the node's primary input; a missed sample is a missed measurement
 *   app (2->3)      owns application state, and becomes the Phase 1 control loop
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

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/comms.h"
#include "app/display.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "app/sensor.h"
#include "app/supervisor.h"
#include "hal/gpio.h"
#include "hal/i2c.h"
#include "hal/system.h"
#include "hal/uart.h"
#include "hal/watchdog.h"
#include "rtos/rtos.h"

namespace node {

// Borrowed hardware handles. Any of the optional ones may be null, and the node
// degrades rather than crashing: a board with no display still runs.
struct NodeHardware {
  hal_i2c_iface_t *i2c = nullptr;
  const rtos_iface_t *rtos = nullptr;
  hal_gpio_iface_t *gpio = nullptr;         // optional: status LED and trace pins
  hal_uart_iface_t *uart = nullptr;         // optional: comms task
  hal_watchdog_iface_t *watchdog = nullptr; // optional: independence is lost without it
  hal_system_iface_t *system = nullptr;     // optional: reset cause and boot count
};

// The application's own state. Phase 0 has no actuator, so the rule below is a
// placeholder that establishes the structure — the app layer owns derived state and
// decides, target-independently — which Phase 1 replaces with the PID control loop
// driving PWM. It is not pretending to be control.
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
};

} // namespace node

#endif // APP_NODE_H
