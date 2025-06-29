#include "app/node.h"

namespace node {

namespace {

// Task table, in descending priority so the order here matches the reasoning in
// node.h. Names are short because FreeRTOS keeps only a pointer, but they are what a
// debugger shows, so they are worth having.
struct TaskSpec {
  TaskId id;
  const char *name;
};

constexpr TaskSpec kTaskTable[] = {
    {TaskId::Supervisor, "supervisor"},
    {TaskId::Sensor, "sensor"},
    {TaskId::App, "app"},
    {TaskId::Comms, "comms"},
    {TaskId::Display, "display"},
};

static_assert(sizeof(kTaskTable) / sizeof(kTaskTable[0]) == kTaskCount,
              "the task table must describe exactly the five tasks of FR-0.1");

} // namespace

const char *to_string(PlantState state) {
  switch (state) {
    case PlantState::Unknown: return "unknown";
    case PlantState::Dry:     return "dry";
    case PlantState::Ok:      return "ok";
    case PlantState::Wet:     return "wet";
  }
  return "unknown";
}

bool Node::RenderedState::operator==(const RenderedState &other) const {
  return fresh == other.fresh && ever_valid == other.ever_valid &&
         temperature_milli_c == other.temperature_milli_c &&
         humidity_milli_rh == other.humidity_milli_rh &&
         consecutive_failures == other.consecutive_failures && plant == other.plant &&
         supervisor_tripped == other.supervisor_tripped && boot_count == other.boot_count;
}

Node::Node(const NodeHardware &hardware)
    : hw_(hardware),
      recovery_(hw_.i2c),
      arbiter_(hw_.i2c, hw_.rtos, &recovery_),
      supervisor_(hw_.rtos, hw_.watchdog),
      sensor_(&arbiter_, hw_.rtos),
      display_{&arbiter_},
      comms_(hw_.uart, &arbiter_, hw_.rtos,
             CommsSources{&sensor_, &supervisor_, &arbiter_, hw_.system,
                          &display_.stats(), &identity_}) {}

bool Node::init() {
  if (hw_.i2c == nullptr || hw_.rtos == nullptr) {
    return false;
  }
  if (!arbiter_.valid()) {
    return false;
  }

  // Configure the pins the node drives. A board with no GPIO supplied still runs; only
  // the LED and the measurement trace pins are lost.
  if (hw_.gpio != nullptr) {
    hw_.gpio->configure_output(hw_.gpio->ctx, kPinStatusLed, HAL_GPIO_OUT_PUSH_PULL);
    hw_.gpio->configure_output(hw_.gpio->ctx, kPinTraceControl, HAL_GPIO_OUT_PUSH_PULL);
    hw_.gpio->configure_output(hw_.gpio->ctx, kPinTraceIsr, HAL_GPIO_OUT_PUSH_PULL);
    hw_.gpio->configure_output(hw_.gpio->ctx, kPinTraceTask, HAL_GPIO_OUT_PUSH_PULL);
  }

  // The panel comes up before anything that might fail, so that a later failure can be
  // shown on the node rather than only on a host that may not be attached.
  display_ready_ = display_.init();

  // A missing or unresponsive sensor is reported over comms and shows as an invalid
  // reading; it is not a reason to refuse to boot, because a node that cannot display
  // its own fault is less useful than one that boots and complains.
  sensor_.init();

  if (hw_.watchdog != nullptr) {
    hw_.watchdog->start(hw_.watchdog->ctx, kWatchdogTimeoutMs);
  }

  // Every task except the supervisor is watched. The supervisor is the one doing the
  // watching, and it is covered by the watchdog itself: if it stops feeding, the node
  // resets.
  supervisor_.watch(TaskId::Sensor, kSensorPeriodMs);
  supervisor_.watch(TaskId::Display, kDisplayPeriodMs);
  supervisor_.watch(TaskId::App, kAppPeriodMs);
  supervisor_.watch(TaskId::Comms, kCommsPeriodMs);

  comms_.send_banner();
  return true;
}

bool Node::start() {
  if (hw_.rtos == nullptr || hw_.rtos->task_create == nullptr) {
    return false;
  }

  for (const TaskSpec &spec : kTaskTable) {
    const size_t index = static_cast<size_t>(spec.id);
    contexts_[index].node = this;
    contexts_[index].id = spec.id;

    rtos_task_t *task = hw_.rtos->task_create(
        hw_.rtos->ctx, spec.name, &Node::task_entry, &contexts_[index],
        task_stack_words(spec.id), task_priority(spec.id));
    if (task == nullptr) {
      return false;
    }
    stats_.tasks_created++;
  }

  // FR-0.1 is a count, so the count is checked rather than assumed.
  return stats_.tasks_created == kTaskCount;
}

void Node::task_entry(void *arg) {
  auto *context = static_cast<TaskContext *>(arg);
  if (context == nullptr || context->node == nullptr) {
    return;
  }

  Node *node = context->node;
  const TaskId id = context->id;
  const uint32_t period = task_period_ms(id);

  for (;;) {
    node->tick(id);
    node->hw_.rtos->delay_ms(node->hw_.rtos->ctx, period);
  }
}

void Node::tick(TaskId id) {
  if (id == TaskId::Count) {
    return;
  }
  stats_.ticks[static_cast<size_t>(id)]++;

  // The heartbeat is recorded on entry, not on completion. A task proves it is alive by
  // reaching its loop; if a task's own work runs long, the next heartbeat is late and
  // the supervisor notices, which is the behaviour FR-0.4 asks for.
  //
  // The supervisor does not heartbeat itself: it is the watcher, and it is watched by
  // the watchdog through whether it manages to feed.
  if (id != TaskId::Supervisor) {
    supervisor_.heartbeat(id);
  }

  switch (id) {
    case TaskId::Supervisor: tick_supervisor(); break;
    case TaskId::Sensor:     tick_sensor(); break;
    case TaskId::Display:    tick_display(); break;
    case TaskId::App:        tick_app(); break;
    case TaskId::Comms:      tick_comms(); break;
    case TaskId::Count:      break;
  }
}

void Node::tick_supervisor() {
  supervisor_.run();

  // Toggle unconditionally: the LED reports that the scheduler is still running, which
  // is a different question from whether the watchdog was fed.
  if (hw_.gpio != nullptr) {
    status_led_on_ = !status_led_on_;
    hw_.gpio->write(hw_.gpio->ctx, kPinStatusLed, status_led_on_);
  }
}

void Node::tick_sensor() { sensor_.update(); }

void Node::tick_display() { render_status(); }

void Node::tick_app() {
  // The marker for the NFR-1.8 jitter measurement; Phase 1 toggles this around the
  // control loop instead of around this classification.
  if (hw_.gpio != nullptr) {
    hw_.gpio->toggle(hw_.gpio->ctx, kPinTraceControl);
  }

  app_.evaluations++;

  const SensorSnapshot snap = sensor_.snapshot();
  app_.last_reading = snap;

  // fresh(), not reading.valid: valid only means a reading once arrived, and the service
  // deliberately keeps the last good value after a failure, so checking valid() here
  // would have the app classifying a frozen number indefinitely.
  if (!snap.fresh()) {
    // Refuse to act on a reading that is absent or no longer being refreshed. Keeping
    // the previous state is deliberately the conservative choice: a silent sensor is not
    // evidence that conditions changed.
    app_.rejected_readings++;
    return;
  }

  if (snap.reading.humidity_milli_rh < kPlantDryMilliRh) {
    app_.plant = PlantState::Dry;
  } else if (snap.reading.humidity_milli_rh > kPlantWetMilliRh) {
    app_.plant = PlantState::Wet;
  } else {
    app_.plant = PlantState::Ok;
  }
}

void Node::tick_comms() { comms_.update(); }

Node::RenderedState Node::current_rendered_state() const {
  RenderedState state;

  const SensorSnapshot snap = sensor_.snapshot();
  state.fresh = snap.fresh();
  state.ever_valid = snap.reading.valid;
  state.temperature_milli_c = snap.reading.temperature_milli_c;
  state.humidity_milli_rh = snap.reading.humidity_milli_rh;
  state.consecutive_failures = snap.consecutive_failures;
  state.plant = app_.plant;
  state.supervisor_tripped = supervisor_.trip().tripped;
  state.boot_count = hw_.system != nullptr ? hw_.system->boot_count(hw_.system->ctx) : 0;

  return state;
}

bool Node::render_status() {
  if (!display_ready_) {
    return false;
  }

  const RenderedState next = current_rendered_state();
  if (next == rendered_) {
    // Nothing the panel shows has changed, so nothing is put on the bus. On a bench
    // with a steady reading this makes the display cost zero bus bytes, which is the
    // strongest form of the partial-refresh behaviour FR-0.5 asks for.
    //
    // Note that this comparison includes the staleness counter, so a reading that stops
    // arriving is a change and does get redrawn. Comparing only the numbers would leave
    // a frozen value on the panel looking exactly like a healthy node.
    stats_.display_skips++;
    return false;
  }

  // Only the four text rows are repainted, so the damage is 128x32 rather than the
  // whole 128x64 panel. The fill is what erases the previous, possibly longer, text;
  // drawing over it without clearing would leave stray glyph fragments behind.
  canvas_.fill_rect(Rect{0, 0, kDisplayWidth, 32}, false);

  char text[24];
  canvas_.draw_text(0, 0, kNodeShortName);

  if (next.fresh) {
    LineBuilder temperature(text, sizeof(text));
    temperature.str("T ").fixed(next.temperature_milli_c, 2).str(" C");
    canvas_.draw_text(0, 8, temperature.c_str());

    LineBuilder humidity(text, sizeof(text));
    humidity.str("RH ").fixed(next.humidity_milli_rh, 2).str(" %");
    canvas_.draw_text(0, 16, humidity.c_str());

    const char *plant = "RH --";
    switch (next.plant) {
      case PlantState::Dry: plant = "RH LO"; break;
      case PlantState::Ok:  plant = "RH OK"; break;
      case PlantState::Wet: plant = "RH HI"; break;
      case PlantState::Unknown: plant = "RH --"; break;
    }
    canvas_.draw_text(0, 24, plant);
  } else if (next.ever_valid) {
    // A reading exists but has stopped being refreshed: say so rather than show the last
    // number, because a stale number is indistinguishable from a live one.
    canvas_.draw_text(0, 8, "SENSOR");
    LineBuilder misses(text, sizeof(text));
    misses.str("STALE ").u32(next.consecutive_failures);
    canvas_.draw_text(0, 16, misses.c_str());
    canvas_.draw_text(0, 24, "-");
  } else {
    canvas_.draw_text(0, 8, "SENSOR --");
    canvas_.draw_text(0, 16, "NO DATA");
  }

  if (next.supervisor_tripped) {
    // Reported on the panel as well as over comms, because a node that is about to be
    // reset by its own watchdog may have no host attached to ask.
    canvas_.draw_text(64, 24, "ERR");
  }

  const bool flushed = display_.flush(canvas_.framebuffer(), canvas_.damage().bounds());
  canvas_.clear_damage();

  if (flushed) {
    stats_.display_renders++;
    // Committed only on success, so a flush the bus rejected is retried next cycle
    // rather than being treated as drawn.
    rendered_ = next;
  }
  return flushed;
}

} // namespace node
