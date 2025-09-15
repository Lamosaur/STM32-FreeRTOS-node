#include "app/node.h"

namespace node {

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
      acquisition_(adc_stamp_.iface(), hw_.rtos),
      control_(&tuning_),
      display_{&arbiter_},
      comms_(hw_.uart, &arbiter_, hw_.rtos,
             CommsSources{&sensor_, &supervisor_, &arbiter_, hw_.system,
                          &display_.stats(), &identity_, &tuning_, &acquisition_, &control_}) {
  // The ISR-side stamping wrapper is bound here, in the constructor, rather than in init():
  // the acquisition service was handed `adc_stamp_.iface()` in the member-initialiser list
  // above, so the forwardee has to be in place before anything can call start(). A null
  // `hw_.adc` is handled by the wrapper's own null check and by AcquisitionService's.
  adc_stamp_.bind(hw_.adc, hw_.clock, hw_.gpio);
}

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

  // The actuator comes up before anything that could ask it to run, and its init() leaves
  // the gate off. Order relative to the arbiter: the gate needs no bus and no lock, so it
  // is the one output that can be put into its fail-safe state before any service that
  // might fail is even started — which is the order in which a fault after this point
  // leaves the pump stopped.
  if (hw_.actuator != nullptr && hw_.actuator->init != nullptr) {
    if (!hw_.actuator->init(hw_.actuator->ctx)) {
      // A gate that could not be brought up safely is treated as absent rather than as
      // stopped-and-forgotten: hal/actuator.h is explicit that a failed init() is a reason
      // to withhold control entirely, because a gate driven through an unconfigured pin
      // may sit high. Dropping the handle means the control loop still evaluates and its
      // decisions are still visible, but nothing energises a pin this code does not own.
      hw_.actuator = nullptr;
    }
  }

  // The panel comes up before anything that might fail, so that a later failure can be
  // shown on the node rather than only on a host that may not be attached.
  display_ready_ = display_.init();

  // A missing or unresponsive sensor is reported over comms and shows as an invalid
  // reading; it is not a reason to refuse to boot, because a node that cannot display
  // its own fault is less useful than one that boots and complains.
  sensor_.init();

  // The analog channel (FR-1.1). Started here, in the task context that is about to be
  // superseded by the five tasks, because hal/adc.h offers no way to ask whether it is
  // running and no way to start it late without risking the control loop acting on a
  // period it never measured. A refusal is recorded by the service (start_failures) and
  // leaves the fix in place: the fail-safe holds the gate off until readings arrive, so an
  // absent ADC stops the pump instead of letting the loop decide on nothing.
  acquisition_.start();

  if (hw_.watchdog != nullptr) {
    hw_.watchdog->start(hw_.watchdog->ctx, kWatchdogTimeoutMs);
  }

  // Every task except the supervisor is watched. The supervisor is the one doing the
  // watching, and it is covered by the watchdog itself: if it stops feeding, the node
  // resets.
  // Periods come from the task table rather than from loose constants, so a task's
  // heartbeat deadline cannot drift out of step with the loop that actually runs it.
  supervisor_.watch(TaskId::Sensor, task_period_ms(TaskId::Sensor));
  supervisor_.watch(TaskId::Display, task_period_ms(TaskId::Display));
  supervisor_.watch(TaskId::App, task_period_ms(TaskId::App));
  supervisor_.watch(TaskId::Comms, task_period_ms(TaskId::Comms));

  comms_.send_banner();
  return true;
}

bool Node::start() {
  if (hw_.rtos == nullptr || hw_.rtos->task_create == nullptr) {
    return false;
  }

  // One table, from board.h: a task's priority, stack and period are reviewed together,
  // and its FreeRTOS name is part of the same specification.
  for (const TaskSpec &spec : kTaskSpecs) {
    const size_t index = static_cast<size_t>(spec.id);
    contexts_[index].node = this;
    contexts_[index].id = spec.id;

    rtos_task_t *task = hw_.rtos->task_create(
        hw_.rtos->ctx, spec.name, &Node::task_entry, &contexts_[index],
        spec.stack_words, spec.priority);
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
  // The Phase 0 classification of the I2C humidity reading stays exactly as it was: it is
  // what the panel shows and what the telemetry line reports, and it is deliberately not
  // what the control loop acts on (decision 1 of the Phase 1 record makes soil moisture the
  // controlled variable).
  app_.evaluations++;

  const SensorSnapshot ambient = sensor_.snapshot();
  app_.last_reading = ambient;

  // fresh(), not reading.valid: valid only means a reading once arrived, and the service
  // deliberately keeps the last good value after a failure, so checking valid() here
  // would have the app classifying a frozen number indefinitely.
  if (!ambient.fresh()) {
    // Refuse to act on a reading that is absent or no longer being refreshed. Keeping
    // the previous state is deliberately the conservative choice: a silent sensor is not
    // evidence that conditions changed.
    app_.rejected_readings++;
  } else if (ambient.reading.humidity_milli_rh < kPlantDryMilliRh) {
    app_.plant = PlantState::Dry;
  } else if (ambient.reading.humidity_milli_rh > kPlantWetMilliRh) {
    app_.plant = PlantState::Wet;
  } else {
    app_.plant = PlantState::Ok;
  }

  // Phase 1, in three parts, and the order is the design:
  //
  //   1. drain the acquisition ring if this tick is a drain tick, so the snapshot read
  //      below is the freshest one available rather than one up to kAcquisitionDrainPeriodMs
  //      old;
  //   2. evaluate the control loop if the acquisition's own block numbering has crossed a
  //      control-period boundary — never on a task delay (design-spec.md §6);
  //   3. drive the gate, every tick, because the gate's edge resolution is this task's
  //      period and the window's on-time may expire between two control evaluations.
  const uint32_t now_ms = hw_.rtos->now_ms(hw_.rtos->ctx);
  const uint32_t us = now_us();

  drain_acquisition();

  // One snapshot per tick, taken once and used by both the NFR-1.9 measurement and the
  // control decision. Taking it twice would allow the two to describe different readings,
  // which is precisely the incoherence the tuning store's snapshot scheme avoids.
  const AnalogSnapshot snapshot = acquisition_.snapshot();

  // NFR-1.9's task-side edge: the ISR judged a block, and this is the first task entry that
  // can see it. The latency recorded is therefore "interrupt to the task that observes it",
  // which is what the logic analyzer capture of kPinTraceIsr -> kPinTraceTask shows. It is
  // not "interrupt to the consumer": the block is consumed at the next drain tick, up to
  // kAcquisitionDrainPeriodMs later, and that delay is the polling cadence's, not the
  // RTOS's. Both numbers are worth having and they are deliberately not added together —
  // the second is a design property, the first is what NFR-1.9 asks for.
  const uint32_t stamped = adc_stamp_.stamped_blocks();
  if (stamped != last_stamped_blocks_) {
    last_stamped_blocks_ = stamped;
    if (hw_.gpio != nullptr) {
      hw_.gpio->toggle(hw_.gpio->ctx, kPinTraceTask);
    }
    if (hw_.clock != nullptr) {
      // Unsigned subtraction, so a wrap of the microsecond counter between the stamp and
      // this tick reads as a huge interval rather than a small one. The counter wraps every
      // 71.6 minutes and the interval here is a task period, so a wrap in between is
      // possible at most once in 71 minutes and is counted in the series' oversized
      // samples rather than folded into the mean as if it were a real latency.
      timing_.note_latency_us(us - adc_stamp_.last_stamp_us());
      note_timing_window();
    }
  }

  // The control period, counted in blocks rather than in ticks. `block_index` is the
  // producer's own numbering of the block the published value came from, and those blocks
  // are the ones TIM3's TRGO produced, so the group index changes once every
  // kAdcBlocksPerControlPeriod blocks of *hardware* time. A tick that is late, early, or
  // skipped therefore does not move the period: it only decides when inside the second the
  // crossing is noticed.
  if (snapshot.valid) {
    const uint32_t group = snapshot.block_index / kAdcBlocksPerControlPeriod;
    if (!have_control_group_) {
      // The first value to arrive starts the loop. Not "group 0 evaluates immediately":
      // block indices below ten would then evaluate on the first tick after start-up with a
      // partial period behind them, and the integral would integrate over it.
      have_control_group_ = true;
      control_block_group_ = group;
      recompute_control(now_ms, us, snapshot);
    } else if (group != control_block_group_) {
      // A jump of more than one group means the control task missed whole periods (a stall,
      // or an acquisition overrun). Each skipped group is counted rather than evaluated:
      // running the PID once per missed group would integrate an interval the loop never
      // saw the plant over, and the sample-time-aware dt already accounts for the elapsed
      // time exactly once.
      stats_.control_skipped_periods += (group - control_block_group_) - 1U;
      control_block_group_ = group;
      recompute_control(now_ms, us, snapshot);
    }
  }

  apply_gate(now_ms, snapshot);
}

void Node::drain_acquisition() {
  app_ticks_++;

  // kAcquisitionDrainEveryAppTicks, and not "every tick": see kAcquisitionDrainPeriodMs in
  // board.h for both bounds. In one line — draining faster than the producer makes
  // AcquisitionService record empty updates as stale readings.
  if ((app_ticks_ % kAcquisitionDrainEveryAppTicks) != 0U) {
    return;
  }

  stats_.acquisition_drains++;
  (void)acquisition_.update();
}

void Node::recompute_control(uint32_t now_ms, uint32_t now_us_value,
                             const AnalogSnapshot &snapshot) {
  // The NFR-1.8 marker, toggled at the control evaluation rather than at task entry: the
  // app task wakes 40 times a second and the control period is one second, so a marker at
  // entry would measure the scheduler's period and say nothing about the loop. This is the
  // edge the analyzer capture pairs with the previous one to give the period, and it is the
  // same instant the series below is sampled at.
  if (hw_.gpio != nullptr) {
    hw_.gpio->toggle(hw_.gpio->ctx, kPinTraceControl);
  }

  // The period, measured with the microsecond clock rather than assumed to be the nominal
  // 1 s. What it contains: the hardware block period (which is exact) plus however late the
  // scheduler ran this task relative to the block boundary it crossed (which is not, and is
  // bounded by the app task's period). The deviation histogram therefore shows the
  // quantisation first and any scheduling trouble second, and the mean shows whether the
  // count of blocks and the wall clock agree over the window — which is the check that the
  // 1 kHz trigger is really 1 kHz.
  if (hw_.clock != nullptr && have_last_control_us_) {
    timing_.note_period_us(now_us_value - last_control_us_);
    note_timing_window();
  }
  last_control_us_ = now_us_value;
  have_last_control_us_ = true;

  stats_.control_evaluations++;
  (void)control_.update(now_ms, snapshot);

  // The fail-safe latch is re-decided here and only here: this is the one place the loop
  // looks at the reading, so it is the one place where "the pump may run again" can be
  // established. A tick that finds the reading unusable sets the latch; this clears it only
  // when the loop has just decided a new window on a fresh reading.
  fail_safe_latched_ = !snapshot.fresh();
}

void Node::apply_gate(uint32_t now_ms, const AnalogSnapshot &snapshot) {
  if (hw_.actuator == nullptr) {
    return;
  }

  // The fail-safe (decision 8), on every tick rather than once per control period: a probe
  // that goes silent mid-window must stop the pump at the next 25 ms tick, not up to a
  // second later. `fresh()` and not `valid`, for the same reason the display checks it:
  // AcquisitionService keeps the last good value after a failure, so `valid` stays true
  // through a dead DMA channel and a loop that trusted it would water a pot on the strength
  // of a number that stopped changing minutes ago.
  if (!snapshot.fresh()) {
    fail_safe_latched_ = true;
    stats_.fail_safe_ticks++;
  }

  // The window inside the loop's last decision, gated by the latch above.
  const bool want_on = !fail_safe_latched_ && control_.window().on(now_ms);

  if (want_on == gate_on_) {
    return;
  }

  if (want_on) {
    if (hw_.actuator->on != nullptr) {
      hw_.actuator->on(hw_.actuator->ctx);
      stats_.gate_on_edges++;
    }
  } else {
    // Always safe to call, at any time, repeatedly: hal/actuator.h's contract, and the
    // reason this is the branch taken for every failure above.
    if (hw_.actuator->off != nullptr) {
      hw_.actuator->off(hw_.actuator->ctx);
      stats_.gate_off_edges++;
    }
  }
  gate_on_ = want_on && hw_.actuator->on != nullptr;
}

uint32_t Node::now_us() const {
  if (hw_.clock == nullptr || hw_.clock->now_us == nullptr) {
    return 0U;
  }
  return hw_.clock->now_us(hw_.clock->ctx);
}

void Node::note_timing_window() {
  // The timing service ignores samples once its window is full and says so, so a caller
  // that never reports never notices it has lost the data. Nothing in Phase 1 pulls the
  // series off the board yet (see the report on the interface gap), so what this can do is
  // count the completed windows and reset, keeping the measurement running — a node that
  // silently stopped recording after 17 minutes would make NFR-1.8's "measured, not
  // assumed" true only for the first 17 minutes of the run.
  if (timing_.full()) {
    stats_.timing_windows++;
    timing_.reset();
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

    // The initialiser is the Unknown case, so that branch needs no assignment of its
    // own; repeating the string there is a dead store.
    const char *plant = "RH --";
    switch (next.plant) {
      case PlantState::Dry: plant = "RH LO"; break;
      case PlantState::Ok:  plant = "RH OK"; break;
      case PlantState::Wet: plant = "RH HI"; break;
      case PlantState::Unknown: break;
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
