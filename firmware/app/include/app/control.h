/* control.h — the Phase 1 closed loop: soil moisture in, a time-proportioned pump
 * window out.
 *
 * What this file is. Four pieces, each testable without the others:
 *
 *   - ControlTuning and ControlTuningStore — the setpoint, gains and limits, published by
 *     the comms task and read by the control task (FR-1.4) without either of them taking a
 *     lock (decision 9).
 *   - PidController — the fixed-point arithmetic of the control law, sample-time aware,
 *     with the integral clamped and the output clamped to [0, max_duty].
 *   - WindowOutput — the time-proportioning state machine. It takes `now_ms` from the
 *     caller rather than reading a clock, so a host test can walk a whole window with no
 *     sleep, no clock double, and no RTOS.
 *   - ControlLoop — the three above wired together, plus the fail-safe (decision 8) and
 *     the minimum on-time rule (decision 5).
 *
 * What this file is not, and the boundary is deliberate. It contains no FreeRTOS task, no
 * clock, no actuator, and no reference to node.cpp: design-spec.md §6 puts the control
 * period on a hardware timer and the gate decision next to the gate, and wiring the loop
 * into the app task and the real `hal_actuator_iface_t` is a separate change. The loop
 * therefore *reports* its decision (duty, on-time, whether this window is on) and the
 * caller applies it. That split is also what makes every claim below testable on the host.
 *
 * Why there is no mutex here, restated because it is the one decision a reader is most
 * likely to "fix". The loop reads the tuning through a two-slot store and a single atomic
 * index. A mutex would be worse than useless: it would put a lock acquisition inside the
 * very loop whose jitter NFR-1.8 measures, and it is itself an instance of the
 * priority-inversion case FR-1.7 exists to measure — the control task (priority 3) waiting
 * on a lock the display task (priority 1) holds while it clocks out an OLED frame. A
 * tuning value is not worth either cost. See ControlTuningStore for what replaces it.
 *
 * Nothing here allocates, blocks, yields or locks, and none of it can: the loop's only
 * inputs are a millisecond timestamp and a snapshot that the caller has already taken.
 * That snapshot is produced under AcquisitionService's mutex, but by the caller and before
 * the loop is entered — the loop names no RTOS primitive and has no way to acquire one.
 * The consequence is that a control evaluation cannot be delayed by the comms task, the
 * display task or an I2C transfer, which is what decision 9 asks for.
 */
#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <atomic>
#include <cstdint>

#include "app/acquisition.h"
#include "app/board.h"

namespace node {

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Everything a host can change at run time (FR-1.4). One struct rather than separate
// setpoint/gain/limit setters, because the two rules that matter are about the *set*: the
// fields are validated together (a duty cap that cannot deliver a minimum on-time is a
// pump that never runs), and they are published together, so a reader can never see a new
// setpoint with the old gains and act on the mixture.
//
// The limits are here rather than being compile-time constants because they are part of
// the safety envelope an operator may want to tighten — turning a 50 % cap into 20 % for a
// pot that has already been over-watered is a runtime action, not a reflash.
struct ControlTuning {
  int32_t setpoint_permille = kControlSetpointPermille; // 0..1000, the calibration span
  int32_t kp_milli = kControlDefaultKpMilli;
  int32_t ki_milli = kControlDefaultKiMilli;
  int32_t kd_milli = kControlDefaultKdMilli;
  uint32_t min_on_time_ms = kControlMinOnTimeMs; // 0..kControlPeriodMs
  int32_t max_duty_permille = kControlMaxDutyPermille; // 1..1000, and >= min_on_time
};

// Which field a refused tuning is about, so the host link can say which number it rejected
// instead of only that something was wrong. `None` means the set is acceptable.
enum class TuningField : uint8_t {
  None = 0,
  Setpoint,
  Kp,
  Ki,
  Kd,
  MinOnTime,
  MaxDuty,
};

// `setpoint`, `kp`, ... — the short names a host types, and the names the error line uses.
const char *to_string(TuningField field);

// The first field that is out of range, or None. Validation *refuses* rather than clamps
// silently: a clamped value is a loop that behaves differently from the one the person on
// the other end of the link believes they configured, with nothing on the wire to say so.
TuningField tuning_error(const ControlTuning &tuning);

// Convenience for callers that only need the yes/no.
inline bool tuning_valid(const ControlTuning &tuning) {
  return tuning_error(tuning) == TuningField::None;
}

// The setpoint, gains and limits, published by one task and read by another without a
// lock (decision 9).
//
// The mechanism, in full, because "lock-free" is a claim that has to be discharged rather
// than asserted. Two slots are kept. The writer never touches the slot the reader is
// using: it fills the inactive one and then stores its index into an atomic. The reader
// loads that index once and copies the slot it names.
//
// Why that is enough, on this target:
//
//   - A reader can never observe a half-written tuning, because the writer only ever
//     writes a slot the reader is not reading. The index store is the publication point,
//     with release ordering, so the reader that observes the new index observes the slot
//     contents that go with it.
//   - A reader *can* read a stale tuning — it copies the old slot if the index changed
//     after it loaded it. That is harmless for a tuning value: every published set is a
//     complete, valid set, and one control period (1000 ms) spent on the previous one is
//     not a fault. This is the property that makes a lock unnecessary rather than merely
//     inconvenient: the data has no invariant that spans a read.
//   - On a single-core Cortex-M3 with 32-bit aligned access, a torn *word* is impossible
//     in the first place (no store tearing, no unaligned splitting), and std::atomic
//     makes the intent explicit and portable rather than relying on that.
//
// The writer is single — the comms task. Two writers would need arbitration, and that is
// the point at which this design would need rethinking rather than extending: publication
// is a single producer's job. This is documented rather than enforced, because the
// alternative (a lock in the writer) would put the lock back in the reader's path.
class ControlTuningStore {
 public:
  ControlTuningStore();

  ControlTuningStore(const ControlTuningStore &) = delete;
  ControlTuningStore &operator=(const ControlTuningStore &) = delete;

  // The reader's side: load the index once, then copy the slot it names. Safe to call from
  // the control task at any time, and deliberately cheap (one atomic load, one 24-byte
  // copy) because it is on the control path.
  ControlTuning load() const;

  // The writer's side. Refuses an invalid tuning and leaves the published one in force,
  // returning false. Otherwise fills the inactive slot and swaps the index.
  bool publish(const ControlTuning &next);

 private:
  ControlTuning slots_[2]{};
  std::atomic<uint32_t> active_{0};
};

// ---------------------------------------------------------------------------
// The control law
// ---------------------------------------------------------------------------

// The result of one evaluation, with the individual terms exposed. A test can assert them
// separately — a sum that happens to be right for a wrong reason is the failure mode this
// struct exists to catch — and a comms or step-capture path can report the P, I and D
// shares of the output rather than only the total.
struct PidTerms {
  int32_t error_permille = 0;        // setpoint - measured
  int32_t p_duty_permille = 0;
  int32_t i_duty_permille = 0;
  int32_t d_duty_permille = 0;
  int32_t integral_permille_seconds = 0; // the state after this update, clamped
  int32_t requested_duty_permille = 0;   // P + I + D, before the output clamp
  int32_t duty_permille = 0;             // after the clamp to [0, max_duty]
  bool integral_clamped = false;  // the state hit its limit on this update
  bool i_clamped = false;         // the I term hit +-max_duty (see the note in control.cpp)
  bool output_clamped = false;    // the clamp to [0, max_duty] changed the value, at either end
};

// The PID arithmetic of the decision record, in integer arithmetic. There is no float
// anywhere in this firmware: the F103 has no FPU, so a float would mean soft-float calls
// for a number that is printed with three digits, and the loop's job is to be the one
// piece of this system whose timing is a measured property (NFR-1.8).
//
// Sample-time aware, because the loop's period is set by a hardware timer and a late
// evaluation is the interesting case, not an impossible one: the integral advances by
// error x dt_ms, and the derivative divides by the dt it actually saw. A period that slips
// to 2 s therefore contributes twice the integral of a 1 s one, which is the correct
// answer for a plant that saw 2 s of the conditioned output.
class PidController {
 public:
  PidController() = default;

  // Takes the tuning as given. Validation belongs to ControlTuningStore and to the host
  // link that reports the refusal: a controller that silently ignored a gain it disliked
  // would compute for a loop nobody asked for, which is worse than computing with the
  // number it was handed. This call does re-derive the integral's state limit from the
  // live tuning, so raising the duty cap at run time raises the integrator's authority with
  // it, and lowering ki re-clamps the state at the next update rather than leaving a stale,
  // larger one in place.
  void set_tuning(const ControlTuning &tuning) { tuning_ = tuning; }
  const ControlTuning &tuning() const { return tuning_; }

  // One evaluation. `dt_ms` is the interval since the previous evaluation of this
  // controller, not a nominal period.
  PidTerms update(int32_t measured_permille, uint32_t dt_ms);

  // The terms from the last update. A caller that needs the integral after a fail-safe
  // window reads it here, because a fail-safe window does not evaluate the law at all.
  const PidTerms &terms() const { return terms_; }

 private:
  ControlTuning tuning_{};
  PidTerms terms_{};

  // The integral state, in permille-seconds, and the previous error for the derivative.
  // Kept as members rather than inside PidTerms because they are the controller's memory
  // rather than the last update's output. int32 is safe because the state is clamped to at
  // most kControlIntegralStateGuardPermilleSeconds on every update, and every product
  // involving it is taken in 64 bits.
  int32_t integral_permille_seconds_ = 0;
  int32_t previous_error_permille_ = 0;
  bool have_previous_error_ = false;

  // Note what is deliberately absent: a reset(). The integral is the only state that
  // removes the steady-state offset, so clearing it mid-run is how a working loop starts
  // carrying a permanent error; and the decided fail-safe *holds* it (decision 8) rather
  // than unwinding it, precisely because the alternative re-earns the offset from scratch
  // every time the sensor hiccups. A caller that wants a clean start constructs a new
  // controller.
};

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

// Time-proportioning (decision 4): inside one control window the output is on for
// `on_time_ms` and off for the rest. Not PWM — see hal/actuator.h for why, and for the
// pull-down that makes "off" the state a dead MCU fails into.
//
// The state machine takes the current time from its caller rather than reading a clock.
// That is not only a testability convenience (though it is that: a whole window is walked
// in a test with no sleep, no clock double and no RTOS): the loop's notion of time has to
// be the same one the period measurement (NFR-1.8) uses, and the only component that knows
// which clock that is, is the caller.
class WindowOutput {
 public:
  // `window_ms` is the length of one control window. It is a constructor parameter rather
  // than a constant read from board.h so that a test can exercise the boundaries at a
  // different scale; the loop passes the decided control period.
  explicit WindowOutput(uint32_t window_ms = kControlPeriodMs);

  // Start a new window at `now_ms` and command `on_time_ms` of it. Supersedes any window
  // already in progress, which is what the loop wants: each control evaluation defines the
  // window that starts with it.
  //
  // An on-time longer than the window is clamped to the window, because "on for longer
  // than the whole window" is not something the time-proportioning scheme can express and
  // silently turning the pump on permanently (which is what truncating elsewhere would do)
  // is the wrong way to find that out.
  void begin_window(uint32_t now_ms, uint32_t on_time_ms);

  // Should the gate be energised at `now_ms`? True only inside the commanded on-time.
  //
  // Unsigned subtraction, so a `now_ms` that is behind the window start — a wrapped
  // millisecond clock, or a caller that passed a stale timestamp — reads as a huge elapsed
  // time and answers false. False is the safe answer, which is why the wrap case needs no
  // special branch.
  bool on(uint32_t now_ms) const;

  // End the window's on-time immediately: the fail-safe. After this, on() is false for the
  // rest of the window, so a reading that goes stale mid-pulse stops the pump at the next
  // evaluation rather than leaving it on until the commanded on-time expires.
  void abort();

  bool active() const { return active_; }
  uint32_t window_ms() const { return window_ms_; }
  uint32_t on_time_ms() const { return on_time_ms_; }
  uint32_t started_at_ms() const { return started_at_ms_; }

 private:
  uint32_t window_ms_;
  uint32_t started_at_ms_ = 0;
  uint32_t on_time_ms_ = 0;
  bool active_ = false;
};

// ---------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------

// One evaluation's decision, reported rather than applied (the caller owns the gate).
struct ControlDecision {
  uint32_t now_ms = 0;
  uint32_t dt_ms = 0;
  int32_t measured_permille = 0;
  int32_t setpoint_permille = 0;

  // What the law asked for, and what the window will actually deliver. They differ in
  // exactly one case: a nonzero duty whose on-time fell below the minimum, which becomes
  // zero (decision 5). Keeping both means "the controller wanted to act and the plumbing
  // would not let it" is visible rather than silently equal to "the controller wanted
  // nothing", which is the state a pot that is not being watered looks like.
  //
  // `requested_duty_permille` is the law's output after its own clamp to [0, max_duty] —
  // the value PidTerms::duty_permille carries — not the unclamped P + I + D sum, which is
  // PidTerms::requested_duty_permille. Two "requested" fields is one too many; they are
  // named for what they are, and this is the one the window was asked for.
  int32_t requested_duty_permille = 0;
  int32_t duty_permille = 0;
  uint32_t on_time_ms = 0;

  bool fail_safe = false; // no usable reading this cycle: the output is forced to 0
  bool saturated = false; // the law asked for more than the cap: the loop is not in control
  bool skipped = false;   // a nonzero on-time was below the minimum and became 0

  PidTerms terms{};
};

struct ControlStats {
  uint32_t evaluations = 0;
  uint32_t saturated_windows = 0; // the cap was reached, so the loop was not in control
  uint32_t fail_safe_windows = 0; // no fresh reading: the output was forced to 0
  uint32_t min_on_time_skips = 0; // nonzero duty, on-time below the minimum, window skipped
  uint32_t zero_duty_windows = 0; // nothing was delivered (includes every skip)
};

// The whole loop, one evaluation per control period.
class ControlLoop {
 public:
  // `tuning` is borrowed and may be null, in which case the compiled-in defaults are used
  // — a node whose comms task has no tuning source still controls, and the store is how a
  // host changes that, not how the loop finds out what the setpoint is.
  //
  // `control_period_ms` is both the period the caller is expected to call at and the length
  // of the output window; the decided value is 1000 ms and the parameter exists so that a
  // test can drive a different scale.
  explicit ControlLoop(const ControlTuningStore *tuning = nullptr,
                       uint32_t control_period_ms = kControlPeriodMs);

  ControlLoop(const ControlLoop &) = delete;
  ControlLoop &operator=(const ControlLoop &) = delete;

  // One control period. `now_ms` is the caller's clock and `snapshot` a reading the caller
  // has already taken — see the header comment for why both are arguments rather than
  // things this class fetches.
  //
  // In order, per decision 8, 6, 5 and 4: read the tuning; take the sample interval; if
  // the reading is absent or stale, abort the window and report zero output without
  // evaluating the law (the integral is *held*, not unwound); otherwise evaluate the PID,
  // convert the duty to an on-time, apply the minimum on-time rule, and open the window.
  ControlDecision update(uint32_t now_ms, const AnalogSnapshot &snapshot);

  const ControlDecision &last() const { return last_; }
  const ControlStats &stats() const { return stats_; }

  // Exposed for diagnostics and tests. Between them these are the loop's entire state, so
  // a test can assert on the mechanism (the integral's value, the window's boundaries)
  // rather than inferring it from the output.
  const PidController &pid() const { return pid_; }
  const WindowOutput &window() const { return window_; }
  uint32_t control_period_ms() const { return control_period_ms_; }

 private:
  const ControlTuningStore *tuning_;
  PidController pid_;
  WindowOutput window_;
  uint32_t control_period_ms_;

  // The previous evaluation's timestamp, and whether there has been one. One timestamp
  // serves both the sample interval and the decision to use the nominal period first:
  // there is no measured interval before the first evaluation, and integrating over a
  // fabricated 0 ms would silently drop the first sample's contribution.
  uint32_t last_evaluation_ms_ = 0;
  bool evaluated_once_ = false;

  ControlDecision last_{};
  ControlStats stats_{};
};

} // namespace node

#endif // APP_CONTROL_H
