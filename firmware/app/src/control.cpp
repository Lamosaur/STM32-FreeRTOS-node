#include "app/control.h"

namespace node {
namespace {

// Every product in the control law is taken in 64 bits and narrowed once, here. The reason
// is not that today's numbers need it — with the default gains they do not — but that the
// gains are runtime-settable up to kControlMaxGainMilli and the error ranges over a whole
// calibrated span: kp_milli * error_permille alone reaches 1e8, and the derivative term
// multiplies by 1000 before dividing. A silent 32-bit overflow in a control law is a pump
// that does something unrelated to the reading, which is the failure class the fail-safe
// exists for and is not one this code should be able to reach at all.
int32_t to_int32_saturated(int64_t value) {
  if (value > INT32_MAX) {
    return INT32_MAX;
  }
  if (value < INT32_MIN) {
    return INT32_MIN;
  }
  return static_cast<int32_t>(value);
}

// The setpoint and the duty limits are compared in permille; the on-time limits in
// milliseconds. Both are int32/uint32 in the tuning, so the comparisons below convert
// explicitly rather than relying on the usual arithmetic conversions, which would mix
// signedness (-Wsign-conversion is a build gate here, deliberately).

} // namespace

const char *to_string(TuningField field) {
  switch (field) {
    case TuningField::None:     return "none";
    case TuningField::Setpoint: return "setpoint";
    case TuningField::Kp:       return "kp";
    case TuningField::Ki:       return "ki";
    case TuningField::Kd:       return "kd";
    case TuningField::MinOnTime: return "min";
    case TuningField::MaxDuty:  return "max";
  }
  return "none";
}

TuningField tuning_error(const ControlTuning &tuning) {
  if (tuning.setpoint_permille < kControlMinSetpointPermille ||
      tuning.setpoint_permille > kControlMaxSetpointPermille) {
    return TuningField::Setpoint;
  }
  if (tuning.kp_milli < 0 || tuning.kp_milli > kControlMaxGainMilli) {
    return TuningField::Kp;
  }
  if (tuning.ki_milli < 0 || tuning.ki_milli > kControlMaxGainMilli) {
    return TuningField::Ki;
  }
  if (tuning.kd_milli < 0 || tuning.kd_milli > kControlMaxGainMilli) {
    return TuningField::Kd;
  }
  // The minimum on-time is bounded by the window it lives in: a 1500 ms minimum pulse
  // inside a 1000 ms window is not a large minimum, it is a configuration in which every
  // window is skipped.
  if (tuning.min_on_time_ms > kControlPeriodMs) {
    return TuningField::MinOnTime;
  }
  if (tuning.max_duty_permille < 1 || tuning.max_duty_permille > 1000) {
    return TuningField::MaxDuty;
  }

  // The cross-check, and why the two limits are validated as a set rather than one at a
  // time: a cap that cannot deliver a minimum on-time is a pump that never runs. Every
  // window would be skipped, the loop would report a healthy nonzero duty, and the only
  // visible symptom would be soil that keeps drying — which reads as a hardware fault. It
  // is almost always a typo (`set max=4.0` for 40 permille instead of 50), which is
  // exactly the mistake a bound is for.
  //
  // Reported as MaxDuty rather than MinOnTime because the cap is the field people type,
  // while the minimum on-time describes the pump and is set once. A refusal that named the
  // wrong field would send someone to check the pump specifications for a mistyped cap.
  const int64_t cap_on_time_ms =
      (static_cast<int64_t>(tuning.max_duty_permille) * kControlPeriodMs) / 1000;
  if (cap_on_time_ms < static_cast<int64_t>(tuning.min_on_time_ms)) {
    return TuningField::MaxDuty;
  }

  return TuningField::None;
}

ControlTuningStore::ControlTuningStore() = default;

ControlTuning ControlTuningStore::load() const {
  // One load, then the slot it named. Acquire, pairing with the release in publish(): a
  // reader that observes the new index is guaranteed to observe the slot contents that
  // were written before it.
  const uint32_t index = active_.load(std::memory_order_acquire);
  // The index is only ever 0 or 1. The mask is here so that remains a property of this
  // load rather than of every writer that will ever exist; without it, a future writer
  // that stored something else would read outside the array, and that is a fault the
  // control task cannot absorb.
  return slots_[index & 1U];
}

bool ControlTuningStore::publish(const ControlTuning &next) {
  if (!tuning_valid(next)) {
    // Refused, and the previously published tuning stays in force: a partially applied set
    // would be a loop nobody asked for. The caller reports which field.
    return false;
  }

  // Relaxed on the read of the index: only the publishing task ever writes it, so there is
  // nothing to synchronise with here. The ordering that matters is the release below.
  const uint32_t current = active_.load(std::memory_order_relaxed);
  const uint32_t inactive = current ^ 1U;

  // The writer never touches the slot the reader may be copying. That single-sided rule is
  // what makes a torn read impossible without a lock: whatever a reader is doing, the bytes
  // it is reading are not being written.
  slots_[inactive] = next;

  // The publication point. Release, so every store above is visible to a reader that
  // observes this index.
  active_.store(inactive, std::memory_order_release);
  return true;
}

PidTerms PidController::update(int32_t measured_permille, uint32_t dt_ms) {
  PidTerms terms{};

  terms.error_permille = tuning_.setpoint_permille - measured_permille;

  // P: kp_milli is duty-permille per error-permille, in thousandths, so a 150 permille
  // error with kp_milli = 2000 asks for 300 permille of duty.
  terms.p_duty_permille =
      to_int32_saturated((static_cast<int64_t>(tuning_.kp_milli) *
                          static_cast<int64_t>(terms.error_permille)) /
                         1000);

  // I, in two steps: accumulate the state, then scale it. Keeping the state in
  // permille-seconds rather than folding ki into it is what lets the clamp be a statement
  // about the integrator's authority that survives a gain change.
  //
  // The accumulation is unconditional, including when dt_ms is 0 or ki is 0. That is the
  // decision record's formula, and it is also the honest order of operations: whether the
  // integral should be frozen is a question about the *fail-safe*, and that answer is
  // "do not evaluate the law at all" (see ControlLoop), not "evaluate it with a zero dt".
  const int64_t increment = (static_cast<int64_t>(terms.error_permille) *
                             static_cast<int64_t>(dt_ms)) /
                            1000;
  int64_t state = static_cast<int64_t>(integral_permille_seconds_) + increment;

  // The clamp. See board.h for how the limit is derived and why it bounds the integral's
  // contribution rather than the state in its own unit; in one line, the limit is
  // max_duty_permille x 1000 / ki_milli.
  //
  // It is applied after the accumulation rather than by refusing the increment, so the
  // state cannot creep past the limit one window at a time. Clamping the *increment* is the
  // common alternative and it is worse here: the state would then approach the limit
  // asymptotically and never actually reach it, which makes the clamp a slope limit rather
  // than a bound — and the failure it exists to prevent (the integral holding the pump on
  // after the reading crosses the setpoint) is a failure about a bound.
  const int64_t limit = static_cast<int64_t>(
      control_integral_state_limit(tuning_.max_duty_permille, tuning_.ki_milli));
  if (state > limit) {
    state = limit;
    terms.integral_clamped = true;
  } else if (state < -limit) {
    state = -limit;
    terms.integral_clamped = true;
  }
  integral_permille_seconds_ = static_cast<int32_t>(state);
  terms.integral_permille_seconds = integral_permille_seconds_;

  terms.i_duty_permille = to_int32_saturated(
      (static_cast<int64_t>(tuning_.ki_milli) *
       static_cast<int64_t>(integral_permille_seconds_)) /
      1000);

  // Belt and braces, and it cannot fire while the state limit is derived as it is: the
  // state limit exists so that this product cannot exceed max_duty_permille, apart from the
  // truncation in deriving it. It is kept because the two are computed independently, and a
  // later change to either one must not be able to command the actuator beyond its cap by
  // way of the integral.
  if (terms.i_duty_permille > tuning_.max_duty_permille) {
    terms.i_duty_permille = tuning_.max_duty_permille;
    terms.i_clamped = true;
  } else if (terms.i_duty_permille < -tuning_.max_duty_permille) {
    terms.i_duty_permille = -tuning_.max_duty_permille;
    terms.i_clamped = true;
  }

  // D, in the decided order — multiply by 1000, divide by dt, divide by 1000 — so that the
  // truncation is the documented one rather than an accident of reassociation.
  //
  // Zero in three cases, each for its own reason: kd_milli == 0 (the default, and the
  // decision record's choice), dt_ms == 0 (there is no interval to differentiate over, and
  // dividing would be a fault), and the first evaluation (there is no previous error, so
  // there is no derivative to take — inventing a zero previous error would manufacture a
  // spike proportional to the whole error, which is the one thing a derivative term must
  // never do at start-up).
  if (tuning_.kd_milli == 0 || dt_ms == 0 || !have_previous_error_) {
    terms.d_duty_permille = 0;
  } else {
    const int64_t delta =
        static_cast<int64_t>(terms.error_permille) - static_cast<int64_t>(previous_error_permille_);
    terms.d_duty_permille = to_int32_saturated(
        static_cast<int64_t>(tuning_.kd_milli) * delta * 1000 /
        static_cast<int64_t>(dt_ms) / 1000);
  }
  previous_error_permille_ = terms.error_permille;
  have_previous_error_ = true;

  const int64_t requested = static_cast<int64_t>(terms.p_duty_permille) +
                            static_cast<int64_t>(terms.i_duty_permille) +
                            static_cast<int64_t>(terms.d_duty_permille);
  terms.requested_duty_permille = to_int32_saturated(requested);

  // The output clamp. Both ends are deliberate: the upper end is the decided duty cap, and
  // the lower end is 0 because this actuator can only add water — a negative command would
  // mean "remove water", and there is no pump that can do that. A reading above the
  // setpoint therefore commands exactly 0 rather than a negative number that a later
  // conversion might reinterpret.
  if (requested > static_cast<int64_t>(tuning_.max_duty_permille)) {
    terms.duty_permille = tuning_.max_duty_permille;
    terms.output_clamped = true;
  } else if (requested < 0) {
    terms.duty_permille = 0;
    terms.output_clamped = true;
  } else {
    terms.duty_permille = static_cast<int32_t>(requested);
  }

  terms_ = terms;
  return terms;
}

WindowOutput::WindowOutput(uint32_t window_ms) : window_ms_(window_ms) {}

void WindowOutput::begin_window(uint32_t now_ms, uint32_t on_time_ms) {
  started_at_ms_ = now_ms;
  on_time_ms_ = on_time_ms > window_ms_ ? window_ms_ : on_time_ms;
  active_ = true;
}

bool WindowOutput::on(uint32_t now_ms) const {
  if (!active_) {
    return false;
  }
  const uint32_t elapsed_ms = now_ms - started_at_ms_;
  return elapsed_ms < on_time_ms_;
}

void WindowOutput::abort() {
  active_ = false;
  on_time_ms_ = 0;
}

ControlLoop::ControlLoop(const ControlTuningStore *tuning, uint32_t control_period_ms)
    : tuning_(tuning), window_(control_period_ms), control_period_ms_(control_period_ms) {}

ControlDecision ControlLoop::update(uint32_t now_ms, const AnalogSnapshot &snapshot) {
  // Read the tuning once, at the top. Once, because a caller that sees the setpoint in the
  // decision must be looking at the setpoint that produced it: re-reading mid-evaluation
  // would allow a decision whose terms came from one set of gains and whose reported
  // setpoint came from another, which is exactly the incoherence decision 9's
  // publication scheme exists to avoid.
  const ControlTuning tuning = tuning_ != nullptr ? tuning_->load() : ControlTuning{};
  pid_.set_tuning(tuning);

  ControlDecision decision{};
  decision.now_ms = now_ms;
  decision.measured_permille = snapshot.moisture_permille;
  decision.setpoint_permille = tuning.setpoint_permille;
  stats_.evaluations++;

  // The sample interval. The first evaluation has no measured one, so it uses the nominal
  // period: the alternative (0 ms) would drop the first sample's entire contribution from
  // the integral, and a loop that ignores its first look at the plant is one that starts
  // its history one sample late for no reason anyone could point to.
  const uint32_t dt_ms = evaluated_once_ ? (now_ms - last_evaluation_ms_) : control_period_ms_;
  last_evaluation_ms_ = now_ms;
  evaluated_once_ = true;
  decision.dt_ms = dt_ms;

  // The fail-safe (decision 8): an absent or stale reading forces the output to 0.
  //
  // Note which predicate: fresh(), not valid. AcquisitionService deliberately keeps the
  // last good value after a failure, so `valid` stays true through a dead DMA channel and
  // a stopped timer, and a loop that checked it would keep watering a pot on the strength
  // of a number that stopped changing minutes ago. This is the same distinction node.cpp
  // makes for the display, for the same reason.
  //
  // What happens to the integral: it is held, not unwound and not cleared. Held, because
  // during a blind interval the loop has no information about the error, so winding it
  // down would be acting on a guess; not cleared, because the integral is the only state
  // that removes the steady-state offset, and clearing it makes every sensor hiccup cost
  // the loop the offset it spent minutes earning. The dt bookkeeping advances anyway, so
  // the interval that just elapsed is accounted for exactly once and the recovery's
  // integral step is one control period rather than the whole blind gap.
  if (!snapshot.fresh()) {
    stats_.fail_safe_windows++;
    window_.abort();
    decision.fail_safe = true;
    decision.requested_duty_permille = 0;
    decision.duty_permille = 0;
    decision.on_time_ms = 0;
    // The held terms, so a caller can see the integral that survived the gap rather than
    // having to trust that it did.
    decision.terms = pid_.terms();
    last_ = decision;
    return decision;
  }

  const PidTerms terms = pid_.update(snapshot.moisture_permille, dt_ms);
  decision.terms = terms;
  decision.requested_duty_permille = terms.duty_permille;

  // Saturation is about the *upper* clamp and nothing else: it means the law asked for more
  // water than it is allowed to deliver, so the loop is not in control of the reading and
  // the reading is not where it was asked to be. A negative sum clamped up to zero is the
  // opposite situation — the soil is wetter than the setpoint and this actuator cannot take
  // water back — and folding the two together would make "saturated" fire on a healthy pot
  // that is simply wet, which is the number a person watching the node would learn to
  // ignore. PidTerms::output_clamped covers both ends; this covers the one that matters.
  decision.saturated = terms.requested_duty_permille > tuning.max_duty_permille;
  if (decision.saturated) {
    stats_.saturated_windows++;
  }

  // The duty is a fraction of the window: duty permille of a period in milliseconds is
  // (duty * period) / 1000 milliseconds of on-time. With the decided 1000 ms period and a
  // duty range of 0..500 this is exact — the on-time in milliseconds is numerically the
  // duty in permille — and the general form is written out because the period is a
  // parameter, and because a truncating conversion is easier to check than a special case.
  uint32_t on_time_ms = static_cast<uint32_t>(
      (static_cast<int64_t>(terms.duty_permille) * static_cast<int64_t>(control_period_ms_)) /
      1000);

  // The minimum on-time rule (decision 5). A nonzero on-time below the minimum becomes 0:
  // skip the window rather than tick a pump that cannot act. See board.h for why a short
  // pulse is not a small delivery, and note that the skip is counted, so a loop that has
  // been pushed entirely below the threshold is diagnosable from the stats instead of
  // looking like a pump that has stopped.
  if (on_time_ms > 0 && on_time_ms < tuning.min_on_time_ms) {
    on_time_ms = 0;
    decision.skipped = true;
    stats_.min_on_time_skips++;
  }

  window_.begin_window(now_ms, on_time_ms);
  decision.on_time_ms = window_.on_time_ms();

  // What the window will actually deliver, computed from the on-time rather than copied
  // from the request: with a period other than 1000 ms the conversion truncates, and
  // reporting the requested duty would then overstate what the pump is about to do. A
  // zero-length window delivers nothing, which is also the answer that avoids dividing by
  // it.
  const uint32_t delivered_permille =
      control_period_ms_ == 0
          ? 0U
          : static_cast<uint32_t>((static_cast<uint64_t>(decision.on_time_ms) * 1000U) /
                                  control_period_ms_);
  decision.duty_permille = static_cast<int32_t>(delivered_permille);

  if (decision.on_time_ms == 0) {
    // Counted over evaluations only. A fail-safe window also delivers nothing, but it is
    // counted as a fail-safe window, and keeping the two apart is what makes
    // fail_safe_windows + zero_duty_windows + acting windows == evaluations, which is the
    // identity a reader (and a test) can check.
    stats_.zero_duty_windows++;
  }

  last_ = decision;
  return decision;
}

} // namespace node
