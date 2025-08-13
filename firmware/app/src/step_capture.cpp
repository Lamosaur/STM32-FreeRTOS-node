#include "app/step_capture.h"

namespace node {

StepCapture::StepCapture(int32_t step_threshold, uint32_t max_duration_ms)
    : threshold_(step_threshold), max_duration_ms_(max_duration_ms) {}

void StepCapture::arm(int32_t setpoint, uint32_t now_ms) {
  state_ = State::Armed;
  armed_setpoint_ = setpoint;
  armed_at_ms_ = now_ms;
  started_at_ms_ = now_ms;
  // Re-arming discards the previous capture, because the buffer is a bring-up instrument
  // rather than a log: two step responses appended together in one CSV would have two
  // baselines and two steps in it, and every figure the host analysis derives from it
  // would be meaningless. The caller that wants the previous one exports it before arming.
  count_ = 0;
  stats_.arms++;
}

void StepCapture::disarm() {
  state_ = State::Idle;
}

uint32_t StepCapture::elapsed_ms(uint32_t now_ms) const {
  // Unsigned subtraction, so a wrap of the millisecond clock between arming and the sample
  // produces the correct elapsed time rather than a huge one. See rtos.h on now_ms.
  return now_ms - armed_at_ms_;
}

bool StepCapture::update(uint32_t now_ms, int32_t setpoint, int32_t measurement,
                         int32_t output) {
  if (state_ == State::Idle || state_ == State::Complete) {
    // Nothing to do, but counted: a node recording nothing while the host expects a
    // capture is worth seeing in the stats rather than being silently free.
    stats_.idle_updates++;
    return false;
  }

  if (state_ == State::Capturing) {
    return write(now_ms, setpoint, measurement, output);
  }

  // State::Armed — waiting for the setpoint to move.
  //
  // The difference is taken in 64 bits so that two setpoints at opposite ends of the int32
  // range cannot wrap into a small difference and hide a step.
  const int64_t difference = static_cast<int64_t>(setpoint) - static_cast<int64_t>(armed_setpoint_);
  const int64_t magnitude = difference < 0 ? -difference : difference;
  // At the threshold counts as a step, below it does not: an inclusive boundary is a number
  // a test can pin from both sides, whereas "more than" would leave the exact value's
  // behaviour to whoever reads the code next.
  if (magnitude < static_cast<int64_t>(threshold_)) {
    return false;
  }

  // Detected. The capture window starts at the step, not at arming: a capture armed a
  // minute before the setpoint was commanded would otherwise have no window left.
  state_ = State::Capturing;
  started_at_ms_ = now_ms;
  stats_.captures++;

  // Two records at this one timestamp, bracketing the step: the state before it and the
  // state after it. Both are needed and neither can be reconstructed later — the
  // measurement before the plant moved is the baseline the host analysis measures rise
  // time and overshoot from, and the new setpoint is what the response is heading for.
  const bool wrote_pre_step =
      write(now_ms, armed_setpoint_, measurement, output);
  const bool wrote_step = write(now_ms, setpoint, measurement, output);
  return wrote_pre_step || wrote_step;
}

bool StepCapture::write(uint32_t now_ms, int32_t setpoint, int32_t measurement,
                        int32_t output) {
  if (count_ >= kStepCaptureCapacity) {
    // The buffer is full, and the capture stops here rather than wrapping to overwrite the
    // oldest record. Justified because the two ends of the record are not equally
    // valuable: rise time, overshoot and the step edge all live in the first samples, and a
    // ring would keep the settled tail (which is over-sampled by definition) while
    // discarding the transient that the capture exists to measure. The cost is that a
    // response which has not settled when the buffer fills is exported truncated, and the
    // host analyser reports "not settled in the captured window" for it rather than
    // inventing a settling time.
    state_ = State::Complete;
    stats_.full_stops++;
    return false;
  }

  if ((now_ms - started_at_ms_) >= max_duration_ms_) {
    // The duration bound, for the case the buffer does not catch: a control task running
    // far slower than designed, or a step that never stops being interesting. Same policy
    // and same cost as above. A window of zero records nothing at all, which is the correct
    // reading of a zero-length capture.
    state_ = State::Complete;
    stats_.timeout_stops++;
    return false;
  }

  StepRecord sample{};
  sample.t_ms = elapsed_ms(now_ms);
  sample.setpoint = setpoint;
  sample.measurement = measurement;
  sample.output = output;
  records_[count_] = sample;
  count_++;
  stats_.records++;
  return true;
}

const StepRecord &StepCapture::record(size_t index) const {
  // A zero record rather than a null or an assert: the export loop is driven by count(),
  // so an out-of-range index is a programming error, and the least harmful answer is an
  // empty row that a CSV reader will reject rather than a crash on a node in a greenhouse.
  static const StepRecord kEmpty{};
  return index < count_ ? records_[index] : kEmpty;
}

size_t StepCapture::format_header(char *out, size_t capacity) {
  if (out == nullptr || capacity == 0) {
    return 0;
  }
  LineBuilder line(out, capacity);
  line.str("t_ms,setpoint,measurement,output");
  return line.size();
}

size_t StepCapture::format_record(size_t index, char *out, size_t capacity) const {
  if (index >= count_) {
    // Terminated rather than left untouched: a caller that ignores the return value would
    // otherwise export whatever the buffer held last, which on a CSV stream is a
    // plausible-looking row from an earlier record.
    if (out != nullptr && capacity > 0) {
      out[0] = '\0';
    }
    return 0;
  }
  if (out == nullptr || capacity == 0) {
    return 0;
  }

  const StepRecord &sample = records_[index];
  LineBuilder line(out, capacity);
  // LineBuilder's integer formatting only — no printf and no locale, which keeps the CSV
  // byte-exact and assertable in a host test, and keeps newlib's formatter out of the
  // image. Output is signed because a controller's output can legitimately be negative
  // (below the actuator's rest position), and a CSV that silently dropped the sign would
  // make the host analysis read a different response from the one that happened.
  line.u32(sample.t_ms)
      .ch(',')
      .i32(sample.setpoint)
      .ch(',')
      .i32(sample.measurement)
      .ch(',')
      .i32(sample.output);
  return line.size();
}

} // namespace node
