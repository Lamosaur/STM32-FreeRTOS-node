/* step_capture.h — bounded recording of one step response (FR-1.5).
 *
 * FR-1.5 asks for step-response data exported for plotting off-target, and design-spec.md
 * §9 records rise time, overshoot and settling time as the figures it has to produce. The
 * node cannot plot and should not try: what it can do is record one response faithfully
 * and hand the samples to a host. This class is the recorder, host/step_response.py is the
 * analysis, and the CSV between them is the interface.
 *
 * Two things shape the design.
 *
 * The step has to be detectable rather than commanded, because the interesting sample is
 * the one *before* the plant moves and nothing in this system can be asked to capture
 * retroactively. So arm() takes the setpoint as it stands now, and the recorder waits: the
 * step is the moment the setpoint the control task reports moves away from the armed value
 * by more than a threshold. Arming can therefore happen well before the setpoint is
 * commanded, which is what makes an unattended capture possible at all.
 *
 * And the record has to be bounded, because the part has 20 KB of RAM and no heap budget
 * for a variable-size log. The bound is the buffer filling or the capture window expiring,
 * whichever comes first.
 *
 * No control law is implemented here or anywhere in this phase. `output` is recorded as an
 * opaque integer supplied by the caller: giving it a unit would be a decision about a
 * controller that has not been chosen yet, and inventing one would let a later reader
 * believe the loop exists.
 */
#ifndef APP_STEP_CAPTURE_H
#define APP_STEP_CAPTURE_H

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/comms.h"

namespace node {

// kStepCaptureCapacity, kStepThresholdPermille and kStepMaxDurationMs are configured in
// app/board.h with the rest of the Phase 1 tunables. They were written here because
// board.h was not writable at the time; the reasoning that matters for this class is
// recorded next to them, and the three facts this header depends on are:
//
//   - the capacity is the RAM bound (128 records x 16 bytes = 2 KB), so a capture cannot
//     grow without someone deciding to spend the RAM;
//   - the threshold is in the setpoint's own units, so a host nudging the setpoint by one
//     count is not mistaken for a step;
//   - the duration is measured from the step, not from arming, because arming early is
//     the normal case.

// One sample of the response.
//
// `t_ms` is time since arm(), not since the step: the difference between the two is
// exactly the information a reader needs to see how long the setpoint sat where it was,
// and it is one subtraction for the host if it wants the other reference. At the step
// itself t_ms is the arm-to-step delay, which is the first row's value in the CSV.
struct StepRecord {
  uint32_t t_ms = 0;
  int32_t setpoint = 0;
  int32_t measurement = 0;
  int32_t output = 0;
};

struct StepCaptureStats {
  uint32_t arms = 0;           // arm() calls
  uint32_t captures = 0;       // arms that went on to detect a step
  uint32_t records = 0;        // records written across all captures
  uint32_t full_stops = 0;     // captures that ended because the buffer filled
  uint32_t timeout_stops = 0;  // captures that ended at kStepMaxDurationMs
  uint32_t idle_updates = 0;   // update() calls with no capture in progress
};

class StepCapture {
 public:
  static constexpr size_t kCapacity = kStepCaptureCapacity;

  explicit StepCapture(int32_t step_threshold = kStepThresholdPermille,
                       uint32_t max_duration_ms = kStepMaxDurationMs);

  StepCapture(const StepCapture &) = delete;
  StepCapture &operator=(const StepCapture &) = delete;

  // Begin watching for a step, discarding any previous capture. `now_ms` is the caller's
  // clock — the class takes timestamps rather than reading one, so a host test can drive a
  // response at any rate with no clock double, and so the recording is tied to the clock
  // the control loop actually runs on (a DWT-based one, not the RTOS tick).
  void arm(int32_t setpoint, uint32_t now_ms);

  // Stop watching or recording, keeping the records already taken so they can still be
  // exported. Used when a capture is abandoned rather than completed.
  void disarm();

  // One control-loop cycle. Detects the step if it is still waiting for one, and records
  // the sample if a capture is running. Returns true when this call wrote a record.
  //
  // The cycle that detects the step writes TWO records at the same timestamp: the last
  // pre-step state (the setpoint that was armed, and the measurement taken before the plant
  // could move) and then the stepped state (the new setpoint, same measurement). Writing
  // the edge explicitly is what makes the CSV self-describing — the step size is derivable
  // from the file rather than having to be passed alongside it — and it puts the baseline
  // the host analysis needs on the wire, which a capture that only started at the step
  // could not do, because nothing can sample the past retroactively. One extra record of
  // the 128 is the cost.
  bool update(uint32_t now_ms, int32_t setpoint, int32_t measurement, int32_t output);

  bool armed() const { return state_ == State::Armed; }
  bool capturing() const { return state_ == State::Capturing; }
  bool complete() const { return state_ == State::Complete; }
  bool idle() const { return state_ == State::Idle; }

  size_t count() const { return count_; }

  // The record at `index`, or an empty record when out of range. Returning a reference
  // keeps the export loop allocation-free and lets a caller index without copying 16 bytes
  // per line.
  const StepRecord &record(size_t index) const;

  int32_t armed_setpoint() const { return armed_setpoint_; }
  uint32_t armed_at_ms() const { return armed_at_ms_; }
  int32_t step_threshold() const { return threshold_; }
  uint32_t max_duration_ms() const { return max_duration_ms_; }
  const StepCaptureStats &stats() const { return stats_; }

  // --- CSV export (FR-1.5) -----------------------------------------------------
  // Line-oriented rather than "format the whole thing": a 128-record capture is about
  // 2.5 KB of text, which the comms task has no buffer for. It sends the header and then
  // one line per record through its own 128-byte line buffer, so exporting costs no more
  // stack than sending a telemetry line. Both return the byte count, 0 if the index is
  // out of range.
  static size_t format_header(char *out, size_t capacity);
  size_t format_record(size_t index, char *out, size_t capacity) const;

 private:
  enum class State : uint8_t {
    Idle,      // nothing armed
    Armed,     // watching for the step
    Capturing, // recording
    Complete,  // stopped, at a bound
  };

  // Timestamps advance monotonically within a capture, but the RTOS clock wraps at 2^32 ms
  // (rtos.h), so elapsed time is a subtraction rather than a comparison, exactly as the
  // supervisor does it.
  uint32_t elapsed_ms(uint32_t now_ms) const;

  // Append one record, if the buffer and the capture window both allow it. Marks the
  // capture complete and counts which bound was hit when they do not. Returns true if a
  // record was written.
  bool write(uint32_t now_ms, int32_t setpoint, int32_t measurement, int32_t output);

  State state_ = State::Idle;
  int32_t armed_setpoint_ = 0;
  uint32_t armed_at_ms_ = 0;
  uint32_t started_at_ms_ = 0; // when the step was detected
  int32_t threshold_;
  uint32_t max_duration_ms_;
  size_t count_ = 0;
  StepRecord records_[kStepCaptureCapacity]{};
  StepCaptureStats stats_{};
};

} // namespace node

#endif // APP_STEP_CAPTURE_H
