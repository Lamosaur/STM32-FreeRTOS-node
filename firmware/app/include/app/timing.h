/* timing.h — measurement of the two scheduling numbers Phase 1 must produce rather than
 * assume: control-loop period jitter (NFR-1.8) and worst-case ISR-to-task latency
 * (NFR-1.9).
 *
 * design-spec.md §9 verifies both with a logic analyzer, and that is the right instrument
 * for the edge itself. What a scope cannot do is summarise: "the pulse was 10.03 ms" is
 * one sample, and the number NFR-1.8 asks for is a property of the distribution — how far
 * the period wanders, how often, and whether the excursions are one tick or a hundred.
 * This class is the other half of that measurement: the control task times its own period
 * (or reads a DWT cycle counter in the ISR for the latency case) and records it here, and
 * the report is what goes into docs/measurements.md.
 *
 * Why everything here is integer. The STM32F103 is a Cortex-M3: no FPU, so every float
 * operation becomes a call into the soft-float library. For a mean and a standard
 * deviation that would mean linking that library into the image and paying tens of cycles
 * per operation to produce a number that is then printed with two digits. The variance is
 * computed from the sum and the sum of squares in uint64 — both exact for the domain
 * below — and the standard deviation comes out of an integer square root. The cost is
 * that the result is the floor of the true value, which at microsecond resolution is
 * below the resolution of the instrument that produced the samples.
 *
 * Ownership: this class is single-writer, with no mutex, unlike SensorService. The
 * samples come from the control task, which is also the only sensible place to format the
 * report ("my period jitter") — and putting a mutex on this path would put a lock
 * acquisition inside the loop whose jitter is being measured, which is precisely the
 * measurement NFR-1.8 must not perturb. A caller that wants a report from another task
 * should have the control task format it into a buffer and hand that over.
 */
#ifndef APP_TIMING_H
#define APP_TIMING_H

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/comms.h"

namespace node {

// kTimingMaxValueUs and kTimingWindowSamples are the two bounds on a measurement window.
// They are configured in app/board.h with the rest of the Phase 1 tunables (this header
// used to carry them, before board.h was written), because the window length is a property
// of the measurement plan rather than of this class: the fast bound is the arithmetic
// guarantee below — the sums stay exact only while the window is finite — and the slow
// bound is how long a run reports for, which is a question about NFR-0.7 rather than about
// statistics.
//
// Their behaviour, stated here because it is this class that implements it: when the
// window is full, further samples are ignored and full() says so, so the caller reports and
// resets. Ignoring rather than re-arming automatically is deliberate — an automatic reset
// would silently discard the window being accumulated, and a caller that never reports
// would never learn it had lost the data.

// Histogram bucket edges, in microseconds of deviation from the nominal. Explicit edges
// rather than an even split, because the interesting question is not "how wide is the
// distribution" but "is the excursion one scheduler tick or a hundred": the first five
// buckets separate a well-behaved loop (a few microseconds, interrupt latency) from a
// tick-quantised one (1 ms and its multiples), and the last two cover a stall big enough
// that something did not run.
constexpr uint32_t kTimingBucketEdgesUs[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000};

// One bucket per edge, plus the overflow bucket for anything above the last edge.
constexpr size_t kTimingBucketCount =
    (sizeof(kTimingBucketEdgesUs) / sizeof(kTimingBucketEdgesUs[0])) + 1;

// The bucket holding the deviation `deviation_us`: edges[i-1] < deviation <= edges[i],
// with the first bucket taking anything up to the first edge and the last taking anything
// above the final one.
constexpr size_t timing_bucket_index(uint32_t deviation_us) {
  for (size_t i = 0; i + 1 < kTimingBucketCount; ++i) {
    if (deviation_us <= kTimingBucketEdgesUs[i]) {
      return i;
    }
  }
  return kTimingBucketCount - 1;
}

// The upper bound printed for a bucket. The overflow bucket reports the last edge and is
// printed with a '>' rather than a '<='.
constexpr uint32_t timing_bucket_upper_bound_us(size_t index) {
  constexpr size_t kLastEdge = (sizeof(kTimingBucketEdgesUs) / sizeof(kTimingBucketEdgesUs[0])) - 1;
  return kTimingBucketEdgesUs[index < kLastEdge ? index : kLastEdge];
}

// One stream of microsecond measurements: count, min, max, mean, standard deviation, and a
// histogram of the deviation from a nominal.
//
// The raw values and the deviations are kept apart on purpose. A histogram of the raw
// period would be a single bucket, because every sample is within a few percent of 100 ms;
// what has a shape is the deviation. But the raw statistics are still what a reader wants
// first, since they are what exposes a systematic offset — a tick-based period that is
// consistently 102.4 ms rather than 100 ms is a different defect from a jittery one, and
// only the raw mean shows it.
class TimingSeries {
 public:
  // Value and deviation in microseconds. `deviation_us` is how far this sample is from
  // whatever this stream's nominal is (0 for latency).
  void note(uint32_t value_us, uint32_t deviation_us);
  void reset();

  uint32_t count() const { return count_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= kTimingWindowSamples; }
  uint32_t oversized_samples() const { return oversized_; }

  // All zero when no sample has been recorded; the report prints n=0 in that case, which
  // is unambiguous, whereas a min of 0 for an unmeasured stream would not be.
  uint32_t min_us() const { return min_; }
  uint32_t max_us() const { return max_; }
  uint32_t mean_us() const;
  uint32_t stddev_us() const;

  uint32_t bucket_count() const { return static_cast<uint32_t>(kTimingBucketCount); }
  uint32_t bucket(size_t index) const;
  uint32_t bucketed_samples() const;

  // `n=2048 min=99800 max=100400 mean=100001 sd=38`
  size_t format_stats(LineBuilder &line, const char *label) const;

  // `period hist us: <=1:1200 <=2:400 ...` — one line per `group` of six buckets, because
  // twelve buckets on one line do not fit the 128-byte line budget the comms task sends.
  size_t format_histogram(LineBuilder &line, const char *label, size_t group) const;

 private:
  uint32_t count_ = 0;
  uint64_t sum_ = 0;
  uint64_t sum_squares_ = 0;
  uint32_t min_ = 0;
  uint32_t max_ = 0;
  uint32_t oversized_ = 0;
  uint32_t histogram_[kTimingBucketCount]{};
};

// Number of buckets printed per histogram line. Six keeps a line well inside the comms
// buffer; the report is six lines rather than two long ones, and a caller can stream them.
constexpr size_t kTimingBucketsPerLine = 6;

// The nominal control period, used only to turn a measured period into a deviation.
//
// This now comes from the decided control period in board.h (1 s, design-spec.md §6.1)
// rather than from the app task's period, which is what it used to fall back to while the
// control period was undecided. The two disagreed the moment the decision was made, and a
// nominal of 100 ms against a real period of 1 s would have reported every sample as a
// 900 ms deviation — a jitter figure that is entirely an artefact of the wrong constant.
constexpr uint32_t kDefaultNominalPeriodUs = kControlPeriodMs * 1000U;

class TimingService {
 public:
  explicit TimingService(uint32_t nominal_period_us = kDefaultNominalPeriodUs);

  TimingService(const TimingService &) = delete;
  TimingService &operator=(const TimingService &) = delete;

  // One control-loop period, as measured between two entries of the control task.
  void note_period_us(uint32_t period_us);

  // One ISR-to-task latency: the ISR takes a timestamp when it fires and the woken task
  // takes another at entry; the difference is what this records. The DWT cycle counter is
  // the timestamp source, which is why the class takes a value and knows nothing about
  // clocks.
  void note_latency_us(uint32_t latency_us);

  const TimingSeries &period() const { return period_; }
  const TimingSeries &latency() const { return latency_; }

  // Only affects how a period is turned into a deviation; changing it does not rewrite the
  // samples already recorded, so it is set before the run rather than during it.
  uint32_t nominal_period_us() const { return nominal_period_us_; }
  void set_nominal_period_us(uint32_t us) { nominal_period_us_ = us; }

  // True once either window is complete: the caller should report and reset.
  bool full() const { return period_.full() || latency_.full(); }

  void reset();

  // Lines, in order: period statistics, two period histogram lines, latency statistics,
  // two latency histogram lines.
  static constexpr size_t kReportLines = 6;

  // A line buffer sized for one line. The comms task's own line buffer is the same size, so
  // a caller that streams the report one line at a time needs no extra storage.
  static constexpr size_t kLineBytes = kCommsRxBufferBytes;

  // A buffer that holds the whole report. 6 lines of at most 128 bytes plus separators.
  static constexpr size_t kReportBytes = 512;

  // Format line `index` into `out`. Returns the byte count, or 0 if the index is out of
  // range. Exposed separately from report() so the comms task can stream the report
  // through its 128-byte line buffer instead of holding 512 bytes of stack.
  size_t format_line(size_t index, char *out, size_t capacity) const;

  // The whole report, newline-separated and NUL-terminated, truncated at a line boundary
  // if `capacity` is too small. Returns the byte count written.
  size_t report(char *out, size_t capacity) const;

 private:
  uint32_t nominal_period_us_;
  TimingSeries period_;
  TimingSeries latency_;
};

} // namespace node

#endif // APP_TIMING_H
