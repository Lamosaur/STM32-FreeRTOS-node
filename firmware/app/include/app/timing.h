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

#include <atomic>
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

// ---------------------------------------------------------------------------
// The handover
// ---------------------------------------------------------------------------

// A single-writer/single-reader handover for the report *text*, which is what lets the
// comms task send the NFR-1.8/NFR-1.9 figures that the app task measured. It exists
// because TimingService is single-writer by design (its header explains why the sample
// path cannot take a lock), so a report from another task has to be a formatted buffer
// that is handed over, not a shared series — see the note in comms.h's source list.
//
// Why this shape. The alternative to a lock is a buffer the two tasks pass back and
// forth, and the discipline that makes a plain buffer safe there is strict alternation:
// the writer (the app task) fills the buffer only while no publication is outstanding,
// and the reader (the comms task) reads it only while one is. Three single bits carry
// the protocol:
//
//   requested_        reader -> writer: "format the series as it stands"
//   ready_            writer -> reader: "a report is waiting"
//   reset_pending_    reader -> writer: "the report you published was a full window;
//                     start the next one"
//
// The memory-ordering claim, because race-freedom is the one claim this class exists to
// make. The writer sees ready_ false (acquire), stores buffer_, length_ and
// window_complete_, then stores ready_ true (release). The reader sees ready_ true
// (acquire), copies what it needs, then stores ready_ false (release). The writer only
// writes while ready_ is false and the reader only reads while it is true, so the two
// never touch the buffer at the same instant, and each release pairs with the other
// task's next acquire — the buffer writes happen-before the reads that consume them.
// That strict alternation is what makes the plain buffer race-free as the abstract
// machine defines the term, which the tuning store's seqlock (control.h) deliberately
// cannot claim: its reader may copy a slot the writer is filling, while this reader
// cannot, because it waits for the writer's explicit hand-off instead.
//
// Why there is no mutex even though both tasks touch this object. The writer is the app
// task, whose period NFR-1.8 measures: a lock acquisition there is precisely the
// perturbation the measurement must not contain. The alternation above costs nothing
// when nothing is being read, and the reader never waits for the writer: a request is
// picked up on the writer's next tick and the report lands on the reader's next poll
// (comms runs at 50 ms), so the command's "report pending" answer is honest rather than
// a stall.
class TimingReportHandover {
 public:
  // Enough for the whole report: six lines of at most kLineBytes plus separators and the
  // terminator (TimingService::kReportBytes = 512).
  static constexpr size_t kCapacity = TimingService::kReportBytes;

  TimingReportHandover() = default;

  TimingReportHandover(const TimingReportHandover &) = delete;
  TimingReportHandover &operator=(const TimingReportHandover &) = delete;

  // --- writer's side: the task that owns the TimingService (the app task) -------------

  // Format the service's current report into the buffer and mark it ready, unless a
  // previous publication is still unread. Returns false in that case and changes nothing.
  // The caller retries on a later tick rather than overwrite a report the reader has not
  // consumed: the writer may only write while nothing is outstanding (see above).
  bool publish(const TimingService &service, bool window_complete);

  // True while a publication is waiting to be consumed. This is the writer's gate; the
  // name is the reader's view of the same bit, and both must agree on it.
  bool ready() const { return ready_.load(std::memory_order_acquire); }

  // True when the reader consumed a *complete-window* report, i.e. the writer should
  // reset its series so the next window starts accumulating. Returns false when there is
  // nothing to reset. Only the writer may call this, and it is the only task that may
  // reset the series — the reset happens on the writer's side so the series is touched by
  // exactly one task.
  bool take_reset();

  // True when the reader asked for a snapshot since the writer last serviced a request,
  // and the request is now cleared. The writer pairs this with its own tick logic: if it
  // returns true (or the series is full), publish(); if neither, leave the series alone.
  // Only the writer may call this.
  bool take_request();

  // --- reader's side: the comms task ---------------------------------------------------

  // Ask for a snapshot of the series as it stands. Non-destructive: what comes back is a
  // partial window if that is what has accumulated, and the series keeps accumulating
  // after the request is serviced.
  void request();

  // The published report. buffer() and length() are valid only while ready() is true and
  // must not be used after consume(); the reader copies what it needs first.
  const char *buffer() const { return buffer_; }
  size_t length() const { return length_; }

  // True when the outstanding publication is a complete window (the writer published
  // because the series filled, not because it was asked). The reader folds this into
  // consume(), which is how the writer learns to reset.
  bool window_complete() const { return window_complete_; }

  // Mark the outstanding publication consumed and, if it was a complete window, arm the
  // writer's reset so the next window starts only once the current one was actually read
  // rather than dropped.
  void consume();

 private:
  char buffer_[kCapacity]{};
  uint32_t length_ = 0;             // written before ready_ goes true, read after
  bool window_complete_ = false;    // same discipline
  std::atomic<bool> ready_{false};  // writer -> reader
  std::atomic<bool> requested_{false};    // reader -> writer
  std::atomic<bool> reset_pending_{false}; // reader -> writer
};

} // namespace node

#endif // APP_TIMING_H
