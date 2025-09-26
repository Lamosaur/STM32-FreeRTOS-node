#include "app/timing.h"

namespace node {
namespace {

// Floor of the square root, without floating point.
//
// Newton's method on integers converges in a handful of iterations from any start above
// the root, and because each step is exact integer division the result is exactly
// floor(sqrt(value)) rather than an approximation of it. The alternative — casting to
// double — is what this file exists to avoid: on a Cortex-M3 that is a library call per
// iteration plus the conversion, for a number printed with two digits.
//
// The intermediate `x + value / x` can only overflow if value is within one of UINT64_MAX.
// The largest value passed here is a variance, bounded by (kTimingMaxValueUs)^2 = 1e12, so
// there is no path to it.
uint64_t isqrt(uint64_t value) {
  if (value == 0) {
    return 0;
  }
  uint64_t root = value;
  uint64_t next = (root + 1) / 2;
  while (next < root) {
    root = next;
    next = (root + value / root) / 2;
  }
  return root;
}

} // namespace

void TimingSeries::note(uint32_t value_us, uint32_t deviation_us) {
  // A full window ignores further samples rather than overwriting the oldest, so the
  // statistics describe one contiguous run of the loop instead of a sliding average that
  // spans runs. The alternative — a ring — would make the woken-up worst case disappear
  // from the report after a few thousand good samples, which is the one number NFR-1.9
  // exists to produce.
  if (full()) {
    return;
  }

  uint32_t value = value_us;
  if (value > kTimingMaxValueUs) {
    value = kTimingMaxValueUs;
    oversized_++;
  }

  uint32_t deviation = deviation_us;
  if (deviation > kTimingMaxValueUs) {
    deviation = kTimingMaxValueUs;
  }

  if (count_ == 0 || value < min_) {
    min_ = value;
  }
  if (count_ == 0 || value > max_) {
    max_ = value;
  }
  sum_ += value;
  sum_squares_ += static_cast<uint64_t>(value) * static_cast<uint64_t>(value);
  histogram_[timing_bucket_index(deviation)]++;
  count_++;
}

void TimingSeries::reset() {
  count_ = 0;
  sum_ = 0;
  sum_squares_ = 0;
  min_ = 0;
  max_ = 0;
  oversized_ = 0;
  for (size_t i = 0; i < kTimingBucketCount; ++i) {
    histogram_[i] = 0;
  }
}

uint32_t TimingSeries::mean_us() const {
  if (count_ == 0) {
    return 0;
  }
  // Rounded, not truncated: a truncated mean of jitter samples is biased low by up to a
  // microsecond, which on a 10 us mean is ten percent of the answer.
  return static_cast<uint32_t>((sum_ + count_ / 2U) / count_);
}

uint32_t TimingSeries::stddev_us() const {
  // One sample has no spread. Zero is the honest answer, and it is also what the identity
  // below produces, but stating it here keeps the identity from being read as the
  // definition of the answer.
  if (count_ < 2) {
    return 0;
  }

  // Population variance from the sum and the sum of squares:
  //
  //   variance = (n * sum(x^2) - sum(x)^2) / n^2
  //
  // The unsigned wrap question, answered rather than hoped: the window holds at most
  // kTimingWindowSamples samples and every sample is clamped to kTimingMaxValueUs, so
  // n * sum(x^2) and sum(x)^2 are both bounded by (2048 * 10^6)^2, which is about 4.3e18.
  // uint64 holds 1.8e19, so neither product can wrap, and the identity itself is
  // non-negative by Cauchy-Schwarz, so the subtraction cannot underflow either. That bound
  // is why the window is bounded: an unbounded run would eventually wrap, silently, after
  // hours — which is exactly the kind of failure a 72 h unattended run (NFR-0.7) is meant
  // to survive.
  const uint64_t n = count_;
  const uint64_t numerator = n * sum_squares_ - sum_ * sum_;
  const uint64_t variance = numerator / (n * n);

  // Floor of the square root: the reported standard deviation is never larger than the
  // true one, by less than 1 us.
  return static_cast<uint32_t>(isqrt(variance));
}

uint32_t TimingSeries::bucket(size_t index) const {
  return index < kTimingBucketCount ? histogram_[index] : 0;
}

uint32_t TimingSeries::bucketed_samples() const {
  uint32_t total = 0;
  for (size_t i = 0; i < kTimingBucketCount; ++i) {
    total += histogram_[i];
  }
  return total;
}

size_t TimingSeries::format_stats(LineBuilder &line, const char *label) const {
  line.str(label)
      .str(" us: n=")
      .u32(count_)
      .str(" min=")
      .u32(min_)
      .str(" max=")
      .u32(max_)
      .str(" mean=")
      .u32(mean_us())
      .str(" sd=")
      .u32(stddev_us());

  // Printed only when it happened, and then printed, because a clamped sample is a
  // measurement that did not fit the model: the mean over such a window is a lower bound,
  // and a reader who cannot see that would read it as the answer.
  if (oversized_ > 0) {
    line.str(" over=").u32(oversized_);
  }
  return line.size();
}

size_t TimingSeries::format_histogram(LineBuilder &line, const char *label,
                                      size_t group) const {
  const size_t first = group * kTimingBucketsPerLine;
  if (first >= kTimingBucketCount) {
    return line.size();
  }

  line.str(label).str(" hist us:");
  for (size_t i = first; i < first + kTimingBucketsPerLine && i < kTimingBucketCount; ++i) {
    line.ch(' ');
    // The last bucket is everything above the final edge, so it prints as ">2000" rather
    // than continuing the "<=" series with an edge it does not have.
    if (i + 1 == kTimingBucketCount) {
      line.ch('>');
    } else {
      line.str("<=");
    }
    line.u32(timing_bucket_upper_bound_us(i)).ch(':').u32(histogram_[i]);
  }
  return line.size();
}

TimingService::TimingService(uint32_t nominal_period_us)
    : nominal_period_us_(nominal_period_us) {}

void TimingService::note_period_us(uint32_t period_us) {
  // The deviation is computed in unsigned arithmetic with an explicit larger/smaller test
  // rather than by subtracting signed values: periods and nominals are both near 2^32 only
  // in theory, but a signed difference here would be the one place in this file where a
  // wrap could change a bucket instead of being caught.
  const uint32_t deviation = period_us > nominal_period_us_
                                 ? period_us - nominal_period_us_
                                 : nominal_period_us_ - period_us;
  period_.note(period_us, deviation);
}

void TimingService::note_latency_us(uint32_t latency_us) {
  // The nominal latency is zero — an ISR that wakes a task instantly is the ideal — so the
  // deviation is the value itself, and the latency histogram buckets are microseconds
  // absolute.
  latency_.note(latency_us, latency_us);
}

void TimingService::reset() {
  period_.reset();
  latency_.reset();
}

size_t TimingService::format_line(size_t index, char *out, size_t capacity) const {
  LineBuilder line(out, capacity);
  switch (index) {
    case 0:
      period_.format_stats(line, "period");
      // The nominal is reported with the period because it is what the histogram's
      // deviations are measured from: without it the histogram is not interpretable.
      line.str(" nom=").u32(nominal_period_us_);
      break;
    case 1:
    case 2:
      period_.format_histogram(line, "period", index - 1);
      break;
    case 3:
      latency_.format_stats(line, "latency");
      break;
    case 4:
    case 5:
      latency_.format_histogram(line, "latency", index - 4);
      break;
    default:
      return 0;
  }
  return line.size();
}

size_t TimingService::report(char *out, size_t capacity) const {
  if (out == nullptr || capacity == 0) {
    return 0;
  }
  out[0] = '\0';

  size_t used = 0;
  for (size_t i = 0; i < kReportLines; ++i) {
    char line_buffer[kLineBytes];
    const size_t length = format_line(i, line_buffer, sizeof(line_buffer));

    // One byte for the separator except after the last line, one for the terminator. A
    // line that does not fit is dropped rather than truncated: half a histogram line reads
    // as a complete one with missing buckets.
    const size_t separator = (i == 0) ? 0 : 1;
    if (used + separator + length + 1 > capacity) {
      break;
    }
    if (separator != 0) {
      out[used++] = '\n';
    }
    for (size_t c = 0; c < length; ++c) {
      out[used++] = line_buffer[c];
    }
    out[used] = '\0';
  }
  return used;
}

bool TimingReportHandover::publish(const TimingService &service, bool window_complete) {
  if (ready_.load(std::memory_order_acquire)) {
    // A report the reader has not consumed is not overwritten: the protocol in the header
    // is that the buffer is only written while nothing is outstanding.
    return false;
  }
  length_ = static_cast<uint32_t>(service.report(buffer_, kCapacity));
  window_complete_ = window_complete;
  ready_.store(true, std::memory_order_release);
  return true;
}

bool TimingReportHandover::take_reset() {
  return reset_pending_.exchange(false, std::memory_order_acq_rel);
}

bool TimingReportHandover::take_request() {
  return requested_.exchange(false, std::memory_order_acq_rel);
}

void TimingReportHandover::request() {
  requested_.store(true, std::memory_order_release);
}

void TimingReportHandover::consume() {
  // Read the "was it a full window" bit *before* clearing ready_: the writer may begin a
  // new publication the moment ready_ goes false, and that publication would overwrite
  // window_complete_. Storing false after the read makes the read happen-before any such
  // write.
  const bool complete = window_complete_;
  ready_.store(false, std::memory_order_release);
  if (complete) {
    reset_pending_.store(true, std::memory_order_release);
  }
}

} // namespace node
