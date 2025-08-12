/* Tests for the timing statistics (NFR-1.8, NFR-1.9).
 *
 * The value of a statistics class is entirely in its arithmetic, so the arithmetic is
 * pinned against numbers computed by hand rather than against the implementation:
 *
 *   - a five-sample set whose mean and standard deviation are small enough to work out on
 *     paper, including the fact that the reported standard deviation is the floor of the
 *     true one (sqrt(8) is 2.828…, and 2 is what is reported);
 *   - a three-sample set with a large spread, to catch a variance computed without the
 *     n * sum(x^2) - sum(x)^2 form;
 *   - a single sample, which must give a standard deviation of exactly 0 rather than a
 *     division by zero or a garbage value.
 *
 * The overrun and clamping behaviour is tested too, because it is what makes the sums
 * exact: the window is bounded and the values are clamped, and the worst case the bound
 * allows must still produce the right answer rather than wrapping silently.
 */
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

#include "app/timing.h"

namespace node {
namespace {

// The sample set used throughout: 10, 12, 14, 16, 18 microseconds.
//
//   n    = 5
//   sum  = 70                        mean = 70 / 5 = 14
//   sumsq= 100 + 144 + 196 + 256 + 324 = 1020
//   var  = (5 * 1020 - 70^2) / 25 = (5100 - 4900) / 25 = 200 / 25 = 8
//   sd   = sqrt(8) = 2.828427..., reported as 2
constexpr uint32_t kSet[] = {10, 12, 14, 16, 18};

void note_set(TimingSeries &series, uint32_t nominal_us = 0) {
  for (uint32_t value : kSet) {
    const uint32_t deviation = value > nominal_us ? value - nominal_us : nominal_us - value;
    series.note(value, deviation);
  }
}

TEST(TimingSeriesTest, MatchesAHandComputedMeanAndDeviation) {
  TimingSeries series;
  note_set(series);

  EXPECT_EQ(series.count(), 5u);
  EXPECT_EQ(series.min_us(), 10u);
  EXPECT_EQ(series.max_us(), 18u);
  EXPECT_EQ(series.mean_us(), 14u);
  // sqrt(8) = 2.828..., and the integer square root reports 2. Stated in the test because
  // "the standard deviation is an integer" is a decision, not an approximation to hide.
  EXPECT_EQ(series.stddev_us(), 2u);
}

TEST(TimingSeriesTest, MatchesASecondHandComputedSetWithAWideSpread) {
  TimingSeries series;
  series.note(5, 5);
  series.note(100, 100);
  series.note(7, 7);

  //   n = 3, sum = 112, sumsq = 25 + 10000 + 49 = 10074
  //   var = (3 * 10074 - 112^2) / 9 = (30222 - 12544) / 9 = 17678 / 9 = 1964 (truncated)
  //   sd  = sqrt(1964) = 44.31..., reported as 44 (44^2 = 1936, 45^2 = 2025)
  EXPECT_EQ(series.count(), 3u);
  EXPECT_EQ(series.min_us(), 5u);
  EXPECT_EQ(series.max_us(), 100u);
  EXPECT_EQ(series.mean_us(), 37u); // 112 / 3 = 37.33, rounded
  EXPECT_EQ(series.stddev_us(), 44u);
}

TEST(TimingSeriesTest, ASingleSampleHasNoSpread) {
  TimingSeries series;
  series.note(42, 42);

  EXPECT_EQ(series.count(), 1u);
  EXPECT_EQ(series.min_us(), 42u);
  EXPECT_EQ(series.max_us(), 42u);
  EXPECT_EQ(series.mean_us(), 42u);
  EXPECT_EQ(series.stddev_us(), 0u);
}

TEST(TimingSeriesTest, TwoSamplesRoundTheMeanRatherThanTruncating) {
  TimingSeries series;
  series.note(1, 0);
  series.note(2, 0);

  // 3 / 2 = 1.5, and a truncated mean would report 1. A microsecond of bias on a mean of
  // a few microseconds is a tenth of the answer.
  EXPECT_EQ(series.mean_us(), 2u);
}

TEST(TimingSeriesTest, AnEmptySeriesReportsZeros) {
  TimingSeries series;

  EXPECT_TRUE(series.empty());
  EXPECT_EQ(series.count(), 0u);
  EXPECT_EQ(series.mean_us(), 0u);
  EXPECT_EQ(series.stddev_us(), 0u);
  EXPECT_EQ(series.bucketed_samples(), 0u);
}

// ---------------------------------------------------------------------------
// The histogram
// ---------------------------------------------------------------------------
TEST(TimingSeriesTest, BucketsFollowTheDocumentedEdges) {
  // Boundaries, from the edge table in app/timing.h: {1, 2, 5, 10, 20, 50, 100, 200, 500,
  // 1000, 2000} with one bucket per edge plus an overflow bucket.
  EXPECT_EQ(timing_bucket_index(0), 0u);
  EXPECT_EQ(timing_bucket_index(1), 0u);
  EXPECT_EQ(timing_bucket_index(2), 1u);
  EXPECT_EQ(timing_bucket_index(5), 2u);
  EXPECT_EQ(timing_bucket_index(6), 3u);
  EXPECT_EQ(timing_bucket_index(2000), kTimingBucketCount - 2);
  EXPECT_EQ(timing_bucket_index(2001), kTimingBucketCount - 1);
  EXPECT_EQ(timing_bucket_index(1000000), kTimingBucketCount - 1);
}

TEST(TimingSeriesTest, BucketCountsSumToTheSampleCount) {
  TimingSeries series;
  // One sample per bucket, walking just past each edge.
  for (uint32_t deviation : {0u, 2u, 5u, 10u, 20u, 50u, 100u, 200u, 500u, 1000u, 2000u, 5000u}) {
    series.note(deviation, deviation);
  }

  EXPECT_EQ(series.count(), 12u);
  EXPECT_EQ(series.bucketed_samples(), series.count())
      << "a sample that lands in no bucket is a sample the shape of the jitter is missing";

  for (size_t i = 0; i < kTimingBucketCount; ++i) {
    EXPECT_EQ(series.bucket(i), 1u) << "bucket " << i;
  }
}

// ---------------------------------------------------------------------------
// Bounded window and clamped values: what keeps the sums exact
// ---------------------------------------------------------------------------
TEST(TimingSeriesTest, TheWindowIsBoundedSoTheSumsCannotWrap) {
  TimingSeries series;

  // The worst case the arithmetic bound allows: a full window of the largest value the
  // series will record. n * sum(x^2) is about 4.2e18 against a uint64 maximum of 1.8e19, so
  // this must produce the right answer rather than wrapping.
  for (uint32_t i = 0; i < kTimingWindowSamples; ++i) {
    series.note(kTimingMaxValueUs, kTimingMaxValueUs);
  }

  EXPECT_TRUE(series.full());
  EXPECT_EQ(series.count(), kTimingWindowSamples);
  EXPECT_EQ(series.mean_us(), kTimingMaxValueUs);
  EXPECT_EQ(series.min_us(), kTimingMaxValueUs);
  EXPECT_EQ(series.max_us(), kTimingMaxValueUs);
  // Every sample is identical, so the variance is exactly zero. A wrapped sum would show
  // up here as a non-zero spread, which is the failure this bound exists to make
  // impossible.
  EXPECT_EQ(series.stddev_us(), 0u);

  // Further samples are ignored rather than overwriting the oldest: the statistics
  // describe one contiguous window, and a stopped consumer cannot silently lose the worst
  // case that has already been recorded.
  series.note(1, 1);
  EXPECT_EQ(series.count(), kTimingWindowSamples);
  EXPECT_EQ(series.min_us(), kTimingMaxValueUs);

  series.reset();
  EXPECT_FALSE(series.full());
  EXPECT_EQ(series.count(), 0u);
  EXPECT_EQ(series.bucketed_samples(), 0u);
}

TEST(TimingSeriesTest, AValueAboveTheCeilingIsClampedAndCounted) {
  TimingSeries series;
  series.note(kTimingMaxValueUs + 1, 0);

  EXPECT_EQ(series.max_us(), kTimingMaxValueUs);
  EXPECT_EQ(series.oversized_samples(), 1u);

  // And it says so on the line rather than folding the clamped sample into the mean as if
  // it had been measured: a mean over a clamped window is a lower bound, and a reader who
  // cannot see that would read it as the answer.
  char buffer[128];
  LineBuilder line(buffer, sizeof(buffer));
  series.format_stats(line, "latency");
  EXPECT_NE(std::string(buffer).find("over=1"), std::string::npos);
}

// ---------------------------------------------------------------------------
// The service: two streams, one report
// ---------------------------------------------------------------------------
TEST(TimingServiceTest, PeriodStatisticsAreOfTheRawPeriod) {
  TimingService timing(100'000);
  for (uint32_t i = 0; i < 5; ++i) {
    timing.note_period_us(100'000);
  }

  char buffer[TimingService::kLineBytes];
  ASSERT_GT(timing.format_line(0, buffer, sizeof(buffer)), 0u);

  // The raw period, not the deviation: a loop that runs consistently at 102.4 ms is a
  // different defect from a jittery one, and only the raw mean shows it. The nominal is on
  // the line because it is what the histogram below it is measured from.
  EXPECT_STREQ(buffer, "period us: n=5 min=100000 max=100000 mean=100000 sd=0 nom=100000");
}

TEST(TimingServiceTest, ThePeriodHistogramIsOfTheDeviationFromTheNominal) {
  TimingService timing(100'000);
  timing.note_period_us(100'000); // deviation 0
  timing.note_period_us(100'002); // deviation 2  -> bucket 1
  timing.note_period_us(99'998);  // deviation -2 -> bucket 1, sign does not matter here
  timing.note_period_us(105'000); // deviation 5000 -> the overflow bucket

  char first[TimingService::kLineBytes];
  char second[TimingService::kLineBytes];
  ASSERT_GT(timing.format_line(1, first, sizeof(first)), 0u);
  ASSERT_GT(timing.format_line(2, second, sizeof(second)), 0u);

  EXPECT_STREQ(first, "period hist us: <=1:1 <=2:2 <=5:0 <=10:0 <=20:0 <=50:0");
  EXPECT_STREQ(second, "period hist us: <=100:0 <=200:0 <=500:0 <=1000:0 <=2000:0 >2000:1");
}

TEST(TimingServiceTest, LatencyBucketsAreAbsoluteMicroseconds) {
  TimingService timing;

  // The ideal latency is zero, so a latency sample's deviation is the sample itself.
  timing.note_latency_us(3);
  timing.note_latency_us(15);
  timing.note_latency_us(5000);

  char first[TimingService::kLineBytes];
  char second[TimingService::kLineBytes];
  ASSERT_GT(timing.format_line(4, first, sizeof(first)), 0u);
  ASSERT_GT(timing.format_line(5, second, sizeof(second)), 0u);

  EXPECT_STREQ(first, "latency hist us: <=1:0 <=2:0 <=5:1 <=10:0 <=20:1 <=50:0");
  EXPECT_STREQ(second, "latency hist us: <=100:0 <=200:0 <=500:0 <=1000:0 <=2000:0 >2000:1");

  // The raw latency statistics are of the latency itself.
  EXPECT_EQ(timing.latency().min_us(), 3u);
  EXPECT_EQ(timing.latency().max_us(), 5000u);
  EXPECT_EQ(timing.latency().count(), 3u);
}

TEST(TimingServiceTest, TheTwoStreamsAreIndependent) {
  TimingService timing(10'000);
  timing.note_period_us(10'000);
  timing.note_period_us(10'000);
  timing.note_latency_us(20);

  EXPECT_EQ(timing.period().count(), 2u);
  EXPECT_EQ(timing.latency().count(), 1u);
  EXPECT_FALSE(timing.full());
}

TEST(TimingServiceTest, FullWhenEitherWindowIsComplete) {
  TimingService timing;
  for (uint32_t i = 0; i < kTimingWindowSamples; ++i) {
    timing.note_latency_us(10);
  }

  EXPECT_TRUE(timing.full());

  timing.reset();
  EXPECT_FALSE(timing.full());
  EXPECT_EQ(timing.latency().count(), 0u);
}

TEST(TimingServiceTest, ReportHasStableLinesForTheCommsTaskToStream) {
  TimingService timing(100'000);
  timing.note_period_us(100'050);
  timing.note_latency_us(12);

  char report[TimingService::kReportBytes];
  const size_t length = timing.report(report, sizeof(report));

  ASSERT_GT(length, 0u);
  const std::string text(report);

  // Six lines, five separators: a stable shape matters because the comms task streams
  // these one at a time and the host tool has to recognise them.
  size_t newlines = 0;
  for (char c : text) {
    if (c == '\n') {
      newlines++;
    }
  }
  EXPECT_EQ(newlines, TimingService::kReportLines - 1);
  EXPECT_NE(text.find("period us:"), std::string::npos);
  EXPECT_NE(text.find("period hist us:"), std::string::npos);
  EXPECT_NE(text.find("latency us:"), std::string::npos);
  EXPECT_NE(text.find("latency hist us:"), std::string::npos);

  // Every line fits the comms task's line buffer, so streaming the report needs no extra
  // storage beyond what a telemetry line already uses.
  for (size_t i = 0; i < TimingService::kReportLines; ++i) {
    char line[TimingService::kLineBytes];
    const size_t line_length = timing.format_line(i, line, sizeof(line));
    EXPECT_GT(line_length, 0u);
    EXPECT_LT(line_length, TimingService::kLineBytes);
  }

  // An out-of-range index formats nothing rather than reading past the report.
  char line[TimingService::kLineBytes];
  EXPECT_EQ(timing.format_line(TimingService::kReportLines, line, sizeof(line)), 0u);
}

TEST(TimingServiceTest, AReportThatDoesNotFitIsTruncatedAtALineBoundary) {
  TimingService timing;
  timing.note_period_us(100'000);

  // Room for the first line (64 bytes) and its terminator, and not for the second. The
  // capacity must cover the terminator as well as the text, which is why kReportBytes
  // leaves margin rather than being the exact sum of the line lengths.
  char small[96];
  const size_t length = timing.report(small, sizeof(small));

  ASSERT_GT(length, 0u);
  ASSERT_LT(length, sizeof(small));
  // One line fits and the next is dropped whole: half a histogram line would read as a
  // complete one with missing buckets.
  EXPECT_EQ(std::string(small).find('\n'), std::string::npos);
  EXPECT_NE(std::string(small).find("period us:"), std::string::npos);
}

} // namespace
} // namespace node
