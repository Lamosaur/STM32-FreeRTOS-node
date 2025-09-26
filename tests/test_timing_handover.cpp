/* Tests for the NFR-1.8/NFR-1.9 report handover (TimingReportHandover).
 *
 * The two tasks in the real system are the app task (writer) and the comms task
 * (reader), and what this class must guarantee is spelled out in timing.h: the writer
 * formats the series into the buffer and the reader takes it, in strict alternation, so
 * that the plain buffer is never touched by both at once. The tests below pin the
 * protocol, not the concurrency — the alternation is structural (the writer refuses to
 * overwrite an unread report, the reader only consumes after the writer's hand-off), so
 * what a test can drive deterministically is each transition and its effect on the bits
 * that carry the protocol.
 *
 * The full-window path matters beyond the protocol itself: a complete window must arm the
 * writer's reset only after the reader has actually read the report, which is the
 * difference between "the next window starts once this one was reported" and the old
 * behaviour — silently dropping the data when the series filled.
 */
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "app/timing.h"

namespace node {
namespace {

TEST(TimingHandoverTest, PublishMakesTheReportAvailableAndConsumeClearsIt) {
  TimingService timing;
  TimingReportHandover handover;

  timing.note_period_us(1'000'000);
  timing.note_period_us(1'000'004);

  ASSERT_TRUE(handover.publish(timing, /*window_complete=*/false));
  EXPECT_TRUE(handover.ready());
  EXPECT_FALSE(handover.window_complete());
  EXPECT_GT(handover.length(), 0u);

  // The handed-over text is the service's report, verbatim — a test can compare the
  // bytes because both are produced by the same formatting function.
  char expected[TimingReportHandover::kCapacity];
  const size_t n = timing.report(expected, sizeof(expected));
  ASSERT_LE(n, sizeof(expected));
  EXPECT_EQ(handover.length(), n);
  EXPECT_EQ(std::memcmp(handover.buffer(), expected, n), 0);

  handover.consume();
  EXPECT_FALSE(handover.ready());
}

TEST(TimingHandoverTest, PublishRefusesWhileAPublicationIsOutstanding) {
  TimingService timing;
  TimingReportHandover handover;

  timing.note_latency_us(5);
  ASSERT_TRUE(handover.publish(timing, false));
  const size_t first_length = handover.length();
  char first_report[TimingReportHandover::kCapacity];
  std::memcpy(first_report, handover.buffer(), first_length);

  // A second publish against an unread report must change nothing — the whole safety
  // property is that the writer never overwrites a buffer the reader has not consumed.
  timing.note_latency_us(9);
  EXPECT_FALSE(handover.publish(timing, false));
  EXPECT_TRUE(handover.ready());
  EXPECT_EQ(handover.length(), first_length);
  EXPECT_EQ(std::memcmp(handover.buffer(), first_report, first_length), 0);

  handover.consume();
  ASSERT_TRUE(handover.publish(timing, false)); // now the writer may publish again
  EXPECT_TRUE(handover.ready());
}

TEST(TimingHandoverTest, ACompleteWindowArmsTheResetAndAPartialOneDoesNot) {
  // A request's snapshot (partial window) must not arm the writer's reset: the series
  // keeps accumulating, so there is nothing to reset.
  TimingReportHandover partial;
  TimingService series;
  ASSERT_TRUE(partial.publish(series, /*window_complete=*/false));
  partial.consume();
  EXPECT_FALSE(partial.take_reset());

  // A publication that *was* a complete window arms the reset exactly once, and only the
  // writer's take_reset() clears it — a second take returns false.
  TimingReportHandover complete;
  TimingService full;
  ASSERT_TRUE(complete.publish(full, /*window_complete=*/true));
  complete.consume();
  EXPECT_TRUE(complete.take_reset());
  EXPECT_FALSE(complete.take_reset());
}

TEST(TimingHandoverTest, RequestIsServicedOnceAndThenCleared) {
  TimingReportHandover handover;

  EXPECT_FALSE(handover.take_request()); // nothing requested yet
  handover.request();
  EXPECT_TRUE(handover.take_request());
  EXPECT_FALSE(handover.take_request());
}

TEST(TimingHandoverTest, WriterServicesARequestWithTheSeriesAsItStands) {
  // The intended writer flow for a `timing` command: a request is serviced on the next
  // app tick with whatever has accumulated (possibly a partial window), and consuming a
  // partial report arms no reset, so the series keeps accumulating afterwards.
  TimingService timing;
  TimingReportHandover handover;
  timing.note_period_us(2'000'000); // a partial window: 1 of 1024 samples

  handover.request();
  ASSERT_TRUE(handover.take_request());
  ASSERT_TRUE(handover.publish(timing, /*window_complete=*/timing.full()));

  EXPECT_TRUE(handover.ready());
  EXPECT_FALSE(handover.window_complete()); // not full yet
  handover.consume();
  EXPECT_FALSE(handover.take_reset());
  EXPECT_EQ(timing.period().count(), 1u); // untouched by the round trip
}

TEST(TimingHandoverTest, FullCycleResetsTheSeriesOnlyAfterTheReportIsRead) {
  // The complete lifecycle: the window fills, the writer publishes it, the reader
  // consumes it, the writer resets — and the next window starts from zero. The reset
  // happens after the read (take_reset), not at the moment the window fills, which is
  // the whole point of the handover compared with the old count-and-drop.
  TimingService timing;
  TimingReportHandover handover;
  for (uint32_t i = 0; i < kTimingWindowSamples; ++i) {
    timing.note_period_us(1'000'000);
  }
  ASSERT_TRUE(timing.full());

  ASSERT_TRUE(handover.publish(timing, /*window_complete=*/true));
  EXPECT_TRUE(handover.window_complete());
  handover.consume();
  ASSERT_TRUE(handover.take_reset());
  timing.reset();

  EXPECT_FALSE(timing.full());
  EXPECT_EQ(timing.period().count(), 0u);
}

TEST(TimingHandoverTest, PendingReportBlocksANewWindowUntilConsumed) {
  // The writer-side gate in practice: while a full report is unread, publish() refuses,
  // so a window that completes without a reader simply waits rather than being dropped.
  // The report is still there when the structure is re-checked after a consume.
  TimingService timing;
  TimingReportHandover handover;
  for (uint32_t i = 0; i < kTimingWindowSamples; ++i) {
    timing.note_period_us(1'000'000);
  }
  ASSERT_TRUE(handover.publish(timing, true));
  EXPECT_TRUE(handover.ready());

  EXPECT_FALSE(handover.publish(timing, true)); // still waiting for a reader
  handover.consume();
  EXPECT_FALSE(handover.ready());
  EXPECT_TRUE(handover.take_reset());
}

} // namespace
} // namespace node