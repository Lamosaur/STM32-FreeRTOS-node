/* Tests for the step-response recorder (FR-1.5).
 *
 * Four properties matter, and each of them is a way the recorder could look right while
 * being useless:
 *
 *   - arming must record nothing until the setpoint actually moves, because the point of
 *     arming early is that the step is commanded later;
 *   - the step must be detected at the threshold and not below it, so the boundary is a
 *     documented number rather than an off-by-one;
 *   - the bounds must hold exactly, so a capture cannot overrun the 128-record buffer that
 *     is a tenth of the part's RAM;
 *   - the CSV must round-trip: the columns and their order are the interface to
 *     host/step_response.py, and a reordered column would swap a command for a response
 *     and produce plausible, wrong figures.
 */
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "app/board.h"
#include "app/step_capture.h"

namespace node {
namespace {

// The header literal shared with host/step_response.py's COLUMNS and asserted again in
// tests/test_host_tools.py. It appears in three places on purpose: it is a wire format, and
// a wire format that is only written down once is one refactor away from changing silently.
constexpr char kCsvHeader[] = "t_ms,setpoint,measurement,output";

class StepCaptureTest : public ::testing::Test {
 protected:
  // The defaults: a 50-permille threshold and a 30 s window.
  StepCapture capture;
};

// --- a minimal CSV reader, standing in for the host parser -------------------------
//
// The host analyser is Python and cannot be called from here, so the round-trip is done
// against the same contract: split on commas, expect exactly four integer fields in the
// order the header declares, and compare them with the records the capture holds.
bool parse_integer(const std::string &text, long *out) {
  if (text.empty()) {
    return false;
  }
  size_t index = 0;
  bool negative = false;
  if (text[index] == '-') {
    negative = true;
    index++;
  }
  if (index >= text.size()) {
    return false;
  }
  long value = 0;
  for (; index < text.size(); ++index) {
    const char c = text[index];
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + (c - '0');
  }
  *out = negative ? -value : value;
  return true;
}

std::vector<std::string> split_commas(const std::string &line) {
  std::vector<std::string> fields;
  std::string current;
  for (char c : line) {
    if (c == ',') {
      fields.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  fields.push_back(current);
  return fields;
}

// ---------------------------------------------------------------------------
// Arming
// ---------------------------------------------------------------------------
TEST_F(StepCaptureTest, ArmingRecordsNothingUntilTheSetpointMoves) {
  capture.arm(300, 1000);
  EXPECT_TRUE(capture.armed());

  // Ten cycles of a control loop that has not been commanded yet. A recorder that started
  // at arm() would fill its buffer with a flat line before the step ever arrived.
  for (uint32_t i = 0; i < 10; ++i) {
    EXPECT_FALSE(capture.update(1000 + i * 100, 300, 100, 0));
  }

  EXPECT_EQ(capture.count(), 0u);
  EXPECT_EQ(capture.stats().captures, 0u);
  EXPECT_FALSE(capture.capturing());
  EXPECT_TRUE(capture.armed());
}

TEST_F(StepCaptureTest, TheStepIsDetectedAtTheThresholdAndNotBelowIt) {
  capture.arm(300, 0);

  // 49 counts of movement, one below the 50-count threshold: not a step.
  EXPECT_FALSE(capture.update(100, 349, 100, 0));
  EXPECT_EQ(capture.count(), 0u);
  EXPECT_TRUE(capture.armed());

  // Exactly the threshold: a step. The boundary is inclusive so that the test can pin it
  // from both sides.
  EXPECT_TRUE(capture.update(200, 350, 100, 0));
  EXPECT_TRUE(capture.capturing());

  // Two records, written at the same instant: the last pre-step state and the stepped
  // state. Both are needed — the first carries the baseline the host analysis measures
  // from, the second carries the setpoint the response is heading for.
  ASSERT_EQ(capture.count(), 2u);
  ASSERT_EQ(capture.stats().captures, 1u);

  EXPECT_EQ(capture.record(0).t_ms, 200u);
  EXPECT_EQ(capture.record(0).setpoint, 300);
  EXPECT_EQ(capture.record(0).measurement, 100);
  EXPECT_EQ(capture.record(1).t_ms, 200u);
  EXPECT_EQ(capture.record(1).setpoint, 350);
  EXPECT_EQ(capture.record(1).measurement, 100);
}

TEST_F(StepCaptureTest, ADownwardStepIsDetectedToo) {
  capture.arm(800, 0);

  EXPECT_FALSE(capture.update(50, 751, 700, 0));
  EXPECT_TRUE(capture.update(100, 750, 700, 0));

  ASSERT_EQ(capture.count(), 2u);
  EXPECT_EQ(capture.record(0).setpoint, 800);
  EXPECT_EQ(capture.record(1).setpoint, 750);
}

TEST_F(StepCaptureTest, RecordsCarryTimeSinceArming) {
  capture.arm(300, 1000);
  ASSERT_TRUE(capture.update(2000, 400, 120, 7)); // the step, 1000 ms after arming

  ASSERT_TRUE(capture.update(2100, 400, 250, -3));

  const StepRecord &sample = capture.record(2);
  // Time since *arming*, not since the step: the arm-to-step delay is information, and one
  // subtraction on the host recovers the other reference.
  EXPECT_EQ(sample.t_ms, 1100u);
  EXPECT_EQ(sample.setpoint, 400);
  EXPECT_EQ(sample.measurement, 250);
  EXPECT_EQ(sample.output, -3);
  EXPECT_EQ(capture.record(0).t_ms, 1000u);
}

TEST_F(StepCaptureTest, ReArmingDiscardsThePreviousCapture) {
  capture.arm(300, 0);
  ASSERT_TRUE(capture.update(100, 400, 10, 0));
  ASSERT_EQ(capture.count(), 2u);

  capture.arm(500, 1000);

  // Two captures in one CSV would have two baselines and two steps in it, and every figure
  // derived from it would be meaningless.
  EXPECT_EQ(capture.count(), 0u);
  EXPECT_TRUE(capture.armed());
  EXPECT_EQ(capture.armed_setpoint(), 500);
  EXPECT_EQ(capture.stats().arms, 2u);
}

TEST_F(StepCaptureTest, DisarmStopsRecordingAndKeepsTheRecords) {
  capture.arm(0, 0);
  ASSERT_TRUE(capture.update(10, 100, 0, 0));
  ASSERT_EQ(capture.count(), 2u);

  capture.disarm();

  EXPECT_TRUE(capture.idle());
  EXPECT_FALSE(capture.update(20, 100, 0, 0));
  EXPECT_EQ(capture.count(), 2u) << "the records taken are still exportable";
  EXPECT_EQ(capture.stats().idle_updates, 1u);
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------
TEST_F(StepCaptureTest, TheBufferBoundHoldsExactly) {
  capture.arm(0, 0);
  ASSERT_TRUE(capture.update(10, 100, 0, 0)); // the step: two records

  // Drive the capture until it stops on its own. Bounded so that a broken state machine
  // fails the test instead of hanging it.
  uint32_t now = 20;
  for (int i = 0; i < 1000 && capture.capturing(); ++i) {
    capture.update(now, 100, 1, 0);
    now += 10;
  }

  EXPECT_EQ(capture.count(), StepCapture::kCapacity) << "the buffer is exactly full";
  EXPECT_TRUE(capture.complete());
  EXPECT_EQ(capture.stats().full_stops, 1u);
  EXPECT_EQ(capture.stats().timeout_stops, 0u);

  // Once complete, further cycles are ignored and counted.
  EXPECT_FALSE(capture.update(now, 100, 1, 0));
  EXPECT_EQ(capture.count(), StepCapture::kCapacity);
  EXPECT_EQ(capture.stats().idle_updates, 1u);

  // Nothing was overwritten: the first records, which hold the step edge and the
  // transient, are still the first records. A ring would have kept the settled tail and
  // discarded exactly the part the analysis needs.
  EXPECT_EQ(capture.record(0).setpoint, 0);
  EXPECT_EQ(capture.record(1).setpoint, 100);
  EXPECT_EQ(capture.record(StepCapture::kCapacity - 1).setpoint, 100);
}

TEST_F(StepCaptureTest, TheDurationBoundStopsACapture) {
  // A 500 ms window, reached long before the 128-record buffer at a 100 ms period.
  StepCapture timed(50, 500);
  timed.arm(0, 0);
  ASSERT_TRUE(timed.update(100, 100, 0, 0)); // the step at t = 100 ms
  ASSERT_EQ(timed.count(), 2u);

  for (uint32_t now = 200; now <= 500; now += 100) {
    ASSERT_TRUE(timed.update(now, 100, 0, 0));
  }
  ASSERT_EQ(timed.count(), 6u);

  // 600 - 100 = 500 ms of window, which is where it stops: the sample at exactly the bound
  // is the first one not recorded.
  EXPECT_FALSE(timed.update(600, 100, 0, 0));

  EXPECT_TRUE(timed.complete());
  EXPECT_EQ(timed.stats().timeout_stops, 1u);
  EXPECT_EQ(timed.stats().full_stops, 0u);
  EXPECT_EQ(timed.count(), 6u);
}

// ---------------------------------------------------------------------------
// The CSV contract
// ---------------------------------------------------------------------------
TEST_F(StepCaptureTest, TheHeaderIsTheWireFormatTheHostExpects) {
  char buffer[64];
  ASSERT_GT(StepCapture::format_header(buffer, sizeof(buffer)), 0u);
  EXPECT_STREQ(buffer, kCsvHeader);
}

TEST_F(StepCaptureTest, RecordsRoundTripThroughTheHostParser) {
  capture.arm(300, 1000);
  ASSERT_TRUE(capture.update(2000, 400, 150, 7));

  // An integer exponential approach, so the rows are a plausible response with no floating
  // point in the test at all.
  int32_t measurement = 150;
  int32_t output = 7;
  uint32_t now = 2100;
  for (int i = 0; i < 40 && capture.capturing(); ++i) {
    measurement += (500 - measurement) / 4;
    output += (i % 3) - 1; // signed on purpose: the CSV must keep the sign
    capture.update(now, 400, measurement, output);
    now += 100;
  }
  ASSERT_GT(capture.count(), 3u);

  // The header, then one row per record, parsed back the way the host does.
  char line[64];
  ASSERT_GT(StepCapture::format_header(line, sizeof(line)), 0u);
  EXPECT_EQ(split_commas(line), (std::vector<std::string>{"t_ms", "setpoint", "measurement", "output"}));

  // Every record, not just the first: a format that only works for index 0 would pass a
  // weaker test.
  for (size_t i = 0; i < capture.count(); ++i) {
    ASSERT_GT(capture.format_record(i, line, sizeof(line)), 0u) << "record " << i;
    const std::vector<std::string> fields = split_commas(line);
    ASSERT_EQ(fields.size(), 4u) << "record " << i;

    long values[4] = {0, 0, 0, 0};
    for (size_t f = 0; f < fields.size(); ++f) {
      ASSERT_TRUE(parse_integer(fields[f], &values[f])) << "field " << f << " of record " << i;
    }

    const StepRecord &sample = capture.record(i);
    EXPECT_EQ(values[0], static_cast<long>(sample.t_ms)) << "column t_ms, record " << i;
    EXPECT_EQ(values[1], static_cast<long>(sample.setpoint)) << "column setpoint, record " << i;
    EXPECT_EQ(values[2], static_cast<long>(sample.measurement)) << "column measurement, record " << i;
    EXPECT_EQ(values[3], static_cast<long>(sample.output)) << "column output, record " << i;
  }
}

TEST_F(StepCaptureTest, ARecordFormatsToItsExactBytes) {
  capture.arm(300, 1000);
  ASSERT_TRUE(capture.update(2000, 400, 150, 7));
  ASSERT_TRUE(capture.update(2100, 400, 250, -3));

  char buffer[64];
  ASSERT_GT(capture.format_record(2, buffer, sizeof(buffer)), 0u);

  // Built by LineBuilder's integer formatting only — no printf, no locale, no
  // floating-point formatting — which is what makes the bytes assertable here and keeps
  // newlib's formatter out of a 64 KB image.
  EXPECT_STREQ(buffer, "1100,400,250,-3");
}

TEST_F(StepCaptureTest, AnOutOfRangeRecordFormatsNothing) {
  char buffer[64];
  buffer[0] = 'x';

  EXPECT_EQ(capture.format_record(0, buffer, sizeof(buffer)), 0u);

  // Terminated rather than left holding the previous row: a caller that ignored the return
  // value would otherwise stream a stale line as if it were the next record.
  EXPECT_EQ(buffer[0], '\0');
  EXPECT_EQ(capture.format_record(0, nullptr, sizeof(buffer)), 0u);
}

TEST_F(StepCaptureTest, AnOutOfRangeRecordReadsAsAnEmptyRowRatherThanPastTheEnd) {
  const StepRecord &missing = capture.record(0);

  EXPECT_EQ(missing.t_ms, 0u);
  EXPECT_EQ(missing.setpoint, 0);
}

} // namespace
} // namespace node
