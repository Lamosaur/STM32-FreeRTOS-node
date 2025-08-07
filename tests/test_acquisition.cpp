/* Tests for the analog acquisition service (FR-1.1, FR-1.4, FR-1.12).
 *
 * The three claims worth testing here are the ones a plausible-looking wrong
 * implementation would also pass a weaker test for:
 *
 *   - the average is over exactly one block, so delivering half a block of one value and
 *     half of another must average to the midpoint rather than to either;
 *   - the block the service publishes is a *copy*, so mutating the caller's buffer after
 *     the callback returns (which is what the DMA engine does) must not change the answer;
 *   - the block callback allocates nothing and takes no lock, asserted structurally by
 *     replacing the global allocator for the duration of the call and by comparing every
 *     counter before and after rather than by reading the code.
 *
 * The conversion's arithmetic is checked against values worked out by hand in both probe
 * polarities, because a sign error there is a systematic error in every reading rather
 * than a visible glitch.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <vector>

#include "app/acquisition.h"
#include "app/board.h"
#include "support/fake_adc.h"
#include "support/fake_rtos.h"

namespace {

// --- global allocation watch -------------------------------------------------------
//
// Replacing the global allocator is the only way to assert "this path does not allocate"
// as a fact about the running code instead of a claim about the source. It is armed for
// the duration of one call and disarmed immediately, so GoogleTest's own allocations and
// the test's own vectors are not counted.
//
// The whole family is replaced, allocating and freeing, because the sanitizers pair
// allocations with deallocations: replacing only the allocating half leaves a malloc-backed
// allocation being released by the runtime's operator delete, which ASan reports as an
// alloc-dealloc mismatch before any test runs.
std::atomic<int> g_allocations{0};
std::atomic<bool> g_track_allocations{false};

void note_allocation() {
  if (g_track_allocations.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
}

class AllocationWatch {
 public:
  AllocationWatch() : before_(g_allocations.load(std::memory_order_relaxed)) {
    g_track_allocations.store(true, std::memory_order_relaxed);
  }
  ~AllocationWatch() { g_track_allocations.store(false, std::memory_order_relaxed); }

  AllocationWatch(const AllocationWatch &) = delete;
  AllocationWatch &operator=(const AllocationWatch &) = delete;

  int allocations() const {
    return g_allocations.load(std::memory_order_relaxed) - before_;
  }

 private:
  int before_;
};

} // namespace

void *operator new(std::size_t size) {
  note_allocation();
  void *memory = std::malloc(size == 0 ? 1 : size);
  if (memory == nullptr) {
    throw std::bad_alloc();
  }
  return memory;
}

void *operator new[](std::size_t size) {
  return ::operator new(size);
}

void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  note_allocation();
  return std::malloc(size == 0 ? 1 : size);
}

void *operator new[](std::size_t size, const std::nothrow_t &tag) noexcept {
  return ::operator new(size, tag);
}

void operator delete(void *memory) noexcept {
  std::free(memory);
}

void operator delete[](void *memory) noexcept {
  std::free(memory);
}

void operator delete(void *memory, std::size_t) noexcept {
  std::free(memory);
}

void operator delete[](void *memory, std::size_t) noexcept {
  std::free(memory);
}

void operator delete(void *memory, const std::nothrow_t &) noexcept {
  std::free(memory);
}

void operator delete[](void *memory, const std::nothrow_t &) noexcept {
  std::free(memory);
}

namespace node {
namespace {

using test::FakeAdc;
using test::FakeRtos;

// One full block of `value`.
std::vector<uint16_t> block(uint16_t value, size_t count = kAdcBlockSamples) {
  return std::vector<uint16_t>(count, value);
}

class AcquisitionTest : public ::testing::Test {
 protected:
  // The calibration the board defaults to: high counts when dry (3600) and low when wet
  // (1400), which is what a resistive probe does.
  static constexpr uint16_t kDry = kProbeCountsDry;
  static constexpr uint16_t kWet = kProbeCountsWet;

  FakeAdc adc;
  FakeRtos rtos;
};

// ---------------------------------------------------------------------------
// The conversion, as a pure function
// ---------------------------------------------------------------------------
TEST(AcquisitionConversionTest, HitsBothCalibrationEndpoints) {
  const Calibration calibration{}; // board.h defaults: 3600 dry, 1400 wet

  EXPECT_EQ(counts_to_moisture_permille(kProbeCountsDry, calibration), 0);
  EXPECT_EQ(counts_to_moisture_permille(kProbeCountsWet, calibration), 1000);
  // Halfway along the span, by hand: (3600 - 2500) = 1100 of a 2200 span is 500 permille.
  EXPECT_EQ(counts_to_moisture_permille(2500, calibration), 500);
}

TEST(AcquisitionConversionTest, ClampsOutsideTheCalibratedPoints) {
  const Calibration calibration{};

  // Drier than the dry endpoint, and wetter than the wet endpoint. A line through two
  // measured points says nothing beyond them, so these are clamped rather than
  // extrapolated: 4000 counts is about -181 permille by the formula, and "-181 permille of
  // soil moisture" is not a fact about soil.
  EXPECT_EQ(counts_to_moisture_permille(65535, calibration), 0);
  EXPECT_EQ(counts_to_moisture_permille(3601, calibration), 0);
  EXPECT_EQ(counts_to_moisture_permille(0, calibration), 1000);
  EXPECT_EQ(counts_to_moisture_permille(1399, calibration), 1000);
}

TEST(AcquisitionConversionTest, HandlesAnInvertedProbePolarity) {
  // A front end whose wet reading is *higher* than its dry one. The same four values must
  // come out mirrored, which they only do if the span's sign is carried through the
  // division instead of being assumed positive.
  const Calibration inverted{kProbeCountsWet, kProbeCountsDry}; // 1400 dry, 3600 wet
  ASSERT_TRUE(inverted.valid());

  EXPECT_EQ(counts_to_moisture_permille(1400, inverted), 0);
  EXPECT_EQ(counts_to_moisture_permille(3600, inverted), 1000);
  EXPECT_EQ(counts_to_moisture_permille(2500, inverted), 500);
  EXPECT_EQ(counts_to_moisture_permille(4000, inverted), 1000); // clamped, not negative
  EXPECT_EQ(counts_to_moisture_permille(1000, inverted), 0);
}

TEST(AcquisitionConversionTest, RoundsRatherThanTruncating) {
  const Calibration calibration{};

  // A midpoint case first: (3600 - 2499) = 1101 of a 2200 span, and 1101 * 1000 / 2200 is
  // 500.45, which rounds or truncates to 500 either way. Included because it is the value
  // a reader would check by hand.
  EXPECT_EQ(counts_to_moisture_permille(2499, calibration), 500);
  EXPECT_EQ(counts_to_moisture_permille(2501, calibration), 500);

  // And the case that separates the two: 101 counts below dry is 101 * 1000 / 2200 =
  // 45.909 permille, so the nearest integer is 46 and a truncating divide would report 45.
  // A one-permille bias is invisible in one sample and obvious in a trend, which is why it
  // is pinned here rather than left to the reader.
  EXPECT_EQ(counts_to_moisture_permille(3499, calibration), 46);
}

TEST(AcquisitionConversionTest, ADegenerateCalibrationHasNoAnswer) {
  // Equal endpoints: the slope is undefined. Zero is reported rather than something
  // invented, and valid() is what a caller checks before installing one.
  const Calibration degenerate{2000, 2000};
  EXPECT_FALSE(degenerate.valid());
  EXPECT_EQ(counts_to_moisture_permille(2000, degenerate), 0);
}

// ---------------------------------------------------------------------------
// start()
// ---------------------------------------------------------------------------
TEST_F(AcquisitionTest, StartRequestsTheBoardsRateAndBlockSize) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.valid());

  ASSERT_TRUE(service.start());

  EXPECT_EQ(adc.start_calls(), 1);
  EXPECT_TRUE(adc.started());
  EXPECT_EQ(adc.rate_hz(), kAdcSampleRateHz);
  EXPECT_EQ(adc.block_samples(), kAdcBlockSamples);
  EXPECT_TRUE(adc.has_callback()) << "no callback means no samples will ever arrive";
  EXPECT_TRUE(service.started());

  // The rate and the block size together are the publish cadence FR-1.1 implies, so the
  // two numbers reaching the HAL are checked against each other and not just against
  // themselves.
  EXPECT_EQ(kAdcSampleRateHz / static_cast<uint32_t>(kAdcBlockSamples), kAdcPublishRateHz);
  EXPECT_EQ(kAdcPublishRateHz, 10u);
}

TEST_F(AcquisitionTest, ARefusedStartIsReportedRatherThanSilentlyIdle) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  adc.set_start_ok(false);

  EXPECT_FALSE(service.start());

  EXPECT_EQ(service.stats().start_failures, 1u);
  EXPECT_FALSE(service.started());
  EXPECT_FALSE(adc.started());
}

TEST_F(AcquisitionTest, StopReachesTheHal) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  service.stop();

  EXPECT_EQ(adc.stop_calls(), 1);
  EXPECT_FALSE(service.started());
}

TEST_F(AcquisitionTest, StartingAgainDoesNotPublishAPreviousRunsBlocks) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());
  adc.deliver_constant(1234);

  ASSERT_TRUE(service.start());

  // The ring was reset with the restart, so nothing from the first run can be published as
  // if it were a fresh measurement.
  EXPECT_EQ(service.update(), 0u);
  EXPECT_FALSE(service.snapshot().valid);
}

// ---------------------------------------------------------------------------
// Averaging
// ---------------------------------------------------------------------------
TEST_F(AcquisitionTest, TheAverageIsOverExactlyOneBlockSize) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // Half the block at one value and half at another: the mean can only be 2000 if all 100
  // samples were summed. An implementation that averaged 50, or that dropped the tail,
  // would answer 4000 or 0.
  std::vector<uint16_t> mixed = block(4000);
  for (size_t i = kAdcBlockSamples / 2; i < kAdcBlockSamples; ++i) {
    mixed[i] = 0;
  }
  adc.deliver(mixed);

  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().raw_counts, 2000);
  EXPECT_EQ(service.stats().blocks_processed, 1u);
}

TEST_F(AcquisitionTest, TheAverageRoundsRatherThanTruncating) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // 99 samples of 100 and one of 151 sums to 10051, which is 100.51: rounded to 101.
  std::vector<uint16_t> nearly = block(100);
  nearly[kAdcBlockSamples - 1] = 151;
  adc.deliver(nearly);

  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().raw_counts, 101);
}

TEST_F(AcquisitionTest, TheCallbackCopiesTheBlockSoTheDmaBufferCanBeReused) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  std::vector<uint16_t> samples = block(4000);
  adc.deliver(samples);

  // This is what the hardware does the instant the callback returns: the DMA channel is
  // still pointed at the same buffer and starts filling the other half of it.
  std::fill(samples.begin(), samples.end(), static_cast<uint16_t>(0));

  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().raw_counts, 4000)
      << "the published value came from the reused hardware buffer, not from a copy";
}

TEST_F(AcquisitionTest, ABlockOfTheWrongLengthIsCountedAndNotAveraged) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  adc.deliver_constant(1500, kAdcBlockSamples / 2);

  EXPECT_EQ(service.stats().count_mismatches, 1u);
  EXPECT_EQ(service.update(), 0u) << "a short block is not a 100-sample average";
  EXPECT_EQ(service.stats().blocks_processed, 0u);
  EXPECT_EQ(service.snapshot().consecutive_failures, 1u);
  EXPECT_FALSE(service.snapshot().fresh());
}

// ---------------------------------------------------------------------------
// The overrun policy
// ---------------------------------------------------------------------------
TEST_F(AcquisitionTest, BlocksAreDroppedWhenTheConsumerStallsAndTheFreshestSurvives) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // Three blocks delivered without a single update(): the consumer is stalled, which is
  // what a long I2C transfer or a higher-priority interrupt storm would do.
  adc.deliver_constant(1000);
  adc.deliver_constant(2000);
  adc.deliver_constant(3000);

  EXPECT_EQ(service.stats().blocks_received, 3u);
  EXPECT_EQ(service.pending_blocks(), 3u) << "the producer never waits for the consumer";
  EXPECT_EQ(service.stats().blocks_dropped, 0u)
      << "the producer cannot know what the consumer has read; dropping is counted at drain";

  // Only kAdcBlockRingCapacity slots survive, and they are the newest ones.
  ASSERT_EQ(service.update(), 2u);

  const AcquisitionStats stats = service.stats();
  EXPECT_EQ(stats.blocks_dropped, 1u);
  EXPECT_EQ(stats.blocks_processed, 2u);
  EXPECT_EQ(stats.publishes, 2u);
  EXPECT_EQ(service.pending_blocks(), 0u);

  const AnalogSnapshot snapshot = service.snapshot();
  EXPECT_EQ(snapshot.raw_counts, 3000) << "the freshest value is more useful than a backlog";
  EXPECT_EQ(snapshot.sequence, 2u);
  EXPECT_EQ(snapshot.block_index, 2u) << "the newest block's index, not the overwritten one";
}

TEST_F(AcquisitionTest, AStallDoesNotGrowLatencyWithoutBound) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // Ten blocks while the consumer is away. The backlog must not be ten deep on return:
  // the ring holds two, eight are reported as dropped, and the value published afterwards
  // is the most recent one rather than the oldest.
  for (uint16_t value = 1000; value < 10000; value = static_cast<uint16_t>(value + 1000)) {
    adc.deliver_constant(value);
  }

  EXPECT_EQ(service.update(), 2u);

  const AcquisitionStats stats = service.stats();
  EXPECT_EQ(stats.blocks_received, 9u);
  EXPECT_EQ(stats.blocks_dropped, 7u);
  EXPECT_EQ(stats.blocks_processed, 2u);
  EXPECT_EQ(service.snapshot().raw_counts, 9000);
}

TEST_F(AcquisitionTest, TheBlockCallbackNeitherAllocatesNorTakesTheLock) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // The ring is already full, so this callback is the one with a decision to make. A
  // callback that allocated, that took the publish mutex, or that waited for room would
  // not return. The assertions below are structural rather than timed: a wall-clock bound
  // would be flaky on a loaded build machine, and a test that waits on a blocking callback
  // would hang CI instead of failing it.
  adc.deliver_constant(1000);
  adc.deliver_constant(2000);
  const AcquisitionStats before = service.stats();
  std::vector<uint16_t> samples = block(3000);

  int allocations = 0;
  {
    AllocationWatch watch;
    adc.deliver(samples);
    allocations = watch.allocations();
  }

  EXPECT_EQ(allocations, 0) << "the interrupt path allocated, which it must never do";

  const AcquisitionStats after = service.stats();
  EXPECT_EQ(after.blocks_received, before.blocks_received + 1) << "it must count the block";
  EXPECT_EQ(after.blocks_dropped, before.blocks_dropped)
      << "the ISR overwrites the oldest slot; only the consumer knows what that lost";
  EXPECT_EQ(after.blocks_processed, before.blocks_processed) << "no work beyond the copy";
  EXPECT_EQ(after.publishes, before.publishes) << "the ISR must not publish";
  EXPECT_TRUE(rtos.locks_balanced()) << "the callback left a mutex held";
  EXPECT_EQ(service.pending_blocks(), 3u) << "the callback returned with the ring overfull";
}

// ---------------------------------------------------------------------------
// The published snapshot
// ---------------------------------------------------------------------------
TEST_F(AcquisitionTest, ThePublishedValueTracksTheCalibrationEndpoints) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  adc.deliver_constant(kDry);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().moisture_permille, 0);

  adc.deliver_constant(kWet);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().moisture_permille, 1000);

  adc.deliver_constant(2500);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().moisture_permille, 500);
  EXPECT_EQ(service.snapshot().raw_counts, 2500);
  EXPECT_EQ(service.snapshot().sequence, 3u);
}

TEST_F(AcquisitionTest, ThePublishedValueClampsOutsideTheCalibratedPoints) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // Saturated soil and bone-dry air, both beyond the calibrated points.
  adc.deliver_constant(0);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().moisture_permille, 1000);

  adc.deliver_constant(65535);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_EQ(service.snapshot().moisture_permille, 0);
}

TEST_F(AcquisitionTest, RecalibrationAppliesToTheValueAlreadyPublished) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  // 3400 counts against the default calibration: 200 below dry of a 2200 span is 91
  // permille, worked out by hand.
  adc.deliver_constant(3400);
  ASSERT_EQ(service.update(), 1u);
  ASSERT_EQ(service.snapshot().moisture_permille, 91);
  ASSERT_EQ(service.snapshot().sequence, 1u);

  // FR-1.4: an inverted probe, fitted at run time.
  ASSERT_TRUE(service.set_calibration(Calibration{kWet, kDry}));

  const AnalogSnapshot recalibrated = service.snapshot();
  // 3400 is 2000 counts above the inverted dry endpoint of 1400, so 2000/2200 = 909.
  EXPECT_EQ(recalibrated.moisture_permille, 909)
      << "recalibration must not wait for the next block: the raw count is unchanged, so "
         "the new value is exact";
  EXPECT_EQ(recalibrated.raw_counts, 3400) << "the same measurement, reinterpreted";
  EXPECT_EQ(recalibrated.sequence, 1u) << "no new measurement, so no new sequence number";
  EXPECT_EQ(service.calibration().counts_dry, kWet);
}

TEST_F(AcquisitionTest, ACalibrationThatCannotProduceANumberIsRefused) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  EXPECT_FALSE(service.set_calibration(Calibration{2000, 2000}));

  // The previous calibration is still in force rather than one that would report zero for
  // every reading.
  EXPECT_EQ(service.calibration().counts_dry, kDry);
  EXPECT_EQ(service.calibration().counts_wet, kWet);
}

TEST_F(AcquisitionTest, AStoppedProducerDoesNotLookLikeSteadySoil) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  adc.deliver_constant(2000);
  ASSERT_EQ(service.update(), 1u);
  ASSERT_TRUE(service.snapshot().fresh());

  // The DMA channel dies. The last value is kept — it is still the best estimate — but it
  // must stop claiming to be current.
  ASSERT_EQ(service.update(), 0u);

  const AnalogSnapshot stale = service.snapshot();
  EXPECT_TRUE(stale.valid);
  EXPECT_EQ(stale.raw_counts, 2000);
  EXPECT_EQ(stale.sequence, 1u) << "no new measurement, so no new sequence number";
  EXPECT_EQ(stale.consecutive_failures, 1u);
  EXPECT_FALSE(stale.fresh()) << "a frozen number is indistinguishable from working soil";
  EXPECT_EQ(service.stats().empty_updates, 1u);

  // And recovers, with the failure count cleared.
  adc.deliver_constant(2500);
  ASSERT_EQ(service.update(), 1u);
  EXPECT_TRUE(service.snapshot().fresh());
  EXPECT_EQ(service.snapshot().consecutive_failures, 0u);
  EXPECT_EQ(service.snapshot().sequence, 2u);
}

TEST_F(AcquisitionTest, TheSequenceDoesNotAdvanceOnAFailedPublish) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());
  adc.deliver_constant(2000);

  rtos.fail_next_lock();
  EXPECT_EQ(service.update(), 0u) << "nothing was published, so update() reports nothing";

  EXPECT_EQ(service.snapshot().sequence, 0u) << "sequence counts successes, not attempts";
  EXPECT_EQ(service.snapshot().consecutive_failures, 0u)
      << "a lock failure is not recorded as a stale reading: it could not be recorded at all";
  EXPECT_EQ(service.stats().publish_lock_failures, 1u);

  // The block was still consumed from the ring: a lock failure must not stall the consumer
  // and turn into unbounded latency later.
  EXPECT_EQ(service.stats().blocks_processed, 1u);
  EXPECT_EQ(service.pending_blocks(), 0u);
}

TEST_F(AcquisitionTest, AnUnlockedSnapshotIsInvalidRatherThanTorn) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());
  adc.deliver_constant(2000);
  ASSERT_EQ(service.update(), 1u);
  ASSERT_EQ(service.snapshot().sequence, 1u);

  rtos.fail_next_lock();
  const AnalogSnapshot snapshot = service.snapshot();

  EXPECT_FALSE(snapshot.valid) << "a caller that sees invalid keeps its own previous value";
  EXPECT_EQ(snapshot.sequence, 0u);
}

TEST_F(AcquisitionTest, TheHardwareBlockCountIsReportedForComparison) {
  AcquisitionService service{adc.iface(), rtos.iface()};
  ASSERT_TRUE(service.start());

  adc.deliver_constant(2000);
  adc.deliver_constant(3000);

  // The HAL's own count and the service's callback count must agree: a difference is the
  // only way a lost callback would ever be visible.
  EXPECT_EQ(service.stats().blocks_received, adc.blocks());
  EXPECT_EQ(service.stats().hardware_blocks, adc.blocks());
  EXPECT_EQ(service.stats().last_counts, 0) << "nothing averaged yet";
}

} // namespace
} // namespace node
