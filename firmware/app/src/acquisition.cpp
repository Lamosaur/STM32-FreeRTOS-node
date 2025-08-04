#include "app/acquisition.h"

namespace node {
namespace {

// The ring capacity as the arithmetic type used by the index comparisons. Kept as a
// named constant so that every comparison in the ring works in one type and no
// conversion is implicit.
constexpr uint32_t kRingCapacity = static_cast<uint32_t>(kAdcBlockRingCapacity);

// The samples per block in the same type, for the averaging loop.
constexpr uint32_t kSamplesPerBlock = static_cast<uint32_t>(kAdcBlockSamples);

// The ring indexes blocks by counting them, so a zero-capacity ring would divide by zero
// rather than merely misbehave. board.h's value is 2; this makes a later edit that sets it
// to 0 a compile error instead of a fault on the first interrupt.
static_assert(kAdcBlockRingCapacity >= 1, "the block ring needs at least one slot");

// The denominator of the moisture conversion is at most 2 * 65535, and the numerator at
// most 65535 * 2000, so every intermediate in counts_to_moisture_permille stays well
// inside int32 — no 64-bit arithmetic is needed and none is used.
constexpr int32_t kPermilleFullScale = 1000;

// How long update() waits for the publish mutex. Not on any critical path: the only other
// holder is a comms task writing a calibration or reading a snapshot, both of which are
// microseconds. A caller that cannot get the lock in 20 ms has a scheduling problem that
// a longer timeout would only hide.
constexpr uint32_t kPublishLockTimeoutMs = 20;

} // namespace

int32_t counts_to_moisture_permille(uint16_t counts, const Calibration &calibration) {
  const int32_t dry = static_cast<int32_t>(calibration.counts_dry);
  const int32_t wet = static_cast<int32_t>(calibration.counts_wet);
  const int32_t span = dry - wet;

  // A degenerate calibration has no line to interpolate along. There is no honest answer,
  // so the reading is 0 rather than a guess, and set_calibration() refuses to install one
  // in the first place — this is the defensive path for a Calibration built by hand.
  if (span == 0) {
    return 0;
  }

  // Positive when the reading is drier than the dry endpoint, which is the direction that
  // must clamp to 0 — with either probe polarity, because both terms carry the same sign.
  const int32_t offset = dry - static_cast<int32_t>(counts);

  // Round to nearest rather than truncating: a truncating divide would bias every reading
  // towards the dry end, and a 1-permille systematic error is exactly the kind of thing
  // that is invisible in one sample and obvious in a trend.
  const int32_t permille = (offset * 2 * kPermilleFullScale + span) / (span * 2);

  if (permille < 0) {
    return 0;
  }
  if (permille > kPermilleFullScale) {
    return kPermilleFullScale;
  }
  return permille;
}

AcquisitionService::AcquisitionService(hal_adc_iface_t *adc, const rtos_iface_t *rtos,
                                       const Calibration &calibration)
    : adc_(adc), rtos_(rtos), mutex_(nullptr), calibration_(calibration) {
  if (rtos_ != nullptr && rtos_->mutex_create != nullptr) {
    mutex_ = rtos_->mutex_create(rtos_->ctx);
  }
}

bool AcquisitionService::start() {
  if (adc_ == nullptr || adc_->start == nullptr) {
    start_failures_++;
    return false;
  }

  // The ring is reset before the HAL is started, not after: the HAL's block counter is
  // "since start()" (hal/adc.h), so blocks a previous run left in the ring belong to that
  // run. Publishing one of them after this start would present a stale reading as a new
  // measurement, which is the failure mode the sequence number exists to expose.
  blocks_written_.store(0, std::memory_order_relaxed);
  blocks_read_.store(0, std::memory_order_relaxed);
  blocks_received_.store(0, std::memory_order_relaxed);
  count_mismatches_.store(0, std::memory_order_relaxed);

  const bool ok = adc_->start(adc_->ctx, kAdcSampleRateHz, kAdcBlockSamples,
                              &AcquisitionService::on_block, this);
  if (!ok) {
    start_failures_++;
    return false;
  }
  started_ = true;
  return true;
}

void AcquisitionService::stop() {
  if (adc_ != nullptr && adc_->stop != nullptr) {
    adc_->stop(adc_->ctx);
  }
  started_ = false;
}

void AcquisitionService::on_block(void *ctx, const uint16_t *samples, size_t count,
                                  uint32_t block_index) {
  auto *self = static_cast<AcquisitionService *>(ctx);
  if (self == nullptr || samples == nullptr) {
    return;
  }

  self->blocks_received_.fetch_add(1, std::memory_order_relaxed);

  const uint32_t written = self->blocks_written_.load(std::memory_order_relaxed);
  BlockSlot &slot = self->slots_[written % kRingCapacity];

  // The copy, and the reason there is a copy rather than a pointer handed to the consumer:
  // the DMA buffer is hardware-owned, so it may be overwritten the instant this returns.
  // Copying into application-owned storage is what makes a producer/consumer race
  // structurally impossible rather than merely unlikely — there is no window in which the
  // consumer holds a pointer the DMA engine can write through. It costs 200 bytes at
  // 10 blocks/s, which is 2 kB/s against a 72 MHz core with a 400-byte ring; the ring's
  // size is the real cost of this design decision, and it is bounded and paid at compile
  // time.
  const size_t usable = count < kAdcBlockSamples ? count : kAdcBlockSamples;
  for (size_t i = 0; i < usable; ++i) {
    slot.samples[i] = samples[i];
  }
  // The tail of a partial block is zeroed rather than left holding the previous block's
  // samples. The consumer discards such a block anyway, but a deterministic buffer is what
  // lets a test assert the contents instead of only the counter.
  for (size_t i = usable; i < kAdcBlockSamples; ++i) {
    slot.samples[i] = 0;
  }

  slot.block_index = block_index;
  slot.complete = (count == kAdcBlockSamples);
  if (!slot.complete) {
    self->count_mismatches_.fetch_add(1, std::memory_order_relaxed);
  }

  // Release: every store above must be visible to a consumer that observes this index.
  // There is no matching acquire on this side — the ISR never reads the ring's contents —
  // and no dropped-block accounting either, because the producer cannot know whether the
  // slot it just overwrote had been consumed. It overwrites the oldest slot either way,
  // which is the policy; update() is what counts what was lost.
  self->blocks_written_.store(written + 1, std::memory_order_release);
}

uint32_t AcquisitionService::update() {
  uint32_t published = 0;
  bool drained = false;
  bool lock_failure = false;

  for (;;) {
    const uint32_t written = blocks_written_.load(std::memory_order_acquire);
    uint32_t read = blocks_read_.load(std::memory_order_relaxed);
    if (read == written) {
      break;
    }

    // The producer overwrites the oldest slot in place, so once it is more than a ring
    // ahead of the consumer, the blocks at the old `read` are already gone. The freshest
    // kRingCapacity blocks are the ones that survive, and stepping over the rest is what
    // makes the consumer's view of the ring consistent again.
    //
    // Overwriting the oldest rather than refusing the new block is a deliberate choice: a
    // control loop wants the freshest value, and a queue that kept every block would turn
    // a transient stall into permanently increased latency followed by a burst of stale
    // samples. The cost is that the drop is silent to the producer, so it is counted here,
    // where it is known exactly.
    if (written - read > kRingCapacity) {
      blocks_dropped_ += (written - read) - kRingCapacity;
      read = written - kRingCapacity;
      blocks_read_.store(read, std::memory_order_release);
    }

    // A local copy, for the same reason the ISR copies: this task can be preempted
    // between here and the read of blocks_written_ below, and a slot the ISR has written
    // again is a mix of two blocks. 200 bytes of the caller's stack, which is why this
    // array is not a member.
    uint16_t block[kAdcBlockSamples];
    const BlockSlot &slot = slots_[read % kRingCapacity];
    const uint32_t block_index = slot.block_index;
    const bool complete = slot.complete;
    for (size_t i = 0; i < kAdcBlockSamples; ++i) {
      block[i] = slot.samples[i];
    }

    const uint32_t written_after = blocks_written_.load(std::memory_order_acquire);
    if (written_after - read > kRingCapacity) {
      // Lapped while copying: the producer has wrapped round to this slot, so the copy
      // above is partly the block we meant and partly its successor. Publishing its mean
      // would be publishing a value no converter ever produced, so the block is discarded
      // and the ring is resynchronised on the newest blocks. This needs the consumer to
      // be two block periods behind *during a 200-byte copy* — 200 ms of stall at 10 Hz —
      // which is why it is a correctness guard rather than a likely path.
      blocks_dropped_++;
      blocks_read_.store(written_after - kRingCapacity, std::memory_order_release);
      continue;
    }
    blocks_read_.store(read + 1, std::memory_order_release);
    drained = true;

    if (!complete) {
      // Already counted by the ISR. Not averaged: a mean over a short block would be a
      // different filter from the one the oversampling was designed around.
      continue;
    }

    // A sum of 100 uint16 values peaks at 6,553,500, so uint32 is comfortable; uint64
    // here would be 100 64-bit additions per block for no benefit. Rounded, for the same
    // reason as the conversion.
    uint32_t sum = 0;
    for (uint32_t i = 0; i < kSamplesPerBlock; ++i) {
      sum += static_cast<uint32_t>(block[i]);
    }
    const uint16_t mean =
        static_cast<uint16_t>((sum + kSamplesPerBlock / 2U) / kSamplesPerBlock);

    blocks_processed_++;
    last_counts_ = mean;
    if (publish(mean, block_index)) {
      published++;
    } else {
      lock_failure = true;
    }
  }

  if (published == 0) {
    if (!drained) {
      empty_updates_++;
    }
    if (!lock_failure) {
      note_failure();
    }
  }
  return published;
}

bool AcquisitionService::publish(uint16_t raw_counts, uint32_t block_index) {
  if (mutex_ == nullptr) {
    publish_lock_failures_++;
    return false;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, kPublishLockTimeoutMs, nullptr) != RTOS_OK) {
    publish_lock_failures_++;
    return false;
  }

  AnalogSnapshot next{};
  next.raw_counts = raw_counts;
  next.moisture_permille = counts_to_moisture_permille(raw_counts, calibration_);
  next.valid = true;
  next.taken_at_ms = rtos_->now_ms(rtos_->ctx);
  next.sequence = snapshot_.sequence + 1; // counts published values, not attempts
  next.block_index = block_index;
  next.consecutive_failures = 0;
  snapshot_ = next;

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  publishes_++;
  return true;
}

bool AcquisitionService::set_calibration(const Calibration &calibration) {
  if (!calibration.valid()) {
    return false;
  }
  if (mutex_ == nullptr) {
    publish_lock_failures_++;
    return false;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, kPublishLockTimeoutMs, nullptr) != RTOS_OK) {
    publish_lock_failures_++;
    return false;
  }

  calibration_ = calibration;

  // The raw count is calibration-independent, so the published value can be recomputed
  // exactly rather than waiting for the next block. One division inside the lock is
  // cheaper than a display that shows the old calibration for another 100 ms while a
  // technician watches the effect of their change.
  if (snapshot_.valid) {
    snapshot_.moisture_permille =
        counts_to_moisture_permille(snapshot_.raw_counts, calibration_);
  }

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return true;
}

Calibration AcquisitionService::calibration() const {
  if (mutex_ == nullptr) {
    return Calibration{};
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, kPublishLockTimeoutMs, nullptr) != RTOS_OK) {
    return Calibration{};
  }
  const Calibration copy = calibration_;
  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return copy;
}

AnalogSnapshot AcquisitionService::snapshot() const {
  if (mutex_ == nullptr) {
    return AnalogSnapshot{};
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, kPublishLockTimeoutMs, nullptr) != RTOS_OK) {
    // An invalid snapshot rather than a torn one: a caller that sees valid == false keeps
    // its previous value, which is the safe outcome.
    return AnalogSnapshot{};
  }
  const AnalogSnapshot copy = snapshot_;
  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return copy;
}

void AcquisitionService::note_failure() {
  // The last published value is kept — it is still the best estimate available — but the
  // failure count is advanced so a consumer can see that nothing new has arrived.
  if (mutex_ == nullptr) {
    return;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, kPublishLockTimeoutMs, nullptr) != RTOS_OK) {
    publish_lock_failures_++;
    return;
  }
  snapshot_.consecutive_failures++;
  rtos_->mutex_unlock(rtos_->ctx, mutex_);
}

AcquisitionStats AcquisitionService::stats() const {
  AcquisitionStats out;
  out.blocks_received = blocks_received_.load(std::memory_order_relaxed);
  out.blocks_processed = blocks_processed_;
  out.blocks_dropped = blocks_dropped_;
  out.count_mismatches = count_mismatches_.load(std::memory_order_relaxed);
  out.publishes = publishes_;
  out.empty_updates = empty_updates_;
  out.start_failures = start_failures_;
  out.publish_lock_failures = publish_lock_failures_;
  out.last_counts = last_counts_;
  if (adc_ != nullptr && adc_->blocks != nullptr) {
    out.hardware_blocks = adc_->blocks(adc_->ctx);
  }
  return out;
}

uint32_t AcquisitionService::pending_blocks() const {
  const uint32_t written = blocks_written_.load(std::memory_order_acquire);
  const uint32_t read = blocks_read_.load(std::memory_order_relaxed);
  // The raw difference, not clamped to the capacity: a difference greater than the
  // capacity is exactly the overrun condition, and hiding it here would make the one
  // number worth watching look healthy.
  return written - read;
}

} // namespace node
