/* acquisition.h — the analog acquisition service: completed DMA blocks in, published
 * soil-moisture readings out (FR-1.1, FR-1.2, FR-1.4).
 *
 * The shape of this service follows SensorService (app/sensor.h) on purpose: a snapshot
 * guarded by one RTOS mutex, a stats struct of plain counters, no heap, and a `update()`
 * that a host test drives one cycle at a time. The analog path differs in exactly one
 * place — its producer is an interrupt rather than the calling task — and that difference
 * is where all the care in this file goes.
 *
 * The producer/consumer problem, stated once here because the rest of the class is shaped
 * by it:
 *
 *   - The producer runs in the DMA interrupt and must not block, allocate, or take the
 *     mutex. It therefore cannot publish the snapshot, and it cannot wait for room.
 *   - The hardware buffer is only valid until the callback returns, so the producer must
 *     copy each block into memory the application owns.
 *   - The consumer publishes at 10 Hz; the producer delivers at 10 Hz; but a stall of the
 *     consumer's task (a higher-priority interrupt storm, one long I2C transfer) must not
 *     grow latency without bound.
 *
 * The result is a fixed ring that the producer overwrites in place and a consumer that
 * always resynchronises to the freshest blocks, which is why there is no queue here and
 * no allocation: kAdcBlockRingCapacity slots are all this path will ever need, and the
 * overrun counter is the honest report of anything the design could not keep up with.
 */
#ifndef APP_ACQUISITION_H
#define APP_ACQUISITION_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "hal/adc.h"
#include "rtos/rtos.h"

namespace node {

// Two-point probe calibration: the averaged counts measured in air-dry soil and in
// saturated soil, both runtime-settable (FR-1.4) so recalibrating does not mean
// reflashing. Defaults are the kProbeCounts* placeholders in app/board.h.
//
// Deliberately not a slope and offset: a probe's transfer function is not known well
// enough to justify an affine model with a physical meaning, whereas two measured points
// and the line between them is a model whose only claim is the two points. Nothing here
// is derived from physics, so nothing here is wrong when the physics turns out different —
// it is simply re-measured.
struct Calibration {
  uint16_t counts_dry = kProbeCountsDry;
  uint16_t counts_wet = kProbeCountsWet;

  // A resistive probe reads high counts when dry and low when wet (board.h's defaults);
  // a capacitive or voltage-output front end can be the other way round. Both are
  // handled, so this predicate is about a genuinely degenerate calibration only.
  bool valid() const { return counts_dry != counts_wet; }
};

// Averaged raw counts -> soil moisture in permille of the calibration span: 0 is the
// air-dry endpoint, 1000 the saturated one.
//
// A free function rather than a member, so the conversion can be tested directly against
// values worked out by hand, without a service, a HAL or an RTOS — the same reasoning as
// Aht20::raw_to_milli_celsius in app/sensor.h.
//
// Polarity: the span is computed as counts_dry - counts_wet, so a probe whose wet reading
// is *higher* than its dry one (an inverted front end) produces a negative span and the
// same answer, mirrored. Getting this wrong in either direction is a systematic error of
// the whole reading rather than a visible glitch, which is why it is tested both ways.
//
// Values outside the two calibrated points are clamped, not extrapolated: a line through
// two measured points says nothing about what is beyond them, and "1500 permille" of soil
// moisture is not a fact about soil.
int32_t counts_to_moisture_permille(uint16_t counts, const Calibration &calibration);

// One published analog reading. `valid` is false until a block has been processed, in
// which case the other fields are not meaningful.
struct AnalogSnapshot {
  uint16_t raw_counts = 0;       // the averaged raw count this value came from
  int32_t moisture_permille = 0; // 0..1000, clamped to the calibration endpoints
  bool valid = false;
  uint32_t taken_at_ms = 0;   // task clock, at the moment of publication
  uint32_t sequence = 0;      // increments on every successful publish
  uint32_t block_index = 0;   // the producer's numbering of that block (see hal/adc.h)

  // Failed publish cycles since the last success: an update() that drained nothing, or
  // that drained only blocks it could not average.
  //
  // This exists for the same reason SensorSnapshot carries it: once a reading has been
  // published, a producer that stops leaves the last good value in place, and `valid`
  // stays true. Without this counter a consumer cannot tell a live reading from one that
  // has been frozen since the DMA channel died, and a steady number on a display is
  // indistinguishable from a working node. For the analog path the failure is not
  // hypothetical: a stopped timer, a busy DMA channel or a task that never runs again all
  // look exactly like stable soil.
  //
  // A publish that could not take the lock is deliberately not counted here — it could not
  // be recorded under that lock either — and is reported as publish_lock_failures.
  uint32_t consecutive_failures = 0;

  bool fresh() const { return valid && consecutive_failures == 0; }
};

struct AcquisitionStats {
  uint32_t blocks_received = 0;  // callbacks entered
  uint32_t blocks_processed = 0; // blocks averaged into a published value
  uint32_t blocks_dropped = 0;   // blocks the ring could not hold; see update()
  uint32_t count_mismatches = 0; // blocks whose count was not kAdcBlockSamples
  uint32_t publishes = 0;        // snapshot updates
  uint32_t empty_updates = 0;    // update() calls that found nothing to drain
  uint32_t start_failures = 0;   // HAL rejected the requested rate or configuration
  uint32_t publish_lock_failures = 0;
  uint32_t hardware_blocks = 0; // the HAL's own count at the last stats() call
  uint16_t last_counts = 0;     // most recent averaged raw count, before calibration
};

// The sensor task's analog counterpart. `update()` is the task side; everything else is
// reachable from a task or, for the block callback, from interrupt context.
class AcquisitionService {
 public:
  // `adc` and `rtos` are borrowed and must outlive this object. The mutex is created
  // once, here, and deliberately never destroyed: a 72 h unattended run (NFR-0.7) must
  // not fragment the FreeRTOS heap.
  AcquisitionService(hal_adc_iface_t *adc, const rtos_iface_t *rtos,
                     const Calibration &calibration = Calibration{});

  AcquisitionService(const AcquisitionService &) = delete;
  AcquisitionService &operator=(const AcquisitionService &) = delete;

  // True if the mutex was created. When false, every update() fails to publish rather
  // than publishing unsynchronised.
  bool valid() const { return mutex_ != nullptr; }

  // Start free-running acquisition at the board's rate and block size (FR-1.1) and
  // reset the ring. Returns false — with start_failures counted — if the HAL refused the
  // configuration, which is the only case in which no data will ever arrive.
  bool start();
  void stop();

  // Whether start() succeeded and stop() has not been called since.
  bool started() const { return started_; }

  // Drain every pending block, average each one, convert it, and publish.
  //
  // Returns the number of values published (0 when nothing was drained, or when the
  // publish lock could not be taken). Draining is a loop rather than one block per call
  // so that a consumer which was late catches up in one wake-up instead of publishing a
  // backlog one period at a time; the published snapshot is the last of them, which is
  // the value a control loop wants.
  //
  // Overrun policy, in one line: blocks the ring could not hold are dropped oldest-first
  // and counted in blocks_dropped, and what survives is always the freshest. The reasoning
  // and the cost are in acquisition.cpp, next to the code that does it.
  uint32_t update();

  // A consistent copy of the latest reading, safe to call from any task.
  AnalogSnapshot snapshot() const;

  // Recalibration at run time (FR-1.4). Recomputes the published moisture from the last
  // averaged raw count, so the new calibration is visible immediately rather than one
  // publish period later. The sequence is deliberately not advanced: no new measurement
  // was taken, and a consumer counting samples must not see a phantom one.
  //
  // Returns false if the calibration is degenerate (equal endpoints; there is no line to
  // interpolate along) or if the lock could not be taken — leaving the previous
  // calibration in force rather than installing one that cannot produce a number.
  bool set_calibration(const Calibration &calibration);

  Calibration calibration() const;

  // Plain-value snapshot, assembled rather than copied: the ISR-side counters are atomic
  // and the task-side ones are only written by update(), so there is no single struct to
  // read consistently (same reasoning as I2cArbiter::stats()).
  AcquisitionStats stats() const;

  // Blocks currently waiting to be drained. For diagnostics and tests; the consumer does
  // not use it to decide anything, because that would be the polling this design avoids.
  uint32_t pending_blocks() const;

  // The block callback. Public so a test can invoke it directly, which is how the
  // "no allocation and no blocking in interrupt context" property is asserted without a
  // real interrupt. It is the same function the HAL is handed by start().
  static void on_block(void *ctx, const uint16_t *samples, size_t count,
                       uint32_t block_index);

 private:
  // One slot of the ring. `complete` is carried per block rather than assumed: a HAL that
  // hands over a short block is reporting something the consumer must not average as if
  // it were a full 100-sample window, because that would silently change the averaging
  // window and with it the noise rejection the oversampling is there for.
  struct BlockSlot {
    uint16_t samples[kAdcBlockSamples];
    uint32_t block_index;
    bool complete;
  };

  // Publish one averaged block, under the mutex. Returns false if the lock could not be
  // taken. Called with the ring already advanced, so a lock failure loses the value but
  // not the block.
  bool publish(uint16_t raw_counts, uint32_t block_index);

  // Publish a failed cycle: the last value is kept, the staleness counter advances.
  void note_failure();

  // The ring. Written by the ISR through on_block, read by update().
  //
  // Two indices, each with exactly one writer, which is what makes this lock-free:
  // blocks_written_ is the producer's, blocks_read_ is the consumer's, and neither is
  // ever written by the other side. The samples themselves are copied by the producer
  // before it releases blocks_written_, and read by the consumer after it acquires
  // blocks_written_, so a slot's contents are visible whenever the index says they are.
  BlockSlot slots_[kAdcBlockRingCapacity]{};
  std::atomic<uint32_t> blocks_written_{0};
  std::atomic<uint32_t> blocks_read_{0};

  // Written from interrupt context; atomic so that a task reading them cannot observe a
  // torn value, and so a future move of a counter across the boundary cannot introduce a
  // race silently.
  std::atomic<uint32_t> blocks_received_{0};
  std::atomic<uint32_t> count_mismatches_{0};

  hal_adc_iface_t *adc_;
  const rtos_iface_t *rtos_;
  rtos_mutex_t *mutex_;

  // Guarded by mutex_.
  AnalogSnapshot snapshot_{};
  Calibration calibration_{};

  // Task-side counters: written only by update() and set_calibration().
  uint32_t blocks_processed_ = 0;
  uint32_t blocks_dropped_ = 0;
  uint32_t publishes_ = 0;
  uint32_t empty_updates_ = 0;
  uint32_t start_failures_ = 0;
  uint32_t publish_lock_failures_ = 0;
  uint16_t last_counts_ = 0;
  bool started_ = false;
};

} // namespace node

#endif // APP_ACQUISITION_H
