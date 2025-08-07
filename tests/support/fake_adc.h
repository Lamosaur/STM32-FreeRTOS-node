/* fake_adc.h — host double for hal_adc_iface_t.
 *
 * Two jobs, and the second one is the reason this is a fake rather than a stub:
 *
 *   1. Record what start() was asked for, so the rate and block size the application
 *      requests can be asserted against board.h rather than assumed. FR-1.1 is about the
 *      trigger rate being a hardware decision, so the number that reaches the HAL is worth
 *      pinning.
 *
 *   2. Let a test hand over blocks on demand — as many as it likes, in whatever order,
 *      including faster than the consumer drains. That is what makes the overrun path
 *      testable: the policy under test is "the producer keeps going and the consumer
 *      catches up on the freshest data", and a fake that could only deliver as fast as the
 *      consumer read it could never exercise it.
 *
 * There is deliberately no "poll the conversion" entry point here for the same reason
 * hal/adc.h has none: with no way to ask whether a conversion is ready, a test cannot be
 * written that depends on polling, so the interface's central claim (FR-1.1) cannot erode
 * by accident.
 *
 * Host-only; never compiled into the target image.
 */
#ifndef NODE_TEST_FAKE_ADC_H
#define NODE_TEST_FAKE_ADC_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hal/adc.h"

namespace node::test {

class FakeAdc {
 public:
  FakeAdc() {
    iface_.ctx = this;
    iface_.start = &FakeAdc::start_cb;
    iface_.stop = &FakeAdc::stop_cb;
    iface_.blocks = &FakeAdc::blocks_cb;
  }

  hal_adc_iface_t *iface() { return &iface_; }

  // --- scenario -----------------------------------------------------------

  // Refuse to start, standing in for a requested rate the peripheral clocks cannot
  // produce. Nothing is started and no callback will arrive, exactly as hal/adc.h
  // specifies.
  void set_start_ok(bool ok) { start_ok_ = ok; }

  // Hand one block to whoever registered a callback, as the DMA half-transfer interrupt
  // would. `count` is passed through unchanged so a test can also hand over a block of the
  // wrong length and see what the consumer does with it.
  //
  // The block index is the fake's own counter since the last successful start(), matching
  // the contract in hal/adc.h.
  void deliver(const uint16_t *samples, size_t count) {
    if (on_block_ == nullptr) {
      return;
    }
    blocks_++;
    on_block_(on_block_ctx_, samples, count, blocks_ - 1);
  }

  void deliver(const std::vector<uint16_t> &samples) {
    deliver(samples.data(), samples.size());
  }

  // A block of `count` copies of `value`. Defaults to the block size the application asked
  // for at start(), which is what a real implementation would deliver.
  void deliver_constant(uint16_t value, size_t count = 0) {
    const size_t n = count == 0 ? requested_block_samples_ : count;
    std::vector<uint16_t> samples(n, value);
    deliver(samples);
  }

  // --- observation --------------------------------------------------------

  int start_calls() const { return start_calls_; }
  int stop_calls() const { return stop_calls_; }
  bool started() const { return started_; }

  uint32_t rate_hz() const { return rate_hz_; }
  size_t block_samples() const { return requested_block_samples_; }
  bool has_callback() const { return on_block_ != nullptr; }

  // Blocks handed over since start(), the same value the HAL interface reports.
  uint32_t blocks() const { return blocks_; }

 private:
  static bool start_cb(void *ctx, uint32_t sample_rate_hz, size_t block_samples,
                       hal_adc_block_fn on_block, void *on_block_ctx) {
    auto *self = static_cast<FakeAdc *>(ctx);
    self->start_calls_++;
    self->rate_hz_ = sample_rate_hz;
    self->requested_block_samples_ = block_samples;
    // Recorded even when the start is refused: this is a log of the call, and a test that
    // asserts what was asked for should not have to make the call succeed first.
    self->on_block_ = on_block;
    self->on_block_ctx_ = on_block_ctx;
    if (!self->start_ok_) {
      return false;
    }
    self->started_ = true;
    self->blocks_ = 0;
    return true;
  }

  static void stop_cb(void *ctx) {
    auto *self = static_cast<FakeAdc *>(ctx);
    self->stop_calls_++;
    self->started_ = false;
  }

  static uint32_t blocks_cb(void *ctx) {
    return static_cast<FakeAdc *>(ctx)->blocks_;
  }

  hal_adc_iface_t iface_{};
  hal_adc_block_fn on_block_ = nullptr;
  void *on_block_ctx_ = nullptr;
  bool start_ok_ = true;
  bool started_ = false;
  uint32_t rate_hz_ = 0;
  size_t requested_block_samples_ = 0;
  uint32_t blocks_ = 0;
  int start_calls_ = 0;
  int stop_calls_ = 0;
};

} // namespace node::test

#endif // NODE_TEST_FAKE_ADC_H
