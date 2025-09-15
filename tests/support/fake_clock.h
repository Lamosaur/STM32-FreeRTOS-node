/* fake_clock.h — host double for hal_clock_iface_t.
 *
 * A test-settable microsecond clock, so the measurements the node feeds into
 * app/timing.h can be asserted exactly instead of being "some number above zero". Both
 * NFR-1.8 (control-period jitter) and NFR-1.9 (ISR-to-task latency) are differences of two
 * readings, so a fake that only ever returned a constant would let a node that fed the
 * series zeros, or fed it the wrong timestamp twice, pass every test.
 *
 * `advance_us` is the whole interface a test needs: move the clock, drive the node, assert
 * on the series the node recorded. Deliberately no wall-clock timer — a test that slept for
 * a real millisecond would be slower, and flakier, than one that says how much time passed.
 *
 * Host-only; never compiled into the target image.
 */
#ifndef NODE_TEST_FAKE_CLOCK_H
#define NODE_TEST_FAKE_CLOCK_H

#include <cstdint>

#include "hal/clock.h"

namespace node::test {

class FakeClock {
 public:
  FakeClock() {
    iface_.ctx = this;
    iface_.now_us = &FakeClock::now_us_cb;
  }

  hal_clock_iface_t *iface() { return &iface_; }

  // --- scenario -----------------------------------------------------------

  void set_now_us(uint32_t us) { now_us_ = us; }
  void advance_us(uint32_t us) { now_us_ += us; }
  uint32_t now_us() const { return now_us_; }

  // The counter of reads, so a test can assert the node did not take a timestamp it never
  // used (a period measurement that reads the clock and then ignores it looks identical to
  // a correct one from the series alone).
  int reads() const { return reads_; }

 private:
  static uint32_t now_us_cb(void *ctx) {
    auto *self = static_cast<FakeClock *>(ctx);
    self->reads_++;
    return self->now_us_;
  }

  hal_clock_iface_t iface_{};
  uint32_t now_us_ = 0;
  int reads_ = 0;
};

} // namespace node::test

#endif // NODE_TEST_FAKE_CLOCK_H
