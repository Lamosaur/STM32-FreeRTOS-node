/* fake_watchdog.h — host double for hal_watchdog_iface_t.
 *
 * FR-0.4 is "an independent task feeds a watchdog and detects missed task heartbeats".
 * Proving it needs a watchdog that counts feeds rather than one that returns void and
 * is taken on trust: the test's real assertion is that feeding *stops*, which is only
 * observable if something counts.
 */
#ifndef NODE_TEST_FAKE_WATCHDOG_H
#define NODE_TEST_FAKE_WATCHDOG_H

#include <cstdint>

#include "hal/watchdog.h"

namespace node::test {

class FakeWatchdog {
 public:
  FakeWatchdog() {
    iface_.ctx = this;
    iface_.start = &FakeWatchdog::start_cb;
    iface_.feed = &FakeWatchdog::feed_cb;
  }

  hal_watchdog_iface_t *iface() { return &iface_; }

  // Convenience so a test does not have to repeat the ctx argument.
  bool start(uint32_t timeout_ms) { return iface_.start(iface_.ctx, timeout_ms); }
  void feed() { iface_.feed(iface_.ctx); }

  int starts() const { return starts_; }
  int feeds() const { return feeds_; }
  bool started() const { return started_; }
  uint32_t timeout_ms() const { return timeout_ms_; }
  void set_start_ok(bool ok) { start_ok_ = ok; }

 private:
  static bool start_cb(void *ctx, uint32_t timeout_ms) {
    auto *self = static_cast<FakeWatchdog *>(ctx);
    self->starts_++;
    if (!self->start_ok_) {
      return false;
    }
    self->started_ = true;
    self->timeout_ms_ = timeout_ms;
    return true;
  }

  static void feed_cb(void *ctx) { static_cast<FakeWatchdog *>(ctx)->feeds_++; }

  hal_watchdog_iface_t iface_{};
  int starts_ = 0;
  int feeds_ = 0;
  uint32_t timeout_ms_ = 0;
  bool started_ = false;
  bool start_ok_ = true;
};

} // namespace node::test

#endif // NODE_TEST_FAKE_WATCHDOG_H
