/* fake_actuator.h — host double for hal_actuator_iface_t.
 *
 * A recorder, not a stub, and the recording is the point: the two claims the Phase 1
 * wiring has to make about the gate are *ordering* claims. The gate must be off before
 * anything could energise it, and it must go off when a reading goes stale — and neither
 * of those is visible from "the pump was on at the end of the test".
 *
 * So every command is kept in order in `commands()`, as characters, and a test can assert
 * on the whole sequence. `init` is recorded too, because the interesting sequence for the
 * fail-safe question is not just on/off but *when* the first off() happened relative to
 * the first on().
 *
 * Host-only; never compiled into the target image.
 */
#ifndef NODE_TEST_FAKE_ACTUATOR_H
#define NODE_TEST_FAKE_ACTUATOR_H

#include <cstddef>
#include <string>

#include "hal/actuator.h"

namespace node::test {

class FakeActuator {
 public:
  // The characters commands() is built from, so a test does not have to spell them as
  // literals: 'i' init, 'n' on, 'f' off.
  static constexpr char kInit = 'i';
  static constexpr char kOn = 'n';
  static constexpr char kOff = 'f';

  FakeActuator() {
    iface_.ctx = this;
    iface_.init = &FakeActuator::init_cb;
    iface_.on = &FakeActuator::on_cb;
    iface_.off = &FakeActuator::off_cb;
  }

  hal_actuator_iface_t *iface() { return &iface_; }

  // --- scenario -----------------------------------------------------------

  // Stand in for a gate that could not be brought up safely. hal/actuator.h requires the
  // caller to treat that as "the pump is unavailable" rather than as "the pump is stopped",
  // and the node's response to it is worth a test.
  void set_init_ok(bool ok) { init_ok_ = ok; }

  // --- observation --------------------------------------------------------

  bool initialised() const { return initialised_; }
  int init_calls() const { return init_calls_; }

  // The gate's commanded state: what the last command said, not what a pin would read back
  // (hal/actuator.h has no readback, and for a reason — see its comment).
  bool on() const { return on_; }

  int on_calls() const { return on_calls_; }
  int off_calls() const { return off_calls_; }

  const std::string &commands() const { return commands_; }
  void clear_commands() { commands_.clear(); }

 private:
  static bool init_cb(void *ctx) {
    auto *self = static_cast<FakeActuator *>(ctx);
    self->init_calls_++;
    self->commands_.push_back(kInit);
    if (!self->init_ok_) {
      // A failed init does NOT leave the gate recorded as off, deliberately: the point of
      // the failure is that the driver could not establish the off state, and a fake that
      // claimed otherwise would let a caller that ignored the return value look correct.
      return false;
    }
    self->initialised_ = true;
    // init() must leave the gate off — hal/actuator.h's contract, and the ordering the
    // STM32 implementation exists to guarantee.
    self->commands_.push_back(kOff);
    self->off_calls_++;
    self->on_ = false;
    return true;
  }

  static void on_cb(void *ctx) {
    auto *self = static_cast<FakeActuator *>(ctx);
    self->on_calls_++;
    self->commands_.push_back(kOn);
    self->on_ = true;
  }

  static void off_cb(void *ctx) {
    auto *self = static_cast<FakeActuator *>(ctx);
    self->off_calls_++;
    self->commands_.push_back(kOff);
    self->on_ = false;
  }

  hal_actuator_iface_t iface_{};
  bool init_ok_ = true;
  bool initialised_ = false;
  bool on_ = false;
  int init_calls_ = 0;
  int on_calls_ = 0;
  int off_calls_ = 0;
  std::string commands_;
};

} // namespace node::test

#endif // NODE_TEST_FAKE_ACTUATOR_H
