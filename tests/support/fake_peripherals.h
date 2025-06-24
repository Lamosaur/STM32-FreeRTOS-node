/* fake_peripherals.h — host doubles for the small HAL interfaces.
 *
 * GPIO, UART and the system interface are all "record what was asked and answer what
 * the test configured", so they live together. Each one is deliberately a recorder
 * rather than a stub: the assertions these support are about *what the node did*, such
 * as whether the status LED was driven or which bytes reached the host.
 */
#ifndef NODE_TEST_FAKE_PERIPHERALS_H
#define NODE_TEST_FAKE_PERIPHERALS_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hal/gpio.h"
#include "hal/system.h"
#include "hal/uart.h"

namespace node::test {

class FakeGpio {
 public:
  FakeGpio() {
    iface_.ctx = this;
    iface_.configure_output = &FakeGpio::configure_output_cb;
    iface_.configure_input = &FakeGpio::configure_input_cb;
    iface_.write = &FakeGpio::write_cb;
    iface_.read = &FakeGpio::read_cb;
    iface_.toggle = &FakeGpio::toggle_cb;
  }

  hal_gpio_iface_t *iface() { return &iface_; }

  int outputs_configured() const { return outputs_configured_; }
  int writes() const { return writes_; }
  int toggles() const { return toggles_; }
  bool last_level() const { return last_level_; }
  uint32_t last_toggled_pin() const { return last_toggled_pin_; }
  bool configured(hal_pin_t pin) const {
    for (hal_pin_t p : configured_pins_) {
      if (p == pin) {
        return true;
      }
    }
    return false;
  }

 private:
  static bool configure_output_cb(void *ctx, hal_pin_t pin, hal_gpio_out_mode_t /*mode*/) {
    auto *self = static_cast<FakeGpio *>(ctx);
    self->outputs_configured_++;
    self->configured_pins_.push_back(pin);
    return true;
  }

  static bool configure_input_cb(void *ctx, hal_pin_t pin, bool /*pull_up*/) {
    auto *self = static_cast<FakeGpio *>(ctx);
    self->configured_pins_.push_back(pin);
    return true;
  }

  static bool write_cb(void *ctx, hal_pin_t /*pin*/, bool level) {
    auto *self = static_cast<FakeGpio *>(ctx);
    self->writes_++;
    self->last_level_ = level;
    return true;
  }

  static bool read_cb(void * /*ctx*/, hal_pin_t /*pin*/, bool *level) {
    *level = false;
    return true;
  }

  static bool toggle_cb(void *ctx, hal_pin_t pin) {
    auto *self = static_cast<FakeGpio *>(ctx);
    self->toggles_++;
    self->last_toggled_pin_ = pin;
    return true;
  }

  hal_gpio_iface_t iface_{};
  std::vector<hal_pin_t> configured_pins_;
  int outputs_configured_ = 0;
  int writes_ = 0;
  int toggles_ = 0;
  bool last_level_ = false;
  uint32_t last_toggled_pin_ = 0;
};

// A UART double that captures everything transmitted and replays whatever a test has
// queued as received. The captured bytes are what a host would actually see, so a test
// can assert on the exact protocol output rather than on a proxy for it.
class FakeUart {
 public:
  FakeUart() {
    iface_.ctx = this;
    iface_.init = &FakeUart::init_cb;
    iface_.write = &FakeUart::write_cb;
    iface_.read = &FakeUart::read_cb;
    iface_.rx_available = &FakeUart::rx_available_cb;
  }

  hal_uart_iface_t *iface() { return &iface_; }

  // Queue bytes as if the host had sent them.
  void push_rx(const std::string &text) {
    for (char c : text) {
      rx_.push_back(static_cast<uint8_t>(c));
    }
  }

  void set_write_fails(bool fails) { write_fails_ = fails; }
  void set_init_ok(bool ok) { init_ok_ = ok; }

  std::string transmitted() const { return std::string(tx_.begin(), tx_.end()); }
  void clear_transmitted() { tx_.clear(); }
  size_t rx_pending() const { return rx_.size(); }
  int init_calls() const { return init_calls_; }

 private:
  static bool init_cb(void *ctx, uint32_t /*baud*/) {
    auto *self = static_cast<FakeUart *>(ctx);
    self->init_calls_++;
    return self->init_ok_;
  }

  static size_t write_cb(void *ctx, const uint8_t *data, size_t len,
                         uint32_t /*timeout_ms*/) {
    auto *self = static_cast<FakeUart *>(ctx);
    if (self->write_fails_) {
      return 0;
    }
    self->tx_.insert(self->tx_.end(), data, data + len);
    return len;
  }

  static size_t read_cb(void *ctx, uint8_t *data, size_t len, uint32_t /*timeout_ms*/) {
    auto *self = static_cast<FakeUart *>(ctx);
    size_t count = 0;
    while (count < len && !self->rx_.empty()) {
      data[count++] = self->rx_.front();
      self->rx_.erase(self->rx_.begin());
    }
    return count;
  }

  static size_t rx_available_cb(void *ctx) {
    return static_cast<FakeUart *>(ctx)->rx_.size();
  }

  hal_uart_iface_t iface_{};
  std::vector<uint8_t> tx_;
  std::vector<uint8_t> rx_;
  bool write_fails_ = false;
  bool init_ok_ = true;
  int init_calls_ = 0;
};

class FakeSystem {
 public:
  FakeSystem() {
    iface_.ctx = this;
    iface_.reset_cause = &FakeSystem::reset_cause_cb;
    iface_.boot_count = &FakeSystem::boot_count_cb;
  }

  hal_system_iface_t *iface() { return &iface_; }

  void set_reset_cause(hal_reset_cause_t cause) { cause_ = cause; }
  void set_boot_count(uint32_t count) { boot_count_ = count; }

 private:
  static hal_reset_cause_t reset_cause_cb(void *ctx) {
    return static_cast<FakeSystem *>(ctx)->cause_;
  }

  static uint32_t boot_count_cb(void *ctx) {
    return static_cast<FakeSystem *>(ctx)->boot_count_;
  }

  hal_system_iface_t iface_{};
  hal_reset_cause_t cause_ = HAL_RESET_POWER_ON;
  uint32_t boot_count_ = 1;
};

} // namespace node::test

#endif // NODE_TEST_FAKE_PERIPHERALS_H
