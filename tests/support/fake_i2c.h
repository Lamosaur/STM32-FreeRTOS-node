/* fake_i2c.h — host double for hal_i2c_iface_t.
 *
 * Two jobs:
 *
 *   1. Record every call in order, so a test can assert on the *exact edge sequence*
 *      the recovery sequencer emitted (FR-0.3) rather than on a vague "it called
 *      recovery". The order of SCL and SDA edges is the thing being verified.
 *
 *   2. Detect concurrent transfers. The FR-0.2 stress test runs real threads through
 *      the arbiter; if two transfers are ever in flight at once, mutex protection is
 *      broken and the counter catches it. A test that only checked "no error
 *      returned" would pass with a completely absent mutex.
 *
 * Host-only; never compiled into the target image.
 */
#ifndef NODE_TEST_FAKE_I2C_H
#define NODE_TEST_FAKE_I2C_H

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "hal/i2c.h"

namespace node::test {

enum class BusOp {
  Transfer,
  TransferDma,
  TransferWait,
  ReadLines,
  RecoverBegin,
  SclWrite,
  SdaWrite,
  SdaRead,
  SclRead,
  DelayUs,
  RecoverEnd,
};

inline const char *to_string(BusOp op) {
  switch (op) {
    case BusOp::Transfer:     return "transfer";
    case BusOp::TransferDma:  return "transfer-dma";
    case BusOp::TransferWait: return "transfer-wait";
    case BusOp::ReadLines:    return "read-lines";
    case BusOp::RecoverBegin: return "recover-begin";
    case BusOp::SclWrite:     return "scl-write";
    case BusOp::SdaWrite:     return "sda-write";
    case BusOp::SdaRead:      return "sda-read";
    case BusOp::SclRead:      return "scl-read";
    case BusOp::DelayUs:      return "delay-us";
    case BusOp::RecoverEnd:   return "recover-end";
  }
  return "?";
}

// One recorded call. `level` is the level for the write ops; `addr`/`arg` carry the
// transfer address and the delay duration.
struct BusLogEntry {
  BusOp op = BusOp::Transfer;
  bool level = false;
  uint16_t addr = 0;
  uint32_t arg = 0;
};

// "The slave holds SDA low forever" — clocking it out will not work.
inline constexpr int kNeverRelease = -1;

class FakeI2c {
 public:
  FakeI2c() {
    iface_.ctx = this;
    iface_.transfer = &FakeI2c::transfer_cb;
    iface_.transfer_start_dma = &FakeI2c::transfer_start_dma_cb;
    iface_.transfer_wait = &FakeI2c::transfer_wait_cb;
    iface_.read_lines = &FakeI2c::read_lines_cb;
    iface_.recover_begin = &FakeI2c::recover_begin_cb;
    iface_.scl_write = &FakeI2c::scl_write_cb;
    iface_.sda_write = &FakeI2c::sda_write_cb;
    iface_.sda_read = &FakeI2c::sda_read_cb;
    iface_.scl_read = &FakeI2c::scl_read_cb;
    iface_.delay_us = &FakeI2c::delay_us_cb;
    iface_.recover_end = &FakeI2c::recover_end_cb;
  }

  // Non-const: the node takes the mutable hardware handles it is handed, while the
  // arbiter and recovery take const ones, which convert freely.
  hal_i2c_iface_t *iface() { return &iface_; }

  // --- scenario -----------------------------------------------------------

  // Simulate a slave holding SDA low. It releases after `clocks` SCL rising edges
  // during a recovery, or never if `kNeverRelease`. Any value other than 0 also makes
  // the bus read as non-idle up front, which is what triggers a recovery attempt.
  void hold_sda_low_for_clocks(int clocks) {
    std::lock_guard<std::mutex> g(mtx_);
    release_after_clocks_ = clocks;
    sda_held_ = (clocks != 0);
  }

  // Every transfer returns this status unless a failure burst is queued.
  void set_transfer_status(hal_i2c_status_t status) { transfer_status_ = status; }

  // The next `n` transfers fail with `status`, then transfers return
  // transfer_status() again. Forces the arbiter down the recovery path exactly once.
  void fail_transfers(int n, hal_i2c_status_t status) {
    std::lock_guard<std::mutex> g(mtx_);
    fail_remaining_ = n;
    fail_status_ = status;
  }

  // Fail only transfers to one address. A real bus has a dead device on it, not a dead
  // bus, and failing every address also disables the display the test is trying to
  // observe — which is how a test ends up asserting the wrong thing.
  void fail_address(uint16_t addr7, hal_i2c_status_t status) {
    std::lock_guard<std::mutex> g(mtx_);
    addr_failures_[addr7] = status;
  }

  void clear_address_failures() {
    std::lock_guard<std::mutex> g(mtx_);
    addr_failures_.clear();
  }

  void set_recover_begin_ok(bool ok) { recover_begin_ok_ = ok; }
  void set_recover_end_ok(bool ok) { recover_end_ok_ = ok; }
  void set_read_lines_ok(bool ok) { read_lines_ok_ = ok; }
  void set_rx_fill(uint8_t value) { rx_fill_ = value; }

  // Canned bytes returned by a pure read (rx_len > 0), standing in for a real device's
  // register contents. When unset, every received byte is rx_fill(). A short response
  // is padded with rx_fill() so a test can hand over a truncated reply and see what the
  // driver does with it.
  void set_read_data(std::vector<uint8_t> data) {
    std::lock_guard<std::mutex> g(mtx_);
    read_data_ = std::move(data);
  }

  // Sleep inside each transfer, to widen the window a missing mutex would slip through.
  void set_transfer_delay_us(uint32_t us) { transfer_delay_us_ = us; }

  // --- observation --------------------------------------------------------

  std::vector<BusLogEntry> log() const {
    std::lock_guard<std::mutex> g(mtx_);
    return log_;
  }

  void clear_log() {
    std::lock_guard<std::mutex> g(mtx_);
    log_.clear();
  }

  int count(BusOp op) const {
    std::lock_guard<std::mutex> g(mtx_);
    int n = 0;
    for (const auto &e : log_) {
      if (e.op == op) {
        ++n;
      }
    }
    return n;
  }

  int transfer_calls() const { return transfers_.load(); }
  int transfers_ok() const { return transfers_ok_.load(); }
  int recoveries_run() const { return recoveries_.load(); }
  int overlaps() const { return overlaps_.load(); }
  int max_concurrent_transfers() const { return max_concurrent_.load(); }

  // DMA path (FR-0.5).
  int dma_writes() const { return dma_writes_.load(); }
  int dma_bytes() const { return dma_bytes_.load(); }
  bool dma_busy() const { return dma_busy_; }

  // Addresses of every transfer attempted, in order. Retries show up as repeats.
  std::vector<uint16_t> transfer_addrs() const {
    std::lock_guard<std::mutex> g(mtx_);
    std::vector<uint16_t> out;
    for (const auto &e : log_) {
      if (e.op == BusOp::Transfer) {
        out.push_back(e.addr);
      }
    }
    return out;
  }

  // Payload of the most recent transfer's write phase.
  std::vector<uint8_t> last_tx() const {
    std::lock_guard<std::mutex> g(mtx_);
    return last_tx_;
  }

  // Length of the most recent transfer's read phase. A driver's framing is as much a part of
  // its correctness as its arithmetic — reading six bytes of a seven-byte frame, or seven of
  // a six-byte one, is exactly the mistake a sensor swap invites — so the length it asked
  // for is observable rather than implied by whatever bytes happened to be in the reply.
  size_t last_rx_len() const {
    std::lock_guard<std::mutex> g(mtx_);
    return last_rx_len_;
  }

  // The SCL edges emitted during the most recent recovery, in order. true = rising.
  std::vector<bool> scl_edges() const {
    std::lock_guard<std::mutex> g(mtx_);
    std::vector<bool> out;
    for (const auto &e : log_) {
      if (e.op == BusOp::SclWrite) {
        out.push_back(e.level);
      }
    }
    return out;
  }

  // Number of STOP conditions generated: SDA rising while SCL is released high.
  int stop_conditions() const { return stop_conditions_.load(); }

  // Whether the master released SDA at any point, which the release sequence must do
  // before it starts clocking.
  bool master_released_sda() const { return master_released_sda_.load(); }

  // Total SCL rising edges. This counts the nine release pulses *and* the SCL release
  // inside the trailing STOP, so it is 10 for a correct sequence. The fake reports the
  // raw count and the test separates the phases, rather than the fake guessing which
  // edges "were clocks".
  int scl_rising_edges() const { return scl_rising_edges_.load(); }

  // SCL rising edges emitted before the STOP begins: the nine release pulses of FR-0.3,
  // counted the way the scope capture counts them.
  int recovery_clocks_in_release() const {
    std::lock_guard<std::mutex> g(mtx_);
    int pulses = 0;
    for (const auto &e : log_) {
      if (e.op == BusOp::SdaWrite && !e.level) {
        break; // the STOP starts here
      }
      if (e.op == BusOp::SclWrite && e.level) {
        ++pulses;
      }
    }
    return pulses;
  }

 private:
  static hal_i2c_status_t transfer_cb(void *ctx, uint16_t addr7, const uint8_t *tx,
                                      size_t tx_len, uint8_t *rx, size_t rx_len,
                                      uint32_t /*timeout_ms*/) {
    auto *self = static_cast<FakeI2c *>(ctx);

    const int live = self->concurrent_.fetch_add(1) + 1;
    if (live > 1) {
      self->overlaps_.fetch_add(1);
    }
    int prev = self->max_concurrent_.load();
    while (live > prev && !self->max_concurrent_.compare_exchange_weak(prev, live)) {
    }
    self->transfers_.fetch_add(1);

    const uint32_t delay_us = self->transfer_delay_us_.load();
    if (delay_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    }

    hal_i2c_status_t status;
    {
      std::lock_guard<std::mutex> g(self->mtx_);
      BusLogEntry entry;
      entry.op = BusOp::Transfer;
      entry.addr = addr7;
      entry.arg = static_cast<uint32_t>(tx_len);
      self->log_.push_back(entry);
      self->last_tx_.assign(tx, tx + tx_len);
      self->last_rx_len_ = rx_len;

      const auto by_addr = self->addr_failures_.find(addr7);
      if (by_addr != self->addr_failures_.end()) {
        status = by_addr->second;
      } else if (self->fail_remaining_ > 0) {
        --self->fail_remaining_;
        status = self->fail_status_;
      } else {
        status = self->transfer_status_;
      }
    }

    if (status == HAL_I2C_OK) {
      self->transfers_ok_.fetch_add(1);
      std::lock_guard<std::mutex> g(self->mtx_);
      for (size_t i = 0; i < rx_len; ++i) {
        rx[i] = i < self->read_data_.size() ? self->read_data_[i] : self->rx_fill_;
      }
    }

    self->concurrent_.fetch_sub(1);
    return status;
  }

  static bool transfer_start_dma_cb(void *ctx, uint16_t addr7, const uint8_t *tx,
                                    size_t tx_len) {
    auto *self = static_cast<FakeI2c *>(ctx);
    if (self->dma_busy_) {
      return false; // a previous transfer is still in flight
    }

    const uint32_t delay_us = self->transfer_delay_us_.load();
    if (delay_us > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    }

    std::lock_guard<std::mutex> g(self->mtx_);
    BusLogEntry entry;
    entry.op = BusOp::TransferDma;
    entry.addr = addr7;
    entry.arg = static_cast<uint32_t>(tx_len);
    self->log_.push_back(entry);
    // Copy the payload: the caller's buffer stays valid until transfer_wait() returns,
    // but copying lets a test inspect what was sent even after the fact.
    self->last_tx_.assign(tx, tx + tx_len);

    if (self->fail_remaining_ > 0) {
      --self->fail_remaining_;
      self->dma_status_ = self->fail_status_;
    } else {
      self->dma_status_ = self->transfer_status_;
    }

    self->dma_busy_ = true;
    self->dma_writes_.fetch_add(1);
    self->dma_bytes_.fetch_add(static_cast<int>(tx_len));
    self->transfers_.fetch_add(1);
    return true;
  }

  static hal_i2c_status_t transfer_wait_cb(void *ctx, uint32_t /*timeout_ms*/) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::TransferWait, false, 0, 0});
    if (!self->dma_busy_) {
      return HAL_I2C_BUS_ERROR; // waited on a transfer that was never started
    }
    self->dma_busy_ = false;
    if (self->dma_status_ == HAL_I2C_OK) {
      self->transfers_ok_.fetch_add(1);
    }
    return self->dma_status_;
  }

  static bool read_lines_cb(void *ctx, bool *scl_high, bool *sda_high) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::ReadLines, false, 0, 0});
    *scl_high = self->scl_high_;
    *sda_high = !self->sda_held_;
    return self->read_lines_ok_;
  }

  static bool recover_begin_cb(void *ctx) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::RecoverBegin, false, 0, 0});
    self->recoveries_.fetch_add(1);
    self->in_recovery_ = true;
    self->scl_rising_edges_ = 0;
    self->clocks_in_recovery_ = 0;
    // The master no longer drives either line; SCL is released high, SDA is whatever
    // the slave is doing with it.
    self->scl_high_ = true;
    return self->recover_begin_ok_;
  }

  static bool scl_write_cb(void *ctx, bool level) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::SclWrite, level, 0, 0});
    self->scl_high_ = level;

    if (self->in_recovery_ && level) {
      ++self->clocks_in_recovery_;
      self->scl_rising_edges_.fetch_add(1);
      if (self->sda_held_ && self->release_after_clocks_ != kNeverRelease &&
          self->clocks_in_recovery_ >= self->release_after_clocks_) {
        self->sda_held_ = false;
      }
    }
    return true;
  }

  static bool sda_write_cb(void *ctx, bool level) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::SdaWrite, level, 0, 0});

    // A STOP is SDA rising while SCL is released high. Recording it here means the
    // test can assert the sequence produced a real STOP, not just nine clocks.
    if (level && self->scl_high_ && !self->master_sda_high_) {
      self->stop_conditions_.fetch_add(1);
    }
    if (level) {
      self->master_released_sda_.store(true);
    }
    self->master_sda_high_ = level;
    return true;
  }

  static bool sda_read_cb(void *ctx, bool *level) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::SdaRead, false, 0, 0});
    *level = !self->sda_held_;
    return true;
  }

  static bool scl_read_cb(void *ctx, bool *level) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::SclRead, false, 0, 0});
    *level = self->scl_high_;
    return true;
  }

  static void delay_us_cb(void *ctx, uint32_t us) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::DelayUs, false, 0, us});
  }

  static bool recover_end_cb(void *ctx) {
    auto *self = static_cast<FakeI2c *>(ctx);
    std::lock_guard<std::mutex> g(self->mtx_);
    self->log_.push_back(BusLogEntry{BusOp::RecoverEnd, false, 0, 0});
    self->in_recovery_ = false;
    self->scl_high_ = true;
    if (!self->sda_held_) {
      self->master_sda_high_ = true;
    }
    return self->recover_end_ok_;
  }

  hal_i2c_iface_t iface_{};
  mutable std::mutex mtx_;

  std::vector<BusLogEntry> log_;
  std::vector<uint8_t> last_tx_;
  std::vector<uint8_t> read_data_;
  std::map<uint16_t, hal_i2c_status_t> addr_failures_;
  size_t last_rx_len_ = 0;

  bool scl_high_ = true;      // level on SCL (released high when idle)
  bool sda_held_ = false;     // a slave is holding SDA low
  int release_after_clocks_ = 0;
  int clocks_in_recovery_ = 0;
  bool in_recovery_ = false;
  bool master_sda_high_ = false;
  bool dma_busy_ = false;
  hal_i2c_status_t dma_status_ = HAL_I2C_OK;

  hal_i2c_status_t transfer_status_ = HAL_I2C_OK;
  hal_i2c_status_t fail_status_ = HAL_I2C_BUS_ERROR;
  int fail_remaining_ = 0;
  bool recover_begin_ok_ = true;
  bool recover_end_ok_ = true;
  bool read_lines_ok_ = true;
  uint8_t rx_fill_ = 0;

  std::atomic<uint32_t> transfer_delay_us_{0};
  std::atomic<int> concurrent_{0};
  std::atomic<int> max_concurrent_{0};
  std::atomic<int> overlaps_{0};
  std::atomic<int> transfers_{0};
  std::atomic<int> transfers_ok_{0};
  std::atomic<int> recoveries_{0};
  std::atomic<int> stop_conditions_{0};
  std::atomic<int> scl_rising_edges_{0};
  std::atomic<int> dma_writes_{0};
  std::atomic<int> dma_bytes_{0};
  std::atomic<bool> master_released_sda_{false};
};

} // namespace node::test

#endif // NODE_TEST_FAKE_I2C_H
