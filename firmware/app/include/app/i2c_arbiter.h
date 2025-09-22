/* i2c_arbiter.h — the single serialisation point for the shared I2C bus (FR-0.2).
 *
 * Every I2C peripheral in this system sits on one bus, and more than one task wants it:
 * the sensor task reads an AHT20 every second while the display task pushes pixels to
 * an SSD1306. Interleaving those two transfers corrupts both, so all access goes
 * through this object, which owns one priority-inheriting mutex.
 *
 * The mutex is the whole design, and it is why FR-1.7 exists: the arbiter sits on the
 * critical path of every sensor and display cycle, so if that mutex did not inherit
 * priority, a low-priority display flush could hold the bus while a high-priority
 * sensor task waited behind it, and the control loop downstream would inherit that
 * delay.
 *
 * Recovery policy, which is the part worth stating explicitly:
 *   - A NACK is not a fault. The device simply did not answer, which is a fact about
 *     the device, and clocking the bus cannot change it. NACKs are returned as-is and
 *     never trigger recovery.
 *   - A bus error or timeout can mean a slave is holding SDA, so those do trigger the
 *     FR-0.3 release, followed by one retry.
 */
#ifndef APP_I2C_ARBITER_H
#define APP_I2C_ARBITER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "app/i2c_recovery.h"
#include "hal/i2c.h"
#include "rtos/rtos.h"

namespace node {

enum class BusResult {
  Ok,
  Nack,        // the device did not acknowledge; not a bus fault
  BusError,    // line fault that recovery could not clear
  Timeout,     // the peripheral never completed
  Unavailable, // the mutex was not acquired within the lock timeout
};

const char *to_string(BusResult result);

// A copyable snapshot. Read it with stats(), which returns plain values rather than
// handing out the live counters, so a reporting task cannot be caught mid-update.
struct ArbiterStats {
  uint32_t transfers = 0;         // transfers that completed OK
  uint32_t dma_transfers = 0;     // of those, the ones issued over the DMA path
  uint32_t dma_bytes = 0;         // bytes pushed by those transfers
  uint32_t contended = 0;         // acquisitions that had to block on the mutex
  uint32_t lock_timeouts = 0;     // acquisitions that timed out
  uint32_t recoveries = 0;        // bus faults cleared by the FR-0.3 release
  uint32_t recovery_failures = 0; // bus faults that survived recovery and retry
  uint32_t failed_transfers = 0;  // transfers that returned non-OK
  uint32_t bus_faults = 0;        // of those, the ones that were bus faults
};

struct ArbiterConfig {
  uint32_t lock_timeout_ms = 50;
  int recovery_attempts = kRecoveryMaxAttempts;
};

class I2cArbiter {
 public:
  // `bus` and `recovery` are borrowed and must outlive this object. The mutex is
  // created here, once, and intentionally never destroyed: creating and freeing
  // synchronisation objects at run time is how a long unattended run (NFR-0.7)
  // fragments the heap.
  I2cArbiter(const hal_i2c_iface_t *bus, const rtos_iface_t *rtos, I2cRecovery *recovery,
             ArbiterConfig config = {});

  I2cArbiter(const I2cArbiter &) = delete;
  I2cArbiter &operator=(const I2cArbiter &) = delete;

  // True if the mutex was created; if false, every transfer returns Unavailable.
  bool valid() const { return mutex_ != nullptr; }

  // Write then read, atomically with respect to every other task on the bus.
  BusResult transfer(uint16_t addr7, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                     size_t rx_len);

  BusResult write(uint16_t addr7, const uint8_t *tx, size_t tx_len);
  BusResult read(uint16_t addr7, uint8_t *rx, size_t rx_len);

  // Write via DMA (FR-0.5). The arbiter still holds the bus mutex for the whole
  // transfer, so this does not let another task onto the bus early — what it buys is
  // that the CPU is not shifting bytes for the duration, which on a 1024-byte frame
  // flush is most of the transfer.
  //
  // `tx` must stay valid until this returns; it does, because the call waits.
  BusResult transfer_dma(uint16_t addr7, const uint8_t *tx, size_t tx_len);

  // Force an unconditional bus release, ignoring whether the lines look idle. Used by
  // the FR-0.2 stress run to prove recovery works mid-contention.
  RecoveryOutcome force_recovery();

  // A plain-value snapshot of the counters.
  //
  // Deliberately not a reference to the live counters: the lock-timeout counter is
  // incremented on the path where the mutex was *not* acquired, so it cannot be
  // protected by that mutex, and a multi-threaded stress run (the FR-0.2 test) would
  // otherwise be a data race on it. The internal counters are atomic and this returns
  // a consistent-enough copy for diagnostics.
  ArbiterStats stats() const;
  void reset_stats();

  // The mutex, for tests that need to hold the bus open.
  rtos_mutex_t *mutex() const { return mutex_; }

 private:
  // A bus error or timeout is worth a recovery attempt; a NACK never is.
  static bool is_bus_fault(hal_i2c_status_t status);

  static BusResult map_status(hal_i2c_status_t status);

  // One peripheral attempt. `dma` selects the DMA path, which is write-only.
  hal_i2c_status_t raw(bool dma, uint16_t addr7, const uint8_t *tx, size_t tx_len,
                       uint8_t *rx, size_t rx_len);

  // The lock, recovery, retry and stats bookkeeping shared by every transfer shape.
  //
  // All entry points funnel through here rather than each re-implementing it, because
  // forgetting the bookkeeping in one of them is exactly how a bus quietly stops being
  // arbitrated. Returns the mapped result.
  BusResult transfer_impl(bool dma, uint16_t addr7, const uint8_t *tx, size_t tx_len,
                          uint8_t *rx, size_t rx_len);

  // Live counters. Every one of these is written from a path that may run concurrently
  // with another task's transfer; all but lock_timeouts happen to sit inside the
  // critical section today, but making them atomic means a future change that moves a
  // counter out of it cannot silently introduce a race.
  struct Counters {
    std::atomic<uint32_t> transfers{0};
    std::atomic<uint32_t> dma_transfers{0};
    std::atomic<uint32_t> dma_bytes{0};
    std::atomic<uint32_t> contended{0};
    std::atomic<uint32_t> lock_timeouts{0};
    std::atomic<uint32_t> recoveries{0};
    std::atomic<uint32_t> recovery_failures{0};
    std::atomic<uint32_t> failed_transfers{0};
    std::atomic<uint32_t> bus_faults{0};
  };

  const hal_i2c_iface_t *bus_;
  const rtos_iface_t *rtos_;
  I2cRecovery *recovery_;
  ArbiterConfig config_;
  rtos_mutex_t *mutex_;
  Counters counters_{};
};

} // namespace node

#endif // APP_I2C_ARBITER_H
