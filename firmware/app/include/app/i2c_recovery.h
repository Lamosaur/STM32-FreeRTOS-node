/* i2c_recovery.h — the 9-clock bus release (FR-0.3).
 *
 * A slave interrupted mid-byte can hold SDA low indefinitely, which wedges the bus and
 * makes every subsequent transfer fail. The fix is to clock the slave out of its
 * stuck state instead of resetting the MCU: nine SCL pulses with SDA released, then a
 * STOP, then re-initialise the peripheral.
 *
 * This class emits that sequence through hal_i2c_iface_t and reports exactly what
 * happened. It is deliberately target-independent so the sequence can be verified on
 * the host against a recording fake (tests/test_i2c_recovery.cpp) — the scope capture
 * FR-0.3 asks for then confirms the electrical behaviour of a sequence that is already
 * known to be logically correct, rather than being the first time it is checked.
 */
#ifndef APP_I2C_RECOVERY_H
#define APP_I2C_RECOVERY_H

#include <cstdint>

#include "app/board.h"
#include "hal/i2c.h"

namespace node {

// Raw levels of the two bus lines.
struct BusLines {
  bool scl_high = false;
  bool sda_high = false;

  // An idle bus is both lines released high through the pull-ups.
  bool idle() const { return scl_high && sda_high; }
};

enum class RecoveryOutcome {
  NotNeeded,      // the bus was already idle; no edges were emitted
  Recovered,      // the sequence ran and the bus is idle afterwards
  SdaStuckLow,    // SDA never released: a slave is still holding it
  PeripheralFail, // the peripheral could not be taken over or restored
};

const char *to_string(RecoveryOutcome outcome);

struct RecoveryStats {
  uint32_t attempts = 0;
  uint32_t not_needed = 0;
  uint32_t recovered = 0;
  uint32_t sda_stuck_low = 0;
  uint32_t peripheral_fail = 0;
};

class I2cRecovery {
 public:
  explicit I2cRecovery(const hal_i2c_iface_t *bus, int clocks = kRecoveryClocks,
                       uint32_t half_period_us = kRecoveryHalfPeriodUs);

  // Sample the raw bus levels, without emitting anything.
  bool lines(BusLines *out) const;

  // Run the full release sequence. Safe to call on a healthy bus: it checks first and
  // returns NotNeeded without touching the lines.
  RecoveryOutcome run();

  const RecoveryStats &stats() const { return stats_; }

  // SCL pulses emitted by the most recent run(), for the verification harness.
  int last_clocks_emitted() const { return last_clocks_; }

  // True if SDA was observed released at some point during the most recent run().
  bool last_sda_released() const { return last_sda_released_; }

 private:
  // One clock pulse: raise SCL, then lower it, leaving SCL low. The caller has already
  // parked SCL low, so a pulse is raise-then-lower and no SCL transition can be
  // confused with a STOP. Returns false if an edge failed.
  bool pulse_clock();

  // STOP condition: SDA pulled low while SCL is low, SCL raised, SDA released. Returns
  // false if an edge failed.
  bool stop_condition();

  const hal_i2c_iface_t *bus_;
  int clocks_;
  uint32_t half_period_us_;

  RecoveryStats stats_{};
  int last_clocks_ = 0;
  bool last_sda_released_ = false;
};

} // namespace node

#endif // APP_I2C_RECOVERY_H
