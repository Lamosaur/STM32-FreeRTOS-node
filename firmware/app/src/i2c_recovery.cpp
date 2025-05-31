#include "app/i2c_recovery.h"

namespace node {

const char *to_string(RecoveryOutcome outcome) {
  switch (outcome) {
    case RecoveryOutcome::NotNeeded:      return "not-needed";
    case RecoveryOutcome::Recovered:      return "recovered";
    case RecoveryOutcome::SdaStuckLow:    return "sda-stuck-low";
    case RecoveryOutcome::PeripheralFail: return "peripheral-fail";
  }
  return "unknown";
}

I2cRecovery::I2cRecovery(const hal_i2c_iface_t *bus, int clocks, uint32_t half_period_us)
    : bus_(bus), clocks_(clocks), half_period_us_(half_period_us) {}

bool I2cRecovery::lines(BusLines *out) const {
  if (out == nullptr || bus_ == nullptr || bus_->read_lines == nullptr) {
    return false;
  }
  return bus_->read_lines(bus_->ctx, &out->scl_high, &out->sda_high);
}

bool I2cRecovery::pulse_clock() {
  // The caller has already parked SCL low, so a pulse is raise-then-lower. Leaving SCL
  // low at the end of each pulse means only the deliberate STOP raises SDA while SCL
  // is high.
  if (!bus_->scl_write(bus_->ctx, true)) {
    return false;
  }
  bus_->delay_us(bus_->ctx, half_period_us_);
  if (!bus_->scl_write(bus_->ctx, false)) {
    return false;
  }
  bus_->delay_us(bus_->ctx, half_period_us_);
  return true;
}

bool I2cRecovery::stop_condition() {
  // A STOP is SDA rising while SCL is high. SCL is already low, so pull SDA low first,
  // then raise SCL, then release SDA. That is exactly one STOP and no other edge that
  // could be mistaken for one.
  if (!bus_->sda_write(bus_->ctx, false)) {
    return false;
  }
  bus_->delay_us(bus_->ctx, half_period_us_);
  if (!bus_->scl_write(bus_->ctx, true)) {
    return false;
  }
  bus_->delay_us(bus_->ctx, half_period_us_);
  if (!bus_->sda_write(bus_->ctx, true)) {
    return false;
  }
  bus_->delay_us(bus_->ctx, half_period_us_);
  return true;
}

RecoveryOutcome I2cRecovery::run() {
  stats_.attempts++;
  last_clocks_ = 0;
  last_sda_released_ = false;

  if (bus_ == nullptr || bus_->recover_begin == nullptr) {
    stats_.peripheral_fail++;
    return RecoveryOutcome::PeripheralFail;
  }

  // If the bus is already idle there is nothing to release, and clocking a healthy bus
  // would emit edges that a slave could misinterpret.
  BusLines before{};
  if (lines(&before) && before.idle()) {
    stats_.not_needed++;
    last_sda_released_ = true;
    return RecoveryOutcome::NotNeeded;
  }

  if (!bus_->recover_begin(bus_->ctx)) {
    stats_.peripheral_fail++;
    return RecoveryOutcome::PeripheralFail;
  }

  // Park SCL low *before* releasing SDA.
  //
  // Releasing SDA while SCL is high is itself a STOP condition, which is not harmful
  // but is incidental: it makes the emitted edge sequence depend on what the slave
  // happened to be doing when the fault occurred. Parking SCL low first means the
  // sequence is fixed, and the capture FR-0.3 asks for shows exactly nine clocks and
  // exactly one deliberate STOP.
  bool ok = bus_->scl_write(bus_->ctx, false);
  bus_->delay_us(bus_->ctx, half_period_us_);

  // Release SDA: the whole point is to let a stuck slave finish driving it, and the
  // master must not hold it low while clocking.
  if (ok) {
    ok = bus_->sda_write(bus_->ctx, true);
  }
  bus_->delay_us(bus_->ctx, half_period_us_);

  for (int i = 0; ok && i < clocks_; ++i) {
    ok = pulse_clock();
    if (!ok) {
      break;
    }
    last_clocks_++;

    bool sda_high = false;
    if (bus_->sda_read(bus_->ctx, &sda_high) && sda_high) {
      last_sda_released_ = true;
    }
  }

  if (ok) {
    ok = stop_condition();
  }

  if (!bus_->recover_end(bus_->ctx)) {
    stats_.peripheral_fail++;
    return RecoveryOutcome::PeripheralFail;
  }

  if (!ok) {
    // The lines could not be driven at all; the peripheral is back, but the bus state
    // is unknown.
    stats_.sda_stuck_low++;
    return RecoveryOutcome::SdaStuckLow;
  }

  BusLines after{};
  if (lines(&after) && after.idle()) {
    stats_.recovered++;
    return RecoveryOutcome::Recovered;
  }

  // SCL was clocked and the peripheral re-initialised, yet SDA is still held low: the
  // slave needs its own reset, not more clocks.
  stats_.sda_stuck_low++;
  return RecoveryOutcome::SdaStuckLow;
}

} // namespace node
