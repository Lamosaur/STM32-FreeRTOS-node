#include "app/i2c_arbiter.h"

namespace node {

const char *to_string(BusResult result) {
  switch (result) {
    case BusResult::Ok:          return "ok";
    case BusResult::Nack:        return "nack";
    case BusResult::BusError:    return "bus-error";
    case BusResult::Timeout:     return "timeout";
    case BusResult::Unavailable: return "unavailable";
  }
  return "unknown";
}

I2cArbiter::I2cArbiter(const hal_i2c_iface_t *bus, const rtos_iface_t *rtos,
                       I2cRecovery *recovery, ArbiterConfig config)
    : bus_(bus),
      rtos_(rtos),
      recovery_(recovery),
      config_(config),
      mutex_(nullptr) {
  if (rtos_ != nullptr && rtos_->mutex_create != nullptr) {
    mutex_ = rtos_->mutex_create(rtos_->ctx);
  }
}

ArbiterStats I2cArbiter::stats() const {
  ArbiterStats out;
  out.transfers = counters_.transfers.load();
  out.dma_transfers = counters_.dma_transfers.load();
  out.dma_bytes = counters_.dma_bytes.load();
  out.contended = counters_.contended.load();
  out.lock_timeouts = counters_.lock_timeouts.load();
  out.recoveries = counters_.recoveries.load();
  out.recovery_failures = counters_.recovery_failures.load();
  out.failed_transfers = counters_.failed_transfers.load();
  out.bus_faults = counters_.bus_faults.load();
  return out;
}

void I2cArbiter::reset_stats() {
  counters_.transfers.store(0);
  counters_.dma_transfers.store(0);
  counters_.dma_bytes.store(0);
  counters_.contended.store(0);
  counters_.lock_timeouts.store(0);
  counters_.recoveries.store(0);
  counters_.recovery_failures.store(0);
  counters_.failed_transfers.store(0);
  counters_.bus_faults.store(0);
}

bool I2cArbiter::is_bus_fault(hal_i2c_status_t status) {
  return status == HAL_I2C_BUS_ERROR || status == HAL_I2C_TIMEOUT;
}

BusResult I2cArbiter::map_status(hal_i2c_status_t status) {
  switch (status) {
    case HAL_I2C_OK:        return BusResult::Ok;
    case HAL_I2C_NACK:      return BusResult::Nack;
    case HAL_I2C_BUS_ERROR: return BusResult::BusError;
    case HAL_I2C_TIMEOUT:   return BusResult::Timeout;
  }
  return BusResult::BusError;
}

hal_i2c_status_t I2cArbiter::raw(bool dma, uint16_t addr7, const uint8_t *tx, size_t tx_len,
                                 uint8_t *rx, size_t rx_len) {
  if (dma) {
    if (bus_->transfer_start_dma == nullptr || bus_->transfer_wait == nullptr) {
      return HAL_I2C_BUS_ERROR;
    }
    if (!bus_->transfer_start_dma(bus_->ctx, addr7, tx, tx_len)) {
      return HAL_I2C_BUS_ERROR;
    }
    return bus_->transfer_wait(bus_->ctx, kI2cTransferTimeoutMs);
  }
  return bus_->transfer(bus_->ctx, addr7, tx, tx_len, rx, rx_len, kI2cTransferTimeoutMs);
}

BusResult I2cArbiter::transfer_impl(bool dma, uint16_t addr7, const uint8_t *tx,
                                    size_t tx_len, uint8_t *rx, size_t rx_len) {
  if (mutex_ == nullptr) {
    return BusResult::Unavailable;
  }

  bool waited = false;
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, config_.lock_timeout_ms, &waited) != RTOS_OK) {
    // Written outside the critical section, which is exactly why the counters are
    // atomic rather than plain.
    counters_.lock_timeouts.fetch_add(1);
    return BusResult::Unavailable;
  }
  if (waited) {
    counters_.contended.fetch_add(1);
  }

  hal_i2c_status_t status = raw(dma, addr7, tx, tx_len, rx, rx_len);

  // Recover and retry while the failure looks like a bus fault rather than a device
  // that is simply not answering.
  int attempts = 0;
  bool attempted_recovery = false;
  while (is_bus_fault(status) && attempts < config_.recovery_attempts) {
    ++attempts;
    attempted_recovery = true;

    const RecoveryOutcome outcome = recovery_->run();
    if (outcome == RecoveryOutcome::Recovered) {
      counters_.recoveries.fetch_add(1);
    } else if (outcome == RecoveryOutcome::SdaStuckLow ||
               outcome == RecoveryOutcome::PeripheralFail) {
      // More clocking will not help; stop rather than burn the retry budget.
      break;
    }
    // NotNeeded means the lines already read idle, so one retry is still worth it.

    status = raw(dma, addr7, tx, tx_len, rx, rx_len);
  }

  const bool still_faulted = is_bus_fault(status);
  if (attempted_recovery && still_faulted) {
    counters_.recovery_failures.fetch_add(1);
  }

  const BusResult result = map_status(status);
  if (result == BusResult::Ok) {
    counters_.transfers.fetch_add(1);
    if (dma) {
      counters_.dma_transfers.fetch_add(1);
      counters_.dma_bytes.fetch_add(static_cast<uint32_t>(tx_len));
    }
  } else {
    counters_.failed_transfers.fetch_add(1);
    if (still_faulted) {
      counters_.bus_faults.fetch_add(1);
    }
  }

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return result;
}

BusResult I2cArbiter::transfer(uint16_t addr7, const uint8_t *tx, size_t tx_len, uint8_t *rx,
                               size_t rx_len) {
  return transfer_impl(false, addr7, tx, tx_len, rx, rx_len);
}

BusResult I2cArbiter::write(uint16_t addr7, const uint8_t *tx, size_t tx_len) {
  return transfer_impl(false, addr7, tx, tx_len, nullptr, 0);
}

BusResult I2cArbiter::read(uint16_t addr7, uint8_t *rx, size_t rx_len) {
  return transfer_impl(false, addr7, nullptr, 0, rx, rx_len);
}

BusResult I2cArbiter::transfer_dma(uint16_t addr7, const uint8_t *tx, size_t tx_len) {
  return transfer_impl(true, addr7, tx, tx_len, nullptr, 0);
}

RecoveryOutcome I2cArbiter::force_recovery() {
  if (mutex_ == nullptr) {
    return RecoveryOutcome::PeripheralFail;
  }

  bool waited = false;
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, config_.lock_timeout_ms, &waited) != RTOS_OK) {
    counters_.lock_timeouts.fetch_add(1);
    return RecoveryOutcome::PeripheralFail;
  }
  if (waited) {
    counters_.contended.fetch_add(1);
  }

  const RecoveryOutcome outcome = recovery_->run();
  if (outcome == RecoveryOutcome::Recovered) {
    counters_.recoveries.fetch_add(1);
  } else if (outcome == RecoveryOutcome::SdaStuckLow ||
             outcome == RecoveryOutcome::PeripheralFail) {
    counters_.recovery_failures.fetch_add(1);
  }

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return outcome;
}

} // namespace node
