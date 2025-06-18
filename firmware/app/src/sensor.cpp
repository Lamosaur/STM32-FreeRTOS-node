#include "app/sensor.h"

namespace node {
namespace {

// SHT3x command words, most significant byte first (datasheet section 4).
constexpr uint8_t kCmdMeasureHighNoStretch = 0x24;
constexpr uint8_t kCmdMeasureHighNoStretchArg = 0x00;
constexpr uint8_t kCmdSoftReset = 0x30;
constexpr uint8_t kCmdSoftResetArg = 0xA2;

// A measurement result is two 16-bit words, each followed by its own CRC: 6 bytes.
constexpr size_t kMeasurementBytes = 6;

// Datasheet full-scale spans. Temperature is -45..+130 C, humidity 0..100 %RH, both
// linear in the 16-bit raw value.
constexpr int32_t kTemperatureOffsetMilli = -45'000;
constexpr int32_t kTemperatureSpanMilli = 175'000;
constexpr int32_t kHumiditySpanMilliRh = 100'000;
constexpr int32_t kRawFullScale = 65'535;

} // namespace

uint8_t Sht31::crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = static_cast<uint8_t>((crc << 1) ^ 0x31U);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

int32_t Sht31::raw_to_milli_celsius(uint16_t raw) {
  // The intermediate product overflows 32 bits (175000 * 65535 is about 1.15e10), so
  // the multiply is done in 64 bits. It happens once per second, so the cost is
  // irrelevant next to the correctness of not silently wrapping.
  const int64_t product = static_cast<int64_t>(kTemperatureSpanMilli) * raw;
  return kTemperatureOffsetMilli + static_cast<int32_t>(product / kRawFullScale);
}

int32_t Sht31::raw_to_milli_rh(uint16_t raw) {
  const int64_t product = static_cast<int64_t>(kHumiditySpanMilliRh) * raw;
  const int32_t milli_rh = static_cast<int32_t>(product / kRawFullScale);
  // The datasheet's humidity range is 0..100 %RH, so an out-of-range result would mean
  // the raw word was corrupt despite passing CRC. Clamp rather than report a value the
  // sensor cannot produce.
  if (milli_rh < 0) {
    return 0;
  }
  if (milli_rh > kHumiditySpanMilliRh) {
    return kHumiditySpanMilliRh;
  }
  return milli_rh;
}

Sht31::Sht31(I2cArbiter *bus, uint16_t addr7) : bus_(bus), addr7_(addr7) {}

bool Sht31::reset() {
  const uint8_t command[2] = {kCmdSoftReset, kCmdSoftResetArg};
  const bool ok = bus_->write(addr7_, command, sizeof(command)) == BusResult::Ok;
  if (!ok) {
    stats_.bus_errors++;
  }
  return ok;
}

bool Sht31::trigger() {
  const uint8_t command[2] = {kCmdMeasureHighNoStretch, kCmdMeasureHighNoStretchArg};
  const bool ok = bus_->write(addr7_, command, sizeof(command)) == BusResult::Ok;
  if (!ok) {
    stats_.bus_errors++;
  }
  return ok;
}

bool Sht31::read(Sht31Reading *out) {
  Sht31Reading reading{};

  uint8_t raw[kMeasurementBytes] = {0};
  if (bus_->read(addr7_, raw, sizeof(raw)) != BusResult::Ok) {
    stats_.bus_errors++;
    *out = reading;
    return false;
  }

  // Each 16-bit word carries its own CRC byte, so a single corrupted byte in either
  // word is caught rather than propagating into the converted value.
  if (crc8(&raw[0], 2) != raw[2] || crc8(&raw[3], 2) != raw[5]) {
    stats_.crc_errors++;
    *out = reading;
    return false;
  }

  reading.raw_temperature = static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) |
                                                 raw[1]);
  reading.raw_humidity = static_cast<uint16_t>((static_cast<uint16_t>(raw[3]) << 8) |
                                               raw[4]);
  reading.temperature_milli_c = raw_to_milli_celsius(reading.raw_temperature);
  reading.humidity_milli_rh = raw_to_milli_rh(reading.raw_humidity);
  reading.valid = true;

  stats_.measurements++;
  *out = reading;
  return true;
}

SensorService::SensorService(I2cArbiter *bus, const rtos_iface_t *rtos, uint16_t addr7,
                             uint32_t conversion_ms)
    : bus_(bus),
      rtos_(rtos),
      driver_(bus, addr7),
      conversion_ms_(conversion_ms),
      mutex_(nullptr) {
  if (rtos_ != nullptr && rtos_->mutex_create != nullptr) {
    mutex_ = rtos_->mutex_create(rtos_->ctx);
  }
}

bool SensorService::init() {
  if (!driver_.reset()) {
    return false;
  }
  // The soft reset needs a moment before the device accepts a command.
  rtos_->delay_ms(rtos_->ctx, 2);
  return true;
}

bool SensorService::update() {
  stats_.polls++;

  if (!driver_.trigger()) {
    stats_.failed++;
    note_failure();
    return false;
  }

  // The bus is free for the other tasks here; see the comment on update() in the header.
  rtos_->delay_ms(rtos_->ctx, conversion_ms_);

  Sht31Reading reading{};
  if (!driver_.read(&reading) || !reading.valid) {
    stats_.failed++;
    note_failure();
    return false;
  }

  if (mutex_ == nullptr) {
    stats_.publish_lock_failures++;
    stats_.failed++;
    return false;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, 20, nullptr) != RTOS_OK) {
    stats_.publish_lock_failures++;
    stats_.failed++;
    return false;
  }

  SensorSnapshot next{};
  next.reading = reading;
  next.taken_at_ms = rtos_->now_ms(rtos_->ctx);
  next.sequence = snapshot_.sequence + 1; // sequence counts successes, not attempts
  next.consecutive_failures = 0;
  snapshot_ = next;

  rtos_->mutex_unlock(rtos_->ctx, mutex_);

  stats_.published++;
  return true;
}

void SensorService::note_failure() {
  // The last good reading is kept — it is still the best estimate available — but the
  // failure count is published so a consumer can tell how stale it is.
  if (mutex_ == nullptr) {
    return;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, 20, nullptr) != RTOS_OK) {
    stats_.publish_lock_failures++;
    return;
  }
  snapshot_.consecutive_failures++;
  rtos_->mutex_unlock(rtos_->ctx, mutex_);
}

SensorSnapshot SensorService::snapshot() const {
  if (mutex_ == nullptr) {
    return SensorSnapshot{};
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, 20, nullptr) != RTOS_OK) {
    // Return an invalid snapshot rather than a torn one: a caller that gets
    // reading.valid == false keeps its previous value, which is the safe outcome.
    return SensorSnapshot{};
  }
  const SensorSnapshot copy = snapshot_;
  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return copy;
}

} // namespace node
