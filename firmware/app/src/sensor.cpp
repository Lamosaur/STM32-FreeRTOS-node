#include "app/sensor.h"

namespace node {
namespace {

// AHT20 command bytes, in the order they go on the wire.
//
//   initialize  0xBE 0x08 0x00 — calibrate. (The AHT10 and the shared Adafruit-family
//               drivers send 0xE1 0x08 0x00 instead; see the header for why this follows
//               the AHT20 datasheet.)
//   measure     0xAC 0x33 0x00 — trigger a temperature + humidity conversion.
constexpr uint8_t kCmdInitialize = 0xBE;
constexpr uint8_t kCmdInitializeArg = 0x08;
constexpr uint8_t kCmdTrigger = 0xAC;
constexpr uint8_t kCmdTriggerArg = 0x33;
constexpr uint8_t kCmdNop = 0x00;

// Status byte bits.
constexpr uint8_t kStatusBusy = 0x80;
constexpr uint8_t kStatusCalibrated = 0x08;

// One frame is 7 bytes: a status byte, five data bytes, and a CRC over the first six.
//
// This is NOT the SHT3x layout that the previous driver read (two 16-bit words, each with
// its own CRC byte). The AHT20 packs 20 bits of humidity and 20 bits of temperature into
// the five data bytes and appends a single CRC for the whole frame:
//
//   [0] status
//   [1] humidity[19:12]
//   [2] humidity[11:4]
//   [3] humidity[3:0] << 4 | temperature[19:16]
//   [4] temperature[15:8]
//   [5] temperature[7:0]
//   [6] CRC-8 over bytes 0..5
//
// A driver that kept the SHT3x framing would read zero humidity and a CRC mismatch on
// every frame — which fails loudly rather than quietly, and is the reason this comment
// quotes the datasheet's layout rather than paraphrasing it.
constexpr size_t kFrameBytes = 7;
constexpr size_t kCrcCoveredBytes = 6;

// The 20-bit full scale of both quantities, and the spans the datasheet's formulas use.
constexpr int64_t kRawFullScale = 1'048'576; // 2^20
constexpr int64_t kTemperatureOffsetMilli = -50'000;
constexpr int64_t kTemperatureSpanMilli = 200'000;
constexpr int64_t kHumiditySpanMilliRh = 100'000;
constexpr int64_t kHumidityMaxMilliRh = 100'000;

// The calibration handshake: a settling wait after the command, then a bounded poll of
// the status byte. Ten attempts at 2 ms is 20 ms of polling, comfortably past the "few
// milliseconds" the device needs to calibrate; what the bound is for is the device that
// never goes idle, which without it would hang SensorService::init() and with it the boot.
constexpr uint32_t kCalibrationDelayMs = 10;
constexpr uint32_t kCalibrationPollDelayMs = 2;
constexpr int kCalibrationPollAttempts = 10;

} // namespace

uint8_t Aht20::crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        // Cast before the shift: `crc << 1` promotes to int, and combining that with the
        // unsigned polynomial is an int-to-unsigned conversion.
        crc = static_cast<uint8_t>((static_cast<uint32_t>(crc) << 1) ^ 0x31U);
      } else {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
  }
  return crc;
}

bool Aht20::busy(uint8_t status) { return (status & kStatusBusy) != 0U; }

bool Aht20::calibrated(uint8_t status) { return (status & kStatusCalibrated) != 0U; }

int32_t Aht20::raw_to_milli_celsius(uint32_t raw) {
  // The datasheet's formula, in integers: raw / 2^20 * 200 - 50, scaled to milli-degrees.
  // The product is taken in 64 bits because 200000 * 1048575 is about 2.1e11.
  const int64_t product = kTemperatureSpanMilli * static_cast<int64_t>(raw);
  return static_cast<int32_t>(kTemperatureOffsetMilli + (product / kRawFullScale));
}

int32_t Aht20::raw_to_milli_rh(uint32_t raw) {
  const int64_t product = kHumiditySpanMilliRh * static_cast<int64_t>(raw);
  const int64_t milli_rh = product / kRawFullScale;
  // The datasheet's humidity range is 0..100 %RH, so a result outside it would mean the
  // raw field was corrupt despite passing CRC. Clamped rather than reported, for the same
  // reason the previous driver clamped: a number the sensor cannot produce is not a
  // measurement, and reporting it invites a control loop to act on it.
  if (milli_rh < 0) {
    return 0;
  }
  if (milli_rh > kHumidityMaxMilliRh) {
    return static_cast<int32_t>(kHumidityMaxMilliRh);
  }
  return static_cast<int32_t>(milli_rh);
}

Aht20::Aht20(I2cArbiter *bus, uint16_t addr7) : bus_(bus), addr7_(addr7) {}

bool Aht20::status_byte(uint8_t *out) {
  if (out == nullptr) {
    return false;
  }

  uint8_t status = 0;
  if (bus_->read(addr7_, &status, 1) != BusResult::Ok) {
    stats_.bus_errors++;
    return false;
  }
  *out = status;
  return true;
}

bool Aht20::begin_calibration() {
  const uint8_t command[3] = {kCmdInitialize, kCmdInitializeArg, kCmdNop};
  if (bus_->write(addr7_, command, sizeof(command)) != BusResult::Ok) {
    stats_.bus_errors++;
    return false;
  }
  stats_.calibrations++;
  return true;
}

bool Aht20::trigger() {
  const uint8_t command[3] = {kCmdTrigger, kCmdTriggerArg, kCmdNop};
  const bool ok = bus_->write(addr7_, command, sizeof(command)) == BusResult::Ok;
  if (!ok) {
    stats_.bus_errors++;
  }
  return ok;
}

bool Aht20::read(Aht20Reading *out) {
  Aht20Reading reading{};

  uint8_t raw[kFrameBytes] = {0};
  if (bus_->read(addr7_, raw, sizeof(raw)) != BusResult::Ok) {
    stats_.bus_errors++;
    *out = reading;
    return false;
  }

  // One CRC over the status byte and the five data bytes. Checking the whole frame rather
  // than only the data means corruption in the status byte is caught too, and the status
  // byte is not decoration: it carries the busy and calibrated bits the conversion below
  // relies on.
  if (crc8(raw, kCrcCoveredBytes) != raw[kCrcCoveredBytes]) {
    stats_.crc_errors++;
    *out = reading;
    return false;
  }

  reading.status = raw[0];
  if (busy(reading.status)) {
    // A frame that arrived while the device says it is still converting is not a
    // measurement. Counted separately from a CRC error because the causes are different:
    // a CRC error is a bus or timing fault, this is a caller that read too early — and the
    // two want different fixes.
    stats_.short_reads++;
    *out = reading;
    return false;
  }

  reading.raw_humidity = ((static_cast<uint32_t>(raw[1]) << 12) |
                          (static_cast<uint32_t>(raw[2]) << 4) |
                          (static_cast<uint32_t>(raw[3]) >> 4));
  reading.raw_temperature = ((static_cast<uint32_t>(raw[3] & 0x0FU) << 16) |
                             (static_cast<uint32_t>(raw[4]) << 8) |
                             static_cast<uint32_t>(raw[5]));

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
  // The bring-up handshake, in three parts, with the waiting done here rather than in the
  // driver so that the driver stays free of any clock:
  //
  //   1. the calibration command,
  //   2. a settling wait — the device needs a few milliseconds before it will answer,
  //   3. a bounded poll of the status byte until it reports itself idle.
  if (!driver_.begin_calibration()) {
    return false;
  }
  rtos_->delay_ms(rtos_->ctx, kCalibrationDelayMs);

  bool saw_calibrated = false;
  for (int attempt = 0; attempt < kCalibrationPollAttempts; ++attempt) {
    uint8_t status = 0;
    if (!driver_.status_byte(&status)) {
      // A read that failed is not the same as a device that is busy: the bus is the
      // problem, and retrying the same read is what the arbiter already does. Counted by
      // the driver as a bus error and retried here until the budget runs out.
      rtos_->delay_ms(rtos_->ctx, kCalibrationPollDelayMs);
      continue;
    }

    saw_calibrated = saw_calibrated || Aht20::calibrated(status);
    if (!Aht20::busy(status)) {
      if (!saw_calibrated) {
        // Idle but never once calibrated: allowed to proceed (see the header), recorded so
        // that a bench session can see it rather than having to infer it.
        driver_.note_uncalibrated();
      }
      return true;
    }
    rtos_->delay_ms(rtos_->ctx, kCalibrationPollDelayMs);
  }

  // Never went idle inside the budget. Reported as a failed init so the caller treats the
  // sensor as absent rather than as one whose readings are merely late.
  return false;
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

  Aht20Reading reading{};
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
