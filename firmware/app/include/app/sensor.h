/* sensor.h — the SHT31 temperature/humidity driver and the sensor task service.
 *
 * This is the node's primary I2C device and the second of the two bus users, so it is
 * half of what makes FR-0.2 a real requirement: without it the I2C arbiter would have
 * exactly one client and the mutex would be decoration.
 *
 * Readings are fixed-point, not float. The F103 has no FPU, so a float here means
 * linking soft-float routines into the image for values that are displayed as decimal
 * digits anyway; milli-units give 0.001 C resolution against a sensor whose noise is
 * nearer 0.01 C, and they make the conversion exactly comparable in a host test
 * instead of approximately comparable.
 */
#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/i2c_arbiter.h"
#include "rtos/rtos.h"

namespace node {

// One decoded measurement. `valid` is false when the bus failed or a CRC did not match,
// in which case the raw and converted fields are not meaningful and the caller must
// keep its previous value rather than publish a plausible-looking wrong number.
struct Sht31Reading {
  bool valid = false;
  uint16_t raw_temperature = 0;
  uint16_t raw_humidity = 0;
  int32_t temperature_milli_c = 0; // -45000 .. +130000
  int32_t humidity_milli_rh = 0;   // 0 .. 100000
};

struct Sht31Stats {
  uint32_t measurements = 0;
  uint32_t crc_errors = 0;
  uint32_t bus_errors = 0;
  uint32_t short_reads = 0;
};

class Sht31 {
 public:
  explicit Sht31(I2cArbiter *bus, uint16_t addr7 = kAddrSht31);

  Sht31(const Sht31 &) = delete;
  Sht31 &operator=(const Sht31 &) = delete;

  // Soft reset (command 0x30A2). Returns false if the bus rejected it.
  bool reset();

  // Start a single measurement, high repeatability, clock stretching disabled
  // (command 0x2400). Returns immediately; the conversion takes about 15 ms, so the
  // caller waits before read() and the bus is free for other tasks meanwhile.
  bool trigger();

  // Fetch the 6-byte result, checking both CRCs. On any failure `out->valid` is false
  // and the appropriate counter is bumped.
  bool read(Sht31Reading *out);

  const Sht31Stats &stats() const { return stats_; }

  // CRC-8 as the SHT3x datasheet specifies it: polynomial 0x31, initial value 0xFF,
  // no reflection, no final XOR. Public and static so a host test can pin it against
  // the datasheet's own example (0xBE, 0xEF -> 0x92).
  static uint8_t crc8(const uint8_t *data, size_t len);

  // Datasheet conversion, in the units described above. Exposed for testing.
  static int32_t raw_to_milli_celsius(uint16_t raw);
  static int32_t raw_to_milli_rh(uint16_t raw);

 private:
  I2cArbiter *bus_;
  uint16_t addr7_;
  Sht31Stats stats_{};
};

struct SensorSnapshot {
  Sht31Reading reading{};
  uint32_t taken_at_ms = 0;
  uint32_t sequence = 0; // increments on every successful measurement

  // Failed poll cycles since the last success.
  //
  // Needed because `reading.valid` alone cannot express staleness: once a measurement has
  // succeeded, a later bus failure leaves the last good value in place (deliberately —
  // it is still the best estimate available) and `valid` stays true. Without this
  // counter a consumer cannot tell a fresh reading from one that has been frozen for a
  // minute, and a frozen number on a display is indistinguishable from a working node.
  uint32_t consecutive_failures = 0;

  // True when the reading is both present and current.
  bool fresh() const { return reading.valid && consecutive_failures == 0; }
};

struct SensorServiceStats {
  uint32_t polls = 0;
  uint32_t published = 0;
  uint32_t failed = 0;
  uint32_t publish_lock_failures = 0;
};

// The sensor task's logic, with no RTOS task plumbing of its own so that a host test can
// drive it one cycle at a time.
class SensorService {
 public:
  SensorService(I2cArbiter *bus, const rtos_iface_t *rtos, uint16_t addr7 = kAddrSht31,
                uint32_t conversion_ms = kSht31ConversionMs);

  SensorService(const SensorService &) = delete;
  SensorService &operator=(const SensorService &) = delete;

  bool valid() const { return mutex_ != nullptr; }

  // Soft-reset the device so it starts from a known state.
  bool init();

  // One poll cycle: trigger, wait out the conversion, read, publish.
  //
  // The I2C mutex is held for the trigger and the read but deliberately NOT across the
  // conversion delay. Holding it for the full 20 ms would block the display task on a
  // bus it is not using, which is exactly the class of stall the arbiter exists to
  // avoid.
  bool update();

  // A consistent copy of the latest reading, safe to call from any task.
  SensorSnapshot snapshot() const;

  const SensorServiceStats &stats() const { return stats_; }
  const Sht31 &driver() const { return driver_; }

 private:
  // Publish a failed cycle: the reading is kept, the staleness counter advances.
  void note_failure();

  I2cArbiter *bus_;
  const rtos_iface_t *rtos_;
  Sht31 driver_;
  uint32_t conversion_ms_;
  rtos_mutex_t *mutex_;
  SensorSnapshot snapshot_{};
  SensorServiceStats stats_{};
};

} // namespace node

#endif // APP_SENSOR_H
