/* sensor.h — the AHT20 temperature/humidity driver and the sensor task service.
 *
 * This is the node's primary I2C device and the second of the two bus users, so it is
 * half of what makes FR-0.2 a real requirement: without it the I2C arbiter would have
 * exactly one client and the mutex would be decoration.
 *
 * The device is an "AHT20 + BMP280" combo module: the AHT20 at 0x38 (this driver) and a
 * BMP280 at 0x76 which is present on the bus and deliberately never read — see
 * app/board.h's address list for why pressure is not on any requirement path.
 *
 * Readings are fixed-point, not float. The F103 has no FPU, so a float here means
 * linking soft-float routines into the image for values that are displayed as decimal
 * digits anyway; milli-units give 0.001 C resolution against a sensor whose accuracy is
 * +-0.3 C, and they make the conversion exactly comparable in a host test instead of
 * approximately comparable.
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
struct Aht20Reading {
  bool valid = false;

  // The AHT20's two quantities are 20 bits each, not 16: the frame packs 20 bits of
  // relative humidity and 20 bits of temperature into five bytes, with the low nibble of
  // the humidity sharing a byte with the high nibble of the temperature. So the raw fields
  // are uint32_t, and truncating them to 16 would silently discard the four most
  // significant bits — the difference between a plausible reading and a wrong one.
  uint32_t raw_temperature = 0;
  uint32_t raw_humidity = 0;

  // The status byte that came with the frame, kept because it is the only evidence the
  // device offers that this frame is a completed measurement rather than a leftover:
  // bit 7 is busy, bit 3 is the calibrated flag. Exposed rather than discarded so a
  // bring-up session can see the device's own view of itself.
  uint8_t status = 0;

  int32_t temperature_milli_c = 0; // -50000 .. +150000
  int32_t humidity_milli_rh = 0;   // 0 .. 100000
};

struct Aht20Stats {
  uint32_t measurements = 0;
  uint32_t crc_errors = 0;
  uint32_t bus_errors = 0;
  uint32_t short_reads = 0;       // frames rejected because the measurement was not complete
  uint32_t calibrations = 0;      // calibration commands the device acknowledged
  uint32_t uncalibrated_boots = 0; // bring-ups where the calibrated bit never appeared
};

class Aht20 {
 public:
  explicit Aht20(I2cArbiter *bus, uint16_t addr7 = kAddrAht20);

  Aht20(const Aht20 &) = delete;
  Aht20 &operator=(const Aht20 &) = delete;

  // Ask the device to calibrate: command 0xBE 0x08 0x00, which the AHT20 datasheet
  // specifies and which is what the Linux kernel's aht10 driver was corrected to send.
  // (The AHT10 and the shared Adafruit-family drivers use 0xE1 for the same field; both
  // have been seen to work on AHT20s, but the datasheet is the authority here.)
  //
  // Returns false if the bus rejected the command. What this does NOT do is wait — the
  // driver has no clock, deliberately, so the sequence command -> wait -> poll lives in
  // SensorService::init(), which does have one. See the header of that function.
  bool begin_calibration();

  // Start a measurement (command 0xAC 0x33 0x00). Returns immediately; the conversion
  // takes about 80 ms (kAht20ConversionMs), so the caller waits before read() and the bus
  // is free for other tasks meanwhile.
  bool trigger();

  // Fetch the 7-byte frame and check its CRC. On any failure `out->valid` is false and the
  // appropriate counter is bumped.
  bool read(Aht20Reading *out);

  // The status byte alone, as a bare 1-byte read. Used by the bring-up handshake and
  // exposed because it is the cheapest way for a session to ask the device how it sees
  // itself.
  bool status_byte(uint8_t *out);

  // True if the status byte says a conversion is still in progress (bit 7). The driver
  // does not need it — SensorService sleeps the datasheet's conversion time instead, which
  // is simpler and does not spend bus time — but a caller that would rather poll than
  // sleep can use it, and the flag is worth naming rather than leaving as a magic bit.
  static bool busy(uint8_t status);
  static bool calibrated(uint8_t status);

  const Aht20Stats &stats() const { return stats_; }

  // Recorded by SensorService::init() when the bring-up poll found the device idle without
  // ever reporting the calibrated bit. A setter rather than a flag the service reaches into,
  // because the statistics belong to the driver that produced the reads.
  void note_uncalibrated() { stats_.uncalibrated_boots++; }

  // CRC-8 as the AHT20 specifies it: polynomial 0x31, initial value 0xFF, no reflection, no
  // final XOR. The same algorithm as the SHT3x's, which is why this code survived the
  // sensor change unaltered. Public and static so a host test can pin it against the
  // datasheet's own example (0xBE, 0xEF -> 0x92).
  static uint8_t crc8(const uint8_t *data, size_t len);

  // Datasheet conversion, in the units described above. Exposed for testing.
  //
  //   RH(%)  = raw / 2^20 * 100
  //   T(C)   = raw / 2^20 * 200 - 50
  //
  // The 20-bit full scale is why the intermediates are 64-bit: 200000 * 1048575 is about
  // 2.1e11, which is well past int32.
  static int32_t raw_to_milli_celsius(uint32_t raw);
  static int32_t raw_to_milli_rh(uint32_t raw);

 private:
  I2cArbiter *bus_;
  uint16_t addr7_;
  Aht20Stats stats_{};
};

struct SensorSnapshot {
  Aht20Reading reading{};
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
  SensorService(I2cArbiter *bus, const rtos_iface_t *rtos, uint16_t addr7 = kAddrAht20,
                uint32_t conversion_ms = kAht20ConversionMs);

  SensorService(const SensorService &) = delete;
  SensorService &operator=(const SensorService &) = delete;

  bool valid() const { return mutex_ != nullptr; }

  // Bring the device to a ready state: send the calibration command, wait, then poll the
  // status byte until the device reports itself idle.
  //
  // The poll is bounded, because an AHT20 that never stops reporting "busy" must not hang
  // the boot — this runs inside Node::init(), where a hang presents as a board that never
  // comes up with nothing on the wire to say why.
  //
  // The calibrated bit (status bit 3) is checked and counted but is NOT required for this
  // to return true. That is the Linux kernel driver's reading of the same datasheet: it
  // accepts a device that is simply not busy, and the SparkFun library's own comment warns
  // that "it's not clear how and when the cal bit clears". Refusing to run on a device that
  // can measure is a worse failure than running on one whose self-report is incomplete, so
  // the bit is carried into every reading (Aht20Reading::status) and counted in
  // Aht20Stats::uncalibrated_boots instead of being turned into a veto.
  bool init();

  // One poll cycle: trigger, wait out the conversion, read, publish.
  //
  // The I2C mutex is held for the trigger and the read but deliberately NOT across the
  // conversion delay. Holding it for the full 80 ms would block the display task on a
  // bus it is not using, which is exactly the class of stall the arbiter exists to
  // avoid.
  bool update();

  // A consistent copy of the latest reading, safe to call from any task.
  SensorSnapshot snapshot() const;

  const SensorServiceStats &stats() const { return stats_; }
  const Aht20 &driver() const { return driver_; }

 private:
  // Publish a failed cycle: the reading is kept, the staleness counter advances.
  void note_failure();

  I2cArbiter *bus_;
  const rtos_iface_t *rtos_;
  Aht20 driver_;
  uint32_t conversion_ms_;
  rtos_mutex_t *mutex_;
  SensorSnapshot snapshot_{};
  SensorServiceStats stats_{};
};

} // namespace node

#endif // APP_SENSOR_H
