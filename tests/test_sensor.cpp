/* Tests for the SHT31 driver and sensor service.
 *
 * The CRC check and the conversion maths are the two places a sensor driver can be
 * quietly wrong: a bad CRC that passes produces a confident wrong number, and an
 * overflowed conversion produces one that is merely odd. Both are pinned here against
 * values derived by hand rather than against the implementation.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "app/board.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "app/sensor.h"
#include "support/fake_i2c.h"
#include "support/fake_rtos.h"

namespace node {
namespace {

using test::FakeI2c;
using test::FakeRtos;

// Builds the 6-byte reply a real SHT31 sends: two 16-bit words, each with its own CRC.
std::vector<uint8_t> measurement(uint16_t raw_temperature, uint16_t raw_humidity) {
  std::vector<uint8_t> out(6, 0);
  out[0] = static_cast<uint8_t>(raw_temperature >> 8);
  out[1] = static_cast<uint8_t>(raw_temperature & 0xFF);
  out[2] = Sht31::crc8(out.data(), 2);
  out[3] = static_cast<uint8_t>(raw_humidity >> 8);
  out[4] = static_cast<uint8_t>(raw_humidity & 0xFF);
  out[5] = Sht31::crc8(&out[3], 2);
  return out;
}

class SensorTest : public ::testing::Test {
 protected:
  FakeI2c bus;
  FakeRtos rtos;
  I2cRecovery recovery{bus.iface()};
  I2cArbiter arbiter{bus.iface(), rtos.iface(), &recovery};
};

TEST_F(SensorTest, CrcMatchesTheDatasheetExample) {
  // The SHT3x datasheet states that the CRC of 0xBE, 0xEF is 0x92, and both of these
  // were worked through by hand from the algorithm (poly 0x31, init 0xFF) rather than
  // taken from this implementation, which is what makes them a test rather than a
  // restatement.
  const uint8_t example[2] = {0xBE, 0xEF};
  EXPECT_EQ(Sht31::crc8(example, 2), 0x92);

  const uint8_t zeros[2] = {0x00, 0x00};
  EXPECT_EQ(Sht31::crc8(zeros, 2), 0x81);

  // A one-bit change must change the CRC, which is the whole point of having one.
  const uint8_t flipped[2] = {0xBE, 0xEE};
  EXPECT_NE(Sht31::crc8(flipped, 2), 0x92);
}

TEST_F(SensorTest, ConversionHitsBothEndpoints) {
  EXPECT_EQ(Sht31::raw_to_milli_celsius(0), -45'000);
  EXPECT_EQ(Sht31::raw_to_milli_celsius(65'535), 130'000);
  EXPECT_EQ(Sht31::raw_to_milli_rh(0), 0);
  EXPECT_EQ(Sht31::raw_to_milli_rh(65'535), 100'000);

  // Mid-scale is where a 32-bit intermediate would have overflowed silently.
  // 175000 * 32768 / 65535 is 87501 (the remainder is 21965), so the endpoint-exact
  // arithmetic lands one milli-degree above the nominal 42.500 C midpoint.
  EXPECT_EQ(Sht31::raw_to_milli_celsius(32'768), 42'501);
  EXPECT_EQ(Sht31::raw_to_milli_rh(32'768), 50'000);
}

TEST_F(SensorTest, HumidityIsClampedToWhatTheSensorCanProduce) {
  // Nothing in the datasheet's range can exceed 100 %RH, so a value that does means the
  // raw word was wrong in a way the CRC did not catch. Reporting a number the sensor
  // cannot produce would be worse than clamping.
  EXPECT_EQ(Sht31::raw_to_milli_rh(65'535), 100'000);
  EXPECT_LE(Sht31::raw_to_milli_rh(65'534), 100'000);
}

TEST_F(SensorTest, TriggerSendsTheMeasureCommand) {
  Sht31 sensor(&arbiter);

  ASSERT_TRUE(sensor.trigger());

  EXPECT_EQ(bus.last_tx(), (std::vector<uint8_t>{0x24, 0x00}));
  EXPECT_EQ(bus.transfer_addrs(), (std::vector<uint16_t>{kAddrSht31}));
}

TEST_F(SensorTest, ResetSendsTheSoftResetCommand) {
  Sht31 sensor(&arbiter);

  ASSERT_TRUE(sensor.reset());

  EXPECT_EQ(bus.last_tx(), (std::vector<uint8_t>{0x30, 0xA2}));
}

TEST_F(SensorTest, ReadDecodesAValidMeasurement) {
  Sht31 sensor(&arbiter);
  bus.set_read_data(measurement(32'768, 32'768));

  Sht31Reading reading;
  ASSERT_TRUE(sensor.read(&reading));

  EXPECT_TRUE(reading.valid);
  EXPECT_EQ(reading.raw_temperature, 32'768);
  EXPECT_EQ(reading.raw_humidity, 32'768);
  EXPECT_EQ(reading.temperature_milli_c, 42'501);
  EXPECT_EQ(reading.humidity_milli_rh, 50'000);
  EXPECT_EQ(sensor.stats().measurements, 1u);
}

TEST_F(SensorTest, ReadRejectsACorruptedTemperatureWord) {
  Sht31 sensor(&arbiter);
  std::vector<uint8_t> reply = measurement(32'768, 32'768);
  reply[2] = static_cast<uint8_t>(reply[2] ^ 0x01); // break the first CRC
  bus.set_read_data(reply);

  Sht31Reading reading;
  EXPECT_FALSE(sensor.read(&reading));

  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().crc_errors, 1u);
  EXPECT_EQ(sensor.stats().measurements, 0u);
}

TEST_F(SensorTest, ReadRejectsACorruptedHumidityWord) {
  // Each word carries its own CRC, so corruption in the second half must be caught too.
  Sht31 sensor(&arbiter);
  std::vector<uint8_t> reply = measurement(32'768, 32'768);
  reply[5] = static_cast<uint8_t>(reply[5] ^ 0x80);
  bus.set_read_data(reply);

  Sht31Reading reading;
  EXPECT_FALSE(sensor.read(&reading));
  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().crc_errors, 1u);
}

TEST_F(SensorTest, ReadThatHitsABusErrorIsNotAReading) {
  Sht31 sensor(&arbiter);
  bus.set_transfer_status(HAL_I2C_NACK);

  Sht31Reading reading;
  EXPECT_FALSE(sensor.read(&reading));

  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().bus_errors, 1u);
  EXPECT_EQ(sensor.stats().crc_errors, 0u);
}

// ---------------------------------------------------------------------------
// SensorService
// ---------------------------------------------------------------------------
TEST_F(SensorTest, ServiceInitResetsTheDeviceAndWaits) {
  SensorService service(&arbiter, rtos.iface());
  ASSERT_TRUE(service.valid());

  ASSERT_TRUE(service.init());

  EXPECT_EQ(bus.last_tx(), (std::vector<uint8_t>{0x30, 0xA2}));
  // The soft reset needs a moment before the device accepts a command.
  EXPECT_GE(rtos.now_ms(), 2u);
}

TEST_F(SensorTest, ServiceUpdateTriggersWaitsReadsAndPublishes) {
  SensorService service(&arbiter, rtos.iface(), kAddrSht31, 20);
  bus.set_read_data(measurement(32'768, 32'768));

  ASSERT_TRUE(service.update());

  // A write of the measure command followed by a 6-byte read.
  EXPECT_EQ(bus.count(test::BusOp::Transfer), 2);
  const std::vector<uint16_t> addrs = bus.transfer_addrs();
  ASSERT_EQ(addrs.size(), 2u);
  EXPECT_EQ(addrs[0], kAddrSht31);
  EXPECT_EQ(addrs[1], kAddrSht31);

  const SensorSnapshot snapshot = service.snapshot();
  EXPECT_TRUE(snapshot.reading.valid);
  EXPECT_EQ(snapshot.sequence, 1u);
  // The raw words, not the numbers handed to the fake, are what comes back: asserting
  // the converted values is what proves the decode and the conversion actually ran.
  EXPECT_EQ(snapshot.reading.raw_temperature, 32'768);
  EXPECT_EQ(snapshot.reading.raw_humidity, 32'768);
  EXPECT_EQ(snapshot.reading.temperature_milli_c, 42'501);
  EXPECT_EQ(snapshot.reading.humidity_milli_rh, 50'000);
  EXPECT_EQ(service.stats().published, 1u);

  // The conversion wait happened without the bus held, which is what lets the display
  // task use the bus during it.
  EXPECT_GE(rtos.now_ms(), 20u);
  EXPECT_TRUE(rtos.locks_balanced());
}

TEST_F(SensorTest, ServiceUpdateKeepsThePreviousReadingWhenTheBusFails) {
  SensorService service(&arbiter, rtos.iface(), kAddrSht31, 20);
  bus.set_read_data(measurement(20'000, 55'000));
  ASSERT_TRUE(service.update());
  const SensorSnapshot first = service.snapshot();

  // Now the device stops answering.
  bus.set_transfer_status(HAL_I2C_NACK);
  EXPECT_FALSE(service.update());

  const SensorSnapshot second = service.snapshot();

  // The last good reading stays published and the sequence does not advance, so a
  // consumer can tell "no new data" from "a new but wrong value".
  EXPECT_TRUE(second.reading.valid);
  EXPECT_EQ(second.reading.humidity_milli_rh, first.reading.humidity_milli_rh);
  EXPECT_EQ(second.sequence, 1u);
  EXPECT_EQ(second.consecutive_failures, 1u);
  EXPECT_FALSE(second.fresh());
  EXPECT_EQ(service.stats().failed, 1u);
  EXPECT_EQ(service.stats().published, 1u);
}

TEST_F(SensorTest, ConsecutiveFailuresResetOnTheNextGoodReading) {
  SensorService service(&arbiter, rtos.iface(), kAddrSht31, 20);
  bus.set_read_data(measurement(32'768, 32'768));
  ASSERT_TRUE(service.update());
  ASSERT_EQ(service.snapshot().consecutive_failures, 0u);

  bus.set_transfer_status(HAL_I2C_NACK);
  ASSERT_FALSE(service.update());
  ASSERT_FALSE(service.update());
  ASSERT_EQ(service.snapshot().consecutive_failures, 2u);

  bus.set_transfer_status(HAL_I2C_OK);
  ASSERT_TRUE(service.update());

  const SensorSnapshot recovered = service.snapshot();
  EXPECT_EQ(recovered.consecutive_failures, 0u);
  EXPECT_TRUE(recovered.fresh());
  EXPECT_EQ(recovered.sequence, 2u);
}

TEST_F(SensorTest, ServiceUpdateDoesNotAdvanceTheSequenceOnABadCrc) {
  SensorService service(&arbiter, rtos.iface(), kAddrSht31, 20);
  bus.set_read_data(measurement(20'000, 55'000));
  ASSERT_TRUE(service.update());

  std::vector<uint8_t> corrupt = measurement(21'000, 56'000);
  corrupt[2] = static_cast<uint8_t>(corrupt[2] ^ 0xFF);
  bus.set_read_data(corrupt);

  EXPECT_FALSE(service.update());
  EXPECT_EQ(service.snapshot().sequence, 1u);
  EXPECT_EQ(service.stats().failed, 1u);
}

TEST_F(SensorTest, SnapshotReportsInvalidRatherThanTornWhenTheLockFails) {
  SensorService service(&arbiter, rtos.iface(), kAddrSht31, 20);
  bus.set_read_data(measurement(20'000, 55'000));
  ASSERT_TRUE(service.update());
  ASSERT_EQ(service.snapshot().sequence, 1u);

  // A caller that cannot take a consistent copy gets "no data", not a half-updated
  // struct it might act on.
  rtos.fail_next_lock();
  const SensorSnapshot snapshot = service.snapshot();
  EXPECT_FALSE(snapshot.reading.valid);
  EXPECT_EQ(snapshot.sequence, 0u);
}

} // namespace
} // namespace node
