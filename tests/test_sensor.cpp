/* Tests for the AHT20 driver and the sensor service.
 *
 * The CRC check and the conversion maths are the two places a sensor driver can be
 * quietly wrong: a bad CRC that passes produces a confident wrong number, and an
 * overflowed conversion produces one that is merely odd. Both are pinned here against
 * values derived by hand rather than against the implementation.
 *
 * What changed with the AHT20 (this file used to test an SHT31, which is not the hardware
 * on the bench): the frame layout and the conversion formulas are different, so the
 * vectors below are new and worked through from the datasheet. The CRC is the one thing
 * that carried over untouched — the AHT20 specifies the same CRC-8 (poly 0x31, init 0xFF)
 * as the SHT3x — which is why its datasheet vector is still pinned here.
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

// Builds the 7-byte frame a real AHT20 sends: a status byte, five data bytes carrying 20
// bits of humidity and 20 bits of temperature, and one CRC over the first six.
//
// The status defaults to "calibrated, not busy" (0x08), which is what a device reports
// between conversions and what the service's bring-up poll is looking for.
std::vector<uint8_t> frame(uint32_t raw_temperature, uint32_t raw_humidity,
                           uint8_t status = 0x08) {
  std::vector<uint8_t> out(7, 0);
  out[0] = status;
  // The 20-bit fields share the third data byte: humidity's low nibble sits in the top half
  // and temperature's high nibble in the bottom half. This is the packing the driver
  // unpacks in read(), written the other way round so the two are independent.
  out[1] = static_cast<uint8_t>((raw_humidity >> 12) & 0xFFU);
  out[2] = static_cast<uint8_t>((raw_humidity >> 4) & 0xFFU);
  out[3] = static_cast<uint8_t>(((raw_humidity & 0x0FU) << 4) | ((raw_temperature >> 16) & 0x0FU));
  out[4] = static_cast<uint8_t>((raw_temperature >> 8) & 0xFFU);
  out[5] = static_cast<uint8_t>(raw_temperature & 0xFFU);
  out[6] = Aht20::crc8(out.data(), 6);
  return out;
}

// The raw values used by the tests below, each chosen so the expected result is a round
// number and can be checked by hand:
//
//   25.000 C  -> raw = (25 + 50) / 200 * 2^20 = 75 * 1048576 / 200 = 393216 exactly
//   40.000 %RH -> raw = 0.4 * 2^20 = 419430.4, so 419431 gives 40000.06 -> 40000 milli-RH
constexpr uint32_t kRaw25C = 393'216;
constexpr uint32_t kRaw40Rh = 419'431;

class SensorTest : public ::testing::Test {
 protected:
  FakeI2c bus;
  FakeRtos rtos;
  I2cRecovery recovery{bus.iface()};
  I2cArbiter arbiter{bus.iface(), rtos.iface(), &recovery};
};

TEST_F(SensorTest, CrcMatchesTheDatasheetExample) {
  // The SHT3x datasheet states that the CRC of 0xBE, 0xEF is 0x92, both of these were
  // worked through by hand from the algorithm (poly 0x31, init 0xFF) rather than taken from
  // this implementation, and the AHT20 specifies the same CRC-8 — which is why the check
  // that guards every frame in this driver is the same code the previous one used.
  const uint8_t example[2] = {0xBE, 0xEF};
  EXPECT_EQ(Aht20::crc8(example, 2), 0x92);

  const uint8_t zeros[2] = {0x00, 0x00};
  EXPECT_EQ(Aht20::crc8(zeros, 2), 0x81);

  // A one-bit change must change the CRC, which is the whole point of having one.
  const uint8_t flipped[2] = {0xBE, 0xEE};
  EXPECT_NE(Aht20::crc8(flipped, 2), 0x92);
}

TEST_F(SensorTest, ConversionHitsBothEndpoints) {
  // T = raw / 2^20 * 200 - 50, so raw 0 is the bottom of the range and raw 2^20 - 1 is a
  // hair under the top. The first is exact; the second is not, because 200000 * 1048575 /
  // 1048576 is 199999.8 and integer division truncates to 199999.
  EXPECT_EQ(Aht20::raw_to_milli_celsius(0), -50'000);
  EXPECT_EQ(Aht20::raw_to_milli_celsius(1'048'575), 149'999);

  // RH = raw / 2^20 * 100, same story at the top end: 99999.9 truncates to 99999.
  EXPECT_EQ(Aht20::raw_to_milli_rh(0), 0);
  EXPECT_EQ(Aht20::raw_to_milli_rh(1'048'575), 99'999);
}

TEST_F(SensorTest, ConversionIsPinnedAgainstHandArithmetic) {
  // 25.000 C, by hand: 200000 * 393216 = 78,643,200,000; / 1048576 = 75,000 exactly;
  // 75,000 - 50,000 = 25,000 milli-degrees.
  EXPECT_EQ(Aht20::raw_to_milli_celsius(393'216), 25'000);

  // 40.000 %RH: 100000 * 419431 = 41,943,100,000; / 1048576 = 39,999.94... = 40,000.06...
  // The quotient's floor is 40000 (1048576 x 40000 = 41,943,040,000, and the remainder
  // 60,000 is less than one more unit), so the answer is 40000 milli-RH.
  EXPECT_EQ(Aht20::raw_to_milli_rh(419'431), 40'000);

  // Mid-scale on both channels, where a 32-bit intermediate would have overflowed
  // silently: 200000 * 524288 is about 1.05e11, and 100000 * 524288 is half that. Both are
  // exact halves of the full scale, so both land on the midpoint by definition.
  EXPECT_EQ(Aht20::raw_to_milli_celsius(524'288), 50'000);
  EXPECT_EQ(Aht20::raw_to_milli_rh(524'288), 50'000);
}

TEST_F(SensorTest, HumidityIsClampedToWhatTheSensorCanProduce) {
  // Nothing in the datasheet's range can exceed 100 %RH, so a value that does means the raw
  // field was wrong in a way the CRC did not catch. Reporting a number the sensor cannot
  // produce would be worse than clamping — and here it is reachable, because the raw field
  // is 20 bits and a caller can hand the converter anything it likes.
  EXPECT_EQ(Aht20::raw_to_milli_rh(1'048'575), 99'999);
  EXPECT_EQ(Aht20::raw_to_milli_rh(0xFFFFFFFFU), 100'000);
}

TEST_F(SensorTest, StatusBitsAreNamed) {
  // Bit 7 is busy, bit 3 is the calibrated flag. Named here because the driver's bring-up
  // handshake and its frame check both read them, and a magic bit in either place is how a
  // driver ends up waiting for the wrong thing.
  EXPECT_TRUE(Aht20::busy(0x80));
  EXPECT_FALSE(Aht20::busy(0x08));
  EXPECT_TRUE(Aht20::calibrated(0x08));
  EXPECT_FALSE(Aht20::calibrated(0x80));
}

TEST_F(SensorTest, InitialiseSendsTheAht20CalibrationCommand) {
  Aht20 sensor(&arbiter);

  ASSERT_TRUE(sensor.begin_calibration());

  // 0xBE, not the AHT10's 0xE1: the AHT20 datasheet specifies 0xBE and the Linux kernel's
  // driver was corrected to match. If a future reader "fixes" this to 0xE1, this test is
  // the one that argues with them.
  EXPECT_EQ(bus.last_tx(), (std::vector<uint8_t>{0xBE, 0x08, 0x00}));
  EXPECT_EQ(bus.transfer_addrs(), (std::vector<uint16_t>{kAddrAht20}));
  EXPECT_EQ(sensor.stats().calibrations, 1u);
}

TEST_F(SensorTest, TriggerSendsTheMeasureCommand) {
  Aht20 sensor(&arbiter);

  ASSERT_TRUE(sensor.trigger());

  EXPECT_EQ(bus.last_tx(), (std::vector<uint8_t>{0xAC, 0x33, 0x00}));
  EXPECT_EQ(bus.transfer_addrs(), (std::vector<uint16_t>{kAddrAht20}));
}

TEST_F(SensorTest, ReadDecodesAValidFrame) {
  Aht20 sensor(&arbiter);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh));

  Aht20Reading reading;
  ASSERT_TRUE(sensor.read(&reading));

  EXPECT_TRUE(reading.valid);
  EXPECT_EQ(reading.status, 0x08u);
  EXPECT_EQ(reading.raw_temperature, kRaw25C);
  EXPECT_EQ(reading.raw_humidity, kRaw40Rh);
  EXPECT_EQ(reading.temperature_milli_c, 25'000);
  EXPECT_EQ(reading.humidity_milli_rh, 40'000);
  EXPECT_EQ(sensor.stats().measurements, 1u);
}

TEST_F(SensorTest, ReadKeepsAllTwentyBits) {
  // The reason the raw fields are uint32_t: 20-bit values do not fit a uint16_t, and
  // truncating the top nibble of the temperature turns 25 C into -25 C without any CRC
  // noticing. Raw 0xF0000 (983040) is 150 C — no real reading, but the arithmetic is what
  // is under test: 200000 * 983040 / 1048576 = 187500, i.e. 137.500 C.
  Aht20 sensor(&arbiter);
  bus.set_read_data(frame(983'040, 1'048'575));

  Aht20Reading reading;
  ASSERT_TRUE(sensor.read(&reading));

  EXPECT_EQ(reading.raw_temperature, 983'040u);
  EXPECT_EQ(reading.raw_humidity, 1'048'575u);
  EXPECT_EQ(reading.temperature_milli_c, 137'500);
  EXPECT_EQ(reading.humidity_milli_rh, 99'999);
}

TEST_F(SensorTest, ReadRejectsAFrameWithABadCrc) {
  Aht20 sensor(&arbiter);
  std::vector<uint8_t> reply = frame(kRaw25C, kRaw40Rh);
  reply[6] = static_cast<uint8_t>(reply[6] ^ 0x01); // break the CRC
  bus.set_read_data(reply);

  Aht20Reading reading;
  EXPECT_FALSE(sensor.read(&reading));

  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().crc_errors, 1u);
  EXPECT_EQ(sensor.stats().measurements, 0u);
}

TEST_F(SensorTest, ReadRejectsCorruptionInTheSharedDataByte) {
  // Byte 3 carries the low nibble of the humidity and the high nibble of the temperature,
  // so corrupting it must be caught even though neither 16-bit word is "the" damaged one.
  Aht20 sensor(&arbiter);
  std::vector<uint8_t> reply = frame(kRaw25C, kRaw40Rh);
  reply[3] = static_cast<uint8_t>(reply[3] ^ 0x10); // flip a humidity low-nibble bit
  bus.set_read_data(reply);

  Aht20Reading reading;
  EXPECT_FALSE(sensor.read(&reading));
  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().crc_errors, 1u);
}

TEST_F(SensorTest, ReadRejectsAFrameThatArrivedWhileTheDeviceWasBusy) {
  // A frame whose status byte says the conversion is still running is not a measurement,
  // and it is counted separately from a CRC error because the cause and the fix are
  // different: this one is a caller that read too early.
  Aht20 sensor(&arbiter);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh, 0x88)); // busy | calibrated

  Aht20Reading reading;
  EXPECT_FALSE(sensor.read(&reading));

  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().short_reads, 1u);
  EXPECT_EQ(sensor.stats().crc_errors, 0u);
  EXPECT_EQ(sensor.stats().measurements, 0u);
}

TEST_F(SensorTest, ReadThatHitsABusErrorIsNotAReading) {
  Aht20 sensor(&arbiter);
  bus.set_transfer_status(HAL_I2C_NACK);

  Aht20Reading reading;
  EXPECT_FALSE(sensor.read(&reading));

  EXPECT_FALSE(reading.valid);
  EXPECT_EQ(sensor.stats().bus_errors, 1u);
  EXPECT_EQ(sensor.stats().crc_errors, 0u);
}

TEST_F(SensorTest, StatusByteReadIsOneByte) {
  Aht20 sensor(&arbiter);
  bus.set_read_data(std::vector<uint8_t>{0x18}); // calibrated, not busy, some other bit set

  uint8_t status = 0;
  ASSERT_TRUE(sensor.status_byte(&status));
  EXPECT_EQ(status, 0x18u);
  EXPECT_EQ(bus.last_rx_len(), 1u);
  EXPECT_TRUE(Aht20::calibrated(status));
  EXPECT_FALSE(Aht20::busy(status));
}

// ---------------------------------------------------------------------------
// SensorService
// ---------------------------------------------------------------------------
TEST_F(SensorTest, ServiceInitCalibratesAndWaits) {
  SensorService service(&arbiter, rtos.iface());
  ASSERT_TRUE(service.valid());

  // The status reads answer "calibrated and idle" straight away, which is what a device
  // that has finished its self-calibration reports.
  bus.set_read_data(std::vector<uint8_t>{0x08});
  ASSERT_TRUE(service.init());

  // The calibration command is a write; the poll after it is a read.
  EXPECT_EQ(service.driver().stats().calibrations, 1u);
  // The settling wait happened, and it is on the caller's clock.
  EXPECT_GE(rtos.now_ms(), 10u);
}

TEST_F(SensorTest, ServiceInitReportsAnUncalibratedDeviceWithoutRefusingToRun) {
  // A device that is idle but never sets the calibrated bit: the kernel driver's reading of
  // the same datasheet accepts it (the SparkFun library's own comment says it is not clear
  // how and when the bit clears), so this returns true and records the fact rather than
  // vetoing a sensor that can measure.
  SensorService service(&arbiter, rtos.iface());
  bus.set_read_data(std::vector<uint8_t>{0x00}); // idle, not calibrated

  EXPECT_TRUE(service.init());
  EXPECT_EQ(service.driver().stats().uncalibrated_boots, 1u);
}

TEST_F(SensorTest, ServiceInitFailsWhenTheDeviceNeverGoesIdle) {
  // Busy forever. The poll is bounded, so this fails instead of hanging the boot — which is
  // the property that matters, because init() runs inside Node::init().
  SensorService service(&arbiter, rtos.iface());
  bus.set_read_data(std::vector<uint8_t>{0x88}); // busy | calibrated

  EXPECT_FALSE(service.init());
}

TEST_F(SensorTest, ServiceUpdateTriggersWaitsReadsAndPublishes) {
  SensorService service(&arbiter, rtos.iface(), kAddrAht20, 80);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh));

  ASSERT_TRUE(service.update());

  // A write of the measure command followed by a 7-byte read.
  EXPECT_EQ(bus.count(test::BusOp::Transfer), 2);
  const std::vector<uint16_t> addrs = bus.transfer_addrs();
  ASSERT_EQ(addrs.size(), 2u);
  EXPECT_EQ(addrs[0], kAddrAht20);
  EXPECT_EQ(addrs[1], kAddrAht20);
  // The frame is seven bytes, which is the SHT3x's six plus the status byte the AHT20
  // prefixes: a driver that kept the old length would parse a shifted frame.
  EXPECT_EQ(bus.last_rx_len(), 7u);

  const SensorSnapshot snapshot = service.snapshot();
  EXPECT_TRUE(snapshot.reading.valid);
  EXPECT_EQ(snapshot.sequence, 1u);
  // The raw fields, not the numbers handed to the fake, are what comes back: asserting the
  // converted values is what proves the decode and the conversion actually ran.
  EXPECT_EQ(snapshot.reading.raw_temperature, kRaw25C);
  EXPECT_EQ(snapshot.reading.raw_humidity, kRaw40Rh);
  EXPECT_EQ(snapshot.reading.temperature_milli_c, 25'000);
  EXPECT_EQ(snapshot.reading.humidity_milli_rh, 40'000);
  EXPECT_EQ(service.stats().published, 1u);

  // The conversion wait happened without the bus held, which is what lets the display task
  // use the bus during it.
  EXPECT_GE(rtos.now_ms(), 80u);
  EXPECT_TRUE(rtos.locks_balanced());
}

TEST_F(SensorTest, ServiceUpdateKeepsThePreviousReadingWhenTheBusFails) {
  SensorService service(&arbiter, rtos.iface(), kAddrAht20, 80);
  bus.set_read_data(frame(300'000, 700'000));
  ASSERT_TRUE(service.update());
  const SensorSnapshot first = service.snapshot();

  // Now the device stops answering.
  bus.set_transfer_status(HAL_I2C_NACK);
  EXPECT_FALSE(service.update());

  const SensorSnapshot second = service.snapshot();

  // The last good reading stays published and the sequence does not advance, so a consumer
  // can tell "no new data" from "a new but wrong value".
  EXPECT_TRUE(second.reading.valid);
  EXPECT_EQ(second.reading.humidity_milli_rh, first.reading.humidity_milli_rh);
  EXPECT_EQ(second.sequence, 1u);
  EXPECT_EQ(second.consecutive_failures, 1u);
  EXPECT_FALSE(second.fresh());
  EXPECT_EQ(service.stats().failed, 1u);
  EXPECT_EQ(service.stats().published, 1u);
}

TEST_F(SensorTest, ConsecutiveFailuresResetOnTheNextGoodReading) {
  SensorService service(&arbiter, rtos.iface(), kAddrAht20, 80);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh));
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
  SensorService service(&arbiter, rtos.iface(), kAddrAht20, 80);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh));
  ASSERT_TRUE(service.update());

  std::vector<uint8_t> corrupt = frame(400'000, 800'000);
  corrupt[6] = static_cast<uint8_t>(corrupt[6] ^ 0xFF);
  bus.set_read_data(corrupt);

  EXPECT_FALSE(service.update());
  EXPECT_EQ(service.snapshot().sequence, 1u);
  EXPECT_EQ(service.stats().failed, 1u);
}

TEST_F(SensorTest, SnapshotReportsInvalidRatherThanTornWhenTheLockFails) {
  SensorService service(&arbiter, rtos.iface(), kAddrAht20, 80);
  bus.set_read_data(frame(kRaw25C, kRaw40Rh));
  ASSERT_TRUE(service.update());
  ASSERT_EQ(service.snapshot().sequence, 1u);

  // A caller that cannot take a consistent copy gets "no data", not a half-updated struct
  // it might act on.
  rtos.fail_next_lock();
  const SensorSnapshot snapshot = service.snapshot();
  EXPECT_FALSE(snapshot.reading.valid);
  EXPECT_EQ(snapshot.sequence, 0u);
}

} // namespace
} // namespace node
