/* Tests for the host link: the text appender and the command set.
 *
 * The appender exists instead of printf so that output is byte-exactly assertable, and
 * these tests are what cashes that in: they check the exact characters a host would
 * receive, including the awkward cases (INT32_MIN, a truncated line, a command longer
 * than the buffer).
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "support/fake_i2c.h"
#include "support/fake_peripherals.h"
#include "support/fake_rtos.h"

namespace node {
namespace {

using test::FakeI2c;
using test::FakeRtos;
using test::FakeSystem;
using test::FakeUart;

std::string render(int32_t milli, uint32_t decimals) {
  char buffer[32];
  LineBuilder builder(buffer, sizeof(buffer));
  builder.fixed(milli, decimals);
  return std::string(builder.c_str());
}

TEST(LineBuilderTest, AppendsTextAndNumbers) {
  char buffer[64];
  LineBuilder builder(buffer, sizeof(buffer));

  builder.str("n=").u32(0).str(",").u32(4294967295U).str(" i=").i32(-7);

  EXPECT_STREQ(builder.c_str(), "n=0,4294967295 i=-7");
  EXPECT_FALSE(builder.truncated());
}

TEST(LineBuilderTest, HandlesTheMostNegativeInteger) {
  // Negating INT32_MIN overflows, so this is the case a naive implementation gets
  // wrong and then reports as a positive number of similar size.
  char buffer[32];
  LineBuilder builder(buffer, sizeof(buffer));
  builder.i32(INT32_MIN);
  EXPECT_STREQ(builder.c_str(), "-2147483648");
}

TEST(LineBuilderTest, RendersFixedPointValues) {
  EXPECT_EQ(render(42'500, 2), "42.50");
  EXPECT_EQ(render(-4'500, 1), "-4.5");
  EXPECT_EQ(render(0, 2), "0.00");
  EXPECT_EQ(render(-10, 2), "-0.01");
  EXPECT_EQ(render(42'500, 0), "42");
  EXPECT_EQ(render(-123'456, 3), "-123.456");
}

TEST(LineBuilderTest, FixedPointFractionsAreZeroPadded) {
  // 42.05 is the case a naive implementation renders as "42.5", turning a real
  // measurement into a different one.
  EXPECT_EQ(render(42'050, 2), "42.05");
  EXPECT_EQ(render(-42'050, 2), "-42.05");
  EXPECT_EQ(render(1'005, 2), "1.00");

  // Asking for more digits than the value carries is clamped, not padded with zeros.
  EXPECT_EQ(render(42'500, 5), "42.500");
}

TEST(LineBuilderTest, TruncationIsReportedRatherThanHidden) {
  char buffer[8];
  LineBuilder builder(buffer, sizeof(buffer));

  builder.str("0123456789");

  EXPECT_TRUE(builder.truncated());
  EXPECT_EQ(builder.size(), 7u);
  EXPECT_EQ(std::strlen(builder.c_str()), 7u);
}

// ---------------------------------------------------------------------------
// CommsService
// ---------------------------------------------------------------------------
class CommsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sources.system = system.iface();
    sources.arbiter = &arbiter;
    sources.sensor = &sensor;
    sources.identity = &identity;
    service = std::make_unique<CommsService>(uart.iface(), &arbiter, rtos.iface(), sources);
  }

  FakeI2c bus;
  FakeRtos rtos;
  FakeUart uart;
  FakeSystem system;
  I2cRecovery recovery{bus.iface()};
  I2cArbiter arbiter{bus.iface(), rtos.iface(), &recovery};
  SensorService sensor{&arbiter, rtos.iface()};
  NodeIdentity identity;
  CommsSources sources;
  std::unique_ptr<CommsService> service;
};

TEST_F(CommsTest, SendsLinesTerminatedForAHost) {
  ASSERT_TRUE(service->send_line("hello"));
  EXPECT_EQ(uart.transmitted(), "hello\n");
}

TEST_F(CommsTest, CountsAWriteThePortRejected) {
  uart.set_write_fails(true);

  EXPECT_FALSE(service->send_line("hello"));

  EXPECT_EQ(uart.transmitted(), "");
  EXPECT_EQ(service->stats().write_failures, 1u);
}

TEST_F(CommsTest, BannerReportsIdentityResetCauseAndBootCount) {
  system.set_reset_cause(HAL_RESET_WATCHDOG);
  system.set_boot_count(4);

  service->send_banner();

  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("greenhouse-node-01"), std::string::npos);
  EXPECT_NE(out.find("watchdog"), std::string::npos);
  EXPECT_NE(out.find("boots=4"), std::string::npos);
  // The boot count is the NFR-0.7 reset counter, so it has to actually be on the wire.
  EXPECT_EQ(service->stats().lines_sent, 3u);
}

TEST_F(CommsTest, RecognisesTheCommandSet) {
  EXPECT_TRUE(service->handle_command("ver"));
  EXPECT_TRUE(service->handle_command("stat"));
  EXPECT_TRUE(service->handle_command("recover"));
  EXPECT_TRUE(service->handle_command("help"));

  EXPECT_EQ(service->stats().commands_received, 4u);
  EXPECT_EQ(service->stats().unknown_commands, 0u);

  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("ver 0.1.0"), std::string::npos);
  EXPECT_NE(out.find("STAT"), std::string::npos);
  EXPECT_NE(out.find("recover"), std::string::npos);
}

TEST_F(CommsTest, RecoverCommandForcesTheBusReleaseOnDemand) {
  // FR-0.3 is verified by holding the bus and watching the recovery on a scope. This
  // command is how the recovery is triggered at a chosen moment instead of hoping to
  // catch it.
  bus.hold_sda_low_for_clocks(test::kNeverRelease);

  ASSERT_TRUE(service->handle_command("recover"));

  EXPECT_EQ(bus.recoveries_run(), 1);
  EXPECT_EQ(bus.count(test::BusOp::SclWrite), 20); // nine pulses plus the STOP
  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("recover sda-stuck-low"), std::string::npos);
}

TEST_F(CommsTest, AnUnknownCommandIsReportedNotIgnored) {
  EXPECT_FALSE(service->handle_command("frobnicate"));

  EXPECT_EQ(service->stats().unknown_commands, 1u);
  EXPECT_NE(uart.transmitted().find("err unknown: frobnicate"), std::string::npos);
}

TEST_F(CommsTest, UpdateDispatchesACompleteLineFromTheHost) {
  uart.push_rx("ver\r\n");

  service->update();

  EXPECT_EQ(service->stats().commands_received, 1u);
  EXPECT_NE(uart.transmitted().find("ver 0.1.0"), std::string::npos);
}

TEST_F(CommsTest, APartialLineIsNotDispatchedUntilItIsTerminated) {
  uart.push_rx("ve");
  service->update();
  EXPECT_EQ(service->stats().commands_received, 0u);

  uart.push_rx("r\n");
  service->update();
  EXPECT_EQ(service->stats().commands_received, 1u);
}

TEST_F(CommsTest, AnOverlongCommandIsRejectedRatherThanTruncated) {
  // A truncated command could mean something else entirely, so it is refused instead of
  // being dispatched as whatever its first bytes happened to spell.
  std::string long_command(kCommsRxBufferBytes + 10, 'x');
  uart.push_rx(long_command + "\n");

  service->update();

  EXPECT_EQ(service->stats().commands_received, 0u);
  EXPECT_EQ(service->stats().rx_overflows, 1u);
  EXPECT_NE(uart.transmitted().find("err command too long"), std::string::npos);
}

TEST_F(CommsTest, TelemetryIsEmittedOnceThenHeldToItsPeriod) {
  service->telemetry_period_ms = 1000;
  rtos.set_now_ms(5000);

  service->update();
  const size_t after_first = service->stats().telemetry_sent;
  EXPECT_EQ(after_first, 1u);

  // Inside the period: no second line.
  rtos.set_now_ms(5500);
  service->update();
  EXPECT_EQ(service->stats().telemetry_sent, 1u);

  // Past it: one more.
  rtos.set_now_ms(6000);
  service->update();
  EXPECT_EQ(service->stats().telemetry_sent, 2u);
}

TEST_F(CommsTest, TelemetryCanBeSilencedForABringUpSession) {
  service->telemetry_period_ms = 0;

  service->update();
  service->update();

  EXPECT_EQ(service->stats().telemetry_sent, 0u);
  EXPECT_EQ(uart.transmitted(), "");
}

TEST_F(CommsTest, TelemetryReportsAnInvalidReadingAsSuch) {
  char buffer[kCommsRxBufferBytes];
  const size_t length = service->format_telemetry(buffer, sizeof(buffer));

  ASSERT_GT(length, 0u);
  const std::string line(buffer, length);
  // A sensor service exists but has not produced a valid reading yet. It must not report
  // a plausible number: a host has to be able to tell "no sample" from "a sample".
  EXPECT_NE(line.find("T=--.--"), std::string::npos);
  EXPECT_NE(line.find("seq=0"), std::string::npos);
}

TEST_F(CommsTest, TelemetryDistinguishesNoSensorFromNoReading) {
  // The two are genuinely different faults — "this build has no sensor service" versus
  // "the sensor is not answering" — so the wire format keeps them distinct.
  char buffer[kCommsRxBufferBytes];
  CommsSources without_sensor;
  without_sensor.arbiter = &arbiter;

  CommsService silent(uart.iface(), &arbiter, rtos.iface(), without_sensor);
  const size_t length = silent.format_telemetry(buffer, sizeof(buffer));

  ASSERT_GT(length, 0u);
  const std::string line(buffer, length);
  EXPECT_NE(line.find("T=n/a"), std::string::npos);
  EXPECT_EQ(line.find("T=--.--"), std::string::npos);
}

TEST_F(CommsTest, TelemetryCarriesAValidReadingInFixedPoint) {
  // Mid-scale on both channels: 0x8000 -> 42.501 C and 50.000 %RH, which with two
  // decimals is what a host should read.
  std::vector<uint8_t> reply(6, 0);
  reply[0] = 0x80;
  reply[1] = 0x00;
  reply[2] = Sht31::crc8(reply.data(), 2);
  reply[3] = 0x80;
  reply[4] = 0x00;
  reply[5] = Sht31::crc8(&reply[3], 2);
  bus.set_read_data(reply);

  ASSERT_TRUE(sensor.update());

  char buffer[kCommsRxBufferBytes];
  const size_t length = service->format_telemetry(buffer, sizeof(buffer));
  const std::string line(buffer, length);

  // Fixed point, two decimals, no float formatting anywhere in the image.
  EXPECT_NE(line.find("T=42.50"), std::string::npos);
  EXPECT_NE(line.find("RH=50.00"), std::string::npos);
  EXPECT_NE(line.find("seq=1"), std::string::npos);
}

TEST_F(CommsTest, TelemetryCarriesTheBusRecoveryCounters) {
  bus.hold_sda_low_for_clocks(3);
  bus.fail_transfers(1, HAL_I2C_BUS_ERROR);
  const uint8_t payload = 1;
  ASSERT_EQ(arbiter.write(kAddrSht31, &payload, 1), BusResult::Ok);

  char buffer[kCommsRxBufferBytes];
  const size_t length = service->format_telemetry(buffer, sizeof(buffer));
  const std::string line(buffer, length);

  // The FR-0.3 evidence is only useful if it reaches the host.
  EXPECT_NE(line.find("rec=1"), std::string::npos);
}

} // namespace
} // namespace node
