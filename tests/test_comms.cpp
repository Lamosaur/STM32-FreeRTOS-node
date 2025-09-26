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
#include "app/control.h"
#include "app/i2c_arbiter.h"
#include "app/i2c_recovery.h"
#include "app/timing.h"
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
    sources.tuning = &tuning_store;
    sources.timing = &timing_handover;
    service = std::make_unique<CommsService>(uart.iface(), &arbiter, rtos.iface(), sources);
  }

  FakeI2c bus;
  FakeRtos rtos;
  FakeUart uart;
  FakeSystem system;
  I2cRecovery recovery{bus.iface()};
  I2cArbiter arbiter{bus.iface(), rtos.iface(), &recovery};
  SensorService sensor{&arbiter, rtos.iface()};
  ControlTuningStore tuning_store;
  TimingReportHandover timing_handover;
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
  EXPECT_TRUE(service->handle_command("get"));
  EXPECT_TRUE(service->handle_command("set sp=45.0"));

  EXPECT_EQ(service->stats().commands_received, 6u);
  EXPECT_EQ(service->stats().unknown_commands, 0u);

  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("ver 0.1.0"), std::string::npos);
  EXPECT_NE(out.find("STAT"), std::string::npos);
  EXPECT_NE(out.find("recover"), std::string::npos);
  EXPECT_NE(out.find("tune sp=45.00"), std::string::npos);
  // The help line has to mention the commands a host can send, or a bring-up session that
  // starts with `help` never learns that the loop can be tuned over the link.
  EXPECT_NE(out.find("set <k=v>"), std::string::npos);
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

// ---------------------------------------------------------------------------
// Runtime tuning over the link (FR-1.4)
// ---------------------------------------------------------------------------
// The point of these commands is that the setpoint and gains can be changed without
// reflashing, so the tests assert what a host would actually receive *and* what the control
// task would subsequently read, because a link that reports success while leaving the loop
// on its old tuning is the failure that matters.

TEST_F(CommsTest, GetReportsTheTuningInTheUnitsSetAccepts) {
  ASSERT_TRUE(service->handle_command("get"));

  EXPECT_NE(uart.transmitted().find("tune sp=45.00 kp=2.000 ki=0.050 kd=0.000 min=50 max=50.00"),
            std::string::npos);
}

TEST_F(CommsTest, SetChangesEveryFieldWithoutAReflash) {
  EXPECT_TRUE(service->handle_command("set sp=30.0"));
  EXPECT_TRUE(service->handle_command("set kp=2.5"));
  EXPECT_TRUE(service->handle_command("set ki=0.1"));
  EXPECT_TRUE(service->handle_command("set kd=0.5"));
  EXPECT_TRUE(service->handle_command("set min=60"));
  EXPECT_TRUE(service->handle_command("set max=20.0"));

  // What the control task will read on its next evaluation.
  const ControlTuning tuning = tuning_store.load();
  EXPECT_EQ(tuning.setpoint_permille, 300);
  EXPECT_EQ(tuning.kp_milli, 2500);
  EXPECT_EQ(tuning.ki_milli, 100);
  EXPECT_EQ(tuning.kd_milli, 500);
  EXPECT_EQ(tuning.min_on_time_ms, 60u);
  EXPECT_EQ(tuning.max_duty_permille, 200);

  EXPECT_EQ(service->stats().tuning_sets, 6u);
  EXPECT_EQ(service->stats().tuning_refusals, 0u);

  // The confirmation is the normalised value, in the units `get` uses, so a host that sent
  // `sp=30.0` can see that 30.00 is what it installed.
  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("set sp=30.00 ok"), std::string::npos);
  EXPECT_NE(out.find("set min=60 ok"), std::string::npos);
}

TEST_F(CommsTest, TimingCommandSendsAPublishedReportAndConsumesIt) {
  TimingService timing;
  timing.note_period_us(1'000'000);
  timing.note_period_us(1'000'000);
  timing.note_period_us(1'000'000);
  ASSERT_TRUE(timing_handover.publish(timing, /*window_complete=*/false));

  ASSERT_TRUE(service->handle_command("timing"));

  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("period us: n=3 min=1000000 max=1000000 mean=1000000 sd=0"),
            std::string::npos);
  EXPECT_EQ(service->stats().unknown_commands, 0u);
  // The published report was sent, so the handover is empty again and the writer can
  // start the next publication.
  EXPECT_FALSE(timing_handover.ready());
  EXPECT_FALSE(timing_handover.window_complete());
}

TEST_F(CommsTest, TimingCommandRequestsWhenNothingIsPublished) {
  ASSERT_TRUE(service->handle_command("timing"));

  EXPECT_NE(uart.transmitted().find("timing: report pending"), std::string::npos);
  // The app task's view: a request is waiting to be serviced on its next tick.
  EXPECT_TRUE(timing_handover.take_request());
  EXPECT_FALSE(timing_handover.take_request());
  EXPECT_FALSE(timing_handover.ready());
}

TEST_F(CommsTest, TimingCommandServicesARequestOnTheNextPoll) {
  // The full loop as the two tasks would run it: the host asks, this task forwards the
  // request, the app task publishes on its next tick, and the next `timing` sends it.
  ASSERT_TRUE(service->handle_command("timing"));      // -> request()
  ASSERT_TRUE(timing_handover.take_request());         // the app task's next tick
  TimingService timing;
  timing.note_period_us(1'000'000);
  ASSERT_TRUE(timing_handover.publish(timing, false)); // the app task publishes

  uart.clear_transmitted();
  ASSERT_TRUE(service->handle_command("timing"));
  EXPECT_NE(uart.transmitted().find("period us: n=1"), std::string::npos);
  EXPECT_FALSE(timing_handover.ready());               // sent and consumed
}

TEST_F(CommsTest, ARefusedSetIsReportedAndLeavesTheTuningAlone) {
  ASSERT_TRUE(service->handle_command("set sp=45.0"));
  uart.clear_transmitted();

  // Out of the calibration span. The refusal names the field the resulting tuning is
  // invalid because of, which is what a person has to change.
  EXPECT_TRUE(service->handle_command("set sp=120.0"));
  EXPECT_NE(uart.transmitted().find("err set sp=120.0: setpoint"), std::string::npos);

  uart.clear_transmitted();
  // Finer than the controlled variable's resolution: refusing beats storing 450 for a
  // request of 45.05 and leaving the host believing otherwise.
  EXPECT_TRUE(service->handle_command("set sp=45.05"));
  EXPECT_NE(uart.transmitted().find("too precise"), std::string::npos);

  uart.clear_transmitted();
  // A negative gain is positive feedback, and the sign error is invisible in the code.
  EXPECT_TRUE(service->handle_command("set kp=-1"));
  EXPECT_NE(uart.transmitted().find("err set kp=-1: kp"), std::string::npos);

  uart.clear_transmitted();
  EXPECT_TRUE(service->handle_command("set kp=abc"));
  EXPECT_NE(uart.transmitted().find("not a number"), std::string::npos);

  uart.clear_transmitted();
  EXPECT_TRUE(service->handle_command("set frob=1"));
  EXPECT_NE(uart.transmitted().find("unknown key"), std::string::npos);

  uart.clear_transmitted();
  EXPECT_TRUE(service->handle_command("set sp"));
  EXPECT_NE(uart.transmitted().find("expected key=value"), std::string::npos);

  // The `set` verb was understood in every case, so none of these is an unknown command:
  // telling a host its syntax was wrong when its number was sends it to the wrong place.
  EXPECT_EQ(service->stats().unknown_commands, 0u);
  EXPECT_EQ(service->stats().tuning_sets, 1u);
  EXPECT_EQ(service->stats().tuning_refusals, 6u);

  const ControlTuning tuning = tuning_store.load();
  EXPECT_EQ(tuning.setpoint_permille, 450);
  EXPECT_EQ(tuning.kp_milli, 2000);
}

TEST_F(CommsTest, WhatGetPrintsIsWhatSetAccepts) {
  // A reply a host cannot send back is a reply that has to be translated by whoever reads
  // it, and that translation is where a unit error lives. This test takes `get`'s own
  // words, splits them out, and feeds them back.
  ASSERT_TRUE(service->handle_command("set sp=37.5"));
  ASSERT_TRUE(service->handle_command("set kp=1.75"));
  ASSERT_TRUE(service->handle_command("set min=80"));
  ASSERT_TRUE(service->handle_command("set max=12.5"));
  uart.clear_transmitted();

  ASSERT_TRUE(service->handle_command("get"));
  std::string printed = uart.transmitted();
  ASSERT_FALSE(printed.empty());
  EXPECT_EQ(printed.back(), '\n');
  printed.pop_back();
  ASSERT_EQ(printed.compare(0, 5, "tune "), 0);

  const ControlTuning before = tuning_store.load();
  size_t fields_sent = 0;
  size_t cursor = 5;
  while (cursor < printed.size()) {
    const size_t space = printed.find(' ', cursor);
    const std::string field = printed.substr(cursor, space - cursor);
    EXPECT_TRUE(service->handle_command(("set " + field).c_str())) << field;
    ++fields_sent;
    if (space == std::string::npos) {
      break;
    }
    cursor = space + 1;
  }

  EXPECT_EQ(fields_sent, 6u);
  EXPECT_EQ(service->stats().tuning_refusals, 0u);

  const ControlTuning after = tuning_store.load();
  EXPECT_EQ(after.setpoint_permille, before.setpoint_permille);
  EXPECT_EQ(after.kp_milli, before.kp_milli);
  EXPECT_EQ(after.ki_milli, before.ki_milli);
  EXPECT_EQ(after.min_on_time_ms, before.min_on_time_ms);
  EXPECT_EQ(after.max_duty_permille, before.max_duty_permille);
}

TEST_F(CommsTest, EveryTuningLineFitsTheLineBuffer) {
  // kCommsRxBufferBytes is the buffer the receiver fills and the line builder writes into
  // (LineBuilder truncates silently rather than overflowing), so a line that did not fit
  // would arrive at the host clipped — and a clipped tuning line is a plausible wrong
  // number. The widest case is every field at its maximum.
  ASSERT_TRUE(service->handle_command("set sp=100.0"));
  ASSERT_TRUE(service->handle_command("set kp=100.0"));
  ASSERT_TRUE(service->handle_command("set ki=100.0"));
  ASSERT_TRUE(service->handle_command("set kd=100.0"));
  ASSERT_TRUE(service->handle_command("set min=1000"));
  ASSERT_TRUE(service->handle_command("set max=100.0"));
  uart.clear_transmitted();

  ASSERT_TRUE(service->handle_command("get"));

  const std::string out = uart.transmitted();
  EXPECT_NE(out.find("\n"), std::string::npos);
  EXPECT_LT(out.size(), kCommsRxBufferBytes);
  EXPECT_EQ(out.find('\n'), out.size() - 1); // one line, terminated once
}

TEST_F(CommsTest, TuningCommandsSaySoWhenThereIsNoTuningSource) {
  // A node whose comms task has no tuning store still controls on the compiled-in defaults;
  // what it must not do is report a tuning it cannot read or accept a set it cannot apply.
  CommsSources without_tuning;
  without_tuning.arbiter = &arbiter;
  CommsService silent(uart.iface(), &arbiter, rtos.iface(), without_tuning);

  EXPECT_TRUE(silent.handle_command("get"));
  EXPECT_NE(uart.transmitted().find("tune unsupported"), std::string::npos);

  uart.clear_transmitted();
  EXPECT_TRUE(silent.handle_command("set sp=30.0"));
  EXPECT_NE(uart.transmitted().find("err set: unsupported"), std::string::npos);
  EXPECT_EQ(silent.stats().tuning_refusals, 1u);
  EXPECT_EQ(silent.stats().tuning_sets, 0u);
}

TEST_F(CommsTest, ASetIsDispatchedFromTheWireLikeAnyOtherCommand) {
  // The whole path, from bytes to published tuning: this is what FR-1.4 looks like from the
  // host's side.
  uart.push_rx("set sp=32.5\r\n");

  service->update();

  EXPECT_EQ(service->stats().commands_received, 1u);
  EXPECT_EQ(tuning_store.load().setpoint_permille, 325);
  EXPECT_NE(uart.transmitted().find("set sp=32.50 ok"), std::string::npos);
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
  // 42.50 C and 50.00 %RH, which with two decimals is what a host should read.
  //
  // The raw values are chosen so the fixed-point conversion lands exactly:
  //   humidity 50.000 %RH  -> raw = 0.5 * 2^20 = 524288 exactly
  //   temperature 42.500 C -> raw such that raw * 200000 / 1048576 = 92500, i.e. raw in
  //     [92500 * 1048576 / 200000, ...) = [484966.4, ...), so 484967 gives 92500 after the
  //     truncation (92500.11 before it).
  //
  // The frame is the AHT20's: status, five data bytes packing 20 bits of each quantity, and
  // one CRC over the first six. This replaced the SHT3x's six-byte reply when the sensor
  // module on the bench turned out to be an AHT20 + BMP280 rather than an SHT31.
  const uint32_t raw_temperature = 484'967;
  const uint32_t raw_humidity = 524'288;
  std::vector<uint8_t> reply(7, 0);
  reply[0] = 0x08; // calibrated, not busy
  reply[1] = static_cast<uint8_t>((raw_humidity >> 12) & 0xFFU);
  reply[2] = static_cast<uint8_t>((raw_humidity >> 4) & 0xFFU);
  reply[3] = static_cast<uint8_t>(((raw_humidity & 0x0FU) << 4) |
                                  ((raw_temperature >> 16) & 0x0FU));
  reply[4] = static_cast<uint8_t>((raw_temperature >> 8) & 0xFFU);
  reply[5] = static_cast<uint8_t>(raw_temperature & 0xFFU);
  reply[6] = Aht20::crc8(reply.data(), 6);
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
  ASSERT_EQ(arbiter.write(kAddrAht20, &payload, 1), BusResult::Ok);

  char buffer[kCommsRxBufferBytes];
  const size_t length = service->format_telemetry(buffer, sizeof(buffer));
  const std::string line(buffer, length);

  // The FR-0.3 evidence is only useful if it reaches the host.
  EXPECT_NE(line.find("rec=1"), std::string::npos);
}

} // namespace
} // namespace node
