#include "app/comms.h"

// The full definition of TimingReportHandover: this task is the handover's reader, so it
// calls buffer()/length()/consume(), none of which exist on the forward declaration.
#include "app/timing.h"

#include <cstring>

namespace node {
namespace {

constexpr size_t kLineCapacity = kCommsRxBufferBytes;

// Appends a string literal without needing strlen at run time.
template <size_t N>
void append_literal(LineBuilder &builder, const char (&text)[N]) {
  builder.str(text);
}

// ---------------------------------------------------------------------------
// `set` / `get`: the wire format for the tuning (FR-1.4)
// ---------------------------------------------------------------------------
// Every tunable is carried as a fixed-point decimal, and the units are chosen so that a
// value `get` prints is a value `set` accepts unchanged. What a technician reads back has
// to be what they can type back; a link where the reply is in different units from the
// request is a link where a person's second action undoes their first.
//
//   sp    percent with at most two decimals   45.0  -> 450 permille
//   max   percent with at most two decimals   50.0  -> 500 permille
//   kp    3 decimals                          2.0   -> 2000 milli
//   ki    3 decimals                          0.05  -> 50 milli
//   kd    3 decimals                          0     -> 0
//   min   milliseconds, no decimals           50
//
// The stored units are the loop's own (permille and milli-units), and the conversion is
// exact by construction: each field declares the scale factor between the wire value in
// thousandths and the stored value, and a value that is not a whole number of stored units
// is refused rather than rounded. `set sp=45.05` is refused, because the controlled
// variable's resolution is 1 permille and quietly storing 450 would leave the host and the
// node disagreeing about what was set.

// Parses a decimal in thousandths: "45.0" -> 45000, "-0.5" -> -500, "2" -> 2000.
//
// Deliberately hand-rolled rather than strtol with a float: a float here would pull
// soft-float routines into the image for parsing (this part has no FPU), and strtol would
// accept "2abc" and trailing whitespace, which is exactly how a mistyped command becomes a
// silently applied one.
bool parse_milli(const char *text, int32_t *out) {
  // An upper bound on what will be parsed at all, before any field's range is considered.
  // 10,000.000 is far above every field's maximum, so anything larger is a mistake in the
  // number rather than in the tuning, and refusing it here keeps the accumulator below
  // int32 regardless of how many digits were typed.
  constexpr int64_t kParseLimitMilli = 10'000'000;

  if (text == nullptr || out == nullptr || text[0] == '\0') {
    return false;
  }

  const char *cursor = text;
  bool negative = false;
  if (*cursor == '-') {
    negative = true;
    ++cursor;
  }

  int64_t whole = 0;
  size_t whole_digits = 0;
  while (*cursor >= '0' && *cursor <= '9') {
    whole = whole * 10 + (*cursor - '0');
    ++whole_digits;
    if (whole > kParseLimitMilli) {
      return false;
    }
    ++cursor;
  }

  int64_t fraction = 0;
  size_t fraction_digits = 0;
  if (*cursor == '.') {
    ++cursor;
    while (*cursor >= '0' && *cursor <= '9') {
      if (fraction_digits == 3) {
        // Four decimals is more resolution than the milli unit has. Truncating would apply
        // a different number from the one on the wire.
        return false;
      }
      fraction = fraction * 10 + (*cursor - '0');
      ++fraction_digits;
      ++cursor;
    }
  }

  // A sign, a point and nothing else is not a number.
  if (whole_digits == 0 && fraction_digits == 0) {
    return false;
  }
  if (*cursor != '\0') {
    return false;
  }

  while (fraction_digits < 3) {
    fraction *= 10;
    ++fraction_digits;
  }

  const int64_t value = whole * 1000 + fraction;
  if (value > kParseLimitMilli) {
    return false;
  }
  *out = static_cast<int32_t>(negative ? -value : value);
  return true;
}

// Converts a parsed milli-unit value into the stored unit, which is `unit_milli` thousandths
// of the wire unit. Refuses a value that is not a whole number of stored units: see above.
bool scale_milli(int32_t parsed, int32_t unit_milli, int32_t *out) {
  if (unit_milli <= 0 || (parsed % unit_milli) != 0) {
    return false;
  }
  *out = parsed / unit_milli;
  return true;
}

bool key_matches(const char *key, size_t length, const char *name) {
  return length == std::strlen(name) && std::strncmp(key, name, length) == 0;
}

// `name=value`. `value_milli` is the value in thousandths of the wire unit and `decimals`
// is how many of them to print, so one helper renders both the reply to `get` and the
// confirmation of a `set` — which is what keeps the two in the same units.
LineBuilder &append_field(LineBuilder &line, const char *name, int32_t value_milli,
                          uint32_t decimals) {
  line.str(name).ch('=').fixed(value_milli, decimals);
  return line;
}

} // namespace

LineBuilder::LineBuilder(char *buffer, size_t capacity)
    : buffer_(buffer), capacity_(capacity) {
  if (capacity_ > 0) {
    buffer_[0] = '\0';
  }
}

LineBuilder &LineBuilder::ch(char c) {
  if (length_ + 1 >= capacity_) {
    truncated_ = true;
    return *this;
  }
  buffer_[length_++] = c;
  buffer_[length_] = '\0';
  return *this;
}

LineBuilder &LineBuilder::str(const char *text) {
  if (text == nullptr) {
    return *this;
  }
  for (const char *p = text; *p != '\0'; ++p) {
    ch(*p);
  }
  return *this;
}

LineBuilder &LineBuilder::u32(uint32_t value) {
  // Digits are produced least-significant first into a small stack buffer, then
  // reversed on the way out. No division by anything but ten, so no library call.
  char digits[10];
  size_t count = 0;
  do {
    digits[count++] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && count < sizeof(digits));

  while (count > 0) {
    ch(digits[--count]);
  }
  return *this;
}

LineBuilder &LineBuilder::i32(int32_t value) {
  if (value < 0) {
    ch('-');
    // Negating INT32_MIN overflows, so the magnitude is taken in unsigned arithmetic.
    const uint32_t magnitude = static_cast<uint32_t>(-(static_cast<int64_t>(value)));
    return u32(magnitude);
  }
  return u32(static_cast<uint32_t>(value));
}

LineBuilder &LineBuilder::fixed(int32_t milli, uint32_t decimals) {
  // The value is in milli-units, so three digits after the point is all the resolution
  // it has. Clamping rather than padding with zeros: printing extra digits would show
  // precision the value does not carry, which is the kind of thing someone later reads
  // as a measurement.
  if (decimals > 3) {
    decimals = 3;
  }

  const bool negative = milli < 0;
  const uint32_t magnitude = static_cast<uint32_t>(
      negative ? -(static_cast<int64_t>(milli)) : static_cast<int64_t>(milli));

  if (decimals == 0) {
    if (negative) {
      ch('-');
    }
    return u32(magnitude / 1000U);
  }

  if (negative) {
    ch('-');
  }
  u32(magnitude / 1000U);
  ch('.');

  // The fraction is built as three digits and then truncated to the requested count, so
  // it is zero-padded for free: 42.05 must not come out as 42.5.
  const char fraction[4] = {
      static_cast<char>('0' + (magnitude / 100U) % 10U),
      static_cast<char>('0' + (magnitude / 10U) % 10U),
      static_cast<char>('0' + magnitude % 10U),
      '\0',
  };
  for (uint32_t i = 0; i < decimals; ++i) {
    ch(fraction[i]);
  }
  return *this;
}

CommsService::CommsService(hal_uart_iface_t *uart, I2cArbiter *arbiter,
                           const rtos_iface_t *rtos, const CommsSources &sources)
    : uart_(uart), arbiter_(arbiter), rtos_(rtos), sources_(sources) {}

bool CommsService::send_line(const char *text) {
  if (uart_ == nullptr || text == nullptr) {
    return false;
  }
  const size_t length = std::strlen(text);
  const size_t wrote = uart_->write(uart_->ctx, reinterpret_cast<const uint8_t *>(text),
                                    length, kUartWriteTimeoutMs);
  if (wrote != length) {
    stats_.write_failures++;
    return false;
  }
  // A host reading lines needs the terminator; send it separately so the payload write
  // length is exactly the text length.
  const uint8_t newline = static_cast<uint8_t>('\n');
  if (uart_->write(uart_->ctx, &newline, 1, kUartWriteTimeoutMs) != 1) {
    stats_.write_failures++;
    return false;
  }
  stats_.lines_sent++;
  return true;
}

void CommsService::send_banner() {
  char buffer[kLineCapacity];

  LineBuilder line(buffer, sizeof(buffer));
  line.str("boot ").str(sources_.identity != nullptr ? sources_.identity->name : kNodeName);
  send_line(line.c_str());

  LineBuilder version(buffer, sizeof(buffer));
  version.str("ver ").str(sources_.identity != nullptr ? sources_.identity->version : "?");
  version.str(" built ")
      .str(sources_.identity != nullptr ? sources_.identity->build_date : "?")
      .ch(' ')
      .str(sources_.identity != nullptr ? sources_.identity->build_time : "?");
  send_line(version.c_str());

  LineBuilder reset(buffer, sizeof(buffer));
  reset.str("reset ");
  if (sources_.system != nullptr) {
    switch (sources_.system->reset_cause(sources_.system->ctx)) {
      case HAL_RESET_POWER_ON:     reset.str("power-on"); break;
      case HAL_RESET_EXTERNAL_PIN: reset.str("external-pin"); break;
      case HAL_RESET_WATCHDOG:     reset.str("watchdog"); break;
      case HAL_RESET_SOFTWARE:     reset.str("software"); break;
      case HAL_RESET_LOW_POWER:    reset.str("low-power"); break;
      case HAL_RESET_UNKNOWN:      reset.str("unknown"); break;
    }
    reset.str(" boots=").u32(sources_.system->boot_count(sources_.system->ctx));
  } else {
    reset.str("unknown");
  }
  send_line(reset.c_str());
}

size_t CommsService::format_telemetry(char *out, size_t capacity) const {
  LineBuilder line(out, capacity);

  if (sources_.sensor != nullptr) {
    const SensorSnapshot snap = sources_.sensor->snapshot();
    if (snap.reading.valid) {
      line.str("T=").fixed(snap.reading.temperature_milli_c, 2).str("C RH=")
          .fixed(snap.reading.humidity_milli_rh, 2)
          .str("% seq=")
          .u32(snap.sequence);
    } else {
      // An invalid reading is reported as such, never as a stale number that a host
      // would take for a fresh one.
      line.str("T=--.-- RH=--.-- seq=").u32(snap.sequence);
    }
  } else {
    line.str("T=n/a RH=n/a");
  }

  // The primary quantity, and the one the control loop acts on. Reported in permille of
  // the probe's calibrated span, which is the unit the loop and the setpoint command both
  // use, so a host can compare the reading with `get` without converting anything.
  if (sources_.acquisition != nullptr) {
    const AnalogSnapshot moist = sources_.acquisition->snapshot();
    if (moist.valid) {
      line.str(" moist=").u32(static_cast<uint32_t>(moist.moisture_permille));
      if (!moist.fresh()) {
        // A reading that has stopped being refreshed is flagged rather than hidden: the
        // failure this prevents is a host logging a frozen number as live soil moisture,
        // which is the same failure the display's staleness counter exists to prevent.
        line.str("(stale)");
      }
    } else {
      line.str(" moist=--");
    }
  }

  if (sources_.arbiter != nullptr) {
    const ArbiterStats bus = sources_.arbiter->stats();
    line.str(" bus=")
        .u32(bus.transfers)
        .str("/")
        .u32(bus.failed_transfers)
        .str(" rec=")
        .u32(bus.recoveries);
  }

  if (sources_.supervisor != nullptr) {
    const SupervisorStats sup = sources_.supervisor->stats();
    line.str(" feeds=").u32(sup.feeds);
    if (sources_.supervisor->trip().tripped) {
      line.str(" TRIP=").str(to_string(sources_.supervisor->trip().task));
    }
  }

  return line.size();
}

void CommsService::send_status() {
  char buffer[kLineCapacity];

  LineBuilder line(buffer, sizeof(buffer));
  line.str("STAT lines=").u32(stats_.lines_sent).str(" cmd=").u32(stats_.commands_received);

  if (sources_.arbiter != nullptr) {
    const ArbiterStats bus = sources_.arbiter->stats();
    line.str(" tx=")
        .u32(bus.transfers)
        .str(" dma=")
        .u32(bus.dma_transfers)
        .str(" fail=")
        .u32(bus.failed_transfers)
        .str(" rec=")
        .u32(bus.recoveries)
        .str(" recfail=")
        .u32(bus.recovery_failures)
        .str(" cont=")
        .u32(bus.contended)
        .str(" tmo=")
        .u32(bus.lock_timeouts);
  }

  if (sources_.display != nullptr) {
    line.str(" disp=")
        .u32(sources_.display->bytes_sent)
        .str("/")
        .u32(sources_.display->partial_flushes);
  }

  if (sources_.sensor != nullptr) {
    const SensorServiceStats sensor = sources_.sensor->stats();
    line.str(" sen=")
        .u32(sensor.published)
        .str("/")
        .u32(sensor.failed);
  }

  if (sources_.supervisor != nullptr) {
    const SupervisorStats sup = sources_.supervisor->stats();
    line.str(" feeds=")
        .u32(sup.feeds)
        .str(" miss=")
        .u32(sup.misses)
        .str(" worst=")
        .u32(sup.worst_lateness_ms);
  }

  // Phase 1. `adc=` is blocks received/counted by the DMA consumer/dropped by the ring —
  // the three numbers that say whether the analog channel is real — and `ctl=` is control
  // evaluations, fail-safe windows and saturations, which is how a node whose pump is not
  // running says which of the three reasons it is.
  //
  // Deliberately counts rather than the timing series: that one is single-writer and is
  // reported by the app task, not here. See CommsSources.
  if (sources_.acquisition != nullptr) {
    const AcquisitionStats adc = sources_.acquisition->stats();
    line.str(" adc=")
        .u32(adc.blocks_received)
        .str("/")
        .u32(adc.blocks_processed)
        .str("/")
        .u32(adc.blocks_dropped);
  }

  if (sources_.control != nullptr) {
    const ControlStats ctl = sources_.control->stats();
    line.str(" ctl=")
        .u32(ctl.evaluations)
        .str("/")
        .u32(ctl.fail_safe_windows)
        .str("/")
        .u32(ctl.saturated_windows);
  }

  send_line(line.c_str());
}

void CommsService::send_tuning() {
  char buffer[kLineCapacity];
  LineBuilder line(buffer, sizeof(buffer));

  if (sources_.tuning == nullptr) {
    // Not "the defaults": a host cannot tell defaults from a tuning it cannot read, and a
    // node that reports numbers it is not using is worse than one that reports none.
    send_line("tune unsupported");
    return;
  }

  const ControlTuning tuning = sources_.tuning->load();
  line.str("tune ");
  append_field(line, "sp", tuning.setpoint_permille * 100, 2);
  line.ch(' ');
  append_field(line, "kp", tuning.kp_milli, 3);
  line.ch(' ');
  append_field(line, "ki", tuning.ki_milli, 3);
  line.ch(' ');
  append_field(line, "kd", tuning.kd_milli, 3);
  line.ch(' ');
  append_field(line, "min", static_cast<int32_t>(tuning.min_on_time_ms) * 1000, 0);
  line.ch(' ');
  append_field(line, "max", tuning.max_duty_permille * 100, 2);
  send_line(line.c_str());
}

void CommsService::send_timing() {
  if (sources_.timing == nullptr) {
    // Same refusal style as `get`: report the absence rather than invent figures.
    send_line("timing unsupported");
    return;
  }

  if (sources_.timing->ready()) {
    // A publication is waiting: send it and acknowledge it in the same command, so the
    // writer can start the next window. The report is newline-separated and every line
    // fits the 128-byte line budget (TimingService::format_line guarantees it), so each
    // fragment below is exactly one sendable line.
    const char *report = sources_.timing->buffer();
    const size_t length = sources_.timing->length();
    char buffer[kLineCapacity];
    size_t start = 0;
    while (start < length) {
      size_t end = start;
      while (end < length && report[end] != '\n') {
        ++end;
      }
      const size_t n = end - start;
      if (n > 0 && n < sizeof(buffer)) {
        std::memcpy(buffer, report + start, n);
        buffer[n] = '\0';
        (void)send_line(buffer);
      }
      start = end + 1U; // Skip the newline; the next pass starts at the next line.
    }
    sources_.timing->consume();
    return;
  }

  // Nothing published yet: ask the app task to format the series as it stands. The
  // report arrives on a later poll (one app tick to format, then this task's next
  // 50 ms tick to send it), which "pending" says plainly rather than pretending the
  // numbers are here.
  sources_.timing->request();
  send_line("timing: report pending");
}

bool CommsService::handle_set(const char *argument) {
  char buffer[kLineCapacity];

  if (sources_.tuning == nullptr) {
    stats_.tuning_refusals++;
    send_line("err set: unsupported");
    return true;
  }

  // Refusal, in one place so that every reason is counted and phrased the same way. The
  // echo of what was typed is the whole value of the line: a host that sent six commands
  // needs to know which one was rejected.
  const auto refuse = [&](const char *reason) {
    stats_.tuning_refusals++;
    LineBuilder out(buffer, sizeof(buffer));
    out.str("err set ").str(argument).str(": ").str(reason);
    send_line(out.c_str());
    return true;
  };

  const char *equals = std::strchr(argument, '=');
  if (equals == nullptr || equals == argument || equals[1] == '\0') {
    return refuse("expected key=value");
  }

  const char *key = argument;
  const size_t key_length = static_cast<size_t>(equals - argument);
  const char *value_text = equals + 1;

  int32_t parsed = 0;
  if (!parse_milli(value_text, &parsed)) {
    return refuse("not a number");
  }

  ControlTuning next = sources_.tuning->load();
  int32_t scaled = 0;
  const char *canonical_name = nullptr;
  uint32_t canonical_decimals = 0;

  if (key_matches(key, key_length, "sp")) {
    // The wire unit is percent, so the stored permille is a hundredth of the parsed
    // thousandths. 45.0 is 450 permille; 45.05 has no representation and is refused.
    if (!scale_milli(parsed, 100, &scaled)) {
      return refuse("too precise: setpoint is in permille (one decimal in percent)");
    }
    next.setpoint_permille = scaled;
    canonical_name = "sp";
    canonical_decimals = 2;
  } else if (key_matches(key, key_length, "kp")) {
    if (!scale_milli(parsed, 1, &scaled)) {
      return refuse("not a number");
    }
    next.kp_milli = scaled;
    canonical_name = "kp";
    canonical_decimals = 3;
  } else if (key_matches(key, key_length, "ki")) {
    if (!scale_milli(parsed, 1, &scaled)) {
      return refuse("not a number");
    }
    next.ki_milli = scaled;
    canonical_name = "ki";
    canonical_decimals = 3;
  } else if (key_matches(key, key_length, "kd")) {
    if (!scale_milli(parsed, 1, &scaled)) {
      return refuse("not a number");
    }
    next.kd_milli = scaled;
    canonical_name = "kd";
    canonical_decimals = 3;
  } else if (key_matches(key, key_length, "min")) {
    if (!scale_milli(parsed, 1000, &scaled)) {
      return refuse("too precise: the minimum on-time is whole milliseconds");
    }
    if (scaled < 0) {
      return refuse("out of range");
    }
    next.min_on_time_ms = static_cast<uint32_t>(scaled);
    canonical_name = "min";
    canonical_decimals = 0;
  } else if (key_matches(key, key_length, "max")) {
    if (!scale_milli(parsed, 100, &scaled)) {
      return refuse("too precise: the duty cap is in permille (one decimal in percent)");
    }
    next.max_duty_permille = scaled;
    canonical_name = "max";
    canonical_decimals = 2;
  } else {
    return refuse("unknown key (sp, kp, ki, kd, min, max)");
  }

  // The whole resulting set is validated, not just the field that was typed: the limits are
  // validated as a pair, so `set min=900` can be refused because of `max`. The field named
  // in the refusal is therefore the field the *resulting tuning* is invalid because of,
  // which is not always the one that was typed — and that is the useful information, since
  // it is the number that has to change.
  const TuningField invalid = tuning_error(next);
  if (invalid != TuningField::None) {
    return refuse(to_string(invalid));
  }

  if (!sources_.tuning->publish(next)) {
    // Unreachable: the set above was validated by the same rule the store applies. Kept
    // because the store is the writer's only guarantee and a silent failure to publish would
    // be a host believing a value it never installed.
    return refuse("refused by the tuning store");
  }

  stats_.tuning_sets++;
  LineBuilder out(buffer, sizeof(buffer));
  // The confirmation echoes the value in the same units `get` uses — `fixed()` takes
  // thousandths of the wire unit, which is exactly what was parsed — so a host that sent
  // `set sp=45.0` reads back the normalised `set sp=45.00 ok` and can see the value it
  // actually installed.
  out.str("set ");
  append_field(out, canonical_name, parsed, canonical_decimals);
  out.str(" ok");
  send_line(out.c_str());
  return true;
}

bool CommsService::handle_command(const char *line) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }
  stats_.commands_received++;

  char buffer[kLineCapacity];

  if (std::strcmp(line, "ver") == 0) {
    LineBuilder out(buffer, sizeof(buffer));
    out.str("ver ")
        .str(sources_.identity != nullptr ? sources_.identity->version : "?")
        .str(" node=")
        .str(sources_.identity != nullptr ? sources_.identity->name : kNodeName);
    send_line(out.c_str());
    return true;
  }

  if (std::strcmp(line, "stat") == 0) {
    send_status();
    return true;
  }

  if (std::strcmp(line, "get") == 0) {
    send_tuning();
    return true;
  }

  if (std::strncmp(line, "set ", 4) == 0) {
    return handle_set(line + 4);
  }

  if (std::strcmp(line, "recover") == 0) {
    // Forces the FR-0.3 sequence on demand, so a held bus can be released without a
    // power cycle while someone watches the scope.
    LineBuilder out(buffer, sizeof(buffer));
    if (arbiter_ == nullptr) {
      out.str("recover unsupported");
    } else {
      out.str("recover ").str(to_string(arbiter_->force_recovery()));
      const ArbiterStats bus = arbiter_->stats();
      out.str(" rec=").u32(bus.recoveries).str(" recfail=").u32(bus.recovery_failures);
    }
    send_line(out.c_str());
    return true;
  }

  if (std::strcmp(line, "timing") == 0) {
    send_timing();
    return true;
  }

  if (std::strcmp(line, "update") == 0) {
    // Phase 2 handshake: ask the bootloader to accept an update. The marker write and
    // the reset are separate interface calls so a failed marker can be reported
    // honestly — a reset without the marker would boot normally and the host would
    // wonder what happened. The farewell is sent before the reset; the reset does not
    // wait for the port to drain, and the marker is what survives.
    if (sources_.system == nullptr || sources_.system->update_request == nullptr) {
      send_line("update unsupported");
      return true;
    }
    if (!sources_.system->update_request(sources_.system->ctx)) {
      send_line("update: marker failed; the node keeps running");
      return true;
    }
    send_line("update: rebooting into the bootloader");
    if (sources_.system->system_reset != nullptr) {
      sources_.system->system_reset(sources_.system->ctx);
    }
    return true;
  }

  if (std::strcmp(line, "help") == 0) {
    send_line("cmd ver | stat | get | set <k=v> | recover | timing | update | help");
    return true;
  }

  stats_.unknown_commands++;
  LineBuilder out(buffer, sizeof(buffer));
  out.str("err unknown: ").str(line);
  send_line(out.c_str());
  return false;
}

void CommsService::drain_input() {
  if (uart_ == nullptr) {
    return;
  }

  uint8_t byte = 0;
  while (uart_->read(uart_->ctx, &byte, 1, 0) == 1) {
    const char c = static_cast<char>(byte);

    if (c == '\n' || c == '\r') {
      if (line_overflow_) {
        // The command was longer than the buffer, so it cannot be trusted; report it
        // rather than dispatching a truncated command that might mean something else.
        stats_.rx_overflows++;
        send_line("err command too long");
      } else if (line_length_ > 0) {
        line_[line_length_] = '\0';
        handle_command(line_);
      }
      line_length_ = 0;
      line_overflow_ = false;
      continue;
    }

    if (line_length_ + 1 >= sizeof(line_)) {
      line_overflow_ = true;
      continue;
    }
    line_[line_length_++] = c;
  }
}

void CommsService::update() {
  drain_input();

  if (telemetry_period_ms == 0) {
    return;
  }

  const uint32_t now = rtos_->now_ms(rtos_->ctx);
  if (!telemetry_sent_once_ || (now - last_telemetry_ms_) >= telemetry_period_ms) {
    char buffer[kLineCapacity];
    const size_t length = format_telemetry(buffer, sizeof(buffer));
    if (length > 0 && send_line(buffer)) {
      stats_.telemetry_sent++;
    }
    last_telemetry_ms_ = now;
    telemetry_sent_once_ = true;
  }
}

} // namespace node
