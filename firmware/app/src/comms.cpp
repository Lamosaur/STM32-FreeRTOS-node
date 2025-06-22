#include "app/comms.h"

#include <cstring>

namespace node {
namespace {

constexpr size_t kLineCapacity = kCommsRxBufferBytes;

// Appends a string literal without needing strlen at run time.
template <size_t N>
void append_literal(LineBuilder &builder, const char (&text)[N]) {
  builder.str(text);
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

  send_line(line.c_str());
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

  if (std::strcmp(line, "help") == 0) {
    send_line("cmd ver | stat | recover | help");
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
