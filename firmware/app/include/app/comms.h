/* comms.h — the host link: telemetry out, a small command set in.
 *
 * This is the fifth task of FR-0.1 and the node's only interactive surface. In Phase 2
 * this same port carries the framed firmware-update protocol; the framing is kept out
 * of here so that the identical length-prefixed, CRC-16, ACK/retry code can run on both
 * sides of the link when it exists.
 *
 * Text is built with a hand-rolled appender rather than printf on purpose: no varargs,
 * no locale, no floating-point formatting, and none of newlib's formatter linked into a
 * 64 KB image. It also makes the output byte-exactly assertable in a host test.
 *
 * The command set is deliberately small and is aimed at bring-up rather than features.
 * `recover` in particular exists so that FR-0.3 can be exercised on demand: hold the bus
 * with a jumper, issue the command, and watch the nine clock pulses on a scope.
 */
#ifndef APP_COMMS_H
#define APP_COMMS_H

#include <cstddef>
#include <cstdint>

#include "app/board.h"
#include "app/display.h"
#include "app/i2c_arbiter.h"
#include "app/sensor.h"
#include "app/supervisor.h"
#include "hal/system.h"
#include "hal/uart.h"
#include "rtos/rtos.h"

namespace node {

// Identity reported on boot and by the `ver` command. The build strings come from the
// compiler so that a board found on a bench months later says what it is running.
struct NodeIdentity {
  const char *name = kNodeName;
  const char *version = "0.1.0";
  const char *build_date = "";
  const char *build_time = "";
};

// An append-only text buffer that never overflows and never allocates. Truncation is
// recorded rather than hidden, so a test can assert that a long line was clipped
// instead of silently producing a plausible shorter one.
class LineBuilder {
 public:
  LineBuilder(char *buffer, size_t capacity);

  LineBuilder &ch(char c);
  LineBuilder &str(const char *text);
  LineBuilder &u32(uint32_t value);
  LineBuilder &i32(int32_t value);

  // Renders a milli-unit value with exactly `decimals` digits after the point, zero
  // padded and truncated: fixed(42500, 2) -> "42.50", fixed(-4500, 1) -> "-4.5",
  // fixed(10, 2) -> "0.01", fixed(42500, 0) -> "42".
  //
  // `decimals` is digits to show, not a scale factor: the value is always in
  // milli-units, so 3 is the most that carries information and more is clamped to 3.
  LineBuilder &fixed(int32_t milli, uint32_t decimals);

  size_t size() const { return length_; }
  bool empty() const { return length_ == 0; }
  bool truncated() const { return truncated_; }
  const char *c_str() const { return buffer_; }

 private:
  char *buffer_;
  size_t capacity_;
  size_t length_ = 0;
  bool truncated_ = false;
};

struct CommsStats {
  uint32_t lines_sent = 0;
  uint32_t telemetry_sent = 0;
  uint32_t commands_received = 0;
  uint32_t unknown_commands = 0;
  uint32_t write_failures = 0;
  uint32_t rx_overflows = 0; // a command longer than the line buffer was discarded
};

struct CommsSources {
  const SensorService *sensor = nullptr;
  const Supervisor *supervisor = nullptr;
  const I2cArbiter *arbiter = nullptr;
  const hal_system_iface_t *system = nullptr;
  const DisplayStats *display = nullptr;
  const NodeIdentity *identity = nullptr;
};

class CommsService {
 public:
  CommsService(hal_uart_iface_t *uart, I2cArbiter *arbiter, const rtos_iface_t *rtos,
               const CommsSources &sources = {});

  CommsService(const CommsService &) = delete;
  CommsService &operator=(const CommsService &) = delete;

  // One iteration: drain the receive path, dispatch any complete command, and emit
  // telemetry if the interval has elapsed.
  void update();

  // Emit a line immediately, honouring the newline. Returns false if the port rejected
  // it. Public because the boot banner and the sensor service both want it.
  bool send_line(const char *text);

  // Emit the boot banner: identity, reset cause, boot count.
  void send_banner();

  // The telemetry line, without sending it. Exposed so a test can assert the exact
  // bytes a host would receive.
  size_t format_telemetry(char *out, size_t capacity) const;

  // Handle one command line. Returns true if it was recognised.
  bool handle_command(const char *line);

  const CommsStats &stats() const { return stats_; }

  // Telemetry cadence. Set to 0 to silence periodic output, which is what a bring-up
  // session wants when only commands matter.
  uint32_t telemetry_period_ms = kTelemetryPeriodMs;

 private:
  void drain_input();
  void send_status();

  hal_uart_iface_t *uart_;
  I2cArbiter *arbiter_;
  const rtos_iface_t *rtos_;
  CommsSources sources_;

  char line_[kCommsRxBufferBytes]{};
  size_t line_length_ = 0;
  bool line_overflow_ = false;

  uint32_t last_telemetry_ms_ = 0;
  bool telemetry_sent_once_ = false;

  CommsStats stats_{};
};

} // namespace node

#endif // APP_COMMS_H
