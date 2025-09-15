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
 * with a jumper, issue the command, and watch the nine clock pulses on a scope. `set` and
 * `get` are the FR-1.4 path — the setpoint, gains and duty limits of the control loop,
 * changed without reflashing — and they are the one part of this link that writes to
 * something the control task reads.
 */
#ifndef APP_COMMS_H
#define APP_COMMS_H

#include <cstddef>
#include <cstdint>

#include "app/acquisition.h"
#include "app/board.h"
#include "app/control.h"
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
  uint32_t tuning_sets = 0;      // `set` commands that changed the published tuning
  uint32_t tuning_refusals = 0;  // `set` commands refused, for any reason
};

struct CommsSources {
  const SensorService *sensor = nullptr;
  const Supervisor *supervisor = nullptr;
  const I2cArbiter *arbiter = nullptr;
  const hal_system_iface_t *system = nullptr;
  const DisplayStats *display = nullptr;
  const NodeIdentity *identity = nullptr;

  // The setpoint, gains and limits (FR-1.4). Not const, unlike the sources above, because
  // this task is the *writer*: the whole point of the command is that the host changes the
  // loop's tuning without a reflash, and a const source would only be readable.
  //
  // Appended at the end of the struct rather than next to the others so that the positional
  // aggregate initialisers at existing call sites keep meaning what they meant; a field
  // inserted in the middle would silently rebind every initialiser after it.
  ControlTuningStore *tuning = nullptr;

  // Phase 1's two read-only sources, appended after `tuning` for the same reason.
  //
  // They exist so that the analog channel and the control loop are visible over the link
  // without a debugger: `moist=`, `adc=` and `ctl=` on the status and telemetry lines. That
  // is the difference between "the wiring is in the image" (which nm can show) and "the
  // wiring is doing something" (which needs a number, and which on a bench is the first
  // question asked of a node whose pump is not running).
  //
  // The timing service is deliberately NOT here. It is single-writer by design (its header
  // explains why: a lock on the sample path would perturb the jitter it measures), so a
  // report from this task would be a race against the control task. Getting NFR-1.8's
  // numbers off the board needs the app task to format them into a handover buffer, which
  // is a change to that class rather than to this source list.
  const AcquisitionService *acquisition = nullptr;
  const ControlLoop *control = nullptr;
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

  // The `get` command: the published tuning, in the units `set` accepts, so that what a host
  // reads back is a line it can send again unchanged.
  void send_tuning();

  // The `set` command, with `argument` pointing at the text after "set ". Returns true
  // (the verb was understood) whether the value was accepted or refused: a refusal is
  // reported on the wire and counted, and the alternative — reporting a refused `set` as an
  // unknown command — would tell a host its syntax was wrong when its number was.
  bool handle_set(const char *argument);

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
