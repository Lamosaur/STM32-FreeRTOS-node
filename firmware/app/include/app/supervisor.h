/* supervisor.h — watchdog feeding and heartbeat monitoring (FR-0.4).
 *
 * FR-0.4 requires two things, and they are deliberately the same object: "an
 * independent task MUST feed a watchdog and detect missed task heartbeats". The point
 * of tying them together is that the node resets when a task stops running, rather
 * than quietly losing a subsystem while a timer callback keeps the watchdog happy.
 *
 * That is also why there is no timer-callback entry point here. Feeding a watchdog
 * from a timer interrupt proves only that interrupts still fire, which is the one
 * thing that is almost never broken. run() is meant to be called from a dedicated
 * task, and if that task never runs, the machine resets — which is the correct
 * outcome, not a bug.
 *
 * The supervisor is not the I2C arbiter's owner and shares no bus with it, so it
 * cannot itself be blocked by a stuck bus transfer.
 */
#ifndef APP_SUPERVISOR_H
#define APP_SUPERVISOR_H

#include <cstdint>

#include "app/board.h"
#include "hal/watchdog.h"
#include "rtos/rtos.h"

namespace node {

enum class HealthState {
  Ok,
  Never,  // registered but has never reported in
  Late,   // inside the deadline but past its period, so worth reporting
  Missing // past its deadline
};

const char *to_string(HealthState state);
const char *to_string(TaskId id);

// Why the supervisor stopped feeding. Survives in RAM for the current boot, so a task
// that recovers, or a node that is reset by something else, can still be diagnosed.
struct TripInfo {
  bool tripped = false;
  TaskId task = TaskId::Count;
  uint32_t lateness_ms = 0;
  uint32_t at_ms = 0;
};

struct SupervisorStats {
  uint32_t polls = 0;            // times run() was called
  uint32_t feeds = 0;            // watchdog feeds issued
  uint32_t feeds_withheld = 0;   // polls that refused to feed
  uint32_t misses = 0;           // polls that found at least one task past its deadline
  uint32_t lock_failures = 0;    // health could not be read, so feeding was withheld
  uint32_t worst_lateness_ms = 0;
};

struct SupervisorConfig {
  // A heartbeat deadline is this many task periods. Three missed periods means the
  // task is not merely late, it is gone; one is too tight to survive ordinary
  // preemption by the other tasks.
  uint32_t deadline_periods = kHeartbeatDeadlinePeriods;
  uint32_t lock_timeout_ms = 10;
};

struct TaskHealth {
  TaskId id = TaskId::Count;
  uint32_t period_ms = 0;
  uint32_t deadline_ms = 0;
  uint32_t last_beat_ms = 0;
  uint32_t beats = 0;
  bool registered = false;
  bool ever_beat = false;
};

class Supervisor {
 public:
  // `watchdog` is borrowed and must outlive this object.
  Supervisor(const rtos_iface_t *rtos, hal_watchdog_iface_t *watchdog,
             SupervisorConfig config = {});

  Supervisor(const Supervisor &) = delete;
  Supervisor &operator=(const Supervisor &) = delete;

  bool valid() const { return mutex_ != nullptr; }

  // Start watching a task. Registration counts as the task's first heartbeat, taken
  // from the current clock, so a node does not trip its own watchdog during the window
  // between start-up and the first time each task runs.
  bool watch(TaskId id, uint32_t period_ms);

  // Report from the monitored task. Safe to call at any time from any task.
  void heartbeat(TaskId id);

  // The supervisor task body, one iteration. Returns true if the watchdog was fed.
  //
  // Feeds only when every watched task is inside its deadline. Any other outcome —
  // a missing heartbeat, or an unreadable registry — withholds the feed, which is the
  // safe direction: a node whose health cannot be established must not be kept alive.
  bool run();

  HealthState health(TaskId id) const;
  const TaskHealth *task(TaskId id) const;

  const SupervisorStats &stats() const { return stats_; }
  const TripInfo &trip() const { return trip_; }

  // Clear the trip latch and counters. For tests, and for a deliberate recovery path.
  void reset();

 private:
  const rtos_iface_t *rtos_;
  hal_watchdog_iface_t *watchdog_;
  SupervisorConfig config_;
  rtos_mutex_t *mutex_;

  TaskHealth tasks_[kTaskCount]{};
  SupervisorStats stats_{};
  TripInfo trip_{};
};

} // namespace node

#endif // APP_SUPERVISOR_H
