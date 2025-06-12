#include "app/supervisor.h"

namespace node {

const char *to_string(HealthState state) {
  switch (state) {
    case HealthState::Ok:      return "ok";
    case HealthState::Never:   return "never";
    case HealthState::Late:    return "late";
    case HealthState::Missing: return "missing";
  }
  return "unknown";
}

const char *to_string(TaskId id) {
  switch (id) {
    case TaskId::Supervisor: return "supervisor";
    case TaskId::Sensor:     return "sensor";
    case TaskId::App:        return "app";
    case TaskId::Comms:      return "comms";
    case TaskId::Display:    return "display";
    case TaskId::Count:      break;
  }
  return "unknown";
}

Supervisor::Supervisor(const rtos_iface_t *rtos, hal_watchdog_iface_t *watchdog,
                       SupervisorConfig config)
    : rtos_(rtos), watchdog_(watchdog), config_(config), mutex_(nullptr) {
  if (rtos_ != nullptr && rtos_->mutex_create != nullptr) {
    mutex_ = rtos_->mutex_create(rtos_->ctx);
  }
}

bool Supervisor::watch(TaskId id, uint32_t period_ms) {
  if (id == TaskId::Count || !valid()) {
    return false;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, config_.lock_timeout_ms, nullptr) != RTOS_OK) {
    return false;
  }

  TaskHealth &slot = tasks_[static_cast<size_t>(id)];
  slot.id = id;
  slot.period_ms = period_ms;
  slot.deadline_ms = period_ms * config_.deadline_periods;
  // Seeded from the current clock: registration is a promise that the task is about to
  // start running, not evidence that it already missed anything.
  slot.last_beat_ms = rtos_->now_ms(rtos_->ctx);
  slot.beats = 0;
  slot.registered = true;
  slot.ever_beat = false;

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
  return true;
}

void Supervisor::heartbeat(TaskId id) {
  if (id == TaskId::Count || !valid()) {
    return;
  }
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, config_.lock_timeout_ms, nullptr) != RTOS_OK) {
    // A task that cannot record its own heartbeat has its beat lost, and will be
    // reported as missing. That is the honest outcome: the supervisor must not invent
    // liveness it could not observe.
    return;
  }

  TaskHealth &slot = tasks_[static_cast<size_t>(id)];
  slot.last_beat_ms = rtos_->now_ms(rtos_->ctx);
  slot.beats++;
  slot.ever_beat = true;

  rtos_->mutex_unlock(rtos_->ctx, mutex_);
}

bool Supervisor::run() {
  stats_.polls++;

  if (mutex_ == nullptr || watchdog_ == nullptr) {
    stats_.feeds_withheld++;
    stats_.lock_failures++;
    return false;
  }

  bool waited = false;
  if (rtos_->mutex_lock(rtos_->ctx, mutex_, config_.lock_timeout_ms, &waited) != RTOS_OK) {
    // Health is unknowable, so the watchdog must not be fed. Withholding here is what
    // stops a wedged supervisor from holding a dead node in a half-alive state.
    stats_.lock_failures++;
    stats_.feeds_withheld++;
    return false;
  }

  const uint32_t now = rtos_->now_ms(rtos_->ctx);

  TaskId worst_task = TaskId::Count;
  uint32_t worst_lateness = 0;
  bool any_missing = false;

  for (const TaskHealth &slot : tasks_) {
    if (!slot.registered) {
      continue;
    }
    // Unsigned subtraction, so this stays correct across the 32-bit millisecond wrap.
    const uint32_t age = now - slot.last_beat_ms;
    if (age > slot.deadline_ms) {
      const uint32_t lateness = age - slot.deadline_ms;
      if (!any_missing || lateness > worst_lateness) {
        worst_task = slot.id;
        worst_lateness = lateness;
      }
      any_missing = true;
    }
  }

  rtos_->mutex_unlock(rtos_->ctx, mutex_);

  if (any_missing) {
    stats_.misses++;
    stats_.feeds_withheld++;
    if (worst_lateness > stats_.worst_lateness_ms) {
      stats_.worst_lateness_ms = worst_lateness;
    }
    // Latched on the first miss only, so the original cause is what gets reported even
    // if later polls find a different task missing.
    if (!trip_.tripped) {
      trip_.tripped = true;
      trip_.task = worst_task;
      trip_.lateness_ms = worst_lateness;
      trip_.at_ms = now;
    }
    return false;
  }

  watchdog_->feed(watchdog_->ctx);
  stats_.feeds++;
  return true;
}

HealthState Supervisor::health(TaskId id) const {
  if (id == TaskId::Count) {
    return HealthState::Never;
  }
  const TaskHealth &slot = tasks_[static_cast<size_t>(id)];
  if (!slot.registered) {
    return HealthState::Never;
  }
  const uint32_t now = rtos_->now_ms(rtos_->ctx);
  const uint32_t age = now - slot.last_beat_ms;
  if (age > slot.deadline_ms) {
    return HealthState::Missing;
  }
  if (age > slot.period_ms) {
    return HealthState::Late;
  }
  return HealthState::Ok;
}

const TaskHealth *Supervisor::task(TaskId id) const {
  if (id == TaskId::Count) {
    return nullptr;
  }
  const TaskHealth &slot = tasks_[static_cast<size_t>(id)];
  return slot.registered ? &slot : nullptr;
}

void Supervisor::reset() {
  for (TaskHealth &slot : tasks_) {
    slot = TaskHealth{};
  }
  stats_ = SupervisorStats{};
  trip_ = TripInfo{};
}

} // namespace node
