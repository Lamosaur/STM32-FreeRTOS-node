/* fake_rtos.h — host double for rtos_iface_t.
 *
 * Unlike a stub that just returns success, this one is backed by real
 * std::timed_mutex, so a host test can run genuine threads through the I2C arbiter and
 * observe genuine contention. That matters because FR-0.2's claim is about
 * *serialisation under stress*, and a single-threaded harness cannot falsify it: the
 * test would pass just as happily with no mutex at all if the fake bus never had two
 * callers in flight.
 *
 * rtos_mutex and rtos_task are declared outside the extern "C" block in rtos.h so that
 * they can carry real C++ members here; see the comment in that header.
 */
#ifndef NODE_TEST_FAKE_RTOS_H
#define NODE_TEST_FAKE_RTOS_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rtos/rtos.h"

// The tag declared by rtos.h, defined here as a real host mutex.
struct rtos_mutex {
  std::timed_mutex mtx;
  std::atomic<int> depth{0};
  std::atomic<int> max_depth{0};
  std::atomic<int> contentions{0};
  std::atomic<uint32_t> holder_priority{0};
};

struct rtos_task {
  std::string name;
  uint32_t stack_words = 0;
  uint32_t priority = 0;
  uint32_t high_water = 0;
};

namespace node::test {

class FakeRtos {
 public:
  FakeRtos() {
    iface_.ctx = this;
    iface_.mutex_create = &FakeRtos::mutex_create_cb;
    iface_.mutex_lock = &FakeRtos::mutex_lock_cb;
    iface_.mutex_unlock = &FakeRtos::mutex_unlock_cb;
    iface_.mutex_holder_priority = &FakeRtos::mutex_holder_priority_cb;
    iface_.now_ms = &FakeRtos::now_ms_cb;
    iface_.delay_ms = &FakeRtos::delay_ms_cb;
    iface_.task_create = &FakeRtos::task_create_cb;
    iface_.task_stack_high_water = &FakeRtos::task_stack_high_water_cb;
    iface_.yield = &FakeRtos::yield_cb;
  }

  ~FakeRtos() {
    for (rtos_mutex *m : mutexes_) {
      delete m;
    }
    for (rtos_task *t : tasks_) {
      delete t;
    }
  }

  FakeRtos(const FakeRtos &) = delete;
  FakeRtos &operator=(const FakeRtos &) = delete;

  const rtos_iface_t *iface() const { return &iface_; }

  // --- control ------------------------------------------------------------

  uint32_t now_ms() const { return now_ms_.load(); }
  void set_now_ms(uint32_t ms) { now_ms_.store(ms); }
  void advance_ms(uint32_t ms) { now_ms_.fetch_add(ms); }

  // Make the next mutex_lock fail with RTOS_TIMEOUT, as if another task held the bus
  // past the deadline.
  void fail_next_lock() { fail_next_lock_.store(true); }

  // When false (the default) delay_ms advances the virtual clock instead of sleeping,
  // so tests that walk a deadline 72 h into the future stay instant.
  void set_delay_realtime(bool realtime) { delay_realtime_.store(realtime); }

  // --- observation --------------------------------------------------------

  int total_contentions() const { return total_contentions_.load(); }
  int mutexes_created() const { return static_cast<int>(mutexes_.size()); }

  // Highest nesting depth any mutex ever reached. Must be 1: anything higher would mean
  // two holders at once.
  int max_mutex_depth() const {
    int worst = 0;
    for (rtos_mutex *m : mutexes_) {
      worst = std::max(worst, m->max_depth.load());
    }
    return worst;
  }

  // True if every mutex was released as many times as it was acquired.
  bool locks_balanced() const {
    for (rtos_mutex *m : mutexes_) {
      if (m->depth.load() != 0) {
        return false;
      }
    }
    return true;
  }

  int tasks_created() const { return static_cast<int>(tasks_.size()); }

  const std::vector<rtos_task *> &tasks() const { return tasks_; }

 private:
  static rtos_mutex_t *mutex_create_cb(void *ctx) {
    auto *self = static_cast<FakeRtos *>(ctx);
    auto *m = new rtos_mutex();
    self->mutexes_.push_back(m);
    return m;
  }

  static rtos_status_t mutex_lock_cb(void *ctx, rtos_mutex_t *m, uint32_t timeout_ms,
                                     bool *waited) {
    auto *self = static_cast<FakeRtos *>(ctx);
    if (m == nullptr) {
      return RTOS_ERROR;
    }
    if (self->fail_next_lock_.exchange(false)) {
      if (waited != nullptr) {
        *waited = false;
      }
      return RTOS_TIMEOUT;
    }

    if (m->mtx.try_lock()) {
      if (waited != nullptr) {
        *waited = false;
      }
    } else {
      // Genuine contention: someone else holds it.
      if (waited != nullptr) {
        *waited = true;
      }
      m->contentions.fetch_add(1);
      self->total_contentions_.fetch_add(1);

      if (timeout_ms == 0) {
        return RTOS_TIMEOUT;
      }
      if (!m->mtx.try_lock_for(std::chrono::milliseconds(timeout_ms))) {
        return RTOS_TIMEOUT;
      }
    }

    const int depth = m->depth.fetch_add(1) + 1;
    int prev = m->max_depth.load();
    while (depth > prev && !m->max_depth.compare_exchange_weak(prev, depth)) {
    }
    return RTOS_OK;
  }

  static void mutex_unlock_cb(void *ctx, rtos_mutex_t *m) {
    (void)ctx;
    if (m == nullptr) {
      return;
    }
    m->depth.fetch_sub(1);
    m->mtx.unlock();
  }

  static uint32_t mutex_holder_priority_cb(void * /*ctx*/, rtos_mutex_t *m) {
    return m == nullptr ? 0 : m->holder_priority.load();
  }

  static uint32_t now_ms_cb(void *ctx) {
    return static_cast<FakeRtos *>(ctx)->now_ms_.load();
  }

  static void delay_ms_cb(void *ctx, uint32_t ms) {
    auto *self = static_cast<FakeRtos *>(ctx);
    if (self->delay_realtime_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    } else {
      self->now_ms_.fetch_add(ms);
    }
  }

  static rtos_task_t *task_create_cb(void *ctx, const char *name, rtos_task_fn /*fn*/,
                                     void * /*arg*/, uint32_t stack_words,
                                     uint32_t priority) {
    auto *self = static_cast<FakeRtos *>(ctx);
    auto *t = new rtos_task();
    t->name = name == nullptr ? "" : name;
    t->stack_words = stack_words;
    t->priority = priority;
    // A plausible high-water mark: three quarters of the stack still free.
    t->high_water = stack_words / 4 * 3;
    self->tasks_.push_back(t);
    return t;
  }

  static uint32_t task_stack_high_water_cb(void * /*ctx*/, rtos_task_t *t) {
    return t == nullptr ? 0 : t->high_water;
  }

  static void yield_cb(void * /*ctx*/) { std::this_thread::yield(); }

  rtos_iface_t iface_{};
  std::vector<rtos_mutex *> mutexes_;
  std::vector<rtos_task *> tasks_;
  std::atomic<uint32_t> now_ms_{0};
  std::atomic<int> total_contentions_{0};
  std::atomic<bool> fail_next_lock_{false};
  std::atomic<bool> delay_realtime_{false};
};

} // namespace node::test

#endif // NODE_TEST_FAKE_RTOS_H
