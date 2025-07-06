/* main_target.cpp — the target's entry point.
 *
 * Three jobs, in order:
 *
 *   1. Bring-up diagnostics that only exist on real silicon: the clock actually
 *      achieved, the reset cause, the boot count, the heap, and a scan of the I2C bus.
 *      These are the readings bring-up-log.md Steps 4-6 ask for, and the bus scan is the
 *      single most useful test on a freshly assembled board.
 *
 *   2. Construct the node from the real HAL and start its five tasks.
 *
 *   3. Hand control to the scheduler. `main` never returns.
 *
 * It deliberately contains no application logic. The task bodies live in
 * `app/src/node.cpp`, which is the same code `tests/test_node.cpp` drives one tick at a
 * time on the host. That equivalence is the whole point of FR-0.6, and it is why this
 * file does not create tasks or drive peripherals itself: a second implementation of the
 * architecture here would mean the shipped image was not the tested one.
 */
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "app/board.h"
#include "app/comms.h"
#include "app/node.h"
#include "hal/gpio.h"
#include "hal/i2c.h"
#include "hal/system.h"
#include "hal/uart.h"
#include "hal/watchdog.h"
#include "hal_stm32f1.h"
#include "rtos/rtos.h"
#include "rtos_freertos.h"

namespace {

// ---------------------------------------------------------------------------
// Pin ids are defined twice on purpose. hal/ must not depend on app/, so the HAL keeps
// its own numbering and app/board.h keeps the application's. These assertions are what
// stops the two from drifting apart silently — a drift that would show up as the status
// LED lighting on the wrong pin, or worse, a trace pin driving an I2C line.
// ---------------------------------------------------------------------------
static_assert(node::kPinStatusLed == HAL_PIN_STATUS_LED, "pin id 0 disagreement");
static_assert(node::kPinTraceControl == HAL_PIN_TRACE_CONTROL, "pin id 1 disagreement");
static_assert(node::kPinTraceIsr == HAL_PIN_TRACE_ISR, "pin id 2 disagreement");
static_assert(node::kPinTraceTask == HAL_PIN_TRACE_TASK, "pin id 3 disagreement");
static_assert(node::kPinCount == HAL_PIN_COUNT, "pin count disagreement");

// The five-task stack budget must fit the part. A compile-time check rather than a
// comment, because the F103C8T6 has 20 KB and the FreeRTOS heap is a fixed .bss array:
// overrunning it is a link error at best and a silent shortfall at worst.
constexpr uint32_t kTaskStackWords =
    node::task_stack_words(node::TaskId::Supervisor) +
    node::task_stack_words(node::TaskId::Sensor) +
    node::task_stack_words(node::TaskId::App) + node::task_stack_words(node::TaskId::Comms) +
    node::task_stack_words(node::TaskId::Display);
static_assert(kTaskStackWords * 4U < 12U * 1024U,
              "task stacks plus TCBs must leave room for the kernel and the main stack");

// Build identity, so a board found on a bench months later says what it is running.
constexpr char kBuildDate[] = __DATE__;
constexpr char kBuildTime[] = __TIME__;

hal_uart_iface_t *g_uart = nullptr;
hal_i2c_iface_t *g_i2c = nullptr;

// ---------------------------------------------------------------------------
// Diagnostics, written straight to the UART interface.
//
// These run before the node exists, so they use the raw HAL interfaces rather than the
// arbiter: there is exactly one caller at this point, so a mutex would add nothing, and a
// banner is more useful if it cannot fail for want of one.
//
// The text is built with LineBuilder rather than printf: no varargs, no float formatting,
// and none of newlib's formatter linked into a 64 KB image.
// ---------------------------------------------------------------------------
void emit(const char *text) {
  if (g_uart == nullptr) {
    return;
  }
  const size_t length = std::strlen(text);
  (void)g_uart->write(g_uart->ctx, reinterpret_cast<const uint8_t *>(text), length, 50);
  // The terminator is sent separately so the payload length is exactly the text length.
  const uint8_t newline = static_cast<uint8_t>('\n');
  (void)g_uart->write(g_uart->ctx, &newline, 1, 50);
}

const char *reset_cause_name(hal_reset_cause_t cause) {
  switch (cause) {
    case HAL_RESET_POWER_ON:     return "power-on";
    case HAL_RESET_EXTERNAL_PIN: return "external-pin";
    case HAL_RESET_WATCHDOG:     return "watchdog";
    case HAL_RESET_SOFTWARE:     return "software";
    case HAL_RESET_LOW_POWER:    return "low-power";
    case HAL_RESET_UNKNOWN:      return "unknown";
  }
  return "unknown";
}

void print_banner(hal_system_iface_t *system) {
  char text[96];

  node::LineBuilder line(text, sizeof(text));
  line.str("boot ").str(node::kNodeName).str(" fw 0.1.0 built ").str(kBuildDate).ch(' ').str(
      kBuildTime);
  emit(line.c_str());

  node::LineBuilder clock(text, sizeof(text));
  clock.str("clock ")
      .u32(static_cast<uint32_t>(SystemCoreClock / 1000000U))
      .str(" MHz ")
      .str(hal_stm32f1_hse_ok() ? "HSE+PLL" : "HSI fallback (crystal did not start)");
  emit(clock.c_str());

  node::LineBuilder reset(text, sizeof(text));
  reset.str("reset ").str(reset_cause_name(system->reset_cause(system->ctx)));
  reset.str(" boots=").u32(system->boot_count(system->ctx));
  emit(reset.c_str());

  node::LineBuilder heap(text, sizeof(text));
  heap.str("heap ").u32(static_cast<uint32_t>(xPortGetFreeHeapSize())).str(" bytes free");
  emit(heap.c_str());
}

// Probe every legal 7-bit address and report which ones answer.
//
// The first thing to run on a new board: it separates "the MCU is alive and the bus is
// wired" from "the MCU is alive", in one command, without needing either device's driver
// to be correct. It relies on the zero-length transfer, which hal/i2c.h defines as an
// address probe — a transfer that genuinely puts the address on the bus.
void scan_bus() {
  const char hex[] = "0123456789ABCDEF";
  uint32_t found = 0;

  for (uint16_t addr = 0x03U; addr <= 0x77U; ++addr) {
    if (g_i2c->transfer(g_i2c->ctx, addr, nullptr, 0, nullptr, 0, 20) != HAL_I2C_OK) {
      continue;
    }

    char text[40];
    node::LineBuilder line(text, sizeof(text));
    line.str("  found 0x");
    line.ch(hex[(addr >> 4) & 0x0FU]).ch(hex[addr & 0x0FU]);
    if (addr == node::kAddrSht31) {
      line.str("  (SHT31 sensor)");
    } else if (addr == node::kAddrSsd1306) {
      line.str("  (SSD1306 display)");
    }
    emit(line.c_str());
    ++found;
  }

  if (found == 0U) {
    // Being explicit matters: an empty scan and a broken scan look identical otherwise.
    emit("i2c scan: no devices answered (check pull-ups, wiring and power)");
    return;
  }

  char text[64];
  node::LineBuilder line(text, sizeof(text));
  line.str("i2c scan: ").u32(found).str(" device(s)");
  emit(line.c_str());
}

// ---------------------------------------------------------------------------
// The node
// ---------------------------------------------------------------------------
//
// Constructed at file scope so it lives in .bss rather than on the main stack, and before
// the scheduler on purpose: Node's constructor only creates mutexes, which is legal
// here, while init() and start() talk to hardware and create tasks.
//
// That split is concrete rather than stylistic. Ssd1306::init() flushes over the DMA
// path, and that path waits on a semaphore given by the transfer-complete interrupt. A
// blocking semaphore wait before vTaskStartScheduler() is not legal in FreeRTOS, so
// initialising the panel from main() would be a well-disguised bug.
node::NodeHardware make_hardware() {
  node::NodeHardware hardware;
  hardware.i2c = hal_i2c_stm32f1();
  hardware.rtos = rtos_freertos_iface();
  hardware.gpio = hal_gpio_stm32f1();
  hardware.uart = hal_uart_stm32f1();
  hardware.watchdog = hal_watchdog_stm32f1();
  hardware.system = hal_system_stm32f1();
  return hardware;
}

node::Node g_node(make_hardware());

void bootstrap_task(void *argument) {
  (void)argument;

  // Neither failure path here feeds the watchdog, because the supervisor task is what
  // owns feeding and it will not be running. The independent IWDG then resets the board,
  // which is the intended outcome rather than a hang: a node that cannot initialise
  // should not sit there looking alive.
  if (!g_node.init()) {
    emit("[init] node init FAILED - halting; the watchdog will reset this board");
    vTaskDelete(nullptr);
    return;
  }

  if (!g_node.start()) {
    emit("[init] task creation FAILED - halting; the watchdog will reset this board");
    vTaskDelete(nullptr);
    return;
  }

  {
    char text[64];
    node::LineBuilder line(text, sizeof(text));
    line.str("[init] ").u32(g_node.stats().tasks_created).str(" tasks running");
    emit(line.c_str());
  }

  // Nothing left for this task. The five node tasks are the application, and a sixth task
  // that existed only to report would also be a sixth task nobody watches (FR-0.4). The
  // host reaches the same information through the comms task's `stat` command.
  vTaskDelete(nullptr);
}

} // namespace

extern "C" int main(void) {
  // Clocks, the HAL timebase, the DWT microsecond counter, the backup-domain boot
  // counter, the GPIO table, I2C1 and USART1. The watchdog is deliberately not started
  // here: FR-0.4 requires a task to own that decision.
  hal_stm32f1_init();

  g_uart = hal_uart_stm32f1();
  g_i2c = hal_i2c_stm32f1();

  print_banner(hal_system_stm32f1());
  scan_bus();

  // Bootstrap at the supervisor's priority, so it finishes before anything it starts can
  // be preempted mid-initialisation.
  const BaseType_t created =
      xTaskCreate(bootstrap_task, "bootstrap", 256, nullptr,
                  node::task_priority(node::TaskId::Supervisor), nullptr);
  if (created != pdPASS) {
    emit("[init] cannot create the bootstrap task; no heap left");
    for (;;) {
      // Spin rather than reset: with no tasks there is no supervisor to feed the
      // watchdog, so the board would reset straight back into this same failure, and the
      // banner above would be lost each time.
    }
  }

  vTaskStartScheduler();

  // Reaching here means the scheduler returned, which on this target happens only if the
  // idle task could not be created — that is, the heap ran out during start-up.
  for (;;) {
  }
}

// ---------------------------------------------------------------------------
// CRT stubs required by -nostartfiles
// ---------------------------------------------------------------------------
//
// The image is linked with -nostartfiles so that the vendored startup file owns the
// vector table and the reset path. That also drops crti.o, which is where newlib's
// `_init` normally comes from — and the startup file calls `__libc_init_array`, which
// calls `_init`. Without these two the link fails with `undefined reference to _init`,
// which reads like a library problem rather than a consequence of a deliberate link
// flag.
//
// They are empty on purpose: `_init`/`_fini` are where the C runtime would run
// constructor and destructor *sections*, and `__libc_init_array` already walks
// .init_array itself, which is what actually runs the static constructors in this
// C++ application.
extern "C" void _init(void) {}
extern "C" void _fini(void) {}
