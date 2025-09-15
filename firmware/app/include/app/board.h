/* board.h — every tunable number the application core needs, in one place.
 *
 * This header settles the Phase 0 decisions listed in design-spec.md §5.3 that are
 * not hardware-determined: task count and priorities (#1), the contention-stress
 * duration (#2), the recovery sequence shape (#3), the refresh strategy (#5), and the
 * sensor set (#8). Keeping them here rather than scattered through the sources is what
 * makes them reviewable against the spec.
 *
 * Nothing here is measured yet. Task periods and stack sizes are provisional and are
 * to be replaced by the NFR-1.10/NFR-1.11 measurements; the comments say which
 * requirement will correct each number.
 */
#ifndef APP_BOARD_H
#define APP_BOARD_H

#include <cstddef>
#include <cstdint>

namespace node {

// ---------------------------------------------------------------------------
// Node identity and purpose
// ---------------------------------------------------------------------------
// A greenhouse node: soil moisture is the primary (analog) quantity, on the Phase 1
// timer-triggered ADC path. Phase 0 reads temperature and humidity over I2C, which is
// what makes the shared-bus arbitration of FR-0.2 a real problem rather than a
// contrived one: two different devices, two different tasks, one bus.
constexpr char kNodeName[] = "greenhouse-node-01";

// Short label for the OLED. Deliberately restricted to characters the panel's 5x7 font
// actually covers — see app/display.h — because an uncovered character renders as a
// filled box, and a header of boxes is worse than a short header.
constexpr char kNodeShortName[] = "NODE";

// ---------------------------------------------------------------------------
// Shared I2C bus
// ---------------------------------------------------------------------------
// All I2C peripherals sit on one bus behind the arbiter (FR-0.2).
constexpr uint32_t kI2cBusHz = 400'000;
constexpr uint32_t kI2cTransferTimeoutMs = 20;

// 7-bit device addresses.
//
// The ambient sensor is an "AHT20 + BMP280" combo module: two dies, one carrier, so two
// addresses on the same bus. Only the AHT20 is read. The BMP280 is deliberately NOT
// driven — it answers the bring-up bus scan at 0x76 (0x77 when SDO is strapped high,
// which the module's pull-down normally decides) and nothing in this firmware talks to
// it, because pressure is not on any requirement path and Bosch's compensation routine
// is a sizeable piece of work that would compete with Phase 2 for time. If a later phase
// wants pressure, that is a new driver against this address, not a change to the AHT20
// one.
constexpr uint16_t kAddrAht20 = 0x38;   // temperature + humidity (sensor task)
constexpr uint16_t kAddrBmp280 = 0x76;  // present on the bus, deliberately unread
constexpr uint16_t kAddrSsd1306 = 0x3C; // 128x64 OLED (display task)

// AHT20 measurement time. The datasheet's maximum is 80 ms, and unlike the SHT3x there is
// no "low repeatability" mode to trade accuracy for speed, so the wait is the datasheet's
// number with no margin to shave: the alternative is to poll the status byte's busy bit
// (see Aht20::busy), which the driver exposes but SensorService does not need, because the
// sensor task's period (1000 ms) is twelve times this wait and the bus is released for the
// whole of it either way.
constexpr uint32_t kAht20ConversionMs = 80;

// ---------------------------------------------------------------------------
// I2C bus recovery (FR-0.3)
// ---------------------------------------------------------------------------
// A slave that was mid-byte when the master reset can hold SDA low forever. The
// release is nine SCL pulses with SDA released, then a STOP, then peripheral re-init.
//
// Nine is not arbitrary: a slave that has lost bit alignment can be up to eight bits
// into a byte, so eight clocks are not always enough to walk it to the ACK bit; the
// ninth guarantees it. We always emit all nine rather than stopping early when SDA
// goes high, because a fixed, deterministic edge sequence is what makes the scope
// capture in FR-0.3 a pass/fail check instead of a judgement call.
constexpr int kRecoveryClocks = 9;
constexpr uint32_t kRecoveryHalfPeriodUs = 5; // ~100 kHz bit-banged, well under 400 kHz

// One recovery attempt, followed by one retry of the transfer.
//
// Deliberately not two: a recovery reports whether it cleared the bus, so a second
// identical attempt in the same transfer is redundant — if the bus is still not idle,
// clocking it again changes nothing. Each attempt also holds the I2C mutex (FR-0.2),
// so a longer retry chain would block the other tasks on the bus while achieving
// nothing. Failing after one retry hands the decision back to the caller.
constexpr int kRecoveryMaxAttempts = 1;

// ---------------------------------------------------------------------------
// Scheduling (FR-0.1)
// ---------------------------------------------------------------------------
// Five concurrent tasks. FreeRTOS priorities: a higher number preempts a lower one.
//
// The ordering is not cosmetic. The supervisor is highest so the watchdog is fed on
// time even when everything below is busy. Display is lowest on purpose: it is the
// cosmetic job, and it is also the long-running holder of the I2C mutex, which is
// exactly the priority-inversion case FR-1.7 instruments.
enum class TaskId : uint8_t {
  Supervisor = 0,
  Sensor = 1,
  App = 2,
  Comms = 3,
  Display = 4,
  Count = 5,
};

constexpr uint32_t kTaskCount = static_cast<uint32_t>(TaskId::Count);

// ---- The task budget ------------------------------------------------------
//
// One table rather than three switches, because these properties are reviewed together
// and separately they hide their relationships. Two of them are deliberately equal, and as
// duplicated switch branches those read exactly like a copy-paste slip — which is what a
// linter sees too (clang-tidy's bugprone-branch-clone). Stating them once, side by side,
// says "intended" instead of "suspicious", and makes the whole 20 KB budget readable at a
// glance.
//
// Priorities (FreeRTOS: a higher number preempts a lower one):
//   4 supervisor  highest, because feeding the watchdog late is the one failure that
//                 resets the node
//   3 sensor      the node's primary input; a missed sample is a missed measurement
//   3 app         owns application state, and becomes the Phase 1 control loop. Equal to
//                 the sensor on purpose: neither should preempt the other, and time
//                 slicing between two critical-path tasks is the intended relationship
//   2 comms       a host that stops reading must not stall the node
//   1 display     cosmetic, and the long-running holder of the I2C mutex, which is
//                 exactly the priority-inversion case FR-1.7 instruments
// `name` is what FreeRTOS keeps a pointer to and what a debugger shows, so it is short
// but not cryptic. It lives here rather than in a second table so that a task's whole
// specification is one row.
struct TaskSpec {
  TaskId id;
  const char *name;
  uint32_t priority;
  uint32_t stack_words;
  uint32_t period_ms;
};

// The app task's period, and the reason it is 25 ms rather than the 100 ms Phase 0 ran at.
//
// Phase 1 puts the actuator gate inside this task: the window decides on and off, and the
// task samples that decision. The sampling interval is therefore the resolution of the
// gate's edge, and the number it has to be good against is kControlMinOnTimeMs — the
// shortest pulse the loop is willing to deliver. 25 ms means the shortest pulse the loop
// can command (50 ms) is at least two task periods long, so its on-edge and its off-edge
// each land on a tick of their own and the pulse cannot be swallowed between two samples.
// Any period at or above the minimum on-time breaks that: at 50 ms a legal 50 ms pulse
// would be a coin flip between being delivered and being missed entirely, and the control
// loop's authority would depend on the phase of the scheduler.
//
// The cost is scheduling: the app task wakes 40 times a second instead of 10. It is the
// cheapest task in the image (a comparison, a window evaluation and at most one GPIO
// write per tick) and the control evaluation itself is not tied to this period — see the
// control-period note in the acquisition section below — so the extra wake-ups buy edge
// resolution and nothing else.
//
// The sensor task keeps its 1000 ms period. It is not the acquisition consumer, for the
// reason recorded with kAcquisitionDrainPeriodMs: it blocks in the AHT20's 80 ms
// conversion, so a drain on its tick would be paced by an I2C transfer rather than by the
// block rate.
constexpr uint32_t kAppTaskPeriodMs = 25;

// Stack sizes are in words, as FreeRTOS counts them (4 bytes). Provisional: NFR-1.11
// requires these to be right-sized from uxTaskGetStackHighWaterMark() with 25 % headroom
// kept, and the F103C8T6 has only 20 KB of RAM, so raising one is a real budget decision
// rather than a free change.
constexpr TaskSpec kTaskSpecs[kTaskCount] = {
    {TaskId::Supervisor, "supervisor", 4, 192, 250},
    {TaskId::Sensor, "sensor", 3, 320, 1000},
    {TaskId::App, "app", 3, 320, kAppTaskPeriodMs},
    {TaskId::Comms, "comms", 2, 384, 50},
    {TaskId::Display, "display", 1, 384, 200},
};

// The table must be indexed by TaskId, because everything below looks a row up by casting
// the id. A row inserted in the wrong place would silently give a task another task's
// priority, so the invariant is checked rather than trusted.
constexpr bool task_table_is_indexed_by_id() {
  for (uint32_t row = 0; row < kTaskCount; ++row) {
    if (static_cast<uint32_t>(kTaskSpecs[row].id) != row) {
      return false;
    }
  }
  return true;
}
static_assert(task_table_is_indexed_by_id(), "kTaskSpecs must be ordered by TaskId");

// An out-of-range id resolves to the supervisor's row rather than reading past the array.
// A wrong id is a programming error, and the alternative — returning a default — would
// hide it at the one place it matters.
constexpr const TaskSpec &task_spec(TaskId id) {
  const uint32_t index = static_cast<uint32_t>(id);
  return kTaskSpecs[index < kTaskCount ? index : 0];
}

constexpr uint32_t task_priority(TaskId id) { return task_spec(id).priority; }
constexpr uint32_t task_stack_words(TaskId id) { return task_spec(id).stack_words; }
constexpr uint32_t task_period_ms(TaskId id) { return task_spec(id).period_ms; }

// ---------------------------------------------------------------------------
// Phase 1 — analog acquisition (decision recorded in design-spec.md 6.1)
// ---------------------------------------------------------------------------
// fs = 1 kHz, triggered by a hardware timer, DMA into a double buffer (FR-1.1).
//
// The rate is chosen for oversampling, not for bandwidth. A soil-moisture probe's
// information lives below a few Hz, so the samples exist to be averaged: averaging N
// samples buys about log4(N) bits of effective resolution against the converter's noise
// floor. NFR-1.12 measures how much is actually realised rather than assuming the
// textbook figure, because that gain only materialises if the noise genuinely dithers
// the input.
constexpr uint32_t kAdcSampleRateHz = 1'000;
constexpr size_t kAdcBlockSamples = 100;  // one published value per completed block
constexpr uint32_t kAdcPublishRateHz =
    kAdcSampleRateHz / static_cast<uint32_t>(kAdcBlockSamples);  // 10 Hz

// The anti-alias filter, as components, because the corner is a hardware decision that
// the Phase 1 breadboard and then the Phase 3 board both have to implement (FR-1.2,
// FR-3.10).
//
// The corner sits one decade below Nyquist rather than at it, and the reason matters: a
// single pole rolls off at only -20 dB/decade, so it cannot deliver 12-bit anti-aliasing
// on its own — pushing an alias below one LSB would need a corner thousands of times
// below Nyquist, which would also destroy the signal. What the RC is for is a defined,
// predictable rolloff that attenuates what is actually present on this board (mains hum
// and its harmonics, gate-driver switching) and that can be swept and compared against an
// LTspice prediction, which is what bring-up-log step 7 asks for.
//
//   fc = 1 / (2*pi*R*C) = 1 / (2*pi*10k*150n) = 106.1 Hz, against a Nyquist of 500 Hz.
constexpr uint32_t kAfeResistorOhms = 10'000;
constexpr uint32_t kAfeCapacitorPicoFarads = 150'000;  // 150 nF
constexpr uint32_t kAfeCornerMilliHz = 106'100;

// ---------------------------------------------------------------------------
// Phase 1 — the ADC configuration (FR-1.1) and where the values live
// ---------------------------------------------------------------------------
// The peripheral numbers behind kAdcSampleRateHz, worked out here rather than left to
// the HAL file, because they are a consequence of the clock tree and the source
// impedance rather than of any one driver, and because someone changing the sample rate
// has to see them.
//
//   ADCCLK          PCLK2 / 6 = 72 MHz / 6 = 12 MHz. The datasheet's ceiling is 14 MHz,
//                   and 12 MHz is what the F103's 72 MHz tree gives with the largest
//                   permissible divider. RCC_ADCPCLK2_DIV6.
//   Sampling time   71.5 ADCCLK cycles.
//   Conversion      71.5 + 12.5 = 84 ADCCLK cycles for 12 bits = 7.0 us at 12 MHz.
//   Ceiling         1 / 7.0 us = 142.8 kSPS with this sampling time. (With the shortest
//                   sample time the same ADC reaches ~600 kSPS, which is the figure
//                   hal/adc.h quotes; the difference is entirely the sampling time, and
//                   the sampling time is set by the source, see below.)
//   Requested       1 kHz, which is 143x below the ceiling — the point of oversampling is
//                   the average, not the rate.
//
// Why 71.5 cycles for a ~10 kohm source, as arithmetic rather than as a habit. The
// sample-and-hold capacitor is 8 pF and the internal switch resistance is at most 1 kohm
// (datasheet), so a 10 kohm source presents 11 kohm, and the capacitor has to settle to
// better than half an LSB of a 12-bit conversion:
//
//   t_sample >= (R_ADC + R_AIN) x C_ADC x ln(2^(n+2))
//             = 11 kohm x 8 pF x ln(2^14)
//             = 88 ns x 9.704
//             = 854 ns
//
// 71.5 cycles at 12 MHz is 5.96 us, seven times that. The shortest setting (1.5 cycles =
// 125 ns) misses by 7x the other way, and 7.5 cycles (625 ns) is still short: with the RC
// of the anti-alias filter (10 kohm, 150 nF) the filter's capacitor supplies the sample
// charge rather than the resistor, but assuming the resistor alone is the conservative
// case and it is the case that has to hold before the AFE exists — "straight to the pin
// for first light", with a probe whose output impedance is not specified.
//
// The 150 nF of that RC recovers between samples by exp(-100 us / 1.5 ms), i.e. it is
// back to within 7 % of any droop long before the next conversion; the droop itself is
// 8 pF / 150 nF = 0.005 % of the sample voltage.
constexpr uint32_t kAdcClockHz = 12'000'000;      // PCLK2 / 6
constexpr uint32_t kAdcSampleTimeCycles = 71;     // 71.5 rounded down, see the ADC HAL
constexpr uint32_t kAdcConversionCycles = 84;     // 71.5 sample + 12.5 successive approx.
constexpr uint32_t kAdcConversionMicros = 7;      // 84 cycles at 12 MHz, rounded up

// One DMA half-buffer: the block period and the rate at which a value is published.
constexpr uint32_t kAdcBlockPeriodMs =
    (static_cast<uint32_t>(kAdcBlockSamples) * 1'000U) / kAdcSampleRateHz;  // 100 ms
static_assert(static_cast<uint32_t>(kAdcBlockSamples) * 1'000U ==
                  kAdcBlockPeriodMs * kAdcSampleRateHz,
              "the block period must be exact, or a block is not a whole number of ms");


// Completed blocks awaiting averaging. Two is the minimum that survives one late consumer;
// each extra slot costs kAdcBlockSamples * 2 bytes (200 here) and only helps if the
// consumer stalls for longer than that. Past that point the overrun counter is the honest
// answer rather than an ever-growing queue.
constexpr size_t kAdcBlockRingCapacity = 2;

// Two-point probe calibration: the averaged counts measured in air-dry soil and in
// saturated soil. Linear between them, clamped outside.
//
// These are PLACEHOLDERS and must be replaced with measured values at bring-up. The
// transfer function of a resistive probe depends on its geometry, its supply and the
// soil, so inventing a formula here would be a guess dressed as physics; two measured
// points and a line between them is the honest model. They are also runtime-tunable
// (FR-1.4), so recalibrating does not mean reflashing.
constexpr uint16_t kProbeCountsDry = 3'600;
constexpr uint16_t kProbeCountsWet = 1'400;

// ---------------------------------------------------------------------------
// Phase 1 — instrumentation tunables
// ---------------------------------------------------------------------------
// The two recorders — period/latency statistics (NFR-1.8, NFR-1.9) in app/timing.h and
// the step-response capture (FR-1.5) in app/step_capture.h — were written before this
// file was writable, so their tunables ended up in their own headers. They are tunables
// in exactly the sense this file uses the word: properties of the node's Phase 1
// configuration, reviewed here against the measurement plan, rather than implementation
// details of the code that consumes them. They are stated once, here; the two headers
// alias them and explain what each one bounds.

// Records in one step-response capture.
//
// 128 records at 16 bytes is 2 KB — a tenth of the F103C8T6's 20 KB of RAM — and that is
// the honest cost of not allocating a variable-size buffer on a part that cannot afford
// one. It is deliberately sized for the fast case and not the slow one: it has to hold a
// response that settles, so a faster control period needs a bigger buffer or a shorter
// capture, and the host analyser says "not settled in the captured window" rather than
// inventing a settling time when the buffer fills first.
constexpr size_t kStepCaptureCapacity = 128;

// How far the setpoint must move from its armed value to count as a step, in the
// setpoint's own units (permille of soil moisture, so this is 5 % of the full span).
// A threshold rather than an exact comparison because the setpoint is a value a host can
// nudge over the link, and a one-count change is a typo, not a step response.
constexpr int32_t kStepThresholdPermille = 50;

// How long a capture may run after the step, in milliseconds. Measured from the step,
// not from arming: arming early is the normal case (the recorder is armed, then the host
// commands a new setpoint) and must not consume the window.
constexpr uint32_t kStepMaxDurationMs = 30'000;

// Samples per timing measurement window.
//
// The window is bounded rather than unbounded because of the arithmetic guarantee in
// app/timing.cpp: the sums stay exact only while the window is finite, and a statistic
// that silently starts wrapping after four and a half hours unattended — NFR-0.7 is a
// 72 h run — would be worse than no statistic at all. When the window is full, further
// samples are ignored and full() says so, so the caller reports and resets rather than
// losing the window silently.
constexpr uint32_t kTimingWindowSamples = 1024;

// The largest measurement a timing series will record, in microseconds. Anything above it
// is recorded as the ceiling and counted in oversized_samples(), so an excursion is
// visible as "some samples hit the ceiling" rather than being folded into the mean as if
// it were the ceiling.
//
// Two seconds, which is twice the 1 s control period this measures. Both bounds were
// chosen together, and the pairing is the point:
//
//   * The ceiling must sit well ABOVE the nominal. It was 1 s while the nominal was the
//     100 ms app task period — that is, ten times the value being measured. Once the
//     control period became 1 s, a 1 s ceiling would have clamped to exactly the nominal
//     every excursion above it, which is precisely what NFR-1.8 looks for. A jitter
//     measurement that hides its own tail is worse than no measurement.
//   * The window has to shrink to pay for it. The mean and standard deviation come from
//     uint64 sum and sum-of-squares, and with n samples bounded at x the two intermediates
//     are n*sum(x^2) and (sum x)^2, both n^2*x^2/4 or less. At 2048 x 2,000,000 those
//     reach 1.68e19 against a uint64 maximum of 1.84e19 — 91 % of the range, with no room
//     for the arithmetic to be reasoned about. At 1024 x 2,000,000 they reach 4.19e18,
//     which is 23 %, the same margin the original pairing had.
//
// 1024 seconds of history is still 17 minutes at this period, which is ample for a
// worst-case figure; the window is a diagnostic aid, not a log.
constexpr uint32_t kTimingMaxValueUs = 2'000'000;

// ---------------------------------------------------------------------------
// Phase 1 — closed-loop control
// ---------------------------------------------------------------------------
// The controlled variable is soil moisture in permille of the probe's calibrated span
// (the Phase 1 analog channel: 0 is air-dry, 1000 is saturated). The I2C temperature and
// humidity readings stay monitored-only — they are inputs to a report, not to the loop.
//
// The numbers below are a decision record, not a tuning session. They are duplicated
// here in the form the code is written against, so that a reviewer of the control law and
// a reviewer of the decision cannot end up looking at different values:
//
//   1. controlled variable  soil moisture permille from AcquisitionService
//   2. setpoint             450 permille (45.0 %), runtime-tunable (FR-1.4)
//   3. control period       1000 ms
//   4. output               time-proportioned: on for duty x window, not fast PWM
//   5. minimum on-time      50 ms; a computed on-time in (0, 50) ms becomes 0
//   6. maximum duty         500 permille (50 % of the window)
//   7. control law          PID with kd defaulting to 0, integral clamped
//   8. fail-safe            an absent or stale reading forces the output to 0
//   9. tuning publication   snapshot plus atomic index swap, never a mutex
//  10. no 1 kHz control loop
//
// The period is 1000 ms and is deliberately not 1 kHz: a 1 kHz control loop on this part
// would put a PID evaluation, a setpoint read and a gate decision on a 1 ms deadline that
// the scheduling measurements cannot yet show is achievable, and soil moisture has no
// information above a few Hz to control on. 1000 ms is also what makes the output
// time-proportioned rather than PWM: the window is the period.
constexpr uint32_t kControlPeriodMs = 1'000;

// ---------------------------------------------------------------------------
// Phase 1 — how often the completed blocks are collected, and what the control
// period is counted in
// ---------------------------------------------------------------------------
// Both of these live here, after kControlPeriodMs and kAdcBlockRingCapacity, because both
// are statements about the relationship between the block rate and the control period and
// neither can be read without those two numbers.
//
// THE DRAIN CADENCE. The consumer of the DMA blocks is the app task, every
// kAcquisitionDrainEveryAppTicks of its 25 ms ticks. Both bounds on that period are hard,
// and they are why the cadence is stated here rather than left to whichever task happens
// to call update():
//
//   * SLOWER than the block period, or the drain finds nothing. An empty drain is not a
//     no-op: AcquisitionService records it as a failed publish cycle — an empty update
//     means the producer has stopped, which is the whole staleness mechanism — so a
//     consumer that polls faster than the producer manufactures "stale" readings, and the
//     control loop's fail-safe (decision 8) turns each one into a skipped window. This is
//     the constraint that rules out draining from a fast tick.
//   * FASTER than the ring can absorb: kAdcBlockRingCapacity blocks of margin, so a drain
//     window that spans more than kAdcBlockRingCapacity + 1 blocks loses the oldest. 150 ms
//     against a 200 ms budget means a window holds one or two completed blocks and both are
//     published, so nothing is dropped in the steady state — over a second, ten drains and
//     ten blocks, which is the 10 Hz the block size and rate were chosen for. The margin is
//     also what absorbs a window that runs late: it takes a 200 ms delay before the third
//     block arrives and the overrun counter starts moving. A drain window of exactly one
//     block period would instead sit on the block boundary and lose the race with the DMA
//     interrupt about as often as it won it.
//
// Why not the sensor task, which is where the other input is read: SensorService::update()
// blocks for the AHT20's 80 ms conversion, so a drain on that tick would be paced by an
// I2C transfer (150 + 80 = 230 ms, past the 200 ms the ring absorbs) and the acquisition's
// cadence would depend on how long a humidity reading took. The app task does no I2C, so
// its tick is the one place in the image where a period is what the table says it is.
//
// THE CONTROL PERIOD. design-spec.md §6: "Control period is set by a hardware timer, not
// vTaskDelay, so period accuracy does not depend on tick granularity." The loop is
// therefore evaluated when the acquisition's own block numbering crosses a boundary, never
// on a delay:
//
//   10 blocks x 100 samples / 1000 Hz = 1 s, exactly.
//
// Those blocks are numbered by the DMA half/full-transfer interrupts that TIM2's TRGO
// produces, so the count is hardware time: a control period is ten hardware-timed blocks
// whether the scheduler ran the app task 39 times inside it or 41, and the integral's dt
// is derived from the same quantity. The app task's 25 ms period then decides only *when
// inside that second* the crossing is noticed, which is the scheduling quantity NFR-1.8
// measures and the trace pin marks — not the control period itself.
constexpr uint32_t kAcquisitionDrainEveryAppTicks = 6;
constexpr uint32_t kAcquisitionDrainPeriodMs = kAppTaskPeriodMs * kAcquisitionDrainEveryAppTicks;
static_assert(kAcquisitionDrainPeriodMs > kAdcBlockPeriodMs,
              "draining faster than the producer records false stale cycles");
static_assert(kAcquisitionDrainPeriodMs <=
                  static_cast<uint32_t>(kAdcBlockRingCapacity) * kAdcBlockPeriodMs,
              "a drain window longer than the ring loses the oldest blocks");

constexpr uint32_t kAdcBlocksPerControlPeriod = kControlPeriodMs / kAdcBlockPeriodMs;  // 10
static_assert(kAdcBlocksPerControlPeriod * kAdcBlockPeriodMs == kControlPeriodMs,
              "the control period must be a whole number of acquisition blocks");

constexpr int32_t kControlSetpointPermille = 450;

// A window whose computed on-time is nonzero but shorter than this is skipped entirely
// rather than delivered. A 49 ms pulse does not make a pump deliver 4.9 % of its rated
// flow: below the minimum the pump does not prime, the gate driver's rise time is a
// measurable fraction of the pulse, and the delivered volume becomes a property of the
// plumbing rather than of the commanded number. Zero is the honest command, and the skip
// is counted so that a loop living entirely below the threshold is visible as "every
// window is being skipped" instead of as a mysterious failure to act.
constexpr uint32_t kControlMinOnTimeMs = 50;

// Half the window. The cap is what keeps a saturated controller from holding the pump on
// for the whole period, and it is also what bounds the integral's authority below.
constexpr int32_t kControlMaxDutyPermille = 500;

// ---------------------------------------------------------------------------
// Phase 1 — the board pins the analog channel and the actuator use
// ---------------------------------------------------------------------------
// These three are NOT ids in hal_gpio_iface_t, and that is deliberate. They are not
// general-purpose pins a task drives: PA0 belongs to ADC1, PA1 to the one output the
// fail-safe owns, and PA2 is reserved, so exposing them as gpio ids would invite an
// application task to write to the probe input or to the gate behind the control loop's
// back — which is the one thing the actuator interface exists to make impossible. They
// are therefore facts about the board, stated here for the reviewer, and the mapping to
// port and bit lives in the HAL files that own the peripherals
// (hal/stm32f1/hal_adc_stm32f1.c, hal/stm32f1/hal_actuator_stm32f1.c) with
// hal_stm32f1.h's hardware assignment as the reference.
//
//   | Function                | Pin | Note                                              |
//   |-------------------------|-----|---------------------------------------------------|
//   | Analog in from probe    | PA0 | ADC1_IN0. Through the AFE buffer + RC when that   |
//   |                         |     | exists (FR-1.2, FR-3.10); straight to the pin for  |
//   |                         |     | first light, which is why the sampling time is     |
//   |                         |     | sized for a ~10 kohm source.                       |
//   | Actuator gate           | PA1 | Low-side MOSFET gate, push-pull output, driven     |
//   |                         |     | only through hal/actuator.h. Needs the external    |
//   |                         |     | pull-down documented in that header.               |
//   | Probe power gate        | PA2 | RESERVED, NOT DRIVEN. Not needed for corrosion —   |
//   |                         |     | a capacitive probe does not electrolyse, so        |
//   |                         |     | continuous power is electrically fine — but the    |
//   |                         |     | probe draws about 5 mA continuously, which is the  |
//   |                         |     | largest single item in Phase 4's budget. PA2 is    |
//   |                         |     | left unallocated here so that duty-cycling the     |
//   |                         |     | probe is a Phase 4 change to one pin and not a     |
//   |                         |     | re-layout; nothing in this firmware configures it. |
//
// Already taken, and so unavailable for any of the above: PC13 (status LED), PB0/PB1/PB5
// (trace pins), PB6/PB7 (I2C1), PA9/PA10 (USART1), PA13/PA14 (SWD).
//
// The numbers below are port bits on GPIOA, numbered from 0 as the reference manual
// numbers them. They are named rather than written as literals so that the HAL files and
// the comments above can be read against each other, and the prefix is deliberately
// different from kPin* to keep them from being mistaken for gpio ids.
constexpr uint32_t kProbeAnalogPortBit = 0;  // PA0, ADC1_IN0
constexpr uint32_t kActuatorGatePortBit = 1; // PA1, low-side MOSFET gate
constexpr uint32_t kProbePowerPortBit = 2;   // PA2, reserved — see the table above

// PID gains, in the fixed-point convention the loop's arithmetic uses (there is no float
// anywhere in this firmware: the F103 has no FPU, so a float would mean soft-float
// library calls for a number that is printed with three digits).
//
//   P_duty_permille = kp_milli * error_permille / 1000
//   I_duty_permille = ki_milli * integral_permille_seconds / 1000
//   D_duty_permille = kd_milli * (error - previous_error) * 1000 / dt_ms / 1000
//
// So kp_milli = 2000 means 2.0 duty-permille per error-permille: a 150 permille error
// commands 300 permille of duty. ki_milli = 50 means a 145 permille error sustained for
// 20 s contributes 145 permille of duty — the worked example the gains were chosen from.
//
// kd defaults to 0 and is off for a reason rather than by omission: the measurement is an
// averaged 12-bit reading whose noise the derivative term multiplies. The D term is left
// in the interface (FR-1.4 makes it runtime-settable) because a plant with a long dead
// time and a noisy sensor is exactly where the argument for and against it is made with
// numbers, and tests/test_control.cpp produces that measurement rather than assuming it.
constexpr int32_t kControlDefaultKpMilli = 2'000;
constexpr int32_t kControlDefaultKiMilli = 50;
constexpr int32_t kControlDefaultKdMilli = 0;

// Bounds for runtime tuning (FR-1.4). A value outside them is REFUSED with an error line
// rather than clamped silently: a typed-in gain is a number a person believes, and
// quietly installing 100 instead of the 1000 that was asked for produces a loop that
// behaves differently from the one on the other end of the link, with nothing on the wire
// to say so.
//
// The ranges are wide — 50x the default gain — because the useful gain depends on the
// plant and on the pump, and a bound tight enough to be a tuning opinion would refuse
// legitimate values. What they catch is a slipped decimal point, which is the mistake
// that actually happens over a link.
//
// The setpoint is bounded by the calibration span, because "600 % soil moisture" is not a
// fact about soil; the gains are bounded below at 0, because a negative kp is not a gain
// but positive feedback (a drier reading would command less water), and the sign error is
// invisible in the code and obvious only in a flooded pot.
constexpr int32_t kControlMinSetpointPermille = 0;
constexpr int32_t kControlMaxSetpointPermille = 1000;
constexpr int32_t kControlMaxGainMilli = 100'000;

// The integral clamp — the failure it prevents, and the one sentence of the decision
// record this implementation had to interpret rather than transcribe.
//
// The failure: with a duty cap and slow percolation, the integrator winds up during the
// dead time. The maintainer's worked example has it accumulating ~225 % of the duty the
// plant actually needs over five minutes, and the surplus is then delivered after the
// reading reaches the setpoint, which floods the pot.
//
// The decision: "clamp the integral to ±max_duty_permille". Read with
// `integral_permille_seconds` as the thing being clamped — the state's own unit — the
// limit is ±500 permille·s. At the default gains that state produces at most
// ki_milli / 1000 x 500 = 25 duty-permille of output, and the consequences are not
// subtle:
//
//   - 25 duty-permille is half of kControlMinOnTimeMs, so the integral alone can never
//     command an on-time the actuator is willing to deliver. The static_assert below is
//     exactly that comparison.
//   - No steady state with error 0 is reachable at all. With error 0 the output is the
//     integral, which cannot exceed 25 < 50, so the window is skipped, so the soil dries,
//     so the error is positive again: the loop settles at least
//     (kControlMinOnTimeMs - ki_milli x clamp / 1000) x 1000 / kp_milli = 12.5 permille
//     below the setpoint, and further below for any plant whose demand exceeds 25.
//   - It contradicts the worked example above, which needs an integral state of 2900
//     permille·s after 20 s of a 145 permille error; a 500 permille·s clamp truncates it
//     after 3.4 s.
//
// So the clamp is applied to the integral's *contribution*, which is the reading under
// which the decided number — max_duty_permille, the actuator's own authority — bounds the
// integrator's authority, and under which "~225 % of demand" is a statement about duty.
// In the state's unit the limit is kControlIntegralClampDutyPermille x 1000 / ki_milli;
// at the defaults that is 10,000 permille·s, and at ki = 50 the loop can hold the whole
// 0..500 permille duty range with zero steady-state error. See
// control_integral_state_limit() for the derivation and its ki = 0 case.
//
// This is the one place the implementation chose a reading rather than a value, and it is
// recorded here because a one-line change would restore the literal one.
constexpr int32_t kControlIntegralClampDutyPermille = kControlMaxDutyPermille;

// The integral state's bound when ki_milli is 0, where the I term is identically zero and
// the limit above is undefined (the derivation divides by ki). It is not an anti-windup
// limit — with ki = 0 the state cannot affect the output at all — but the state keeps
// accumulating because the update is unconditional, and an unbounded state would
// eventually wrap; this is the point at which it stops. 1,000,000 permille·s is 11 days of
// a 1 permille error or 17 minutes of a 1000 permille one, and it fits an int32 with room
// for the multiplication below to be done in 64 bits.
constexpr int32_t kControlIntegralStateGuardPermilleSeconds = 1'000'000;

// The integral state limit that makes the integral's contribution at most
// `max_duty_permille`: solve `ki_milli * state / 1000 = max_duty_permille` for the state.
// Evaluated per update from the live tuning, so raising the cap at runtime raises the
// integral's authority with it, and lowering ki re-clamps the state on the next
// evaluation rather than leaving a stale, larger one in place.
constexpr int32_t control_integral_state_limit(int32_t max_duty_permille,
                                               int32_t ki_milli) {
  if (ki_milli <= 0) {
    return kControlIntegralStateGuardPermilleSeconds;
  }
  const int64_t limit = (static_cast<int64_t>(max_duty_permille) * 1000) / ki_milli;
  // A limit below 1 would let the integral hold nothing at all; 1 is the smallest state
  // that means anything. Validation keeps max_duty_permille positive, so this is a
  // floor for defensive use rather than a reachable configuration.
  return limit < 1 ? 1 : static_cast<int32_t>(limit);
}

// The check that decided the reading above: the integral's authority has to reach at
// least the smallest on-time the actuator will deliver, or the integrator is decoration
// and the loop carries a permanent offset. With the literal reading it fails (25 < 50);
// with the implemented one it holds (500 >= 50). A future edit that lowers the cap or
// raises the minimum on-time past this stops compiling, which is the moment to re-read
// the commentary above rather than the moment to delete the assertion.
static_assert(kControlIntegralClampDutyPermille >= static_cast<int32_t>(kControlMinOnTimeMs),
              "the integral must be able to command at least a minimum on-time");

// ---------------------------------------------------------------------------
// Application thresholds
// ---------------------------------------------------------------------------
// Phase 0 has no actuator, so these only classify the reading for the display and the
// telemetry line. The Phase 1 control loop's own setpoint is kControlSetpointPermille
// above; these stay because they describe the I2C humidity reading, which the loop does
// not use.
constexpr int32_t kPlantDryMilliRh = 30'000; // below 30 %RH
constexpr int32_t kPlantWetMilliRh = 70'000; // above 70 %RH

// How many consecutive failed polls make a reading stale enough to stop showing it as a
// number. Two, not one: a single dropped transfer is normal on a shared bus and the
// arbiter already retries it, so reacting immediately would make the display flicker
// between a value and a fault for a bus that is working correctly.
constexpr uint32_t kSensorStaleAfterFailures = 2;

// ---------------------------------------------------------------------------
// Watchdog and heartbeats (FR-0.4)
// ---------------------------------------------------------------------------
// The supervisor feeds the IWDG only while every registered task is inside its
// deadline. A heartbeat deadline is a multiple of the task's period: three missed
// periods means the task is not merely late, it is gone.
constexpr uint32_t kWatchdogTimeoutMs = 1000;
constexpr uint32_t kHeartbeatDeadlinePeriods = 3;

// ---------------------------------------------------------------------------
// Bus contention stress test (FR-0.2, decision §5.3 #2)
// ---------------------------------------------------------------------------
// FR-0.2 needs a defined test rather than an informal one. The stress run drives the
// sensor and display tasks at an artificially short period so that both contend for
// the arbiter on every cycle, for long enough to cover many watchdog and wrap
// boundaries. The pass condition is zero bus-contention faults: every contended
// acquisition is resolved by the mutex, no transfer returns BusError, and the
// contention counter is non-zero (otherwise the test proved nothing).
constexpr uint32_t kStressPeriodMs = 5;
constexpr uint32_t kStressDurationMs = 60'000;

// ---------------------------------------------------------------------------
// OLED display (FR-0.5, decision §5.3 #5)
// ---------------------------------------------------------------------------
constexpr uint16_t kDisplayWidth = 128;
constexpr uint16_t kDisplayHeight = 64;
constexpr size_t kDisplayFramebufferBytes = (kDisplayWidth * kDisplayHeight) / 8;
constexpr uint8_t kDisplayPages = kDisplayHeight / 8;

// A full 128x64 frame is 1024 bytes, which at 400 kHz is ~23 ms of bus time. FR-0.5
// requires partial refresh to beat that, so the display task flushes only the damaged
// page-aligned region and the target is well under 10 ms per update.
constexpr uint32_t kDisplayFullFrameBudgetUs = 23'000;

// ---------------------------------------------------------------------------
// Instrumentation and status pins
// ---------------------------------------------------------------------------
// hal_gpio_iface_t takes an opaque application-level pin id, and these are the ids.
// The HAL owns the mapping from id to port and bit, so the application core compiles
// on the host with no device header.
//
// The trace pins exist for the measurement plan, not for the application: NFR-1.8
// measures control-loop jitter by toggling a pin at control-task entry, and NFR-1.9
// measures worst-case ISR-to-task latency by toggling one in the ISR and another at
// task entry. They cost nothing when no logic analyzer is attached.
constexpr uint32_t kPinStatusLed = 0;  // board LED; heartbeat of last resort
constexpr uint32_t kPinTraceControl = 1; // toggled at control-task entry (NFR-1.8)
constexpr uint32_t kPinTraceIsr = 2;     // toggled in the DMA-complete ISR (NFR-1.9)
constexpr uint32_t kPinTraceTask = 3;    // toggled at the woken task's entry (NFR-1.9)
constexpr uint32_t kPinCount = 4;

// ---------------------------------------------------------------------------
// Comms link
// ---------------------------------------------------------------------------
constexpr uint32_t kUartBaud = 115'200;
constexpr uint32_t kUartWriteTimeoutMs = 50;
constexpr uint32_t kUartReadTimeoutMs = 5;
constexpr size_t kCommsRxBufferBytes = 128;

// ---------------------------------------------------------------------------
// Supervisor diagnostics
// ---------------------------------------------------------------------------
// A node that resets unattended (NFR-0.7) must say why on the next boot, so the reset
// cause and boot count are reported over the comms link.
constexpr uint32_t kTelemetryPeriodMs = 1000;

} // namespace node

#endif // APP_BOARD_H
