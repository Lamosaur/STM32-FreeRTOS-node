# Measurements

Captured figures, not estimates. Section 9 of [design-spec.md](design-spec.md) defines
what each phase must produce; this file is where the numbers land. Section 9.1 is
explicit that a phase is not finished until these exist, so anything still blank below
is an open item rather than a formality.

## How to read this file

Each row states **what was measured**, **how**, and **where the number came from**.
Three provenances are possible and they are not equivalent:

| Provenance | Meaning |
|-----------|---------|
| **Host-verified** | Derived from the target-independent logic under the host test suite. The behaviour is proven; an electrical figure is not. |
| **Bench** | Measured on hardware with an instrument. Names the instrument. |
| **Predicted** | Calculated from component values or datasheet limits. Not yet evidence. |

Nothing here is a claim about hardware that has not been measured on hardware. At the
time of writing no board has been brought up, so every electrical figure on this page is
still open — see [bring-up-log.md](bring-up-log.md).

---

## Phase 0 — host-verified figures

These come from `ctest` in `build/` and require no board. They are the evidence for the
Phase 0 requirements that are statements about logic rather than about volts.

| # | Figure | Value | How | Requirement |
|---|--------|-------|-----|-------------|
| 0-H1 | I2C release clock pulses | **9** | `test_i2c_recovery`, counted from the recorded edge sequence | FR-0.3 |
| 0-H2 | STOP conditions per release | **1** | same; the sequence parks SCL low first so the count is fixed, not slave-dependent | FR-0.3 |
| 0-H3 | Reference clock edges left low after each pulse | **yes** | each pulse is raise-then-lower, so only the deliberate STOP raises SDA while SCL is high | FR-0.3 |
| 0-H4 | Concurrent bus tasks with an overlap | **0** of 4 threads x 100 transfers | `ArbiterTest.ConcurrentTasksNeverOverlapOnTheBus`, with the bus double counting simultaneous transfers | FR-0.2 |
| 0-H5 | Highest concurrent transfers observed | **1** | same test; anything above 1 means the mutex is not serialising | FR-0.2 |
| 0-H6 | Bus-contention faults under stress | **0** | same test | FR-0.2 |
| 0-H7 | Recovery retries per transfer | **1** | `ArbiterTest`; a second identical attempt cannot change a bus the recovery already reported on | FR-0.3 |
| 0-H8 | Recovery runs on a NACK | **never** | `ArbiterTest.NackIsNotABusFaultAndDoesNotTriggerRecovery` | FR-0.3 |
| 0-H9 | Tasks created | **5** | `test_node`, from the task table | FR-0.1 |
| 0-H10 | Task stack budget | **1600 words / 6400 B** | sum over the five tasks; compared against the 20 KB part | FR-0.1, NFR-1.11 |
| 0-H11 | Watchdog feeds with all tasks healthy | **1 per supervisor cycle** | `test_node` | FR-0.4 |
| 0-H12 | Watchdog feeds with one task past its deadline | **0** | `test_node.AStalledTaskStopsTheWatchdogBeingFed`; the other four keep reporting in | FR-0.4 |
| 0-H13 | Heartbeat deadline | **3 task periods** | `test_supervisor`, including the exact-millisecond boundary | FR-0.4 |
| 0-H14 | Deadline arithmetic across the 32-bit ms wrap | **correct** | `test_supervisor.HeartbeatSurvivesTheMillisecondClockWrapping` | NFR-0.7 |
| 0-H15 | Display bytes, full 128x64 frame | **1064** | `flush_full`, 8 pages x (4 command + 1 control + 128 data) | FR-0.5 |
| 0-H16 | Display bytes, 8x8 partial update | **13** | `bytes_for`, asserted equal to what `flush` actually sends | FR-0.5 |
| 0-H16b | Display bytes, 128x32 status band | **532** | same; 4 pages x (5 command + 128 data) | FR-0.5 |
| 0-H17 | Display bus path | **DMA only** | `test_node.DisplayNeverUsesTheBlockingBusPath`: the polled path is never used by the display task | FR-0.5 |
| 0-H18 | Bus bytes for an unchanged screen | **0** | `test_node`: the render is skipped when no displayed value changed | FR-0.5 |
| 0-H19 | Stale-reading detection | **2 consecutive failures** | `test_node`: the panel leaves the number and reports staleness instead of freezing | FR-0.5 |

### FR-0.5 — partial versus full refresh

The one number FR-0.5 names explicitly. Measured on the host against the real flush
code path, with `bytes_for()` cross-checked against the bytes the flush actually put on
the bus.

| Update | Pages touched | Bytes on the bus | Fraction of a full frame |
|--------|---------------|------------------|--------------------------|
| Full frame, 128x64 | 8 | 1064 | 100 % |
| Status screen, 128x32 (what the node draws) | 4 | 532 | 50 % |
| 8x8 region | 1 | 13 | 1.2 % |
| Unchanged screen | 0 | 0 | 0 % |

Counting rule: a page transaction is five bytes of command and control (one control byte,
the page-address command, two column-address commands, and the data control byte) plus one
byte per column of the damaged range, which is where the 8 + 5 = 13 comes from.

Both figures come from the shipping code path rather than from arithmetic: `bytes_for()` is
the prediction and the bus double's byte counter is what actually went out, and the test
asserts the two agree.

At 400 kHz, 532 bytes is about 12 ms of bus time against a full frame's ~23 ms, and an
8-byte update is about 0.3 ms. The status screen is the number that matters, because it is
what the node actually redraws.

---

## Phase 0 — open items (no board yet)

Every row here needs hardware. The procedure column is what makes the row actionable
rather than aspirational.

| # | Figure | Value | Instrument / method | Requirement |
|---|--------|-------|---------------------|-------------|
| 0-B1 | Recovery sequence as seen on the wire | ___ | Scope, 2 channels on SCL and SDA, bus held by a jumper, `recover` issued over the host link | FR-0.3 |
| 0-B2 | Recovery success rate over a long run | ___ | 72 h run, `recfail` counter in `stat` output | FR-0.3, NFR-0.7 |
| 0-B3 | Bus contention faults over 72 h | ___ | `stat` output, `fail` and `tmo` counters | FR-0.2 |
| 0-B4 | Unattended run duration with no manual reset | ___ h | `boots=` in the boot banner; must stay 1 | NFR-0.7 |
| 0-B5 | Display update latency, partial | ___ ms | Logic analyzer on the trace pin around the display task | FR-0.5 |
| 0-B6 | Display update latency, full frame | ___ ms | same, with `flush_full` substituted | FR-0.5 |
| 0-B7 | Task stack headroom, per task | ___ % | `uxTaskGetStackHighWaterMark()` over the host link; each must keep 25 % free | NFR-1.11 |
| 0-B8 | Peak RAM before the heap is exhausted | ___ B | `xPortGetMinimumEverFreeHeapSize()` | NFR-0.7 |

---

## Phase 1 — deterministic sensing and control

Phase 1 is where most of the measurement plan lands. Rows are listed now so the
instrument and the method are decided before the work starts, which is what keeps
"measure it" from turning into "describe it".

| # | Figure | Value | Instrument / method | Requirement |
|---|--------|-------|---------------------|-------------|
| 1-1 | Control-loop period and jitter | ___ us | GPIO edges **at the control evaluation itself** (or a DWT cycle-counter delta between the same two points), logic analyzer | NFR-1.8 |
| 1-2 | Worst-case ISR-to-task latency | ___ us | GPIO in the DMA-complete ISR and at task entry | NFR-1.9 |
| 1-3 | CPU utilisation | ___ % | `vTaskGetRunTimeStats()`, must stay under 60 % | NFR-1.10 |
| 1-4 | Per-task stack headroom | ___ % | `uxTaskGetStackHighWaterMark()` | NFR-1.11 |
| 1-5 | ADC noise floor | ___ LSB rms | Input shorted to analog ground, timer-triggered capture, standard deviation | NFR-1.12 |
| 1-6 | ADC effective resolution | ___ bits ENOB | From 1-5 | NFR-1.12 |
| 1-7 | Anti-alias corner frequency | ___ Hz | Function-generator sweep, scope amplitude per point | FR-1.2 |
| 1-8 | Step response: rise, overshoot, settling | ___ / ___ / ___ | Host plot from exported samples | FR-1.3, FR-1.5 |
| 1-9 | Priority-inversion comparison | ___ | Instrumented run, mutex versus binary semaphore | FR-1.7 |

### 1-1 in more detail, because the obvious method measures the wrong thing

NFR-1.8 names the **control-loop period**, which is 1000 ms and is *not* the app task's
period. The app task runs every 25 ms so that the actuator gate's rising and falling edges
land within 25 ms of where the time-proportioned schedule puts them; the control decision
itself is taken on every tenth acquisition block, which is one second of TIM3's hardware
trigger. A GPIO toggled at task *entry* would therefore report a 25 ms cadence with 25 ms
of jitter, and reporting that as the control-loop figure would be wrong by a factor of
forty while looking entirely reasonable on a scope.

So the edges have to be placed at the control evaluation, not at task entry. Either toggle
a spare pin immediately before and after `ControlLoop::update()`, or better, timestamp with
the DWT cycle counter at those two points and report the difference against 1000 ms — the
cycle counter gives a number without needing a second channel on the analyzer. Toggling a
pin has the advantage of being visible next to the actuator waveform on PA1, which is how
the gate timing is checked at the same time.

### Rows currently blocked

One Phase 1 row cannot be filled in yet, and the reason is in the firmware rather than the
bench:

- **1-8** needs the step-response samples exported. `StepCapture` records them and
  `host/step_response.py` can analyse them, but nothing yet owns an instance, arms it, or
  sends the samples, so the two ends of FR-1.5 are not yet joined.

(Rows 1-1 and 1-2 were blocked here while `TimingService`'s single-writer design kept the
figures on the board; `TimingReportHandover` in `app/timing.h` now hands the report to the
comms task, and the `timing` command sends it. That remains unverified on silicon, like
every other Phase 1 row.)

### 1-9 in more detail

This is the measurement that justifies the arbiter's design, so it is worth stating what
makes it a comparison rather than an observation. Run the same contended-load scenario
twice: once with the arbiter's `xSemaphoreCreateMutex` (priority inheriting, the shipped
design) and once with a binary semaphore of the same place. Record the worst-case time
the high-priority task spends waiting. The mutex figure should be bounded by the
low-priority holder's critical section; without inheritance it should stretch to the
holder's whole run. `rtos_iface_t::mutex_holder_priority` exists so the inheriting case
can be logged from inside the run.

---

## Phase 2 — bootloader and field update

| # | Figure | Value | Instrument / method | Requirement |
|---|--------|-------|---------------------|-------------|
| 2-1 | Full update time | ___ s | Timed host update at 115200 baud, must be under 30 s | NFR-2.7 |
| 2-2 | Image size | ___ B | `arm-none-eabi-size` on the shipped image | NFR-2.7 |
| 2-3 | Boots to a valid image after a power cut mid-update | ___ of ___ | Repeated power-cut test | FR-2.5 |

---

## Phase 3 — custom board

| # | Figure | Value | Instrument / method | Requirement |
|---|--------|-------|---------------------|-------------|
| 3-1 | Rail voltage, 3.3 V digital | ___ V | DMM, no load and loaded | FR-3.9 |
| 3-2 | Rail voltage, VDDA | ___ V | DMM | FR-3.9 |
| 3-3 | Rail ripple, digital | ___ mVpp | Scope, AC coupled, 20 MHz bandwidth limit | FR-3.9 |
| 3-4 | Rail ripple, VDDA | ___ mVpp | Scope, as above | FR-3.9 |
| 3-5 | Crystal frequency and error | ___ Hz, ___ ppm | Scope or frequency counter on MCO | FR-3.4 |
| 3-6 | RS-485 DE/RE turnaround contention | ___ | Scope on DE and A/B | FR-3.1 |
| 3-7 | Modbus poll interval and round-trip latency | ___ ms / ___ ms | `pymodbus` host client | FR-3.2 |
| 3-8 | Phases 1 figures re-taken on the board | see Phase 1 | as Phase 1 | FR-3.17 |

---

## Phase 4 — low power (stretch)

| # | Figure | Value | Instrument / method | Requirement |
|---|--------|-------|---------------------|-------------|
| 4-1 | Run current | ___ mA | Shunt or INA219 across the FR-3.7 jumper, no debugger attached | FR-4.2 |
| 4-2 | STOP current | ___ uA | as above | FR-4.2 |
| 4-3 | STANDBY current | ___ uA | as above | FR-4.2 |
| 4-4 | Wake source and latency | ___ | Debugger plus current trace | FR-4.1 |
| 4-5 | Duty-cycled runtime from a stated capacity | ___ | Measured average current x capacity | NFR-4.3 |

---

## Recording rules

- One figure per row, with its instrument. A number without a method is not a
  measurement.
- Record the *worst* case, not the typical one. NFR-1.8, NFR-1.9 and NFR-1.11 are all
  worst-case requirements, and a mean would quietly satisfy none of them.
- Where a figure differs between the dev board and the custom board, keep both and say
  what explains the difference. FR-3.17 exists precisely because the Phase 1 numbers
  describe the dev board's supply and layout, not this design's.
