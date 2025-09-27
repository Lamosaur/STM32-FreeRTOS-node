# Design Specification and Requirements Plan

**Project:** STM32 FreeRTOS Sensor & Control Node
**Revision:** 0.7 (draft)
**Status:** Phases 0 and 1 implemented and host-verified; Phases 2-3 planned; Phase 4
deferred

**Revision 0.7 change:** the document now describes the hardware and the mechanism the
firmware actually uses. Revision 0.6 got the sensor set wrong and named a tuning-publication
scheme that did not survive its own test:

- The I2C sensor is an **AHT20 + BMP280** module — AHT20 at 0x38, BMP280 at 0x76 and
  deliberately unread — not the SHT31 at 0x44 this document assumed. The CRC-8 is the
  SHT3x's algorithm, which is why that code survived the change; the frame is not, and a
  driver keeping the old framing would have read zero humidity rather than failing loudly.
- The controlled variable is a **capacitive** soil-moisture probe v2.0, 3.3 V, and it stays
  that in the final design. The analog signal path is unchanged by this; the note about a
  resistive probe's transfer function was about the wrong sensor.
- Setpoint and gains are published by a **seqlock whose reader never spins**, not by the
  "atomic swap" revision 0.6 names. The two-slot index swap it described was not atomic and
  delivered torn tunings under a burst of `set` commands; Section 6.1 decision 9 records the
  measurement that replaced it.
- Phase 1's ADC/DMA HAL and actuator output are **written and linked into the image**, so
  the paragraph below stating that they are not is corrected. Of the two unfinished Phase 1
  pieces this note named, one — the timing service's single-writer restriction — is now
  closed by `TimingReportHandover`; the step-response export path remains open and is the
  only one left.

**Revision 0.6 change:** Phase 1's decisions are settled and recorded in Section 6.1, and
three statements in Section 6 that those decisions contradict have been corrected rather
than left standing:

- The control period is **1 s**, chosen from the plant's time constant. The section's
  earlier "not sufficient at 1 kHz" described a fast loop, and is now marked as such —
  measuring the jitter of the 1 s loop satisfies NFR-1.8, which names no rate.
- The output is **time-proportioned actuator switching**, not fast PWM. It still satisfies
  FR-1.3, and Section 6.1 says why.
- Setpoint and gains are published by **snapshot with an atomic swap, not the mutex this
  section originally specified**. A mutex there would put a lock inside the loop whose
  jitter NFR-1.8 measures, and would itself be the priority-inversion case FR-1.7 exists
  to measure.

**Revision 0.5 change:** Phase 0 has been implemented. Its task architecture, I2C
arbiter and release sequence, supervisor, sensor service, display path and host test
suite now exist, and Section 5.3's open decisions are settled and recorded there. The
status statements throughout this document have been corrected accordingly: no longer
"No code written", but also not "verified on hardware" — see the note below.

**What has not happened.** No firmware has run on a board. A Blue Pill is attached over
SWD, its flash has been measured at 128 KB and a verified image sits in it, but the board
will not execute it: BOOT0 is not strapped low, so each reset lands in the ST factory
bootloader (`docs/bring-up-log.md`). Every figure in Section 9 is still open except the
ones marked host-verified in [measurements.md](measurements.md). The evidence for Phases 0
and 1 is evidence about logic verified on the host (FR-0.6), not about volts.

Within Phase 1, the acquisition service, the control loop, the tuning store, the
step-response recorder, the ADC/DMA HAL and the actuator output all exist, are host-tested,
and are linked into the target image, and the timing service's report now reaches the host
through `TimingReportHandover` (`app/timing.h`, with the `timing` command). One piece is
deliberately incomplete and is named here rather than implied: the step-response recorder
has no instance in the node and no export path to the host, so FR-1.5 is not satisfied end
to end. None of it has run on silicon.

**Revision 0.4 change:** the record now states plainly that no implementation exists
yet. Phase 0 is a design to be built, not an as-built description, and the design
decisions it depends on are listed as decisions rather than as claims about existing
code.

**Revision 0.3 change:** the case for a custom board over the dev board, including
the conditions under which staying on the dev board is correct, is recorded in
Section 2.3.

**Revision 0.2 change:** the custom MCU board is reinstated and merged with the
Phase 3 I/O board into a single board design. Rationale in Section 2.2.

---

## 1. Purpose

This document is the single source of truth for what this project is, what it must
do, how each requirement is verified, and in what order the work happens. It specifies
the planned phases in full, Phase 0 included. **Phase 0 is implemented** — the
application layer, the HAL interfaces, the host test suite and the CMake build exist —
but nothing in it has run on hardware yet, so every requirement below is still a target
whose verification is outstanding unless stated otherwise.

Three goals shape every decision below:

1. **Build an embedded system, not a data logger.** Sensing, control, actuation,
   field update, and fieldbus I/O in one coherent artefact.
2. **Produce measured evidence.** Every phase ends with numbers, not features.
3. **Own a complete board.** The board is designed, laid out, fabricated, and brought
   up within this project, so the hardware is a deliverable rather than third-party
   hardware taken as given.

## 2. Scope

### 2.1 In scope

| Phase | Scope | Effort | Cost | Why it is in |
|-------|-------|--------|------|--------------|
| 0 | FreeRTOS node: I2C arbitration, bus recovery, watchdog, DMA display | TBD | ~$15 | Everything later is built on this task and bus architecture, and it is the phase that settles the HAL/application split. Not yet estimated; it is now build work, not documentation. |
| 1 | Timer-triggered ADC + DMA, PID control loop, scheduling measurement, CI | 12-18 h | ~$10 | Highest value per hour. Converts the project from logging to control, and adds the timing and scheduling evidence that makes the system's behaviour defensible rather than assumed. Runs on the dev board, so it has no dependency on the new hardware. |
| 2 | Bootloader + field firmware update | 20-28 h | $0 | The largest single capability gap, and both halves of the framed protocol — host uploader and target bootloader — are written in this phase. Runs on the dev board. |
| 3 | RS-485 / Modbus RTU + custom MCU & I/O board, fabricated and brought up | 45-65 h | ~$120-190 | The full hardware artefact set — schematic, layout, BOM, fabrication, bring-up log — plus the fieldbus base. The board is the only part of the project that exercises layout, power, and analog design. |
| 4 | Low-power modes with measured current draw, on the custom board | ~10 h | included | Stretch only. Depends on the Phase 3 board, which carries the current-measurement jumper precisely to enable this. |

### 2.2 Scope decisions

**The custom MCU board is in scope.** Two reasons:

1. It is a platform the project owns permanently. Phases 0-2 will be developed on the
   dev board for speed, then ported, so the board becomes part of the system rather
   than a side exercise.
2. It is the only element that exercises the board-level work at all: layout, power
   design, the analog supply and signal path, protection, and bring-up. Firmware on a
   dev board cannot produce any of that, because the vendor has already done it.

The full argument — what the dev board cannot do, and the conditions under which
staying on it is the correct call — is in Section 2.3.

**Merged, not added: one board, not two.** The board absorbs what was going to be a
separate I/O carrier board — MCU, analog front end, RS-485 transceiver, actuator
driver, power, and connectors on one design. Two boards would mean two fabrications,
two bring-ups, and two sets of layout mistakes for no additional evidence. This is
the difference between reinstating the board at +30 h and reinstating it at +70 h.

**Sequencing protects against a blocking dependency.** The bootloader, Modbus
protocol, and control loop are all to be proven on the known-good dev board *before* the
board exists. A board bug therefore cannot block firmware progress, and bring-up
cannot be confused with a firmware defect.

| Cut | Rationale |
|-----|-----------|
| **Separate I/O carrier board** | Merged into the MCU board. Building both doubles fabrication and bring-up cost for a subset of the same evidence. |
| **CAN bus** | Requires a transceiver *and* a second node to be meaningful. RS-485/Modbus covers the industrial fieldbus base at lower cost and works with a single node. Revisit only if multi-node or automotive requirements appear. |
| **LittleFS / FatFs persistence** | Real but low marginal value here: wear levelling and power-fail atomicity matter less than the bootloader and control-loop work they would compete with for time. Phase 2 keeps a small version via update-metadata storage. |
| **Standalone priority-inversion demo** | Folded into FR-1.7 as a measurement on the Phase 0 I2C arbiter rather than built as a separate feature. |
| **RTOS replacement (Zephyr, bare-metal rewrite)** | Rewriting working firmware produces no new capability and delays every phase. |
| **BGA / QFN packages, USB, high-speed interfaces** | Kept off the board so it stays two-layer and hand-solderable. Assembly difficulty is not what the board is for. |

### 2.3 Board rationale — custom board versus dev board

The board is the most expensive decision in this plan, so the reasoning is recorded
here rather than left implicit: why not stay on the dev board, which already works.

#### 2.3.1 Three phases the dev board cannot serve

1. **Phase 4 is not possible on it.** The dev board's AMS1117-class regulator draws
   milliamps quiescently, the USB bridge and power LED draw more, and STM32F103 STOP
   mode draws roughly 14-20 µA. The board's own overhead is therefore hundreds of
   times larger than the MCU current being measured. Measuring it would mean
   desoldering the regulator, removing the LED, and cutting a trace — destructively,
   and differently on each clone board. On the custom board it is a jumper (FR-3.7).
2. **Phase 1's ADC results would describe the dev board, not this design.** On most
   F103 clones VDDA is tied to VDD with little or no filtering, so a measured noise
   floor and ENOB characterise the board's supply rail rather than the signal chain.
   Bolting external filtering onto a breadboard recovers part of this, but that is
   most of the analog supply work the board exists to do properly.
3. **Phase 3 has nowhere to live.** No RS-485 transceiver, no gate driver, no input
   protection, no filtered analog rail, no test points. All of it would be
   breadboard-only, which the next subsection rules out.

#### 2.3.2 A breadboard is not a substitute

Contact resistance, stray capacitance, and a shared return impedance are exactly what
a microvolt-level analog front end is sensitive to. An anti-alias roll-off measured on
a breadboard is a property of the wiring, not of the filter. A ground plane and a
separated analog return make the measurement a property of this design, which is what
makes the number defensible when it is questioned.

#### 2.3.3 What only the board produces

| Dev board | Custom board |
|-----------|--------------|
| Hardware decisions are fixed by the vendor | Chosen here, with consequences that can be measured |
| A regulator that was not selected for this design | Input protection, regulation, and filtered VDDA, designed and justified |
| Sleep current unmeasurable without rework | A jumper (FR-3.7) |
| ADC noise floor belongs to the board | Belongs to this design, against a filtered reference |
| Artefacts: firmware only | Schematic, layout, BOM, board photograph, bring-up log, measurements |
| Failure modes already eliminated by the vendor | Cold joints, wrong footprint, missing decoupling, crystal failing to start, brownout |

Bring-up debugging on self-designed hardware is the part no dev board teaches,
because a dev board has already had those failure modes removed. A first board with
documented mistakes and a "what I would change" section is worth more than a
flawless-looking one, because it is the record of the loop actually being closed.

#### 2.3.4 Counter-case: when staying on the dev board is correct

The board is roughly the last 20% of the project's functional value for 40% or more
of its effort. Phases 0, 1, 2, and 3a on the dev board carry the control, scheduling,
fieldbus, bootloader, and update work — most of the firmware substance. The board's
return is that it makes the hardware half of the design real, not that it adds
firmware capability. Staying on the dev board is the right call when any of these hold:

- **Firmware is the priority.** The board produces nothing usable before the
  fabrication lead time is even over, so nothing firmware-side improves by starting it.
- **Another mixed-signal board is already in flight.** Two board designs at once is how
  both stall; the one already committed should finish first.
- **The goal is primarily firmware, RTOS, and control work.** The dev board is strictly
  better per hour for that, and the board teaches nothing about scheduling.
- **Time is short.** Phase 1 and Phase 2 finished beats Phase 3 half-designed; an
  abandoned board is worth less than a finished small project.

#### 2.3.5 Decision rule

Keep the board in the plan, place it last, and gate it.

- Phases 0, 1, 2, and 3a are to be completed on the dev board first.
- Step 3b does not start until those phases are complete *and* there is capacity to give
  layout undivided attention.
- Board work starting later than planned is not a failure state. The board is what gets
  built when there is time to build it properly.

The board is justified because it is the only element that makes the hardware half of
the design real, and because Phase 4 depends on it. It is not urgent, because nothing
firmware-side is blocked by its absence.

## 3. System architecture

Nothing in this section is implemented. Section 3.1 is the Phase 0 design; Section 3.2
is what the system becomes once Phases 1-3 are done.

### 3.1 Phase 0 design (dev board)

- A single FreeRTOS application on an STM32F103C8T6 with 5 concurrent tasks.
- All I2C peripherals share one bus behind a **mutex-protected arbiter**; every task
  acquires the mutex for the duration of a transfer.
- **Bus recovery**: on a detected fault, the bus is released with 9 clock pulses and
  the peripheral is re-initialised, instead of requiring a system reset.
- An **independent supervisor task** feeds a watchdog and tracks task heartbeats.
- The display path uses **DMA with dirty-rectangle refresh** rather than blocking
  full-frame writes.
- The codebase is split into a **C HAL** and a **C++ application layer** as separate
  CMake targets behind a service interface, so target-independent logic compiles and
  runs on the host.

Requirements for this phase and how each is verified are in Section 4.1; the design
is in Section 5.

### 3.2 Target design (after Phase 3)

The same firmware running on a custom board that carries the MCU, an analog
acquisition path (AFE → ADC, timer-triggered, DMA), a fixed-period PID control task
driving PWM, a Modbus RTU slave on RS-485, and the power subsystem — all on one
two-layer design.

## 4. Requirements

Convention: **MUST** is required for the phase to be complete; **SHOULD** is expected
but may be traded against schedule; **MAY** is optional.

### 4.1 Phase 0 — FreeRTOS node

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-0.1 | The system MUST run no fewer than 5 concurrent tasks under FreeRTOS. | Task count and priorities in code review |
| FR-0.2 | All tasks sharing the I2C bus MUST arbitrate access through a mutex-protected arbiter, with no bus-contention faults under stress. | Stress run with concurrent sensor and display traffic |
| FR-0.3 | The system MUST recover from a stuck I2C bus (9-clock release + peripheral re-init) without a power cycle. | Deliberately held bus, recovery observed on scope |
| FR-0.4 | An independent task MUST feed a watchdog and detect missed task heartbeats. | Watchdog trip test from a stalled task |
| FR-0.5 | Display updates MUST use DMA and partial refresh rather than blocking full-frame writes. | Timing comparison of partial vs. full-frame refresh |
| FR-0.6 | Target-independent logic MUST build and run on the host for testing. | Host test run in CI |
| NFR-0.7 | The system MUST run unattended for 72 h with zero manual resets. | Timed unattended run with a reset counter |

Section 5.3 lists the design decisions that must be settled before this phase is
implemented.

### 4.2 Phase 1 — deterministic sensing and control

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-1.1 | ADC conversion MUST be triggered by a hardware timer at a fixed rate and transferred by DMA into a double buffer, with no CPU polling of the conversion-complete flag. | Code review + logic analyzer on the timer/ADC trigger |
| FR-1.2 | An RC anti-alias filter MUST precede the ADC input, with its corner placed below Nyquist for the chosen sample rate. | Component values + swept measurement |
| FR-1.3 | A control task MUST run PID at a fixed period and drive a PWM actuator output. | Logic analyzer on PWM; step-response capture |
| FR-1.4 | Setpoint and PID gains SHOULD be adjustable at runtime without reflashing. | Host tool sets a value; closed-loop response changes |
| FR-1.5 | The system MUST export step-response data for plotting off-target. | Host plot generated from captured data |
| FR-1.6 | Host unit tests MUST run in CI on every push, with a static analysis step. | Green CI run in the repository |
| FR-1.7 | The I2C arbiter mutex MUST be shown to use priority inheritance, with the priority-inversion case measured for comparison. | Instrumented run comparing mutex vs. binary semaphore |
| NFR-1.8 | Control-loop period jitter MUST be measured and reported, not assumed. | GPIO toggle in the control task, logic analyzer |
| NFR-1.9 | Worst-case ISR-to-task latency MUST be measured and reported. | GPIO toggle in ISR and at task entry, logic analyzer |
| NFR-1.10 | CPU utilisation MUST be measured and kept below 60% in normal operation. | `vTaskGetRunTimeStats()` output |
| NFR-1.11 | Every task MUST retain at least 25% stack headroom, with sizes right-sized from measurement. | `uxTaskGetStackHighWaterMark()` output |
| NFR-1.12 | ADC noise floor and effective resolution MUST be measured and reported. | Short-input capture, standard deviation in LSB, converted to ENOB |

### 4.3 Phase 2 — bootloader and field firmware update

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-2.1 | A bootloader MUST reside in a protected flash region and be reachable on every boot. | Flash layout map + boot-time trace |
| FR-2.2 | An application image MUST be CRC-validated before execution; a failed image MUST leave the node in the bootloader rather than jumping to invalid code. | Corrupted-image test |
| FR-2.3 | The application vector table MUST be relocated correctly (`SCB->VTOR`) and interrupts MUST work after the jump. | Interrupt-driven peripherals verified post-update |
| FR-2.4 | The host uploader MUST use one framed UART protocol (length-prefixed, CRC-16, ACK/retry) shared with the target, rather than an ad-hoc format per side. | Host tool source + successful update |
| FR-2.5 | An update interrupted by power loss MUST leave a bootable image on the node. | Power-cut mid-update, repeated |
| FR-2.6 | The host SHOULD read firmware version and build metadata from the node. | Version query over UART |
| NFR-2.7 | A full update MUST complete in under 30 s at 115200 baud. | Timed update |
| NFR-2.8 | There MUST be a documented SWD recovery path for a node that fails to boot. | Written procedure, executed once |
| NFR-2.9 | The flash layout MUST be parameterised so it can be re-targeted to the Phase 3 MCU without redesign. | Layout constants in one header; re-target documented |

### 4.4 Phase 3 — custom MCU and I/O board

**Protocol, to be proven on the dev board first:**

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-3.1 | An RS-485 half-duplex transceiver MUST be driven with correct DE/RE direction timing, with no bus contention during turnaround. | Scope on DE and A/B lines |
| FR-3.2 | The node MUST act as a Modbus RTU slave exposing sensor values and control setpoints as holding registers. | Polled from `pymodbus` on the host |

**Board — MCU core:**

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-3.3 | The board MUST carry the MCU in a hand-solderable leaded package (LQFP-48 class; no BGA, no fine-pitch QFN). | BOM + layout |
| FR-3.4 | The board MUST include an external crystal with load capacitors matched to the specified value, an NRST reset circuit, and BOOT0 strapping. | Schematic + measured crystal frequency |
| FR-3.5 | The board MUST expose an SWD header for programming and recovery. | Programming over SWD succeeds (NFR-2.8) |
| FR-3.6 | Every supply rail MUST carry a test point and a ground reference. | Schematic + continuity check |
| FR-3.7 | The MCU rail MUST include a removable current-measurement jumper so Phase 4 can measure current without reworking the board. | Schematic + Phase 4 measurement |

**Board — power:**

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-3.8 | The input MUST have reverse-polarity and transient protection. | Schematic review |
| FR-3.9 | The board MUST regulate to 3.3 V with a dedicated filtered analog supply (VDDA) separated from the digital rail. | Rail measurements + ripple check on scope |

**Board — analog and I/O:**

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-3.10 | The board MUST host the analog front end: input protection, op-amp buffer, and anti-alias filter referenced to VDDA. | Schematic + swept measurement (FR-1.2) |
| FR-3.11 | The board MUST host an RS-485 transceiver with bus termination and TVS protection. | Scope on DE and A/B lines |
| FR-3.12 | The board MUST host an actuator gate driver with flyback protection. | PWM drive verified under load |
| FR-3.13 | All external connectors MUST have overvoltage/transient protection. | Schematic review |

**Board — documentation and process:**

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-3.14 | The board MUST be routable on 2 layers within a stated outline size. | Completed layout on 2 layers |
| FR-3.15 | Layout decisions affecting signal integrity and EMC MUST be documented with reasoning: ground strategy, return paths, analog/digital separation, crystal placement, TVS placement, termination. | Board documentation section |
| FR-3.16 | A bring-up log MUST be written during bring-up, in order, not reconstructed afterward. | `docs/bring-up-log.md` |
| FR-3.17 | The firmware MUST be ported to the board and the Phase 1 measurements re-taken on the new hardware. | `docs/measurements.md` updated with board figures |

### 4.5 Phase 4 — low power (stretch)

| ID | Requirement | Verify by |
|----|-------------|-----------|
| FR-4.1 | The node MUST enter STOP and STANDBY modes, waking on RTC and external interrupt. | Mode entry confirmed on debugger + current measurement |
| FR-4.2 | Current draw MUST be measured in run, STOP, and STANDBY using the FR-3.7 jumper, and a power budget documented. | Shunt or INA219 on the jumper |
| NFR-4.3 | Duty-cycled operation SHOULD be shown to extend a defined battery capacity to a stated runtime. | Measured average current × capacity |

## 5. Phase 0 design — FreeRTOS node

The design below is implemented in `firmware/` and exercised by the host test suite in
`tests/`. Section 5.3 records the decisions that were settled to build it.

### 5.1 Task and resource architecture

| Task | Priority | Period | Responsibility | Shares |
|------|----------|--------|----------------|--------|
| Supervisor | 4 | 250 ms | Feeds the watchdog, tracks task heartbeats | — |
| Sensor | 3 | 1000 ms | Reads the AHT20 over I2C; releases the bus during the conversion | I2C (mutex) |
| Application / control | 3 | 100 ms | Owns application logic and state; target-independent. Becomes the Phase 1 PID loop | — |
| Comms | 2 | 50 ms | Host link for telemetry and tooling | UART |
| Display | 1 | 200 ms | Refreshes the OLED over I2C using DMA + dirty rectangles | I2C (mutex) |

The five tasks are created by `Node::start()`, and the count and priorities are asserted
by `tests/test_node.cpp` rather than left to review, so FR-0.1 fails the build if it
regresses.

The IMU named here in revision 0.4 is superseded by decision 9 in Section 5.3: for a
greenhouse node the I2C sensor is a temperature/humidity device, and the one on the bench
is an AHT20. The controlled quantity is not on this bus at all — it is the capacitive
probe, read through the Phase 1 ADC path.

The I2C mutex is the single serialisation point for the bus, which is why the
priority-inheritance property in FR-1.7 matters: the arbiter sits on the critical path
of every sensor and display cycle.

### 5.2 Module boundaries

- `hal/` — C, one module per peripheral (I2C, SPI, UART, DMA, GPIO, timers), no
  application knowledge.
- `rtos/` — an addition to the layout as originally planned. A narrow interface over the
  RTOS primitives the application uses (mutex, clock, task, stack watermark), with a
  FreeRTOS implementation for the target and a host double for the tests. Without it the
  application layer could not compile on the host at all, because a mutex is exactly the
  sort of thing FR-0.2 and FR-0.6 both need and a board provides.
- `app/` — C++, services behind interfaces, target-independent where possible.
- `tests/` — host-side tests against the target-independent logic, run in CI.
- Build: CMake, with the HAL and application layer as separate targets.

Both `hal/` and `rtos/` are *interfaces* — a struct of function pointers plus an opaque
context — with the implementation chosen at link time. That is the mechanism that makes
FR-0.6 possible, and it is what makes the Phase 3 port tractable: peripheral- and
RTOS-dependent code is confined to `hal/stm32f1/` and `rtos/freertos/`, and the
application core never includes a device or FreeRTOS header.

It also has a cost, recorded here rather than discovered later: an indirect call per
peripheral operation, and the discipline that a module which needs a new primitive must
extend the interface rather than reach past it.

### 5.3 Decisions settled for Phase 0

These were the open decisions blocking implementation. Each is now settled, and the
authority for each is a header or a test rather than this table, so the reasoning lives
next to the code that depends on it.

| # | Decision | Settled as | Where it lives |
|---|----------|-----------|----------------|
| 1 | Task count and priorities | Five tasks: supervisor (4), sensor (3), app (3), comms (2), display (1). Display is lowest on purpose — it is the long-running holder of the I2C mutex, which is the priority-inversion case FR-1.7 measures. | `app/board.h` (`task_priority`), `app/node.h` |
| 2 | Bus contention stress test | Four concurrent tasks x 100 transfers with the bus double counting simultaneous transfers; the pass condition is zero contention faults *and* a non-zero contention count, so the test cannot pass vacuously. | `tests/test_i2c_arbiter.cpp` |
| 3 | I2C recovery implementation | Nine SCL pulses with SDA released, then one STOP, then peripheral re-init. The sequence parks SCL low before releasing SDA so the emitted edges do not depend on the slave's prior state and the capture contains exactly one deliberate STOP. | `app/src/i2c_recovery.cpp`, `tests/test_i2c_recovery.cpp` |
| 4 | Watchdog feeder | A task, not a timer callback. `Supervisor::run()` is the only entry point and it withholds the feed when any watched task is past its deadline or when health cannot be read. | `app/src/supervisor.cpp` |
| 5 | Refresh strategy | Dirty-rectangle partial refresh over the arbiter's DMA path. A bounding box rather than a tile map: O(1) per draw with no allocation, on a 20 KB part. An unchanged screen costs zero bus bytes. | `app/include/app/display.h`, `app/src/node.cpp` |
| 6 | Host-runnable unit tests | GoogleTest via `FetchContent`, `ctest` driver, ASan and UBSan on by default. The application core compiles against `hal/` and `rtos/` interfaces only, so the same `Node::tick()` runs on the host and on target. | `tests/CMakeLists.txt`, `CMakeLists.txt` |
| 7 | 72 h unattended run | `hal_system_iface_t::boot_count()` in a backup register is the reset counter; the run is clean if it stays at 1. Runs on hardware — not yet started. | `hal/include/hal/system.h`, `docs/measurements.md` 0-B3/0-B4 |
| 8 | Sensor set | The final design's set, and the one on the bench: a **capacitive soil-moisture probe v2.0** (analog, 3.3 V) as the controlled variable on the Phase 1 ADC path; an **AHT20** temperature/humidity sensor at 0x38 as the monitored ambient device; a **BMP280** at 0x76 that answers the bus scan and is deliberately not read; and the **SSD1306** at 0x3C for display. The AHT20 plus the SSD1306 is two devices and two tasks on one bus, which is what makes FR-0.2 a real requirement rather than a formality. | `app/board.h` |
| 9 | Node purpose | A **greenhouse** node. Soil moisture is the primary quantity and is analog, so it belongs on the Phase 1 timer-triggered ADC path; Phase 0 reads temperature and humidity over I2C. | `app/board.h`, `README.md` |

**Superseded by decision 9:** the Phase 0 table in Section 5.1 lists an IMU as the I2C
sensor. For a botanical node an IMU is the wrong primary sensor, and the greenhouse
reading is better served by temperature and humidity. The sensor service is written
against the AHT20 but the arbiter and supervisor are sensor-agnostic, so adding an IMU
later is a new driver rather than a redesign.

**Superseded by the bench module (revision 0.7):** this table was written against an SHT31
at 0x44, which the attached module turned out not to be. It is an **AHT20 + BMP280**
combination board, so the driver was replaced with an AHT20 driver. What survived the
substitution and what did not is worth stating, because the distinction is the whole
lesson: the CRC-8 is the SHT3x's algorithm — polynomial 0x31, init 0xFF, no reflection, no
final XOR — so that code was unchanged, while the *frame* is different in a way that fails
silently. The SHT3x returns six bytes carrying two 16-bit words, each with its own CRC; the
AHT20 returns seven bytes carrying one 20-bit word pair with a single CRC over the first
six. A driver that kept the old framing would have read zero humidity and reported a CRC
mismatch rather than a wrong-looking number, which is why `tests/test_sensor.cpp` now pins
the AHT20 framing and `docs/bring-up-log.md` records how the substitution was found.

**Still open, and not blocking Phase 0:** the Section 12 questions on the Phase 3 MCU
choice, the toolchain path, and the board outline size.

## 6. Phase 1 design — deterministic sensing and control

**Signal path:** capacitive soil-moisture probe v2.0 → op-amp buffer → RC anti-alias filter
→ ADC (timer-triggered, DMA circular half/full-complete interrupts) → double buffer →
acquisition service → control task (PID, fixed period) → time-proportioned actuator
switching → gate driver → pump.

**Key design points:**

- Half/full-transfer-complete callbacks only signal a task; no processing in interrupt
  context. This keeps ISR latency (NFR-1.9) defensible.
- Control period is set by a hardware timer, not `vTaskDelay`, so period accuracy does
  not depend on tick granularity.
- Setpoint and gains are published through a **seqlock whose reader never spins**,
  written by the comms task and read by the control task, so tuning does not require a
  reflash (FR-1.4). Section 6.1 decision 9 records why neither the mutex this section
  originally specified nor the two-slot index swap that replaced it was correct.

**Deliverables:** firmware, host step-response plotter, measured scheduling numbers,
CI workflow, `docs/measurements.md` with captured figures.

### 6.1 Decisions settled for Phase 1

Recorded the same way as Section 5.3: the authority for each is a header or a test rather
than this table, so the reasoning lives next to the code that depends on it.

#### Acquisition (FR-1.1, FR-1.2)

| Decision | Settled as | Where it lives |
|----------|-----------|----------------|
| Sample rate | 1 kHz, timer-triggered, DMA into a double buffer. Chosen for oversampling rather than bandwidth: the probe's information is below a few Hz, so the samples exist to be averaged. | `app/board.h`, `hal/include/hal/adc.h` |
| Averaging | 100 samples per published value → 10 Hz | `app/board.h`, `app/src/acquisition.cpp` |
| Anti-alias corner | R = 10 kΩ, C = 150 nF → 106 Hz, one decade below Nyquist | `app/board.h` |
| Why that corner is not anti-aliasing theatre | A single pole rolls off at −20 dB/decade, so pushing an alias below one LSB would need a corner thousands of times below Nyquist, which would also destroy the signal. The RC provides a defined, sweepable rolloff for what is actually present here (mains harmonics, gate-driver switching). | `app/board.h` |
| Probe calibration | Two measured points and a line between them, runtime-settable. A capacitive probe's output depends on its geometry, its supply and the soil's dielectric behaviour, so a formula would be a guess dressed as physics — and the two endpoints in `app/board.h` are still placeholders until a probe is measured in air and in water. | `app/board.h`, `app/include/app/acquisition.h` |
| No polling | Structural, not a convention: `hal_adc_iface_t` has no "conversion ready" and no "read result" entry, so polling cannot be written (FR-1.1) | `hal/include/hal/adc.h` |

#### Control (FR-1.3, FR-1.4, FR-1.5)

| # | Decision | Settled as | Where it lives |
|---|----------|-----------|----------------|
| 1 | Controlled variable | **Soil moisture**, from the analog channel and the capacitive probe v2.0. The I2C temperature/humidity sensor (AHT20) stays monitored only — it does not drive the actuator. | `app/src/control.cpp` |
| 2 | Setpoint | 450 permille (45.0 %), runtime-tunable | `app/board.h` (FR-1.4) |
| 3 | Control period | **1000 ms.** Chosen from the plant, not from the CPU: soil responds over minutes, so a 1 kHz loop would re-decide tens of thousands of times before the process moved, reacting to sensor noise rather than to information. | `app/board.h` |
| 4 | Output form | **Time-proportioned**: within each window the pump is on for `duty × window`. Actuator-agnostic (works for a DC pump or a solenoid valve, which fast PWM would buzz and wear), physically matched to a slow process, and directly visible on a logic analyzer. FR-1.3's "PWM actuator output" is satisfied by this: it is pulse-width modulation at the window rate. | `app/src/control.cpp` |
| 5 | Minimum on-time | **50 ms**; a shorter computed on-time becomes 0. Below it the pump cannot act, so ticking it would consume life without delivering water. | `app/board.h` |
| 6 | Maximum duty | **500 permille** per window — a bound while tuning, and the integral clamp reference | `app/board.h` |
| 7 | Loop type | **PID with `kd` defaulted to 0**, integral clamped. D acts on the rate of change of a slow, noisy reading, which is mostly noise: the measured consequence is quantified in the test that evidences this decision. | `app/src/control.cpp`, `tests/test_control.cpp` |
| 8 | Fail-safe | An absent or stale reading forces the output to **0**, integral held rather than unwound. New in Phase 1: "not acting" now has physical meaning, and a lost probe must stop the pump rather than hold its last duty. The watchdog covers a hung CPU, **not** a MOSFET stuck on, so the gate needs a hardware pull-down. | `hal/include/hal/actuator.h`, `app/src/control.cpp` |
| 9 | Gains publication | **A seqlock whose reader never spins**, never a mutex. A mutex here would put a lock acquisition inside the very loop whose jitter NFR-1.8 measures, and would itself be the priority-inversion case FR-1.7 exists to measure. The writer marks the slot odd, writes the fields, then marks it even; a reader that observes an odd or changed sequence **discards the value and keeps its previous one**. The reader must not retry: the control task outranks the comms task and there is one core, so a spinning reader would wait for a writer that cannot be scheduled — a livelock, not a wait. This replaced a two-slot index swap that *looked* atomic and was not: it delivered torn tunings at a measured 227 reads in 72,521, mixing fields from up to six publications apart into a plausible-looking set nobody had ever sent. | `app/include/app/control.h`, `tests/test_control.cpp` |
| 10 | 1 kHz | **Not** the control period. If that figure is wanted it is a separate, explicitly labelled scheduling test — presenting it as a control requirement would be the measurement plan flattering itself. | — |

**Integral clamp — what "clamped" means, because the wording was ambiguous.** Decision 7
says the integral is clamped to ±`max_duty_permille`. Read as a bound on the stored
integral *state*, whose unit is permille-seconds, that gives the I term at most
`ki/1000 × 500` = 25 duty-permille at the default gains — half the 50 ms minimum on-time.
The loop then cannot hold any setpoint that needs more than 25 permille of duty, so "no
steady-state error", which the same decision set requires, becomes unreachable: the
simulated minimum offset is 12.5 permille on a perfectly matched plant and 61.5 permille on
a realistic one. It also contradicts the worked example in the decision notes, which needs
a state of 2900 permille-seconds after 20 s.

So the clamp binds on the integral's **contribution** to the output, not on the state's own
unit: the state limit is `max_duty_permille × 1000 / ki_milli`, re-derived from the live
tuning on every update. The clamped quantity is then the same one the "~225 % of demand"
figure describes. The clamp *value* is unchanged; only its referent is now stated. The
arithmetic is encoded in a `static_assert` and a test asserts both readings, so restoring
the literal reading is a one-line change if that was ever the intent.

**Residual risk of the decided cap, as arithmetic rather than prose.** Bounding the
integral at the actuator's authority means that after the reading crosses the setpoint the
pump may stay on until the error reaches `−max_duty × 1000 / kp` = −250 permille. The
worst-case over-watering these numbers permit is therefore a +250 permille overshoot; the
simulated windup case gives +75. If that ceiling is too high for the pot, the knob is `kp`
or the duty cap, not the clamp.

**Verification consequence.** NFR-1.8 asks for control-loop period jitter without naming a
rate, so it is satisfied honestly by measuring the jitter of the 1 s loop this plant
needs. The 1 kHz expectation that an earlier revision of this section implied came from
the phrase "not sufficient at 1 kHz" in the key design points above, which describes a
*fast* loop (motor current, heater) rather than this one.

**Still open:** none for Phase 1. The 1 kHz scheduling figure is available on request but
is not planned.

## 7. Phase 2 design — bootloader and field update

**Flash layout:** bootloader in a protected sector, application offset above it,
update metadata (version, CRC, image size) in a dedicated sector. Layout constants
live in one header so NFR-2.9 holds.

**Update sequence:** host sends image → bootloader writes to a staging region →
CRC-16 verified → metadata committed → reset → bootloader validates and jumps with
`SCB->VTOR` relocated. A failed CRC or an interrupted transfer leaves the previous
valid metadata intact (FR-2.5).

**New work, not reuse:** neither half of the framed protocol exists yet. The host
uploader (length-prefixed, CRC-16, ACK/retry) and the target-side bootloader are both
written in this phase, against the framing defined here.

## 8. Phase 3 design — custom MCU and I/O board

### 8.1 Internal sequence

Ordered so that protocol risk is retired before layout begins, and board risk cannot
block firmware.

| Step | Work | Effort |
|------|------|--------|
| 3a | Prove Modbus RTU on the dev board with a breadboard transceiver. Retires protocol and DE/RE timing risk before it is committed to copper. | 8-10 h |
| 3b | Schematic: MCU core, power, analog front end, RS-485, actuator driver, connectors, test points, current jumper. | 8-12 h |
| 3c | Layout: 2 layers, ground strategy, analog/digital separation, crystal placement, DFM review. | 12-18 h |
| 3d | Fabricate and assemble. | ~2 weeks lead |
| 3e | Bring-up in the ordered sequence in Section 8.3. | 6-10 h |
| 3f | Port firmware; re-take Phase 1 measurements on the board (FR-3.17). | 8-12 h |

### 8.2 Board contents

MCU in LQFP-48 with crystal, reset, and SWD; input protection and 3.3 V regulation
with a filtered VDDA; analog front end (protection, buffer, anti-alias) referenced to
VDDA; RS-485 transceiver with termination and TVS; actuator gate driver with flyback;
connectors with transient protection; test points on every rail; a current-measurement
jumper on the MCU rail to enable Phase 4.

**Deliberate omissions:** no BGA, no fine-pitch QFN, no USB, no high-speed interfaces.
This keeps the board two-layer and hand-solderable, so effort goes into the analog and
layout decisions rather than into assembly difficulty.

### 8.3 Bring-up order

Inspection before power → rails with no load (current-limited) → rails under load →
SWD connect and identify → blink/first application → crystal frequency verified →
analog path swept and compared against the LTspice prediction → transceiver idle state
and termination → Modbus poll from the host → re-run Phase 1 measurements. Each step
recorded in `docs/bring-up-log.md` as it happens.

## 9. Verification and measurement plan

Measurements are the deliverable of each phase, not a side effect.

| Measurement | Instrument / method | Phase | Requirement |
|-------------|--------------------|-------|-------------|
| Control-loop period and jitter | GPIO toggle at task entry, logic analyzer | 1, re-taken 3 | NFR-1.8 |
| ISR-to-task latency (worst case) | GPIO toggle in ISR and task, logic analyzer | 1, re-taken 3 | NFR-1.9 |
| CPU utilisation | `vTaskGetRunTimeStats()` | 1 | NFR-1.10 |
| Per-task stack headroom | `uxTaskGetStackHighWaterMark()` | 1 | NFR-1.11 |
| ADC noise floor and ENOB | Short-input capture, standard deviation in LSB | 1, re-taken 3 | NFR-1.12 |
| Anti-alias corner and roll-off | Function-generator sweep, scope amplitude per point | 1, re-taken 3 | FR-1.2, FR-3.10 |
| Step response: rise, overshoot, settling | Host plot from exported samples | 1 | FR-1.3, FR-1.5 |
| Priority inversion comparison | Instrumented run, mutex vs. binary semaphore | 1 | FR-1.7 |
| Update time and image size | Timed host update | 2 | NFR-2.7 |
| Rail voltages and ripple | DMM + scope, no-load and loaded | 3 | FR-3.9 |
| Crystal frequency | Scope or frequency counter on MCO | 3 | FR-3.4 |
| RS-485 turnaround contention | Scope on DE and bus lines | 3 | FR-3.1 |
| Modbus poll rate and round-trip latency | `pymodbus` host client | 3 | FR-3.2 |
| Sleep-mode current per mode | Shunt or INA219 across the FR-3.7 jumper | 4 | FR-4.2 |

### 9.1 The figures this project must produce

Each phase must end with these numbers rather than an estimate: control-loop jitter in
µs, ISR latency in µs, CPU utilisation %, stack headroom %, ADC ENOB in bits, anti-alias
corner frequency, step-response rise time and overshoot, update time in seconds, rail
ripple in mV, crystal error in ppm, Modbus poll interval, sleep current in µA.

## 10. Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| **Scope creep** — the dominant risk | Project stalls half-built; an abandoned board is worth less than a finished small project | Strict phase gating: never start a phase before the previous one is documented. The merged single board removes the two-board multiplier. |
| **Board bug blocks firmware** | Weeks lost | Phases 0, 1, 2, and 3a are all to be completed on the known-good dev board first, so firmware progress never depends on the new hardware. |
| Competing workload for the same time | Two mixed-signal efforts in flight at once; neither finishes | Do not start 3b while another board design is mid-layout. Gate 3b on Phases 0-3a being complete, not on a date. |
| MCU family change invalidates firmware and bootloader | Rework | Decide the MCU before 3b (Section 12). If changing family, NFR-2.9's parameterised flash layout limits the blast radius. |
| First board has defects | Extra spin, ~2 weeks and ~$50 | Include test points, current jumper, and generous decoupling; validate the analog path on breadboard in Phase 1 so the board only re-implements a proven circuit. |
| Fab or parts lead time | Schedule slip | Order parts and boards early; nothing exotic is required. |
| Analog and RS-485 on one 2-layer board | Noise coupling, harder layout | Physically separate the analog section from the transceiver edge; document the compromise (FR-3.15). |

## 11. Effort and cost summary

| Phase | Effort | Cost | Ends with |
|-------|--------|------|-----------|
| 0 | TBD | ~$15 | Working FreeRTOS node with host tests |
| 1 | 12-18 h | ~$10 | Measured control system with CI |
| 2 | 20-28 h | $0 | Field-updatable firmware |
| 3 | 45-65 h | ~$120-190 | Custom board, brought up, firmware ported |
| 4 | ~10 h | included | Measured power budget |
| **Total** | **TBD + ~87-121 h** | **~$145-215** | |

Cost assumes a small run of boards with parts for two or three assemblies.

Phase 0 was previously treated as already complete and so carried no estimate. It is
now build work — five tasks, the HAL/application split, the I2C arbiter and recovery
path, the watchdog supervisor, the DMA display driver, and the host test target — and
its effort must be estimated before the total schedule means anything.

## 12. Open questions

1. **MCU choice for the board — decide before step 3b.** Keep the F103 family
   (LQFP-48) for a first board with minimal new silicon to bring up, or move to a
   Cortex-M4F part such as STM32G431 for a better ADC, hardware FPU for the PID loop,
   and stronger low-power behaviour, at the cost of a HAL written for a less familiar
   family and a bootloader flash layout re-targeted later. Changing family at the same
   time as building a first board doubles the variables during bring-up, which is the
   main argument for staying on F103 for a first board.
2. What is the actual sensor set, and what is the node monitoring?
3. Which flash programmer and toolchain path will be used (ST-Link + OpenOCD, or other)?
4. Is there a target date for finishing any phase, and does it constrain the order?
5. Target board outline size, and whether assembly is hand-soldered or ordered.
