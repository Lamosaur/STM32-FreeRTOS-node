# Bring-up Log

Written during bring-up, in order, as it happens. Not reconstructed afterward.

**Why this file exists:** a bring-up log is the most useful hardware document in this
repository. A schematic states intent; a log showing what you probed, what did not work,
and what you changed records what the board actually did. Do not tidy it into a success
story — record the failures, they are the point.

**Rules:**

- One entry per session or per distinct problem, newest at the bottom.
- Record the *observation* before the *theory*, and the *change* before the *result*.
- Quantify wherever possible: voltage, current, frequency, timing, part designator.
- If something was left unresolved, say so. Unresolved problems with a clear next step
  read as competence; a log with no open items reads as fiction.

---

## Template

```
### YYYY-MM-DD — <short title>

**Goal:** what I was trying to establish this session.

**Setup:** board revision, supply, instruments, firmware commit.

**Observations:**
- what I measured or saw, with values and reference designators

**Hypothesis:** why I thought that was happening.

**Changes:** what I changed, one variable at a time.

**Result:** what happened after the change. Measured, not felt.

**Open:** what is still unresolved, with the next step.

**Follow-up:** what this implies for the design or the next session.
```

---

## Phase 1 — analog front end on breadboard

The purpose of this phase is to prove the circuit before it is committed to copper:
the Phase 3 board should only ever re-implement a circuit proven here first.

### YYYY-MM-DD — First light on the anti-alias filter

**Goal:**
**Setup:**
**Observations:**
**Hypothesis:**
**Changes:**
**Result:**
**Open:**
**Follow-up:**

### YYYY-MM-DD — ADC noise floor measurement

**Goal:** establish the noise floor and effective resolution (NFR-1.12).

**Setup:** input shorted to analog ground, timer-triggered ADC at the target sample
rate, DMA capture of N samples, offline standard deviation.

**Observations:**
- N samples captured, standard deviation = ___ LSB
- equivalent ENOB = ___ bits

**Hypothesis:**
**Changes:**
**Result:**
**Open:**
**Follow-up:**

---

## Phase 3 — custom MCU and I/O board bring-up

**Work these in order. Do not skip ahead.** Each step assumes the previous one passed;
a failure here should stop progress rather than be worked around, because debugging two
faults at once is how boards get damaged.

Note what this ordering buys: by the time the board exists, the firmware, the
bootloader, and the Modbus protocol will all have been proven on the dev board. So any
failure here is a *hardware* failure, which removes the usual "is it the board or the
code?" ambiguity.

### Step 1 — Inspection before power

- Visual: solder bridges, tombstoned passives, connector orientation, MCU pin 1
- Continuity: VCC-to-GND resistance. **Record the value.** A dead short means stop.
- Confirm no power is applied during this step

**Observations:**
**Result:**
**Open:**

### Step 2 — Rails with no load

- Apply supply current-limited to a low value (e.g. 50 mA)
- Measure each rail: 3.3 V digital ___ V, VDDA ___ V, input ___ V
- Record current draw with no load

**Observations:**
**Result:**
**Open:**

### Step 3 — Rails under load

- Increase current limit to the design value
- Measure rail voltage and ripple on the scope at the MCU rail and at VDDA
- Record ripple in mV peak-to-peak for both rails

**Observations:**
**Result:**
**Open:**

### Step 4 — SWD connect and identify

- Connect the programmer; confirm the debugger identifies the expected device ID
- If it fails: check NRST, BOOT0 strapping, SWDIO/SWCLK pull-ups, and supply first

**Observations:**
**Result:**
**Open:**

### Step 5 — First application

- Flash a minimal application; confirm it runs
- Confirm the bootloader from Phase 2 still boots and jumps correctly (NFR-2.8)

**Observations:**
**Result:**
**Open:**

### Step 6 — Crystal frequency

- Output the clock on MCO, or measure at the crystal
- Record measured frequency and the error against nominal in ppm

**Observations:**
**Result:**
**Open:**

### Step 7 — Analog path sweep

- Function generator into the AFE input, scope at the buffer output
- Sweep across and beyond the passband; record amplitude at each frequency
- Compare against the Phase 1 breadboard measurement and the LTspice prediction
- Note any divergence and the likely cause (stray capacitance, input bias, layout)

**Observations:**
**Result:**
**Open:**

### Step 8 — Transceiver idle state

- DE/RE deasserted; measure A/B differential and idle level
- Verify termination resistance across A/B with power off
- Probe DE against the bus during a turnaround to check for contention (FR-3.1)

**Observations:**
**Result:**
**Open:**

### Step 9 — Modbus poll

- `pymodbus` host client polls the node; confirm register values track the sensor
- Record poll interval and round-trip latency

**Observations:**
**Result:**
**Open:**

### Step 10 — Re-run Phase 1 measurements on the board

- Repeat jitter, ISR latency, CPU utilisation, stack headroom, ADC ENOB, step response
- Update `docs/measurements.md` with the board figures (FR-3.17)
- Record any difference from the dev board and what explains it

**Observations:**
**Result:**
**Open:**

---

## Phase 4 — low power

### YYYY-MM-DD — Current per mode

**Goal:** measure run, STOP, and STANDBY current using the FR-3.7 jumper.

**Setup:** current-measurement jumper removed, ammeter or INA219 in series, no
debugger attached (debug power and clock gating both distort sleep current).

**Observations:**
- Run: ___ mA
- STOP: ___ µA
- STANDBY: ___ µA
- Wake source and latency: ___

**Hypothesis:**
**Changes:**
**Result:**
**Open:** which peripherals remain clocked or unpowered in each mode
**Follow-up:**
