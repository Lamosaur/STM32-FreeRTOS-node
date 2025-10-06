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

---

## Dev board (Blue Pill) over SWD — first contact

### 2025-09-20 — Identification, flash budget, and the bootloader trap

**Goal:** identify the attached Blue Pill over the ST-Link, establish how much flash this
part really has (Phase 2's bootloader layout depends on it), flash the Phase 0/1 image,
and verify it boots.

**Setup:** Blue Pill labelled F103C8T6, ST-LINK/V2 (firmware V2J37S7, serial
37FF71064E573436F6981443) on the 4-wire SWD header (3V3, SWDIO, SWCLK, GND). **No NRST
line is connected.** Tools: `st-flash` 1.8.0, `st-util`, `gdb` 17.2. Firmware: HEAD
`17724c0` (after the `_estack` fix). The sandbox masqueraded `/dev/bus/usb`; with the
real device exposed the ST-Link works normally.

**Observations:**

- `st-info`: flash 131072 (1 KiB pages), SRAM 20480, chipid `0x410` (STM32F1xx medium
  density), dev-type STM32F1xx_MD. The part is labelled C8T6 (rated 64 KB) but reports
  **128 KB**.
- `F_SIZE` @ `0x1FFFF7E0` = `0x0080` (128 KB). The unique-ID tail reads ASCII `2RKA86`.
- Upper flash is **real, not aliased**: wrote 1024 bytes of pattern at `0x0801FC00`
  (above the 64K mark), read back 1024/1024 identical, and a control read at
  `0x0800FC00` (the mirrored location, had it aliased) stayed `0xFF`. Page erased after.
  So this board genuinely has 128 KB of working flash — consistent with ST's own
  F103xB linker script, which declares `FLASH LENGTH = 128K` for the same die.
- Board flash was completely blank on arrival.
- **`_estack` bug, found and fixed this session:** the shipped image's initial MSP was
  `0x20004FFF` — odd, inherited verbatim from ST's own `STM32F103XB_FLASH.ld`. The
  project script now derives `_estack = ORIGIN(RAM) + LENGTH(RAM)` = `0x20005000`, and
  the script is registered as a link dependency so the fix cannot silently not relink
  again. Image size unchanged (42364 B).
- Flashing works every time ("Flash written and verified"). Because NRST is not
  connected, `st-flash` resets via AIRCR software reset; `--connect-under-reset` is
  unavailable.
- **Reading RCC registers through `st-flash` is not meaningful.** The ST-Link loads a
  flash loader into SRAM that reconfigures the clock tree while it runs; `RCC_CR` /
  `RCC_CFGR` read back the *loader's* state (a PLL configuration matching neither the
  firmware's HSE nor its HSI fallback path). Lesson: do not infer the firmware's clock
  state from `st-flash` reads.
- **The BKP backup registers are a trustworthy read channel and a boot detector.**
  The loader does not touch them; a 280-byte probe image wrote `0x1357`/`0xB007` to
  `BKP_DR1`/`DR2` and they read back exactly and persist across resets. The real
  firmware increments `BKP_DR1` on every boot (`count_boot()`), so a steady DR1 across
  resets means the firmware never ran.
- `CFSR`/`HFSR` = 0: the part has never hard-faulted. `DFSR` bit 24 (HALTED) is set by
  the debug halt itself, not by a fault.
- **Boot problem:** after flashing the real firmware, 3/3 software resets put the CPU
  in the ST factory bootloader — PC `0x1FFFF3BE` polling USART1, with the system-memory
  vector table at `0x1FFFF000` (`SP 0x200001FC`, reset `0x1FFFF765`). Flash contents are
  correct (`SP=0x20005000`, `PC=0x080064C9`) and verify every time; the CPU simply never
  executes them. The probe image booted from flash exactly once (the `0x1357` write).
- Method: `st-util -n -m -p 4242` + `gdb` reads CPU state through the debug interface
  directly, which is *not* loader-contaminated — that is how the bootloader PC was seen.

**Hypothesis:** BOOT0 is floating or not firmly strapped, so the sampled value differs
across resets; most resets sample high and enter the system bootloader. A hard-strapped
jumper could not be inconsistent the way this board is (1 flash-boot out of 4 resets).

**Changes:** none yet — the jumper is on the user's bench, not reachable from here.

**Result:** unresolved until the user acts.

**Open:**

- **User action required:** set BOOT0 (Blue Pill jumper JP1) firmly to the "0"
  position. After that, any reset should boot the application.
- HSE start has never been observed. The banner will report "HSE+PLL" or "HSI fallback
  (crystal did not start)" over USART1 (115200 8N1, PA9/PA10). The ST-LINK/V2 has no VCP,
  so a USB-UART adapter (or the logic analyzer's UART decoder on PA9) is needed to read it.
- `RCC_CSR` reads `0x1C` (bits 2-4) where the ST F1 layout defines those bits as
  reserved. Combined with the 128 KB F_SIZE and the ASCII UID tail, this part presents
  clone characteristics. Recorded so nobody relies on undocumented register bits, but
  nothing needs them so far.

**Follow-up:**

1. User sets BOOT0 to 0; re-flash or reset; expect the boot banner and, from it, the
   HSI/HSE verdict.
2. If the crystal does not start, that is the designed fallback — proceed; the 72 MHz
   timing constants simply do not apply until it does.
3. Then the first on-board tests: FR-0.3 (hold SDA low, `recover`, count 9 SCL pulses +
   1 STOP) and the Phase 1 ADC path with the capacitive probe connected.

---

## Phase 2 — bootloader on silicon (SWD + GDB, no UART bridge)

Tools: `st-flash`/`st-info`/`st-util` 1.8.0, system `gdb` (x86: ARM regs unreadable,
memory reads fine), `arm-none-eabi-objdump`/`size` 13.2.1. `st-util -n -m -p 4242`
holds the probe while running — `st-flash` then reports "unable to claim"; stop it
(`pkill -x st-util`; `-f` matches your own shell) before any flash op. A hung
ST-Link (visible in `lsusb`, invisible to stlink) recovers with a USB
`USBDEVFS_RESET`, no replug needed.

### 2026-10-14 — BOOT0 to 0, HEAD app boots, boot counter live

**Goal:** clear the Sep-20 boot trap and prove the app runs.
**Setup:** BOOT0 (JP1) to 0. `build-arm/firmware/firmware.bin` (43912 B, has `update`).
**Observations:** `st-info --probe` back to 1 programmer after USB reset. `st-flash
--reset write .../firmware.bin 0x08000000` verified; vectors read back
`SP=0x20005000 PC=0x08006549`. `BKP_DR1` (boot count) `7 -> 9` across a reset;
`BKP_DR2` keeps `0xB007` from the Sep-20 probe session — backup domain never lost
power. `st-flash reset` sometimes delivers two system resets (`+2` on one command,
also seen with no bootloader involved); DR1 is monotonic proof of boot, not an exact
reset count.
**Result:** boot blocker cleared. Image stale (Sep build vs HEAD) — reflashed below.

### 2026-10-14 — Phase 2 geometry installed; ladder happy path

**Goal:** bootloader + slot + slip via SWD only.
**Setup:** `bootloader.bin` (4392 B) at `0x08000000`, `firmware.slot.bin` (43912 B,
`SP=0x20005000 PC=0x08008549`, `%4==0`, `< 56 KB`) at `0x08002000`. Slip A crafted
off-target (magic `B007B007`, seq 1, slot A, size, CRC-32 `ccb243af`, v1, CRC-16
over 28 B) and written to `0x0801E000`, read back identical. Metadata pages were
`0xFF` (fresh).
**Observations:** after reset `DR1 9 -> 10`, `SCB->VTOR == 0x08002000` — newest valid
slip verified over live flash and jumped (FR-2.2/FR-2.3 happy path).
**Open:** banner/HSE verdict still needs a UART sniffer (logic analyzer on PA9).

### 2026-10-14 — `st-flash erase` with address mass-erases (tool hazard)

**Observation:** `st-flash erase 0x08002000 0x400` printed "Mass erase completed
successfully" — 1.8.0 ignores the range and wipes the chip (bootloader included).
**Change:** never use `erase` with an address; single pages are cleared with a
range-limited `write` of `0xFF` (erase + program-nothing), corruption with a `write`
of garbage. Full restore via SWD afterward re-verified the happy path (`DR1 10 ->
11`, `VTOR=0x08002000`) — this doubles as the NFR-2.8 recovery drill.
**Follow-up:** record in the NFR-2.8 procedure: recovery is `write`, never `erase`.

### 2026-10-14 — corrupt-image stay + newest-wins + rollback selection

**Setup:** 1 KB `0xAA` written to slot A page 0 (vectors destroyed, slip A intact).
**Observations:** after reset `DR1` static at 11, `VTOR == 0x00000000` (reset default)
— failed image leaves the node in the bootloader (FR-2.2 letter). Slot A restored;
slot B programmed with the slot-A-linked image and slip B (seq 2) written: newest
wins, `VTOR == 0x08010000` (sequence arbitration on silicon). Slot B corrupted:
`VTOR == 0x08002000` — rollback selects A. (Caveat: only a slot-A-linked image
exists in-tree, so slot B can prove selection, never execution.)
**Result:** decision half of FR-2.3 correct; the branch itself was not — next entry.

### 2026-10-14 — BUG: `boot_jump` clears the Thumb bit (FR-2.3 broken as-written)

**Observation:** after the rollback-to-A reset the core sat in HardFault:
`CFSR = 0x00020000` (INVSTATE), `HFSR = 0x40000000` (FORCED), stacked PC even
(`0x08008548`), stacked LR `0x08000573`.
**Hypothesis:** branch to an even address. `firmware/boot/src/jump.c:40`:
`reset = (reset_fn)(app_pc & ~1u)` — clears the mode bit its own comment says to
set. `objdump` of the shipped `bootloader.elf` confirms: `bic.w r3, r2, #1` +
`blx r3`, and `0x08000573` is exactly that `blx` site + 3. Every jump faults;
no application has ever run through this bootloader.
**Change:** one char: `& ~1u` -> `| 1u`. Rebuilt (`build-arm-fix`, system GCC
13.2.1, clean under `-Werror`): `orr.w r3, r2, #1` + `blx r3`, 4356 B.
**Result:** reflashed; `DR1 13 -> 14`, `VTOR=0x08002000`, `CFSR/HFSR` stay 0 —
clean jump, app boots. Host suite cannot catch this (`jump.c` is target-only):
312/312 pass before and after.
**Open:** earlier `DR1` increments under the broken bootloader are unexplained by
any single consistent model (resets sometimes double — proven with no bootloader
involved — plus batched-op misattribution are candidates); not chased further.

### 2026-10-14 — BUG: 32-bit update marker vs 16-bit BKP_DRx (update rung dead)

**Observation:** `BKP_DR3` written with `0x31504455` (`firmware/boot/include/boot/
update.h:24`) reads back `0x00004455`: F1 backup registers are 16-bit hardware
(CMSIS types them `uint32_t`, so `-Wconversion` stays silent). `BKP->DR3 ==
0x31504455` is false on every boot — the `update` command's marker is ignored and
the update-requested rung unreachable.
**Change:** marker `0x5055` ("UP", != `0xFFFF`/`0x0000`/old residue `0x4455`), read
as `(BKP->DR3 & 0xFFFFu) == BOOT_UPDATE_MARKER`, plus `_Static_assert(marker <=
0xFFFF)` (negative-tested: widening fails the build on host and target).
Rebuilt app + slot (44596 B, still `< 56 KB`; wire `~6 s` vs the 30 s NFR-2.7
budget) and reflashed bootloader + slot A + new slip (CRC-32 `4c4934b4):
mid-sequence reset with a stale slip correctly stayed (CRC mismatch), then
`DR1 -> 16`, `VTOR=0x08002000`, no faults.
**Result:** marker round-trips (`DR3 == 0x5055` read back); rung testable — next.

### 2026-10-14 — update-requested entry + 30 s anti-brick expiry

**Setup:** marker `0x5055` set via GDB (needs PWR `DBP`; a plain write to a
write-protected DR3 is silently dropped — first attempt stored nothing).
**Observations:** after reset `DR1` static, `DR3 == 0x5055`, `VTOR == 0` — update
mode entered, no jump (rung 1). NOTE: every GDB attach halts the core and `quit`
leaves it halted — a 130 s "wait" with periodic reads measured nothing for exactly
this reason; the valid run is one reset, zero debugger contact, then one read.
After 60 s untouched: `DR1 17 -> 19`, `DR3 == 0x0000`, `VTOR=0x08002000`,
`CFSR == 0` — window expired, marker cleared, app booted (anti-brick works).
`USART1->SR` polled `RXNE == 0` throughout, so no noise involvement.
**Open:** exact window length unmeasured (needs LA timestamps on PA9:
banner -> `running slot A` delay); `GET_META`/framed update/timed NFR-2.7 run still
need UART TX.

### 2026-10-14 — first light on the wire: banners, HSE verdict, live telemetry (LA)

**Goal:** catch a real boot on the logic analyzer (Saleae-clone via `sigrok-cli`
0.7.2/`fx2lafw`, 8 MHz; `CH7=PA9`, `CH5=PB6`, `CH3=PB7`, `CH6=PB1`, `CH4=PB0`,
`CH2=PB5`, GND common).
**Setup:** two NRST-button presses caught in back-to-back captures; UART decoded at
115200 (bootloader's 115942 is inside tolerance — verified readable).
**Observations:**
- `boot 0.1.0`, `boot reason: newest slot verified`, `boot: running slot A` —
  the ladder on the wire (FR-2.1).
- App: `boot greenhouse-node-01 fw 0.1.0 built Oct 14 2026 20:27:08` (the fixed
  build, identified), **`clock 72 MHz HSE+PLL`** — first HSE verdict: the crystal
  starts. `reset external-pin boots=7` then `8` (+1 per button press, no doubling),
  `heap 9704 bytes free`, `[init] 5 tasks running` (FR-0.1).
- `i2c scan: no devices answered` — bus is bare; no AHT20/OLED attached.
- Steady state: `T=--.-- RH=--.-- seq=0 moist=724 bus=0/N rec=0 feeds=+4/s` —
  ADC path producing values on silicon (FR-1.1 alive), NACKs counted but never
  treated as bus faults (`rec=0`, FR-0.3), watchdog fed, stale readings shown as
  dashes rather than frozen values. I2C decode shows the `0x38` poll + NACK
  repeating; all trace pins toggle (scheduler/ISR/control alive).
- Telemetry at 1 Hz and feeds at 4 Hz match configured rates, which assume a
  72 MHz SysTick — independent corroboration of HSE before the banner was read.
- Count-chain note: banner `boots=7/8` vs the earlier GDB `DR1=19` — a USB reset of
  the ST-Link power-cycled the target (3V3 comes from the probe) and the backup
  domain restarted at 1, exactly the documented power-loss property the NFR-0.7
  instrument relies on.
**Open:** `FR-0.3` nine-pulse waveform (needs SDA held + `recover` = UART TX);
everything else electrical still needs instruments.
