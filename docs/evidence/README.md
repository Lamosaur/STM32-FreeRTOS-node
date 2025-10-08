# Logic-analyzer evidence — Phase 2 (plus free Phase 0/1 sightings)

Captured 2026-10-14 with a generic FX2 clone (`sigrok-cli` 0.7.2, `fx2lafw`
driver) at 8 MHz, 8 channels. Wiring (0-indexed LA channels):

| LA | Target | Signal |
|----|--------|--------|
| CH7 | PA9 | USART1 TX (all firmware text leaves here) |
| CH5 | PB6 | I2C1 SCL, 400 kHz |
| CH3 | PB7 | I2C1 SDA |
| CH6 | PB1 | trace: DMA-complete ISR entry (NFR-1.9) |
| CH4 | PB0 | trace: control evaluation (NFR-1.8) |
| CH2 | PB5 | trace: woken-task entry (NFR-1.9) |
| GND | GND | common ground (without it, decodes lie) |

`PA1` (actuator gate) was not probed — needs a wire for control-output evidence.
No I2C sensors or display are attached: the bus is bare, so every probe NACKs.

## Files

- `boot_r1.sr`, `boot_r2.sr` — two full NRST-button resets through first
  telemetry (~10 s each). Open in PulseView. Decoders: `uart:rx=D7:baudrate=115200`
  (also reads the bootloader's 115942 baud banner — 0.64% off, inside tolerance),
  `i2c:scl=D5:sda=D3`.
- `boot3.sr` — 10.5 s steady state (telemetry + 1 Hz sensor polls).
- `boot-timeline.png` — all channels across `boot_r1.sr`: boot burst at ~0.5 s,
  1 Hz telemetry spikes, dense I2C scan burst, then sparse polls; trace pins alive
  throughout. Green/red dashed lines: first bootloader byte / first telemetry
  (120 ms apart — the scan NACK-fails fast, no 20 ms timeouts).
- `uart-banner.png` — `PA9` zoomed per line-group with decoded text:
  `boot 0.1.0`, `boot reason: newest slot verified`, `boot: running slot A`,
  `boot greenhouse-node-01 fw 0.1.0 built Oct 14 2026 20:27:08`,
  `clock 72 MHz HSE+PLL` (first HSE verdict), `reset external-pin boots=7`,
  `heap 9704 bytes free`, `[init] 5 tasks running`. The ~80 ms silence between
  `boot 0.1.0` and `boot reason:` is the bootloader CRC-32ing the image over live
  flash. Proves FR-2.1, the FR-2.2 happy-path rung, FR-0.1 (5 tasks), HSE start.
- `banner-r1.txt` — the full decoded text of the above (914 bytes).
- `i2c-sweep.png` — the boot-time scan: every address `0x03`–`0x77` probed exactly
  once, **0 ACK / 117 NACK** (`sigrok -P i2c` summary). Proves `scan_bus()` runs
  the full range on silicon.
- `i2c-poll.png` — one steady-state sensor poll (`write 0x38` + NACK): the sensor
  task polls through the arbiter at ~1 Hz; NACKs never trigger bus recovery
  (`rec=0` in telemetry — FR-0.3: recovery runs on bus faults, never on NACK).
  Full `sigrok -P i2c` summary of `boot_r1.sr`: 129 transactions, 0 ACK
  (117 sweep probes + 11× `0x38` polls + 1 extra `0x3C` display attempt).
- `telemetry-cadence.png` — telemetry inter-line gap, median 0.999 s: the 1 Hz
  `kTelemetryPeriodMs` and the 72 MHz SysTick assumption hold on silicon.

## Regenerate

```sh
cd docs/evidence
python3 render.py   # needs numpy + matplotlib
```

UART decode is this repo's own tolerant 8N1 decoder (byte-identical to sigrok's on
these captures); I2C protocol content is sigrok's (`-P i2c`), waveforms are raw.
Render script kept outside the repo (analysis scratch, not a deliverable).
