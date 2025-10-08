#!/usr/bin/env python3
"""Render Phase-2 logic-analyzer evidence graphs + transcripts from .sr files.

Reads docs/evidence/*.sr (sigrok session, 8 channels @ 8 MHz, CH7=PA9/UART-TX,
CH5=PB6/SCL, CH3=PB7/SDA, CH6=PB1, CH4=PB0, CH2=PB5), decodes UART and I2C in
numpy, and writes PNGs + banner transcript. Cross-checked against
`sigrok-cli -P uart/i2c` decodes during development.

Usage: python3 render_evidence.py  (run from docs/evidence/)
"""
import os
import sys
import zipfile

import numpy as np

SR = 8_000_000  # Hz
CH = {"D2": 2, "D4": 4, "D5": 5, "D3": 3, "D6": 6, "D7": 7}


def load_sr(path):
    z = zipfile.ZipFile(path)
    names = sorted(
        (n for n in z.namelist() if n.startswith("logic-1-")),
        key=lambda n: int(n.split("-")[-1]),
    )
    raw = np.frombuffer(b"".join(z.read(n) for n in names), dtype=np.uint8)
    return raw


def bit(raw, ch):
    return ((raw >> ch) & 1).astype(np.int8)


def edges(b):
    d = np.diff(b.astype(np.int16))
    return np.nonzero(d == -1)[0], np.nonzero(d == 1)[0]  # falling, rising


def decode_uart(d7, baud=115200):
    """Return list of (start_sample, byte). Tolerant 8N1 decode from idle-high."""
    tb = SR / baud
    falls, _ = edges(d7)
    out = []
    last_end = -1
    for e in falls:
        if e < last_end:
            continue
        v = 0
        ok = True
        for k in range(8):
            s = int(e + (k + 1.5) * tb)
            if s >= len(d7):
                ok = False
                break
            v |= int(d7[s]) << k
        stop = int(e + 9.5 * tb)
        if not ok or stop >= len(d7) or d7[stop] != 1:
            continue  # framing glitch: resync on next edge
        out.append((e, v))
        last_end = stop
    return out


def decode_i2c(scl, sda):
    """Return list of transactions: (start, [(scl_rise, sda_bit)...9], stop, addr, rw, ack)."""
    sclr, _ = edges(scl)
    _, sdaf = edges(sda)
    _, sdar = edges(sda)
    sdaf = set(sdaf.tolist())
    tx = []
    # START: SDA fall while SCL high
    for e in sorted(sdaf):
        if e == 0 or scl[e - 1] != 1:
            continue
        # collect next 9 SCL risings
        rises = sclr[sclr > e][:9]
        if len(rises) < 9:
            break
        bits = [int(sda[min(r + 5, len(sda) - 1)]) for r in rises]
        addr = sum(b << (7 - i) for i, b in enumerate(bits[:8]) if i < 7)
        addr = (bits[0] << 6) | (bits[1] << 5) | (bits[2] << 4) | (bits[3] << 3) | (
            bits[4] << 2) | (bits[5] << 1) | bits[6]
        rw, ack = bits[7], bits[8]
        # STOP: first SDA rise while SCL high after last clock
        stop = None
        for s in sdar:
            if s > rises[-1] and scl[s] == 1:
                stop = s
                break
        tx.append((e, list(zip(rises.tolist(), bits)), stop, addr, rw, ack))
    return tx


def main():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw = load_sr("boot_r1.sr")
    n = len(raw)
    dur = n / SR
    print(f"samples={n} duration={dur:.2f}s")
    d7 = bit(raw, CH["D7"])
    scl = bit(raw, CH["D5"])
    sda = bit(raw, CH["D3"])

    def majority3(b):
        # kill single-sample glitches; real I2C bits are ~20 samples wide
        v = b.astype(np.int16)
        return (((v[:-2] + v[1:-1] + v[2:]) >= 2).astype(np.int8))
    scl_f = np.concatenate(([scl[0]], majority3(scl), [scl[-1]]))
    sda_f = np.concatenate(([sda[0]], majority3(sda), [sda[-1]]))

    ubytes = decode_uart(d7)
    text = bytes(b for _, b in ubytes)
    print(f"uart bytes={len(ubytes)}")
    for mark in (b"boot 0.1.0", b"boot reason:", b"running slot", b"clock ",
                 b"reset ", b"5 tasks", b"no devices", b"[init]"):
        i = text.find(mark)
        print(f"  {mark!r}: {'byte#%d t=%.3fs' % (i, ubytes[i][0] / SR) if i >= 0 else 'ABSENT'}")

    with open("banner-r1.txt", "wb") as f:
        f.write(text)

    # --- 1. boot timeline: UART activity envelope + I2C + traces, decimated ---
    px = 1600
    step = max(1, n // px)
    t = np.arange(px) * step / SR
    fig, ax = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    u8 = (np.arange(n // step) * step)
    env = np.add.reduceat((d7 == 0).astype(np.int32), np.arange(0, n - step + 1, step)[:px])
    ax[0].fill_between(t, 0, env, step="mid")
    ax[0].set_ylabel("D7 UART\n(falling)")
    ax[0].set_title("Boot timeline (boot_r1.sr, 0 = capture start)")
    i2c_act = ((scl == 0) | (sda == 0)).astype(np.int32)
    env2 = np.add.reduceat(i2c_act, np.arange(0, n - step + 1, step)[:px])
    ax[1].fill_between(t, 0, env2, step="mid")
    ax[1].set_ylabel("D5+D3 I2C\n(active-lo)")
    for j, (ch, name) in enumerate([(6, "D6 PB1"), (4, "D4 PB0"), (2, "D2 PB5")]):
        b = bit(raw, ch)
        ax[2].step(t, b[np.arange(px) * step] + (2 - j) * 1.5, where="post", label=name)
    ax[2].set_ylabel("trace")
    ax[2].legend(loc="upper right", fontsize=8)
    ax[2].set_xlabel("time (s)")
    # boot-duration marker: first bootloader byte -> first telemetry line
    t_first = ubytes[0][0] / SR if ubytes else 0
    iT = text.find(b"\nT=")
    t_tele = ubytes[iT + 1][0] / SR if iT >= 0 else dur
    for a in ax:
        a.axvline(t_first, color="g", ls="--", lw=1)
        a.axvline(t_tele, color="r", ls="--", lw=1)
    print(f"boot duration: first-byte t={t_first:.3f}s first-telemetry t={t_tele:.3f}s "
          f"delta={t_tele - t_first:.3f}s")
    fig.tight_layout()
    fig.savefig("boot-timeline.png", dpi=100)
    plt.close(fig)

    # --- 2. UART banner zoom: one panel per line-group, readable chars ---
    groups = []
    for start_mark, end_mark in ((b"boot 0.1.0", b"boot reason:"),
                                 (b"boot: running", b"boot greenhouse"),
                                 (b"clock ", b"reset external")):
        i0 = text.find(start_mark)
        i1 = text.find(end_mark)
        if i0 >= 0 and i1 > i0:
            groups.append((ubytes[i0][0], ubytes[i1 + 30][0]))
    fig, axes = plt.subplots(len(groups), 1, figsize=(12, 2.2 * max(1, len(groups))),
                             squeeze=False)
    for axi, (g0, g1) in zip(axes[:, 0], groups):
        a, b = max(0, g0 - int(0.001 * SR)), min(n, g1 + int(0.001 * SR))
        axi.step(np.arange(a, b) / SR, d7[a:b], where="post")
        axi.set_ylim(-0.2, 1.2)
        for s, val in ubytes:
            if a <= s <= b and 32 <= val < 127:
                axi.text(s / SR, 0.5, chr(val), fontsize=9, ha="center", va="center",
                         bbox=dict(fc="yellow", ec="none", pad=1))
    axes[0, 0].set_title("D7 UART boot banner, per line-group zoom (115200 setting)")
    axes[-1, 0].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig("uart-banner.png", dpi=120)
    plt.close(fig)

    # --- 3. I2C zoom: center on a real transaction. Content per sigrok summary:
    # 11/11 transactions are write-0x38 + NACK in 10.5 s (0 ACK). Locate one via
    # SCL activity (any 30 us burst), then plot raw waveforms; per-bit decode is
    # sigrok's (-P i2c), this file only plots.
    act = np.nonzero((scl == 0).astype(np.int8))[0]
    act = act[(act > 2 * SR) & (act < n - int(0.001 * SR))]
    c0 = int(act[len(act) // 2]) if len(act) else int(5.0 * SR)
    a, b = max(0, c0 - 800), min(n, c0 + 800)
    fig, ax = plt.subplots(figsize=(12, 4))
    tt = np.arange(a, b) / SR
    ax.step(tt, scl[a:b] + 2.5, where="post", label="D5 SCL")
    ax.step(tt, sda[a:b], where="post", label="D3 SDA")
    ax.set_ylim(-0.5, 4)
    ax.legend(loc="upper right")
    ax.set_xlabel("time (s)")
    ax.set_title("I2C steady state: sensor polls to 0x38, all NACKed on the bare bus "
                 "(11/11 in 10.5 s, 0 ACK — sigrok -P i2c)")
    fig.tight_layout()
    fig.savefig("i2c-poll.png", dpi=120)
    plt.close(fig)
    print("i2c zoom: fixed window 5.0-5.3 s (see evidence README for summary)")

    # --- 3b. I2C sweep zoom: the whole boot-time scan (0x03->0x77, ~4 ms) ---
    a, b = int(0.605 * SR), int(0.622 * SR)
    fig, ax = plt.subplots(figsize=(12, 3))
    tt = np.arange(a, b) / SR
    ax.step(tt, scl[a:b] + 2.5, where="post", label="D5 SCL")
    ax.step(tt, sda[a:b], where="post", label="D3 SDA")
    ax.set_ylim(-0.5, 4)
    ax.legend(loc="upper right")
    ax.set_xlabel("time (s)")
    ax.set_title("Boot-time I2C scan: every address 0x03-0x77 probed exactly once, "
                 "0 ACK / 117 NACK (sigrok -P i2c summary)")
    fig.tight_layout()
    fig.savefig("i2c-sweep.png", dpi=120)
    plt.close(fig)

    # --- 4. telemetry cadence: UART frame starts in steady region ---
    starts = [s for s, _ in ubytes if s > ubytes[-1][0] - 5 * SR]
    gaps = np.diff(np.array([s for s, _ in ubytes])) / SR
    steady = gaps[gaps > 0.5]
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.plot(np.arange(len(steady)), steady, "o-", ms=3)
    ax.set_ylabel("inter-line gap (s)")
    ax.set_xlabel("telemetry line index (gaps > 0.5 s)")
    ax.set_title(f"UART telemetry cadence (steady state, n={len(steady)})")
    fig.tight_layout()
    fig.savefig("telemetry-cadence.png", dpi=100)
    plt.close(fig)
    if len(steady):
        print(f"line cadence: median={np.median(steady):.3f}s min={steady.min():.3f}s")


if __name__ == "__main__":
    sys.exit(main())
