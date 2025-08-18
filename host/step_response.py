#!/usr/bin/env python3
"""Analyse a captured step response and report the numbers the spec asks for.

FR-1.5 requires the system to export step-response data for plotting off-target, and
design-spec.md §9 lists the figures that come out of it: rise time, overshoot and settling
time. This tool is the other end of that path — it reads the CSV that
`node::StepCapture` (firmware/app/src/step_capture.cpp) exports and turns it into those
figures.

Why a tool rather than a number in the firmware:

* The metrics are definitions, not measurements. "Rise time" is the 10 %-to-90 % crossing
  time, "overshoot" is measured against the commanded step, and "settling time" depends on
  a chosen band. Putting those definitions on the node would freeze them into firmware for
  a decision that is really about how the result is read, and re-deriving them from the raw
  samples on the host costs nothing and can be redone when the definition is questioned.
* The node cannot plot and has no business trying. It can record 128 samples faithfully and
  send them; plotting is the host's job, and it is optional here so that the analysis runs
  on a machine with no matplotlib, which is most CI machines.

The numbers are only as good as the capture: if a response had not settled when the buffer
filled, this reports "not settled in the captured window" rather than inventing a settling
time, because an invented figure is worse than a missing one when it ends up in a
measurements table.

    ./host/step_response.py --csv step.csv
    ./host/step_response.py --csv step.csv --plot step.png
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, TextIO

# The exact header the firmware writes, in the order it writes it, asserted by
# tests/test_host_tools.py against a literal that also appears in tests/test_step_capture.cpp.
# Column order is load-bearing rather than cosmetic: setpoint and measurement are both
# integers in the same units, so a parser that accepted a reordered file would swap a
# command for a response and report a plausible, wrong set of figures.
COLUMNS = ("t_ms", "setpoint", "measurement", "output")
HEADER = ",".join(COLUMNS)

# Settling band, as a fraction of the commanded step. 2 % is the conventional choice and
# the one design-spec.md §9 implies with "+/-2 % band".
SETTLING_BAND_FRACTION = 0.02

# The reading of the steady state is taken as the mean of the last tenth of the capture.
# A single final sample would be one noise realisation; the whole tail would include the
# transient. A tenth is short enough to be steady and long enough to average the noise.
FINAL_WINDOW_FRACTION = 0.1


@dataclass
class StepRecord:
    """One row of the capture, in whatever unit the firmware recorded."""

    t_ms: float
    setpoint: float
    measurement: float
    output: float


@dataclass
class StepAnalysis:
    """The derived figures, or the reason they could not be derived.

    Every numeric field is Optional because "no overshoot" and "overshoot could not be
    measured" are different statements, and collapsing them into 0.0 would put a fabricated
    figure in the report. `problem` is set instead of raising so that the CLI, the plotter
    and a calling script all get the same explanation.
    """

    samples: int = 0
    problem: Optional[str] = None

    initial_setpoint: Optional[float] = None
    final_setpoint: Optional[float] = None
    step_size: Optional[float] = None

    baseline: Optional[float] = None
    final_value: Optional[float] = None
    peak: Optional[float] = None
    peak_time_ms: Optional[float] = None

    rise_time_ms: Optional[float] = None
    overshoot_pct: Optional[float] = None
    settling_time_ms: Optional[float] = None
    steady_state_error: Optional[float] = None

    # False when the response was still outside the settling band at the end of the
    # capture, which is a property of the capture rather than of the analysis.
    settled: bool = False

    @property
    def ok(self) -> bool:
        return self.problem is None

    def report(self) -> str:
        if self.problem is not None:
            return f"step response: cannot analyse ({self.problem})"

        lines = [
            f"samples:            {self.samples}",
            f"setpoint:           {_num(self.initial_setpoint)} -> {_num(self.final_setpoint)}"
            f"  (step {_signed(self.step_size)})",
            f"baseline:           {_num(self.baseline)}",
            f"steady state:       {_num(self.final_value)}",
            f"rise time 10-90%:   {_measure(self.rise_time_ms, 'ms')}",
            f"peak:               {_num(self.peak)}"
            f" at {_measure(self.peak_time_ms, 'ms')}",
            f"overshoot:          {_measure(self.overshoot_pct, '%')}",
            f"settling (+/-2%):   {_measure(self.settling_time_ms, 'ms')}",
            f"steady-state error: {_signed(self.steady_state_error)}",
        ]
        if not self.settled:
            # Said explicitly rather than left to the reader to infer from a missing
            # settling time: the difference between "settled instantly" and "the capture
            # ended first" is the whole question for a slow loop.
            lines.append("note:               not settled within the captured window")
        return "\n".join(lines)


def _num(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.1f}"


def _signed(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:+.1f}"


def _measure(value: Optional[float], unit: str) -> str:
    return "not reached" if value is None else f"{value:.1f} {unit}"


def parse_csv(lines: Iterable[str]) -> list[StepRecord]:
    """Parse the firmware's CSV: a header, then one row per record.

    Blank lines and `#` comments are skipped so that a capture pasted into a file with a
    note at the top still parses. The header is required and must match COLUMNS exactly —
    see the note on COLUMNS for why a tolerant parser would be the wrong kind of tolerant
    here.

    Raises ValueError for anything that is not this format, rather than skipping rows:
    a capture that silently loses half its rows produces a rise time that looks plausible.
    """
    content = [line for line in lines if line.strip() and not line.lstrip().startswith("#")]
    reader = csv.reader(content)

    header = next(reader, None)
    if header is None:
        return []
    if [field.strip() for field in header] != list(COLUMNS):
        raise ValueError(f"expected header {HEADER!r}, got {','.join(header)!r}")

    records: list[StepRecord] = []
    for number, row in enumerate(reader, start=2):
        if len(row) != len(COLUMNS):
            raise ValueError(f"line {number}: expected {len(COLUMNS)} columns, got {len(row)}")
        try:
            records.append(
                StepRecord(
                    t_ms=float(row[0]),
                    setpoint=float(row[1]),
                    measurement=float(row[2]),
                    output=float(row[3]),
                )
            )
        except ValueError as exc:
            raise ValueError(f"line {number}: not a number: {row}") from exc
    return records


def parse_csv_file(path: str) -> list[StepRecord]:
    with open(path, "r", encoding="utf-8", newline="") as handle:
        return parse_csv(handle)


def _crossing_time(times: Sequence[float], values: Sequence[float], level: float) -> Optional[float]:
    """First time the response reaches `level`, interpolated between samples.

    Interpolated because the sample grid is the control period, not the instrument's
    resolution: at a 100 ms period, reporting the crossing to the nearest sample would put
    a 100 ms quantisation on a rise time of a few hundred milliseconds. Linear
    interpolation between two samples is exact for a straight line and a slight
    overestimate for a convex one, which is the right direction for a rise time.
    """
    for i in range(1, len(values)):
        low, high = values[i - 1], values[i]
        if low == high:
            continue
        if (low < level <= high) or (low > level >= high):
            fraction = (level - low) / (high - low)
            return times[i - 1] + fraction * (times[i] - times[i - 1])
    return None


def analyse(records: Sequence[StepRecord]) -> StepAnalysis:
    """Derive rise time, overshoot, settling time and steady-state error.

    The capture's shape, from firmware/app/src/step_capture.cpp: the first record is the
    last pre-step sample (it carries the setpoint that was armed and the measurement taken
    before the plant could move), and every record after it carries the new setpoint. That
    is what makes the step size derivable from the file alone rather than needing to be
    passed in, and this function relies on it.
    """
    if len(records) < 3:
        return StepAnalysis(samples=len(records), problem="fewer than three records")

    initial_setpoint = records[0].setpoint
    final_setpoint = records[-1].setpoint
    if initial_setpoint == final_setpoint:
        return StepAnalysis(
            samples=len(records),
            problem="the setpoint never moved, so there is no step to analyse",
        )

    command = final_setpoint - initial_setpoint

    # Index of the first post-step sample: the step happens between the last pre-step
    # record and this one. Detected by the setpoint rather than by a threshold, because the
    # firmware already applied the threshold and the file records the result.
    step_index = next(
        (i for i, record in enumerate(records) if record.setpoint != initial_setpoint),
        None,
    )
    if step_index is None or step_index == 0:
        return StepAnalysis(
            samples=len(records),
            problem="no pre-step sample precedes the setpoint change",
        )

    pre_step = records[:step_index]
    response = records[step_index:]

    # The baseline is the mean of the pre-step samples. One sample would be one noise
    # realisation, and overshoot is measured from this value, so noise here becomes a
    # percentage error in the overshoot.
    baseline = sum(record.measurement for record in pre_step) / len(pre_step)

    window = max(1, int(len(response) * FINAL_WINDOW_FRACTION))
    final_value = sum(record.measurement for record in response[-window:]) / window

    times = [record.t_ms for record in response]
    values = [record.measurement for record in response]

    peak_index = max(range(len(values)), key=lambda i: values[i])
    peak = values[peak_index]

    analysis = StepAnalysis(
        samples=len(records),
        initial_setpoint=initial_setpoint,
        final_setpoint=final_setpoint,
        step_size=command,
        baseline=baseline,
        final_value=final_value,
        peak=peak,
        peak_time_ms=times[peak_index] - times[0],
        steady_state_error=final_value - final_setpoint,
    )

    # Overshoot against the commanded step, which is the textbook definition and the one
    # exp(-pi*zeta/sqrt(1-zeta^2)) is derived for. A loop with a large steady-state error
    # therefore shows less overshoot than one that lands on the setpoint, which is why the
    # steady-state error is reported on its own line rather than being folded in here.
    analysis.overshoot_pct = max(0.0, (peak - final_value) / abs(command) * 100.0)

    ten_percent = baseline + 0.1 * command
    ninety_percent = baseline + 0.9 * command
    t10 = _crossing_time(times, values, ten_percent)
    t90 = _crossing_time(times, values, ninety_percent)
    if t10 is not None and t90 is not None and t90 >= t10:
        analysis.rise_time_ms = t90 - t10
    # Both None when the response never reached 90 % of the command inside the capture,
    # which is a truncated capture rather than a fast or slow loop.

    band = abs(command) * SETTLING_BAND_FRACTION
    analysis.settling_time_ms = _settling_time(times, values, final_value, band)
    analysis.settled = analysis.settling_time_ms is not None
    return analysis


def _settling_time(
    times: Sequence[float], values: Sequence[float], final_value: float, band: float
) -> Optional[float]:
    """When the response last left the band around `final_value`.

    The band is centred on the value the loop actually reached, not on the setpoint, so
    that a steady-state error does not by itself stop a response being called settled: the
    two are separate defects and are reported separately. The answer is the time of the
    first sample after which every sample stays inside, not an interpolated crossing —
    interpolation would add precision the definition does not have, since the guarantee is
    about the samples.

    Returns None when the capture ends with the response still outside the band, which is
    the honest answer for a capture that was truncated before settling.
    """
    last_outside = None
    for index, value in enumerate(values):
        if abs(value - final_value) > band:
            last_outside = index

    if last_outside is None:
        return times[0]
    if last_outside == len(values) - 1:
        return None
    return times[last_outside + 1]


def matplotlib_available() -> bool:
    """Whether plotting is possible here. The analysis never depends on the answer."""
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        return False
    return True


def plot(records: Sequence[StepRecord], analysis: StepAnalysis, path: str) -> bool:
    """Write a PNG of the response, or return False if matplotlib is not importable.

    Deliberately optional and deliberately quiet about it: design-spec.md §9 wants a plot
    from the exported samples, but a headless CI machine has no matplotlib and must still
    be able to run the analysis. Returning a bool rather than raising keeps that decision
    with the caller.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")  # a file, not a window: this runs on a build machine
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    times = [record.t_ms for record in records]
    fig, (top, bottom) = plt.subplots(2, 1, sharex=True)

    top.plot(times, [record.measurement for record in records], label="measurement")
    top.plot(times, [record.setpoint for record in records], label="setpoint", linestyle="--")
    top.set_ylabel("measurement")
    top.legend(loc="best")
    top.grid(True, alpha=0.3)

    bottom.plot(times, [record.output for record in records], label="output", color="tab:red")
    bottom.set_xlabel("time since arming (ms)")
    bottom.set_ylabel("controller output")
    bottom.grid(True, alpha=0.3)

    if analysis.ok and analysis.rise_time_ms is not None:
        top.set_title(f"rise {analysis.rise_time_ms:.1f} ms, overshoot "
                      f"{analysis.overshoot_pct:.1f} %")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return True


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--csv", help="capture file; omit to read stdin")
    parser.add_argument("--plot", help="write a PNG of the response (needs matplotlib)")
    args = parser.parse_args(list(argv) if argv is not None else None)

    stream: TextIO
    if args.csv:
        try:
            stream = open(args.csv, "r", encoding="utf-8", newline="")
        except OSError as exc:
            print(f"cannot read {args.csv}: {exc}", file=sys.stderr)
            return 2
    else:
        stream = sys.stdin

    try:
        records = parse_csv(stream)
    except ValueError as exc:
        print(f"malformed capture: {exc}", file=sys.stderr)
        return 2
    finally:
        if args.csv:
            stream.close()

    analysis = analyse(records)
    print(analysis.report())

    if args.plot:
        if plot(records, analysis, args.plot):
            print(f"plot written to {args.plot}")
        else:
            print("matplotlib is not installed: no plot written", file=sys.stderr)

    # A non-zero exit for an unanalysable capture, so that a script capturing a step
    # response fails loudly instead of recording an empty result as success.
    return 0 if analysis.ok else 1


if __name__ == "__main__":
    sys.exit(main())
