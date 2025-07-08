#!/usr/bin/env python3
"""Read the node's telemetry over the host link, and record it.

Why this exists rather than a terminal open on /dev/ttyUSB0:

* NFR-0.7 is a 72 h unattended run. Something has to be watching for the whole of it
  and leaving a record, because "it ran for three days and I did not see it reset" is
  not evidence.
* The measurement plan wants counters over time, not at one instant. A single `stat`
  reading cannot show whether recoveries cluster or are evenly spread.
* FR-1.5 asks for step-response data exported for plotting off-target. That path starts
  here: parse the line into numbers, write them somewhere durable.

It reads a serial port, or stdin, so it can be exercised with no node attached:

    ./host/telemetry.py --port /dev/ttyUSB0 --csv run.csv
    printf 'T=21.50C RH=45.00%% seq=1 bus=10/0 rec=0 feeds=4\\n' | ./host/telemetry.py
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Iterable, Iterator, Optional, TextIO

# One telemetry line, as firmware/app/src/comms.cpp emits it. Written as a set of
# independent field patterns rather than one monolithic regex so that a firmware change
# to one field degrades that field instead of dropping the whole line: losing the
# temperature is better than losing the bus counters with it.
_FIELD_PATTERNS = {
    "temperature_c": re.compile(r"T=(-?\d+\.\d+|--\.--)C"),
    "humidity_rh": re.compile(r"RH=(\d+\.\d+|--\.--)%"),
    "sequence": re.compile(r"seq=(\d+)"),
    "bus_transfers": re.compile(r"bus=(\d+)/\d+"),
    "bus_failures": re.compile(r"bus=\d+/(\d+)"),
    "bus_recoveries": re.compile(r"rec=(\d+)"),
    "feed_count": re.compile(r"feeds=(\d+)"),
    "trip_task": re.compile(r"TRIP=(\S+)"),
}

_INVALID = "--.--"

# Every telemetry line begins with the temperature field. Until the wire format grows
# a distinct line prefix, this is the only thing that distinguishes an unsolicited
# telemetry line from a command reply, so it is a named constant rather than a
# literal buried in the parser.
_TELEMETRY_ANCHOR = "T="


@dataclass
class Telemetry:
    """One node telemetry line, decoded.

    `None` means the node explicitly reported "no data" for that field, which is
    different from the field being absent because the firmware build lacks it. The
    parser keeps those two apart: a missing field stays missing, a reported `--.--`
    becomes None. Collapsing them would make a dead sensor look like an old firmware.
    """

    temperature_c: Optional[float] = None
    humidity_rh: Optional[float] = None
    sequence: Optional[int] = None
    bus_transfers: Optional[int] = None
    bus_failures: Optional[int] = None
    bus_recoveries: Optional[int] = None
    feed_count: Optional[int] = None
    trip_task: Optional[str] = None

    def has_data(self) -> bool:
        return self.temperature_c is not None or self.humidity_rh is not None


def _decode(raw: str) -> object:
    if raw == _INVALID:
        return None
    if "." in raw:
        return float(raw)
    try:
        return int(raw)
    except ValueError:
        return raw


def parse_telemetry(line: str) -> Optional[Telemetry]:
    """Decode one telemetry line, or return None if it is not one.

    A telemetry line is identified by its first field, `T=`, and only then are the rest
    parsed independently. That anchor is load-bearing rather than cosmetic: the `stat`
    command's reply also contains `rec=`, so a parser that merely looked for recognisable
    fields would accept a `stat` line and record a recovery count read out of a status
    dump. Matching on content alone cannot tell the two apart, because they genuinely
    share field names.

    The rest is deliberately tolerant: a field that is absent stays absent, so losing one
    field off the end of a line does not lose the bus counters that came before it.
    """
    line = line.strip()
    if not line or not line.startswith(_TELEMETRY_ANCHOR):
        return None

    values: dict = {}
    for name, pattern in _FIELD_PATTERNS.items():
        match = pattern.search(line)
        if match:
            values[name] = _decode(match.group(1))

    return Telemetry(**values)


@dataclass
class RunSummary:
    """What an unattended run is judged on (NFR-0.7 and FR-0.2)."""

    lines: int = 0
    samples_with_data: int = 0
    bus_failures: int = 0
    bus_recoveries: int = 0
    trips: list = field(default_factory=list)
    first_sequence: Optional[int] = None
    last_sequence: Optional[int] = None

    def observe(self, telemetry: Telemetry) -> None:
        self.lines += 1
        if telemetry.has_data():
            self.samples_with_data += 1
        if telemetry.bus_failures is not None:
            self.bus_failures = max(self.bus_failures, telemetry.bus_failures)
        if telemetry.bus_recoveries is not None:
            self.bus_recoveries = max(self.bus_recoveries, telemetry.bus_recoveries)
        if telemetry.trip_task and telemetry.trip_task not in self.trips:
            self.trips.append(telemetry.trip_task)
        if telemetry.sequence is not None:
            if self.first_sequence is None:
                self.first_sequence = telemetry.sequence
            self.last_sequence = telemetry.sequence

    def report(self) -> str:
        lines = [
            f"telemetry lines:      {self.lines}",
            f"lines with a reading: {self.samples_with_data}",
            f"bus failures:         {self.bus_failures}",
            f"bus recoveries:       {self.bus_recoveries}",
            f"supervisor trips:     {', '.join(self.trips) if self.trips else 'none'}",
        ]
        if self.first_sequence is not None:
            # A sequence that stopped advancing means the sensor stopped delivering even
            # if the node stayed up, which a bare uptime figure would hide.
            lines.append(
                f"sensor samples:       {self.last_sequence - self.first_sequence + 1}"
            )
        return "\n".join(lines)


CSV_COLUMNS = [
    "wall_clock",
    "temperature_c",
    "humidity_rh",
    "sequence",
    "bus_transfers",
    "bus_failures",
    "bus_recoveries",
    "feed_count",
    "trip_task",
]


def read_lines(stream: TextIO) -> Iterator[str]:
    """Yield lines as they arrive.

    Iterating the stream directly rather than using readlines() matters: a 72 h run
    would otherwise buffer three days of output in memory before emitting anything.
    """
    for line in stream:
        yield line


def record(
    stream: TextIO,
    csv_writer: Optional[csv.DictWriter],
    echo: bool = False,
) -> RunSummary:
    summary = RunSummary()
    for line in read_lines(stream):
        telemetry = parse_telemetry(line)
        if telemetry is None:
            if echo:
                # Pass banners and command replies through untouched: during bring-up
                # they are the interesting part.
                sys.stdout.write(line if line.endswith("\n") else line + "\n")
                sys.stdout.flush()
            continue

        summary.observe(telemetry)
        if csv_writer is not None:
            row = asdict(telemetry)
            row["wall_clock"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
            # `None` becomes an empty cell, not the string "None", so a spreadsheet reads
            # it as missing rather than as a category.
            csv_writer.writerow({k: ("" if v is None else v) for k, v in row.items()})
        if echo:
            sys.stdout.write(line if line.endswith("\n") else line + "\n")
            sys.stdout.flush()
    return summary


def open_stream(port: Optional[str], baud: int) -> tuple[TextIO, object]:
    if port is None:
        return sys.stdin, None

    try:
        import serial  # type: ignore
    except ImportError as exc:  # pragma: no cover - depends on the host environment
        raise SystemExit(
            "pyserial is required to read a port: python3 -m venv .venv && "
            ".venv/bin/pip install pyserial"
        ) from exc

    device = serial.Serial(port, baud, timeout=1)
    return device, device


def send_command(port: str, baud: int, command: str) -> None:
    """Send one command line and print the reply.

    Used for the FR-0.3 procedure: hold the bus with a jumper, send `recover`, and watch
    the nine clock pulses on a scope.
    """
    stream, device = open_stream(port, baud)
    assert device is not None
    device.write((command + "\n").encode("ascii"))
    device.flush()

    while True:
        raw = stream.readline()
        if not raw:
            break
        print(raw.rstrip("\n"))
        if raw.lstrip().startswith(("recover", "err")) or raw.rstrip().endswith("stat"):
            break


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port; omit to read stdin")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", help="write decoded telemetry to this CSV file")
    parser.add_argument("--echo", action="store_true", help="also pass every line through")
    parser.add_argument(
        "--command",
        help="send one command (for example 'recover') and print the reply, then exit",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.command:
        if not args.port:
            parser.error("--command needs --port")
        send_command(args.port, args.baud, args.command)
        return 0

    stream, device = open_stream(args.port, args.baud)

    csv_file = None
    writer = None
    try:
        if args.csv:
            csv_file = open(args.csv, "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()

        summary = record(stream, writer, echo=args.echo)
    except KeyboardInterrupt:
        # Ctrl-C is how an unattended run is stopped, so it reports rather than
        # tracebacks.
        summary = RunSummary()
    finally:
        if csv_file is not None:
            csv_file.close()
        if device is not None:
            device.close()

    print(summary.report(), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
