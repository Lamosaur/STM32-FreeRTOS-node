"""Tests for the host-side tooling in host/.

These run under ctest alongside the C++ suite so that the Python and the firmware are
checked by the same command. The parser is worth testing properly because it is the only
thing standing between the node's output and the numbers a measurement gets recorded
from: a parser that silently accepts a malformed line puts a wrong figure in
docs/measurements.md.
"""

from __future__ import annotations

import io
import sys
import unittest
from pathlib import Path

# host/ is a directory of scripts rather than a package, so it goes on the path directly.
_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "host"))

import telemetry  # noqa: E402  (path set up above)


class ParseTelemetryTests(unittest.TestCase):
    def test_parses_a_real_line(self) -> None:
        line = "T=23.45C RH=53.12% seq=17 bus=412/3 rec=1 feeds=48"
        parsed = telemetry.parse_telemetry(line)

        self.assertIsNotNone(parsed)
        assert parsed is not None
        self.assertEqual(parsed.temperature_c, 23.45)
        self.assertEqual(parsed.humidity_rh, 53.12)
        self.assertEqual(parsed.sequence, 17)
        self.assertEqual(parsed.bus_transfers, 412)
        self.assertEqual(parsed.bus_failures, 3)
        self.assertEqual(parsed.bus_recoveries, 1)
        self.assertEqual(parsed.feed_count, 48)
        self.assertTrue(parsed.has_data())

    def test_negative_temperature(self) -> None:
        parsed = telemetry.parse_telemetry("T=-4.20C RH=71.00% seq=2")
        assert parsed is not None
        self.assertEqual(parsed.temperature_c, -4.20)

    def test_reported_no_data_is_none_not_missing(self) -> None:
        # The node says "--.--" when its sensor has no reading. That is a different
        # statement from "this firmware has no such field", and collapsing the two would
        # make a dead sensor look like an old build.
        parsed = telemetry.parse_telemetry("T=--.-- RH=--.-- seq=5")
        assert parsed is not None
        self.assertIsNone(parsed.temperature_c)
        self.assertIsNone(parsed.humidity_rh)
        self.assertEqual(parsed.sequence, 5)
        self.assertFalse(parsed.has_data())

    def test_absent_field_stays_absent(self) -> None:
        parsed = telemetry.parse_telemetry("T=20.00C RH=40.00% seq=1")
        assert parsed is not None
        self.assertIsNone(parsed.bus_transfers)
        self.assertIsNone(parsed.trip_task)

    def test_trip_is_captured(self) -> None:
        parsed = telemetry.parse_telemetry("T=20.00C RH=40.00% seq=1 feeds=9 TRIP=display")
        assert parsed is not None
        self.assertEqual(parsed.trip_task, "display")

    def test_non_telemetry_lines_are_rejected(self) -> None:
        # Banners and command replies must not become rows of nulls in the CSV.
        for line in (
            "",
            "\n",
            "boot greenhouse-node-01 fw 0.1.0",
            "ver 0.1.0 node=greenhouse-node-01",
            "STAT tx=1 rec=0",
            "i2c scan: 2 device(s)",
        ):
            with self.subTest(line=line):
                self.assertIsNone(telemetry.parse_telemetry(line))

    def test_a_truncated_line_yields_what_it_has(self) -> None:
        # Losing the temperature should not lose the bus counters that were already on
        # the wire — those are the numbers a 72 h run is judged on.
        parsed = telemetry.parse_telemetry("T=20.00C RH=40.00% seq=1 bus=90/0")
        assert parsed is not None
        self.assertEqual(parsed.bus_transfers, 90)
        self.assertEqual(parsed.bus_failures, 0)
        self.assertIsNone(parsed.bus_recoveries)


class RunSummaryTests(unittest.TestCase):
    def test_counts_failures_recoveries_and_trips(self) -> None:
        summary = telemetry.RunSummary()
        summary.observe(telemetry.parse_telemetry("T=20.00C RH=40.00% seq=1 bus=10/0 rec=0"))
        summary.observe(telemetry.parse_telemetry("T=20.00C RH=40.00% seq=2 bus=14/1 rec=1"))
        summary.observe(
            telemetry.parse_telemetry("T=20.00C RH=40.00% seq=3 bus=20/1 rec=1 TRIP=sensor")
        )

        self.assertEqual(summary.lines, 3)
        self.assertEqual(summary.bus_failures, 1)
        self.assertEqual(summary.bus_recoveries, 1)
        self.assertEqual(summary.trips, ["sensor"])
        # The counters are cumulative on the node, so the summary must report the peak
        # rather than the last value: a reset would otherwise erase the evidence.
        self.assertEqual(summary.first_sequence, 1)
        self.assertEqual(summary.last_sequence, 3)

    def test_report_mentions_the_things_a_run_is_judged_on(self) -> None:
        summary = telemetry.RunSummary()
        summary.observe(telemetry.parse_telemetry("T=20.00C RH=40.00% seq=1 bus=10/0 rec=0"))
        report = summary.report()
        for expected in ("bus failures", "bus recoveries", "supervisor trips", "sensor samples"):
            self.assertIn(expected, report)


class RecordTests(unittest.TestCase):
    def test_records_only_telemetry_and_counts_the_rest(self) -> None:
        stream = io.StringIO(
            "boot greenhouse-node-01\n"
            "T=21.00C RH=45.00% seq=1 bus=8/0 rec=0 feeds=4\n"
            "STAT tx=8\n"
            "T=21.10C RH=45.10% seq=2 bus=9/0 rec=0 feeds=5\n"
        )

        summary = telemetry.record(stream, csv_writer=None)

        self.assertEqual(summary.lines, 2)
        self.assertEqual(summary.last_sequence, 2)


if __name__ == "__main__":
    unittest.main()
