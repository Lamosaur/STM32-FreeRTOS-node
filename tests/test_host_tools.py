"""Tests for the host-side tooling in host/.

These run under ctest alongside the C++ suite so that the Python and the firmware are
checked by the same command. The parser is worth testing properly because it is the only
thing standing between the node's output and the numbers a measurement gets recorded
from: a parser that silently accepts a malformed line puts a wrong figure in
docs/measurements.md.
"""

from __future__ import annotations

import contextlib
import io
import math
import sys
import tempfile
import unittest
from pathlib import Path

# host/ is a directory of scripts rather than a package, so it goes on the path directly.
_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT / "host"))

import step_response  # noqa: E402  (path set up above)
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


# ---------------------------------------------------------------------------
# host/step_response.py — the FR-1.5 step-response analysis
# ---------------------------------------------------------------------------
#
# The interesting property of these tests is that the expected answers do not come from
# the tool. They come from closed forms: the 10-90 % rise time of a first-order response is
# tau*ln(9), the overshoot of a second-order underdamped response is
# exp(-pi*zeta/sqrt(1-zeta^2)), and the settling time of a first-order response into a 2 %
# band is tau*ln(50). Each test states the formula it asserts against and the tolerance
# that the sample grid justifies, so a failure says which of the two is wrong.


def step_csv(
    pre_step_setpoint: float,
    step_setpoint: float,
    baseline: float,
    points: "list[tuple[float, float]]",
) -> str:
    """Build the CSV the firmware exports for a given response curve.

    The shape matters and is asserted in tests/test_step_capture.cpp: the first row is the
    last pre-step sample (it carries the armed setpoint and the measurement taken before the
    plant moved) and every row after it carries the stepped setpoint. The first post-step
    row is at the same instant as the pre-step one, so the step edge is explicit in the
    file.
    """
    rows = [step_response.HEADER, f"0,{pre_step_setpoint},{baseline},0"]
    for t_ms, value in points:
        rows.append(f"{t_ms},{step_setpoint},{value},0")
    return "\n".join(rows) + "\n"


def first_order(tau_ms: float, gain: float, duration_ms: float, step_ms: float):
    """Samples of gain * (1 - exp(-t/tau)) on a grid, as (t, value) pairs."""
    return [
        (float(t), gain * (1.0 - math.exp(-t / tau_ms)))
        for t in range(0, int(duration_ms) + 1, int(step_ms))
    ]


class StepResponseParsingTests(unittest.TestCase):
    def test_the_header_is_the_one_the_firmware_writes(self) -> None:
        # The identical literal appears in tests/test_step_capture.cpp, and it is produced by
        # firmware/app/src/step_capture.cpp. It is a wire format in three places on purpose:
        # a format written down once is one refactor away from changing silently.
        self.assertEqual(step_response.HEADER, "t_ms,setpoint,measurement,output")
        self.assertEqual(
            step_response.COLUMNS, ("t_ms", "setpoint", "measurement", "output")
        )

    def test_a_record_produced_by_the_firmware_parses(self) -> None:
        # The exact bytes tests/test_step_capture.cpp asserts for a record, with the header
        # the firmware emits. If the C++ formatting changes, this stops parsing.
        records = step_response.parse_csv(
            ["t_ms,setpoint,measurement,output", "1100,400,250,-3"]
        )
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0].t_ms, 1100.0)
        self.assertEqual(records[0].setpoint, 400.0)
        self.assertEqual(records[0].measurement, 250.0)
        # Signed: a controller output below the actuator's rest position is legitimate, and
        # dropping the sign would change the response the tool reports on.
        self.assertEqual(records[0].output, -3.0)

    def test_a_reordered_header_is_refused(self) -> None:
        # setpoint and measurement are both integers in the same units, so a tolerant parser
        # would swap a command for a response and report plausible, wrong figures.
        with self.assertRaises(ValueError):
            step_response.parse_csv(["t_ms,measurement,setpoint,output", "0,1,2,3"])

    def test_a_row_with_the_wrong_column_count_is_refused(self) -> None:
        # Silently dropping rows would leave a rise time that looks plausible.
        with self.assertRaises(ValueError):
            step_response.parse_csv([step_response.HEADER, "0,1,2"])

    def test_comments_and_blank_lines_are_skipped(self) -> None:
        records = step_response.parse_csv(
            [
                "# captured on the bench, greenhouse node 01\n",
                step_response.HEADER,
                "\n",
                "0,300,100,0\n",
                "10,300,120,0\n",
            ]
        )
        self.assertEqual(len(records), 2)

    def test_a_trailing_note_on_a_data_line_is_not_supported(self) -> None:
        # Whole-line comments only. A '#' after the last field is a fifth column, and the
        # parser refuses it rather than guessing where the data ends: the firmware never
        # writes one, so a file that has one was hand-edited and is not what was captured.
        with self.assertRaises(ValueError):
            step_response.parse_csv([step_response.HEADER, "0,300,100,0 # note"])


class StepResponseAnalysisTests(unittest.TestCase):
    def test_first_order_rise_time_matches_the_analytic_formula(self) -> None:
        # For y(t) = 1 - exp(-t/tau) the 10 % and 90 % crossings are at tau*ln(10/9) and
        # tau*ln(10), so the rise time is tau*ln(9) = 2.1972 tau. With tau = 100 ms that is
        # 219.72 ms, independent of the units the samples are in.
        tau = 100.0
        records = step_response.parse_csv(
            step_csv(0, 1000, 0.0, first_order(tau, 1000.0, 1000.0, 10.0)).splitlines()
        )
        analysis = step_response.analyse(records)

        self.assertTrue(analysis.ok, analysis.problem)
        self.assertAlmostEqual(analysis.rise_time_ms, tau * math.log(9.0), delta=1.0)
        # The 2 % settling time of a first-order response is tau*ln(50) = 391.2 ms. The
        # reported value is 400 ms because the definition is about samples and the sample
        # grid is 10 ms: the last sample outside the band is at 390 ms.
        self.assertAlmostEqual(analysis.settling_time_ms, tau * math.log(50.0), delta=10.0)
        self.assertTrue(analysis.settled)
        # A first-order response does not overshoot, and the small positive number reported
        # is the noise-free tail of the exponential measured against the mean of the last
        # tenth of the capture.
        self.assertLess(analysis.overshoot_pct, 0.05)
        self.assertAlmostEqual(analysis.peak, 1000.0, delta=1.0)
        self.assertAlmostEqual(analysis.baseline, 0.0, delta=1e-9)
        self.assertLess(abs(analysis.steady_state_error), 0.5)
        self.assertEqual(analysis.step_size, 1000.0)

    def test_a_loop_that_never_reaches_90_percent_reports_no_rise_time(self) -> None:
        # Unity gain is not guaranteed: a response that settles at 90 % of its command never
        # crosses the 90 % line, so there is no rise time to report. None rather than the end
        # of the capture, which is the answer that would look plausible and be wrong.
        records = step_response.parse_csv(
            step_csv(0, 1000, 0.0, first_order(100.0, 900.0, 1000.0, 10.0)).splitlines()
        )
        analysis = step_response.analyse(records)

        self.assertTrue(analysis.ok, analysis.problem)
        self.assertIsNone(analysis.rise_time_ms)
        self.assertIn("not reached", analysis.report())
        # The steady-state error is the figure that explains why: -100 out of a 1000 command.
        self.assertAlmostEqual(analysis.steady_state_error, -100.0, delta=1.0)

    def test_second_order_overshoot_matches_the_damping_formula(self) -> None:
        # The standard second-order underdamped response
        #   y(t) = 1 - exp(-zeta*wn*t)/sqrt(1-zeta^2) * sin(wd*t + acos(zeta)),  wd = wn*sqrt(1-zeta^2)
        # has its first peak at t = pi/wd and an overshoot of exactly
        #   exp(-pi*zeta/sqrt(1-zeta^2)).
        # With zeta = 0.5 that is 16.303 %, at a peak time of 181.38 ms. Sampled at 1 ms the
        # peak is placed to within one sample, which is 0.0004 % of overshoot because the
        # curve is flat at its maximum, so a 0.1 percentage point tolerance is generous.
        zeta, wn = 0.5, 20.0
        wd = wn * math.sqrt(1.0 - zeta**2)
        phi = math.acos(zeta)

        def response(t_s: float) -> float:
            return 1000.0 * (
                1.0
                - math.exp(-zeta * wn * t_s)
                / math.sqrt(1.0 - zeta**2)
                * math.sin(wd * t_s + phi)
            )

        points = [(float(t), response(t / 1000.0)) for t in range(0, 1501, 1)]
        records = step_response.parse_csv(
            step_csv(100, 1100, 0.0, points).splitlines()
        )
        analysis = step_response.analyse(records)

        self.assertTrue(analysis.ok, analysis.problem)
        expected_overshoot = 100.0 * math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta**2))
        self.assertAlmostEqual(analysis.overshoot_pct, expected_overshoot, delta=0.1)
        self.assertAlmostEqual(analysis.peak_time_ms, 1000.0 * math.pi / wd, delta=1.0)
        # No closed form for the settling time exists; the decaying envelope
        # exp(-zeta*wn*t)/sqrt(1-zeta^2) reaches the 2 % band at
        # t = ln(0.02*sqrt(1-zeta^2))/(-zeta*wn) = 405.6 ms, and the response cannot settle
        # before that, so the reported value must be close to it from either side.
        self.assertAlmostEqual(analysis.settling_time_ms, 405.6, delta=10.0)
        self.assertTrue(analysis.settled)
        self.assertAlmostEqual(analysis.steady_state_error, -100.0, delta=1.0)

    def test_an_undamped_oscillation_is_never_called_settled(self) -> None:
        # Constant-amplitude oscillation: it never enters the band and stays there. The
        # analyser must say so rather than reporting the end of the capture as a settling
        # time, which is the failure mode that puts a wrong number in docs/measurements.md.
        points = [
            (float(t), 1000.0 + 300.0 * math.sin(2.0 * math.pi * 2.0 * t / 1000.0))
            for t in range(0, 2001, 10)
        ]
        analysis = step_response.analyse(
            step_response.parse_csv(step_csv(0, 1000, 0.0, points).splitlines())
        )

        self.assertTrue(analysis.ok, analysis.problem)
        self.assertIsNone(analysis.settling_time_ms)
        self.assertFalse(analysis.settled)
        self.assertIn("not settled", analysis.report())
        # And the overshoot is measured: 300 above a 1000 command is 30 % of the step at the
        # first peak, on top of the offset the oscillation is centred on.
        self.assertGreater(analysis.overshoot_pct, 25.0)

    def test_the_figures_are_unit_agnostic(self) -> None:
        # The same curve expressed in different units must give the same times and the same
        # percentages: a tool that reported millivolts or permille inconsistently would make
        # two measurements of the same loop look different.
        small = step_response.analyse(
            step_response.parse_csv(
                step_csv(0, 1, 0.0, first_order(100.0, 1.0, 1000.0, 10.0)).splitlines()
            )
        )
        large = step_response.analyse(
            step_response.parse_csv(
                step_csv(0, 5000, 0.0, first_order(100.0, 5000.0, 1000.0, 10.0)).splitlines()
            )
        )

        self.assertAlmostEqual(small.rise_time_ms, large.rise_time_ms, delta=1e-6)
        self.assertAlmostEqual(small.settling_time_ms, large.settling_time_ms, delta=1e-6)
        self.assertAlmostEqual(small.overshoot_pct, large.overshoot_pct, delta=1e-6)

    def test_a_capture_with_no_step_is_reported_as_unanalysable(self) -> None:
        analysis = step_response.analyse(
            step_response.parse_csv(
                [step_response.HEADER, "0,500,100,0", "10,500,110,0", "20,500,120,0"]
            )
        )

        self.assertFalse(analysis.ok)
        self.assertIsNone(analysis.rise_time_ms)
        self.assertIn("never moved", analysis.problem or "")
        self.assertIn("cannot analyse", analysis.report())

    def test_an_empty_capture_is_reported_as_unanalysable(self) -> None:
        analysis = step_response.analyse([])
        self.assertFalse(analysis.ok)
        self.assertIn("fewer than three records", analysis.problem or "")


class StepResponseCliTests(unittest.TestCase):
    def _run(self, text: str) -> "tuple[int, str]":
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as handle:
            handle.write(text)
            path = handle.name
        output = io.StringIO()
        try:
            with contextlib.redirect_stdout(output):
                code = step_response.main(["--csv", path])
        finally:
            Path(path).unlink()
        return code, output.getvalue()

    def test_a_good_capture_reports_the_figures_and_exits_zero(self) -> None:
        code, output = self._run(
            step_csv(0, 1000, 0.0, first_order(100.0, 1000.0, 1000.0, 10.0))
        )

        self.assertEqual(code, 0)
        for label in ("rise time", "overshoot", "settling", "steady-state error"):
            self.assertIn(label, output)

    def test_an_unanalysable_capture_exits_nonzero(self) -> None:
        # A script capturing a step response must fail loudly rather than record an empty
        # result as success.
        code, output = self._run(step_response.HEADER + "\n")

        self.assertEqual(code, 1)
        self.assertIn("cannot analyse", output)

    def test_a_malformed_capture_exits_with_a_distinct_code(self) -> None:
        code, _ = self._run("t_ms,setpoint,measurement\n0,1,2\n")
        self.assertEqual(code, 2)

    def test_the_analysis_never_depends_on_matplotlib(self) -> None:
        # design-spec.md §9 wants a plot from the exported samples, but CI has no
        # matplotlib and the analysis must still run there. Importing this module, running
        # the analysis and reporting all happen without it; only plot() needs it.
        self.assertIsInstance(step_response.matplotlib_available(), bool)

        if step_response.matplotlib_available():
            self.skipTest("matplotlib is installed here, so the absence path cannot run")

        records = step_response.parse_csv(
            step_csv(0, 1000, 0.0, first_order(100.0, 1000.0, 1000.0, 10.0)).splitlines()
        )
        analysis = step_response.analyse(records)
        self.assertTrue(analysis.ok, analysis.problem)
        self.assertFalse(step_response.plot(records, analysis, "/dev/null"))


# ---------------------------------------------------------------------------
# The flasher: Phase 2's host half of the update protocol (FR-2.4, NFR-2.7)
# ---------------------------------------------------------------------------

import flasher  # noqa: E402


class FakePort(flasher.Port):
    """Serves canned reply frames byte by byte, and records everything sent."""

    def __init__(self, replies):
        self._buffer = b"".join(replies)
        self._pos = 0
        self.sent = bytearray()

    def write(self, data: bytes) -> None:
        self.sent += data

    def read_byte(self, timeout_ms: int) -> Optional[int]:
        if self._pos >= len(self._buffer):
            return None
        byte = self._buffer[self._pos]
        self._pos += 1
        return byte

    def close(self) -> None:
        pass


def ok_reply(payload: bytes = b"") -> bytes:
    return flasher.encode_frame(flasher.STATUS_OK, payload)


def meta_payload(slot: int = 1, size: int = 0x400, version: int = 7) -> bytes:
    """The 32-byte slip layout from boot/metadata.h: magic, sequence, slot, size, crc,
    version, then two reserved words. The host reads slot(8), size(12), version(20)."""
    return b"".join(
        int.to_bytes(v, 4, "little")
        for v in (0xB007B007, 9, slot, size, 0x12345678, version, 0, 0)
    )


def split_frames(data: bytes):
    """Parse a byte stream into (status/cmd, payload) frames through the same receiver
    the session uses — the test thus relies on the code under test only to the extent it
    exercises the receiver, which the canonical-frame tests pin independently."""
    rx = flasher.Receiver()
    frames = []
    for byte in data:
        done = rx.feed(byte)
        if done is not None:
            frames.append(done)
    return frames


class CrcTests(unittest.TestCase):
    def test_crc16_matches_the_ccitt_false_check_value(self) -> None:
        self.assertEqual(flasher.crc16(b"123456789"), 0x29B1)

    def test_crc32_matches_the_ieee_check_value(self) -> None:
        self.assertEqual(flasher.crc32(b"123456789"), 0xCBF43926)


class FrameTests(unittest.TestCase):
    def test_the_canonical_frame_is_the_same_bytes_as_the_c_side(self) -> None:
        # test_boot_frame.cpp decodes exactly these bytes through the C receiver; this is
        # FR-2.4's "one framed protocol" made concrete across the two test suites.
        self.assertEqual(
            flasher.encode_frame(flasher.CMD_HELLO), bytes.fromhex("aa01003e2e")
        )

    def test_an_encoded_reply_decodes_through_the_receiver(self) -> None:
        rx = flasher.Receiver()
        last = None
        for byte in ok_reply(b"boot 0.1.0"):
            last = rx.feed(byte)
        self.assertEqual(last, (flasher.STATUS_OK, b"boot 0.1.0"))

    def test_a_corrupted_frame_is_refused_and_the_receiver_resyncs(self) -> None:
        bad = bytearray(ok_reply(b"boot 0.1.0"))
        bad[-1] ^= 0xFF
        rx = flasher.Receiver()
        for byte in bad:
            self.assertIsNone(rx.feed(byte))
        last = None
        for byte in ok_reply(b"boot 0.1.0"):
            last = rx.feed(byte)
        self.assertEqual(last, (flasher.STATUS_OK, b"boot 0.1.0"))


class SessionTests(unittest.TestCase):
    def test_a_nak_with_a_reason_aborts(self) -> None:
        port = FakePort([flasher.encode_frame(flasher.STATUS_BAD_ADDR)])
        with self.assertRaises(flasher.UpdateError) as ctx:
            flasher.Session(port).command(flasher.CMD_ERASE, (0).to_bytes(4, "little"))
        self.assertIn("bad address", str(ctx.exception))

    def test_a_bad_crc_reply_is_retried_in_place(self) -> None:
        bad = bytearray(ok_reply())
        bad[-1] ^= 0x01
        port = FakePort([bytes(bad), ok_reply()])
        status, _ = flasher.Session(port).command(flasher.CMD_HELLO)
        self.assertEqual(status, flasher.STATUS_OK)


class UpdateTests(unittest.TestCase):
    def test_a_full_update_targets_the_other_slot_and_commits_the_real_crc(self) -> None:
        image = bytes(range(256)) * 5  # 1280 bytes: two pages, exercising the clamp on
                                       # the last write of each page
        replies = [
            ok_reply(b"boot 0.1.0"),    # HELLO
            ok_reply(meta_payload(slot=1, size=0x400, version=3)),  # GET_META: active A
        ]
        for _ in range(2):              # ERASE x2
            replies.append(ok_reply())
        for _ in range(7):              # WRITE: 5 for page 0, 2 for page 1
            replies.append(ok_reply())
        replies.append(ok_reply())      # COMMIT
        replies.append(ok_reply())      # RESET

        port = FakePort(replies)
        summary = flasher.update(port, image, version=9)
        self.assertIn("slot B", summary)      # active A -> the update writes B
        self.assertIn("committed v9", summary)

        frames = split_frames(bytes(port.sent))
        commands = [cmd for cmd, _ in frames]
        # Per page: ERASE the page, then WRITE it (5 frames for a full page, 2 for the
        # half page) — the same erase-before-write discipline the target enforces.
        self.assertEqual(commands, [
            flasher.CMD_HELLO,
            flasher.CMD_GET_META,
            flasher.CMD_ERASE,
        ] + [flasher.CMD_WRITE] * 5 + [
            flasher.CMD_ERASE,
        ] + [flasher.CMD_WRITE] * 2 + [flasher.CMD_COMMIT, flasher.CMD_RESET])

        # Both ERASEs target slot B's pages, not slot A — the running app's pages are
        # never erased before the new image is verified (the boot-0 rule).
        erases = [payload for cmd, payload in frames if cmd == flasher.CMD_ERASE]
        self.assertEqual(int.from_bytes(erases[0][:4], "little"), flasher.SLOT_B_BASE)
        self.assertEqual(
            int.from_bytes(erases[1][:4], "little"),
            flasher.SLOT_B_BASE + flasher.FLASH_PAGE_SIZE,
        )

        # Every byte of the image arrived, in order, across the WRITE frames.
        written = b"".join(payload[4:] for cmd, payload in frames if cmd == flasher.CMD_WRITE)
        self.assertEqual(written, image)

        # COMMIT carries the real size, CRC-32 and version, so the bootloader's own
        # re-verification over flash agrees with the file on the host.
        commit = next(payload for cmd, payload in frames if cmd == flasher.CMD_COMMIT)
        self.assertEqual(commit, (
            len(image).to_bytes(4, "little")
            + flasher.crc32(image).to_bytes(4, "little")
            + (9).to_bytes(4, "little")
        ))

    def test_read_metadata_reports_a_fresh_part(self) -> None:
        port = FakePort([ok_reply(meta_payload(slot=0, size=0, version=0))])
        slot, size, version = flasher.read_metadata(flasher.Session(port))
        self.assertEqual((slot, size, version), (0, 0, 0))


class BudgetTests(unittest.TestCase):
    def test_the_42k_image_fits_nfr_27s_30_second_budget(self) -> None:
        # 115200 baud is 11520 bytes/s; the upper bound must come in well under 30 s even
        # with the current ~42 KB app. (A real bound is a few percent less.)
        size = 42 * 1024 + 512  # larger than today's image
        bytes_needed = flasher.transfer_budget_bytes(size)
        self.assertLess(bytes_needed, 30 * 11520)


if __name__ == "__main__":
    unittest.main()
