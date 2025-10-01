#!/usr/bin/env python3
"""Send a firmware image to the bootloader over the framed update protocol (Phase 2).

This is the host half of FR-2.4's "one framed protocol": the bytes are defined by
firmware/boot/include/boot/frame.h, and the tests on both sides pin the same frame
(see test_host_tools.py and test_boot_frame.cpp), which is how "one protocol" is
checked rather than asserted.

The flow of an update is the boot-0 rule played out on the wire:

    HELLO        confirm a bootloader is listening (and which version)
    GET_META     learn the currently active slot, so the new image goes to the OTHER
                 one — the running app is never erased before the new one is verified
    for page in image pages:
        ERASE(page)            page-granular on the F1
        WRITE(page+chunks)     the 1 KB page in <=236-byte chunks, still within the
                               erased page
    COMMIT(size, crc32, ver)   the bootloader re-verifies the image's CRC-32 over live
                               flash and only then writes the metadata slip — a power
                               cut anywhere before this leaves the old app bootable
    RESET                      boot the fresh image

Usage:

    ./host/flasher.py --port /dev/ttyUSB0 --image firmware.bin
    ./host/flasher.py --port /dev/ttyUSB0 --info      # HELLO + GET_META only

The protocol logic is pure and runs against any object with write()/read_byte()/
close(): the CLI uses pyserial when present, the tests use a fake.
"""

from __future__ import annotations

import argparse
import binascii
import sys
import time
import zlib
from dataclasses import dataclass
from typing import Optional, Tuple

# ---------------------------------------------------------------------------
# The protocol, mirrored from firmware/boot/include/boot/frame.h. The layout
# constants below are the host's copy of boot/layout.h (NFR-2.9); the tests pin
# both copies, exactly like the linker-script cross-checks.
# ---------------------------------------------------------------------------
SYNC = 0xAA
MAX_LEN = 240  # payload bytes per frame; a WRITE carries 4 address bytes + <=236 data

CMD_HELLO = 1
CMD_GET_META = 2
CMD_ERASE = 3
CMD_WRITE = 4
CMD_COMMIT = 5
CMD_RESET = 6

STATUS_OK = 0
STATUS_BAD_CRC = 1
STATUS_BAD_CMD = 2
STATUS_BAD_LEN = 3
STATUS_BAD_ADDR = 4
STATUS_BAD_STATE = 5
STATUS_VERIFY_FAIL = 6
STATUS_FLASH_ERROR = 7

STATUS_NAMES = {
    STATUS_BAD_CRC: "bad crc",
    STATUS_BAD_CMD: "bad command",
    STATUS_BAD_LEN: "bad length",
    STATUS_BAD_ADDR: "bad address",
    STATUS_BAD_STATE: "bad state",
    STATUS_VERIFY_FAIL: "image verify failed",
    STATUS_FLASH_ERROR: "flash error",
}

# The host's copy of boot/layout.h's flash map. The 128 KB part measured at bring-up.
FLASH_PAGE_SIZE = 0x400
SLOT_A_BASE = 0x08002000
SLOT_B_BASE = 0x08010000
SLOT_SIZE = 0xE000
SLOT_NAMES = {1: "A", 2: "B"}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no XOR.
    binascii.crc_hqx is exactly this algorithm with a caller-supplied initial value;
    the check value on b"123456789" is 0x29B1, pinned by the tests on both sides."""
    return binascii.crc_hqx(data, 0xFFFF)


def crc32(data: bytes) -> int:
    """CRC-32 (IEEE 802.3): init 0xFFFFFFFF, final XOR 0xFFFFFFFF. zlib.crc32 is the
    same algorithm; check value on b"123456789" is 0xCBF43926."""
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_frame(command: int, payload: bytes = b"") -> bytes:
    """One frame: 0xAA, command, length, payload, CRC-16 little-endian over the
    command..payload bytes. Mirrors boot_frame_encode on the target."""
    if len(payload) > MAX_LEN:
        raise ValueError(f"payload {len(payload)} bytes exceeds the {MAX_LEN} wire limit")
    body = bytes([command, len(payload)]) + payload
    crc = crc16(body)
    return bytes([SYNC]) + body + bytes([crc & 0xFF, crc >> 8])


def slot_base(slot: int) -> int:
    if slot == 1:
        return SLOT_A_BASE
    if slot == 2:
        return SLOT_B_BASE
    raise ValueError(f"not a slot: {slot}")


def other_slot(slot: int) -> int:
    return 2 if slot == 1 else 1


# ---------------------------------------------------------------------------
# A port: anything with write(bytes) and read_byte() is usable.
# ---------------------------------------------------------------------------
class Port:
    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read_byte(self, timeout_ms: int) -> Optional[int]:
        """One byte, or None on timeout. timeout_ms is a hint a backend may ignore."""
        raise NotImplementedError

    def close(self) -> None:
        pass


class SerialPort(Port):
    """pyserial backend, imported lazily so the protocol half runs without it."""

    def __init__(self, device: str, baud: int = 115200) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required for --port; install it or feed the protocol "
                "-side tests with a fake port"
            ) from exc
        self._s = serial.Serial(device, baud, timeout=2.0)

    def write(self, data: bytes) -> None:
        self._s.write(data)

    def read_byte(self, timeout_ms: int) -> Optional[int]:
        byte = self._s.read(1)
        if not byte:
            return None
        return byte[0]

    def close(self) -> None:
        self._s.close()


class Receiver:
    """The byte-driven frame receiver, the Python mirror of the C state machine in
    boot/frame.c. Fed one byte at a time; returns a completed (status, payload) or
    None when the frame is still in flight or was garbage to resync past."""

    def __init__(self) -> None:
        self._phase = 0  # 0 sync, 1 cmd, 2 len, 3 payload, 4 crc-low, 5 crc-high
        self._buf = bytearray()
        self._expected = 0
        self._crc_low = 0

    def feed(self, byte: int) -> Optional[Tuple[int, bytes]]:
        if self._phase == 0:  # sync
            if byte == SYNC:
                self._phase = 1
                self._buf = bytearray()
            return None
        if self._phase == 1:  # command
            self._buf.append(byte)
            self._phase = 2
            return None
        if self._phase == 2:  # length
            if byte > MAX_LEN:
                self._phase = 0  # resync; the frame is over the wire limit
                return None
            self._buf.append(byte)
            self._expected = byte
            self._phase = 4 if byte == 0 else 3
            return None
        if self._phase == 3:  # payload
            self._buf.append(byte)
            self._expected -= 1
            if self._expected == 0:
                self._phase = 4
            return None
        if self._phase == 4:  # CRC low byte
            self._crc_low = byte
            self._phase = 5
            return None
        if self._phase == 5:  # CRC high byte: the frame is complete
            self._phase = 0  # resync regardless of the verdict
            crc_received = (byte << 8) | self._crc_low
            if crc16(bytes(self._buf)) != crc_received:
                return None  # bad CRC: the caller treats it as a retry
            return self._buf[0], bytes(self._buf[2:])
        return None


class UpdateError(Exception):
    """A command was refused, or the link failed in a way retries do not fix."""


class Session:
    """One stop-and-wait exchange per command, with retry on a lost or bad-CRC reply.

    The retry policy mirrors the wire: a frame that never gets a clean reply costs the
    host a retry (the bootloader's receiver resyncs on its own); a NAK with a reason
    other than "bad crc" is a genuine refusal and aborts the update, because retrying a
    refused ERASE would not change the answer."""

    def __init__(self, port: Port, retries: int = 5) -> None:
        self._port = port
        self._retries = retries
        self._receiver = Receiver()

    def command(self, command: int, payload: bytes = b"", timeout_ms: int = 2000) -> Tuple[int, bytes]:
        frame = encode_frame(command, payload)
        last_error = "no reply"
        for _ in range(self._retries):
            self._port.write(frame)
            deadline = time.time() + (timeout_ms / 1000.0)
            reply = None
            while time.time() < deadline:
                byte = self._port.read_byte(50)
                if byte is None:
                    continue
                completed = self._receiver.feed(byte)
                if completed is not None:
                    reply = completed
                    break
            if reply is None:
                last_error = "timeout"
                continue
            status, reply_payload = reply
            if status != STATUS_OK and status != STATUS_BAD_CRC:
                raise UpdateError(f"command {command} refused: {STATUS_NAMES.get(status, status)}")
            return status, reply_payload
        raise UpdateError(f"command {command}: {last_error} after {self._retries} retries")


# ---------------------------------------------------------------------------
# The update itself.
# ---------------------------------------------------------------------------
def read_metadata(session: Session) -> Tuple[int, int, int]:
    """The active slot, its image size and version, from GET_META. Returns
    (slot 0, 0, 0) when there is no valid metadata yet (a fresh part)."""
    _, payload = session.command(CMD_GET_META)
    if len(payload) != 32:
        raise UpdateError(f"metadata reply is {len(payload)} bytes, expected 32")
    fields = [int.from_bytes(payload[o : o + 4], "little") for o in range(0, 32, 4)]
    return fields[2], fields[3], fields[5]  # active_slot, image_size, version


def write_image(session: Session, image: bytes, target_slot: int) -> None:
    """Erase and program every page the image touches, page by page. The F1 programs
    half-words within an erased page, so the sequence is ERASE(page) then WRITE chunks
    that stay inside that page — the same discipline the target's update loop enforces
    (boot_update.c's BAD_STATE rule)."""
    base = slot_base(target_slot)
    for offset in range(0, len(image), FLASH_PAGE_SIZE):
        chunk = image[offset : offset + FLASH_PAGE_SIZE]
        address = base + offset
        session.command(CMD_ERASE, address.to_bytes(4, "little"))
        for chunk_offset in range(0, len(chunk), MAX_LEN - 4):
            part = chunk[chunk_offset : chunk_offset + MAX_LEN - 4]
            payload = (address + chunk_offset).to_bytes(4, "little") + part
            session.command(CMD_WRITE, payload)


def transfer_budget_bytes(image_size: int) -> int:
    """Upper bound on the wire bytes an update of image_size needs, for NFR-2.7's
    30 s budget: one frame per command with a full payload, plus a reply, per frame.
    The real image (currently ~42 KB) needs a few percent less than this bound."""
    pages = (image_size + FLASH_PAGE_SIZE - 1) // FLASH_PAGE_SIZE
    writes = 0
    remaining = image_size
    while remaining > 0:
        page = min(FLASH_PAGE_SIZE, remaining)
        writes += (page + (MAX_LEN - 4) - 1) // (MAX_LEN - 4)
        remaining -= page
    frames = 2 + pages + writes + 2  # hello, meta, erase(s), write(s), commit, reset
    return frames * (5 + MAX_LEN + 5)  # full payload frame + a reply frame each


def commit_image(session: Session, image: bytes, version: int) -> None:
    session.command(CMD_COMMIT,
                    len(image).to_bytes(4, "little")
                    + crc32(image).to_bytes(4, "little")
                    + version.to_bytes(4, "little"))


def update(port: Port, image: bytes, version: int = 1) -> str:
    """Run one full update; returns a human-readable summary of what happened."""
    session = Session(port)

    status, hello = session.command(CMD_HELLO)
    if status != STATUS_OK or not hello.startswith(b"boot "):
        raise UpdateError(f"HELLO: expected a bootloader, got status {status} {hello!r}")

    active_slot, active_size, active_version = read_metadata(session)
    target_slot = other_slot(active_slot) if active_slot else 1
    summary = (
        f"bootloader {hello.decode('ascii', 'replace')}; "
        f"active slot {SLOT_NAMES.get(active_slot, 'none')}"
        f"{f' (v{active_version}, {active_size} B)' if active_slot else ''}; "
        f"writing slot {SLOT_NAMES[target_slot]}"
    )

    write_image(session, image, target_slot)
    commit_image(session, image, version)
    session.command(CMD_RESET)
    return summary + f"; committed v{version} and reset"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", metavar="DEVICE", help="serial device (requires pyserial)")
    parser.add_argument("--image", metavar="BIN", help="application image to install")
    parser.add_argument("--info", action="store_true", help="HELLO + GET_META, then stop")
    parser.add_argument("--version", type=int, default=1, help="app build number (default 1)")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    if not args.port:
        print("--port is required (or use the tests' fake port)", file=sys.stderr)
        return 2

    port = SerialPort(args.port)
    try:
        if args.info:
            session = Session(port)
            status, hello = session.command(CMD_HELLO)
            print(f"bootloader: {hello.decode('ascii', 'replace')} (status {status})")
            slot, size, version = read_metadata(session)
            if slot:
                print(f"active: slot {SLOT_NAMES[slot]} v{version}, {size} bytes")
            else:
                print("active: none (fresh part)")
            return 0
        if not args.image:
            print("--image is required unless --info", file=sys.stderr)
            return 2
        with open(args.image, "rb") as f:
            image = f.read()
        print(update(port, image, args.version))
        return 0
    finally:
        port.close()


if __name__ == "__main__":
    sys.exit(main())