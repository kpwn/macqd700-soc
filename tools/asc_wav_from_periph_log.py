#!/usr/bin/env python3
"""Export ASC FIFO activity from a rom-boot periph log into a WAV file.

This tool is intentionally narrow: it understands the periph event log
format emitted by ``tb_rom_boot.cpp`` and reconstructs a playable PCM
artifact from ASC FIFO writes plus the small set of control registers that
affect playback rate, volume, and FIFO clear behavior.

It does not try to be cycle-accurate audio synthesis. The output is a
reasonable audio artifact built from the observed FIFO write stream.
"""

from __future__ import annotations

import argparse
import re
import sys
import wave
from dataclasses import dataclass, field
from pathlib import Path
from typing import List


LINE_RE = re.compile(
    r"^cycle=(?P<cycle>\d+)\s+committed=(?P<committed>\d+)\s+"
    r"pc=0x(?P<pc>[0-9a-fA-F]+)\s+category=(?P<category>[A-Za-z0-9_]+)\s+"
    r"event=(?P<event>[A-Za-z0-9_]+)\s+addr=0x(?P<addr>[0-9a-fA-F]+)\s+"
    r"value=0x(?P<value>[0-9a-fA-F]+)"
)

ASC_BASE = 0x50014000


def _u8_to_pcm16(sample: int, volume: int) -> int:
    centred = sample - 0x80
    scaled = (centred * volume) // 255
    pcm = scaled << 8
    if pcm < -32768:
        return -32768
    if pcm > 32767:
        return 32767
    return pcm


def _rate_seed_to_hz(rate_seed: int) -> int:
    rate_seed = max(1, int(rate_seed))
    return max(1, int((1_000_000 + rate_seed // 2) // rate_seed))


@dataclass
class AscCapture:
    left: List[int] = field(default_factory=list)
    right: List[int] = field(default_factory=list)
    volume: int = 0xFF
    rate_seed: int = 45
    stereo_hint: bool = False

    def record_fifo_a(self, value: int) -> None:
        self.left.append(_u8_to_pcm16(value, self.volume))

    def record_fifo_b(self, value: int) -> None:
        self.right.append(_u8_to_pcm16(value, self.volume))
        self.stereo_hint = True

    def fifo_clear(self) -> None:
        self.left.clear()
        self.right.clear()

    def sample_rate_hz(self, override: int | None) -> int:
        if override is not None:
            return override
        return _rate_seed_to_hz(self.rate_seed)

    def channel_count(self, force_mono: bool, force_stereo: bool) -> int:
        if force_mono:
            return 1
        if force_stereo:
            return 2
        if self.stereo_hint or self.right:
            return 2
        return 1

    def frames(self, channel_count: int) -> List[tuple[int, ...]]:
        if channel_count == 1:
            source = self.left if self.left else self.right
            return [(sample,) for sample in source]

        nframes = max(len(self.left), len(self.right))
        frames: List[tuple[int, ...]] = []
        for idx in range(nframes):
            left = self.left[idx] if idx < len(self.left) else 0
            right = self.right[idx] if idx < len(self.right) else 0
            frames.append((left, right))
        return frames


def _parse_event_line(line: str):
    match = LINE_RE.match(line)
    if not match:
        return None
    return {
        "category": match.group("category"),
        "event": match.group("event"),
        "addr": int(match.group("addr"), 16),
        "value": int(match.group("value"), 16) & 0xFF,
    }


def _apply_asc_write(capture: AscCapture, addr: int, value: int) -> None:
    off = addr - ASC_BASE
    if off < 0 or off >= 0x1000:
        return

    if off < 0x400:
        capture.record_fifo_a(value)
        return
    if off < 0x800:
        capture.record_fifo_b(value)
        return

    if off == 0x801:
        return
    if off == 0x802:
        capture.stereo_hint = bool(value & 0x02)
        return
    if off == 0x803:
        if value & 0x80:
            capture.fifo_clear()
        return
    if off in (0x806, 0x80A):
        capture.volume = value
        return
    if off == 0x807:
        capture.rate_seed = 23 if (value & 0x03) == 0x03 else 45
        return
    if off == 0x808:
        capture.rate_seed = value or 1
        return


def build_capture(log_path: Path) -> AscCapture:
    capture = AscCapture()
    with log_path.open("r", encoding="utf-8") as fh:
        for line in fh:
            event = _parse_event_line(line)
            if not event or event["category"] != "ASC" or event["event"] != "write":
                continue
            _apply_asc_write(capture, event["addr"], event["value"])
    return capture


def write_wav(
    wav_path: Path,
    capture: AscCapture,
    sample_rate: int | None,
    force_mono: bool,
    force_stereo: bool,
) -> int:
    channels = capture.channel_count(force_mono=force_mono, force_stereo=force_stereo)
    frames = capture.frames(channels)
    rate_hz = capture.sample_rate_hz(sample_rate)
    wav_path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(wav_path), "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(rate_hz)
        payload = bytearray()
        if channels == 1:
            for (sample,) in frames:
                payload += int(sample).to_bytes(2, byteorder="little", signed=True)
        else:
            for left, right in frames:
                payload += int(left).to_bytes(2, byteorder="little", signed=True)
                payload += int(right).to_bytes(2, byteorder="little", signed=True)
        wf.writeframes(bytes(payload))
    return len(frames)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Reconstruct a WAV artifact from ASC event-log writes."
    )
    parser.add_argument("log", help="peripheral event log from tb_rom_boot")
    parser.add_argument("wav", help="output WAV path")
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=None,
        help="override output sample rate in Hz (default: infer from ASC rate seed)",
    )
    parser.add_argument(
        "--mono",
        action="store_true",
        help="force mono output even when FIFO_B activity is present",
    )
    parser.add_argument(
        "--stereo",
        action="store_true",
        help="force stereo output even if FIFO_B activity is absent",
    )
    args = parser.parse_args(argv)

    if args.sample_rate is not None and args.sample_rate <= 0:
        print("asc_wav_from_periph_log: --sample-rate must be positive", file=sys.stderr)
        return 2
    if args.mono and args.stereo:
        print(
            "asc_wav_from_periph_log: choose only one of --mono/--stereo",
            file=sys.stderr,
        )
        return 2

    log_path = Path(args.log)
    wav_path = Path(args.wav)
    if not log_path.is_file():
        print(f"asc_wav_from_periph_log: missing log {log_path}", file=sys.stderr)
        return 2

    capture = build_capture(log_path)
    if not capture.left and not capture.right:
        print("asc_wav_from_periph_log: no ASC FIFO writes found", file=sys.stderr)
        return 1

    frames = write_wav(
        wav_path,
        capture,
        sample_rate=args.sample_rate,
        force_mono=args.mono,
        force_stereo=args.stereo,
    )
    rate_hz = capture.sample_rate_hz(args.sample_rate)
    channels = capture.channel_count(force_mono=args.mono, force_stereo=args.stereo)
    print(
        f"asc_wav_from_periph_log: wrote {frames} frames "
        f"at {rate_hz} Hz, {channels} ch -> {wav_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
