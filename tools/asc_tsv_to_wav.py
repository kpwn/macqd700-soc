#!/usr/bin/env python3
"""
asc_tsv_to_wav.py — convert tb-fpga-top-rom +audio_dump TSV to a WAV.

Input format (per tb_fpga_top_rom.cpp asc_capture_sample()):
    # retired<TAB>sim_time<TAB>pcm_l<TAB>pcm_r
    <retired>\t<sim_time>\t<pcm_l_int16>\t<pcm_r_int16>

The TSV captures one row per audio_sample_valid edge — i.e. one row
per actual ASC sample-tick that fired in the sim.  We derive the sample
rate from the sim_time deltas rather than trusting an external assumption.

In SIM_MODEL on tb_fpga_top_rom, sys_clk_p is the testbench's reference
edge.  Each tick() advances sim_time by 2.  The peripheral path runs on
pb_clk = sys_clk / PB_CLK_RST_DIVIDE = sys_clk / 4 (in SIM_MODEL), so
one pb_clk cycle = 8 sim_time units.

Sonora's hardwired sample rate is ~22 kHz (Q700: 783360 Hz phi2 / 35
≈ 22 381 Hz).  If the captured sample-tick interval implies a much
different rate, either the rate-divider is misconfigured or the
sim/HW clock plan diverges from PB_CLK_HZ.

Usage:
    python3 tools/asc_tsv_to_wav.py /tmp/asc_audio.tsv /tmp/asc_audio.wav
    [--rate 22050]    # override derived rate
    [--sim-time-ns N] # ns per sim_time unit (default: 2.5 = sys_clk @ 200 MHz)
"""
import argparse
import struct
import sys
import wave


def read_tsv(path):
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 4:
                continue
            try:
                samples.append(
                    (int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3]))
                )
            except ValueError:
                continue
    return samples


def derive_rate(samples, sim_time_ns):
    if len(samples) < 2:
        return None
    deltas = [samples[i + 1][1] - samples[i][1] for i in range(len(samples) - 1)]
    # Median to be robust to gaps where the FIFO underflowed and
    # sample-tick paused — those would produce huge outlier deltas.
    deltas_sorted = sorted(deltas)
    median = deltas_sorted[len(deltas_sorted) // 2]
    if median <= 0:
        return None
    period_s = median * sim_time_ns * 1e-9
    return 1.0 / period_s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument(
        "--rate",
        type=int,
        default=None,
        help="override sample rate (Hz); default = derived from sim_time deltas",
    )
    ap.add_argument(
        "--sim-time-ns",
        type=float,
        default=2.5,
        help="ns per sim_time unit (default 2.5 → sys_clk = 200 MHz)",
    )
    args = ap.parse_args()

    samples = read_tsv(args.input)
    if not samples:
        print("no samples in input", file=sys.stderr)
        return 1

    rate = args.rate or derive_rate(samples, args.sim_time_ns)
    if rate is None:
        print("could not derive sample rate; pass --rate", file=sys.stderr)
        return 1

    # Span / duration / first+last
    sim_t0 = samples[0][1]
    sim_t1 = samples[-1][1]
    duration_s = (sim_t1 - sim_t0) * args.sim_time_ns * 1e-9
    pcm_l_min = min(s[2] for s in samples)
    pcm_l_max = max(s[2] for s in samples)
    pcm_r_min = min(s[3] for s in samples)
    pcm_r_max = max(s[3] for s in samples)
    nonzero_l = sum(1 for s in samples if s[2] != 0)
    nonzero_r = sum(1 for s in samples if s[3] != 0)
    distinct_l = len(set(s[2] for s in samples))
    distinct_r = len(set(s[3] for s in samples))

    print(f"input: {args.input}")
    print(f"  samples         : {len(samples)}")
    print(f"  sim duration    : {duration_s*1000:.2f} ms")
    print(f"  derived rate    : {rate:.1f} Hz")
    print(f"  sim_time_ns     : {args.sim_time_ns} ns/unit")
    print(f"  pcm_l range     : [{pcm_l_min}, {pcm_l_max}] (nonzero {nonzero_l}, distinct {distinct_l})")
    print(f"  pcm_r range     : [{pcm_r_min}, {pcm_r_max}] (nonzero {nonzero_r}, distinct {distinct_r})")
    print(f"  first 5 samples : {[(s[2], s[3]) for s in samples[:5]]}")
    print(f"  last 5 samples  : {[(s[2], s[3]) for s in samples[-5:]]}")

    # Write 16-bit signed stereo WAV at the derived (or overridden) rate.
    # We don't try to time-warp to the simulator-produced rate vs Sonora's
    # 22 kHz target — the WAV is written at WHATEVER rate the sim produced
    # samples, so the user hears exactly what the FPGA would emit through
    # the modulator if its sample-tick fired at the same cadence.
    framerate = int(round(rate))
    with wave.open(args.output, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(framerate)
        frames = bytearray()
        for _, _, l, r in samples:
            # int16 little-endian, clamp just in case
            l = max(-32768, min(32767, int(l)))
            r = max(-32768, min(32767, int(r)))
            frames += struct.pack("<hh", l, r)
        w.writeframes(bytes(frames))

    print(f"wrote {args.output} (16-bit stereo @ {framerate} Hz, {duration_s*1000:.0f} ms)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
