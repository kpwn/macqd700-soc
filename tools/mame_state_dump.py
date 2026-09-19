#!/usr/bin/env python3
"""mame_state_dump.py — orchestrate MAME → state-replay snapshot.

Boots macqd700 in MAME debugger, sets a breakpoint at SNAPSHOT_PC, and
when hit, dumps:
  - Architectural regs (D0..D7, A0..A7, SR, VBR, USP, SSP, ISP, PC) to
    <out>.txt
  - N bytes of low DRAM (0x00000000..0xN-1), default 4 MiB, overridable
    with --dram-bytes, to <out>.dram

Both files together form a snapshot the RTL testbench can load via
+state_replay=<out> (with side files <out>.txt and <out>.dram).
tb_fpga_top_rom.cpp's loader accepts any DRAM size >= 4 MiB (it reads to
EOF), so --dram-bytes and the C++ side stay in sync automatically.

IMPORTANT: for a snapshot taken after the CPU has enabled its MMU (TC's
enable bit set -- check the dumped STATE_MMU TC= line), --dram-bytes MUST
cover whatever physical address SRP/URP roots its page tables at, or the
RTL replay's page walker reads uninitialized DDR content for the very
first translation (of the snapshot PC itself) and faults immediately.
Pass at least the -ramsize used for capture (--ramsize), e.g.
`--ramsize 8M --dram-bytes 0x800000`.  This is not a hypothetical: a
late-boot snapshot with SRP pointing near the top of an 8 MiB RAM config
and the old fixed 4 MiB window reproduces this exactly (see
docs/BUG_calibration_word_misplaced_0d00.md, the Part that added this
flag).

Usage:
  ./tools/mame_state_dump.py --pc 0x408000fa --out /tmp/snap_pre_buserr \
      [--seconds 10]
  ./tools/mame_state_dump.py --pc 0x40806198 --out /tmp/snap_pre_prune \
      --seconds 45 --dram-bytes 0x800000 --ramsize 8M \
      --hard ~/mame_q700_good/hd753.chd \
      --rompath ~/mame_q700_good/roms \
      --cfg-dir ~/mame_q700_good/cfg --nvram-dir ~/mame_q700_good/nvram \
      --diff-dir ~/mame_q700_good/diff

Requires:
  - mame on PATH (tested 0.264 / Ubuntu 24.04)
  - macqd700 romset zip at /tmp/mame_rompath/macqd700.zip
"""

import argparse
import atexit
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

DRAM_BYTES = 0x400000  # legacy default; override with --dram-bytes


def _parse_hex_u32(text: str, name: str) -> int:
    try:
        value = int(text, 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be hex, got {text!r}") from exc
    if not 0 <= value <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"{name} out of 32-bit range: {text!r}")
    return value


def _validate_debugger_atom(text: str, name: str) -> str:
    # MAME debugger scripts use commas, semicolons, braces, and newlines as
    # command/action delimiters.  Reject them here so --cond and generated
    # paths cannot escape the bpset action.
    if not text:
        raise argparse.ArgumentTypeError(f"{name} must not be empty")
    if re.search(r"[\n\r{},;]", text):
        raise argparse.ArgumentTypeError(
            f"{name} contains MAME debugger delimiters: {text!r}")
    return text


def _mame_path(path: Path, name: str) -> str:
    text = str(path)
    if re.search(r"[\s,{};\n\r]", text):
        raise SystemExit(
            f"[state-dump] ERROR: {name} path is not safe for MAME debugger "
            f"script syntax: {text!r}")
    return text


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pc", required=True,
                    help="snapshot PC, e.g. 0x408000fa")
    ap.add_argument("--out", required=True,
                    help="output basename (writes <out>.txt + <out>.dram)")
    ap.add_argument("--seconds", type=int, default=10,
                    help="MAME emul-seconds to run (default 10)")
    ap.add_argument("--cond", default="1",
                    help="MAME debugger breakpoint condition (default: 1)")
    ap.add_argument("--rompath", default="/tmp/mame_rompath")
    ap.add_argument("--dram-bytes", default=f"0x{DRAM_BYTES:X}",
                    help="DRAM window to snapshot, from phys 0 (default "
                         "0x400000 = 4 MiB).  MUST be >= the highest "
                         "physical address the snapshot's own MMU page "
                         "tables (SRP/URP) can reach, or the RTL replay's "
                         "page walker reads uninitialized DDR content and "
                         "faults on the very first fetch at the snapshot "
                         "PC -- this is not a hypothetical, it is exactly "
                         "what happens with a late-boot (MMU-enabled) "
                         "snapshot and the old fixed 4 MiB default.  Use "
                         "at least the -ramsize passed to MAME, e.g. "
                         "0x800000 for an 8 MiB config.")
    ap.add_argument("--hard", default=None,
                    help="optional MAME '-hard <chd-or-image>' HDD image "
                         "path, for snapshots after boot has started "
                         "reading the SCSI HD (matches the project's "
                         "~/mame_q700_good/run.sh 'verified-good' recipe, "
                         "which is NOT MAME's default and required for a "
                         "reference trace that matches HD-dependent boot "
                         "phases).")
    ap.add_argument("--ramsize", default=None,
                    help="optional MAME '-ramsize <NM>' override, e.g. "
                         "'8M'.  Should agree with --dram-bytes.")
    ap.add_argument("--cfg-dir", default=None)
    ap.add_argument("--nvram-dir", default=None)
    ap.add_argument("--diff-dir", default=None)
    args = ap.parse_args()

    pc = _parse_hex_u32(args.pc, "--pc")
    cond = _validate_debugger_atom(args.cond, "--cond")
    dram_bytes = _parse_hex_u32(args.dram_bytes, "--dram-bytes")

    out_base = Path(args.out)
    out_dir = out_base.parent if out_base.parent != Path("") else Path(".")
    out_dir.mkdir(parents=True, exist_ok=True)
    txt_out = Path(str(out_base) + ".txt")
    dram_out = Path(str(out_base) + ".dram")
    pack_path = out_base

    with tempfile.NamedTemporaryFile(
            prefix=out_base.name + ".", suffix=".txt",
            dir=out_dir, delete=False) as tf:
        tmp_txt = Path(tf.name)
    with tempfile.NamedTemporaryFile(
            prefix=out_base.name + ".", suffix=".dram",
            dir=out_dir, delete=False) as df:
        tmp_dram = Path(df.name)
    tmp_txt.unlink(missing_ok=True)
    tmp_dram.unlink(missing_ok=True)
    tmp_pack = None

    def cleanup_temps() -> None:
        for path in (tmp_txt, tmp_dram, tmp_pack):
            if path is None:
                continue
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass

    atexit.register(cleanup_temps)

    mame_txt = _mame_path(tmp_txt, "trace")
    mame_dram = _mame_path(tmp_dram, "DRAM")

    # Build MAME debug script.
    # bpset action format: tracelog formatted line, save memory, q (quit).
    # MAME's `save filename,start,length[,space]` writes raw bytes.
    # We dump 4 MiB of low DRAM (covers RAM_BYTES the harness preloads).
    # MAME bp action separates commands with `;` inside `{...}`.
    # Keep the action minimal: tracelog (one line of regs), save (4 MiB
    # DRAM dump to <out>.dram), q (quit).  Tracelog format string limits
    # vary with MAME version; keep arg count modest by splitting reg
    # families across multiple tracelog calls.
    # MAME 68k debugger exposes pc, sr, vbr, usp, sp; ISP/MSP are not
    # individually addressable (active SP = `sp`).  We capture `sp` as
    # the live A7 (matches `a7`); RTL replay seeds USP/SSP/ISP shadows
    # using SR.S+SR.M to decide which bank `sp` belongs to.
    #
    # Also dump MMU CSRs (TC / SRP / URP / DTT0 / DTT1 / ITT0 / ITT1)
    # so the RTL walker uses MAME-equivalent translation state when
    # replaying from a snapshot taken after MMU enable.
    script = (
        f'bpset 0x{pc:X},{cond},'
        '{'
        f'trace {mame_txt},0,noloop;'
        'tracelog "STATE_HDR PC=%X SR=%X VBR=%X USP=%X SP=%X\\n",'
        'pc,sr,vbr,usp,sp;'
        'tracelog "STATE_D D0=%X D1=%X D2=%X D3=%X D4=%X D5=%X D6=%X D7=%X\\n",'
        'd0,d1,d2,d3,d4,d5,d6,d7;'
        'tracelog "STATE_A A0=%X A1=%X A2=%X A3=%X A4=%X A5=%X A6=%X A7=%X\\n",'
        'a0,a1,a2,a3,a4,a5,a6,a7;'
        'tracelog "STATE_MMU TC=%X SRP=%X URP=%X DTT0=%X DTT1=%X ITT0=%X ITT1=%X CACR=%X\\n",'
        'tc,srp,urp,dtt0,dtt1,itt0,itt1,cacr;'
        'trace off;'
        f'save {mame_dram},0,0x{dram_bytes:X};'
        'q'
        '}\n'
        'g\n'
    )

    with tempfile.NamedTemporaryFile(
            mode="w", suffix=".txt", delete=False) as f:
        f.write(script)
        script_path = f.name

    print(f"[state-dump] script at {script_path}")
    print(f"[state-dump] target PC=0x{pc:X}")
    print(f"[state-dump] DRAM dump → {dram_out}")
    print(f"[state-dump] trace (incl STATE line) → {txt_out}")

    env = os.environ.copy()
    env["QT_QPA_PLATFORM"] = "offscreen"

    cmd = [
        "mame",
        "-rompath", args.rompath,
        "-video", "none",
        "-sound", "none",
        "-nothrottle",
        "-debug",
        "-debugscript", script_path,
        "-seconds_to_run", str(args.seconds),
    ]
    if args.cfg_dir:
        cmd += ["-cfg_directory", args.cfg_dir]
    if args.nvram_dir:
        cmd += ["-nvram_directory", args.nvram_dir]
    if args.diff_dir:
        cmd += ["-diff_directory", args.diff_dir]
    if args.ramsize:
        cmd += ["-ramsize", args.ramsize]
    # "macqd700" must precede device options like -hard (MAME rejects
    # "unknown option -hard" if it comes first) -- matches the project's
    # own ~/mame_q700_good/run.sh convention.
    cmd += ["macqd700"]
    if args.hard:
        cmd += ["-hard", args.hard]
    print(f"[state-dump] running: {' '.join(cmd)}")
    rc = subprocess.call(cmd, env=env, stdin=subprocess.DEVNULL)
    print(f"[state-dump] mame exit={rc}")
    if rc != 0:
        print("[state-dump] ERROR: MAME exited non-zero", file=sys.stderr)
        return 1

    # Verify outputs exist + extract STATE line.
    if not tmp_txt.exists():
        print(f"[state-dump] ERROR: trace not produced at {tmp_txt}",
              file=sys.stderr)
        return 2
    if not tmp_dram.exists():
        print(f"[state-dump] ERROR: DRAM dump not produced at {tmp_dram} "
              f"(likely BP not hit within {args.seconds}s)", file=sys.stderr)
        return 3

    state_hdr = state_d = state_a = state_mmu = None
    with open(tmp_txt) as f:
        for line in f:
            if line.startswith("STATE_HDR "):
                state_hdr = line[len("STATE_HDR "):].strip()
            elif line.startswith("STATE_D "):
                state_d = line[len("STATE_D "):].strip()
            elif line.startswith("STATE_A "):
                state_a = line[len("STATE_A "):].strip()
            elif line.startswith("STATE_MMU "):
                state_mmu = line[len("STATE_MMU "):].strip()
            if state_hdr and state_d and state_a and state_mmu:
                break
    if not (state_hdr and state_d and state_a and state_mmu):
        print(f"[state-dump] ERROR: incomplete STATE in {tmp_txt} "
              f"(hdr={bool(state_hdr)} d={bool(state_d)} a={bool(state_a)} "
              f"mmu={bool(state_mmu)})", file=sys.stderr)
        return 4

    state_line = " ".join([state_hdr, state_d, state_a, state_mmu])
    print(f"[state-dump] {state_line}")
    dram_size = tmp_dram.stat().st_size
    print(f"[state-dump] {tmp_dram}: {dram_size} bytes")
    if dram_size != dram_bytes:
        print(f"[state-dump] ERROR: expected {dram_bytes} bytes, got {dram_size}",
              file=sys.stderr)
        return 5

    # Repack into a single state file <out> for tb_fpga_top_rom +state_replay.
    with tempfile.NamedTemporaryFile(
            prefix=out_base.name + ".", suffix=".state",
            dir=out_dir, delete=False) as out:
        tmp_pack = Path(out.name)
        out.write(b"# state_replay_v1\n")
        for kv in state_line.split():
            out.write((kv + "\n").encode())
        out.write(b"DRAM\n")  # marker; binary follows
        with open(tmp_dram, "rb") as df:
            out.write(df.read())

    os.replace(tmp_txt, txt_out)
    os.replace(tmp_dram, dram_out)
    os.replace(tmp_pack, pack_path)
    print(f"[state-dump] packed snapshot → {pack_path} "
          f"({pack_path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
