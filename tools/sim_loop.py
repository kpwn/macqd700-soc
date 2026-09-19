#!/usr/bin/env python3
"""Run simulation make targets with the repo-standard Verilator environment."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


STANDARD_ENV = {
    "VERILATOR_THREADS": "4",
    "VERILATOR_JOBS": "4",
    "MAKEFLAGS": "-j1",
}

DEFAULT_SHM_ROOT = Path("/dev/shm/m68k")
BLOCKED_TARGET_WORDS = {
    "clock-report",
    "ddr-pincheck",
    "fpga",
    "gui",
    "impl",
    "jtag",
    "synth",
    "timing",
    "vivado",
}
MAKE_ASSIGNMENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*[?:+!]?=")
SUMMARY_PATTERNS = (
    re.compile(r"^summary:\s+.*\bPASS=\d+", re.IGNORECASE),
    re.compile(r"^\[rom-boot-stop-summary\]\s+summary:\s+", re.IGNORECASE),
    re.compile(r"\btb-all:\s+.*\bPASS=\d+", re.IGNORECASE),
    re.compile(r"\bPASS=\d+.*\b(FAIL|MISMATCH|TIMEOUT)=\d+", re.IGNORECASE),
)
ROM_LASTN_RE = re.compile(r"^\[rom-boot\]\s+last-N trace at (?P<path>\S+)\s*$")


class UsageError(Exception):
    """Invalid CLI usage that should be reported without a traceback."""


@dataclass(frozen=True)
class Invocation:
    repo_root: Path
    command: list[str]
    env_overrides: dict[str, str]
    target: str
    state_dir: Path | None = None
    log_path: Path | None = None

    def display(self) -> str:
        env_parts = [f"{key}={shlex.quote(value)}" for key, value in self.env_overrides.items()]
        command_parts = [shlex.quote(part) for part in self.command]
        return f"cd {shlex.quote(str(self.repo_root))} && " + " ".join(env_parts + command_parts)

    def to_json(self) -> dict[str, object]:
        git_head = None
        git_status = None
        try:
            git_head = run_git(self.repo_root, ["rev-parse", "HEAD"])
            git_status = run_git(self.repo_root, ["status", "--short"])
        except (FileNotFoundError, subprocess.CalledProcessError, UsageError):
            pass
        return {
            "repo_root": str(self.repo_root),
            "command": self.command,
            "env_overrides": self.env_overrides,
            "target": self.target,
            "log_path": str(self.log_path) if self.log_path else None,
            "display": self.display(),
            "git_head": git_head,
            "git_status_short": git_status,
            "recorded_at": datetime.now(timezone.utc).isoformat(),
        }

    @classmethod
    def from_json(cls, payload: dict[str, object], state_dir: Path | None = None) -> "Invocation":
        repo_root = Path(str(payload["repo_root"]))
        command = [str(item) for item in payload["command"]]  # type: ignore[index]
        env_overrides = {
            str(key): str(value)
            for key, value in dict(payload["env_overrides"]).items()  # type: ignore[arg-type]
        }
        target = str(payload.get("target", command[1] if len(command) > 1 else "make"))
        return cls(repo_root=repo_root, command=command, env_overrides=env_overrides, target=target, state_dir=state_dir)


def run_git(repo_root: Path | None, args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


def find_repo_root() -> Path:
    try:
        return Path(run_git(None, ["rev-parse", "--show-toplevel"])).resolve()
    except subprocess.CalledProcessError as exc:
        raise UsageError("run this from inside an m68k-ooo git worktree") from exc


def git_state_dir(repo_root: Path) -> Path:
    try:
        raw = run_git(repo_root, ["rev-parse", "--git-path", "sim-loop"])
    except subprocess.CalledProcessError as exc:
        raise UsageError("failed to resolve git-private sim-loop state directory") from exc
    path = Path(raw)
    if not path.is_absolute():
        path = repo_root / path
    return path


def is_make_assignment(arg: str) -> bool:
    return bool(MAKE_ASSIGNMENT_RE.match(arg))


def first_make_target(make_args: list[str]) -> str:
    for arg in make_args:
        if arg == "--":
            continue
        if arg.startswith("-"):
            continue
        if is_make_assignment(arg):
            continue
        return arg
    raise UsageError("provide a make target, for example: tools/sim_loop.py test TEST=moveq")


def target_is_blocked(target: str) -> bool:
    words = set(re.split(r"[-_]", target.lower()))
    if target.lower() in BLOCKED_TARGET_WORDS:
        return True
    return bool(words & BLOCKED_TARGET_WORDS)


def require_sim_target(target: str, allow_non_sim: bool) -> None:
    if allow_non_sim:
        return
    if target_is_blocked(target):
        raise UsageError(
            f"refusing target '{target}' because this helper is for simulation targets; "
            "rerun with --allow-non-sim only if you intentionally want make to handle it"
        )


def slugify(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "-", text).strip("-")
    return slug or "worktree"


def has_make_var(make_args: Iterable[str], name: str) -> bool:
    prefix = f"{name}="
    return any(arg.startswith(prefix) for arg in make_args)


def apply_shm_args(
    repo_root: Path,
    make_args: list[str],
    env_overrides: dict[str, str],
    shm_root: Path,
    *,
    create_dirs: bool = True,
) -> list[str]:
    prefix = shm_root / slugify(repo_root.name)
    build_dir = prefix / "build"
    romboot_dir = prefix / "romboot"
    tmp_dir = prefix / "tmp"
    if create_dirs:
        for path in (build_dir, romboot_dir, tmp_dir):
            path.mkdir(parents=True, exist_ok=True)
    env_overrides["TMPDIR"] = str(tmp_dir)

    updated = list(make_args)
    if not has_make_var(updated, "BUILD_DIR"):
        updated.append(f"BUILD_DIR={build_dir}")
    if not has_make_var(updated, "ROMBOOT_OUTPUT_ROOT"):
        updated.append(f"ROMBOOT_OUTPUT_ROOT={romboot_dir}")
    return updated


def build_invocation(
    repo_root: Path,
    make_args: list[str],
    *,
    state_dir: Path | None = None,
    log_path: Path | None = None,
    shm: bool = False,
    shm_root: Path = DEFAULT_SHM_ROOT,
    allow_non_sim: bool = False,
    create_shm_dirs: bool = True,
) -> Invocation:
    if not make_args:
        raise UsageError("provide a make target, for example: tools/sim_loop.py tb-alu")

    target = first_make_target(make_args)
    require_sim_target(target, allow_non_sim)

    env_overrides = dict(STANDARD_ENV)
    effective_args = list(make_args)
    if shm:
        effective_args = apply_shm_args(
            repo_root,
            effective_args,
            env_overrides,
            shm_root,
            create_dirs=create_shm_dirs,
        )

    return Invocation(
        repo_root=repo_root,
        command=["make", *effective_args],
        env_overrides=env_overrides,
        target=target,
        state_dir=state_dir,
        log_path=log_path,
    )


def format_duration(seconds: float) -> str:
    total = int(round(seconds))
    minutes, sec = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}h{minutes:02d}m{sec:02d}s"
    if minutes:
        return f"{minutes}m{sec:02d}s"
    return f"{sec}s"


def extract_summary_from_lines(lines: Iterable[str]) -> str | None:
    for line in reversed([line.strip() for line in lines if line.strip()]):
        if any(pattern.search(line) for pattern in SUMMARY_PATTERNS):
            return line
    return None


def extract_summary(log_path: Path) -> str | None:
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            return extract_summary_from_lines(f.readlines())
    except FileNotFoundError:
        return None


def resolve_artifact_path(repo_root: Path, raw_path: str) -> Path:
    path = Path(raw_path)
    if path.is_absolute():
        return path
    return repo_root / path


def extract_rom_summary_from_log_lines(lines: Iterable[str]) -> str | None:
    for line in reversed([line.strip() for line in lines if line.strip()]):
        if line.startswith("[rom-boot-stop-summary] summary:"):
            return line
    return None


def find_last_rom_lastn_path(repo_root: Path, lines: Iterable[str]) -> Path | None:
    for line in reversed([line.strip() for line in lines if line.strip()]):
        match = ROM_LASTN_RE.match(line)
        if match:
            return resolve_artifact_path(repo_root, match.group("path"))
    return None


def render_rom_lastn_summary(repo_root: Path, log_path: Path) -> str | None:
    try:
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except FileNotFoundError:
        return None
    if extract_rom_summary_from_log_lines(lines):
        return None
    lastn_path = find_last_rom_lastn_path(repo_root, lines)
    if lastn_path is None or not lastn_path.exists():
        return None

    import rom_boot_stop_summary

    header, samples = rom_boot_stop_summary.parse_trace(lastn_path)
    if not samples:
        return None
    focus = rom_boot_stop_summary.pick_focus(samples)
    tail = samples[-1]
    return rom_boot_stop_summary.render_compact_summary(
        header.reason if header else "unknown",
        samples,
        focus,
        tail,
    )


def safe_target_name(target: str) -> str:
    return slugify(target).replace(".", "_")


def next_log_path(state_dir: Path, target: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    logs = state_dir / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    return logs / f"{stamp}-{safe_target_name(target)}.log"


def write_script(path: Path, invocation: Invocation) -> None:
    env_parts = [shlex.quote(f"{key}={value}") for key, value in invocation.env_overrides.items()]
    command = " ".join(["env", *env_parts, *[shlex.quote(part) for part in invocation.command]])
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"cd {shlex.quote(str(invocation.repo_root))}\n"
        f"exec {command}\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def record_invocation(state_dir: Path, name: str, invocation: Invocation) -> None:
    state_dir.mkdir(parents=True, exist_ok=True)
    (state_dir / f"{name}.json").write_text(
        json.dumps(invocation.to_json(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_script(state_dir / f"{name}.sh", invocation)


def load_invocation(state_dir: Path, name: str) -> Invocation:
    path = state_dir / f"{name}.json"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise UsageError(f"no {name.replace('-', ' ')} command recorded yet") from exc
    return Invocation.from_json(payload, state_dir=state_dir)


def run_invocation(invocation: Invocation) -> int:
    if invocation.log_path is None:
        raise ValueError("run_invocation requires a log path")

    env = os.environ.copy()
    env.update(invocation.env_overrides)

    print(f"[sim-loop] command: {invocation.display()}")
    print(f"[sim-loop] log:     {invocation.log_path}")

    started = time.monotonic()
    with invocation.log_path.open("w", encoding="utf-8", errors="replace") as log:
        proc = subprocess.Popen(
            invocation.command,
            cwd=invocation.repo_root,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            bufsize=1,
        )
        assert proc.stdout is not None
        try:
            for line in proc.stdout:
                sys.stdout.write(line)
                log.write(line)
        except KeyboardInterrupt:
            proc.terminate()
            proc.wait()
            raise
        returncode = proc.wait()

    elapsed = time.monotonic() - started
    status = "PASS" if returncode == 0 else "FAIL"
    summary = extract_summary(invocation.log_path)
    print(
        f"[sim-loop] result:  {status} exit={returncode} "
        f"runtime={format_duration(elapsed)} log={invocation.log_path}"
    )
    if summary:
        print(f"[sim-loop] summary: {summary}")
    rom_summary = render_rom_lastn_summary(invocation.repo_root, invocation.log_path)
    if rom_summary:
        print(f"[sim-loop] rom:     {rom_summary}")
    return returncode


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a make simulation target with VERILATOR_THREADS=4, "
            "VERILATOR_JOBS=4, and MAKEFLAGS=-j1."
        )
    )
    parser.add_argument("make_args", nargs=argparse.REMAINDER, help="make target followed by optional VAR=value args")
    parser.add_argument("--shm", action="store_true", help="place BUILD_DIR, ROM boot traces, and TMPDIR under /dev/shm/m68k/<worktree>")
    parser.add_argument("--shm-root", type=Path, default=DEFAULT_SHM_ROOT, help="scratch root for --shm")
    parser.add_argument("--dry-run", action="store_true", help="print the command without running or recording it")
    parser.add_argument("--repeat-fail", action="store_true", help="rerun the last command that exited non-zero")
    parser.add_argument("--show-last-fail", action="store_true", help="print the last failing command and script path")
    parser.add_argument("--allow-non-sim", action="store_true", help="allow targets that look like Vivado/JTAG/hardware flows")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        repo_root = find_repo_root()
        state_dir = git_state_dir(repo_root)

        if args.show_last_fail:
            invocation = load_invocation(state_dir, "last-fail")
            print(invocation.display())
            print(f"script: {state_dir / 'last-fail.sh'}")
            return 0

        if args.repeat_fail:
            if args.make_args:
                raise UsageError("--repeat-fail does not accept a new target")
            invocation = load_invocation(state_dir, "last-fail")
        else:
            invocation = build_invocation(
                repo_root,
                args.make_args,
                state_dir=state_dir,
                shm=args.shm,
                shm_root=args.shm_root,
                allow_non_sim=args.allow_non_sim,
                create_shm_dirs=not args.dry_run,
            )

        if args.dry_run:
            print(f"[sim-loop] dry-run: {invocation.display()}")
            return 0

        log_path = next_log_path(state_dir, invocation.target)
        invocation = Invocation(
            repo_root=invocation.repo_root,
            command=invocation.command,
            env_overrides=invocation.env_overrides,
            target=invocation.target,
            state_dir=state_dir,
            log_path=log_path,
        )
        record_invocation(state_dir, "last-run", invocation)
        returncode = run_invocation(invocation)
        if returncode != 0:
            record_invocation(state_dir, "last-fail", invocation)
        return returncode
    except UsageError as exc:
        print(f"sim_loop.py: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[sim-loop] interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
