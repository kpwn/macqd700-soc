"""OpenOCD batch-command helpers for the JTAG bring-up tools.

This stays intentionally small: the caller supplies the OpenOCD config
files and the Tcl commands to execute.  The helper only resolves the
binary, assembles the command line, and parses simple KEY=VALUE output.
"""

from __future__ import annotations

import os
import re
import shutil
from pathlib import Path
from typing import Iterable, Sequence


KV_LINE_RE = re.compile(r"^([A-Za-z0-9_]+)=0x([0-9A-Fa-f]+)$")


def resolve_openocd(cli_value: str | None) -> str:
    if cli_value:
        return cli_value
    if os.environ.get("OPENOCD"):
        return os.environ["OPENOCD"]
    found = shutil.which("openocd")
    return found or "openocd"


def resolve_openocd_cfgs(cli_values: Sequence[str] | None) -> list[str]:
    if cli_values:
        return [str(Path(value)) for value in cli_values]
    env_value = os.environ.get("OPENOCD_CFG", "").strip()
    if not env_value:
        return []
    return [part for part in env_value.split(os.pathsep) if part]


def openocd_cmd(binary: str, cfgs: Sequence[str], *commands: str,
                target: str | None = None) -> list[str]:
    cmd = [binary]
    for cfg in cfgs:
        cmd.extend(["-f", str(cfg)])
    cmd.extend(["-c", "init"])
    if target:
        cmd.extend(["-c", f"targets {target}"])
    for command in commands:
        cmd.extend(["-c", command])
    cmd.extend(["-c", "shutdown"])
    return cmd


def parse_kv_output(text: str) -> dict[str, int]:
    """Parse simple `KEY=0x...` lines from OpenOCD Tcl output."""
    values: dict[str, int] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        m = KV_LINE_RE.match(line)
        if m:
            values[m.group(1)] = int(m.group(2), 16)
    return values


def format_openocd_command(binary: str, cfgs: Sequence[str], *commands: str,
                           target: str | None = None) -> str:
    """Return a shell-escaped preview of an OpenOCD batch invocation."""
    import shlex

    return shlex.join(openocd_cmd(binary, cfgs, *commands, target=target))

