"""atraps — A-line (Mac OS Toolbox/OS trap) opcode-word → name table.

Classic Mac OS uses the whole `1010` (`$A`) top-nibble opcode space for
software traps ("A-line" instructions, decoded here as UOP F-line's
sibling — see the 68040 PRM's unimplemented-instruction A-line trap).
Two disjoint sub-spaces share that nibble:

  * **OS traps**, `$A000`-`$A7FF` (bit 11 clear): the trap number is the
    LOW BYTE only (`opword & 0x00FF`, 256 possible traps).  Bits 8-11 are
    flag bits (a "new"/register-saving convention historically used by
    the trap dispatcher) — NOT part of the trap number.  This is the
    Device Manager / File Manager glue block: `_Open`, `_Read`,
    `_Control`, ... plus a handful of OS Utility calls like `_DTInstall`.

  * **Toolbox traps**, `$A800`-`$AFFF` (bit 11 set): the trap number is
    the low 10 bits (`opword & 0x03FF`, 1024 possible traps).  Bit 10 is
    a flag (this project's own ROM notes call it the old-style-vs-new
    distinction — see `docs/BUG_low_ram_corruption_post_dafb.md` /
    `project_vector10_wildjump_2026_07_22.md` in the sibling m68k-ooo
    repo's memory notes, which independently derive `entry_addr =
    0x1E00 + (opword - 0xAC00) * 4` for the "old-style" `$A800-$ABFF`
    sub-range).  Masking with `0x03FF` folds both the bit-10 flag and any
    register-saving flag away, canonicalising every alias of a given
    trap down to one `0xA800-0xABFF` value.

Every value in ``ATRAPS`` below is a CANONICAL (post-mask) opcode word —
i.e. exactly what a real ROM disassembly would show for that trap with
its flag bits at their default (0) setting.  ``lookup()`` masks an
arbitrary live opword the same way before looking it up, so a decorated
variant (extra flag bits set) still resolves correctly.

Cross-checked against the small hardcoded table in `tools/jtag_repl.tcl`
(`atrap_print_list`, ~line 1812): `_Open`/`_Close`/`_Read`/`_Write`/
`_Control`/`_Status`/`_DTInstall`/`_InitGraf`/`_WaitNextEvent`/
`_Dequeue`/`_Enqueue`/`_SCSIDispatch`/`_SysError` all match that table
exactly — no disagreement to flag.

Correctness-over-coverage: this table is intentionally NOT a complete
trap dictionary.  Every entry is either (a) cross-checked against
`jtag_repl.tcl`'s own table, (b) confirmed by name in this project's own
ROM-disassembly debugging notes, or (c) a trap number I hold genuinely
high confidence in from long-standing, extremely stable public
convention.  Anything I was not sure of is deliberately left out — see
the comment block at the bottom of this file for the omission list
rather than silently guessing a wrong number onto a real name.
"""

from __future__ import annotations

# Bit 11 (0x0800) of an A-line opword: 0 = OS trap, 1 = Toolbox trap.
TOOLBOX_BIT = 0x0800

# OS traps: only the low byte is the trap number; bits 8-11 are flags.
OS_TRAP_MASK = 0x00FF

# Toolbox traps: the low 10 bits are the trap number; bit 10 (and any
# higher bits within the nibble) are flags.
TOOLBOX_TRAP_MASK = 0x03FF

ATRAPS: dict[int, str] = {
    # ── OS traps ($A000-$A0FF after masking) ───────────────────────────
    # Device Manager / File Manager glue block.  This exact consecutive
    # $A000-$A018 sequence is long-standing, extremely stable classic Mac
    # OS convention (not sourced from this project's own notes).
    0xA000: "_Open",
    0xA001: "_Close",
    0xA002: "_Read",
    0xA003: "_Write",
    0xA004: "_Control",
    0xA005: "_Status",
    0xA006: "_KillIO",
    0xA007: "_GetVolInfo",
    0xA008: "_Create",
    0xA009: "_Delete",
    0xA00A: "_OpenRF",
    0xA00B: "_Rename",
    0xA00C: "_GetFileInfo",
    0xA00D: "_SetFileInfo",
    0xA00E: "_UnmountVol",
    0xA00F: "_MountVol",
    0xA010: "_Allocate",
    0xA011: "_GetEOF",
    0xA012: "_SetEOF",
    0xA013: "_FlushVol",
    0xA014: "_GetVol",
    0xA015: "_SetVol",
    0xA017: "_Eject",
    0xA018: "_GetFPos",
    # Memory Manager: repo-confirmed by name in this project's own ROM
    # disassembly, not from general recollection.
    0xA029: "_HLock",       # docs/BUG_low_ram_corruption_post_dafb.md: "_HLock = 0xA029"
    0xA02E: "_BlockMove",   # docs/bug_b_atrap_divergence.md: "4081BFDE  A02E  _BlockMove"
    # OS Utilities: cross-checked against tools/jtag_repl.tcl.
    0xA082: "_DTInstall",

    # ── Toolbox traps ($A800-$ABFF after masking) ──────────────────────
    # Cross-checked against tools/jtag_repl.tcl.
    0xA815: "_SCSIDispatch",
    0xA860: "_WaitNextEvent",
    0xA86E: "_InitGraf",
    0xA96E: "_Dequeue",
    0xA96F: "_Enqueue",
    0xA9C9: "_SysError",
    # High-confidence, long-standing public convention (not in
    # jtag_repl.tcl's table, not sourced from this project's own notes).
    0xA850: "_InitCursor",
    0xA851: "_SetCursor",
    0xA970: "_GetNextEvent",
    0xA9A0: "_GetResource",
    0xA9C8: "_SysBeep",
    0xA9F4: "_ExitToShell",
    0xA9FF: "_Debugger",
    0xABFF: "_DebugStr",
}


def lookup(opword: int) -> str | None:
    """Resolve a 16-bit A-line opcode word to a trap name, or None.

    Masks flag bits per the OS-vs-Toolbox convention documented at the
    top of this file before looking the trap number up in ``ATRAPS``.
    Returns None for anything not in the ``$A000-$AFFF`` A-line space at
    all, and also for any A-line opword whose (masked) trap number simply
    isn't in this deliberately-incomplete table.
    """
    opword &= 0xFFFF
    if (opword & 0xF000) != 0xA000:
        return None
    if opword & TOOLBOX_BIT:
        canonical = 0xA800 | (opword & TOOLBOX_TRAP_MASK)
    else:
        canonical = 0xA000 | (opword & OS_TRAP_MASK)
    return ATRAPS.get(canonical)


def by_name(name: str) -> int | None:
    """Resolve a trap name to its canonical A-line opcode word, or None.

    Case-insensitive, and the leading underscore is optional, so all of
    ``_SCSIDispatch`` / ``SCSIDispatch`` / ``scsidispatch`` work — a human
    typing ``monitor atrap arm scsidispatch`` should not have to remember
    which spelling this table used."""
    key = name.lower().lstrip("_")
    for opword, trap in ATRAPS.items():
        if trap.lower().lstrip("_") == key:
            return opword
    return None


def names() -> list[str]:
    """Every known trap name, sorted — for shell/monitor completion."""
    return sorted(ATRAPS.values())


# ── Deliberately OMITTED (do not add without independent confirmation) ──
#
# Requested-but-not-included names, and why:
#
#   _GetVolInfo..._GetFPos block above IS included (high confidence);
#   everything below this line genuinely was NOT, because I could not
#   reach the same confidence bar for BOTH the exact trap number and
#   which of the two trap spaces (OS vs Toolbox) it lives in:
#
#   _InitQueue      — Memory/OS Utilities era call; number not confirmed.
#   _NewHandle      — Memory Manager; multiple candidate numbers recalled
#                      inconsistently (plain OS-trap vs newer Toolbox
#                      "clean" trap), not resolved here.
#   _NewPtr         — same issue as _NewHandle.
#   _DisposHandle   — same issue as _NewHandle.
#   _HUnlock        — _HLock (0xA029) IS confirmed (see above); I did NOT
#                      independently confirm _HUnlock actually sits at the
#                      adjacent 0xA02A rather than elsewhere, so it is
#                      left out rather than assumed-adjacent.
#   _VInstall       — Vertical Retrace Manager; number not confirmed.
#   _VRemove        — same issue as _VInstall.
#   _SlotVInstall   — NuBus slot manager; number not confirmed.
#
# A human extending this table should pull real values from a Mac OS
# Trap.a / SysTraps.h reference (or from a MAME/Musashi trap-name table)
# rather than trusting an LLM's unaided recollection for these.
