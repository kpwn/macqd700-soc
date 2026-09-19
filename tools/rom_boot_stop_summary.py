#!/usr/bin/env python3
"""Summarize the final useful stop point from a tb_rom_boot last-N trace.

The ROM boot harness writes a compact per-cycle ring buffer when
`+lastn_trace=<n>` and `+lastn_trace_path=<path>` are enabled.  This tool
turns that log into a short, grep-free summary focused on the last sample
that still carries exception/vector information.

Usage:

    python3 tools/rom_boot_stop_summary.py \
        build/sim/rom_boot_stop_summary_lastn.log

Optional `--log` and `--trace` arguments are accepted so the report can
include the matching harness stderr log and committed-instruction trace
paths used for the run.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


HEADER_RE = re.compile(
    r"^# rom-boot last-N-cycle trace capacity=(?P<capacity>\d+) "
    r"samples=(?P<samples>\d+) reason=(?P<reason>.*)$"
)
SAMPLE_RE = re.compile(
    r"^lastn\[(?P<idx>\d+)\] sim=(?P<sim>\d+) committed=(?P<committed>\d+) "
    r"dbg_last_pc=0x(?P<dbg_last_pc>[0-9a-fA-F]+) dbg_pc=0x(?P<dbg_pc>[0-9a-fA-F]+) "
    r"rob_v=(?P<rob_v>\d+) rob_c=(?P<rob_c>\d+) rob_pc=0x(?P<rob_pc>[0-9a-fA-F]+) "
    r"rob_vec=(?P<rob_vec>\d+) (?:overlay=\d+/\d+ )?commit_exc_wait=(?P<commit_exc_wait>\d+) "
    r"commit_take_exc=(?P<commit_take_exc>\d+) exc_state=(?P<exc_state>\d+) "
    r"exc_vec=(?P<exc_vec>\d+) exc_fault_pc=0x(?P<exc_fault_pc>[0-9a-fA-F]+) "
    r"exc_a7=0x(?P<exc_a7>[0-9a-fA-F]+)\b"
)
IF_RE = re.compile(
    r"\bif=(?P<req>\d+)/0x(?P<addr>[0-9a-fA-F]+)/"
    r"(?P<rvalid>\d+)/(?P<pending>\d+)\b"
)
DAXI_AR_RE = re.compile(
    r"\bdaxi_ar=(?P<valid>\d+)/(?P<ready>\d+)/0x(?P<addr>[0-9a-fA-F]+)\b"
)
DAXI_R_RE = re.compile(
    r"\bdaxi_r=(?P<valid>\d+)/(?P<ready>\d+)/(?P<resp>\d+)\b"
)
DAXI_AW_RE = re.compile(
    r"\bdaxi_aw=(?P<valid>\d+)/(?P<ready>\d+)/0x(?P<addr>[0-9a-fA-F]+)\b"
)
DAXI_W_RE = re.compile(r"\bdaxi_w=(?P<valid>\d+)/(?P<ready>\d+)\b")
DAXI_B_RE = re.compile(
    r"\bdaxi_b=(?P<valid>\d+)/(?P<ready>\d+)/(?P<resp>\d+)\b"
)
OVERLAY_RE = re.compile(r"\boverlay=(?P<active>\d+)/(?P<via1>\d+)\b")
TOKEN_RE = re.compile(r"\b(?P<name>[A-Za-z0-9_]+)=(?P<value>\S+)")


@dataclass(frozen=True)
class Header:
    capacity: int
    samples: int
    reason: str


@dataclass(frozen=True)
class Sample:
    idx: int
    sim: int
    committed: int
    dbg_last_pc: int
    dbg_pc: int
    rob_v: int
    rob_c: int
    rob_pc: int
    rob_vec: int
    commit_exc_wait: int
    commit_take_exc: int
    exc_state: int
    exc_vec: int
    exc_fault_pc: int
    exc_a7: int
    overlay_active: Optional[int] = None
    via1_overlay_live: Optional[int] = None
    daxi_ar_valid: Optional[int] = None
    daxi_ar_ready: Optional[int] = None
    daxi_ar_addr: Optional[int] = None
    daxi_r_valid: Optional[int] = None
    daxi_r_ready: Optional[int] = None
    daxi_r_resp: Optional[int] = None
    daxi_aw_valid: Optional[int] = None
    daxi_aw_ready: Optional[int] = None
    daxi_aw_addr: Optional[int] = None
    daxi_w_valid: Optional[int] = None
    daxi_w_ready: Optional[int] = None
    daxi_b_valid: Optional[int] = None
    daxi_b_ready: Optional[int] = None
    daxi_b_resp: Optional[int] = None
    if_req: Optional[int] = None
    if_addr: Optional[int] = None
    if_rvalid: Optional[int] = None
    if_pending: Optional[int] = None
    dcache_state: Optional[int] = None
    dcache_beat: Optional[int] = None
    lsu_state: Optional[int] = None
    lsu_ea: Optional[int] = None
    lsu_commit_store: Optional[int] = None
    lsu_flush: Optional[int] = None
    dc_req: Optional[int] = None
    dc_is_write: Optional[int] = None
    dc_addr: Optional[int] = None
    dc_rvalid: Optional[int] = None
    dc_bvalid: Optional[int] = None
    dmmu_req_valid: Optional[int] = None
    dmmu_lsu_ready: Optional[int] = None
    dmmu_need_walk: Optional[int] = None
    dmmu_w_busy: Optional[int] = None
    dmmu_req_pending: Optional[int] = None
    dmmu_fault: Optional[int] = None
    dmmu_va: Optional[int] = None
    dmmu_pa: Optional[int] = None
    walk_state: Optional[int] = None
    walk_last_fault: Optional[int] = None
    walk_last_ok: Optional[int] = None


@dataclass(frozen=True)
class Diagnosis:
    label: str
    region: str
    evidence: str

    def compact(self) -> str:
        return f"diagnosis={self.label} region={self.region} evidence={self.evidence}"


def parse_u32(text: str) -> int:
    return int(text[2:] if text.lower().startswith("0x") else text, 16)


def parse_line_tokens(line: str) -> dict[str, str]:
    return {m.group("name"): m.group("value") for m in TOKEN_RE.finditer(line)}


def parse_int_token(text: str) -> int:
    return parse_u32(text) if text.lower().startswith("0x") else int(text, 10)


def parse_slash_ints(tokens: dict[str, str], name: str) -> list[int]:
    raw = tokens.get(name)
    if raw is None:
        return []
    try:
        return [parse_int_token(part) for part in raw.split("/")]
    except ValueError:
        return []


def parse_trace(path: Path) -> tuple[Optional[Header], list[Sample]]:
    header: Optional[Header] = None
    samples: list[Sample] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                m = HEADER_RE.match(line)
                if m:
                    header = Header(
                        capacity=int(m.group("capacity")),
                        samples=int(m.group("samples")),
                        reason=m.group("reason"),
                    )
                continue
            m = SAMPLE_RE.match(line)
            if not m:
                continue
            fields = dict(
                idx=int(m.group("idx")),
                sim=int(m.group("sim")),
                committed=int(m.group("committed")),
                dbg_last_pc=parse_u32(m.group("dbg_last_pc")),
                dbg_pc=parse_u32(m.group("dbg_pc")),
                rob_v=int(m.group("rob_v")),
                rob_c=int(m.group("rob_c")),
                rob_pc=parse_u32(m.group("rob_pc")),
                rob_vec=int(m.group("rob_vec")),
                commit_exc_wait=int(m.group("commit_exc_wait")),
                commit_take_exc=int(m.group("commit_take_exc")),
                exc_state=int(m.group("exc_state")),
                exc_vec=int(m.group("exc_vec")),
                exc_fault_pc=parse_u32(m.group("exc_fault_pc")),
                exc_a7=parse_u32(m.group("exc_a7")),
            )
            m_overlay = OVERLAY_RE.search(line)
            if m_overlay:
                fields.update(
                    overlay_active=int(m_overlay.group("active")),
                    via1_overlay_live=int(m_overlay.group("via1")),
                )
            else:
                tokens = parse_line_tokens(line)
                overlay = parse_slash_ints(tokens, "overlay")
                if len(overlay) >= 2:
                    fields.update(
                        overlay_active=overlay[0],
                        via1_overlay_live=overlay[1],
                    )
            m_daxi_ar = DAXI_AR_RE.search(line)
            if m_daxi_ar:
                fields.update(
                    daxi_ar_valid=int(m_daxi_ar.group("valid")),
                    daxi_ar_ready=int(m_daxi_ar.group("ready")),
                    daxi_ar_addr=parse_u32(m_daxi_ar.group("addr")),
                )
            m_daxi_r = DAXI_R_RE.search(line)
            if m_daxi_r:
                fields.update(
                    daxi_r_valid=int(m_daxi_r.group("valid")),
                    daxi_r_ready=int(m_daxi_r.group("ready")),
                    daxi_r_resp=int(m_daxi_r.group("resp")),
                )
            m_daxi_aw = DAXI_AW_RE.search(line)
            if m_daxi_aw:
                fields.update(
                    daxi_aw_valid=int(m_daxi_aw.group("valid")),
                    daxi_aw_ready=int(m_daxi_aw.group("ready")),
                    daxi_aw_addr=parse_u32(m_daxi_aw.group("addr")),
                )
            m_daxi_w = DAXI_W_RE.search(line)
            if m_daxi_w:
                fields.update(
                    daxi_w_valid=int(m_daxi_w.group("valid")),
                    daxi_w_ready=int(m_daxi_w.group("ready")),
                )
            m_daxi_b = DAXI_B_RE.search(line)
            if m_daxi_b:
                fields.update(
                    daxi_b_valid=int(m_daxi_b.group("valid")),
                    daxi_b_ready=int(m_daxi_b.group("ready")),
                    daxi_b_resp=int(m_daxi_b.group("resp")),
                )
            m_if = IF_RE.search(line)
            if m_if:
                fields.update(
                    if_req=int(m_if.group("req")),
                    if_addr=parse_u32(m_if.group("addr")),
                    if_rvalid=int(m_if.group("rvalid")),
                    if_pending=int(m_if.group("pending")),
                )
            tokens = parse_line_tokens(line)
            dcache = parse_slash_ints(tokens, "dcache")
            if len(dcache) >= 4:
                fields.update(dcache_state=dcache[0], dcache_beat=dcache[3])
            lsu = parse_slash_ints(tokens, "lsu")
            if len(lsu) >= 14:
                fields.update(
                    lsu_state=lsu[3],
                    lsu_ea=lsu[6],
                    lsu_commit_store=lsu[12],
                    lsu_flush=lsu[13],
                )
            dc = parse_slash_ints(tokens, "dc")
            if len(dc) >= 8:
                fields.update(
                    dc_req=dc[0],
                    dc_is_write=dc[1],
                    dc_addr=dc[2],
                    dc_rvalid=dc[5],
                    dc_bvalid=dc[7],
                )
            dmmu = parse_slash_ints(tokens, "dmmu")
            if len(dmmu) >= 16:
                fields.update(
                    dmmu_req_valid=dmmu[0],
                    dmmu_lsu_ready=dmmu[2],
                    dmmu_need_walk=dmmu[3],
                    dmmu_w_busy=dmmu[4],
                    dmmu_req_pending=dmmu[5],
                    dmmu_fault=dmmu[10],
                    dmmu_va=dmmu[14],
                    dmmu_pa=dmmu[15],
                )
            walk = parse_slash_ints(tokens, "walk")
            if len(walk) >= 15:
                fields.update(
                    walk_state=walk[0],
                    walk_last_fault=walk[12],
                    walk_last_ok=walk[13],
                )
            samples.append(Sample(**fields))
    return header, samples


def fmt_pc(value: int) -> str:
    return f"0x{value:08x}"


def fmt_opt_pc(value: Optional[int]) -> str:
    return "n/a" if value is None else fmt_pc(value)


def fmt_opt(value: Optional[int]) -> str:
    return "n/a" if value is None else str(value)


def addr_region(addr: Optional[int]) -> str:
    if addr is None:
        return "none"
    if 0x40000000 <= addr <= 0x40FFFFFF:
        return "rom"
    if addr < 0x10000000:
        return "ram-low"
    if 0x50000000 <= addr <= 0x50FFFFFF:
        return "q700-io"
    if 0xF9000000 <= addr <= 0xF91FFFFF:
        return "vram"
    if 0xF9800000 <= addr <= 0xF9800FFF:
        return "dafb"
    if addr >= 0xF0000000:
        return "io-high"
    return "other"


def is_interesting(sample: Sample) -> bool:
    return any(
        (
            sample.rob_vec != 0,
            sample.exc_vec != 0,
            sample.commit_take_exc != 0,
            sample.commit_exc_wait != 0,
            sample.exc_state != 0,
            sample.rob_v != 0,
            sample.rob_c != 0,
        )
    )


def pick_focus(samples: list[Sample]) -> Sample:
    for sample in reversed(samples):
        if is_interesting(sample):
            return sample
    return samples[-1]


def render_sample(prefix: str, sample: Sample) -> str:
    line = (
        f"{prefix}: idx={sample.idx} sim={sample.sim} committed={sample.committed} "
        f"dbg_last_pc={fmt_pc(sample.dbg_last_pc)} dbg_pc={fmt_pc(sample.dbg_pc)} "
        f"rob_v={sample.rob_v} rob_c={sample.rob_c} rob_pc={fmt_pc(sample.rob_pc)} "
        f"rob_vec={sample.rob_vec} commit_exc_wait={sample.commit_exc_wait} "
        f"commit_take_exc={sample.commit_take_exc} exc_state={sample.exc_state} "
        f"exc_vec={sample.exc_vec} exc_fault_pc={fmt_pc(sample.exc_fault_pc)} "
        f"exc_a7={fmt_pc(sample.exc_a7)}"
    )
    if sample.overlay_active is not None:
        line += (
            f" overlay={sample.overlay_active}/"
            f"{fmt_opt(sample.via1_overlay_live)}"
        )
    if sample.if_req is not None:
        line += (
            f" if={sample.if_req}/{fmt_opt_pc(sample.if_addr)}/"
            f"{fmt_opt(sample.if_rvalid)}/{fmt_opt(sample.if_pending)}"
        )
    if sample.daxi_ar_valid is not None:
        line += (
            f" daxi_ar={sample.daxi_ar_valid}/{fmt_opt(sample.daxi_ar_ready)}/"
            f"{fmt_opt_pc(sample.daxi_ar_addr)}"
        )
    if sample.daxi_r_valid is not None:
        line += (
            f" daxi_r={sample.daxi_r_valid}/{fmt_opt(sample.daxi_r_ready)}/"
            f"{fmt_opt(sample.daxi_r_resp)}"
        )
    if sample.daxi_aw_valid is not None:
        line += (
            f" daxi_aw={sample.daxi_aw_valid}/{fmt_opt(sample.daxi_aw_ready)}/"
            f"{fmt_opt_pc(sample.daxi_aw_addr)}"
        )
    if sample.daxi_w_valid is not None:
        line += f" daxi_w={sample.daxi_w_valid}/{fmt_opt(sample.daxi_w_ready)}"
    if sample.daxi_b_valid is not None:
        line += (
            f" daxi_b={sample.daxi_b_valid}/{fmt_opt(sample.daxi_b_ready)}/"
            f"{fmt_opt(sample.daxi_b_resp)}"
        )
    return line


def tail_run_length(samples: list[Sample], attr: str) -> int:
    if not samples:
        return 0
    value = getattr(samples[-1], attr)
    count = 0
    for sample in reversed(samples):
        if getattr(sample, attr) != value:
            break
        count += 1
    return count


def pick_last_ifetch(samples: list[Sample]) -> Optional[Sample]:
    for sample in reversed(samples):
        if sample.if_req or sample.if_pending or sample.if_rvalid:
            return sample
    return None


def render_tail_activity(samples: list[Sample]) -> str:
    tail = samples[-1]
    last_ifetch = pick_last_ifetch(samples)
    last_if_addr = last_ifetch.if_addr if last_ifetch else None
    last_if_req = last_ifetch.if_req if last_ifetch else None
    last_if_pending = last_ifetch.if_pending if last_ifetch else None
    return (
        "[rom-boot-stop-summary] tail-activity: "
        f"same_dbg_pc={tail_run_length(samples, 'dbg_pc')} "
        f"same_dbg_last_pc={tail_run_length(samples, 'dbg_last_pc')} "
        f"same_committed={tail_run_length(samples, 'committed')} "
        f"tail_overlay={fmt_opt(tail.overlay_active)}/{fmt_opt(tail.via1_overlay_live)} "
        f"tail_if={fmt_opt(tail.if_req)}/{fmt_opt_pc(tail.if_addr)}/"
        f"{fmt_opt(tail.if_rvalid)}/{fmt_opt(tail.if_pending)} "
        f"tail_daxi_ar={fmt_opt(tail.daxi_ar_valid)}/{fmt_opt(tail.daxi_ar_ready)}/"
        f"{fmt_opt_pc(tail.daxi_ar_addr)} "
        f"tail_daxi_r={fmt_opt(tail.daxi_r_valid)}/{fmt_opt(tail.daxi_r_ready)}/"
        f"{fmt_opt(tail.daxi_r_resp)} "
        f"tail_daxi_aw={fmt_opt(tail.daxi_aw_valid)}/{fmt_opt(tail.daxi_aw_ready)}/"
        f"{fmt_opt_pc(tail.daxi_aw_addr)} "
        f"tail_daxi_w={fmt_opt(tail.daxi_w_valid)}/{fmt_opt(tail.daxi_w_ready)} "
        f"tail_daxi_b={fmt_opt(tail.daxi_b_valid)}/{fmt_opt(tail.daxi_b_ready)}/"
        f"{fmt_opt(tail.daxi_b_resp)} "
        f"last_active_if={fmt_opt(last_if_req)}/{fmt_opt_pc(last_if_addr)}/"
        f"{fmt_opt(last_if_pending)}"
    )


def diagnose_stall(samples: list[Sample], focus: Sample, tail: Sample) -> Diagnosis:
    same_committed = tail_run_length(samples, "committed")
    same_dbg_pc = tail_run_length(samples, "dbg_pc")
    same_dbg_last_pc = tail_run_length(samples, "dbg_last_pc")
    active_addr = (
        tail.daxi_ar_addr if tail.daxi_ar_valid else
        tail.daxi_aw_addr if tail.daxi_aw_valid else
        tail.if_addr if tail.if_req or tail.if_pending else
        tail.dc_addr if tail.dc_req else
        tail.lsu_ea
    )
    region = addr_region(active_addr)

    if (
        focus.exc_vec
        or focus.rob_vec
        or focus.commit_exc_wait
        or focus.commit_take_exc
        or focus.exc_state
    ):
        return Diagnosis(
            "core-exception",
            region,
            f"rob_vec={focus.rob_vec},exc_vec={focus.exc_vec},exc_state={focus.exc_state}",
        )

    if tail.daxi_ar_valid and tail.daxi_ar_ready == 0:
        return Diagnosis(
            "bus-read-address",
            region,
            f"arvalid=1,arready=0,addr={fmt_opt_pc(tail.daxi_ar_addr)}",
        )
    if tail.daxi_r_ready and not tail.daxi_r_valid:
        return Diagnosis(
            "bus-read-data",
            region,
            f"rready=1,rvalid=0,araddr={fmt_opt_pc(tail.daxi_ar_addr)}",
        )
    if tail.daxi_aw_valid and tail.daxi_aw_ready == 0:
        return Diagnosis(
            "bus-write-address",
            region,
            f"awvalid=1,awready=0,addr={fmt_opt_pc(tail.daxi_aw_addr)}",
        )
    if tail.daxi_w_valid and tail.daxi_w_ready == 0:
        return Diagnosis("bus-write-data", region, "wvalid=1,wready=0")
    if tail.daxi_b_ready and not tail.daxi_b_valid:
        return Diagnosis(
            "bus-write-response",
            region,
            f"bready=1,bvalid=0,awaddr={fmt_opt_pc(tail.daxi_aw_addr)}",
        )

    if (tail.if_req or tail.if_pending) and not tail.if_rvalid:
        return Diagnosis(
            "ifetch-wait",
            addr_region(tail.if_addr),
            f"if={fmt_opt(tail.if_req)}/{fmt_opt_pc(tail.if_addr)}/"
            f"{fmt_opt(tail.if_rvalid)}/{fmt_opt(tail.if_pending)}",
        )

    if tail.dmmu_need_walk or tail.dmmu_w_busy or tail.dmmu_req_pending:
        return Diagnosis(
            "core-dmmu-walk",
            addr_region(tail.dmmu_va),
            f"need={fmt_opt(tail.dmmu_need_walk)},wbusy={fmt_opt(tail.dmmu_w_busy)},"
            f"pending={fmt_opt(tail.dmmu_req_pending)},walk_state={fmt_opt(tail.walk_state)}",
        )
    if tail.dmmu_fault or tail.walk_last_fault:
        return Diagnosis(
            "core-dmmu-fault",
            addr_region(tail.dmmu_va),
            f"fault={fmt_opt(tail.dmmu_fault)},walk_fault={fmt_opt(tail.walk_last_fault)},"
            f"va={fmt_opt_pc(tail.dmmu_va)}",
        )
    if tail.dc_req and not (tail.daxi_ar_valid or tail.daxi_aw_valid or tail.daxi_w_valid):
        return Diagnosis(
            "core-dcache-request",
            addr_region(tail.dc_addr),
            f"dc_req=1,wr={fmt_opt(tail.dc_is_write)},dcache_state={fmt_opt(tail.dcache_state)}",
        )
    if tail.lsu_state and same_committed > 1:
        return Diagnosis(
            "core-lsu",
            addr_region(tail.lsu_ea),
            f"lsu_state={fmt_opt(tail.lsu_state)},ea={fmt_opt_pc(tail.lsu_ea)}",
        )
    if same_committed == len(samples) and same_dbg_pc == len(samples):
        return Diagnosis(
            "core-idle-or-deadlock",
            region,
            f"committed_stable={same_committed},dbg_pc_stable={same_dbg_pc}",
        )
    if same_dbg_last_pc > 1:
        return Diagnosis(
            "rom-loop-or-retire-stall",
            region,
            f"same_dbg_last_pc={same_dbg_last_pc},same_committed={same_committed}",
        )
    return Diagnosis(
        "progressing",
        region,
        f"same_committed={same_committed},same_dbg_pc={same_dbg_pc},"
        f"same_dbg_last_pc={same_dbg_last_pc}",
    )


def render_compact_summary(
    reason: str,
    samples: list[Sample],
    focus: Sample,
    tail: Sample,
    diagnosis: Optional[Diagnosis] = None,
) -> str:
    if diagnosis is None:
        diagnosis = diagnose_stall(samples, focus, tail)
    return (
        f"[rom-boot-stop-summary] summary: reason={reason} "
        f"{diagnosis.compact()} "
        f"focus_committed={focus.committed} "
        f"focus_dbg_last_pc={fmt_pc(focus.dbg_last_pc)} "
        f"focus_dbg_pc={fmt_pc(focus.dbg_pc)} "
        f"focus_rob_pc={fmt_pc(focus.rob_pc)} "
        f"focus_rob_vec={focus.rob_vec} "
        f"focus_exc_vec={focus.exc_vec} "
        f"focus_exc_state={focus.exc_state} "
        f"tail_committed={tail.committed} "
        f"tail_dbg_last_pc={fmt_pc(tail.dbg_last_pc)} "
        f"tail_dbg_pc={fmt_pc(tail.dbg_pc)} "
        f"tail_rob_pc={fmt_pc(tail.rob_pc)} "
        f"tail_rob_vec={tail.rob_vec} "
        f"tail_exc_vec={tail.exc_vec} "
        f"tail_exc_state={tail.exc_state} "
        f"tail_same_dbg_pc={tail_run_length(samples, 'dbg_pc')} "
        f"tail_same_dbg_last_pc={tail_run_length(samples, 'dbg_last_pc')} "
        f"tail_same_committed={tail_run_length(samples, 'committed')} "
        f"tail_overlay={fmt_opt(tail.overlay_active)}/{fmt_opt(tail.via1_overlay_live)} "
        f"tail_if={fmt_opt(tail.if_req)}/{fmt_opt_pc(tail.if_addr)}/"
        f"{fmt_opt(tail.if_rvalid)}/{fmt_opt(tail.if_pending)} "
        f"tail_daxi_ar={fmt_opt(tail.daxi_ar_valid)}/{fmt_opt(tail.daxi_ar_ready)}/"
        f"{fmt_opt_pc(tail.daxi_ar_addr)} "
        f"tail_daxi_r={fmt_opt(tail.daxi_r_valid)}/{fmt_opt(tail.daxi_r_ready)}/"
        f"{fmt_opt(tail.daxi_r_resp)} "
        f"tail_daxi_aw={fmt_opt(tail.daxi_aw_valid)}/{fmt_opt(tail.daxi_aw_ready)}/"
        f"{fmt_opt_pc(tail.daxi_aw_addr)} "
        f"tail_daxi_w={fmt_opt(tail.daxi_w_valid)}/{fmt_opt(tail.daxi_w_ready)} "
        f"tail_daxi_b={fmt_opt(tail.daxi_b_valid)}/{fmt_opt(tail.daxi_b_ready)}/"
        f"{fmt_opt(tail.daxi_b_resp)}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("lastn", type=Path, help="tb_rom_boot last-N log path")
    ap.add_argument("--log", type=Path, help="matching harness stderr log")
    ap.add_argument("--trace", type=Path, help="matching committed trace log")
    ap.add_argument(
        "--compact",
        action="store_true",
        help="print a single-line CI/handoff summary instead of the expanded report",
    )
    args = ap.parse_args()

    header, samples = parse_trace(args.lastn)
    if not samples:
        print(f"[rom-boot-stop-summary] no samples found in {args.lastn}", file=sys.stderr)
        return 1

    focus = pick_focus(samples)
    tail = samples[-1]

    print(f"[rom-boot-stop-summary] lastn={args.lastn}")
    if args.log:
        print(f"[rom-boot-stop-summary] log={args.log}")
    if args.trace:
        print(f"[rom-boot-stop-summary] trace={args.trace}")
    if header:
        print(
            f"[rom-boot-stop-summary] header: capacity={header.capacity} "
            f"samples={header.samples} reason={header.reason}"
        )
    else:
        print("[rom-boot-stop-summary] header: not found")

    if args.compact:
        print(render_compact_summary(header.reason if header else "unknown", samples, focus, tail))
        return 0

    print(render_sample("[rom-boot-stop-summary] focus", focus))
    if tail != focus:
        print(render_sample("[rom-boot-stop-summary] tail ", tail))
    diagnosis = diagnose_stall(samples, focus, tail)
    print(f"[rom-boot-stop-summary] diagnosis: {diagnosis.compact()}")
    print(render_tail_activity(samples))
    return 0


if __name__ == "__main__":
    sys.exit(main())
