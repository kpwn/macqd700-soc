#!/usr/bin/env python3
"""P6W mutation check for the hot WRITE door (`make tb-l2c-dw-mutants`).

Each mutant is ONE deliberately broken mechanism of the write door, applied
to a scratch copy of rtl/soc.  The DW_EN=1 tb_l2c build must FAIL on every
one of them -- a mutant that passes means the suite cannot see that
mechanism, and the "green" of `make tb-l2c-dw` says nothing about it.

A mutant is 'CAUGHT' if the run exits non-zero, prints a FAIL, or dies on an
RTL assertion; the script reports which, so a catch by assertion is
distinguishable from a catch by a data/ordering check.

Usage: tools/l2c_dw_mutants.py [name ...]   (default: all)
Env:   L2C_MUT_OPS (scoreboard ops per mutant run, default 5000)
"""
import os, shutil, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (name, file, old, new, what it breaks)
MUTANTS = [
    ("hot_write_marked_read", "l2c_ctrl.v",
     "a_is_write <= cur_is_write_c;", "a_is_write <= write_sel;",
     "a hot write enters the pipeline flagged as a READ"),
    ("gather_mask_unqualified", "l2c_ctrl.v",
     "wire       cur_gfull_c = (gst == G_FULL) && !dw_write_sel;",
     "wire       cur_gfull_c = (gst == G_FULL);",
     "a hot write accepted while a cold gathered line waits takes the full-line masks"),
    ("cold_aw_advances_on_hot", "l2c_ctrl.v",
     "(door_take_c && !read_sel && !dw_write_sel && !flw_avail_c)",
     "(door_take_c && !read_sel && !flw_avail_c)",
     "a hot write accept advances the COLD AW tracker"),
    ("b_misrouted_to_s_axi", "l2c.v",
     "wire mshr_b_is_cold_c = (DW_PORT_EN == 0) || mshr_rsp_is_lsu_c;\n    wire hit_b_is_cold_c  = (DW_PORT_EN == 0) || hit_rsp_is_lsu_c;",
     "wire mshr_b_is_cold_c = 1'b1;\n    wire hit_b_is_cold_c  = 1'b1;",
     "the s_axi B arbiter also grabs hot-door write completions"),
    ("dw_src_tagged_lsu", "l2c_ctrl.v",
     "wire [SRC_BITS-1:0]   cur_src = (dh_read_sel || dw_write_sel) ? SRCV_DH",
     "wire [SRC_BITS-1:0]   cur_src = dh_read_sel ? SRCV_DH",
     "hot writes carry the LSU source tag"),
    ("dw_wready_ignores_set_haz", "l2c_ctrl.v",
     "assign dw_wready = dw_write_sel && !rst_busy && !is_bypass_c && accept_slot_c && !set_haz_c;",
     "assign dw_wready = dw_write_sel && !rst_busy && !is_bypass_c && accept_slot_c;",
     "the W handshake is decoupled from the accept (a beat can be taken but not accepted)"),
    # EQUIVALENT MUTANT, kept on purpose and expected to SURVIVE.  It lets a
    # hot WRITE follower overlap a same-line READ leader in the lookup
    # pipeline.  That pairing is safe: a read leader writes no tag or data on
    # a hit, and on a miss the follower resolves through mshr_lu_hit (live
    # MSHR state, not its stale array view) and merges or retries (Rule 2).
    # The hazard that matters -- a READ follower of a WRITE leader -- is keyed
    # on the leader's a_is_write/q_is_write, which hot_write_marked_read
    # covers.  If this mutant is ever CAUGHT, that argument is wrong: stop.
    ("set_haz_ignores_hot_write", "l2c_ctrl.v",
     "wire set_haz_c = (a_same_set_c && !(a_same_line_c && !a_is_write && !cur_is_write_c)) ||\n                     (q_same_set_c && !(q_same_line_c && !q_is_write && !cur_is_write_c));",
     "wire set_haz_c = (a_same_set_c && !(a_same_line_c && !a_is_write && !write_sel)) ||\n                     (q_same_set_c && !(q_same_line_c && !q_is_write && !write_sel));",
     "a hot write FOLLOWER may overlap a same-line read leader in the pipeline"),
    ("dw_favor_stuck_on_reads", "l2c_ctrl.v",
     "dw_favor <= !dw_write_sel;", "dw_favor <= 1'b0;",
     "hot reads always beat hot writes"),
    ("no_hazard_yield", "l2c_ctrl.v",
     "hot_haz_q <= (DW_PORT_EN != 0) && hot_take_c", "hot_haz_q <= 1'b0 && hot_take_c",
     "hot-class cycles refused by set_haz are not yielded to the cold path"),
    ("guard_forwards_rom", "dh_write_window_guard.v",
     "wire in_ram = ((s_awaddr >> lg2) == 32'd0);",
     "wire in_ram = ((s_awaddr >> lg2) == 32'd0) || s_aw_is_rom;",
     "the write guard forwards ROM-mirror writes instead of dropping them"),
    ("guard_forwards_everything", "dh_write_window_guard.v",
     "wire in_ram = ((s_awaddr >> lg2) == 32'd0);", "wire in_ram = 1'b1;",
     "no write guard at all"),
    ("guard_local_b_not_ordered", "dh_write_window_guard.v",
     "wire fwd_quiet   = (b_owed == {CW{1'b0}}) && (w_owed == {CW{1'b0}});",
     "wire fwd_quiet   = (w_owed == {CW{1'b0}});",
     "a locally answered burst may overtake a forwarded one's B"),
    ("guard_ignores_cpu_reset", "dh_write_window_guard.v",
     "wire quiesce     = abandon || pad_q;", "wire quiesce     = 1'b0;",
     "no abandoned-burst padding / B swallowing on a CPU reset"),
    ("posted_b_at_aw", "dh_write_window_guard.v",
     "    assign s_bvalid  = !quiesce && (l_resp ? 1'b1 : m_bvalid);\n    assign s_bid     = l_resp ? l_id : m_bid;\n    assign s_bresp   = l_resp ? (l_rom ? 2'b00 : 2'b11) : m_bresp;\n    assign m_bready  = quiesce ? !l_resp : (l_resp ? 1'b0 : s_bready);",
     "    reg mut_pb_v; reg [ID_WIDTH-1:0] mut_pb_id;\n"
     "    always @(posedge clk) if (rst) mut_pb_v <= 1'b0; else if (m_awvalid && m_awready) begin mut_pb_v <= 1'b1; mut_pb_id <= s_awid; end else if (s_bready) mut_pb_v <= 1'b0;\n"
     "    assign s_bvalid  = !quiesce && (l_resp ? 1'b1 : mut_pb_v);\n    assign s_bid     = l_resp ? l_id : mut_pb_id;\n    assign s_bresp   = l_resp ? (l_rom ? 2'b00 : 2'b11) : 2'b00;\n    assign m_bready  = l_resp ? 1'b0 : 1'b1;",
     "a POSTED write: B answered when the AW is accepted, before the data is in L2"),
]

VFLAGS = ["--cc", "--exe", "--build", "--assert", "--x-assign", "fast", "--x-initial", "fast", "-O3",
          "-Wall", "-Wno-UNUSED", "-Wno-UNDRIVEN", "-Wno-PINCONNECTEMPTY", "-Wno-PINMISSING",
          "-Wno-DECLFILENAME", "-Wno-CASEINCOMPLETE", "-Wno-WIDTHTRUNC", "-Wno-WIDTHEXPAND",
          "-Wno-SELRANGE", "-Wno-fatal"]
RTL = ["ifetch_window_guard.v", "dh_write_window_guard.v", "l2c_pri8.v", "l2c_reset.v", "l2c_tags.v",
       "l2c_data.v", "l2c_victim_sel.v", "l2c_mshr.v", "l2c_victim.v", "l2c_bypass.v", "l2c_ctrl.v", "l2c.v"]

EXPECTED_SURVIVORS = {"set_haz_ignores_hot_write"}

def run_one(name, fname, old, new, what, work):
    src = os.path.join(work, "rtl_" + name)
    shutil.copytree(os.path.join(ROOT, "rtl", "soc"), src)
    p = os.path.join(src, fname)
    text = open(p).read()
    n = text.count(old)
    if n != 1:
        return ("STALE", f"anchor matched {n} times -- the mutant no longer applies; update it")
    open(p, "w").write(text.replace(old, new))
    mdir = os.path.join(work, "obj_" + name)
    cmd = (["verilator"] + VFLAGS + ["-I" + src, "-Mdir", mdir, "--top-module", "tb_l2c",
           "-GDH_EN=1", "-GDW_EN=1"] + [os.path.join(src, f) for f in RTL] +
           [os.path.join(ROOT, "tb", "tb_l2c.v"), os.path.join(ROOT, "tb", "tb_l2c.cpp"),
            "-CFLAGS", "-std=c++17"])
    b = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if b.returncode != 0:
        return ("BUILD-FAIL", b.stdout[-400:])
    env = dict(os.environ, L2C_RAND_OPS=os.environ.get("L2C_MUT_OPS", "5000"))
    try:
        r = subprocess.run([os.path.join(mdir, "Vtb_l2c")], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, timeout=900, env=env)
        out, rc = r.stdout, r.returncode
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
        open(os.path.join(work, name + ".log"), "w").write(out)
        fails = sorted(set(l[7:].split(" (")[0] for l in out.splitlines() if l.startswith("[FAIL]")))
        return ("CAUGHT", "hang (900 s timeout)" + ("; FAIL before it: " + ", ".join(fails) if fails else ""))
    open(os.path.join(work, name + ".log"), "w").write(out)
    fails = [l for l in out.splitlines() if l.startswith("[FAIL]")]
    asserts = [l for l in out.splitlines() if "ASSERT" in l]
    if fails:
        return ("CAUGHT", "FAIL: " + ", ".join(sorted(set(l[7:].split(" (")[0] for l in fails))))
    if asserts or rc != 0:
        checks = [l.strip()[len("FAIL check: "):].split(" (/")[0] for l in out.splitlines()
                  if l.strip().startswith("FAIL check:")]
        first = ("; first failed check: " + checks[0][:90]) if checks else ""
        return ("CAUGHT", "assertion/abort: " + (asserts[0].strip()[:110] if asserts else f"rc={rc}") + first)
    return ("MISSED", "every scenario passed")

def main():
    want = set(sys.argv[1:])
    work = tempfile.mkdtemp(prefix="l2c_dw_mut_", dir=os.environ.get("TMPDIR"))
    print(f"l2c_dw_mutants: scratch {work}")
    missed = 0
    for m in MUTANTS:
        if want and m[0] not in want:
            continue
        verdict, detail = run_one(*m, work)
        if m[0] in EXPECTED_SURVIVORS:
            verdict = "EQUIV-OK" if verdict == "MISSED" else "EQUIV-BAD:" + verdict
        print(f"  [{verdict:10s}] {m[0]:28s} -- {m[4]}\n                 {detail}", flush=True)
        if verdict not in ("CAUGHT", "EQUIV-OK"):
            missed += 1
    print(f"l2c_dw_mutants: {missed} not caught" if missed else "l2c_dw_mutants: every mutant caught")
    return 1 if missed else 0

if __name__ == "__main__":
    sys.exit(main())
