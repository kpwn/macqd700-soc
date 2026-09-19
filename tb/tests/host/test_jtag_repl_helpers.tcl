# tb/tests/host/test_jtag_repl_helpers.tcl
#
# Host-side unit test for the correctness-critical helper procs in
# tools/jtag_repl.tcl.  No hardware, no Vivado: the procs are extracted by
# name and exercised against a mocked `rd`.
#
# What this pins down (all of these were live silent-fabrication traps):
#
#   * parse_num  — REPL numeric operands are HEX BY DEFAULT.  The dispatcher
#     used to run operands through Tcl `expr`, which parses a bare
#     8-hex-digit token as DECIMAL: `r 40800000` read 0x026E8F80 instead.
#     Silent, plausible, and it has caused wrong conclusions.
#   * require_aligned — the 32-bit JTAG-AXI master silently ALIGNS DOWN a
#     byte address inside the Vivado hw_axi layer, where no amount of Tcl
#     can fix it.  Reject at the command layer instead.
#   * rdx / rdx_or_empty — `rd` returns a BARE hex string plus the
#     0xBADA0BAD failure sentinel.  Feeding either to arithmetic yields a
#     plausible wrong number.  rdx returns an INTEGER or raises.
#   * dbg_has_feature — an unknown feature name must ALWAYS raise, including
#     on bitstreams whose OFF_FEATURES is unreadable, so a typo cannot
#     quietly answer "no".
#   * atrap_* (A-trap breakpoint helpers) — slot/value/mask validation must
#     reject anything that could never fire (wrong slot, non-A-line value,
#     out-of-range value/mask) instead of silently arming a dead
#     breakpoint; atrap_wr_verify must raise on a readback mismatch instead
#     of trusting an unverified write; the DBG_FEATURE_NAMES bit positions
#     for atrap_bp/atrap_regcap/atrap_d0qual must stay pinned at 15/16/17.
#   * wp_* (data-watchpoint helpers) — the WPn_CTRL encoding, the
#     big-endian byte-lane packing, and the mis-positioned-value-filter
#     warning.  Hardware requires EVERY armed lane to be written by the
#     access; `value 0x12 lanes 0xF` against a byte store used to match a
#     0x00000000 store on the live bitstream, and must now neither match
#     nor be armed silently.
#   * icache_probe_raw / ic_set_of / ic_tag_of / ic_line_of — the I-cache
#     probe.  Its geometry is NOT the D-cache's (64 sets x 16 B vs 32 x 32 B),
#     and its selector packs set/way/word at bits 0/6/8 rather than 0/5/7.
#     Reusing the D-cache numbers returns a real cache line from the wrong
#     place, which reads as a genuine coherency finding.  The probe is also
#     request/ack, so a done=0 poll timeout must RAISE — returning the
#     previous probe's answer would be indistinguishable from a real one.
#   * mon_sense_describe — the DAFB monitor-sense decoder.  Its format string
#     originally contained a bare "[extended monitor]", which Tcl reads as
#     COMMAND SUBSTITUTION: every extended sense code threw "invalid command
#     name extended" instead of printing.  The `mon-sense` command is also
#     feature-gated on OFF_FEATURES bit 18 precisely because an unmapped read
#     returns 0 and 0 is a LEGAL sense code, so "not supported" and "Mac 21in
#     Color Display" would otherwise be indistinguishable.
#
# Run: make tb-jtag-repl-host   (or: tclsh tb/tests/host/test_jtag_repl_helpers.tcl)
# Must be run from the repo root.

# without touching hardware.
set fh [open tools/jtag_repl.tcl r]; set src [read $fh]; close $fh

foreach name {parse_num require_aligned rdx rdx_or_empty dbg_has_feature dbg_features
              dbg_version dbg_is_core040_epoch dbg_feature_names
              rd_burst_all_identical sr_check_valid
              atrap_ctrl_off atrap_check_value atrap_check_mask
              atrap_pack_match atrap_unpack_match atrap_wr_verify
              wp_ctrl_encode wp_byte_lane wp_lane_warning
              mon_sense_describe video_reject_reason_name
              icache_probe_raw ic_set_of ic_tag_of ic_line_of
              bp_halt_latched} {
    if {![regexp "\n(proc $name \{.*?\n\})\n" $src -> body]} {
        puts "EXTRACT FAILED: $name"; exit 1
    }
    eval $body
}
set ::DBG_BASE 0x50900000
set ::OFF_VERSION 0x000
set ::OFF_FEATURES 0x0A0
# Extracted from source (NOT re-typed here) so a renumbering/reordering of
# both feature-name tables in tools/jtag_repl.tcl are caught below by comparing
# lsearch results against hardcoded expected bit indices, instead of this
# test silently carrying its own independently-drifting copy of the list.
foreach table {DBG_FEATURE_NAMES_LEGACY DBG_FEATURE_NAMES_CORE040} {
    if {![regexp "\nset $table \\{(.*?)\n\\}\n" $src -> names_body]} {
        puts "EXTRACT FAILED: $table"; exit 1
    }
    set ::$table $names_body
}
set ::CORE040_DEBUG_VERSION 0xDEB60100

set fails 0
proc check {name got exp} {
    if {$got ne $exp} { puts "  FAIL $name: got '$got' expected '$exp'"; incr ::fails } \
    else { puts "  ok   $name" }
}
proc check_err {name script} {
    if {[catch {uplevel 1 $script} e]} { puts "  ok   $name (errored: [string range $e 0 60]...)" } \
    else { puts "  FAIL $name: expected an error, got no error"; incr ::fails }
}

puts "parse_num — hex is the default (THE bug: bare hex was read as decimal)"
check "bare 8-hex address"  [parse_num 40800000] 1082130432
check "bare hex w/ letters" [parse_num 2E9B4]    190900
check "0x prefix"           [parse_num 0x40800000] 1082130432
check "# prefix"            [parse_num #ff]      255
check "explicit decimal d"  [parse_num d1000]    1000
check "explicit decimal 1000d" [parse_num 1000d] 1000
check_err "garbage rejected, not silently 0" {parse_num zzz}
check_err "empty rejected"                   {parse_num ""}
check_err "bad hex after 0x rejected"        {parse_num 0xzz}

puts "require_aligned — reject rather than silently align down"
check "aligned passes" [expr {[require_aligned 0x40800000]}] 1082130432
check_err "unaligned rejected" {require_aligned 0x40800002}

puts "rdx — never hand back the failure sentinel as data"
proc rd {addr} { return "DEB60006" }
check "rdx parses as HEX not decimal" [rdx 0] 3736469510
check "rdx_or_empty ok path"          [rdx_or_empty 0] 3736469510
proc rd {addr} { return "BADA0BAD" }
check_err "rdx raises on BADA0BAD"    {rdx 0}
check "rdx_or_empty returns empty on BADA0BAD" [rdx_or_empty 0] ""

puts "dbg_has_feature"
proc rd {addr} { return "00001FFF" }   ;# bits 0..12 set
check "bit 0 dbg_reset_domain" [dbg_has_feature dbg_reset_domain] 1
check "bit 12 perf_counters"   [dbg_has_feature perf_counters]    1
check "bit 13 watchpoints off" [dbg_has_feature watchpoints]      0
proc rd {addr} { return "00000000" }
check "all-zero FEATURES = no features claimed" [dbg_has_feature cfg_wipe] 0
proc rd {addr} { return "BADA0BAD" }
check "failed read = no feature claimed"        [dbg_has_feature cfg_wipe] 0
check_err "unknown feature name is an error"    {dbg_has_feature nonesuch}

puts "rd_burst_all_identical — a repeated-word burst must be detected, never trusted"
# HW-proven failure: dump-mem 0x0002e8a0 6 returned 0x0098205f SIX times
# while single-word reads of the same range differed (non-incrementing
# burst at the hw_axi layer).  The pure detector must flag that shape so
# rd_burst cross-checks addr+4 and fails loudly instead of returning it.
check "six identical words flagged" \
    [rd_burst_all_identical {0098205f 0098205f 0098205f 0098205f 0098205f 0098205f}] 1
check "case-insensitive identical flagged" \
    [rd_burst_all_identical {DEADBEEF deadbeef}] 1
check "varying words pass" \
    [rd_burst_all_identical {0098205f 30280010 6efa4a40}] 0
check "single word is trivially identical (n<=1 never bursts)" \
    [rd_burst_all_identical {0098205f}] 1

puts "sr_check_valid — impossible SR bit patterns must be rejected, not printed as data"
# HW-proven failure: live-arch reported SR=0x3850 — bits 11 and 6 set,
# both reserved-zero on a 68040 (implemented mask 0xF71F).  That reading
# blocked a hardware verdict on ori.w #$0700,SR.
check "0x3850 (the hardware value) is INVALID" \
    [expr {[sr_check_valid 0x3850] ne ""}] 1
check "bit 11 alone invalid"  [expr {[sr_check_valid 0x0800] ne ""}] 1
check "bit 7 alone invalid"   [expr {[sr_check_valid 0x0080] ne ""}] 1
check "bit 6 alone invalid"   [expr {[sr_check_valid 0x0040] ne ""}] 1
check "bit 5 alone invalid"   [expr {[sr_check_valid 0x0020] ne ""}] 1
check "reset SR 0x2700 valid" [sr_check_valid 0x2700] ""
check "user SR 0x0010 valid"  [sr_check_valid 0x0010] ""
check "full legal mask valid" [sr_check_valid 0xF71F] ""

puts "watchpoints feature bit"
proc rd {addr} { return "00002FFF" }   ;# bits 0..12 + 13 set
check "bit 13 watchpoints on" [dbg_has_feature watchpoints] 1

puts "atrap — legacy feature-name bit positions (guards a renumbering regression)"
check "atrap_bp is bit 15"     [lsearch -exact $::DBG_FEATURE_NAMES_LEGACY atrap_bp]      15
check "atrap_regcap is bit 16" [lsearch -exact $::DBG_FEATURE_NAMES_LEGACY atrap_regcap]  16
check "atrap_d0qual is bit 17" [lsearch -exact $::DBG_FEATURE_NAMES_LEGACY atrap_d0qual]  17
proc rd {addr} { return "00038000" }   ;# bits 15,16,17 set, nothing else
check "dbg_has_feature atrap_bp"     [dbg_has_feature atrap_bp]     1
check "dbg_has_feature atrap_regcap" [dbg_has_feature atrap_regcap] 1
check "dbg_has_feature atrap_d0qual" [dbg_has_feature atrap_d0qual] 1
proc rd {addr} { return "00000000" }
check "atrap_bp off when FEATURES all-zero" [dbg_has_feature atrap_bp] 0

puts "atrap — CSR offsets (0x0E0..0x108), extracted from tools/jtag_repl.tcl itself"
foreach {name expect} {
    OFF_ATRAP0_CTRL     0x0E0
    OFF_ATRAP0_MATCH    0x0E4
    OFF_ATRAP0_D0VAL    0x0E8
    OFF_ATRAP1_CTRL     0x0EC
    OFF_ATRAP1_MATCH    0x0F0
    OFF_ATRAP1_D0VAL    0x0F4
    OFF_ATRAP_SKIP_ONCE 0x0F8
    OFF_ATRAP_HIT       0x0FC
    OFF_ATRAP_HIT_PC    0x100
    OFF_ATRAP_HIT_A0    0x104
    OFF_ATRAP_HIT_D0    0x108
} {
    if {![regexp "\nset $name +(0x\[0-9A-Fa-f\]+)" $src -> got]} {
        puts "  FAIL offset $name: not found in tools/jtag_repl.tcl"; incr fails; continue
    }
    set ::$name [expr {$got}]
    check "offset $name" [expr {$got}] [expr {$expect}]
}

puts "atrap — MATCH register packing: {mask\[31:16\], value\[15:0\]}"
# NOTE polarity is the OPPOSITE of the WPn_AMASK watchpoint convention
# (1 = ignore there; 1 = CARE/compare here) -- this pins the packing, not
# the polarity note, which is a comment-only concern.
check "pack value=0xA815 mask=0xFFFF" [format 0x%08X [atrap_pack_match 0xA815 0xFFFF]] 0xFFFFA815
check "pack value=0xA800 mask=0xFF00" [format 0x%08X [atrap_pack_match 0xA800 0xFF00]] 0xFF00A800
check "unpack roundtrips {value mask}" [atrap_unpack_match 0xFFFFA815] {43029 65535}
check "unpack ignores upper 32 bits of a wider int" [atrap_unpack_match 0x1FFFFA815] {43029 65535}

puts "atrap — argument validation (a silently never-firing breakpoint is the failure mode to avoid)"
check_err "slot 2 rejected"                       {atrap_ctrl_off 2}
check_err "slot -1 rejected"                       {atrap_ctrl_off -1}
check "slot 0 accepted"                           [atrap_ctrl_off 0] $::OFF_ATRAP0_CTRL
check "slot 1 accepted"                           [atrap_ctrl_off 1] $::OFF_ATRAP1_CTRL
check_err "value 0x4E71 (not A-line) rejected"    {atrap_check_value 0x4E71}
check_err "value 0x1A815 (out of 16-bit range) rejected" {atrap_check_value 0x1A815}
check "value 0xA815 (A-line) accepted"            [atrap_check_value 0xA815] 0xA815
check "value 0xA000 (A-line, low end) accepted"   [atrap_check_value 0xA000] 0xA000
check_err "mask 0x10000 (out of 16-bit range) rejected" {atrap_check_mask 0x10000}
check "mask 0xFFFF accepted"                      [atrap_check_mask 0xFFFF] 0xFFFF

puts "atrap_wr_verify — readback verification must catch a mismatched write, not pass silently"
proc wr {addr data} { set ::last_wr_data $data }
proc rd {addr} { return [format %08X $::last_wr_data] }
check "matching readback passes"  [atrap_wr_verify $::OFF_ATRAP0_CTRL 0x1 "atrap0 CTRL"] 1
proc rd {addr} { return "DEADBEEF" }
check_err "mismatched readback raises, not silently passes" \
    {atrap_wr_verify $::OFF_ATRAP0_CTRL 0x1 "atrap0 CTRL"}
proc rd {addr} { return "BADA0BAD" }
check_err "failed readback (BADA0BAD sentinel) also raises" \
    {atrap_wr_verify $::OFF_ATRAP0_CTRL 0x1 "atrap0 CTRL"}

puts "watch — WPn_CTRL encoding (bit0 en, bit1 load, bit2 store, bit3 value, bits\[11:8\] lanes)"
check "kind w  no value"      [format 0x%X [wp_ctrl_encode w  "" 0xF]] 0x5
check "kind r  no value"      [format 0x%X [wp_ctrl_encode r  "" 0xF]] 0x3
check "kind rw no value"      [format 0x%X [wp_ctrl_encode rw "" 0xF]] 0x7
check "kind w  value lanes 8" [format 0x%X [wp_ctrl_encode w 0x12000000 0x8]] 0x80D
check "kind rw value lanes F" [format 0x%X [wp_ctrl_encode rw 0xDEADBEEF 0xF]] 0xF0F
check_err "bad kind rejected"           {wp_ctrl_encode x "" 0xF}
check_err "value with zero lanes rejected (would never fire)" \
    {wp_ctrl_encode w 0x12 0x0}

puts "watch — byte-lane packing (byte at the LOWEST address sits in bits\[31:24\] = lane 3)"
check "addr ...00 -> lane mask/shift" [wp_byte_lane 0x00016E84] {8 24}
check "addr ...01 -> lane mask/shift" [wp_byte_lane 0x00016E85] {4 16}
check "addr ...02 -> lane mask/shift" [wp_byte_lane 0x00016E86] {2 8}
check "addr ...03 -> lane mask/shift" [wp_byte_lane 0x00016E87] {1 0}

puts "watch — mis-positioned value filter must WARN (hardware requires every armed lane to be written)"
# The live-HW mistake: `value 0x12 lanes 0xF` against a byte store.  The RTL
# now refuses to match it; the host must say why instead of going quiet.
check "byte value + default lanes warns" \
    [expr {[wp_lane_warning 0x00016E84 0x12 0xF] ne ""}] 1
check "correctly-positioned value does not warn" \
    [wp_lane_warning 0x00016E84 0x12000000 0x8] ""
check "no value filter does not warn" \
    [wp_lane_warning 0x00016E84 "" 0xF] ""
check "full-longword value does not warn" \
    [wp_lane_warning 0x00016E84 0xDEADBEEF 0xF] ""

puts "mon-sense — legacy feature-name bit position + decoder"
check "mon_sense is bit 18" [lsearch -exact $::DBG_FEATURE_NAMES_LEGACY mon_sense] 18
proc rd {addr} { return "00040000" }   ;# bit 18 only
check "dbg_has_feature mon_sense" [dbg_has_feature mon_sense] 1
# The gate matters because 0 is a LEGAL sense code: on a bitstream without
# the CSR the unmapped read returns 0, which would otherwise print as
# "Mac 21in Color Display" rather than "not supported".
proc rd {addr} { return "00000000" }
check "mon_sense off when FEATURES all-zero" [dbg_has_feature mon_sense] 0

# Standard codes (bit 6 clear) name a display; extended codes (bit 6 set)
# decode as ext(bc,ac,ab) over mon[5:4]/mon[3:2]/mon[1:0] — the same bit
# positions rtl/mac/video.v's sense_response() reads.
check "0x06 is the shipping default (Hi-Res 12-14in)" \
    [mon_sense_describe 0x06] "0x06 Mac Hi-Res 12-14\" 640x480"
check "0x00 is a real display, not 'unset'" \
    [mon_sense_describe 0x00] "0x00 Mac 21\" Color Display"
check "0x07 is 'no monitor'" [mon_sense_describe 0x07] "0x07 no monitor"
# THE regression: a bare [extended monitor] in the format string is Tcl
# command substitution, so this used to raise instead of returning a string.
check "0x5D decodes as bc=1/ac=3/ab=1 (extended)" \
    [mon_sense_describe 0x5D] "0x5D ext(bc=1,ac=3,ab=1) \[extended monitor\]"
check "0x6D is ext(2,3,1) — NOT 0x5D, a long-standing comment error" \
    [mon_sense_describe 0x6D] "0x6D ext(bc=2,ac=3,ab=1) \[extended monitor\]"
check "0x7F opens every extended field" \
    [mon_sense_describe 0x7F] "0x7F ext(bc=3,ac=3,ab=3) \[extended monitor\]"
# Bits above [6:0] must be dropped, matching the RTL's w_data_r[6:0].
check "bits\[31:7\] are masked off" \
    [mon_sense_describe 0xFFFFFF86] [mon_sense_describe 0x06]

# ─────────────────────────────────────────────────────────────────────────
# Effective-halt gating, advance preconditions, cache-op completion, and
# VIO probe-width padding.
#
# Every one of these guards a tool that returned a PLAUSIBLE WRONG ANSWER
# instead of an error:
#
#   * effective_halt / require_effective_halt — HALT_REASON bit 3 is the
#     only bit meaning "the CPU actually stopped".  The others are requests
#     and latches.  dcache-op/icache-op issued against a running CPU used
#     to return busy=0 done=0 and no error at all, which is a SILENT NO-OP:
#     JTAG reads DDR, the D-cache is write-back, so the push-then-read
#     recipe was quietly doing nothing and memory reads disagreed with what
#     was visibly on screen.
#   * advance — reads the retired-inst count at the last halt and arms
#     halt-after at (count + N).  If the halt had not LANDED that read
#     returns 0, so the target became 0+N, already in the past: nothing
#     ever halted again while a bisect ladder kept printing confident
#     "good" rows.  Five bogus probes went by in one ladder.
#   * cache_op_poll — done=0 means the maintenance walk did NOT happen, so
#     anything read afterwards is exactly as stale as before.  Must raise.
#   * vio_fmt_value — Vivado requires EXACTLY ceil(WIDTH/4) hex characters
#     for a HEX-radix probe.  probe_out0 (vio_boot_ctrl) went 4b -> 5b at
#     probe_map=v20 (876de3c, PRAM-zap bit), so `vio_set 8` started dying
#     with "[Designutils 20-1474] ... has [1] value characters, required
#     [2]" and vio-reset-halt-exc broke outright.  The pad width must be
#     queried from the LIVE probe, never hardcoded.
# ─────────────────────────────────────────────────────────────────────────
foreach name {effective_halt require_effective_halt break_pc_reached
              cache_op_poll advance vio_fmt_value} {
    if {![regexp "\n(proc $name \{.*?\n\})\n" $src -> body]} {
        puts "EXTRACT FAILED: $name"; exit 1
    }
    eval $body
}

# A refusal is only useful if it says the right thing — these guards exist
# to be READ by whoever hit them, so pin the message content, not just the
# fact that something raised.
proc check_err_match {name script pattern} {
    if {![catch {uplevel 1 $script} e]} {
        puts "  FAIL $name: expected an error, got none"; incr ::fails; return
    }
    if {[string match $pattern $e]} { puts "  ok   $name" } \
    else { puts "  FAIL $name: message did not match '$pattern'\n        got: $e"; incr ::fails }
}
proc check_no_err {name script} {
    if {[catch {uplevel 1 $script} e]} {
        puts "  FAIL $name: unexpected error: $e"; incr ::fails
    } else { puts "  ok   $name" }
}

set ::OFF_HALT_REASON      0x030
set ::OFF_HALT_CTL         0x03C
set ::OFF_HALT_HIT_PC      0x038
set ::OFF_HALT_AFTER_LO    0x020
set ::OFF_HALT_AFTER_HI    0x024
set ::OFF_CONTROL          0x000
set ::OFF_DCACHE_OP        0x210
set ::HALT_AFTER_EN        0x1
proc dbg_rd {off} { return [rd [expr {$::DBG_BASE + $off}]] }
# Stubbed: the real one issues a dozen reads and is not what these tests pin.
proc halt_status_line {} { return "halt: <mocked>" }

puts "effective_halt — HALT_REASON bit 3, and ONLY bit 3, means 'CPU stopped'"
proc rd {addr} { return "00000008" }
check "bit 3 set = effective halt"            [effective_halt] 1
proc rd {addr} { return "00000000" }
check "nothing set = not halted"              [effective_halt] 0
# The trap: manual/halt-after/break-pc REQUEST bits look like a halt.
proc rd {addr} { return "00000007" }
check "bits 0,1,2 set but bit 3 clear = NOT halted" [effective_halt] 0
proc rd {addr} { return "00000010" }
check "halt-after ENABLE (bit 4) alone = NOT halted" [effective_halt] 0
# This is the exact HALT_REASON seen on hardware mid-boot while `dcache-op
# push` was being issued and its result trusted (reason=0x10, effective=0).
proc rd {addr} { return "00000019" }
check "0x19 (manual+enable+effective) IS halted" [effective_halt] 1

puts "require_effective_halt — must REFUSE, not warn, when the CPU is running"
proc rd {addr} { return "00000000" }
check_err_match "refusal names the command" \
    {require_effective_halt "dcache-op push" "halt first"} "*dcache-op push*"
check_err_match "refusal says the CPU is RUNNING" \
    {require_effective_halt "advance"} "*RUNNING*"
check_err_match "refusal cites HALT_REASON bit 3" \
    {require_effective_halt "advance"} "*HALT_REASON bit 3*"
check_err_match "refusal carries the hint" \
    {require_effective_halt "x" "use dcache-probe instead"} "*dcache-probe*"
proc rd {addr} { return "00000008" }
check_no_err "permits when effectively halted" \
    {require_effective_halt "dcache-op push" "halt first"}

puts "cache_op_poll — done=0 is a FAILED op, never a status line to move past"
proc rd {addr} { return "00000000" }   ;# busy=0 done=0: the silent no-op
check_err_match "done=0 raises and says DID NOT COMPLETE" \
    {cache_op_poll $::OFF_DCACHE_OP "dcache-op push"} "*DID NOT COMPLETE*"
check_err_match "the raise says the cache was NOT maintained" \
    {cache_op_poll $::OFF_DCACHE_OP "dcache-op push"} "*NOT maintained*"
check_err_match "the raise warns the reads are stale" \
    {cache_op_poll $::OFF_DCACHE_OP "dcache-op push"} "*stale*"
proc rd {addr} { return "00000002" }   ;# done=1
check_no_err "done=1 completes quietly" \
    {cache_op_poll $::OFF_DCACHE_OP "dcache-op push"}

puts "break_pc_reached — bit 2 latched AND HIT_PC matching, not either alone"
proc rd {addr} {
    if {$addr == [expr {$::DBG_BASE + $::OFF_HALT_REASON}]} { return "00000004" }
    return "40800100"
}
check "latched + matching PC = reached"       [break_pc_reached 0x40800100] 1
check "latched but DIFFERENT PC = not reached" [break_pc_reached 0x40800200] 0
proc rd {addr} {
    if {$addr == [expr {$::DBG_BASE + $::OFF_HALT_REASON}]} { return "00000000" }
    return "40800100"
}
check "HIT_PC matches but nothing latched = not reached" [break_pc_reached 0x40800100] 0

puts "bp_halt_latched — the halt SOURCE, decoded per epoch (not HALT_CTL bit 5)"
# THE BUG THIS PINS (hardware, 2026-08-22): `continue` and `step` decided
# "am I stopped on a precise breakpoint?" by testing HALT_CTL bit 5.  That bit
# is RAZ on the Stage 5 core, so the answer was ALWAYS 0 there: skip_once was
# never armed, decode re-injected SYS_DBG_BREAK at the same PC, and the
# breakpoint re-fired forever with no forward progress.  The core040 answer
# must come from the OFF_HALT_REASON primary code (5 = precise PC breakpoint).
proc mk_rd {version reason ctl} {
    set ::mock_version $version; set ::mock_reason $reason; set ::mock_ctl $ctl
    proc rd {addr} {
        if {$addr == [expr {$::DBG_BASE + $::OFF_VERSION}]}      { return $::mock_version }
        if {$addr == [expr {$::DBG_BASE + $::OFF_HALT_REASON}]}  { return $::mock_reason }
        if {$addr == [expr {$::DBG_BASE + $::OFF_HALT_CTL}]}     { return $::mock_ctl }
        return "00000000"
    }
}
# core040: HALT_CTL reads 0 (RAZ) — the answer must still be correct.
mk_rd "DEB60100" "00000005" "00000000"
check "core040 reason=5 with HALT_CTL RAZ = latched"   [bp_halt_latched] 1
mk_rd "DEB60100" "00000001" "00000000"
check "core040 reason=1 (manual) = not latched"        [bp_halt_latched] 0
mk_rd "DEB60100" "00000003" "00000000"
check "core040 reason=3 (halt-after) = not latched"    [bp_halt_latched] 0
mk_rd "DEB60100" "00000006" "00000000"
check "core040 reason=6 (exception) = not latched"     [bp_halt_latched] 0
# Primary code is the LOW 3 BITS; upper bits must not disturb it.
mk_rd "DEB60100" "0000000D" "00000000"
check "core040 reason=0x0D (code 5 + bit3) = latched"  [bp_halt_latched] 1
# core040 must NOT consult HALT_CTL bit 5 at all.
mk_rd "DEB60100" "00000001" "00000020"
check "core040 ignores HALT_CTL bit 5 when reason says otherwise" [bp_halt_latched] 0
# Legacy bitstreams keep the old decode.
mk_rd "DEB50000" "00000000" "00000020"
check "legacy HALT_CTL bit 5 set = latched"            [bp_halt_latched] 1
mk_rd "DEB50000" "00000005" "00000000"
check "legacy ignores the core040 reason code"         [bp_halt_latched] 0

puts "advance — the preconditions that stop a poisoned bisect ladder"
proc dbg_wr {off data} { }
proc clear_auto_halt {{extra -1}} { }
proc halt_enable_bits {} { return 0 }
proc current_halt_inst {} { return $::mock_inst }
set ::mock_inst 0
proc rd {addr} { return "00000000" }   ;# not halted
check_err_match "refuses when the halt has not landed" \
    {advance 100 0} "*landed halt*"
# THE bug: halted, but inst-count reads 0 -> target 0+N is already in the
# past -> halt-after never fires again for the rest of the session.
proc rd {addr} { return "00000008" }   ;# effectively halted...
set ::mock_inst 0                      ;# ...but the count never landed
check_err_match "refuses on inst-count==0 even when 'halted'" \
    {advance 100 0} "*inst-count reads 0*"
check_err_match "refusal warns NOTHING WOULD EVER HALT AGAIN" \
    {advance 100 0} "*NOTHING WOULD EVER HALT AGAIN*"
check_err "rejects N<=0"                      {advance 0 0}
check_err "rejects negative N"                {advance -5 0}
# Landed: base 1000, advance 100, ends at >= 1100 while halted.
set ::mock_inst 1000
proc current_halt_inst {} {
    if {$::mock_phase eq "after"} { return 1100 }
    return 1000
}
set ::mock_phase "before"
proc after {args} { set ::mock_phase "after" }
check "returns 1 when it lands at the target"  [advance 100 0] 1
# Released but never re-halted: must report failure, not silence.
proc rd {addr} {
    if {$::mock_phase eq "after"} { return "00000000" }
    return "00000008"
}
set ::mock_phase "before"
check "returns 0 when the halt never lands"    [advance 100 0] 0
# Halted EARLY (something else stopped it) — also a failed probe.
proc rd {addr} { return "00000008" }
proc current_halt_inst {} { return 1000 }
set ::mock_phase "before"
check "returns 0 when halted short of the target" [advance 100 0] 0
rename after {}

puts "vio_fmt_value — pad width comes from the LIVE probe, never hardcoded"
# The regression that broke vio-reset-halt-exc: 5-bit probe, value 8.
proc get_property {prop obj} {
    if {$prop eq "WIDTH"} { return $::mock_width }
    return "vio_boot_ctrl"
}
set ::mock_width 5
check "5-bit probe pads 0x8 to two digits"    [vio_fmt_value p 8] "08"
check "5-bit probe pads 0x0 to two digits"    [vio_fmt_value p 0] "00"
check "5-bit probe keeps 0x10 (PRAM zap bit)" [vio_fmt_value p 16] "10"
check "5-bit probe keeps 0x1F"                [vio_fmt_value p 31] "1F"
# Width is bitstream-dependent and HAS already changed once (4 -> 5).
set ::mock_width 4
check "4-bit probe emits ONE digit"           [vio_fmt_value p 8] "8"
set ::mock_width 1
check "1-bit probe emits one digit"           [vio_fmt_value p 1] "1"
set ::mock_width 16
check "16-bit probe emits four digits"        [vio_fmt_value p 8] "0008"
# Overflow must be visible, not silently wrapped into a different command.
set ::mock_width 5
check "value wider than the probe is truncated to width" [vio_fmt_value p 0x3F] "1F"
proc get_property {prop obj} {
    if {$prop eq "WIDTH"} { return "" }
    return "vio_boot_ctrl"
}
check_err_match "unreadable WIDTH raises rather than guessing" \
    {vio_fmt_value p 8} "*cannot read WIDTH*"

# ─────────────────────────────────────────────────────────────────────────
# I-cache probe helpers (`icache-probe` / `icache-lookup`).
#
# The geometry constants are duplicated here ON PURPOSE, as literals taken
# from cpu/rtl/core/fetch/icache.v (4 KB, 4-way, 16 B lines -> 64 sets,
# set = addr[9:4], tag = addr[31:10]).  If someone "helpfully" reuses the
# D-cache's 5-bit set / 8-word line, every lookup silently reports the
# WRONG line -- and a wrong cache line is completely plausible-looking
# output, which is the failure mode this whole file exists to prevent.
# ─────────────────────────────────────────────────────────────────────────
puts "icache-probe — geometry, selector packing, and completion polling"
check "icache_probe is legacy bit 19" [lsearch -exact $::DBG_FEATURE_NAMES_LEGACY icache_probe] 19

puts "core040 — Stage-5 feature-name bit positions"
check "arch_apply_stays_halted is bit 19" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 arch_apply_stays_halted] 19
check "arch_dirty_apply is bit 20" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 arch_dirty_apply] 20
check "cache_maint_only is bit 21" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 cache_maint_only] 21
check "macro_retire_count is bit 22" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 macro_retire_count] 22
check "stop_status_v2 is bit 23" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 stop_status_v2] 23
check "branch_ring is bit 24" [lsearch -exact $::DBG_FEATURE_NAMES_CORE040 branch_ring] 24
proc rd {addr} { return "00080000" }   ;# bit 19 only
check "dbg_has_feature icache_probe" [dbg_has_feature icache_probe] 1
proc rd {addr} { return "00000000" }
check "icache_probe off when FEATURES all-zero" [dbg_has_feature icache_probe] 0

set ::IC_NUM_SETS   64
set ::IC_LINE_BYTES 16
set ::IC_NUM_WAYS   4
set ::OFF_ICACHE_PROBE_SEL   0x218
set ::OFF_ICACHE_PROBE_TAG   0x21C
set ::OFF_ICACHE_PROBE_FLAGS 0x220
set ::OFF_ICACHE_PROBE_DATA  0x224

# The live investigation address, so the numbers below are checkable by
# hand against the RTL: 0x0075ABE0 -> set 62, tag 0x001D6A.
check "set = addr\[9:4\]"        [ic_set_of  0x0075ABE0] 62
check "tag = addr\[31:10\]"      [format 0x%06X [ic_tag_of 0x0075ABE0]] 0x001D6A
check "line-align drops addr\[3:0\]" \
    [format 0x%08X [ic_line_of 0x0075ABE4]] 0x0075ABE0
# 64 sets, not the D-cache's 32: set 62 must NOT fold to 30.
check "set index is 6 bits, not 5" [ic_set_of 0x000003E0] 62
check "set index wraps at 64"      [ic_set_of 0x00000400] 0

# Argument validation.  Every one of these would otherwise pack a
# truncated selector and return a real line from the WRONG place.
# `after` was renamed away by the advance/vio sections above; the probe
# poll loop calls it between reads.
if {[info commands after] eq ""} { proc after {args} { } }
proc dbg_wr {off data} { set ::last_sel $data }
proc dbg_rd {off} {
    if {$off == $::OFF_ICACHE_PROBE_FLAGS} { return "00000003" }
    if {$off == $::OFF_ICACHE_PROBE_TAG}   { return "0001D6A" }
    return "2F2EFFFC"
}
check_err "rejects set 64 (only 0..63 exist)"  {icache_probe_raw 64 0 0}
check_err "rejects negative set"               {icache_probe_raw -1 0 0}
check_err "rejects way 4"                      {icache_probe_raw 0 4 0}
check_err "rejects word 4 (16 B line = 4 longwords)" {icache_probe_raw 0 0 4}
check_err "rejects a non-numeric set"          {icache_probe_raw x 0 0}

# Selector packing: set[5:0], way[7:6], word[9:8].  Copying the D-cache's
# way<<5 / word<<7 packing here would alias set bits 5 into the way field.
set ::last_sel 0
icache_probe_raw 62 3 2
check "selector packs set/way/word at 0/6/8" \
    [format 0x%03X $::last_sel] 0x2FE
set ::last_sel 0
icache_probe_raw 63 0 0
check "set 63 does not spill into the way field" \
    [format 0x%03X $::last_sel] 0x03F

# Result decoding off FLAGS/TAG/DATA.
set r [icache_probe_raw 62 3 2]
check "valid comes from FLAGS bit 0"  [dict get $r valid] 1
check "tag comes from OFF_..._TAG"    [format 0x%06X [dict get $r tag]] 0x001D6A
check "data comes from OFF_..._DATA"  [format 0x%08X [dict get $r data]] 0x2F2EFFFC

# THE trap this probe could produce: the done bit never sets (the CPU is
# running and the I-cache never gets an idle cycle), so TAG/DATA still hold
# the PREVIOUS probe's answer -- a perfectly plausible cache line for a
# set/way that was never sampled.  Must raise, never return.
proc dbg_rd {off} {
    if {$off == $::OFF_ICACHE_PROBE_FLAGS} { return "00000005" }  ;# valid+busy, done=0
    if {$off == $::OFF_ICACHE_PROBE_TAG}   { return "0001D6A" }
    return "2F2EFFFC"
}
check_err_match "done=0 raises rather than returning a stale line" \
    {icache_probe_raw 0 0 0} "*DID NOT COMPLETE*"
check_err_match "the raise says nothing was sampled" \
    {icache_probe_raw 0 0 0} "*NOTHING was sampled*"
check_err_match "the raise tells you to halt" \
    {icache_probe_raw 0 0 0} "*halt first*"

puts ""
puts "-- video_reject_reason_name (scan-out placement admission verdicts) --"
# The numbers are a wire protocol between mode_admit.v's REJ_*
# localparams and this decoder.  A silent divergence is WORSE than no decode:
# it names the wrong gate confidently, and the whole reason the channel exists
# is that the previous answer ("rd_en=0, no reason given") cost a day.
#
# Extract the RTL's own list and require the two to agree, rather than
# re-typing the mapping here where it could drift unnoticed.
set rfh [open rtl/board/video_phy/mode_admit.v r]
set rtl [read $rfh]; close $rfh
foreach {sym want_code want_word} {
        REJ_NONE          0 "OK"
        REJ_DEPTH_UNSUP   1 "DEPTH_UNSUPPORTED"
        REJ_STRIDE_SHORT  2 "STRIDE_SHORT"
        REJ_FRAME_OOM     3 "FRAME_OUT_OF_MEMORY"
        REJ_GEOMETRY_ZERO 4 "GEOMETRY_ZERO"} {
    if {![regexp "localparam \\\[3:0\\\] $sym\\s*=\\s*4'd(\\d+)" $rtl -> got_code]} {
        puts "  FAIL $sym: not found in mode_admit.v"; incr fails; continue
    }
    check "$sym is 4'd$want_code in the RTL" $got_code $want_code
    check "code $want_code decodes to $want_word" \
        [string match "${want_word}*" [video_reject_reason_name $want_code]] 1
}
# An unknown code must say so loudly rather than fall through to a plausible
# name -- a decoder older than the bitstream is exactly when that matters.
check "an unknown code is reported as UNKNOWN" \
    [string match "UNKNOWN(9)*" [video_reject_reason_name 9]] 1
check "the unknown-code message points at the RTL list" \
    [string match "*mode_admit.v*" [video_reject_reason_name 9]] 1

puts ""
if {$fails} { puts "$fails CHECK(S) FAILED"; exit 1 }
puts "ALL HOST-SIDE HELPER CHECKS PASSED"
