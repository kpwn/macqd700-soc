#!/bin/bash
# cdef_fail_scsi_trace.sh — capture the 53C96 register trace at the moment the
# CDEF load fails, to test whether scsi.v returned CHECK CONDITION / NOT READY.
#
# HYPOTHESIS UNDER TEST (rtl/mac/scsi.v, the two "READ timeout waiting for
# backing store" arms near lines 2957 and 2984):
#
#     end else if (vh_kicked && !vh_busy) begin
#         if (vh_wait_ctr >= (VH_WAIT_TIMEOUT - 16'd1)) begin
#             busy_error_pending <= 1'b1;
#             medium_not_present <= 1'b1;
#             xfer_status <= 8'h02;   // CHECK CONDITION
#             sense_key   <= 4'd2;    // NOT READY
#             sense_asc   <= 8'h3A;   // medium not present
#
# `vh_wait_ctr` is cleared only when data arrives or on timeout -- NOT while
# vh_busy is high -- so short repeated not-busy gaps inside one command
# ACCUMULATE toward VH_WAIT_TIMEOUT (=1024 cycles, ~10us @100MHz).  A spurious
# timeout would return NOT READY on a healthy read, which the File Manager
# surfaces as ioErr, which makes _LoadResource fail, which is SysError(88).
#
# Crucially this fires in scsi.v's OWN logic, so sd_ctrl's error counter stays
# ZERO -- exactly the contradiction measured (ResErr=-36 with 0 SD errors).
#
# If the trace shows NO CHECK CONDITION near the failure, this hypothesis is
# WRONG and must be dropped rather than patched around.

cd "$(dirname "$0")/.." || exit 1
WAIT=${WAIT:-430}
TRIES=${TRIES:-4}
OUT=${OUT:-/tmp/scsi_at_cdef_fail.csv}
jt() { JT_WAIT=${2:-20} tools/jt.sh "$1" 2>&1; }
wait_halt() { local s=$SECONDS
    while [ $((SECONDS-s)) -lt "$1" ]; do
        jt "halt-status" 8 | grep -q 'effective=1' && return 0; done; return 1; }

for t in $(seq 1 "$TRIES"); do
    echo "=== attempt $t/$TRIES ==="
    for c in "watch 0 off" "watch 1 off" "atrap 0 off" "atrap 1 off" "break-pc off" "halt-clear"; do
        jt "$c" >/dev/null; done
    jt "break-pc 0x40815E6A" 25 | grep -E "slot=" | sed 's/^/  /'
    jt "scsi-trace rearm" 25 | sed 's/^/  /'
    jt "vio-hard-reset" 30 >/dev/null

    if ! wait_halt "$WAIT"; then echo "  boot did not reach the failure; retrying"; continue; fi
    echo "  HALTED at the CDEF failure arm"
    jt "scsi-trace freeze" 25 | sed 's/^/  /'
    jt "scsi-trace dump $OUT" 120 | tail -3 | sed 's/^/  /'

    if [ ! -s "$OUT" ]; then echo "  no trace file produced"; exit 2; fi
    echo
    echo "  trace rows: $(wc -l < "$OUT")"
    echo "  --- last 25 rows ---"
    tail -25 "$OUT" | sed 's/^/    /'
    echo
    echo "  --- looking for CHECK CONDITION (0x02) / sense NOT READY (key 2, ASC 0x3A) ---"
    grep -inE "0x02|,02,|3a|not.?ready|check" "$OUT" | tail -20 | sed 's/^/    /' || echo "    (no obvious match)"
    exit 0
done
echo "=== never caught a failing boot ==="; exit 1
