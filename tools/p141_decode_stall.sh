#!/usr/bin/env bash
# p141 -- decode the four live stall-state CSRs into the actual answer.
#
#   tools/p141_decode_stall.sh <dc> <grant> <exc> <walk>     (hex, 0x-prefixed)
#
# The point of this script is that the raw words are useless on the bench and
# very easy to misread by hand. `dcIdleForMaint` is a 17-way conjunction whose
# terms are NOT all the same polarity -- thirteen of them block when SET, but
# the four AXI write-channel `*Done` flags block when CLEAR -- and getting that
# backwards would invert the conclusion. So the blocking test is encoded once,
# here, rather than re-derived at 3am from a hex dump.
#
# Bit layouts are defined by, and must stay in sync with:
#   DcachePlugin.logic.dbgStallDcPack        (next to `dcIdleForMaint` itself)
#   LsEuPlugin.logic.dbgStallGrantPack       (next to the grant machine)
#   ExceptionUnit.dbgStallExcPack            (next to `quiesceHoldOut`)
#   TableWalker.io.dbgPack                   (its declaration comment)
set -u

DC=${1:?usage: $0 <dc> <grant> <exc> <walk>}
GR=${2:?}; EX=${3:?}; WK=${4:?}
LIVENESS=${5:-UNKNOWN}

v () { printf '%d' $(( $1 )) ; }          # hex -> dec
b () { echo $(( ( $(v "$1") >> $2 ) & 1 )) ; }
f () { echo $(( ( $(v "$1") >> $2 ) & ( (1 << $3) - 1 ) )) ; }

echo "raw: dc=$DC grant=$GR exc=$EX walk=$WK"

# ── 1. WHICH dcIdleForMaint TERM IS BLOCKING ─────────────────────────────────
# This is the question the experiment exists to answer.
echo
echo "-- D-cache quiesce (dcIdleForMaint terms) --"
declare -a HI=(resetSweepBusy busy ldS1Valid ldS2Valid loadShadowValid \
               earlyProbeValid pendingStoreMiss pendingWtKickoff s0Valid \
               stS1Valid stS2Valid stS3Valid serialStoreInFlight storeMissBarrier)
blockers=""
for i in "${!HI[@]}"; do
    [ "$(b "$DC" "$i")" = 1 ] && blockers="$blockers ${HI[$i]}"
done
# Opposite polarity: these are DONE flags, so a ZERO is what holds quiesce low.
declare -a LO=(stAwDone stWDone evictAwDone evictWDone)
for i in "${!LO[@]}"; do
    [ "$(b "$DC" $((14 + i)))" = 0 ] && blockers="$blockers !${LO[$i]}"
done
stout=$(f "$DC" 18 4)
[ "$stout" -ne 0 ] && blockers="$blockers storeOutstanding=$stout"
if [ -z "$blockers" ]; then
    echo "   NO blocking term -- the D-cache is quiescent"
else
    echo "   BLOCKING:$blockers"
fi
echo "   dcIdleForMaint=$(b "$DC" 22)  maintBusyReg=$(b "$DC" 23)  maintQuiesced=$(b "$DC" 24)"

# ── 1b. IF `busy` IS THE BLOCKER: WHICH STATE, AND IS AXI STILL ASKING? ───────
# p141 could get no further than "busy", because `busy` is True across the whole
# EVICT_WR/REFILL/REPLAY excursion. With evictAwDone/evictWDone both set, EVICT_WR
# waiting on a B whose id never matches D_PUSH and REFILL waiting on an R that
# never arrives are BIT-IDENTICAL in [24:0]. These bits (p142, DcachePlugin
# [31:25]) separate them. On a p141-or-older bitstream they read 0 and this block
# says so rather than decoding zeros as fact -- the mistake 5d146b4d already fixed
# once for `wedge-status`.
FSMV=$(f "$DC" 25 2)
UPPER=$(f "$DC" 25 7)
if [ "$UPPER" -eq 0 ]; then
    echo "   [31:25] all zero -- either genuinely IDLE with the AXI quiet, or this"
    echo "           bitstream predates the p142 probe. Check the netlist, do not assume."
else
    case "$FSMV" in
        0) fsm="IDLE" ;;
        1) fsm="EVICT_WR" ;;
        2) fsm="REFILL" ;;
        3) fsm="REPLAY" ;;
    esac
    echo "   load FSM=$fsm  arSent=$(b "$DC" 27)"
    echo "   AXI: ar.valid=$(b "$DC" 28) r.valid=$(b "$DC" 29) b.valid=$(b "$DC" 30)" \
         "aw|w.valid=$(b "$DC" 31)"
    # The interpretation, stated here so it is not re-derived under time pressure.
    #
    # GATED ON CONFIRMED-FROZEN. Every line below reads a STALL into the bits, and most
    # of these states are perfectly ordinary on a RUNNING machine -- `REFILL` with
    # `arSent=1` and no `r.valid` is just a refill in flight, which is what a healthy
    # core looks like most of the time it is missing. Printing "lost or misrouted read
    # response" for that would manufacture exactly the confident-looking noise this
    # campaign has already been misled by twice. `p141_ab_boot.sh` calls this only when
    # retire is FROZEN and passes FROZEN as $5; a hand invocation that omits it gets the
    # raw state and no story.
    case "${LIVENESS^^}" in LIVE|ADVANCING) SUPPRESS=1 ;; *) SUPPRESS=0 ;; esac
    if [ "${LIVENESS^^}" = "UNKNOWN" ]; then
        echo "   (caller did not state retire liveness -- the reading below is only valid"
        echo "    for a core confirmed FROZEN and STATIC. On a RUNNING core, REFILL with"
        echo "    arSent=1 and no r.valid is a normal in-flight refill, not a stall.)"
    fi
    if [ "$SUPPRESS" = 1 ]; then
        echo "   retire is ADVANCING -- interpretation suppressed. These bits are an"
        echo "   instantaneous sample of a running machine and read as a stall when they"
        echo "   are nothing of the kind."
    elif [ "$(b "$DC" 1)" = 1 ]; then
        case "$fsm" in
        EVICT_WR)
            if [ "$(b "$DC" 31)" = 1 ]; then
                echo "   => EVICT_WR is still PRESENTING aw/w and the fabric is not accepting:"
                echo "      a downstream stall, not a lost response."
            else
                echo "   => EVICT_WR's aw+w are DONE and it is waiting for a B with id==D_PUSH."
                echo "      Nothing is being asked for, so the B is either never coming back or"
                echo "      came back with a non-matching id and was silently dropped by the"
                echo "      global axi.b.ready:=True. THIS is the lost-response shape."
            fi ;;
        REFILL)
            if [ "$(b "$DC" 27)" = 0 ]; then
                echo "   => REFILL has NOT issued its AR yet (arSent=0): the AR is being refused"
                echo "      upstream (arbiter grant or fabric), not lost."
            elif [ "$(b "$DC" 29)" = 1 ]; then
                echo "   => the R beat IS present but not accepted -- refillWriteHold is holding"
                echo "      r.ready low; look at the store pipe, not the fabric."
            else
                echo "   => the AR was issued and NO R has come back. Lost or misrouted read"
                echo "      response. Note AxiDMerge routes R by LATCHED OWNER, never by id."
            fi ;;
        REPLAY)
            echo "   => REPLAY is held by storeDrainRefillHold (stS2Valid||stS3Valid);"
            echo "      those bits are [10]/[11] above." ;;
        IDLE)
            echo "   => busy=1 while the FSM reads IDLE: busy is a register and is also True"
            echo "      for the first IDLE cycle after an excursion. A STATIC sample here"
            echo "      would mean busy is stuck set with no owner -- a different bug." ;;
        esac
    fi
fi

# ── 2. WHO HOLDS THE WALKER GRANT ────────────────────────────────────────────
echo
echo "-- D-cache port ownership / grant machine --"
own () { case "$1" in 0) echo CORE ;; 1) echo ITLB ;; 2) echo DTLB ;; *) echo "?$1" ;; esac ; }
echo "   ldOwner=$(own "$(f "$GR" 0 2)")  stOwner=$(own "$(f "$GR" 2 2)")"
# NOTE, verified in the generated Verilog: grant[6] and exc[3] are THE SAME NET
# (LsEuPlugin's `quiesceHold` input is driven by `exc.quiesceHoldOut`). They
# agreeing is a wiring tautology, NOT two independent measurements -- do not
# read agreement between them as corroboration of anything.
echo "   ldGrantOk=$(b "$GR" 4) stGrantOk=$(b "$GR" 5) quiesceHold=$(b "$GR" 6) [== exc quiesceHoldOut, same net]"
echo "   walkGrantHeld=$(b "$GR" 7) ownsLoad=$(b "$GR" 8) ownsStore=$(b "$GR" 9)"
echo "   coreStOutstanding=$(f "$GR" 12 4)  walkStOutstanding=$(b "$GR" 16)"
echo "   ldBusyExc=$(b "$GR" 17) ldFifoFull=$(b "$GR" 18) coreLsLoadReq=$(b "$GR" 19)"
echo "   walkLdReq itlb=$(b "$GR" 20) dtlb=$(b "$GR" 21)   walkStReq itlb=$(b "$GR" 22) dtlb=$(b "$GR" 23)"
echo "   loadCmd v/r=$(b "$GR" 24)/$(b "$GR" 25) loadRsp=$(b "$GR" 26)  store v/r=$(b "$GR" 27)/$(b "$GR" 28) ack=$(b "$GR" 29)"
if [ "$(b "$GR" 10)" = 1 ]; then
    echo "   *** walkerPortWedge SET -- the design's own detector fired: a walker has"
    echo "       held a D-cache port with NO progress for walkerWedgeLimit cycles ***"
fi
[ "$(b "$GR" 31)" = 1 ] && echo "   walkWedgeCnt is non-zero (a stall is accumulating)"
[ "$(b "$GR" 11)" = 1 ] && echo "   walkGrantProgress=1 (traffic is still moving)"

# ── 3. WHERE THE EXCEPTION SEQUENCER IS ──────────────────────────────────────
echo
echo "-- ExceptionUnit --"
declare -a ST=(IDLE E_DRAIN E_STORE E_STWAIT E_VECREQ E_VECWAIT E_REDIR \
               R_DRAIN R_SRREQ R_SRWAIT R_PCREQ R_PCWAIT R_PCREQ2 R_PCWAIT2 \
               R_FMTREQ R_FMTWAIT R_REDIR S_DRAIN S_APPLY S_MAINTWAIT S_REDIR)
state="(none)"
for i in "${!ST[@]}"; do
    [ "$(b "$EX" $((6 + i)))" = 1 ] && state="${ST[$i]}"
done
echo "   state=$state  active=$(b "$EX" 0) redirectValid=$(b "$EX" 5)"
echo "   sqDrained=$(b "$EX" 1) dcQuiesced=$(b "$EX" 2) quiesceHoldOut=$(b "$EX" 3) maintDoneIn=$(b "$EX" 4)"
# Name the waiter AND what it waits on, in one line.
case "$state" in
  E_DRAIN|R_DRAIN|S_DRAIN)
    if [ "$(b "$EX" 1)" = 0 ] || [ "$(b "$EX" 2)" = 0 ]; then
      echo "   *** $state is WAITING on sqDrained=$(b "$EX" 1) && dcQuiesced=$(b "$EX" 2)"
      echo "       -- cross-reference the BLOCKING list above; that is the stall ***"
    fi ;;
  S_MAINTWAIT)
    [ "$(b "$EX" 4)" = 0 ] && echo "   *** S_MAINTWAIT is WAITING on maintDoneIn ***" ;;
esac

# ── 4. THE WALKERS THEMSELVES ────────────────────────────────────────────────
echo
echo "-- Table walkers --"
wdec () {
    local w=$1 name=$2 base=$3
    local st="(none)"
    declare -a WS=(IDLE RD_ROOT RD_PTR RD_PAGE FINISH)
    for i in "${!WS[@]}"; do
        [ "$(b "$w" $((base + i)))" = 1 ] && st="${WS[$i]}"
    done
    local cv=$(b "$w" $((base + 6))) cr=$(b "$w" $((base + 7)))
    printf "   %-5s state=%-8s cmdSent=%s loadCmd v/r=%s/%s rsp=%s start=%s\n" \
        "$name" "$st" "$(b "$w" $((base + 5)))" "$cv" "$cr" \
        "$(b "$w" $((base + 8)))" "$(b "$w" $((base + 9)))"
    if [ "$cv" = 1 ] && [ "$cr" = 0 ]; then
        echo "         *** $name walker is BLOCKED: descriptor read asserted, D-cache not ready ***"
    fi
}
wdec "$WK" DTLB 0
wdec "$WK" ITLB 16
