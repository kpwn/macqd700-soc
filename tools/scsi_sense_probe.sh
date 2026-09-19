#!/bin/bash
# scsi_sense_probe.sh — decode the widened vio_scsi_sd probe (97 bits).
#
# Field map after the 2026-08-18 observability change (new fields appended
# ABOVE the old ones, so every previously documented bit position is unchanged):
#   [96]    medium_not_present (sticky)
#   [95:92] sense_key      [91:84] sense_asc
#   [83:76] CHECK CONDITION count (saturating)
#   [75:44] first failing SD LBA   [43:28] completions   [27:20] sd_ctrl errors
#   [19:16] sd err_cause  [15:12] err cmd  [11:4] detail
#   [3] sticky  [2] busy  [1] irq  [0] drq
#
# Attribution of a live CHECK CONDITION:
#   key 2 / ASC 0x3A -> READ backing-store timeout arm (sets medium_not_present)
#   key 5 / ASC 0x21 -> !vh_chk_ok, LBA out of range
#   key 5 / ASC 0x20 -> invalid opcode
#   key 3 / ASC 0x11 -> genuine sd_ctrl error (its counters would also move)
#
# Measured context: the .ASYC00 driver takes CHECK CONDITION on
# READ(10) LBA 2314 x1 while the volume reports ENABLED / 4194304 blocks.

cd "$(dirname "$0")/.." || exit 1
RAW=$(JT_WAIT=${JT_WAIT:-40} tools/jt.sh "vio-read scsi" 2>&1 | grep -oE 'vio_scsi_sd = [0-9a-fA-F]+' | awk '{print $3}')
if [ -z "$RAW" ]; then echo "no probe value (old bitstream? wrong filter?)"; exit 1; fi
python3 - "$RAW" <<'PY'
import sys
h=sys.argv[1].strip(); v=int(h,16); bits=len(h)*4
g=lambda hi,lo:(v>>lo)&((1<<(hi-lo+1))-1)
print(f"  raw={h}  ({bits} bits)")
if bits >= 187:
    ok=g(130,130); blks=g(154,131); lba=g(186,155)
    print(f"  RANGE-CHECK INPUTS: chk_ok={ok}  xfer_blocks={blks}  xfer_lba={lba}")
    if ok==0:
        print("    *** chk_ok is FALSE -> scsi.v:2905 will reject the read ***")
        if blks==0:
            print("        cause: xfer_blocks == 0.  sd_scsi_lba_mapper requires")
            print("        scsi_blocks != 0, and vh_chk_blocks is wired to the LIVE")
            print("        xfer_blocks -- so the FSM is range-checking before the CDB")
            print("        loaded the block count (or after it was cleared at")
            print("        selection, scsi.v:2462).  THAT is the bug.")
        else:
            print("        xfer_blocks is non-zero, so check the LBA/num_lbas operands.")
if bits >= 130:
    nl=g(129,98); ds=g(97,97)
    print(f"  vh_dev_sel = {ds}  ({'target B / RAM disk' if ds else 'target A / SD volume'})")
    print(f"  vh_num_lbas as seen by the range check = {nl} (0x{nl:08X})")
    if ds==1:
        print("    *** routing at the RAM-disk volume -- vhdd_mux.v:178 feeds ITS capacity")
        print("        to scsi.v's range check, so a read to the SD disk fails closed ***")
    if nl==0:
        print("    *** capacity 0 -> EVERY read is out of range -> ILLEGAL REQUEST/ASC 0x21 ***")
if bits < 97:
    print("  *** this bitstream predates the sense-probe change (76-bit probe) ***")
    print(f"  completions={g(43,28)} errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)}")
    raise SystemExit
key=g(95,92); asc=g(91,84); mnp=g(96,96); cc=g(83,76)
keys={0:'NO SENSE',2:'NOT READY',3:'MEDIUM ERROR',5:'ILLEGAL REQUEST',6:'UNIT ATTENTION'}
ascs={0x00:'-',0x3A:'medium not present',0x21:'LBA out of range',
      0x20:'invalid opcode',0x11:'unrecovered read error'}
print(f"  medium_not_present = {mnp}")
print(f"  sense_key = {key} ({keys.get(key,'?')})   sense_asc = 0x{asc:02X} ({ascs.get(asc,'?')})")
print(f"  CHECK CONDITION count = {cc}")
print(f"  completions={g(43,28)} sd_errors={g(27,20)} cause={g(19,16)} sticky={g(3,3)} busy={g(2,2)}")
print()
if key==2 and asc==0x3A:
    print("  => the READ backing-store TIMEOUT arm is firing (medium_not_present).")
    print("     The vh_wait_ctr contract fix did not stop it, so it is tripping")
    print("     for another reason -- investigate the vh_busy/vh_done handshake.")
elif key==5 and asc==0x21:
    print("  => !vh_chk_ok: the RANGE CHECK is rejecting a valid LBA.")
    print("     The volume reports 4194304 blocks and the LBA is 2314, so the")
    print("     extent check or vh_num_lbas routing (vh_dev_sel/vhdd_mux) is wrong.")
elif key==5 and asc==0x20:
    print("  => INVALID OPCODE: scsi.v is rejecting a command it should support.")
elif key==3:
    print("  => MEDIUM ERROR from a real sd_ctrl fault; check its err_cause/LBA.")
elif cc==0:
    print("  => no CHECK CONDITION recorded yet on this boot.")
PY
