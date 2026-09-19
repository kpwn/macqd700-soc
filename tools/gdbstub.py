#!/usr/bin/env python3
"""gdbstub -- point real GDB at the m68k-ooo FPGA Macintosh.

    GDB  ←TCP→  gdbstub.py  ←FIFO→  jtag_repl.tcl  ←JTAG-AXI→  FPGA

Quick start (full instructions: docs/debugging_the_mac_on_fpga.md)::

    # 1. build a symbol file so backtraces are readable
    tools/macsym.py build -o build/macos.elf

    # 2. with jtag_repl.tcl already running on /tmp/jtag_in /tmp/jtag_out:
    tools/gdbstub.py --port 1234

    # 3. in another terminal
    gdb-multiarch build/macos.elf
    (gdb) target remote :1234
    (gdb) bt
    (gdb) monitor atrap arm _SCSIDispatch
    (gdb) continue

No board handy?  ``--fake`` runs the whole stack against a simulated CPU so
you can learn the workflow, or develop against the protocol, offline::

    tools/gdbstub.py --fake --port 1234

What GDB gets
-------------
registers (D0-D7/A0-A7/PC/SR plus USP/SSP/ISP/VBR/MMU CSRs), memory read and
write, up to 4 hardware breakpoints, 2 data watchpoints, single-step,
continue, Ctrl-C, and accurate stop reasons.

What it deliberately does NOT do
--------------------------------
* **No software breakpoints.**  GDB gets 4 hardware breakpoints and an honest
  error when they run out.  Patching an illegal opcode into memory would take
  the exception, push a frame, and leave the machine somewhere the stub cannot
  cleanly unwind from — it would look like it worked right up until it lied
  about where you were.
* **No FPU registers.**  FP0-7/FPCR/FPSR/FPIAR are not plumbed.
* Watchpoints match **physical** addresses (post-MMU).  With translation on,
  a virtual address from GDB is not necessarily the address the hardware
  compares.  ``monitor watch`` reports what was actually armed.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gdbdbg  # noqa: E402
from gdbdbg import Target, TargetError, StopReason  # noqa: E402

try:
    from macsym import SymbolTable, SymbolError
except ImportError:  # pragma: no cover - macsym ships alongside this file
    SymbolTable = None
    SymbolError = Exception

try:
    sys.path.insert(0, str(Path(__file__).resolve().parent / "macsyms"))
    import atraps as atrap_names
except Exception:  # pragma: no cover - optional
    atrap_names = None


# ── register layout ─────────────────────────────────────────────────────
# 0-17 are the stock m68k-int set every GDB/IDA m68k front end understands.
# 18+ are our extension registers, described to the client via target.xml.
REGS = (
    [f"D{i}" for i in range(8)] +
    [f"A{i}" for i in range(8)] +
    ["PS", "PC"] +
    ["USP", "SSP", "ISP", "VBR", "SFC", "DFC", "CACR",
     "TC", "ITT0", "ITT1", "DTT0", "DTT1", "URP", "SRP"]
)
N_REGS = len(REGS)
G_PACKET_BYTES = N_REGS * 4

#: m68k exception vector → POSIX signal, so the IDE paints the right icon.
VEC_TO_SIGNAL = {
    2: 11,   # bus error            → SIGSEGV
    3: 10,   # address error        → SIGBUS
    4: 4,    # illegal instruction  → SIGILL
    5: 8,    # zero divide          → SIGFPE
    6: 8,    # CHK / CHK2           → SIGFPE
    7: 8,    # TRAPV                → SIGFPE
    8: 7,    # privilege violation  → SIGEMT
    9: 5,    # trace                → SIGTRAP
    10: 4,   # A-line               → SIGILL
    11: 4,   # F-line               → SIGILL
    14: 4,   # format error         → SIGILL
    24: 2,   # spurious interrupt   → SIGINT
}
SIGTRAP, SIGINT = 5, 2


# ── GRSP packet plumbing ────────────────────────────────────────────────
def gdb_checksum(payload: bytes) -> bytes:
    return f"{sum(payload) & 0xff:02x}".encode("ascii")


def gdb_pack(payload: bytes) -> bytes:
    return b"$" + payload + b"#" + gdb_checksum(payload)


def gdb_unescape(payload: bytes) -> bytes:
    if b"}" not in payload:
        return payload
    out = bytearray()
    esc = False
    for b in payload:
        if esc:
            out.append(b ^ 0x20)
            esc = False
        elif b == 0x7D:
            esc = True
        else:
            out.append(b)
    return bytes(out)


def gdb_parse_packets(buf: bytes):
    """Pull complete packets out of `buf`.

    Returns (packets, interrupts, bad, remainder).  Checksums ARE verified;
    a corrupt packet is reported in `bad` so the caller can NAK it and let GDB
    retransmit, rather than acting on data that arrived damaged.  Bare 0x03
    bytes (GDB's Ctrl-C) are counted separately."""
    packets: list[bytes] = []
    interrupts = 0
    bad = 0
    while True:
        start = buf.find(b"$")
        if start < 0:
            interrupts += buf.count(b"\x03")
            return packets, interrupts, bad, b""
        interrupts += buf[:start].count(b"\x03")
        buf = buf[start:]
        end = buf.find(b"#")
        if end < 0 or end + 2 >= len(buf):
            return packets, interrupts, bad, buf
        payload = buf[1:end]
        checksum = buf[end + 1:end + 3]
        if checksum.lower() == gdb_checksum(payload).lower():
            packets.append(gdb_unescape(payload))
        else:
            bad += 1
        buf = buf[end + 3:]


class GdbStub:
    def __init__(self, target: Target, port: int, host: str = "127.0.0.1",
                 syms: "SymbolTable | None" = None, verbose: bool = True):
        self.t = target
        self.port = port
        self.host = host
        self.syms = syms
        self.verbose = verbose
        self.no_ack = False
        self.conn: socket.socket | None = None
        #: addr → slot for HW breakpoints we own
        self.bp: dict[int, int] = {}
        #: addr → (slot, kind) for watchpoints; kind in {2,3,4}
        self.wp: dict[int, tuple[int, int]] = {}
        self.halt_exc_mask = 0
        self.halt_exc_enabled = False
        self._warned_mmu_wp = False
        self._notices: list[str] = []

    # -- helpers ---------------------------------------------------------
    def log(self, msg: str):
        if self.verbose:
            print(f"[gdbstub] {msg}", flush=True)

    def sym(self, addr: int) -> str:
        return self.syms.format(addr) if self.syms else f"0x{addr:08x}"

    @staticmethod
    def _brief(comment: str, width: int = 64) -> str:
        """Symbol-map comments carry full provenance (which doc an address
        came from), which is right for the file and far too wide for a
        terminal.  Trim for display only."""
        if not comment:
            return ""
        one = " ".join(comment.split())
        return one if len(one) <= width else one[:width - 1] + "..."

    def notify(self, text: str):
        """Queue a message for GDB's console.

        **`O` packets may only be sent when GDB is not waiting for the reply
        to a synchronous packet** — i.e. while the target is running, or as
        part of a `qRcmd` response.  Sending one as (or before) the answer to
        an `m` packet makes GDB read it AS the answer: it tried to parse
        `O6764...` as memory contents and reported "Invalid hex digit".  So
        messages raised from packet handlers are queued here and flushed at
        the next legal opportunity."""
        self._notices.append(text)
        self.log(text.strip())

    def flush_notices(self):
        for text in self._notices:
            self.send_console(text)
        self._notices.clear()

    def send_console(self, text: str):
        """Write directly to GDB's console via an `O` packet.

        Only safe where the protocol allows it -- see notify()."""
        if not self.conn:
            return
        try:
            payload = b"O" + text.encode("ascii", "replace").hex().encode("ascii")
            self.conn.sendall(gdb_pack(payload))
        except OSError:
            pass

    # -- server ----------------------------------------------------------
    def serve(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(1)
        self.log(f"listening on {self.host}:{self.port}")
        try:
            while True:
                conn, addr = sock.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.log(f"connection from {addr}")
                self.no_ack = False
                self.conn = conn
                try:
                    self.on_attach()
                    self.handle(conn)
                except (BrokenPipeError, ConnectionResetError, OSError) as e:
                    self.log(f"disconnect: {e}")
                except TargetError as e:
                    self.log(f"FATAL target error: {e}")
                finally:
                    conn.close()
                    self.conn = None
                    self.log("session ended")
        except KeyboardInterrupt:
            self.log("interrupted; shutting down")
        finally:
            sock.close()

    def on_attach(self):
        """Halt and report what we are attached to.

        Printing the version / build-id / features up front matters: a good
        share of the historical confusion in this project came from talking to
        a bitstream that did not have the feature being exercised."""
        feats = self.t.features(refresh=True)
        names = [n for b, n in sorted(gdbdbg.FEATURE_BITS.items())
                 if feats & (1 << b)]
        self.log(f"debug_ctrl version 0x{self.t.version():08x} "
                 f"build_id 0x{self.t.build_id():08x}")
        self.log(f"features 0x{feats:05x}: {', '.join(names)}")
        if not self.t.is_halted():
            self.log("halting CPU for attach")
            self.t.halt()
        sr = self.t.stop_reason()
        self.log(f"stopped: {sr.describe(self.syms)}")

    def handle(self, conn: socket.socket):
        buf = b""
        while True:
            data = conn.recv(4096)
            if not data:
                return
            buf += data
            packets, _interrupts, bad, buf = gdb_parse_packets(buf)
            for _ in range(bad):
                if not self.no_ack:
                    conn.sendall(b"-")     # ask GDB to resend
            for pkt in packets:
                if not self.no_ack:
                    conn.sendall(b"+")
                reply = self.dispatch(pkt)
                if reply is not None:
                    conn.sendall(gdb_pack(reply))

    # -- dispatch --------------------------------------------------------
    def dispatch(self, pkt: bytes) -> bytes | None:
        if not pkt:
            return b""
        op = chr(pkt[0])
        try:
            if op == "q": return self.pkt_query(pkt)
            if op == "Q": return self.pkt_qset(pkt)
            if op == "?": return self.stop_reply()
            if op == "g": return self.pkt_read_regs()
            if op == "G": return self.pkt_write_regs(pkt[1:])
            if op == "p": return self.pkt_read_one(pkt[1:])
            if op == "P": return self.pkt_write_one(pkt[1:])
            if op == "m": return self.pkt_read_mem(pkt[1:])
            if op == "M": return self.pkt_write_mem(pkt[1:])
            if op == "X": return b""          # binary write: decline, GDB uses M
            if op == "c": return self.pkt_continue(pkt[1:])
            if op == "s": return self.pkt_step(pkt[1:])
            if op == "Z": return self.pkt_add_break(pkt[1:])
            if op == "z": return self.pkt_rm_break(pkt[1:])
            if op == "v": return self.pkt_v(pkt)
            if op == "H": return b"OK"
            if op == "T": return b"OK"        # thread alive
            if op == "k": return None
            if op == "D":
                self.detach()
                return b"OK"
        except TargetError as e:
            # Surface the real reason in the user's console as well as
            # returning an error code — a bare "E01" has cost this project
            # whole sessions of guessing.
            self.log(f"target error on {op!r}: {e}")
            self.notify(f"gdbstub: {e}\n")
            return b"E01"
        except Exception as e:  # pragma: no cover - defensive
            self.log(f"unhandled error on {op!r}: {e!r}")
            self.notify(f"gdbstub internal error: {e!r}\n")
            return b"E03"
        return b""

    def detach(self):
        for _addr, slot in list(self.bp.items()):
            try:
                self.t.clear_bp(slot)
            except TargetError:
                pass
        for _addr, (slot, _k) in list(self.wp.items()):
            try:
                self.t.clear_watchpoint(slot)
            except TargetError:
                pass
        self.bp.clear()
        self.wp.clear()
        try:
            self.t.discard_pending_regs()
            self.t.resume()
        except TargetError as e:
            self.log(f"detach: could not resume: {e}")

    # -- queries ---------------------------------------------------------
    def pkt_query(self, pkt: bytes) -> bytes:
        if pkt == b"qC":
            return b"QC1"
        if pkt.startswith(b"qSupported"):
            return (b"PacketSize=4000;hwbreak+;swbreak-;"
                    b"qXfer:features:read+;QStartNoAckMode+;vContSupported+")
        if pkt == b"qAttached":
            return b"1"
        if pkt.startswith(b"qfThreadInfo"):
            return b"m1"
        if pkt.startswith(b"qsThreadInfo"):
            return b"l"
        if pkt.startswith(b"qXfer:features:read:target.xml"):
            return self._xfer(self._target_xml(), pkt)
        if pkt.startswith(b"qRcmd,"):
            try:
                cmd = bytes.fromhex(pkt[6:].decode("ascii")).decode(
                    "ascii", "replace").strip()
            except ValueError:
                return b"E01"
            return self.pkt_rcmd(cmd)
        return b""

    @staticmethod
    def _xfer(blob: bytes, pkt: bytes) -> bytes:
        """Serve a qXfer read window (`...:<offset>,<length>`)."""
        try:
            tail = pkt.rsplit(b":", 1)[1]
            off_s, len_s = tail.split(b",")
            off, length = int(off_s, 16), int(len_s, 16)
        except (IndexError, ValueError):
            off, length = 0, len(blob)
        chunk = blob[off:off + length]
        return (b"m" if off + length < len(blob) else b"l") + chunk

    def pkt_qset(self, pkt: bytes) -> bytes:
        if pkt == b"QStartNoAckMode":
            self.no_ack = True
            return b"OK"
        return b""

    #: GDB's own m68k register names.  a6 and a7 are called **fp** and **sp**
    #: — this is not cosmetic: gdb/m68k-tdep.c validates the core feature
    #: against exactly these names, and a description that says "a6"/"a7" is
    #: rejected wholesale ("Architecture rejected target-supplied
    #: description"), after which GDB falls back to its built-in 29-register
    #: layout and every `g` packet we send is reported as truncated.
    XML_CORE_NAMES = ([f"d{i}" for i in range(8)] +
                      [f"a{i}" for i in range(6)] + ["fp", "sp"] +
                      ["ps", "pc"])

    def _target_xml(self) -> bytes:
        rows = []
        for i, name in enumerate(self.XML_CORE_NAMES):
            if name in ("fp", "sp"):
                ty = "data_ptr"
            elif name == "pc":
                ty = "code_ptr"
            else:
                ty = "int32"
            rows.append(f'<reg name="{name}" bitsize="32" regnum="{i}" '
                        f'type="{ty}" group="general"/>')
        ext = [("usp", "data_ptr"), ("ssp", "data_ptr"), ("isp", "data_ptr"),
               ("vbr", "data_ptr"), ("sfc", "int32"), ("dfc", "int32"),
               ("cacr", "int32"), ("tc", "int32"), ("itt0", "int32"),
               ("itt1", "int32"), ("dtt0", "int32"), ("dtt1", "int32"),
               ("urp", "data_ptr"), ("srp", "data_ptr")]
        ext_rows = [
            f'<reg name="{name}" bitsize="32" regnum="{18 + i}" '
            f'type="{ty}" group="system"/>'
            for i, (name, ty) in enumerate(ext)]
        body = ('<?xml version="1.0"?>'
                '<!DOCTYPE target SYSTEM "gdb-target.dtd">'
                '<target version="1.0">'
                '<architecture>m68k</architecture>'
                '<feature name="org.gnu.gdb.m68k.core">'
                + "".join(rows) +
                '</feature>'
                '<feature name="org.m68k-ooo.system">'
                + "".join(ext_rows) +
                '</feature></target>')
        return body.encode("ascii")

    # -- registers -------------------------------------------------------
    def _reg_dict(self) -> dict[str, int]:
        regs = self.t.read_regs()
        regs["PS"] = regs.get("SR", 0)
        return regs

    def pkt_read_regs(self) -> bytes:
        regs = self._reg_dict()
        out = bytearray()
        for name in REGS:
            out += struct.pack(">I", regs.get(name, 0) & 0xFFFFFFFF)
        return out.hex().encode("ascii")

    def pkt_write_regs(self, hex_data: bytes) -> bytes:
        if len(hex_data) != G_PACKET_BYTES * 2:
            return b"E02"
        raw = bytes.fromhex(hex_data.decode())
        for i, name in enumerate(REGS):
            v = struct.unpack(">I", raw[i * 4:(i + 1) * 4])[0]
            tgt = "SR" if name == "PS" else name
            if tgt in Target.WRITABLE_REGS:
                self.t.stage_reg(tgt, v)
        return b"OK"

    def pkt_read_one(self, body: bytes) -> bytes:
        try:
            idx = int(body, 16)
        except ValueError:
            return b"E02"
        if idx >= N_REGS:
            return b"E02"
        regs = self._reg_dict()
        name = REGS[idx]
        if name not in regs:
            # Unknown is unknown.  A zero here would be read as a real value.
            raise TargetError(
                f"register {name} is not available from this bitstream")
        return struct.pack(">I", regs[name] & 0xFFFFFFFF).hex().encode("ascii")

    def pkt_write_one(self, body: bytes) -> bytes:
        idx_hex, _, val_hex = body.partition(b"=")
        try:
            idx = int(idx_hex, 16)
            v = int.from_bytes(bytes.fromhex(val_hex.decode()), "big")
        except ValueError:
            return b"E02"
        if idx >= N_REGS:
            return b"E02"
        name = "SR" if REGS[idx] == "PS" else REGS[idx]
        if name not in Target.WRITABLE_REGS:
            self.notify(
                f"gdbstub: {name} is read-only on this target; write ignored\n")
            return b"E02"
        self.t.stage_reg(name, v)
        return b"OK"

    # -- memory ----------------------------------------------------------
    def pkt_read_mem(self, body: bytes) -> bytes:
        addr_hex, _, len_hex = body.partition(b",")
        try:
            addr, n = int(addr_hex, 16), int(len_hex, 16)
        except ValueError:
            return b"E02"
        if n == 0:
            return b""
        return self.t.read_bytes(addr, n).hex().encode("ascii")

    def pkt_write_mem(self, body: bytes) -> bytes:
        head, _, hex_data = body.partition(b":")
        addr_hex, _, len_hex = head.partition(b",")
        try:
            addr, n = int(addr_hex, 16), int(len_hex, 16)
            raw = bytes.fromhex(hex_data.decode())
        except ValueError:
            return b"E02"
        if len(raw) != n:
            return b"E02"
        self.t.write_bytes(addr, raw)
        return b"OK"

    # -- execution -------------------------------------------------------
    def stop_reply(self, sr: StopReason | None = None) -> bytes:
        """Build a `T` stop reply that says what actually happened."""
        sr = sr or self.t.stop_reason()
        sig = SIGTRAP
        extra = b""
        if sr.kind == "watchpoint":
            kind = 2 if sr.wp_is_store else 3
            for _addr, (slot, k) in self.wp.items():
                if slot == sr.wp_slot and k == 4:
                    kind = 4
            key = {2: b"watch", 3: b"rwatch", 4: b"awatch"}
            extra = key[kind] + b":" + f"{sr.wp_addr:x}".encode("ascii") + b";"
        elif sr.kind == "breakpoint":
            extra = b"hwbreak:;"
        elif sr.kind == "exception":
            sig = VEC_TO_SIGNAL.get(sr.exc_vec, SIGTRAP)
        elif sr.kind == "double-fault":
            sig = 11
        return f"T{sig:02x}".encode("ascii") + extra

    def _announce(self, sr: StopReason):
        """Narrate the stop into GDB's console, with symbols.

        A stop reply is one of the moments `O` packets are legal, so anything
        queued by notify() goes out here too."""
        self.flush_notices()
        self.send_console(f"\n[{sr.describe(self.syms)}]\n")
        if sr.kind == "atrap" and atrap_names is not None:
            name = atrap_names.lookup(sr.atrap_opword)
            if name:
                self.send_console(f"[  trap {name}]\n")
        if sr.kind == "double-fault":
            self.send_console(
                "[  the CPU is wedged: it cannot be resumed, only reset. "
                "Use 'monitor reset'.]\n")
        if sr.atrap_capture_busy and not sr.atrap_valid:
            self.send_console(
                "[  an A-trap A0/D0 capture was still in flight; A0/D0 not "
                "reported rather than reporting a previous trap's values]\n")
        for w in self.t.warnings:
            self.send_console(f"[  warning: {w}]\n")
        self.t.warnings.clear()

    def pkt_continue(self, body: bytes) -> bytes:
        if body:
            try:
                self.t.stage_reg("PC", int(body, 16))
            except ValueError:
                return b"E02"
        self.t.clear_stop_latches()
        self.t.resume()
        return self._wait_for_stop()

    def pkt_step(self, body: bytes) -> bytes:
        if body:
            try:
                self.t.stage_reg("PC", int(body, 16))
            except ValueError:
                return b"E02"
        self.t.clear_stop_latches()
        exact = self.t.step()
        sr = self.t.stop_reason()
        if not exact:
            # Say it plainly.  The PC below is real; the claim "you advanced
            # one instruction" would not be.
            self.notify(
                "\ngdbstub: this was NOT a single instruction step.  Staged "
                "register writes had to be applied, and applying registers "
                "resumes the CPU on this hardware; execution was stopped "
                "again at the next instruction boundary, but an unknown "
                "number of instructions ran first.  The reported PC is "
                "accurate.  To step exactly, avoid writing registers "
                "immediately before a step.\n")
        self._announce(sr)
        return self.stop_reply(sr)

    def _wait_for_stop(self, poll_s: float = 0.05) -> bytes:
        """Poll until the CPU halts, or GDB sends Ctrl-C."""
        conn = self.conn
        assert conn is not None
        conn.setblocking(False)
        try:
            while True:
                if self.t.is_halted():
                    sr = self.t.stop_reason()
                    if sr.kind == "running":
                        # Halted status with no reason yet: the A-trap D0
                        # qualifier deliberately halts for a few cycles and
                        # then auto-resumes on a mismatch.  Do not report
                        # that as a stop.
                        time.sleep(poll_s)
                        continue
                    self._announce(sr)
                    return self.stop_reply(sr)
                try:
                    data = conn.recv(64, socket.MSG_PEEK)
                except (BlockingIOError, InterruptedError):
                    data = b""
                if data:
                    idx = data.find(b"\x03")
                    if idx >= 0:
                        conn.recv(idx + 1)
                        self.log("Ctrl-C from GDB -- halting")
                        self.t.halt()
                        sr = self.t.stop_reason()
                        self._announce(sr)
                        return f"T{SIGINT:02x}".encode("ascii")
                    if b"$" in data:
                        # A packet arrived mid-continue.  GDB only does this
                        # around an interrupt, but whatever the reason, we owe
                        # it a stop reply — and the only honest way to send
                        # one is to actually stop first.  Reporting the
                        # current state while the CPU is still running would
                        # hand back a PC that was never a stopping point.
                        self.log("packet arrived mid-continue — halting")
                        self.t.halt()
                        sr = self.t.stop_reason()
                        self._announce(sr)
                        return self.stop_reply(sr)
                time.sleep(poll_s)
        finally:
            try:
                conn.setblocking(True)
            except OSError:
                pass

    def pkt_v(self, pkt: bytes) -> bytes:
        if pkt.startswith(b"vCont?"):
            return b"vCont;c;C;s;S"
        if pkt.startswith(b"vCont"):
            body = pkt[len(b"vCont"):].lstrip(b";")
            action = body[:1]
            if action in (b"c", b"C"):
                return self.pkt_continue(b"")
            if action in (b"s", b"S"):
                return self.pkt_step(b"")
            return b""
        if pkt.startswith(b"vMustReplyEmpty"):
            return b""
        if pkt.startswith(b"vKill"):
            return b"OK"
        return b""

    # -- breakpoints / watchpoints ---------------------------------------
    def pkt_add_break(self, body: bytes) -> bytes:
        parts = body.split(b",")
        if len(parts) < 2:
            return b"E02"
        try:
            btype = int(parts[0])
            addr = int(parts[1], 16)
            kind = int(parts[2], 16) if len(parts) > 2 else 0
        except ValueError:
            return b"E02"

        if btype in (0, 1):
            if addr in self.bp:
                return b"OK"
            used = set(self.bp.values())
            free = next((s for s in range(Target.N_BP_SLOTS) if s not in used),
                        None)
            if free is None:
                self.notify(
                    f"gdbstub: out of hardware breakpoint slots "
                    f"({Target.N_BP_SLOTS} max, all in use).  This target has "
                    f"no software breakpoints -- delete one first.\n")
                return b"E28"
            self.t.set_bp(free, addr)
            self.bp[addr] = free
            self.log(f"bp slot {free} armed at {self.sym(addr)}")
            return b"OK"

        if btype in (2, 3, 4):
            if addr in self.wp:
                return b"OK"
            if not self.t.has_feature(gdbdbg.FEAT_WATCHPOINTS):
                return b""
            used = {s for s, _k in self.wp.values()}
            free = next((s for s in range(Target.N_WP_SLOTS) if s not in used),
                        None)
            if free is None:
                self.notify(
                    f"gdbstub: out of data watchpoint slots "
                    f"({Target.N_WP_SLOTS} max).\n")
                return b"E28"
            self._warn_wp_translation()
            if kind and kind < 4:
                self.notify(
                    f"gdbstub: watchpoint length {kind} widened -- the hardware "
                    f"comparator is longword-granular, so accesses to "
                    f"neighbouring bytes in the same longword also stop.\n")
            self.t.set_watchpoint(
                free, addr,
                on_load=(btype in (3, 4)),
                on_store=(btype in (2, 4)))
            self.wp[addr] = (free, btype)
            self.log(f"watchpoint slot {free} armed at {self.sym(addr)} "
                     f"(type {btype})")
            return b"OK"
        return b""

    def _warn_wp_translation(self):
        if self._warned_mmu_wp:
            return
        self._warned_mmu_wp = True
        try:
            tc = self.t.dbg_read(gdbdbg.OFF_LIVE_MMU_TC, "LIVE_MMU_TC")
        except TargetError:
            return
        if tc & 0x80000000:
            self.notify(
                "gdbstub: MMU translation is ENABLED (TC=0x%08x).  Data "
                "watchpoints compare PHYSICAL addresses, so a virtual address "
                "from GDB only matches where the mapping is identity.\n" % tc)

    def pkt_rm_break(self, body: bytes) -> bytes:
        parts = body.split(b",")
        if len(parts) < 2:
            return b"E02"
        try:
            btype = int(parts[0])
            addr = int(parts[1], 16)
        except ValueError:
            return b"E02"
        if btype in (0, 1):
            slot = self.bp.pop(addr, None)
            if slot is not None:
                self.t.clear_bp(slot)
            return b"OK"
        if btype in (2, 3, 4):
            entry = self.wp.pop(addr, None)
            if entry is not None:
                self.t.clear_watchpoint(entry[0])
            return b"OK"
        return b""

    # -- monitor ---------------------------------------------------------
    def pkt_rcmd(self, cmd: str) -> bytes:
        try:
            text = self.monitor(cmd)
        except TargetError as e:
            text = f"error: {e}"
        except Exception as e:  # pragma: no cover
            text = f"internal error: {e!r}"
        if text is None:
            text = ""
        # Console output during a monitor command goes as `O` packets; the
        # reply itself is a status.  Returning the hex body AS the reply is a
        # non-standard shape that gdb-multiarch answers with
        # "Protocol error with Rcmd".
        for note in self._notices:
            self.send_console(note)
        self._notices.clear()
        self.send_console(text.rstrip("\n") + "\n")
        return b"OK"

    MONITOR_HELP = """\
monitor commands — Mac-aware debugging on the m68k-ooo FPGA

  help                        this text
  status                      halt state, stop reason, features, build id
  regs                        full register dump (incl. USP/SSP/ISP/VBR/MMU)
  bt [n]                      backtrace by walking the A6 frame chain
  sym <addr>...               symbolize addresses
  syms [pattern]              list loaded symbols
  lomem [name...]             read low-memory globals by name
  queue <fs|dt|vbl|drive>     walk an OS queue and decode its elements
  dce [refnum]                walk the Device Manager unit table / DCEs
  atrap list                  known Mac OS toolbox trap names
  atrap arm <name|0xAxxx> [mask <m>] [d0 <v>] [slot <n>]
  atrap off <slot> | status   A-trap (toolbox) breakpoints
  watch status                data watchpoint slots as armed in hardware
  bp status                   hardware breakpoint slots, read back from HW
  cache flush|on|off          D-cache push control (JTAG reads bypass it)
  features                    decode OFF_FEATURES
  halt-on-exc list|add <vec>|clear [<vec>]|enable|disable
  exc-ring [n] | pc-trace [n] recent exceptions / PC redirects
  reset [hold|release]        unified CPU reset
  repl <cmd...>               run a raw jtag_repl.tcl command
"""

    def monitor(self, cmd: str) -> str:
        parts = cmd.split()
        if not parts:
            return self.MONITOR_HELP
        sub, args = parts[0], parts[1:]
        fn = {
            "help": lambda a: self.MONITOR_HELP,
            "status": self.mon_status,
            "regs": self.mon_regs,
            "bt": self.mon_bt,
            "backtrace": self.mon_bt,
            "sym": self.mon_sym,
            "syms": self.mon_syms,
            "lomem": self.mon_lomem,
            "queue": self.mon_queue,
            "dce": self.mon_dce,
            "atrap": self.mon_atrap,
            "watch": self.mon_watch,
            "bp": self.mon_bp,
            "cache": self.mon_cache,
            "features": self.mon_features,
            "halt-on-exc": self.mon_halt_on_exc,
            "exc-ring": lambda a: self.mon_passthrough("exc-ring", a),
            "pc-trace": lambda a: self.mon_passthrough("pc-trace", a),
            "reset": lambda a: self.mon_passthrough("reset", a, wait=3.0),
            "repl": lambda a: self.mon_passthrough(" ".join(a), [], wait=2.0),
        }.get(sub)
        if fn is None:
            return f"unknown monitor command {sub!r}\n\n{self.MONITOR_HELP}"
        return fn(args)

    def mon_passthrough(self, name: str, args, wait: float = 1.0) -> str:
        line = " ".join([name] + list(args)).strip()
        lines = self.t.raw(line, wait_s=wait)
        return "\n".join(ln for ln in lines if ln != "> READY")

    def mon_status(self, args) -> str:
        sr = self.t.stop_reason()
        out = [
            f"build_id    0x{self.t.build_id():08x}   "
            f"debug_ctrl 0x{self.t.version():08x}",
            f"halted      {sr.halted}",
            f"stop        {sr.describe(self.syms)}",
            f"halt_reason 0x{sr.reason:05x}   hit_pc {self.sym(sr.halt_hit_pc)}",
            f"live PC     {self.sym(sr.live_pc)}",
        ]
        if sr.halted and sr.halt_hit_pc != sr.live_pc:
            out.append("note: HALT_HIT_PC and the live PC disagree; GDB is "
                       "shown HALT_HIT_PC (the instruction about to execute).")
        pend = self.t.pending_regs()
        if pend:
            out.append("staged register writes (applied on next continue/step): "
                       + ", ".join(f"{k}=0x{v:08x}"
                                   for k, v in sorted(pend.items())))
        return "\n".join(out)

    def mon_regs(self, args) -> str:
        r = self.t.read_regs()
        out = []
        for i in range(8):
            out.append(f"D{i} = 0x{r.get(f'D{i}', 0):08x}    "
                       f"A{i} = 0x{r.get(f'A{i}', 0):08x}")
        sr_v = r.get("SR", 0)
        flags = "".join(c for c, b in zip("XNZVC", (4, 3, 2, 1, 0))
                        if sr_v & (1 << b))
        out.append("")
        out.append(f"PC  = {self.sym(r.get('PC', 0))}")
        out.append(f"SR  = 0x{sr_v:04x}  [{flags or '-'}] "
                   f"S={1 if sr_v & 0x2000 else 0} "
                   f"M={1 if sr_v & 0x1000 else 0} "
                   f"IPL={(sr_v >> 8) & 7}")
        out.append(f"VBR = 0x{r.get('VBR', 0):08x}")
        out.append(f"USP = 0x{r.get('USP', 0):08x}  SSP = 0x{r.get('SSP', 0):08x}"
                   f"  ISP = 0x{r.get('ISP', 0):08x}")
        out.append(f"TC  = 0x{r.get('TC', 0):08x}  SRP = 0x{r.get('SRP', 0):08x}"
                   f"  URP = 0x{r.get('URP', 0):08x}")
        return "\n".join(out)

    def mon_bt(self, args) -> str:
        """Backtrace by walking the A6 link-frame chain.

        68k code (including most of Mac OS) builds frames with LINK A6 / UNLK
        A6, so [A6] is the caller's A6 and [A6+4] the return address.  This is
        a heuristic: it cannot see frameless leaf routines, and it stops the
        moment the chain stops looking like a chain rather than inventing
        plausible frames."""
        limit = int(args[0], 0) if args else 24
        r = self.t.read_regs()
        pc, a6, a7 = r.get("PC", 0), r.get("A6", 0), r.get("A7", 0)
        out = [f"#0  {self.sym(pc)}    (A6=0x{a6:08x} A7=0x{a7:08x})"]
        seen = set()
        frame = a6
        for depth in range(1, limit + 1):
            if frame == 0:
                out.append("    (frame chain ends: A6 = 0)")
                break
            if frame & 1:
                out.append(f"    (stopped: A6 = 0x{frame:08x} is odd -- not a "
                           f"valid frame pointer)")
                break
            if frame in seen:
                out.append(f"    (stopped: frame 0x{frame:08x} repeats -- cycle)")
                break
            seen.add(frame)
            try:
                words = self.t.read_words(frame & ~3, 2)
            except TargetError as e:
                out.append(f"    (stopped: cannot read frame 0x{frame:08x}: {e})")
                break
            nxt, ret = words[0], words[1]
            if ret == 0 and nxt == 0:
                out.append("    (frame chain ends: zeroed frame -- note JTAG "
                           "reads bypass the D-cache, so this can also mean "
                           "'not yet written back'; try 'monitor cache flush')")
                break
            out.append(f"#{depth}  {self.sym(ret)}    (frame 0x{frame:08x})")
            if nxt == 0:
                out.append("    (frame chain ends: outermost frame)")
                break
            if nxt <= frame:
                out.append(f"    (stopped: next A6 0x{nxt:08x} does not grow "
                           f"the stack -- chain not trustworthy past here)")
                break
            frame = nxt
        return "\n".join(out)

    def mon_sym(self, args) -> str:
        if not args:
            return "usage: monitor sym <addr>..."
        return "\n".join(self.sym(int(a, 0)) for a in args)

    def mon_syms(self, args) -> str:
        if self.syms is None or not len(self.syms):
            return ("no symbols loaded -- pass --syms, or create "
                    "tools/macsyms/*.syms and restart the stub")
        pat = args[0].lower() if args else ""
        rows = [s for s in self.syms.all_symbols() if pat in s.name.lower()]
        if not rows:
            return f"no symbols matching {pat!r} ({len(self.syms)} loaded)"
        out = [f"0x{s.addr:08x}  {s.name}"
               + (f"    ; {self._brief(s.comment)}" if s.comment else "")
               for s in rows[:200]]
        if len(rows) > 200:
            out.append(f"... and {len(rows) - 200} more")
        return "\n".join(out)

    def _sym_addr(self, name: str) -> int:
        """Resolve a symbol name, or explain exactly what to do about it.

        Refusing beats guessing: an invented low-memory-global address would
        read a real value from entirely the wrong place."""
        if self.syms is None:
            raise TargetError("no symbol file loaded; pass --syms")
        s = self.syms.by_name(name)
        if s is None:
            raise TargetError(
                f"symbol {name!r} is not in the loaded symbol files, and this "
                f"command will not guess an address for it.  Add it to "
                f"tools/macsyms/lowmem.syms and restart the stub.")
        return s.addr

    def _read_sized(self, addr: int, size: int) -> int:
        return int.from_bytes(self.t.read_bytes(addr, size or 4), "big")

    def mon_lomem(self, args) -> str:
        if self.syms is None:
            return "no symbol file loaded; pass --syms"
        if args:
            out = []
            for name in args:
                s = self.syms.by_name(name)
                if s is None:
                    out.append(f"{name}: not in the symbol files (add it to "
                               f"tools/macsyms/lowmem.syms)")
                    continue
                size = s.size or 4
                val = self._read_sized(s.addr, size)
                out.append(f"{s.name:<14} ${s.addr:04X} ({size}B) = "
                           f"0x{val:0{size * 2}x}"
                           + (f"   ; {self._brief(s.comment)}"
                              if s.comment else ""))
            return "\n".join(out)
        rows = [s for s in self.syms.all_symbols()
                if s.is_data and s.addr < 0x10000]
        if not rows:
            return ("no low-memory globals in the symbol files "
                    "(expected tools/macsyms/lowmem.syms)")
        out = []
        for s in rows:
            size = s.size or 4
            try:
                sval = f"0x{self._read_sized(s.addr, size):0{size * 2}x}"
            except TargetError as e:
                sval = f"<read failed: {e}>"
            out.append(f"{s.name:<14} ${s.addr:04X} = {sval}"
                       + (f"   ; {self._brief(s.comment)}" if s.comment else ""))
        return "\n".join(out)

    #: Mac OS QHdr is {qFlags:word, qHead:long, qTail:long} — 10 bytes.
    QUEUES = {
        "fs":    ("FSQHdr",   "File System queue",    "pb"),
        "dt":    ("DTQueue",  "Deferred Task queue",  "dt"),
        "vbl":   ("VBLQueue", "Vertical Blank queue", "vbl"),
        "drive": ("DrvQHdr",  "Drive queue",          "raw"),
    }

    def mon_queue(self, args) -> str:
        if not args or args[0] not in self.QUEUES:
            return "usage: monitor queue <" + "|".join(self.QUEUES) + ">"
        sym_name, title, elem = self.QUEUES[args[0]]
        base = self._sym_addr(sym_name)
        hdr = self.t.read_bytes(base, 10)
        qflags = int.from_bytes(hdr[0:2], "big")
        qhead = int.from_bytes(hdr[2:6], "big")
        qtail = int.from_bytes(hdr[6:10], "big")
        out = [f"{title}  ({sym_name} @ ${base:04X})",
               f"  qFlags=0x{qflags:04x}  qHead=0x{qhead:08x}  "
               f"qTail=0x{qtail:08x}"]
        if qhead == 0:
            out.append("  (queue is EMPTY)")
            return "\n".join(out)
        node, seen, n = qhead, set(), 0
        while node and n < 32:
            if node in seen:
                out.append(f"  ! cycle at 0x{node:08x}")
                break
            if node & 1:
                out.append(f"  ! odd pointer 0x{node:08x} -- queue is corrupt")
                break
            seen.add(node)
            try:
                out.append("  " + self._decode_qelem(node, elem))
                node = int.from_bytes(self.t.read_bytes(node, 4), "big")
            except TargetError as e:
                out.append(f"  ! cannot read element at 0x{node:08x}: {e}")
                break
            n += 1
        if node and n >= 32:
            out.append("  ... (stopped after 32 elements)")
        return "\n".join(out)

    def _decode_qelem(self, node: int, kind: str) -> str:
        def signed16(v):
            return v - 0x10000 if v & 0x8000 else v
        if kind == "pb":
            # Mac OS ParamBlockHeader: qLink(4) qType(2) ioTrap(2)
            # ioCmdAddr(4) ioCompletion(4) ioResult(2) ioNamePtr(4)
            # ioVRefNum(2) ioRefNum(2)
            b = self.t.read_bytes(node, 26)
            qtype = int.from_bytes(b[4:6], "big")
            iotrap = int.from_bytes(b[6:8], "big")
            iocompl = int.from_bytes(b[12:16], "big")
            ioresult = signed16(int.from_bytes(b[16:18], "big"))
            iorefnum = signed16(int.from_bytes(b[24:26], "big"))
            trap = (atrap_names.lookup(iotrap)
                    if atrap_names and (iotrap & 0xF000) == 0xA000 else None)
            return (f"pb 0x{node:08x}  qType={qtype}  "
                    f"ioTrap=0x{iotrap:04x}{'(' + trap + ')' if trap else ''}  "
                    f"ioResult={ioresult}  ioRefNum={iorefnum}  "
                    f"ioCompletion={self.sym(iocompl)}")
        if kind == "dt":
            # DeferredTask: qLink(4) qType(2) dtFlags(2) dtAddr(4) dtParam(4)
            b = self.t.read_bytes(node, 16)
            return (f"dt 0x{node:08x}  qType={int.from_bytes(b[4:6], 'big')}  "
                    f"dtFlags=0x{int.from_bytes(b[6:8], 'big'):04x}  "
                    f"dtAddr={self.sym(int.from_bytes(b[8:12], 'big'))}  "
                    f"dtParam=0x{int.from_bytes(b[12:16], 'big'):08x}")
        if kind == "vbl":
            # VBLTask: qLink(4) qType(2) vblAddr(4) vblCount(2) vblPhase(2)
            b = self.t.read_bytes(node, 14)
            return (f"vbl 0x{node:08x}  qType={int.from_bytes(b[4:6], 'big')}  "
                    f"vblAddr={self.sym(int.from_bytes(b[6:10], 'big'))}  "
                    f"vblCount={int.from_bytes(b[10:12], 'big')}  "
                    f"vblPhase={int.from_bytes(b[12:14], 'big')}")
        b = self.t.read_bytes(node, 8)
        return (f"el 0x{node:08x}  qType={int.from_bytes(b[4:6], 'big')}  "
                f"+6=0x{int.from_bytes(b[6:8], 'big'):04x}")

    def mon_dce(self, args) -> str:
        """Walk the Device Manager unit table.

        UTableBase points at an array of Handles to Device Control Entries
        indexed by unit number; a driver refNum is the one's-complement of its
        unit number."""
        base_ptr = self._sym_addr("UTableBase")
        utable = int.from_bytes(self.t.read_bytes(base_ptr, 4), "big")
        out = [f"UTableBase @ ${base_ptr:04X} -> 0x{utable:08x}"]
        if utable == 0 or utable & 1:
            out.append("  ! unit table pointer is not usable (0 or odd) -- is "
                       "the OS up yet?")
            return "\n".join(out)
        try:
            count = int.from_bytes(
                self.t.read_bytes(self._sym_addr("UnitNtryCnt"), 2), "big")
        except TargetError:
            count = 32
            out.append("  (UnitNtryCnt not in symbol files; scanning 32 units)")
        if args:
            refnum = int(args[0], 0)
            units = [(~refnum) & 0xFFFF if refnum < 0 else refnum]
        else:
            units = list(range(min(count, 64)))
        for unit in units:
            try:
                handle = int.from_bytes(
                    self.t.read_bytes(utable + unit * 4, 4), "big")
            except TargetError as e:
                out.append(f"  unit {unit:3d}: <unreadable: {e}>")
                continue
            if handle == 0:
                continue
            line = (f"  unit {unit:3d} refNum {-(unit + 1):5d} "
                    f"handle 0x{handle:08x}")
            try:
                dce = int.from_bytes(self.t.read_bytes(handle, 4), "big")
                if dce and not dce & 1:
                    b = self.t.read_bytes(dce, 0x20)
                    line += (f" dce 0x{dce:08x} "
                             f"driver={self.sym(int.from_bytes(b[0:4], 'big'))} "
                             f"flags=0x{int.from_bytes(b[4:6], 'big'):04x} "
                             f"qHead=0x{int.from_bytes(b[8:12], 'big'):08x}")
                else:
                    line += f" dce 0x{dce:08x} <not dereferenceable>"
            except TargetError as e:
                line += f" <dce read failed: {e}>"
            out.append(line)
        if len(out) == 1:
            out.append("  (no installed units found)")
        return "\n".join(out)

    def mon_atrap(self, args) -> str:
        if not args:
            return "usage: monitor atrap list|status|arm ...|off <slot>"
        sub = args[0]
        if sub == "list":
            if atrap_names is None:
                return "A-trap name table not available (tools/macsyms/atraps.py)"
            return "\n".join(f"0x{k:04x}  {v}"
                             for k, v in sorted(atrap_names.ATRAPS.items()))
        if sub == "status":
            out = []
            for s in self.t.read_atraps():
                nm = (atrap_names.lookup(s["value"])
                      if atrap_names and s["mask"] == 0xFFFF else None)
                out.append(
                    f"slot {s['slot']}: "
                    f"{'ARMED  ' if s['enabled'] else 'off    '}"
                    f"value=0x{s['value']:04x} mask=0x{s['mask']:04x}"
                    f"{'  ' + nm if nm else ''}"
                    + (f"  d0==0x{s['d0val']:08x}" if s["d0qual"] else ""))
            sr = self.t.stop_reason()
            if sr.atrap_valid:
                nm = atrap_names.lookup(sr.atrap_opword) if atrap_names else None
                out.append(f"last hit: slot {sr.atrap_slot} opword "
                           f"0x{sr.atrap_opword:04x}{'  ' + nm if nm else ''} "
                           f"pc={self.sym(sr.atrap_pc)} "
                           f"A0=0x{sr.atrap_a0:08x} D0=0x{sr.atrap_d0:08x}")
            else:
                out.append("last hit: none latched")
            return "\n".join(out)
        if sub == "off":
            if len(args) < 2:
                return "usage: monitor atrap off <slot>"
            self.t.clear_atrap(int(args[1], 0))
            return f"A-trap slot {args[1]} disarmed"
        if sub == "arm":
            if len(args) < 2:
                return ("usage: monitor atrap arm <name|0xAxxx> [mask <m>] "
                        "[d0 <v>] [slot <n>]")
            tok = args[1]
            if tok.lower().startswith("0x"):
                value = int(tok, 16)
            elif atrap_names is not None:
                value = atrap_names.by_name(tok)
                if value is None:
                    return f"unknown trap name {tok!r} -- try 'monitor atrap list'"
            else:
                return "no A-trap name table; give a numeric 0xAxxx opcode"
            mask, d0, slot = 0xFFFF, None, None
            rest = args[2:]
            while rest:
                key = rest[0]
                if key == "mask" and len(rest) > 1:
                    mask = int(rest[1], 0); rest = rest[2:]
                elif key == "d0" and len(rest) > 1:
                    d0 = int(rest[1], 0); rest = rest[2:]
                elif key == "slot" and len(rest) > 1:
                    slot = int(rest[1], 0); rest = rest[2:]
                else:
                    return f"unexpected token {key!r}"
            if slot is None:
                armed = {s["slot"] for s in self.t.read_atraps() if s["enabled"]}
                slot = next((s for s in range(Target.N_AT_SLOTS)
                             if s not in armed), None)
                if slot is None:
                    return ("both A-trap slots are in use; "
                            "'monitor atrap off <slot>' first")
            self.t.set_atrap(slot, value, mask, d0)
            nm = atrap_names.lookup(value) if atrap_names else None
            return (f"A-trap slot {slot} armed: value=0x{value:04x} "
                    f"mask=0x{mask:04x}{'  ' + nm if nm else ''}"
                    + (f"  qualified on D0==0x{d0:08x}" if d0 is not None else "")
                    + "\n(config survives CPU reset, so you can arm it before "
                      "'monitor reset' to catch an early-boot call)")
        return f"unknown atrap subcommand {sub!r}"

    def mon_watch(self, args) -> str:
        out = []
        for w in self.t.read_watchpoints():
            if not w["enabled"]:
                out.append(f"slot {w['slot']}: off")
                continue
            what = ("load+store" if w["loads"] and w["stores"]
                    else "load" if w["loads"] else "store")
            out.append(
                f"slot {w['slot']}: ARMED addr=0x{w['addr']:08x} "
                f"amask=0x{w['amask']:08x} (1=ignore) {what}"
                + (f" value==0x{w['value']:08x} lanes=0x{w['lanes']:02x}"
                   if w["value_cmp"] else ""))
        sr = self.t.stop_reason()
        out.append(f"last hit: {sr.describe(self.syms)}"
                   if sr.wp_valid else "last hit: none latched")
        out.append("note: watchpoints compare PHYSICAL addresses (post-MMU), "
                   "and the compare is longword-granular.")
        out.append("note: read-modify-write byte stores (bset/bclr on memory) "
                   "are a known miss on this hardware.")
        return "\n".join(out)

    def mon_bp(self, args) -> str:
        out = []
        for i, (pc, en) in enumerate(self.t.read_bp_slots()):
            owner = next((a for a, s in self.bp.items() if s == i), None)
            out.append(f"slot {i}: {'ARMED' if en else 'off  '} "
                       f"pc={self.sym(pc)}"
                       + (f"   (GDB breakpoint at {self.sym(owner)})"
                          if owner is not None else ""))
        out.append(f"{len(self.bp)}/{Target.N_BP_SLOTS} slots used by GDB. "
                   f"This target has no software breakpoints.")
        return "\n".join(out)

    def mon_cache(self, args) -> str:
        if not args:
            return (f"auto D-cache push before memory reads: "
                    f"{'on' if self.t.auto_cache_flush else 'OFF'}\n"
                    f"JTAG reads bypass the CPU write-back D-cache, so with "
                    f"this off, recently written RAM can read as zeros.")
        if args[0] == "on":
            self.t.auto_cache_flush = True
            return "auto D-cache push enabled"
        if args[0] == "off":
            self.t.auto_cache_flush = False
            return ("auto D-cache push DISABLED -- memory reads may now show "
                    "stale/zero data for anything the CPU has written but not "
                    "written back")
        if args[0] == "flush":
            self.t.flush_dcache(force=True)
            return "D-cache pushed to RAM"
        return "usage: monitor cache [on|off|flush]"

    def mon_features(self, args) -> str:
        f = self.t.features(refresh=True)
        out = [f"OFF_FEATURES = 0x{f:08x}"]
        for bit, name in sorted(gdbdbg.FEATURE_BITS.items()):
            out.append(f"  bit {bit:2d} {name:<20} "
                       f"{'yes' if f & (1 << bit) else 'no'}")
        out.append("note: perf_counters (bit 12) is 0 by design -- "
                   "dbg_mispred_count is tied to 0 in the wrapper.")
        return "\n".join(out)

    def mon_halt_on_exc(self, args) -> str:
        if not args or args[0] == "list":
            mask = self.t.get_halt_exc_mask()
            ctl = self.t.dbg_read(gdbdbg.OFF_HALT_CTL, "OFF_HALT_CTL")
            vecs = [v for v in range(256) if mask & (1 << v)]
            return (f"halt-on-exception: "
                    f"{'ENABLED' if ctl & gdbdbg.HALT_EXC_EN else 'disabled'}\n"
                    f"armed vectors: {vecs if vecs else '(none)'}")
        if args[0] == "add" and len(args) > 1:
            vec = int(args[1], 0)
            if not 0 <= vec < 256:
                return f"vector {vec} out of range 0..255"
            self.halt_exc_mask |= 1 << vec
            self.t.set_halt_exc_mask(self.halt_exc_mask, self.halt_exc_enabled)
            return f"armed vector {vec}"
        if args[0] == "clear":
            if len(args) > 1:
                self.halt_exc_mask &= ~(1 << int(args[1], 0))
            else:
                self.halt_exc_mask = 0
            self.t.set_halt_exc_mask(self.halt_exc_mask, self.halt_exc_enabled)
            return "cleared"
        if args[0] in ("enable", "disable"):
            self.halt_exc_enabled = args[0] == "enable"
            self.t.set_halt_exc_mask(self.halt_exc_mask, self.halt_exc_enabled)
            return f"halt-on-exception {args[0]}d"
        return ("usage: monitor halt-on-exc "
                "list|add <vec>|clear [<vec>]|enable|disable")


# ── entry point ─────────────────────────────────────────────────────────
def build_target(args) -> Target:
    if args.fake:
        here = Path(__file__).resolve().parent.parent
        sys.path.insert(0, str(here / "tb" / "tests" / "host"))
        import fake_jtag_repl
        print("[gdbstub] *** --fake: simulated CPU, NOT hardware.  Nothing "
              "observed here says anything about the real board. ***")
        return Target(gdbdbg.FakeReplTransport(fake_jtag_repl.FakeRepl()))
    return Target(gdbdbg.FifoRepl(Path(args.fifo_in), Path(args.fifo_out)))


def load_symbols(args):
    if SymbolTable is None:
        return None
    if args.syms:
        paths = [Path(p) for p in args.syms]
    else:
        paths = sorted((Path(__file__).resolve().parent / "macsyms").glob("*.syms"))
    paths = [p for p in paths if p.exists()]
    if not paths:
        print("[gdbstub] no symbol files found; addresses will be bare hex")
        return None
    try:
        table = SymbolTable.from_files(paths)
    except SymbolError as e:
        # A broken symbol file must stop the show, not half-load: partial
        # symbols put confident wrong names on right addresses.
        print(f"[gdbstub] FATAL: symbol file error: {e}", file=sys.stderr)
        raise SystemExit(2)
    print(f"[gdbstub] loaded {len(table)} symbols from "
          f"{', '.join(p.name for p in paths)}")
    return table


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1234)
    ap.add_argument("--fifo-in", default="/tmp/jtag_in")
    ap.add_argument("--fifo-out", default="/tmp/jtag_out")
    ap.add_argument("--syms", nargs="*", default=None,
                    help="symbol map files (default: tools/macsyms/*.syms)")
    ap.add_argument("--fake", action="store_true",
                    help="run against the offline simulated CPU, no FPGA")
    ap.add_argument("--no-cache-flush", action="store_true",
                    help="do not push the D-cache before memory reads (faster, "
                         "but freshly written RAM may read as zeros)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    syms = load_symbols(args)
    try:
        target = build_target(args)
    except TargetError as e:
        print(f"[gdbstub] cannot reach the target: {e}", file=sys.stderr)
        return 1
    if args.no_cache_flush:
        target.auto_cache_flush = False
    GdbStub(target, args.port, host=args.host, syms=syms,
            verbose=not args.quiet).serve()
    return 0


if __name__ == "__main__":
    sys.exit(main())
