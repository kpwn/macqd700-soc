#!/usr/bin/env python3
"""Serve the FPGA HDMI capture card as an MJPEG-over-HTTP stream.

SINGLE PRODUCER, MANY CONSUMERS.  One GStreamer process opens the capture
device once, at startup, and every HTTP client is served from a shared
latest-frame slot.  This is the whole point of the design: a V4L2 capture
node admits exactly ONE streaming consumer, so the previous
one-gst-per-request layout meant the second viewer silently got a black
or dead stream (and, worse, so did anything else on the host trying to
grab a still).  Now a human can watch in a browser while an agent polls
/snapshot.jpg, at the same time, with no interference.

Endpoints:
  /               small HTML page with a live <img> — open this in a browser
  /stream.mjpg    multipart/x-mixed-replace MJPEG stream (many clients OK)
  /snapshot.jpg   the single latest JPEG — one request, no stream parsing.
                  Prefer this for scripted/agent capture.
  /healthz        plaintext producer state: frames seen, age of latest, size

Only gst-launch-1.0 is required; no ffmpeg/mjpg-streamer/opencv.
"""
import fcntl
import http.server
import os
import socketserver
import subprocess
import sys
import threading
import time
import urllib.parse

DEVICE = os.environ.get("FPGA_VIDEO_DEV", "/dev/video0")
# Seconds without a frame before the producer is considered wedged and is
# killed so run_forever() can respawn it.  This exists because a v4l2 stall
# does NOT end the gst process: gst-launch stays alive and simply stops
# emitting, so the blocking read below never returns and run_forever never
# gets to restart anything.  Observed 2026-08-19: the capture wedged and the
# server served the SAME cached JPEG for 12.3 hours with producer_restarts=0
# and last_error empty -- /snapshot.jpg looked healthy (it returns the cached
# frame) while /stream.mjpg rendered as a freeze.  Age, not liveness of the
# child process, is the only reliable signal.
STALL_TIMEOUT_S = float(os.environ.get("FPGA_VIDEO_STALL_S", "15"))
WIDTH = os.environ.get("FPGA_VIDEO_WIDTH", "1280")
HEIGHT = os.environ.get("FPGA_VIDEO_HEIGHT", "720")
FPS = os.environ.get("FPGA_VIDEO_FPS", "60")
# FPGA_VIDEO_CROP -- crop the captured frame down to the active picture.
#
# The FPGA scales the Mac framebuffer by the largest INTEGER factor that fits the
# output raster (place_plan.v), so at 720p a 640x480 Mac screen renders x1 and sits
# in the MIDDLE of a 1280x720 frame with a black border. Capturing that verbatim
# wastes most of the image. This crops the border away.
#
#   FPGA_VIDEO_CROP=auto        centre-crop to FPGA_VIDEO_SRC_W x FPGA_VIDEO_SRC_H
#                               (default 640x480) inside WIDTH x HEIGHT
#   FPGA_VIDEO_CROP=L,R,T,B     explicit pixel counts to remove from each edge
#   unset                       no crop (and the JPEG passthrough is preserved)
#
# NOTE: cropping forces a decode -> crop -> re-encode, because a passthrough JPEG
# stream is never decoded and cannot be cropped. That costs CPU and re-compresses,
# so it is OFF by default.
CROP = os.environ.get("FPGA_VIDEO_CROP", "").strip()
SRC_W = int(os.environ.get("FPGA_VIDEO_SRC_W", "640"))
SRC_H = int(os.environ.get("FPGA_VIDEO_SRC_H", "480"))
CROP_FALLBACK_QUALITY = int(os.environ.get("FPGA_VIDEO_CROP_QUALITY", "85"))


def crop_elems():
    """videocrop element for CROP, or [] when cropping is off/invalid."""
    if not CROP:
        return []
    if CROP.lower() == "auto":
        w, h = int(WIDTH), int(HEIGHT)
        left = max(0, (w - SRC_W) // 2)
        top = max(0, (h - SRC_H) // 2)
        right, bottom = max(0, w - SRC_W - left), max(0, h - SRC_H - top)
    else:
        try:
            left, right, top, bottom = (int(x) for x in CROP.split(","))
        except ValueError:
            print(f"FPGA_VIDEO_CROP={CROP!r} is not 'auto' or 'L,R,T,B' -- ignoring",
                  file=sys.stderr)
            return []
    if min(left, right, top, bottom) < 0:
        print(f"FPGA_VIDEO_CROP={CROP!r} has a negative edge -- ignoring", file=sys.stderr)
        return []
    if left + right >= int(WIDTH) or top + bottom >= int(HEIGHT):
        print(f"FPGA_VIDEO_CROP={CROP!r} would crop away the whole frame -- ignoring",
              file=sys.stderr)
        return []
    if not any((left, right, top, bottom)):
        return []
    return ["!", "videocrop", f"left={left}", f"right={right}",
            f"top={top}", f"bottom={bottom}"]
# The capture remains at full rate so /snapshot.jpg is always current, while
# the HTTP stream can deliberately skip frames to save WAN/mobile bandwidth.
try:
    STREAM_FPS = float(os.environ.get("FPGA_MJPEG_STREAM_FPS", FPS))
except ValueError as exc:
    raise SystemExit("FPGA_MJPEG_STREAM_FPS must be a positive number") from exc
if STREAM_FPS <= 0:
    raise SystemExit("FPGA_MJPEG_STREAM_FPS must be a positive number")
# The capture card already emits JPEG, so leaving this unset is a zero-copy
# pass-through.  On bandwidth-constrained links, setting it to 0..100 decodes
# and re-encodes each frame at the requested JPEG quality while preserving the
# same MJPEG-over-HTTP endpoint and browser compatibility.
_JPEG_QUALITY_ENV = os.environ.get("FPGA_MJPEG_QUALITY", "").strip()
try:
    JPEG_QUALITY = int(_JPEG_QUALITY_ENV) if _JPEG_QUALITY_ENV else None
except ValueError as exc:
    raise SystemExit("FPGA_MJPEG_QUALITY must be an integer from 0 to 100") from exc
if JPEG_QUALITY is not None and not 0 <= JPEG_QUALITY <= 100:
    raise SystemExit("FPGA_MJPEG_QUALITY must be an integer from 0 to 100")
HLS_ENABLE = os.environ.get("FPGA_HLS_ENABLE", "0").lower() in (
    "1", "true", "yes", "on"
)
try:
    HLS_BITRATE_KBIT = int(os.environ.get("FPGA_HLS_BITRATE_KBIT", "2500"))
except ValueError as exc:
    raise SystemExit("FPGA_HLS_BITRATE_KBIT must be a positive integer") from exc
if HLS_BITRATE_KBIT <= 0:
    raise SystemExit("FPGA_HLS_BITRATE_KBIT must be a positive integer")
HLS_DIR = os.environ.get("FPGA_HLS_DIR", "/dev/shm/fpga_hls")
try:
    HLS_MAX_FILES = int(os.environ.get("FPGA_HLS_MAX_FILES", "60"))
except ValueError as exc:
    raise SystemExit("FPGA_HLS_MAX_FILES must be at least 5") from exc
if HLS_MAX_FILES < 5:
    raise SystemExit("FPGA_HLS_MAX_FILES must be at least 5")
BIND = os.environ.get("FPGA_MJPEG_BIND", "10.200.0.12")
PORT = int(os.environ.get("FPGA_MJPEG_PORT", "8080"))
BOUNDARY = "frame"

SOI = b"\xff\xd8\xff"   # JPEG start-of-image
EOI = b"\xff\xd9"       # JPEG end-of-image

# ── ADB input injection, forwarded to the JTAG REPL ──────────────────────
# The REPL reads commands one-per-line from this FIFO.  We are one writer
# among several (a human at this page, an agent at a shell), so every send
# is serialised under a lock and the fd is opened/closed per command --
# holding it open would keep the FIFO's writer count non-zero and mask a
# dead REPL from everyone else.
#
# O_NONBLOCK ON THE OPEN IS LOAD-BEARING, not a micro-optimisation.  A
# plain blocking open() of a FIFO for writing BLOCKS UNTIL A READER EXISTS
# -- so if the REPL is not running, the HTTP worker thread wedges forever
# and the page just hangs with no error.  With O_NONBLOCK the open fails
# immediately with ENXIO and we can say "REPL not listening" instead.
JTAG_FIFO = os.environ.get("FPGA_JTAG_FIFO", "/tmp/jtag_in")
JTAG_LOCK = threading.Lock()

# ── Cross-process JTAG arbitration ───────────────────────────────────────
# The FIFO has SEVERAL independent writers: this page, an agent at a shell,
# a human running jt.sh.  A command injected into the middle of someone
# else's long operation does not fail loudly -- it is simply consumed by the
# REPL as the next line, corrupting whatever was in flight.  That is not
# hypothetical: a single button press during a 2-minute bulk SD write
# aborted it mid-transfer with a CMD18 verify timeout.
#
# So: an flock(2) advisory lock on a shared file, honoured by every writer.
# We take it NON-BLOCKING and refuse rather than queue -- a click that waits
# 90 s and then fires into a since-changed machine state is worse than one
# that says "busy, try again".  Long operations hold the lock for their
# whole execution (not just the FIFO write), because the REPL keeps running
# the command long after the line has been consumed.
#
# The holder writes "<pid> <tag>" into the file so a refusal can say WHO has
# it. Advisory only: a writer that ignores the lock still gets through, which
# is why the log below exists as the backstop.
JTAG_LOCKFILE = os.environ.get("FPGA_JTAG_LOCK", "/tmp/jtag_in.lock")
JTAG_LOGFILE = os.environ.get("FPGA_JTAG_LOG", "/tmp/jtag_cmd.log")

# The REPL's stdout, for reading back what it said about our command.  Only
# the deliberately-dangerous verbs wait for this (see jtag_send's `reply`
# argument); the ADB verbs stay fire-and-forget so the arrow keys keep their
# current latency.
JTAG_OUTFILE = os.environ.get("FPGA_JTAG_OUT", "/tmp/jtag_out")


def _jtag_log(origin, text, note=""):
    """Append to the shared JTAG audit log.  Best-effort; never raises."""
    try:
        line = (f"{time.strftime('%Y-%m-%dT%H:%M:%S')} pid={os.getpid()} "
                f"origin={origin} {note}{text}\n")
        with open(JTAG_LOGFILE, "a") as fh:
            fh.write(line)
    except OSError:
        pass


def _lock_holder():
    """Best-effort "<pid> <tag>" of whoever holds the JTAG lock.

    Neither writer (this server, nor macqd700-soc tools/jt.sh) CLEARS the
    file on release -- both only overwrite it on acquire -- so the name in it
    is the last ACQUIRER, not necessarily the current holder.  A refusal
    naming the wrong process is exactly the sort of confidently-wrong tool
    output this project keeps getting burned by, so check the pid is still
    alive and label it when it is not.  We cannot repair the protocol from
    this side without changing jt.sh too, but we can stop asserting a name
    we have no evidence for.
    """
    try:
        with open(JTAG_LOCKFILE) as fh:
            who = fh.read().strip()
    except OSError:
        return "unknown"
    if not who:
        return "unknown"
    pid = who.split(" ", 1)[0]
    if pid.isdigit():
        try:
            os.kill(int(pid), 0)
        except ProcessLookupError:
            return f"{who} (that pid is GONE — name is stale, real holder unknown)"
        except OSError:
            pass                              # EPERM: alive, just not ours
    return who


def _vivado_hint():
    """Extra colour for a failed FIFO open.  DIAGNOSTIC ONLY.

    Deliberately NOT the liveness test.  Two reasons it cannot be:

      * `pgrep -x vivado` also matches a `vivado -mode batch` synth/impl run,
        which does not read the FIFO at all -- so a live batch build would
        have this report a healthy REPL when there is none.
      * `pgrep -f vivado` is worse still: -f matches against full command
        lines and has previously matched (and killed) the caller's own.

    The authoritative liveness check is the O_NONBLOCK open of the FIFO
    itself: ENXIO means the kernel sees literally no reader, which is exactly
    the question we are asking.  This only turns "not listening" into a
    sentence that says which of the two situations you are in.
    """
    try:
        rc = subprocess.run(["pgrep", "-x", "vivado"], timeout=3,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        return ""
    n = len(rc.stdout.split())
    if not n:
        return " No vivado process is running at all — start the JTAG REPL."
    return (f" {n} vivado process(es) are running but none is reading the FIFO"
            f" — that is likely a batch synth/impl run, not the REPL.")


def _read_reply(offset, timeout):
    """Return whatever the REPL appended to its stdout past `offset`.

    Bounded wait: stops as soon as the REPL prints its `READY` prompt, else
    gives up at `timeout` and returns what it has.  A missing/unreadable
    output file is not an error -- the REPL may simply not be redirected
    here -- it just means no reply text.
    """
    deadline = time.time() + timeout
    text = ""
    while True:
        try:
            with open(JTAG_OUTFILE, "rb") as fh:
                fh.seek(offset)
                text = fh.read().decode("utf-8", "replace")
        except OSError:
            return ""
        if "READY" in text or time.time() >= deadline:
            break
        time.sleep(0.05)
    lines = [l.strip() for l in text.splitlines()
             if l.strip() and l.strip() != "> READY"]
    return " | ".join(lines)

# Mac virtual keycodes (ADB).  Same table the m68k-adb-input-control skill
# documents; `adb-key` takes these directly.
KEYCODES = {
    'a': 0x00, 's': 0x01, 'd': 0x02, 'f': 0x03, 'h': 0x04, 'g': 0x05,
    'z': 0x06, 'x': 0x07, 'c': 0x08, 'v': 0x09, 'b': 0x0B, 'q': 0x0C,
    'w': 0x0D, 'e': 0x0E, 'r': 0x0F, 'y': 0x10, 't': 0x11, '1': 0x12,
    '2': 0x13, '3': 0x14, '4': 0x15, '6': 0x16, '5': 0x17, '=': 0x18,
    '9': 0x19, '7': 0x1A, '-': 0x1B, '8': 0x1C, '0': 0x1D, ']': 0x1E,
    'o': 0x1F, 'u': 0x20, '[': 0x21, 'i': 0x22, 'p': 0x23, 'l': 0x25,
    'j': 0x26, "'": 0x27, 'k': 0x28, ';': 0x29, '\\': 0x2A, ',': 0x2B,
    '/': 0x2C, 'n': 0x2D, 'm': 0x2E, '.': 0x2F, '`': 0x32,
    '\n': 0x24, '\t': 0x30, ' ': 0x31,
}
# Characters reachable only with Shift held.
SHIFTED = {
    '!': '1', '@': '2', '#': '3', '$': '4', '%': '5', '^': '6', '&': '7',
    '*': '8', '(': '9', ')': '0', '_': '-', '+': '=', '{': '[', '}': ']',
    '|': '\\', ':': ';', '"': "'", '<': ',', '>': '.', '?': '/', '~': '`',
}
KEY_SHIFT = 0x38
KEY_CMD = 0x37
KEY_RETURN = 0x24
KEY_DELETE = 0x33

# DAFB monitor-sense codes, from MAME dafb.cpp:202-216.  Extended codes are
# ext(bc,ac,ab) = 0x40 | bc<<4 | ac<<2 | ab.
#
# WHITELIST, not a free-form field.  `mon-sense` takes any 7-bit value and an
# unlisted one leaves the machine claiming a display nobody has -- recoverable
# only by another mon-sense + reset, which is exactly the situation you cannot
# drive from a page you can no longer see.
# RAM window sizes, as lg2(bytes) for the OFF_RAM_WINDOW_LG2 debug CSR
# (rtl/soc/fpga_top_debug_ctrl.vh, offset 0x058).  The CSR
# clamps to [22..30] = [4 MiB..1 GiB]; the value survives a CPU reset but NOT
# an FPGA reprogram, which restores 26 (64 MiB).
#
# Deliberately stopping at 256 MiB rather than the CSR's 1 GiB ceiling: the
# board has 2 GiB of DDR with only ~1012 MiB free above RAM/ROM/framebuffer,
# so a 1 GiB window would hand Mac OS an address range that is not actually
# backed.  A window larger than the backing store is exactly the sort of thing
# that looks like a CPU bug for a day.  Use the REPL's `ram-window` directly if
# you really want it.
#
# NOTE the >4 MiB windows make early boot visibly slower -- the ROM clears all
# of RAM, so 64 MiB is ~16x the clear of 4 MiB.  Do not mistake that for a hang.
RAM_WINDOWS = {
    22: "4 MiB",
    23: "8 MiB",
    24: "16 MiB",
    25: "32 MiB",
    26: "64 MiB (default)",
    27: "128 MiB",
    28: "256 MiB",
}

# ── SD provisioning (macqd700-soc tools/sd_{os_swap,write_rom}.sh) ───────
# Swapping the OS on the SD card used to mean writing a full 500 MB image at
# ~264-301 KiB/s, i.e. ~33 minutes, which is long enough that nobody does it
# casually.  sd_os_swap.sh writes only the HFS USED extent plus the alternate
# MDB, so 7.0.1 is ~25 s and 7.5.3 ~2.5 min -- short enough to be a button.
#
# It is genuinely destructive: it overwrites the boot volume, so anything the
# running system has not flushed is gone.  Hence confirm=yes, same as NMI.
#
# The swap loads the PROVISIONING bitstream (sd-write-fast exists only there),
# writes, then reloads the main one -- so the Mac is down for the duration and
# the display will go away and come back.  That is the tool working.
SOC_DIR = os.environ.get("FPGA_SOC_DIR", "/home/qwertyoruiop/macqd700-soc")
PROV_BIT = os.environ.get(
    "FPGA_PROV_BIT", os.path.join(SOC_DIR, "build/sd_provision/sd_provision_top.bit"))
MAIN_BIT = os.environ.get(
    "FPGA_MAIN_BIT", os.path.join(SOC_DIR, "build/vivado/fpga_top.bit"))
MAIN_LTX = os.environ.get(
    "FPGA_MAIN_LTX", os.path.join(SOC_DIR, "build/vivado/fpga_top.ltx"))

OS_IMAGES = {
    "701": "System 7.0.1  (~25 s)",
    "753": "System 7.5.3  (~2.5 min)",
}

# Upper bound on a swap.  7.5.3 measures ~2.5 min of writing plus ~40 s of
# load-bit either side; 25 min is far past any healthy run.  The cap exists
# so a wedged Vivado cannot leave the JTAG lock held forever, which would
# silently disable every other button on this page.
OS_SWAP_TIMEOUT = 25 * 60

OS_SWAP = {"running": False, "tag": "", "started": 0.0, "rc": None,
           "lines": [], "note": "", "kind": "os"}
OS_SWAP_STATE_LOCK = threading.Lock()


def _provision_label(kind, tag):
    if kind == "rom":
        return "Quadra 700 ROM"
    return OS_IMAGES.get(tag, tag)


def _provision_worker(kind, tag, lock_fd):
    """Run one provisioning script while holding the JTAG lock throughout.

    The lock is acquired by the caller (so a click can be refused
    synchronously) and released here, because the REPL keeps executing long
    after the command line has been consumed -- releasing at write time would
    let a stray button press land in the middle of a multi-minute SD write,
    which has aborted a transfer before.

    We deliberately do NOT set JTAG_LEASE_HELD: the script then takes
    macqd700-soc's own jtag_lease.sh as well.  The two arbitration schemes
    cover different writers (this page vs. agents at a shell) and respecting
    both is the point.
    """
    if kind == "rom":
        script_name = "sd_write_rom.sh"
        argv = []
    else:
        script_name = "sd_os_swap.sh"
        argv = [tag]
    script = os.path.join(SOC_DIR, "tools", script_name)
    rc, lines = 1, []
    try:
        if not os.access(script, os.X_OK):
            lines.append(f"FATAL: {script} is missing or not executable")
        else:
            child_env = os.environ.copy()
            child_env.update(PROV=PROV_BIT, MAIN=MAIN_BIT, LTX=MAIN_LTX)
            proc = subprocess.Popen(
                [script, *argv], cwd=SOC_DIR, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True,
                env=child_env)
            try:
                for line in proc.stdout:
                    line = line.rstrip()
                    if not line:
                        continue
                    with OS_SWAP_STATE_LOCK:
                        OS_SWAP["lines"].append(line)
                        # Keep the tail only; a verify failure prints a lot.
                        del OS_SWAP["lines"][:-40]
                    lines.append(line)
                rc = proc.wait(timeout=OS_SWAP_TIMEOUT)
            except subprocess.TimeoutExpired:
                proc.kill()
                rc = 124
                lines.append(f"FATAL: timed out after {OS_SWAP_TIMEOUT}s — killed")
    except OSError as exc:
        lines.append(f"FATAL: cannot run {script}: {exc.strerror}")
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            os.close(lock_fd)
        except OSError:
            pass
        with OS_SWAP_STATE_LOCK:
            OS_SWAP["running"] = False
            OS_SWAP["rc"] = rc
        command = " ".join([script_name, *argv])
        _jtag_log("web", command, note=f"DONE(rc={rc}) ")


SENSE_CODES = {
    0x00: '21" Color 1152x870',
    0x01: 'Portrait B&W 15" 640x870',
    0x02: 'RGB 12" 512x384',
    0x03: 'Two-Page B&W 21" 1152x870',
    0x06: 'Hi-Res 12-14" 640x480',
    0x6D: '16" RGB 832x624',
    0x57: 'VGA 640x480',
}


def jtag_send(lines, reply=0.0):
    """Send one or more REPL command lines.  Returns (ok, detail).

    Refuses (does not queue) if another writer holds the cross-process JTAG
    lock -- see the JTAG_LOCKFILE comment above for why refusing beats
    waiting.

    `reply` > 0 waits up to that many seconds for the REPL to answer and
    folds its output into `detail`, so the caller learns the command was
    actually EXECUTED and not merely written into a pipe.  It is opt-in
    because the wait happens WITH THE LOCK STILL HELD (deliberately -- it
    stops a second command landing on top of a dangerous one in flight),
    which is the right trade only for the rare, deliberate verbs.
    """
    payload = "".join(l.rstrip("\n") + "\n" for l in lines).encode()
    pretty = payload.decode().replace("\n", " ; ").strip(" ;")

    with JTAG_LOCK:                      # serialise this process's own threads
        try:
            lock_fd = os.open(JTAG_LOCKFILE, os.O_RDWR | os.O_CREAT, 0o666)
        except OSError as exc:
            return False, f"cannot open JTAG lock {JTAG_LOCKFILE}: {exc.strerror}"
        try:
            try:
                fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                holder = _lock_holder()
                _jtag_log("web", pretty, note="REFUSED(busy) ")
                return False, (f"JTAG busy — held by {holder}. "
                               f"A long operation is in flight; try again shortly.")
            try:
                os.ftruncate(lock_fd, 0)
                os.write(lock_fd, f"{os.getpid()} mjpeg-web".encode())
                try:
                    fifo_fd = os.open(JTAG_FIFO, os.O_WRONLY | os.O_NONBLOCK)
                except OSError as exc:
                    # ENXIO == FIFO exists but nobody reads it: REPL is down.
                    _jtag_log("web", pretty, note="REFUSED(no-repl) ")
                    return False, (f"JTAG REPL not listening on {JTAG_FIFO} "
                                   f"({exc.strerror}).{_vivado_hint()}")
                # Where the REPL's output ends right now, so the reply read
                # below cannot pick up somebody else's older text.
                try:
                    out_at = os.stat(JTAG_OUTFILE).st_size if reply else 0
                except OSError:
                    out_at = 0
                try:
                    os.write(fifo_fd, payload)
                except OSError as exc:
                    _jtag_log("web", pretty, note=f"WRITE-FAIL({exc.strerror}) ")
                    return False, f"write failed: {exc.strerror}"
                finally:
                    os.close(fifo_fd)
                # Give the REPL a moment to consume the line before dropping
                # the lock, so a rapid second click cannot land inside it.
                time.sleep(0.15)
                _jtag_log("web", pretty)
                if reply:
                    said = _read_reply(out_at, reply)
                    pretty = f"{pretty} → {said}" if said else \
                             (f"{pretty} → (no reply within {reply:g}s — the "
                              f"REPL took the line but has not answered; it "
                              f"may be busy)")
            finally:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)
    return True, pretty


def text_to_key_cmds(text, limit=200):
    """Expand a string into adb-key command lines."""
    cmds = []
    for ch in text[:limit]:
        shift = False
        if ch.isupper():
            ch, shift = ch.lower(), True
        elif ch in SHIFTED:
            ch, shift = SHIFTED[ch], True
        code = KEYCODES.get(ch)
        if code is None:
            continue                       # silently skip unmappable chars
        if shift:
            cmds.append(f"adb-key 0x{KEY_SHIFT:02X} down")
            cmds.append(f"adb-key 0x{code:02X} press")
            cmds.append(f"adb-key 0x{KEY_SHIFT:02X} up")
        else:
            cmds.append(f"adb-key 0x{code:02X} press")
    return cmds


def gst_pipeline():
    # Raw concatenated JPEGs on stdout.  multipartmux is deliberately NOT
    # used: we re-frame ourselves so each consumer can be handed whole
    # JPEGs independently, rather than a byte stream only one reader can
    # follow.
    pipeline = [
        "gst-launch-1.0", "-q",
        "v4l2src", f"device={DEVICE}",
        "!", f"image/jpeg,width={WIDTH},height={HEIGHT},framerate={FPS}/1",
    ]
    if HLS_ENABLE and JPEG_QUALITY is not None:
        pipeline += [
            "!", "jpegdec",
            *crop_elems(),
            "!", "videoconvert",
            "!", "tee", "name=raw",
            "raw.", "!", "queue",
            "!", "jpegenc", f"quality={JPEG_QUALITY}",
            "!", "fdsink", "fd=1",
            "raw.", "!", "queue",
        ]
    elif HLS_ENABLE:
        pipeline += [
            "!", "tee", "name=jpeg",
            "jpeg.", "!", "queue",
            "!", "fdsink", "fd=1",
            "jpeg.", "!", "queue",
            "!", "jpegdec",
            *crop_elems(),
            "!", "videoconvert",
        ]
    elif JPEG_QUALITY is not None:
        pipeline += [
            "!", "jpegdec",
            *crop_elems(),
            "!", "videoconvert",
            "!", "jpegenc", f"quality={JPEG_QUALITY}",
        ]
    elif crop_elems():
        # Cropping with no explicit quality: the passthrough path never decodes, so
        # it cannot crop. Force decode -> crop -> re-encode at a default quality.
        pipeline += [
            "!", "jpegdec",
            *crop_elems(),
            "!", "videoconvert",
            "!", "jpegenc", f"quality={CROP_FALLBACK_QUALITY}",
        ]

    if HLS_ENABLE:
        pipeline += [
            "!", "x264enc", "tune=zerolatency", "speed-preset=veryfast",
            f"bitrate={HLS_BITRATE_KBIT}", f"key-int-max={FPS}",
            "!", "h264parse", "config-interval=-1",
            "!", "hlssink2", f"max-files={HLS_MAX_FILES}",
            "playlist-length=3",
            "target-duration=1",
            f"location={os.path.join(HLS_DIR, 'segment%05d.ts')}",
            f"playlist-location={os.path.join(HLS_DIR, 'stream.m3u8')}",
        ]
    else:
        pipeline += ["!", "fdsink", "fd=1"]
    return pipeline


class FrameSource:
    """Owns the capture device.  Publishes the latest whole JPEG."""

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._stamp = 0.0
        self._frames_total = 0
        self._restarts = 0
        self._last_error = ""
        # Monotonic stamp of the last published frame, for the stall
        # watchdog.  Separate from _stamp (wall clock, used by health) so a
        # system clock step cannot make a live producer look wedged.
        self._mono = time.monotonic()

    # -- producer ---------------------------------------------------
    def run_forever(self):
        if HLS_ENABLE:
            os.makedirs(HLS_DIR, exist_ok=True)
        while True:
            try:
                self._pump_once()
            except Exception as exc:                      # noqa: BLE001
                self._last_error = f"{type(exc).__name__}: {exc}"
            # gst died (device unplugged, another process grabbed it, …).
            # Back off briefly and retry rather than wedging the server.
            self._restarts += 1
            time.sleep(1.0)

    def _pump_once(self):
        proc = subprocess.Popen(gst_pipeline(), stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL)
        # Reset the stall clock: a fresh producer gets a full timeout to
        # deliver its first frame rather than inheriting the previous
        # producer's age and being killed immediately.
        with self._cond:
            self._mono = time.monotonic()
        stop = threading.Event()

        def _watchdog():
            while not stop.wait(1.0):
                with self._cond:
                    idle = time.monotonic() - self._mono
                if idle > STALL_TIMEOUT_S:
                    self._last_error = (
                        f"producer stalled {idle:.0f}s with no frame "
                        f"(>{STALL_TIMEOUT_S:.0f}s); killing to respawn")
                    # kill(), not terminate(): a wedged v4l2 read can ignore
                    # SIGTERM, and the point is to unblock our own read().
                    try:
                        proc.kill()
                    except Exception:                     # noqa: BLE001
                        pass
                    return

        wd = threading.Thread(target=_watchdog, name="gst-stall-watchdog",
                              daemon=True)
        wd.start()
        buf = b""
        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    return
                buf += chunk
                # Extract every complete JPEG currently in the buffer.
                while True:
                    start = buf.find(SOI)
                    if start < 0:
                        # Keep a couple of bytes; a marker may straddle reads.
                        buf = buf[-2:]
                        break
                    end = buf.find(EOI, start + len(SOI))
                    if end < 0:
                        buf = buf[start:]
                        break
                    self._publish(buf[start:end + len(EOI)])
                    buf = buf[end + len(EOI):]
        finally:
            stop.set()
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()

    def _publish(self, frame):
        with self._cond:
            self._frame = frame
            self._seq += 1
            self._frames_total += 1
            self._stamp = time.time()
            self._mono = time.monotonic()
            self._cond.notify_all()

    # -- consumers --------------------------------------------------
    def latest(self, timeout=5.0):
        """Newest frame, waiting briefly if none has arrived yet."""
        with self._cond:
            if self._frame is None:
                self._cond.wait(timeout)
            return self._frame, self._seq

    def next_after(self, seq, timeout=5.0):
        """Block for a frame newer than `seq`.  (None, seq) on timeout.

        Slow clients simply skip ahead to the newest frame — they can
        never back-pressure the capture or stall another viewer.
        """
        with self._cond:
            if self._seq <= seq:
                self._cond.wait(timeout)
            if self._seq <= seq:
                return None, seq
            return self._frame, self._seq

    def health(self):
        with self._cond:
            age = (time.time() - self._stamp) if self._stamp else -1.0
            return {
                "frames_total": self._frames_total,
                "seq": self._seq,
                "latest_bytes": len(self._frame) if self._frame else 0,
                "age_s": round(age, 3),
                "producer_restarts": self._restarts,
                "last_error": self._last_error,
                "stall_timeout_s": STALL_TIMEOUT_S,
                "device": DEVICE,
                "stream_fps": STREAM_FPS,
                "jpeg_quality": (JPEG_QUALITY if JPEG_QUALITY is not None
                                 else "capture-passthrough"),
                "hls": HLS_ENABLE,
                "hls_bitrate_kbit": HLS_BITRATE_KBIT if HLS_ENABLE else 0,
                "hls_retention_s": HLS_MAX_FILES if HLS_ENABLE else 0,
            }


SOURCE = FrameSource()

INDEX_HTML = b"""<!doctype html><meta charset=utf-8>
<title>FPGA HDMI</title>
<style>
 html,body{margin:0;min-height:100%;background:#111;color:#ccc;
           font:13px/1.5 ui-monospace,monospace}
 .wrap{min-height:100%;display:flex;flex-direction:column;align-items:center;
       justify-content:center;gap:10px;padding:8px;box-sizing:border-box}
 img,video{max-width:100%;max-height:70vh;image-rendering:pixelated;background:#000}
 .ctl{display:flex;gap:18px;align-items:center;flex-wrap:wrap;
      justify-content:center}
 .pad{display:grid;grid-template-columns:repeat(3,40px);
      grid-template-rows:repeat(3,32px);gap:4px}
 button{background:#222;color:#ddd;border:1px solid #444;border-radius:4px;
        font:inherit;cursor:pointer;padding:4px 8px}
 button:hover{background:#333}
 button:active{background:#4a4a4a}
 /* The NMI is the one control here that is disruptive on a HEALTHY machine,
    so it does not look like its neighbours: its own row, its own colour, and
    a confirm() in front of it. */
 button.danger{background:#3a1414;color:#f0b4b4;border-color:#7a2a2a}
 button.danger:hover{background:#521a1a}
 .pad button{padding:0}
 .col{display:flex;flex-direction:column;gap:6px}
 .row{display:flex;gap:6px;align-items:center}
 input[type=text]{background:#1a1a1a;color:#ddd;border:1px solid #444;
                  border-radius:4px;font:inherit;padding:4px 6px;width:230px}
 select{background:#1a1a1a;color:#ddd;border:1px solid #444;
         border-radius:4px;font:inherit;padding:4px}
 input[type=number]{background:#1a1a1a;color:#ddd;border:1px solid #444;
                    border-radius:4px;font:inherit;padding:4px;width:52px}
 #st{min-height:1.4em;color:#8a8;font-size:12px}
 #st.err{color:#d77}
 a{color:#7ab}
</style>
<div class=wrap>
 <video id=h264 autoplay muted playsinline hidden></video>
 <img id=mjpeg alt="FPGA HDMI capture">

 <div class=ctl>
  <div class=pad>
   <span></span><button id=u>&uarr;</button><span></span>
   <button id=l>&larr;</button><button id=c title="click">&#9679;</button><button id=r>&rarr;</button>
   <span></span><button id=d>&darr;</button><span></span>
  </div>

  <div class=col>
   <div class=row>
    <label>step <input type=number id=step value=8 min=1 max=63></label>
    <button id=press>hold</button><button id=rel>release</button>
    <button id=cmdo title="Command-O (open)">&#8984;O</button>
    <button id=cmdw title="Command-W (close window)">&#8984;W</button>
    <button id=cmdq title="Command-Q (quit application)">&#8984;Q</button>
   </div>
   <div class=row>
    <input type=text id=txt placeholder="type into the Mac, then Enter">
    <button id=send>send</button>
    <button id=ret>&crarr;</button>
    <button id=del title="Delete / backspace (ADB 0x33) - the Mac's Delete key deletes BACKWARDS; forward-delete is a different key and is not wired up">&#9003;</button>
   </div>
   <div class=row>
    <select id=sense>
     <option value="0x06">Hi-Res 12-14" 640x480</option>
     <option value="0x6D">16" RGB 832x624</option>
     <option value="0x00">21" Color 1152x870</option>
     <option value="0x03">Two-Page B&amp;W 21" 1152x870</option>
     <option value="0x02">RGB 12" 512x384</option>
     <option value="0x01">Portrait B&amp;W 15" 640x870</option>
     <option value="0x57">VGA 640x480</option>
    </select>
    <button id=applysense title="set sense code, then reset (Mac OS reads sense only at DAFB init)">apply + reset</button>
    <button id=reset title="WARM reset: reboots the Mac but does NOT re-zero the 256 MB RAM window, so it comes back in ~0.2 s instead of ~4 s. The previous session's RAM survives, which is exactly why this is not a valid way to reproduce a cold-boot bug.">warm reset</button>
    <button id=coldreset title="COLD reset: full platform reset. Re-zeroes the whole 256 MB RAM window before booting, so the machine comes up on genuinely blank memory like a power cycle. Takes ~4 s longer. Use this when RAM contents could matter.">cold reset</button>
   </div>
   <div class=row>
    <span style="font-size:12px;color:#888">PRAM</span>
    <button id=pramsave title="save the live PRAM to the reserved SD sector (LBA 8191) - MANUAL, nothing does this automatically">save</button>
    <button id=pramload title="restore PRAM from the SD sector; a missing or corrupt image falls back to the post-reset defaults and says so">load</button>
    <button id=pramdump title="hex dump the live 256 PRAM bytes to the JTAG log - does not touch the card">dump</button>
    <button id=pramclear title="zap PRAM to its post-reset defaults (Cmd-Opt-P-R)">clear</button>
    <a href="/jtaglog" target="_blank" style="font-size:12px">log</a>
   </div>
   <div class=row>
    <span style="font-size:12px;color:#888">Disk</span>
    <button id=wproton title="write-protect the SD volume: the SCSI target answers WRITE(6)/WRITE(10) with CHECK CONDITION / DATA PROTECT and the card is never written. Mac OS mounts the volume read-only.">lock</button>
    <button id=wprotoff title="allow writes to the SD volume again">unlock</button>
    <button id=wprotstate title="read the current write-protect state (CTRL[2]) into the JTAG log">state</button>
    <span style="font-size:11px;color:#666">CTRL[2] - cleared by any load-bit</span>
   </div>
   <div class=row>
    <span style="font-size:12px;color:#888">RAM</span>
    <select id=ramsize>
     <option value="22">4 MiB</option>
     <option value="23">8 MiB</option>
     <option value="24">16 MiB</option>
     <option value="25">32 MiB</option>
     <option value="26" selected>64 MiB (default)</option>
     <option value="27">128 MiB</option>
     <option value="28">256 MiB</option>
    </select>
    <button id=applyram title="set the RAM window, then reset (Mac OS sizes RAM only at early boot). Larger windows make the ROM's RAM-clear visibly slower - that is not a hang.">apply + reset</button>
   </div>
   <div class=row>
    <span style="font-size:12px;color:#888">SD Flash</span>
    <select id=osimg>
     <option value="701">System 7.0.1  (~25 s)</option>
     <option value="753">System 7.5.3  (~2.5 min)</option>
     <option value="rom">Quadra 700 ROM</option>
    </select>
    <button id=applyos class=danger title="OVERWRITES and verifies the selected SD-card region, then reloads the main bitstream. The screen goes away while the provisioning bitstream is loaded - that is expected.">write + boot</button>
    <span id=osst style="font-size:12px;color:#888"></span>
   </div>
   <div class=row>
    <button id=nmi class=danger title="Breaks into a spinning machine WITHOUT losing its state, unlike reset.">NMI (level 7)</button>
   </div>
  </div>
 </div>

 <div id=st></div>
 <div style="font-size:12px;color:#777">
  /hls/stream.m3u8 &middot; /stream.mjpg &middot; /snapshot.jpg &middot; /healthz &middot; /input
  &mdash; arrow keys also work
 </div>
</div>
<script>
const st = document.getElementById('st');
// Safari (including iPhone/iPad) plays HLS natively.  Prefer the much smaller
// H.264 stream when both the browser and server support it; every other
// browser keeps the existing MJPEG image with no dependency on external JS.
const h264 = document.getElementById('h264');
const mjpeg = document.getElementById('mjpeg');
const useMjpeg = () => {
  h264.hidden = true;
  h264.removeAttribute('src');
  mjpeg.src = '/stream.mjpg';
  mjpeg.hidden = false;
};
if(h264.canPlayType('application/vnd.apple.mpegurl')){
  const tryHls = n => fetch('/hls/stream.m3u8', {cache:'no-store'})
    .then(r => {
      if(!r.ok) throw new Error('HLS not ready');
      h264.src = '/hls/stream.m3u8';
      h264.hidden = false;
      mjpeg.hidden = true;
      h264.play().catch(() => {});
    })
    .catch(() => {
      if(n > 0) setTimeout(() => tryHls(n-1), 1000);
      else useMjpeg();
    });
  h264.addEventListener('error', useMjpeg, {once:true});
  tryHls(10);
} else useMjpeg();
const step = () => Math.max(1, Math.min(63, +document.getElementById('step').value || 8));
async function go(qs){
  try{
    const res = await fetch('/input?' + qs);
    const txt = (await res.text()).trim();
    st.textContent = txt;
    st.className = res.ok ? '' : 'err';
  }catch(e){ st.textContent = 'request failed: ' + e; st.className = 'err'; }
}
const mv = (dx,dy) => go(`a=move&dx=${dx}&dy=${dy}`);
document.getElementById('u').onclick = () => mv(0,-step());
document.getElementById('d').onclick = () => mv(0, step());
document.getElementById('l').onclick = () => mv(-step(),0);
document.getElementById('r').onclick = () => mv(step(),0);
document.getElementById('c').onclick = () => go('a=click');
document.getElementById('press').onclick = () => go('a=press');
document.getElementById('rel').onclick = () => go('a=release');
document.getElementById('ret').onclick = () => go('a=return');
document.getElementById('del').onclick = () => go('a=delete');
document.getElementById('cmdo').onclick = () => go('a=cmd&k=o');
document.getElementById('cmdw').onclick = () => go('a=cmd&k=w');
document.getElementById('cmdq').onclick = () => go('a=cmd&k=q');
document.getElementById('reset').onclick = () => go('a=reset');
document.getElementById('coldreset').onclick = () => {
  if(confirm('Cold reset: re-zeroes the whole 256 MB RAM window before booting (~4 s slower than a warm reset). Continue?'))
    go('a=coldreset');
};
// confirm() guards the accidental click;
// the server separately demands confirm=yes so an accidental REQUEST (a
// bookmark, a prefetch, a replayed URL) cannot fire one either.  The reply
// carried back in #st is the REPL's own output, so a dead REPL or a refused
// lock says so here rather than looking like a successful press.
document.getElementById('nmi').onclick = () => {
  if(confirm('Raise a level-7 NMI?\\n\\n' +
             'This interrupts the Mac wherever it is. On a healthy running ' +
             'system it is disruptive; on a hung one it is how you break in ' +
             'without losing state.'))
    go('a=nmi&confirm=yes');
};
// PRAM persistence.  save/load hit the SD card and can take a moment; the
// REPL's own reply (including its not-halted warning and any failure
// reason) is what lands in #st, so a failure is never silent here.
document.getElementById('wproton').onclick = () => go('a=wprot&on=1');
document.getElementById('wprotstate').onclick = () => go('a=wprot');
document.getElementById('wprotoff').onclick = () => {
  if(confirm('Allow writes to the SD volume again?\\n\\n' +
             'While locked the card cannot be modified by the Mac. Unlocking\\n' +
             'lets the OS write to the boot volume as normal.'))
    go('a=wprot&on=0');
};
document.getElementById('pramsave').onclick = () => go('a=pramsave');
document.getElementById('pramload').onclick = () => go('a=pramload');
document.getElementById('pramdump').onclick = () => go('a=pramdump');
document.getElementById('pramclear').onclick = () => {
  if(confirm('Zap PRAM to its post-reset defaults? Unsaved settings are lost.'))
    go('a=pramclear');
};
document.getElementById('applyram').onclick = () => {
  const v = document.getElementById('ramsize').value;
  const label = document.getElementById('ramsize').selectedOptions[0].text;
  if(confirm('Set the RAM window to ' + label + ' and reset?\\n\\n' +
             'This resets the Mac -- unsaved work is lost. Larger windows make\\n' +
             'the ROM RAM-clear take proportionally longer before video appears.'))
    go('a=ramwindow&lg2=' + encodeURIComponent(v));
};
document.getElementById('applyos').onclick = () => {
  const v = document.getElementById('osimg').value;
  const label = document.getElementById('osimg').selectedOptions[0].text;
  if(v === 'rom'){
    if(!confirm('Restore the Quadra 700 ROM on the SD card?\\n\\n' +
                'This OVERWRITES LBA 0..2047, verifies all 2048 sectors, then\\n' +
                'reloads the main bitstream. The HDD and PRAM regions are outside\\n' +
                'the permitted write range. The Mac goes down for the operation.')) return;
    go('a=romrestore&confirm=yes');
    pollOsSwap();
    return;
  }
  if(!confirm('Write ' + label + ' to the SD card and boot it?\\n\\n' +
              'This OVERWRITES the boot volume -- anything the running system\\n' +
              'has not saved is lost. The Mac goes down for the whole write and\\n' +
              'the video will disappear and come back. Every other control on\\n' +
              'this page is refused until it finishes.')) return;
  go('a=osswap&img=' + encodeURIComponent(v) + '&confirm=yes');
  pollOsSwap();
};
// Poll while a swap runs.  Without this the click looks like it did nothing
// for two and a half minutes, which is exactly when someone clicks it again.
let osTimer = null;
function pollOsSwap(){
  if(osTimer) clearInterval(osTimer);
  osTimer = setInterval(async () => {
    try{
      const r = await fetch('/input?a=osswapstatus');
      const t = (await r.text()).trim();
      document.getElementById('osst').textContent = t.split('\\n')[0];
      if(!t.startsWith('running')){ clearInterval(osTimer); osTimer = null; st.textContent = t; }
    }catch(e){ clearInterval(osTimer); osTimer = null; }
  }, 3000);
}
document.getElementById('applysense').onclick = () => {
  const v = document.getElementById('sense').value;
  go('a=sense&code=' + encodeURIComponent(v));
};
const txt = document.getElementById('txt');
function sendText(){
  if(!txt.value) return;
  go('a=type&s=' + encodeURIComponent(txt.value));
  txt.value = '';
}
document.getElementById('send').onclick = sendText;
txt.addEventListener('keydown', e => {
  if(e.key === 'Enter'){ e.preventDefault(); sendText(); }
  e.stopPropagation();          // don't let arrows in the box drive the pad
});
document.addEventListener('keydown', e => {
  if(e.key === 'Enter'){
    if(e.repeat) return;
    e.preventDefault();
    go('a=click');
    return;
  }
  const k = {ArrowUp:[0,-1],ArrowDown:[0,1],ArrowLeft:[-1,0],ArrowRight:[1,0]}[e.key];
  if(!k) return;
  e.preventDefault();
  mv(k[0]*step(), k[1]*step());
});
</script>
"""


class MJPEGHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def _simple(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            return self._simple(200, "text/html; charset=utf-8", INDEX_HTML)
        if path == "/healthz":
            h = SOURCE.health()
            body = ("\n".join(f"{k}={v}" for k, v in h.items()) + "\n").encode()
            return self._simple(200, "text/plain; charset=utf-8", body)
        if path.startswith("/hls/"):
            name = path[len("/hls/"):]
            allowed = name == "stream.m3u8"
            if name.startswith("segment") and name.endswith(".ts"):
                allowed = name[len("segment"):-len(".ts")].isdigit()
            if not HLS_ENABLE or not allowed:
                return self._simple(404, "text/plain", b"not found\n")
            try:
                with open(os.path.join(HLS_DIR, name), "rb") as fh:
                    body = fh.read()
            except OSError:
                return self._simple(404, "text/plain", b"HLS not ready\n")
            ctype = ("application/vnd.apple.mpegurl" if name.endswith(".m3u8")
                     else "video/mp2t")
            return self._simple(200, ctype, body)
        if path in ("/snapshot.jpg", "/snapshot", "/frame.jpg"):
            frame, _ = SOURCE.latest()
            if frame is None:
                return self._simple(503, "text/plain", b"no frame yet\n")
            return self._simple(200, "image/jpeg", frame)
        if path == "/jtaglog":
            # Audit trail: who sent what, when.  The lock is ADVISORY, so
            # a writer that ignores it still gets through -- this log is
            # the backstop that makes such an interleave visible after
            # the fact instead of just corrupting an operation silently.
            try:
                with open(JTAG_LOGFILE) as fh:
                    tail = fh.readlines()[-200:]
                body = "".join(tail).encode()
            except OSError:
                body = b"(no JTAG log yet)\n"
            return self._simple(200, "text/plain; charset=utf-8", body)
        if path == "/input":
            q = urllib.parse.parse_qs(
                self.path.split("?", 1)[1] if "?" in self.path else ""
            )
            return self._input(q)
        if path in ("/stream.mjpg", "/stream", "/stream.mjpeg"):
            return self._stream()
        # Anything else streams too, so old callers that just GET / with a
        # stream parser keep working.
        return self._stream()

    def _input(self, q):
        """Forward an ADB action to the JTAG REPL.

        Deliberately a small fixed vocabulary rather than a passthrough:
        this port is reachable by anything on the LAN, and the REPL can
        halt the CPU, write memory and reprogram the FPGA.  Only the verbs
        enumerated below reach it -- there is no general command channel.

        That vocabulary is no longer purely adb-*: `reset`, `sense`, the
        pram-* pair and now `nmi` all change machine state.  `nmi` is the
        sharpest of them (it interrupts a HEALTHY machine too), so it is the
        one verb that additionally requires confirm=yes.
        """
        act = (q.get("a") or [""])[0]
        reply_s = 0.0                 # seconds to wait for the REPL's answer

        def clamp63(v):
            # ADB packets carry 7-bit SIGNED deltas; anything past +-63 is
            # not representable in one event and would wrap.
            return max(-63, min(63, v))

        try:
            if act == "move":
                dx = clamp63(int((q.get("dx") or ["0"])[0]))
                dy = clamp63(int((q.get("dy") or ["0"])[0]))
                cmds = [f"adb-mouse {dx} {dy}"]
            elif act == "click":
                cmds = ["adb-mouse 0 0 down", "adb-mouse 0 0 up"]
            elif act == "press":
                cmds = ["adb-mouse 0 0 down"]
            elif act == "release":
                cmds = ["adb-mouse 0 0 up"]
            elif act == "type":
                cmds = text_to_key_cmds((q.get("s") or [""])[0])
                if not cmds:
                    return self._simple(200, "text/plain", b"nothing typeable\n")
            elif act == "key":
                code = int((q.get("code") or ["0"])[0], 0) & 0x7F
                cmds = [f"adb-key 0x{code:02X} press"]
            elif act == "cmd":
                # Command-<key> chord.  The modifier must be held ACROSS the
                # keypress -- Mac OS latches the modifier state at key-down,
                # so sending them as three independent events in the wrong
                # order gives a bare keystroke, not a menu command.
                ch = (q.get("k") or [""])[0][:1].lower()
                code = KEYCODES.get(ch)
                if code is None:
                    return self._simple(400, "text/plain", b"no keycode for that char\n")
                cmds = [f"adb-key 0x{KEY_CMD:02X} down",
                        f"adb-key 0x{code:02X} press",
                        f"adb-key 0x{KEY_CMD:02X} up"]
            elif act == "return":
                cmds = [f"adb-key 0x{KEY_RETURN:02X} press"]
            elif act == "delete":
                cmds = [f"adb-key 0x{KEY_DELETE:02X} press"]
            elif act == "status":
                cmds = ["adb-status"]
            elif act == "reset":
                # WARM reset.  Reboots the Mac.  Recoverable and intended --
                # but it is the first verb here that is not an input event, so
                # it stays an explicit named action rather than anything
                # general.
                #
                # This drives DBG_CONTROL's cold-reset pulse, which feeds
                # fpga_top's dbg_rst_src_level (alongside vio_boot_ctrl[3] and
                # btn2) and therefore does NOT assert core_rst.  boot_fsm sees
                # zero_en=0 and skips the 256 MiB RAM pre-zero pass, so the
                # machine is back in ~0.2 s rather than the measured ~4.06 s.
                # The trade is that the previous session's RAM is still there.
                cmds = ["reset"]
            elif act == "coldreset":
                # COLD reset.  vio-hard-reset pulses the PLATFORM reset (the
                # same path btn3 drives), which asserts core_rst -- clearing
                # fpga_top's warm-boot flag, so boot_fsm runs the full RAM
                # pre-zero pass and the machine boots on blank memory.
                #
                # Deliberately a separate button rather than a modifier on the
                # one above: the two differ in what state survives, which is
                # precisely the thing you do not want to get wrong while
                # chasing a boot bug.
                cmds = ["vio-hard-reset"]
            elif act == "nmi":
                # THE PROGRAMMER'S SWITCH.  Level 7 is the 68k's
                # non-maskable interrupt, so this is the one control that can
                # break into a machine that is spinning with interrupts
                # masked -- the alternative being a reset, which destroys the
                # very state you wanted to look at.
                #
                # Guarded twice, and the two guards protect different things:
                # the page's confirm() stops a stray CLICK, and this
                # confirm=yes parameter stops a stray REQUEST (a bookmark, a
                # browser prefetch, a curl of the URL list, a retry of a
                # logged GET).  Neither one covers the other's case, which is
                # why an NMI takes both.
                if (q.get("confirm") or [""])[0] != "yes":
                    return self._simple(400, "text/plain",
                                        b"NMI needs &confirm=yes\n")
                cmds = ["irq-inject 7"]
                # Worth the held lock: an NMI is exactly the moment you need
                # to know whether the REPL is really there and really ran it.
                reply_s = 3.0
            elif act == "ramwindow":
                try:
                    lg2 = int((q.get("lg2") or ["-1"])[0], 0)
                except ValueError:
                    lg2 = -1
                if lg2 not in RAM_WINDOWS:
                    return self._simple(400, "text/plain",
                                        b"ram window not in the known-good set\n")
                # Same shape as `sense`: Mac OS sizes RAM once at early boot,
                # so setting the CSR on a booted system changes nothing until
                # a reset.  Doing both here is the point -- a bare ram-window
                # would look like it silently did nothing.
                cmds = [f"ram-window {lg2}", "reset"]
            elif act in ("osswapstatus", "provisionstatus"):
                with OS_SWAP_STATE_LOCK:
                    running, tag = OS_SWAP["running"], OS_SWAP["tag"]
                    kind = OS_SWAP["kind"]
                    rc, started = OS_SWAP["rc"], OS_SWAP["started"]
                    tail = list(OS_SWAP["lines"])[-3:]
                label = _provision_label(kind, tag)
                if running:
                    body = (f"running {label} — "
                            f"{int(time.time() - started)}s elapsed\n"
                            + "\n".join(tail))
                elif rc is None:
                    body = "idle"
                elif rc == 0:
                    body = f"done: {label} written, verified, and booted"
                else:
                    body = (f"FAILED (rc={rc}) provisioning {label}\n"
                            + "\n".join(tail))
                return self._simple(200, "text/plain; charset=utf-8",
                                    body.encode() + b"\n")
            elif act in ("osswap", "romrestore"):
                kind = "rom" if act == "romrestore" else "os"
                tag = "rom" if kind == "rom" else (q.get("img") or [""])[0]
                if kind == "os" and tag not in OS_IMAGES:
                    return self._simple(400, "text/plain", b"unknown OS image\n")
                # Same gate as NMI: this one destroys the boot volume.
                if (q.get("confirm") or [""])[0] != "yes":
                    return self._simple(400, "text/plain",
                                        b"provisioning requires confirm=yes\n")
                with OS_SWAP_STATE_LOCK:
                    if OS_SWAP["running"]:
                        return self._simple(
                            503, "text/plain",
                            f"a provisioning job ({OS_SWAP['tag']}) is already "
                            f"running\n".encode())
                # Take the JTAG lock HERE, not in the worker, so this click
                # can be refused synchronously if the REPL is busy -- and hold
                # it for the whole swap (released by the worker).
                try:
                    lock_fd = os.open(JTAG_LOCKFILE,
                                      os.O_RDWR | os.O_CREAT, 0o666)
                except OSError as exc:
                    return self._simple(
                        503, "text/plain",
                        f"cannot open JTAG lock: {exc.strerror}\n".encode())
                try:
                    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError:
                    holder = _lock_holder()
                    os.close(lock_fd)
                    return self._simple(
                        503, "text/plain",
                        f"JTAG busy — held by {holder}. Provisioning cannot "
                        f"share the cable.\n".encode())
                # A FIFO pathname can survive long after its reader dies.
                # Test for an actual reader before starting the worker; a
                # blocking shell redirect to a readerless FIFO otherwise
                # wedges forever before the script's own timeout begins.
                try:
                    fifo_fd = os.open(JTAG_FIFO,
                                      os.O_WRONLY | os.O_NONBLOCK)
                except OSError as exc:
                    fcntl.flock(lock_fd, fcntl.LOCK_UN)
                    os.close(lock_fd)
                    return self._simple(
                        503, "text/plain; charset=utf-8",
                        (f"JTAG REPL not listening on {JTAG_FIFO} "
                         f"({exc.strerror}).{_vivado_hint()}\n").encode())
                os.close(fifo_fd)
                os.ftruncate(lock_fd, 0)
                os.write(lock_fd, f"{os.getpid()} mjpeg-web {kind}".encode())
                with OS_SWAP_STATE_LOCK:
                    OS_SWAP.update(running=True, tag=tag, kind=kind, rc=None,
                                   started=time.time(), lines=[])
                script_log = ("sd_write_rom.sh" if kind == "rom" else
                              f"sd_os_swap.sh {tag}")
                _jtag_log("web", script_log, note="START ")
                threading.Thread(target=_provision_worker,
                                 args=(kind, tag, lock_fd),
                                 daemon=True).start()
                return self._simple(
                    200, "text/plain; charset=utf-8",
                    f"provisioning {_provision_label(kind, tag)} — the Mac goes down for the "
                    f"duration; watch the status beside the picker\n".encode())
            elif act == "sense":
                code = int((q.get("code") or ["-1"])[0], 0)
                if code not in SENSE_CODES:
                    return self._simple(400, "text/plain",
                                        b"sense code not in the known-good set\n")
                # Mac OS samples the sense pins ONLY at DAFB init, so setting
                # the code on a booted system changes nothing visible until a
                # reset.  Doing both here is the whole point -- a bare
                # mon-sense would look like it silently did nothing.
                cmds = [f"mon-sense 0x{code:02X}", "reset"]
            elif act == "wprot":
                # vhdd CTRL[2] (macqd700-soc rtl/soc/vhdd_ctrl.v).  With no
                # `on` parameter this READS the current state into the JTAG
                # log; with one it sets or clears the bit.
                #
                # Refusing the write properly -- CHECK CONDITION / DATA
                # PROTECT -- rather than silently dropping it is deliberate:
                # a silent drop would leave the guest believing the write
                # succeeded, so its in-memory filesystem state would diverge
                # from the disk, which is the corruption this exists to stop.
                #
                # The bit does NOT survive a load-bit: the CSR default is
                # unlocked, so re-arm it after programming.
                on = (q.get("on") or [None])[0]
                if on is None:
                    cmds = ["vhdd-wprot"]
                elif on in ("0", "1"):
                    cmds = [f"vhdd-wprot {'on' if on == '1' else 'off'}"]
                else:
                    return self._simple(400, "text/plain",
                                        b"wprot: on must be 0 or 1\n")
            elif act in ("pramsave", "pramload", "pramdump", "pramclear"):
                # PRAM persistence (macqd700-soc rtl/soc/pram_sd.v).  All
                # four are MANUAL by design -- there is no autosave anywhere
                # in the hardware -- so exposing them as explicit buttons is
                # the whole interface, not a shortcut for something the
                # machine also does by itself.
                #
                # pram-save / pram-load touch the SD card.  The RTL will not
                # preempt an SD transfer that is already in flight, but a
                # SCSI command the Mac starts DURING one stalls on
                # sd_ctrl's ~10 s watchdog; the REPL prints that warning
                # itself when the CPU is not halted, and jtag_send returns
                # the REPL's output, so it reaches the browser.
                cmds = {
                    "pramsave":  ["pram-save"],
                    "pramload":  ["pram-load"],
                    "pramdump":  ["pram-dump"],
                    "pramclear": ["pram-clear"],
                }[act]
            else:
                return self._simple(400, "text/plain", b"unknown action\n")
        except ValueError:
            return self._simple(400, "text/plain", b"bad parameter\n")

        ok, detail = jtag_send(cmds, reply=reply_s)
        body = (("sent: " if ok else "ERROR: ") + detail + "\n").encode()
        return self._simple(200 if ok else 503, "text/plain; charset=utf-8", body)

    def _stream(self):
        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Connection", "close")
        self.end_headers()

        seq = 0
        interval = 1.0 / STREAM_FPS
        next_send = time.monotonic()
        try:
            while True:
                frame, seq = SOURCE.next_after(seq, timeout=10.0)
                if frame is None:
                    continue                      # idle tick; keep waiting
                delay = next_send - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                    # Do not send the frame captured before the sleep: slow
                    # viewers should always see the freshest available image.
                    frame, seq = SOURCE.latest()
                next_send = max(next_send + interval, time.monotonic())
                self.wfile.write(
                    b"--" + BOUNDARY.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                                   # viewer went away

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    threading.Thread(target=SOURCE.run_forever, daemon=True).start()
    server = ThreadingHTTPServer((BIND, PORT), MJPEGHandler)
    quality = (f"re-encoded quality={JPEG_QUALITY}" if JPEG_QUALITY is not None
               else "capture JPEG pass-through")
    print(f"MJPEG stream: http://{BIND}:{PORT}/  "
          f"(device={DEVICE} {WIDTH}x{HEIGHT}@{FPS}, stream={STREAM_FPS:g}fps, "
          f"{quality})")
    print(f"  viewer   http://{BIND}:{PORT}/")
    print(f"  snapshot http://{BIND}:{PORT}/snapshot.jpg")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
