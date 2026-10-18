#!/usr/bin/env python3
"""board_ab.py -- automated board A/B measurement: load a bitstream, wait for the
Finder, run Speedometer 4.02's Benchmark Mix, OCR the result window, read the
REPL `perf` counters, and emit one JSON per bitstream.  `compare` diffs two JSONs.

THE BOARD BELONGS TO THE OWNER.  This tool reprograms the FPGA and types on the
Mac's keyboard; run it only inside an owner-granted window, never in a loop.
`--dry-run` exercises every step against recorded fixtures and touches nothing.

Subcommands
  run <bit> <ltx> [label]     full measurement (see --help of `run`)
  compare <ref.json> <new.json>
  ocr <img>...                OCR Benchmark Mix screenshots, print JSON
  classify <img>...           Finder / Sad Mac / MacsBug / ... screen classifier
  learn <labels.json>         rebuild the digit templates from labelled images
  selftest                    OCR every labelled fixture; exit 1 unless 100%

Dependencies: python3 + Pillow only.  The HDMI capture server
(tools/fpga_mjpeg_server.py) supplies /snapshot.jpg; the JTAG REPL
(tools/jtag_repl.tcl) is driven through its FIFO under the same flock
(/tmp/jtag_in.lock) that tools/jt.sh and the web panel honour.

OCR: the Mac screen is 640x480 and lands 1:1 at (320,120) in a 1280x720 capture
(1.5x at (480,180) in 1920x1080).  The result window is located by its three red
rules (fixed 45/168 px spacing), every row and column is a fixed offset from
them, and each digit is classified by Hamming distance against bitmaps learned
from real screenshots (the font is fixed: 6x9 digits, 2x2 period).  Values are
cross-checked arithmetically -- rating*seconds is a per-test constant (the
Quadra 605 time) and Average == mean of the 10 ratings -- so an OCR slip shows
up as a failed check instead of a plausible wrong number.
"""

import argparse
import fcntl
import io
import json
import os
import re
import socket
import statistics
import sys
import time
import urllib.request

try:
    from PIL import Image
except ImportError:  # pragma: no cover
    sys.exit("board_ab.py needs Pillow (python3-pil)")

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, "board_ab_fixtures")

# --------------------------------------------------------------------------
# Result-window geometry (Mac pixels, relative to the rule after Dhrystones)
# --------------------------------------------------------------------------
ROWS = [  # key, label on screen, text-top relative to rule 2
    ("KWhetstones", "KWhetstones/sec", -37),
    ("Dhrystones", "Dhrystones/sec", -17),
    ("Towers", "Towers (sec)", 11),
    ("QuickSort", "Quick Sort (sec)", 31),
    ("BubbleSort", "Bubble Sort (sec)", 51),
    ("Queens", "Queens (sec)", 71),
    ("Puzzle", "Puzzle (sec)", 91),
    ("Permutations", "Permutations (sec)", 111),
    ("IntMatrix", "Int. Matrix (sec)", 131),
    ("Sieve", "Sieve (sec)", 151),
]
ROW_KEYS = [r[0] for r in ROWS]
AVG_TOP = 175
RULE_GAPS = (45, 168)          # rule1 -> rule2 -> rule3
COL_ABS_R = 195                # right edge of the Abs. column, relative to rule x0
COL_RAT_R = 268
COL_ITR_R = 303
COL_TOL = (8, 4)               # word right edge may sit this far (left, right) of the column edge

# rating * abs(seconds) for the timed tests == Quadra 605 seconds; Dhrystones
# is a rate so abs / rating == Quadra 605 Dhrystones/sec.  Midpoints of the
# intersection of the rounding intervals of 24-39 labelled screenshots.
Q605 = {
    "Dhrystones": 17273.75,
    "Towers": 0.6390, "QuickSort": 0.7127, "BubbleSort": 0.7604,
    "Queens": 0.4035, "Puzzle": 1.0918, "Permutations": 0.8138,
    "IntMatrix": 0.8052, "Sieve": 1.3727,
}

# Digit bitmaps, learned by `board_ab.py learn` from real 1280x720 captures.
GLYPHS = {}  # filled from board_ab_glyphs.json below
GLYPH_FILE = os.path.join(HERE, "board_ab_glyphs.json")


def _load_glyphs():
    global GLYPHS
    try:
        with open(GLYPH_FILE) as fh:
            GLYPHS = {k: v for k, v in json.load(fh).items()}
    except OSError:
        GLYPHS = {}


_load_glyphs()

# --------------------------------------------------------------------------
# Capture -> Mac framebuffer coordinates
# --------------------------------------------------------------------------
KNOWN_RECTS = {
    (1280, 720): (320, 120, 960, 600),
    (1920, 1080): (480, 180, 1440, 900),
}


def load_image(src):
    if isinstance(src, Image.Image):
        return src.convert("RGB")
    if isinstance(src, (bytes, bytearray)):
        return Image.open(io.BytesIO(src)).convert("RGB")
    return Image.open(src).convert("RGB")


def mac_screen(img, rect=None):
    """Return the 640x480 Mac screen as an RGB image."""
    img = load_image(img)
    rect = rect or KNOWN_RECTS.get(img.size)
    if rect is None:
        # Unknown capture mode: assume a centred 4:3 picture filling the height.
        w, h = img.size
        mw = h * 4 // 3
        rect = ((w - mw) // 2, 0, (w + mw) // 2, h)
    m = img.crop(rect)
    if m.size != (640, 480):
        # NEAREST samples one capture pixel inside each Mac pixel's footprint;
        # an averaging filter smears 1-px rules and glyph strokes at 1.5x.
        m = m.resize((640, 480), Image.NEAREST)
    return m


# --------------------------------------------------------------------------
# Screen classifier
# --------------------------------------------------------------------------
def classify_screen(img, rect=None):
    """Classify a capture: finder | speedometer | shutdown_dialog | sadmac |
    macsbug | black | other.  Thresholds calibrated on ~86 board captures
    (Finder, Speedometer, boot grey, Mac OS splash, Sad Mac, MacsBug,
    'not shut down properly' dialog, scrambled video)."""
    m = mac_screen(img, rect)
    g = m.convert("L")
    px, cp = g.load(), m.load()
    menubar = sum(1 for y in range(2, 17) for x in range(40, 620) if px[x, y] > 200) / (15 * 580)
    mb_line = sum(1 for x in range(640) if px[x, 19] < 90 or px[x, 20] < 90) / 640
    apple = sum(1 for y in range(2, 18) for x in range(12, 32)
                if max(cp[x, y]) - min(cp[x, y]) > 80)
    small = g.resize((320, 240), Image.BOX)
    sp = small.load()
    grey = sum(1 for y in range(12, 240) for x in range(320) if 95 < sp[x, y] < 175) / (228 * 320)
    dark = sum(1 for y in range(0, 240, 2) for x in range(0, 320, 2) if sp[x, y] < 40) / (120 * 160)
    white = sum(1 for y in range(0, 240, 2) for x in range(0, 320, 2) if sp[x, y] > 215) / (120 * 160)
    centre_white = sum(1 for y in range(60, 110) for x in range(130, 190) if sp[x, y] > 200) / (50 * 60)
    feats = dict(menubar=round(menubar, 3), menubar_line=round(mb_line, 3), apple=apple,
                 grey=round(grey, 3), dark=round(dark, 3), white=round(white, 3),
                 centre_white=round(centre_white, 3))
    state = "other"
    has_menu = menubar >= 0.75 and mb_line >= 0.9 and apple >= 40
    if has_menu and find_rules(m) is not None:
        state = "speedometer"
    elif has_menu and grey >= 0.35:
        state = "finder"
    elif has_menu:
        state = "menubar"           # some app in front, desktop mostly covered
    elif dark >= 0.97:
        # Black screen.  A Sad Mac is the icon plus an 8-hex-digit code line:
        # on every captured one the white bbox is x 281..360, y 222..296 and
        # the code row (y 285..298) carries ink.  Other black-screen icons
        # (seen right after a load) are NOT failures: they time out instead.
        pts = [(x, y) for y in range(180, 330) for x in range(240, 400) if px[x, y] > 150]
        code = sum(1 for x, y in pts if 285 <= y <= 298)
        if pts and code >= 60 and min(x for x, _ in pts) >= 270 and max(x for x, _ in pts) <= 372:
            state = "sadmac"
        else:
            state = "black"
    elif white >= 0.80 and mb_line < 0.2 and menubar > 0.95:
        state = "macsbug"
    elif menubar > 0.95 and apple == 0 and centre_white > 0.6 and grey > 0.3:
        # blank menu bar + a big white box mid-screen: the boot-time
        # "not shut down properly" alert (needs a Return)
        state = "shutdown_dialog"
    elif grey >= 0.9:
        state = "boot_grey"
    feats["state"] = state
    return feats


# --------------------------------------------------------------------------
# Benchmark Mix OCR
# --------------------------------------------------------------------------
def _is_red(p):
    # tolerant of JPEG 4:2:0 chroma smear on a 1-px rule (pure 218,0,0 can
    # arrive as ~117,43,34 in a 1.5x-scaled capture)
    return p[0] >= 100 and p[0] - p[1] > 50 and p[0] - p[2] > 50


def find_rules(m):
    """Locate the result window: (x0, y_rule2) or None."""
    px = m.load()
    cand = []
    for y in range(480):
        run = best = 0
        start = bstart = 0
        for x in range(640):
            if _is_red(px[x, y]):
                if run == 0:
                    start = x
                run += 1
                if run > best:
                    best, bstart = run, start
            else:
                run = 0
        if best >= 150:
            cand.append((y, bstart))
    # collapse adjacent rows (a scaled capture smears a 1px rule over 2 rows)
    rules = []
    for y, x in cand:
        if rules and y - rules[-1][0] <= 2:
            continue
        rules.append((y, x))
    for i, (y2, x2) in enumerate(rules):
        # rule1 (under the header) may be hidden by a pulled-down menu; the
        # 168 px rule2->rule3 pair with a common left edge is distinctive enough
        if any(abs((y3 - y2) - RULE_GAPS[1]) <= 2 and abs(x3 - x2) <= 2 for y3, x3 in rules[i + 1:]):
            return x2, y2
    return None


def _dark_mask(g, x_lo, x_hi, y_lo, y_hi, thr=110):
    px = g.load()
    return [[1 if px[x, y] < thr else 0 for x in range(x_lo, x_hi)] for y in range(y_lo, y_hi)]


def _words(mask, x_lo, gap=7):
    cols = [i for i in range(len(mask[0])) if any(r[i] for r in mask)]
    words = []
    if not cols:
        return words
    s = p = cols[0]
    for c in cols[1:]:
        if c - p > gap:
            words.append((s + x_lo, p + x_lo))
            s = c
        p = c
    words.append((s + x_lo, p + x_lo))
    return words


def _glyphs_of(g, x0, x1, y_lo, y_hi):
    """Split [x0,x1] in the band into glyph bitmaps (rows aligned to word top)."""
    mask = _dark_mask(g, x0, x1 + 1, y_lo, y_hi)
    rows_with = [i for i, r in enumerate(mask) if any(r)]
    if not rows_with:
        return []
    top = rows_with[0]
    h = 9
    sub = [mask[i] if i < len(mask) else [0] * len(mask[0]) for i in range(top, top + h)]
    out = []
    w = len(sub[0])
    c = 0
    while c < w:
        if any(r[c] for r in sub):
            s = c
            while c < w and any(r[c] for r in sub):
                c += 1
            out.append(["".join("#" if r[k] else "." for k in range(s, c)) for r in sub])
        else:
            c += 1
    # rows below the 9-row cell that still hold ink mean the word is not a number
    extra = any(any(mask[i]) for i in range(top + h, len(mask)))
    return out if not extra else out + [["?"]]


def _classify_glyph(bm):
    if bm == ["?"]:
        return "?", 99
    w = len(bm[0])
    best, bestd = "?", 99
    for ch, tmpls in GLYPHS.items():
        for t in tmpls:
            if abs(len(t[0]) - w) > 1:
                continue
            # compare on the max width, left-aligned and right-aligned; keep the better
            for align in ("l", "r"):
                tw = max(w, len(t[0]))
                a = [(r.ljust(tw, ".") if align == "l" else r.rjust(tw, ".")) for r in bm]
                b = [(r.ljust(tw, ".") if align == "l" else r.rjust(tw, ".")) for r in t]
                d = sum(1 for ra, rb in zip(a, b) for ca, cb in zip(ra, rb) if ca != cb)
                if d < bestd:
                    best, bestd = ch, d
    # The font renders pixel-exact (every glyph in 39 labelled captures matched
    # at distance 0); the closest two templates ('0'/'8') differ by 4 pixels.
    # Accept <=1 so a noisy glyph becomes '?' (-> human read), never a wrong digit.
    if bestd > 1:
        return "?", bestd
    return best, bestd


def _read_word(g, x0, x1, y_lo, y_hi):
    gl = _glyphs_of(g, x0, x1, y_lo, y_hi)
    s = "".join(_classify_glyph(b)[0] for b in gl)
    return s, gl


def _num(s, kind):
    if kind == "int":
        return int(s) if re.fullmatch(r"\d{1,3}", s or "") else None
    return float(s) if re.fullmatch(r"\d+\.\d{3}", s or "") else None


def read_benchmark(img, rect=None):
    """OCR a Benchmark Mix window.  Returns a dict; missing cells are None."""
    m = mac_screen(img, rect)
    loc = find_rules(m)
    res = {"window": loc is not None, "rows": {}, "average": None,
           "running_row": None, "raw": {}}
    if loc is None:
        return res
    x0, y2 = loc
    g = m.convert("L")
    px = m.load()

    def cell(top, right, kind):
        y_lo, y_hi = max(0, y2 + top - 2), min(480, y2 + top + 10)
        lo = max(0, x0 + right - 90)
        hi = min(639, x0 + right + 4)
        mask = _dark_mask(g, lo, hi + 1, y_lo, y_hi)
        for a, b in reversed(_words(mask, lo)):
            if right - COL_TOL[0] <= b - x0 <= right + COL_TOL[1]:
                s, _ = _read_word(g, a, b, y_lo, y_hi)
                return s, _num(s, kind)
        return "", None

    for key, _label, top in ROWS:
        a_s, a = cell(top, COL_ABS_R, "float")
        r_s, r = cell(top, COL_RAT_R, "float")
        i_s, i = cell(top, COL_ITR_R, "int")
        res["rows"][key] = {"abs": a, "rating": r, "itr": i}
        res["raw"][key] = [a_s, r_s, i_s]
        # Speedometer's run marker: a red dot left of the test name
        yc = y2 + top + 4
        red = sum(1 for yy in range(yc - 3, yc + 4) for xx in range(max(0, x0 - 6), x0 + 4)
                  if 0 <= yy < 480 and _is_red(px[xx, yy]))
        if red >= 6:
            res["running_row"] = key
    v_s, v = cell(AVG_TOP, COL_RAT_R, "float")
    res["average"] = v
    res["raw"]["Average"] = v_s
    res["checks"] = check_values(res)
    rws = res["rows"]
    res["complete_abs"] = all(rws[k]["abs"] is not None for k in ROW_KEYS)
    res["complete"] = (res["complete_abs"] and res["average"] is not None and
                       all(rws[k]["rating"] is not None for k in ROW_KEYS))
    # The end-of-run alert ("The tests are complete") covers the Rat./Itr.
    # columns of Towers..Puzzle while Permutations..Sieve stay visible.
    res["alert"] = (res["complete_abs"] and
                    all(rws[k]["rating"] is None for k in ROW_KEYS[2:7]) and
                    all(rws[k]["rating"] is not None for k in ROW_KEYS[7:]))
    return res


def check_values(res):
    """Arithmetic cross-checks.  Each entry: True / False / None (not checkable)."""
    out = {}
    rows = res["rows"]
    for k in ROW_KEYS[1:]:
        a, r = rows[k]["abs"], rows[k]["rating"]
        if a is None or r is None or r <= 0.0005 or a <= 0.0005:
            out[k] = None
            continue
        q = Q605[k]
        # both displayed values are rounded to 0.001: the true product/ratio
        # lies in an interval; Q605 must fall inside it (plus a small margin)
        if k == "Dhrystones":
            lo, hi, m = (a - 0.0005) / (r + 0.0005), (a + 0.0005) / (r - 0.0005), 2.0
        else:
            lo, hi, m = (a - 0.0005) * (r - 0.0005), (a + 0.0005) * (r + 0.0005), 0.001 * q
        out[k] = lo - m <= q <= hi + m
    kw = rows["KWhetstones"]
    out["KWhetstones"] = None if kw["rating"] is None else True
    rs = [rows[k]["rating"] for k in ROW_KEYS]
    if res["average"] is not None and all(v is not None for v in rs):
        out["Average"] = abs(sum(rs) / len(rs) - res["average"]) <= 0.0015
    else:
        out["Average"] = None
    return out


def ocr_ok(res):
    """True when every readable value passed every applicable check."""
    return res["window"] and all(v is not False for v in res.get("checks", {}).values())


# --------------------------------------------------------------------------
# Template learning
# --------------------------------------------------------------------------
def _fmt3(v):
    return "%.3f" % v


def learn(labels_path, out_path=GLYPH_FILE):
    with open(labels_path) as fh:
        labels = json.load(fh)
    base = os.path.dirname(os.path.abspath(labels_path))
    glyphs = {}
    for ent in labels["images"]:
        if not ent.get("learn"):
            continue
        m = mac_screen(os.path.join(base, ent["file"]))
        x0, y2 = find_rules(m)
        g = m.convert("L")
        exp = ent["rows"]

        def take(top, right, text):
            if text is None:
                return
            y_lo, y_hi = y2 + top - 2, y2 + top + 10
            lo = x0 + right - 90
            mask = _dark_mask(g, lo, x0 + right + 5, y_lo, y_hi)
            for a, b in reversed(_words(mask, lo)):
                if right - COL_TOL[0] <= b - x0 <= right + COL_TOL[1]:
                    gl = _glyphs_of(g, a, b, y_lo, y_hi)
                    if len(gl) != len(text):
                        raise SystemExit(f"learn: {ent['file']} {text!r}: {len(gl)} glyphs")
                    for ch, bm in zip(text, gl):
                        lst = glyphs.setdefault(ch, [])
                        if bm not in lst:
                            lst.append(bm)
                    return
            raise SystemExit(f"learn: {ent['file']}: no word for {text!r}")

        for key, _l, top in ROWS:
            row = exp.get(key) or {}
            if row.get("abs") is not None:
                take(top, COL_ABS_R, _fmt3(row["abs"]))
            if row.get("rating") is not None:
                take(top, COL_RAT_R, _fmt3(row["rating"]))
            if row.get("itr") is not None:
                take(top, COL_ITR_R, str(row["itr"]))
        if ent.get("average") is not None:
            take(AVG_TOP, COL_RAT_R, _fmt3(ent["average"]))
    with open(out_path, "w") as fh:
        json.dump(glyphs, fh, indent=0, sort_keys=True)
    _load_glyphs()
    return {k: len(v) for k, v in glyphs.items()}


def selftest(labels_path=None, verbose=False):
    """OCR every labelled image; compare every digit.  Returns (n_ok, n_total, fails)."""
    labels_path = labels_path or os.path.join(FIXTURES, "labels.json")
    with open(labels_path) as fh:
        labels = json.load(fh)
    base = os.path.dirname(os.path.abspath(labels_path))
    fails = []
    n_cells = n_ok = 0
    for ent in labels["images"]:
        path = ent["file"] if os.path.isabs(ent["file"]) else os.path.join(base, ent["file"])
        if not os.path.exists(path):
            fails.append((ent["file"], "missing"))
            continue
        if "paste_at" in ent:   # lossless crop of an HDMI frame -> rebuild the frame
            frame = Image.new("RGB", tuple(ent.get("frame", (1280, 720))))
            frame.paste(Image.open(path).convert("RGB"), tuple(ent["paste_at"]))
            res = read_benchmark(frame)
        else:
            res = read_benchmark(path)
        bad = []
        for key in ROW_KEYS:
            want = ent["rows"].get(key, {})
            for f in ("abs", "rating", "itr"):
                if f not in want:
                    continue
                n_cells += 1
                if res["rows"][key][f] == want[f]:
                    n_ok += 1
                else:
                    bad.append(f"{key}.{f}: got {res['rows'][key][f]} want {want[f]}")
        if "average" in ent:
            n_cells += 1
            if res["average"] == ent["average"]:
                n_ok += 1
            else:
                bad.append(f"Average: got {res['average']} want {ent['average']}")
        if "running_row" in ent:
            n_cells += 1
            if res["running_row"] == ent["running_row"]:
                n_ok += 1
            else:
                bad.append(f"running_row: got {res['running_row']} want {ent['running_row']}")
        if "alert" in ent:
            n_cells += 1
            if res["alert"] == ent["alert"]:
                n_ok += 1
            else:
                bad.append(f"alert: got {res['alert']} want {ent['alert']}")
        if not ocr_ok(res) and ent.get("checks_pass", True):
            bad.append(f"checks failed: {res['checks']}")
        if bad:
            fails.append((ent["file"], bad))
        if verbose:
            print(f"{'OK ' if not bad else 'BAD'} {ent['file']}" + ("" if not bad else f"  {bad}"))
    for ent in labels.get("screens", []):
        path = os.path.join(base, ent["file"])
        st = classify_screen(path)["state"]
        n_cells += 1
        if st == ent["state"]:
            n_ok += 1
        else:
            fails.append((ent["file"], [f"state: got {st} want {ent['state']}"]))
        if verbose:
            print(f"{'OK ' if st == ent['state'] else 'BAD'} {ent['file']} state={st}")
    return n_ok, n_cells, fails


# --------------------------------------------------------------------------
# REPL output parsing
# --------------------------------------------------------------------------
PERF_LINE = re.compile(r"^>\s{2,}([A-Za-z][A-Za-z0-9 :/\-().]+?)\s{2,}([0-9][0-9,]*|--|READ-FAILED)(?:\s|$)")


def _key(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def parse_perf(text):
    """Parse one `perf` report into {counters:{...}, ipc, free_cycles, ...}.
    Generic on the counter NAME so new counters land automatically."""
    out = {"counters": {}}
    for line in text.splitlines():
        m = PERF_LINE.match(line)
        if m and not line.lstrip("> ").startswith(("derived", "free-running", "NOTE")):
            v = m.group(2)
            out["counters"][_key(m.group(1))] = None if v in ("--", "READ-FAILED") else int(v.replace(",", ""))
        m = re.search(r"free-running .*cycles=([0-9,]+) macros=([0-9,]+)", line)
        if m:
            out["free_cycles"] = int(m.group(1).replace(",", ""))
            out["free_macros"] = int(m.group(2).replace(",", ""))
        m = re.search(r"OFF_PERF_CTL=(0x[0-9A-Fa-f]+) counters=(\d+)", line)
        if m:
            out["perf_ctl"] = m.group(1)
            out["n_counters"] = int(m.group(2))
    if "REFUSED" in text:
        out["refused"] = True
    c = out["counters"]
    cyc, inst = c.get("window_cycles"), c.get("retired_macro_insts")
    der = {}
    if cyc and inst:
        der["ipc"] = inst / cyc
        k = inst / 1000.0
        for name, v in c.items():
            if name in ("window_cycles", "retired_macro_insts") or v is None:
                continue
            if name.startswith("stall_cycles"):
                der[name + "_pct"] = 100.0 * v / cyc
            else:
                der[name + "_per_kinst"] = v / k
    out["derived"] = der
    return out


def parse_build_id(text):
    m = re.findall(r"build_id = (0x[0-9A-Fa-f]{8})", text)
    return m[-1] if m else None


def parse_exc_count(text):
    m = re.findall(r"exc_count = 0x([0-9A-Fa-f]+)", text)
    return int(m[-1], 16) if m else None


FAULT_VECTORS = {0x02: "bus error", 0x03: "address error", 0x04: "illegal",
                 0x05: "zero divide", 0x08: "privilege", 0x0e: "format error"}


def parse_exc_ring(text):
    vecs = [int(v, 16) for v in re.findall(r"exc\[\s*\d+\] vec=0x([0-9a-fA-F]+)", text)]
    return vecs


def parse_halt(text):
    d = {}
    for k in ("effective", "dbl_fault", "manual", "exc_halt"):
        m = re.search(r"\b%s=(\d+)" % k, text)
        if m:
            d[k] = int(m.group(1))
    m = re.search(r"pc_live=(0x[0-9a-fA-F]+)", text)
    if m:
        d["pc_live"] = m.group(1)
    return d


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------
class Clock:
    def now(self):
        return time.time()

    def sleep(self, s):
        if s > 0:
            time.sleep(s)


class FakeClock(Clock):
    def __init__(self):
        self.t = 1_760_000_000.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


class Repl:
    """JTAG REPL over its FIFO.  Holds the shared flock for the whole command
    (write -> '> READY'), exactly like tools/jt.sh, and logs to the audit log."""

    def __init__(self, fifo="/tmp/jtag_in", out="/tmp/jtag_out", lock="/tmp/jtag_in.lock",
                 log="/tmp/jtag_cmd.log", tag="board_ab", lock_wait=600, transcript=None):
        self.fifo, self.out, self.lockf, self.log = fifo, out, lock, log
        self.tag, self.lock_wait, self.transcript = tag, lock_wait, transcript

    def cmd(self, line, timeout=60):
        if not os.path.exists(self.fifo):
            raise RuntimeError(f"{self.fifo} missing -- is the JTAG REPL running?")
        fd = os.open(self.lockf, os.O_RDWR | os.O_CREAT, 0o666)
        try:
            t0 = time.time()
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except OSError:
                    if time.time() - t0 > self.lock_wait:
                        raise RuntimeError("JTAG lock busy")
                    time.sleep(0.5)
            os.ftruncate(fd, 0)
            os.write(fd, f"{os.getpid()} {self.tag}".encode())
            try:
                with open(self.log, "a") as fh:
                    fh.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} pid={os.getpid()} origin={self.tag} {line}\n")
            except OSError:
                pass
            start = os.stat(self.out).st_size if os.path.exists(self.out) else 0
            ff = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)
            try:
                os.write(ff, (line.rstrip("\n") + "\n").encode())
            finally:
                os.close(ff)
            deadline = time.time() + timeout
            text = ""
            while time.time() < deadline:
                time.sleep(0.25)
                with open(self.out, "rb") as fh:
                    fh.seek(start)
                    text = fh.read().decode("utf-8", "replace")
                if re.search(r"^> READY", text, re.M):
                    text = text[: re.search(r"^> READY", text, re.M).start()]
                    break
            else:
                text += f"\n[board_ab: no READY within {timeout}s]\n"
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
        if self.transcript:
            self.transcript.write(f">>> {line}\n{text}\n")
            self.transcript.flush()
        return text


class Snapper:
    def __init__(self, base="http://10.200.0.12:8080", timeout=10):
        self.base, self.timeout = base.rstrip("/"), timeout

    def snap(self):
        with urllib.request.urlopen(self.base + "/snapshot.jpg", timeout=self.timeout) as r:
            return r.read()

    def health(self):
        try:
            with urllib.request.urlopen(self.base + "/healthz", timeout=self.timeout) as r:
                return r.read().decode()
        except (OSError, socket.timeout) as e:
            return f"ERROR {e}"


# --------------------------------------------------------------------------
# Dry-run fakes: a scripted board that replays recorded fixtures
# --------------------------------------------------------------------------
class FakeBoard:
    """Simulates REPL + HDMI.  Screens are fixture JPEGs chosen by a tiny state
    machine driven by the commands the harness sends; `perf` synthesises a
    report from a recorded one with cycles scaled to (virtual) wall time."""

    def __init__(self, clock, mhz=200.0, scenario="ok", fixtures=FIXTURES, transcript=None):
        self.clock, self.mhz, self.scenario, self.fx = clock, mhz, scenario, fixtures
        self.transcript = transcript
        with open(os.path.join(fixtures, "labels.json")) as fh:
            self.labels = json.load(fh)
        self.phase = "off"
        self.t_phase = clock.now()
        self.perf_t0 = clock.now()
        self.exc = 0x100
        self.runs_done = 0
        self.with_s = False
        self.keys = []
        with open(os.path.join(fixtures, "perf_sample.txt")) as fh:
            self.perf_tmpl = fh.read()

    def _set(self, ph):
        self.phase, self.t_phase = ph, self.clock.now()

    def _f(self, name):
        return os.path.join(self.fx, self.labels["dryrun"][name])

    def _read(self, name):
        with open(self._f(name), "rb") as fh:
            return fh.read()

    # REPL ------------------------------------------------------------
    def cmd(self, line, timeout=60):
        t = self._cmd(line)
        if self.transcript:
            self.transcript.write(f">>> {line}\n{t}\n")
        return t

    def _cmd(self, line):
        tok = line.split()
        c = tok[0]
        el = self.clock.now() - self.t_phase
        if c == "load-bit":
            self.clock.sleep(8)
            self._set("boot")
            return f"> programmed {tok[1]}\n> jtag_repl: build_id = 0xDC31F130\n"
        if c == "build-id":
            if self.phase == "off":          # --no-load: pretend it is already up
                self._set("finder")
            return "> jtag_repl: build_id = 0xDC31F130\n"
        if c == "exc-count":
            self.exc += int(2000 * max(1.0, el)) if self.scenario != "storm" else int(9e6 * max(1.0, el))
            return f"> exc_count = 0x{self.exc & 0xffffffff:08x}\n"
        if c in ("last-exceptions", "exc-ring", "exc"):
            vec = 0x02 if self.scenario == "storm" else 0x0a
            lines = [f"> exc[{i:2d}] vec=0x{vec:02x} pc=0x4080dc00 fa=0x00000000 handler=0x4080dbfe"
                     for i in range(31, -1, -1)]
            return "> exc-ring head=0 — last 32 entries\n" + "\n".join(lines) + "\n"
        if c == "halt-status":
            return ("> halt: reason=0x00000000 pc_live=0x4080dd54 manual=0 halt_after=0 break_pc=0 "
                    "exc_halt=0 dbl_fault=0 effective=0 exc_count=0x00000000\n")
        if c in ("perf-clear",) or (c == "perf" and len(tok) > 1 and tok[1] == "clear"):
            self.perf_t0 = self.clock.now()
            return "> perf-clear: counters zeroed, window RUNNING\n"
        if c == "perf":
            return self._perf()
        if c == "adb-key":
            code = int(tok[1], 0)
            act = tok[2] if len(tok) > 2 else "press"
            self.keys.append((code, act))
            self._on_key(code, act)
            return f"> adb-key 0x{code:02X} {act}\n"
        if c == "adb-mouse":
            return f"> adb-mouse dx={tok[1]} dy={tok[2]}\n"
        return f"> (dry-run: no canned reply for {c})\n"

    def _perf(self):
        win = self.clock.now() - self.perf_t0
        cyc = int(win * self.mhz * 1e6)
        m = re.search(r"window cycles\s+([0-9,]+)", self.perf_tmpl)
        base = int(m.group(1).replace(",", ""))
        f = cyc / base

        def scale(mm):
            v = int(mm.group(2).replace(",", ""))
            return mm.group(1) + f"{int(v * f):,}".rjust(len(mm.group(2))) + mm.group(3)
        out = []
        for ln in self.perf_tmpl.splitlines():
            if PERF_LINE.match(ln) and "derived" not in ln and "free-running" not in ln:
                ln = re.sub(r"^(>\s+[A-Za-z].*?\s{2,})([0-9][0-9,]*)(\s|$)", scale, ln)
            out.append(ln)
        return "\n".join(out) + "\n"

    def _on_key(self, code, act):
        down_cmd = getattr(self, "_cmd_down", False)
        if code == 0x37:
            self._cmd_down = act == "down"
            return
        if self.phase == "finder" and code == 0x01:
            self.with_s = True
        elif self.phase == "finder" and down_cmd and code == 0x1F and self.with_s:
            self._set("app")
        elif self.phase == "app" and down_cmd and code == 0x0B:
            self._set("mixdlg")
        elif self.phase == "mixdlg" and code == 0x24:
            self._set("running")
        elif self.phase == "alert" and code == 0x24:
            self.runs_done += 1
            self._set("done")
        elif self.phase == "done" and down_cmd and code == 0x0B:
            self._set("mixdlg")

    # HDMI ------------------------------------------------------------
    def snap(self):
        el = self.clock.now() - self.t_phase
        ph = self.phase
        if self.scenario == "sadmac" and ph in ("boot", "finder"):
            return self._read("sadmac")
        if ph == "off":
            return self._read("garbage")
        if ph == "boot":
            if el > 60:
                self._set("finder")
                return self._read("finder")
            return self._read("boot")
        if ph in ("finder", "app", "mixdlg"):
            return self._read("finder")
        if ph == "running":
            if el > 90:
                self._set("alert")
                return self._read("alert")
            return self._read("running")
        if ph == "alert":
            return self._read("alert")
        return self._read("final")

    def health(self):
        return "frames_total=1\nage_s=0.0\n(dry-run)\n"


# --------------------------------------------------------------------------
# Harness
# --------------------------------------------------------------------------
KEY = {"s": 0x01, "o": 0x1F, "b": 0x0B, "return": 0x24, "tab": 0x30, "cmd": 0x37, "shift": 0x38}
KEYCODES = {
    'a': 0x00, 's': 0x01, 'd': 0x02, 'f': 0x03, 'h': 0x04, 'g': 0x05, 'z': 0x06, 'x': 0x07,
    'c': 0x08, 'v': 0x09, 'b': 0x0B, 'q': 0x0C, 'w': 0x0D, 'e': 0x0E, 'r': 0x0F, 'y': 0x10,
    't': 0x11, '1': 0x12, '2': 0x13, '3': 0x14, '4': 0x15, '6': 0x16, '5': 0x17, '=': 0x18,
    '9': 0x19, '7': 0x1A, '-': 0x1B, '8': 0x1C, '0': 0x1D, ']': 0x1E, 'o': 0x1F, 'u': 0x20,
    '[': 0x21, 'i': 0x22, 'p': 0x23, 'l': 0x25, 'j': 0x26, "'": 0x27, 'k': 0x28, ';': 0x29,
    '\\': 0x2A, ',': 0x2B, '/': 0x2C, 'n': 0x2D, 'm': 0x2E, '.': 0x2F, ' ': 0x31,
}

# Owner's verified recipe (memory: how-to-run-speedometer-on-the-board):
# type-to-select "S" in the open Speedometer folder, Cmd-O, ~15 s, Return x3.
DEFAULT_LAUNCH = "key:s; wait:3; cmd:o; wait:15; return; wait:3; return; wait:3; return; wait:3"


class Abort(Exception):
    pass


class Harness:
    def __init__(self, args, repl, snapper, clock, outdir, log):
        self.a, self.repl, self.snapper, self.clock = args, repl, snapper, clock
        self.outdir, self.logf = outdir, log
        self.exc_hist = []

    def log(self, msg):
        line = f"[{time.strftime('%H:%M:%S', time.localtime(self.clock.now()))}] {msg}"
        print(line, flush=True)
        self.logf.write(line + "\n")
        self.logf.flush()

    # -- primitives ---------------------------------------------------------
    def snap(self, name=None):
        data = self.snapper.snap()
        if name:
            with open(os.path.join(self.outdir, name), "wb") as fh:
                fh.write(data)
        return data

    def key(self, code, act="press"):
        self.repl.cmd(f"adb-key 0x{code:02X} {act}", timeout=10)

    def chord(self, ch):
        self.key(KEY["cmd"], "down")
        self.key(KEYCODES[ch])
        self.key(KEY["cmd"], "up")

    def mouse_to(self, x, y):
        """Absolute move: pin at the origin, then step out with |delta|<=4
        (Mac OS passes small deltas 1:1, accelerates large ones)."""
        for _ in range(14):
            self.repl.cmd("adb-mouse -63 -63", timeout=10)
        while x > 0 or y > 0:
            dx, dy = min(4, x), min(4, y)
            self.repl.cmd(f"adb-mouse {dx} {dy}", timeout=10)
            x, y = x - dx, y - dy

    def run_script(self, script):
        for step in [s.strip() for s in script.split(";") if s.strip()]:
            op, _, arg = step.partition(":")
            op = op.strip().lower()
            arg = arg.strip()
            if op == "wait":
                self.clock.sleep(float(arg))
            elif op == "key":
                code = int(arg, 0) if arg.startswith("0x") else KEYCODES[arg.lower()]
                if len(arg) == 1 and arg.isupper():
                    self.key(KEY["shift"], "down"); self.key(code); self.key(KEY["shift"], "up")
                else:
                    self.key(code)
            elif op == "type":
                for ch in arg:
                    self.key(KEYCODES[ch.lower()])
            elif op == "cmd":
                self.chord(arg.lower())
            elif op == "return":
                self.key(KEY["return"])
            elif op == "tab":
                for _ in range(int(arg or 1)):
                    self.key(KEY["tab"])
            elif op == "mouse":
                x, y = (int(v) for v in arg.split(","))
                self.mouse_to(x, y)
            elif op == "click":
                self.repl.cmd("adb-mouse 0 0 down", timeout=10)
                self.repl.cmd("adb-mouse 0 0 up", timeout=10)
            elif op == "dblclick":
                for _ in range(2):
                    self.repl.cmd("adb-mouse 0 0 down", timeout=10)
                    self.repl.cmd("adb-mouse 0 0 up", timeout=10)
            elif op == "snap":
                self.snap(f"{arg or 'step'}.jpg")
            else:
                raise Abort(f"launch script: unknown step {step!r}")

    # -- health ---------------------------------------------------------------
    def health_check(self):
        """Fail fast on a double fault / halted core or a fault-vector storm."""
        h = parse_halt(self.repl.cmd("halt-status", timeout=20))
        if h.get("dbl_fault") or h.get("effective"):
            raise Abort(f"core halted/double-faulted: {h}")
        n = parse_exc_count(self.repl.cmd("exc-count", timeout=20))
        now = self.clock.now()
        if n is not None:
            self.exc_hist.append((now, n))
        if len(self.exc_hist) >= 2:
            (t0, n0), (t1, n1) = self.exc_hist[-2], self.exc_hist[-1]
            rate = ((n1 - n0) & 0xffffffff) / max(1e-3, t1 - t0)
            if rate > self.a.exc_storm_rate:
                vecs = parse_exc_ring(self.repl.cmd("last-exceptions", timeout=30))
                faults = [v for v in vecs if v in FAULT_VECTORS]
                if vecs and len(faults) >= 0.75 * len(vecs):
                    top = max(set(faults), key=faults.count)
                    raise Abort(f"exception storm: {rate:,.0f} exc/s, ring {len(faults)}/{len(vecs)} "
                                f"fault vectors (mostly 0x{top:02x} {FAULT_VECTORS[top]})")
                self.log(f"  high exception rate {rate:,.0f}/s but ring is benign (A-line/IRQ)")

    def wait_finder(self):
        t0 = self.clock.now()
        last_h = -1e9
        stable = 0
        dialog_since = None
        bad = 0
        while True:
            el = self.clock.now() - t0
            if el > self.a.boot_timeout:
                self.snap("boot_timeout.jpg")
                raise Abort(f"no Finder after {self.a.boot_timeout:.0f}s")
            st = classify_screen(self.snap(), self.a.mac_rect)
            s = st["state"]
            if s in ("sadmac", "macsbug"):
                bad += 1
                if bad >= 2:
                    self.snap(f"fail_{s}.jpg")
                    raise Abort(f"{s} on screen at t={el:.0f}s")
            else:
                bad = 0
            if s == "shutdown_dialog":
                dialog_since = dialog_since or self.clock.now()
                if self.clock.now() - dialog_since > 5:
                    self.log("  'not shut down properly' dialog -> Return")
                    self.key(KEY["return"])
                    dialog_since = None
            if s in ("finder", "menubar", "speedometer"):
                stable += 1
                if stable >= 2:
                    self.log(f"  Finder at t={el:.0f}s ({s}); settling {self.a.settle:.0f}s")
                    self.clock.sleep(self.a.settle)
                    return el
            else:
                stable = 0
            if self.clock.now() - last_h >= self.a.health_every:
                last_h = self.clock.now()
                self.health_check()
            self.clock.sleep(self.a.poll)

    def perf_read(self, t_clear):
        txt = self.repl.cmd("perf", timeout=60)
        t = self.clock.now()
        p = parse_perf(txt)
        cyc = p["counters"].get("window_cycles")
        p["window_s"] = round(t - t_clear, 3)
        if cyc and t > t_clear:
            p["clock_mhz"] = round(cyc / (t - t_clear) / 1e6, 2)
        return p

    def run_mix(self, i):
        self.repl.cmd("perf-clear", timeout=30)
        t_clear = self.clock.now()
        self.chord("b")
        self.clock.sleep(2)
        self.key(KEY["return"])          # "Run Set" with the default selection
        t_start = self.clock.now()
        seen_run = False
        prev = None
        same = 0
        alert_seen = False
        while True:
            el = self.clock.now() - t_start
            if el > self.a.mix_timeout:
                self.snap(f"run{i}_timeout.jpg")
                raise Abort(f"Benchmark Mix did not finish in {self.a.mix_timeout:.0f}s")
            data = self.snap()
            st = classify_screen(data, self.a.mac_rect)
            if st["state"] in ("sadmac", "macsbug"):
                with open(os.path.join(self.outdir, f"run{i}_fail.jpg"), "wb") as fh:
                    fh.write(data)
                raise Abort(f"{st['state']} during run {i}")
            r = read_benchmark(data, self.a.mac_rect)
            if r["running_row"]:
                seen_run = True
            if el > 30 and not r["window"] and not seen_run:
                with open(os.path.join(self.outdir, f"run{i}_nowindow.jpg"), "wb") as fh:
                    fh.write(data)
                raise Abort("no Benchmark Mix window 30 s after Cmd-B/Return -- did Speedometer launch? "
                            "(see --launch)")
            sig = json.dumps(r["rows"], sort_keys=True)
            same = same + 1 if sig == prev else 0
            prev = sig
            done = (r["window"] and r["running_row"] is None and r["complete_abs"] and
                    (r["alert"] or (seen_run and same >= 2)))
            if done:
                break
            self.clock.sleep(self.a.poll)
        perf = self.perf_read(t_clear)
        t_end = self.clock.now()
        name = f"run{i}.jpg"
        with open(os.path.join(self.outdir, name), "wb") as fh:
            fh.write(data)
        alert_seen = r["alert"]
        if alert_seen or not r["complete"]:
            self.key(KEY["return"])      # dismiss "The tests are complete"
            self.clock.sleep(2)
            data2 = self.snap(f"run{i}_final.jpg")
            r2 = read_benchmark(data2, self.a.mac_rect)
            # merge: abs from either frame, ratings from the uncovered one
            for k in ROW_KEYS:
                for f in ("abs", "rating", "itr"):
                    if r2["rows"][k][f] is not None:
                        r["rows"][k][f] = r2["rows"][k][f]
            if r2["average"] is not None:
                r["average"] = r2["average"]
            r["checks"] = check_values(r)
            name = f"run{i}_final.jpg"
        ok = ocr_ok(r) and all(r["rows"][k]["abs"] is not None for k in ROW_KEYS)
        rec = {"run": i, "rows": r["rows"], "average": r["average"], "ocr_ok": ok,
               "checks": r["checks"], "snapshot": name, "alert_seen": alert_seen,
               "run_seconds": round(t_end - t_start, 1), "perf": perf}
        if not ok:
            self.log(f"  !! OCR NOT TRUSTED for run {i}: read the JPEG by eye: {os.path.join(self.outdir, name)}")
        return rec

    def idle_window(self, i):
        self.repl.cmd("perf-clear", timeout=30)
        t = self.clock.now()
        self.clock.sleep(self.a.idle_seconds)
        p = self.perf_read(t)
        p["idx"] = i
        return p


PROVENANCE_FILES = ("provenance.txt", "cpu040.provenance", "fpga_top.buildinfo",
                    "fpga_top.buildinfo.resume", "fpga_top.synth_selection")


def read_provenance(bit):
    """Build manifests next to the bitstream.  build_id is the SoC hash and does
    NOT tell two core configs apart -- the env/flags in provenance.txt do."""
    d = os.path.dirname(os.path.abspath(bit))
    out = {}
    for f in PROVENANCE_FILES:
        try:
            with open(os.path.join(d, f)) as fh:
                out[f] = fh.read()[:4000]
        except OSError:
            pass
    return out


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def summarise(runs):
    s = {"n": len(runs), "rows": {}}
    for k in ROW_KEYS:
        a = [r["rows"][k]["abs"] for r in runs]
        rt = [r["rows"][k]["rating"] for r in runs]
        s["rows"][k] = {"abs_mean": _mean(a), "rating_mean": _mean(rt),
                        "abs": a, "rating": rt}
    s["average_mean"] = _mean([r["average"] for r in runs])
    s["average"] = [r["average"] for r in runs]
    ders = [r["perf"].get("derived", {}) for r in runs]
    keys = sorted({k for d in ders for k in d})
    s["derived_mean"] = {k: _mean([d.get(k) for d in ders]) for k in keys}
    s["clock_mhz"] = [r["perf"].get("clock_mhz") for r in runs]
    return s


def cmd_run(a):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    label = a.label or os.path.basename(os.path.dirname(os.path.abspath(a.bit))) or "bit"
    outdir = a.outdir or os.path.join(os.getcwd(), "board_ab_out", f"{stamp}-{label}")
    os.makedirs(outdir, exist_ok=True)
    tr = open(os.path.join(outdir, "repl_transcript.txt"), "w")
    logf = open(os.path.join(outdir, "log.txt"), "w")
    if a.dry_run:
        clock = FakeClock()
        board = FakeBoard(clock, mhz=a.expect_mhz or 200.0, scenario=a.dry_scenario, transcript=tr)
        repl = snapper = board
        if a.poll > 5:
            a.poll = 5
    else:
        clock = Clock()
        repl = Repl(tag=f"board_ab-{label}", transcript=tr)
        snapper = Snapper(a.server)
    h = Harness(a, repl, snapper, clock, outdir, logf)
    res = {"label": label, "bit": os.path.abspath(a.bit), "ltx": os.path.abspath(a.ltx) if a.ltx else None,
           "dry_run": bool(a.dry_run), "started": stamp,
           "launch": a.launch, "runs": [], "idle": [], "status": "running"}
    res["provenance"] = read_provenance(a.bit)
    if a.expect_mhz is None:
        m = re.search(r"target_freq_mhz=(\d+)", res["provenance"].get("cpu040.provenance", ""))
        a.expect_mhz = float(m.group(1)) if m else 200.0
    res["expect_mhz"] = a.expect_mhz
    jpath = os.path.join(outdir, "result.json")

    def save():
        res["summary"] = summarise(res["runs"]) if res["runs"] else None
        with open(jpath, "w") as fh:
            json.dump(res, fh, indent=1)

    rc = 0
    try:
        h.log(f"board_ab {label}: outdir {outdir}{'  [DRY RUN]' if a.dry_run else ''}")
        hz = snapper.health()
        if hz.startswith("ERROR"):
            raise Abort(f"HDMI capture server unreachable: {hz}")
        if not a.no_load:
            h.log(f"load-bit {a.bit}")
            txt = repl.cmd(f"load-bit {a.bit} {a.ltx}".strip(), timeout=300)
            if "programmed" not in txt:
                raise Abort(f"load-bit failed:\n{txt[-800:]}")
            res["build_id"] = parse_build_id(txt)
            if not a.dry_run:
                clock.sleep(2)
        else:
            res["build_id"] = parse_build_id(repl.cmd("build-id", timeout=30))
        h.log(f"build_id {res.get('build_id')}")
        if not a.no_boot_wait:
            h.log("waiting for Finder")
            res["boot_seconds"] = round(h.wait_finder(), 1)
        h.snap("finder.jpg")
        if not a.no_launch:
            h.log(f"launch: {a.launch}")
            h.run_script(a.launch)
            h.snap("launched.jpg")
        for i in range(1, a.runs + 1):
            h.log(f"Benchmark Mix run {i}/{a.runs}")
            rec = h.run_mix(i)
            res["runs"].append(rec)
            save()
            p = rec["perf"]
            d = p.get("derived", {})
            h.log(f"  Dhry {rec['rows']['Dhrystones']['abs']}  Avg {rec['average']}  "
                  f"IPC {d.get('ipc', float('nan')):.4f}  clock {p.get('clock_mhz')} MHz  "
                  f"ocr_ok={rec['ocr_ok']}  {rec['run_seconds']}s")
            mhz = p.get("clock_mhz")
            if mhz and abs(mhz - a.expect_mhz) > 0.05 * a.expect_mhz:
                h.log(f"  !! CLOCK CHECK: cycles/wall = {mhz} MHz, expected {a.expect_mhz} -- wrong build?")
                rec["clock_warning"] = True
            if i < a.runs:
                h.clock.sleep(a.between)
        if a.idle and a.idle_prep:
            h.log(f"idle prep: {a.idle_prep}")
            h.run_script(a.idle_prep)
        for i in range(1, a.idle + 1):
            h.log(f"Finder-idle window {i}/{a.idle} ({a.idle_seconds:.0f}s)")
            res["idle"].append(h.idle_window(i))
            save()
        res["status"] = "ok"
    except Abort as e:
        res["status"] = f"FAILED: {e}"
        h.log(f"ABORT: {e}")
        rc = 2
    except Exception as e:  # keep partial results
        res["status"] = f"ERROR: {type(e).__name__}: {e}"
        h.log(res["status"])
        rc = 3
    save()
    h.log(f"wrote {jpath}")
    tr.close()
    logf.close()
    if res["runs"]:
        print(format_result(res))
        if a.ref:
            cmd_compare(argparse.Namespace(ref=a.ref, new=jpath))
    return rc


def format_result(res):
    s = res["summary"]
    lines = [f"{res['label']}  build_id={res.get('build_id')}  runs={s['n']}  status={res['status']}"]
    for k in ROW_KEYS:
        r = s["rows"][k]
        abss = " ".join("--" if v is None else f"{v:.3f}" for v in r["abs"])
        lines.append(f"  {k:<13} abs[{abss}]  rating {r['rating_mean'] if r['rating_mean'] is None else round(r['rating_mean'], 3)}")
    lines.append(f"  Average       {s['average']}")
    d = s["derived_mean"]
    if "ipc" in d:
        lines.append(f"  IPC {d['ipc']:.4f}  clock {s['clock_mhz']} MHz")
    bad = [r["run"] for r in res["runs"] if not r["ocr_ok"]]
    if bad:
        lines.append(f"  OCR NOT TRUSTED for runs {bad}: read the saved JPEGs")
    return "\n".join(lines)


# --------------------------------------------------------------------------
# compare
# --------------------------------------------------------------------------
SHORT = {"Dhrystones": "Dhry", "Towers": "Tow", "QuickSort": "QSort", "BubbleSort": "Bub",
         "Queens": "Queens", "Puzzle": "Puz", "Permutations": "Perm", "IntMatrix": "IntM",
         "Sieve": "Sieve"}


def _summary_of(path):
    with open(path) as fh:
        d = json.load(fh)
    if "summary" in d and d["summary"]:
        return d, d["summary"]
    if "rows" in d:  # a single `ocr` output or a hand-written reference
        fake = {"runs": [{"rows": d["rows"], "average": d.get("average"), "perf": {}}]}
        return d, summarise(fake["runs"])
    raise SystemExit(f"{path}: no summary/rows")


def cmd_compare(a):
    dr, sr = _summary_of(a.ref)
    dn, sn = _summary_of(a.new)
    print(f"ref {dr.get('label')} (n={sr['n']})  vs  new {dn.get('label')} (n={sn['n']})   "
          f"[rating delta: + = new faster]")
    print(f"  {'test':<13} {'ref abs':>12} {'new abs':>12} {'ref rat':>8} {'new rat':>8} {'d rating':>9}  spread(new)")
    one = []
    for k in ROW_KEYS[1:]:
        r, n = sr["rows"][k], sn["rows"][k]
        ra, na, rr, nr = r["abs_mean"], n["abs_mean"], r["rating_mean"], n["rating_mean"]
        if rr and nr:
            dp = 100.0 * (nr / rr - 1)
        elif ra and na:  # ratings unread: derive from abs
            dp = 100.0 * ((na / ra) - 1) if k == "Dhrystones" else 100.0 * (ra / na - 1)
        else:
            dp = None
        vals = [v for v in n["abs"] if v is not None]
        spread = (f"{100.0 * (max(vals) - min(vals)) / statistics.mean(vals):.2f}%" if len(vals) > 1 else "-")
        f = lambda v, p=3: "--" if v is None else f"{v:.{p}f}"
        print(f"  {k:<13} {f(ra):>12} {f(na):>12} {f(rr):>8} {f(nr):>8} "
              f"{'--' if dp is None else f'{dp:+.2f}%':>9}  {spread}")
        if dp is not None:
            one.append(f"{SHORT[k]} {dp:+.1f}%")
    ra, na = sr["average_mean"], sn["average_mean"]
    if ra and na:
        dp = 100.0 * (na / ra - 1)
        print(f"  {'Average':<13} {'':>12} {'':>12} {ra:>8.3f} {na:>8.3f} {dp:+8.2f}%")
        one.insert(0, f"AVG {dp:+.1f}%")
    dr_, dn_ = sr.get("derived_mean", {}), sn.get("derived_mean", {})
    common = [k for k in dn_ if k in dr_ and dr_[k]]
    if common:
        print("  counters (mean over runs):")
        for k in common:
            print(f"    {k:<36} {dr_[k]:>12.4f} {dn_[k]:>12.4f} {100.0 * (dn_[k] / dr_[k] - 1):+8.2f}%")
    print("ONE-LINE: " + " | ".join(one))
    return 0


# --------------------------------------------------------------------------
def cmd_ocr(a):
    out = []
    for p in a.images:
        r = read_benchmark(p)
        r.pop("raw", None) if not a.raw else None
        r["file"] = p
        r["ocr_ok"] = ocr_ok(r)
        out.append(r)
    print(json.dumps(out if len(out) > 1 else out[0], indent=1))
    return 0 if all(o["ocr_ok"] for o in out) else 1


def cmd_classify(a):
    for p in a.images:
        print(json.dumps({"file": p, **classify_screen(p)}))
    return 0


def cmd_learn(a):
    print(learn(a.labels))
    return 0


def cmd_selftest(a):
    ok, n, fails = selftest(a.labels, verbose=a.verbose)
    for f, why in fails:
        print(f"FAIL {f}: {why}")
    print(f"selftest: {ok}/{n} cells correct ({100.0 * ok / max(1, n):.2f}%)")
    return 0 if ok == n and not fails else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="measure one bitstream on the board")
    r.add_argument("bit")
    r.add_argument("ltx", nargs="?", default="")
    r.add_argument("label", nargs="?", default=None)
    r.add_argument("--runs", type=int, default=2, help="Benchmark Mix repeats (default 2)")
    r.add_argument("--idle", type=int, default=2, help="Finder-idle perf windows after the runs (default 2)")
    r.add_argument("--ref", help="reference result.json: print the compare table + ONE-LINE at the end")
    r.add_argument("--idle-seconds", type=float, default=60)
    r.add_argument("--idle-prep", default="cmd:q; wait:5",
                   help="launch-style script run before the idle windows (default quits Speedometer)")
    r.add_argument("--outdir")
    r.add_argument("--server", default=os.environ.get("BOARD_AB_SERVER", "http://10.200.0.12:8080"))
    r.add_argument("--expect-mhz", type=float, default=None,
                   help="core clock for the cycles/wall check (default: target_freq_mhz from "
                        "cpu040.provenance next to the .bit, else 200)")
    r.add_argument("--boot-timeout", type=float, default=420)
    r.add_argument("--settle", type=float, default=20, help="seconds after the Finder appears before typing")
    r.add_argument("--mix-timeout", type=float, default=600)
    r.add_argument("--poll", type=float, default=1.0)
    r.add_argument("--between", type=float, default=5, help="seconds between mix runs")
    r.add_argument("--health-every", type=float, default=15)
    r.add_argument("--exc-storm-rate", type=float, default=2e6,
                   help="exc/s above which the exception ring is inspected for a fault storm")
    r.add_argument("--launch", default=os.environ.get("BOARD_AB_LAUNCH", DEFAULT_LAUNCH),
                   help="';'-separated steps: key:<c|0xNN> type:<txt> cmd:<c> return tab[:n] "
                        "wait:<s> mouse:<x>,<y> click dblclick snap:<name>")
    r.add_argument("--no-load", action="store_true", help="skip load-bit (board already running it)")
    r.add_argument("--no-boot-wait", action="store_true")
    r.add_argument("--no-launch", action="store_true", help="Speedometer is already frontmost")
    r.add_argument("--mac-rect", type=lambda s: tuple(int(v) for v in s.split(",")), default=None,
                   help="x0,y0,x1,y1 of the Mac screen inside the capture (auto for 720p/1080p)")
    r.add_argument("--dry-run", action="store_true", help="replay fixtures; touch nothing")
    r.add_argument("--dry-scenario", default="ok", choices=["ok", "sadmac", "storm"])
    r.set_defaults(fn=cmd_run)
    c = sub.add_parser("compare")
    c.add_argument("ref")
    c.add_argument("new")
    c.set_defaults(fn=cmd_compare)
    o = sub.add_parser("ocr")
    o.add_argument("images", nargs="+")
    o.add_argument("--raw", action="store_true")
    o.set_defaults(fn=cmd_ocr)
    k = sub.add_parser("classify")
    k.add_argument("images", nargs="+")
    k.set_defaults(fn=cmd_classify)
    lr = sub.add_parser("learn")
    lr.add_argument("labels")
    lr.set_defaults(fn=cmd_learn)
    st = sub.add_parser("selftest")
    st.add_argument("--labels", default=None)
    st.add_argument("-v", "--verbose", action="store_true")
    st.set_defaults(fn=cmd_selftest)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
