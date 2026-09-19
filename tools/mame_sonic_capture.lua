-- mame_sonic_capture.lua  —  DP83932C SONIC register tap for macqd700.
--
-- Captures the bit-exact register access sequence the Q700 ROM + Mac OS
-- Ethernet driver exchange with the on-board National DP83932C SONIC.
-- Output is a CSV trace directly comparable with the RTL-side SONIC trace
-- ring in rtl/mac/q700_eth_sonic.v (read out over JTAG), via
-- tools/sonic_trace_diff.py.
--
-- Modelled on tools/mame_scsi96_capture.lua; read that file first, the
-- hard-won lessons about tap lifetime and tap width apply here verbatim.
--
-- ---------------------------------------------------------------------
-- OUTPUT FORMAT (self-describing: the header line names the columns, and
-- sonic_trace_diff.py honours that header, so the HW-side dumper can pick
-- a different column order as long as it emits its own "# columns:" line)
--
--     # columns: ns,rw,reg,data,lanes,flags
--     <ns>,<R|W>,<reg>,<data>,<lanes>,<flags>
--
--   ns     decimal emulated nanoseconds (informational; the differ drops it)
--   rw     R or W
--   reg    2 hex digits.
--            00..3f  SONIC register index  ((byte_offset & 0xff) >> 2)
--            40..47  Ethernet ID PROM byte  (0x50008000 + n), see below
--   data   4 hex digits, the 16-bit value on D15..D0, with any byte lane
--          the access did NOT touch forced to 00.  (The differ compares
--          only the touched lanes, so this convention is not load-bearing
--          — but it makes half-register accesses unambiguous by eye.)
--   lanes  1 hex digit, SAME ENCODING AS rtl/soc/peripheral_bus.v's
--          `sonic_wstrb`:
--            bit1 = D15..D8 touched  (physical byte +2 of the 32-bit slot)
--            bit0 = D7..D0  touched  (physical byte +3)
--          so 3 = full 16-bit access, 2 = high half only, 1 = low half only.
--          The Mac driver does half-register accesses; that detail is the
--          whole reason this column exists.
--   flags  '-' or a concatenation of:
--            L  the same CPU access also covered the open-bus half of the
--               32-bit slot (bytes +0/+1, D31..D16) — i.e. a MOVE.L or a
--               MOVE.W to the wrong half.  Nothing is connected there.
--            U  the access was in 0x5000a100..0x5000b0ff.  Our RTL mirrors
--               the 64 registers every 0x100 across that whole window;
--               MAME maps ONLY 0x5000a000..0x5000a0ff (see the umask32
--               ratio arithmetic in address_map::import_submaps), so in
--               MAME these go nowhere.  If any 'U' appears, the two sides
--               are NOT modelling the same aperture and the diff must be
--               read with that in mind.
--            P  Ethernet ID PROM byte (reg 40..47), not a SONIC register.
--
-- ---------------------------------------------------------------------
-- ADDRESS MAP (src/mame/apple/macquadra700.cpp:561-562)
--
--   map(0x50008000, 0x50008007).r(ethernet_mac_r).mirror(0x00fc0000);
--   map(0x5000a000, 0x5000b0ff).m(m_sonic, dp83932c_device::map)
--                              .umask32(0x0000ffff).mirror(0x00fc0000);
--
-- umask32(0x0000ffff) on a 32-bit big-endian space puts each 16-bit SONIC
-- register in the LOW half of a 32-bit slot, i.e. physical bytes +2/+3.
-- MAME's import ratio is 32/16 = 2, and the device submap is map(0x00,0x7f)
-- in device bytes, so the parent stride is 4 bytes per register and
--
--     register index = (byte offset within the window) >> 2
--
-- which is exactly rtl/soc/peripheral_bus.v's `sonic_addr = addr[7:2]`.
--
-- The .mirror(0x00fc0000) makes address bits 18..23 don't-care, so the
-- aperture repeats 64 times between 0x50000000 and 0x50fc0000.  We install
-- on all 64 by default (MAME_SONIC_TRACE_MIRRORS=all) rather than guessing
-- which one the driver happens to use: a missed mirror is an EMPTY trace,
-- which is indistinguishable from "the driver never ran".
--
-- The taps are OBSERVERS: install_read_tap sees the value the device
-- already returned, it does not re-issue the read.  Since dp83932c's
-- reg_r() is a pure `return m_reg[offset];` with no side effects, and
-- reg_w() is where every side effect lives, this capture is completely
-- non-destructive.
--
-- ---------------------------------------------------------------------
-- Usage:
--     MAME_SONIC_TRACE_OUT=build/mame_runs/sonic_q700.csv \
--     MAME_SONIC_TRACE_LIMIT=2000000 \
--     ~/mame_q700_good/run.sh -seconds_to_run 180 \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_sonic_capture.lua
--
-- Environment:
--     MAME_SONIC_TRACE_OUT      output path (default build/mame_runs/sonic_q700.csv)
--     MAME_SONIC_TRACE_LIMIT    max events, then exit MAME (default 2000000)
--     MAME_SONIC_TRACE_MIRRORS  "all" (default) | "min" | comma list of base
--                               addresses, e.g. "0x5000a000,0x50f0a000"
--     MAME_SONIC_TRACE_PROM     1 (default) to also tap the Ethernet ID PROM
--                               at 0x50008000..0x50008007, 0 to skip it
--     MAME_SONIC_TRACE_FLUSH    1 (default) flush after every event (safe
--                               against a crash/kill), 0 for speed

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path = os.getenv("MAME_SONIC_TRACE_OUT") or "build/mame_runs/sonic_q700.csv"
local limit    = tonumber(os.getenv("MAME_SONIC_TRACE_LIMIT") or "2000000")
local mirrors  = os.getenv("MAME_SONIC_TRACE_MIRRORS") or "all"
local do_prom  = (os.getenv("MAME_SONIC_TRACE_PROM") or "1") ~= "0"
local do_flush = (os.getenv("MAME_SONIC_TRACE_FLUSH") or "1") ~= "0"

local SONIC_BASE   = 0x5000a000
local SONIC_SPAN   = 0x1100          -- 0x5000a000..0x5000b0ff inclusive
local PROM_BASE    = 0x50008000
local PROM_SPAN    = 0x8
local MIRROR_MASK  = 0x00fc0000

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then
        os.execute(string.format("mkdir -p %q", dir))
    end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_sonic_capture v1 - DP83932C SONIC register trace (macqd700)\n")
file:write("# columns: ns,rw,reg,data,lanes,flags\n")
file:write("# reg 00-3f = SONIC register (offset>>2); 40-47 = Ethernet ID PROM byte\n")
file:write("# lanes bit1 = D15..D8 (byte +2), bit0 = D7..D0 (byte +3); same as sonic_wstrb\n")
file:write("# flags: L = access also spanned the open-bus half (D31..D16)\n")
file:write("#        U = access above 0x5000a0ff (RTL mirrors it, MAME does not map it)\n")
file:write("#        P = Ethernet ID PROM byte, not a SONIC register\n")
file:flush()

local count      = 0
local n_long     = 0
local n_unmapped = 0
local n_prom     = 0
local n_halfreg  = 0

local function ns_now()
    return math.floor(mach.time:as_double() * 1e9)
end

-- Build the list of mirror bases to tap.
local function mirror_bases(base, honour_list)
    local out = {}
    if honour_list and mirrors == "min" then
        -- Canonical base plus the 0x00f00000 mirror, which is the one the
        -- Q700 ROM was measured using for the TurboSCSI window.  Cheap, but
        -- it CAN miss; prefer "all" unless you are chasing tap overhead.
        out[#out+1] = base
        out[#out+1] = base | 0x00f00000
        return out
    elseif honour_list and mirrors ~= "all" then
        for tok in mirrors:gmatch("[^,]+") do
            tok = tok:gsub("%s", "")
            local v = tonumber(tok)
            if v then out[#out+1] = v end
        end
        if #out > 0 then return out end
        print("mame_sonic_capture: MAME_SONIC_TRACE_MIRRORS unparsable, falling back to 'all'")
    end
    -- Every combination of the don't-care bits 18..23.
    for i = 0, 63 do
        local extra = 0
        for b = 0, 5 do
            if ((i >> b) & 1) == 1 then extra = extra | (1 << (18 + b)) end
        end
        out[#out+1] = base | extra
    end
    return out
end

local function emit(ns, rw, reg, data, lanes, flags)
    file:write(string.format("%d,%s,%02x,%04x,%x,%s\n", ns, rw, reg, data, lanes, flags))
    count = count + 1
    if do_flush then file:flush() end
    if count >= limit then
        print(string.format("mame_sonic_capture: hit limit=%d, exiting", limit))
        file:flush(); file:close(); file = nil
        mach:exit()
    end
end

local function on_sonic(rw, offset, data, mask)
    if count >= limit or file == nil then return end
    local sub = offset & ~MIRROR_MASK          -- fold every mirror onto 0x5000xxxx
    if sub < SONIC_BASE or sub >= SONIC_BASE + SONIC_SPAN then return end

    local win = sub - SONIC_BASE
    local reg = (win >> 2) & 0x3f

    -- Big-endian 32-bit space: mem_mask bits 31..24 are the byte at the
    -- LOWEST address.  The SONIC is on D15..D0 = bytes +2/+3.
    local hi = ((mask >> 8)  & 0xff) ~= 0      -- byte +2, D15..D8
    local lo = ((mask >> 0)  & 0xff) ~= 0      -- byte +3, D7..D0
    local ob = ((mask >> 16) & 0xffff) ~= 0    -- bytes +0/+1, open bus

    local lanes = (hi and 2 or 0) | (lo and 1 or 0)
    if lanes == 0 then return end              -- open-bus half only: SONIC never sees it

    local flags = ""
    if ob then flags = flags .. "L"; n_long = n_long + 1 end
    if win >= 0x100 then flags = flags .. "U"; n_unmapped = n_unmapped + 1 end
    if flags == "" then flags = "-" end
    if lanes ~= 3 then n_halfreg = n_halfreg + 1 end

    -- Zero the lanes the access did not touch, so the printed value cannot
    -- be mistaken for a full-register value.
    local d16 = data & mask & 0xffff

    emit(ns_now(), rw, reg, d16, lanes, flags)
end

local function on_prom(rw, offset, data, mask)
    if count >= limit or file == nil then return end
    local sub = offset & ~MIRROR_MASK
    if sub < PROM_BASE or sub >= PROM_BASE + PROM_SPAN then return end
    for lane = 0, 3 do
        if ((mask >> (lane * 8)) & 0xff) ~= 0 then
            local byte_addr = (sub & ~3) + (3 - lane)
            local idx = byte_addr - PROM_BASE
            if idx >= 0 and idx < PROM_SPAN then
                local b = (data >> (lane * 8)) & 0xff
                n_prom = n_prom + 1
                emit(ns_now(), rw, 0x40 + idx, b, 1, "P")
            end
        end
    end
end

-- MUST be a GLOBAL.  The tap objects returned by install_*_tap are owned by
-- Lua; if the only reference is a local in the autoboot chunk, the chunk
-- returns, the GC collects the taps, and MAME segfaults calling through the
-- freed callback.  (Measured on the 53C96 tap: death after a growing,
-- non-deterministic event count — the use-after-free tell.)
_G.sonic_taps = {}
local taps = _G.sonic_taps

local sbases = mirror_bases(SONIC_BASE, true)
for i, base in ipairs(sbases) do
    taps[#taps+1] = prog:install_read_tap(base, base + SONIC_SPAN - 1,
        string.format("sonic_capture_r%d", i),
        function(offset, data, mask) on_sonic("R", offset, data, mask) end)
    taps[#taps+1] = prog:install_write_tap(base, base + SONIC_SPAN - 1,
        string.format("sonic_capture_w%d", i),
        function(offset, data, mask) on_sonic("W", offset, data, mask) end)
end

if do_prom then
    local pbases = mirror_bases(PROM_BASE, false)
    for i, base in ipairs(pbases) do
        taps[#taps+1] = prog:install_read_tap(base, base + PROM_SPAN - 1,
            string.format("sonic_prom_r%d", i),
            function(offset, data, mask) on_prom("R", offset, data, mask) end)
    end
end

print(string.format(
    "mame_sonic_capture: %d taps installed over %d SONIC mirror(s)%s (limit=%d) -> %s",
    #taps, #sbases, do_prom and " + Ethernet ID PROM" or "", limit, out_path))

local function on_stop()
    if file then
        file:write(string.format(
            "# summary: events=%d half_register=%d long_span=%d unmapped_page=%d prom=%d\n",
            count, n_halfreg, n_long, n_unmapped, n_prom))
        file:flush(); file:close(); file = nil
    end
    print(string.format(
        "mame_sonic_capture: stopped, %d events (half-register %d, long-span %d, "
        .. "unmapped-page %d, prom %d)",
        count, n_halfreg, n_long, n_unmapped, n_prom))
    if count == 0 then
        print("mame_sonic_capture: *** ZERO events.  That is NOT evidence the driver")
        print("mame_sonic_capture:     never touched the SONIC -- it is equally consistent")
        print("mame_sonic_capture:     with tapping the wrong mirror.  Re-run with")
        print("mame_sonic_capture:     MAME_SONIC_TRACE_MIRRORS=all before concluding anything.")
    end
end

-- The subscription handle is RAII (util::notifier::subscribe returns a
-- notifier_subscription; see luaengine.ipp make_notifier_adder).  Dropping it
-- UNSUBSCRIBES immediately and the stop callback silently never fires --
-- measured: without the global, no summary line and no stop print at all.
-- Same class of bug as the tap-lifetime one above.
if emu.add_machine_stop_notifier then
    _G.sonic_stop_sub = emu.add_machine_stop_notifier(on_stop)
else
    emu.register_stop(on_stop)
end
