-- mame_periph_strobe_capture.lua  —  Q700 peripheral WRITE strobe-width capture.
--
-- PURPOSE
--   Answer, from a real MAME macqd700 boot, the question SI11 / SI-OPEN-3 of
--   docs/superpowers/specs/2026-08-18-v1-shared-infra-fixes-design.md asks:
--   does real Mac ROM / System 7 driver code EVER issue a multi-hot-strobe
--   (WORD / LONG) write to the byte-granular or strided peripheral slots
--   decoded by rtl/soc/peripheral_bus.v?
--
--   This tool exists because tools/mame_via1_capture.lua CANNOT answer it:
--   its lane loop `break`s after the first hot byte lane (":133-142", ":154-160")
--   on the assumption that "the 68040 always issues byte-wide MMIO accesses to
--   VIA".  That assumption is baked into the instrument, so the instrument can
--   never falsify it.  Here we record the FULL mem_mask of every write, never
--   break, and classify by the same slot decode peripheral_bus.v uses.
--
-- OUTPUT
--   $MAME_STROBE_OUT        CSV, one row per CPU write into the tapped windows:
--                             seq,slot,addr,mask,hot,size,data,pc,cycles
--                           where
--                             slot   peripheral_bus.v slot name (VIA1/VIA2/...)
--                             addr   32-bit aligned handler offset of bit[31:24]
--                             mask   raw 32-bit mem_mask as MAME presents it
--                             hot    popcount of hot BYTE lanes (1..4)
--                             size   architectural byte count (== hot for a
--                                    contiguous run; == hot otherwise)
--                             data   raw 32-bit data word
--                             pc     m68k PC at the access (CURPC if available)
--                           By default only rows with hot > 1 are written, plus
--                           a small sample of hot==1 rows for sanity; set
--                           MAME_STROBE_ALL=1 to log every write.
--   $MAME_STROBE_SUMMARY    per-slot x hot-lane-count histogram, written at exit.
--
-- ENV
--   MAME_STROBE_OUT       (default build/mame_runs/periph_strobe.csv)
--   MAME_STROBE_SUMMARY   (default <OUT>.summary.txt)
--   MAME_STROBE_ALL       1 => log every write, not just multi-hot ones
--   MAME_STROBE_LIMIT     max CSV rows (default 2000000)
--   MAME_STROBE_SECONDS   stop after N emulated seconds (0 = run to -seconds_to_run)
--   MAME_STROBE_SNAPDIR   if set, save a screen snapshot every 5 emulated
--                         seconds there (boot-progress evidence)
--   MAME_STROBE_SAMPLE1   log 1-in-N of the hot==1 rows (default 5000, 0 = none)
--
-- USAGE
--   MAME_STROBE_OUT=out.csv \
--   mame -rompath roms macqd700 -hard disk.hda \
--        -video none -sound none -nothrottle -skip_gameinfo \
--        -seconds_to_run 60 -autoboot_delay 0 \
--        -autoboot_script tools/mame_periph_strobe_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path  = os.getenv("MAME_STROBE_OUT")     or "build/mame_runs/periph_strobe.csv"
local sum_path  = os.getenv("MAME_STROBE_SUMMARY") or (out_path .. ".summary.txt")
local log_all   = (os.getenv("MAME_STROBE_ALL") or "0") ~= "0"
local limit     = tonumber(os.getenv("MAME_STROBE_LIMIT")   or "2000000")
local stop_sec  = tonumber(os.getenv("MAME_STROBE_SECONDS") or "0")
local snap_dir  = os.getenv("MAME_STROBE_SNAPDIR")
local sample1   = tonumber(os.getenv("MAME_STROBE_SAMPLE1") or "5000")

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then os.execute(string.format("mkdir -p %q", dir)) end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_periph_strobe_capture v1 - seq,slot,addr,mask,hot,size,data,pc,cycles\n")
file:write(string.format("# log_all=%s sample1=%d\n", tostring(log_all), sample1))
file:flush()

-- ── Slot decode, transcribed from rtl/soc/peripheral_bus.v decode_slot() ────
-- mac_off = addr[23:0] & ~0xFC0000  (the Q700 addr[23:18] device mirror)
local function slot_of(addr)
    local hi = (addr >> 24) & 0xff
    -- Non-Q700 windows we still want visibility into (DAFB regs / VRAM).
    if hi == 0xf9 then
        if addr >= 0xf9800000 and addr < 0xf9800400 then return "DAFB" end
        return "VRAM_OR_DAFB"
    end
    -- Only the real Q700 I/O aperture.  peripheral_bus.v's decode_slot() also
    -- accepts addr[31:24]==0x00 (its zero-based unit-testbench form), but in
    -- the live MAME system map 0x00xx_xxxx is DRAM, so it must NOT be decoded
    -- as a peripheral here.
    if hi ~= 0x50 then return nil end
    local off_raw = addr & 0xffffff
    local mac_off = off_raw & 0x03ffff
    local nib = (off_raw >> 20) & 0xf
    if nib == 0x8 then return "FAULT_08" end
    if nib == 0x1 then return "FAULT_DMACFG" end
    if nib == 0x9 then return "DBG" end
    if mac_off < 0x002000 then return "VIA1" end
    if mac_off >= 0x002000 and mac_off < 0x004000 then return "VIA2" end
    if mac_off >= 0x008000 and mac_off < 0x008008 then return "ENET" end
    if mac_off >= 0x00a000 and mac_off < 0x00b100 then return "SONIC" end
    if mac_off >= 0x00c000 and mac_off < 0x00e000 then return "SCC" end
    if mac_off >= 0x00e000 and mac_off < 0x00e100 then return "ORWELL" end
    if mac_off >= 0x00f000 and mac_off < 0x00f100 then return "SCSI_REG" end
    if mac_off >= 0x00f100 and mac_off < 0x00f102 then return "SCSI_DMA" end
    if mac_off >= 0x011000 and mac_off < 0x012000 then return "ADBINJ" end
    if mac_off >= 0x014000 and mac_off < 0x016000 then return "ASC" end
    if mac_off >= 0x01e000 and mac_off < 0x020000 then return "IWM" end
    return "FAULT_OTHER"
end

-- ── PC access ───────────────────────────────────────────────────────────────
-- Prefer CURPC (the PC of the instruction performing the access); fall back to
-- PC.  Resolved once at startup so the hot path is a single .value read.
local pc_state = nil
for _, name in ipairs({ "CURPC", "PC" }) do
    local ok, st = pcall(function() return cpu.state[name] end)
    if ok and st then pc_state = st; break end
end
local function read_pc()
    if pc_state then
        local ok, v = pcall(function() return pc_state.value end)
        if ok then return v & 0xffffffff end
    end
    return 0
end

-- ── Device-local register index, transcribed from peripheral_bus.v ─────────
-- Same expressions the RTL uses to address each downstream device, so the
-- coverage report says which REGISTERS of each device the workload actually
-- exercised — a negative multi-hot result is only as strong as the register
-- coverage behind it.
local function reg_of(slot, a)
    if slot == "VIA1" or slot == "VIA2" or slot == "IWM" then
        return (a >> 9) & 0xf                                        -- :1463/:1470/:1617
    elseif slot == "SCSI_REG" then
        return (a >> 4) & 0xf                                        -- :1525-1535
    elseif slot == "SCC" then
        return (((a >> 4) & 0x3) << 2) | (((a >> 1) & 1) << 1) | ((a >> 2) & 1) -- :1514
    elseif slot == "ENET" then
        return a & 0x7                                               -- :1478
    elseif slot == "ORWELL" or slot == "ADBINJ" then
        return a & 0xff                                              -- :1503/:1628
    elseif slot == "SONIC" then
        return a & 0x1fff                                            -- :1490
    elseif slot == "ASC" then
        return a & 0xfff                                             -- :1573-1577
    elseif slot == "SCSI_DMA" then
        return a & 0x1ff                                             -- :1527-1529
    end
    return nil
end

-- ── Counters ────────────────────────────────────────────────────────────────
local regs   = {}      -- regs[slot][reg][hot] = n
local counts = {}      -- counts[slot][hot] = n
local pcs    = {}      -- pcs[slot][hot][pc] = n   (multi-hot only)
local masks  = {}      -- masks[slot][mask] = n
local totals = { writes = 0, multi = 0, rows = 0, seen1 = 0 }

local function bump(tbl, k1, k2)
    local a = tbl[k1]
    if not a then a = {}; tbl[k1] = a end
    a[k2] = (a[k2] or 0) + 1
end

local seq = 0

local function on_write(offset, data, mask)
    local slot = slot_of(offset)
    if not slot then return end

    -- Popcount of hot BYTE lanes; also derive the contiguous span.
    local hot = 0
    local lo, hi = nil, nil
    for b = 0, 3 do
        if ((mask >> ((3 - b) * 8)) & 0xff) ~= 0 then
            hot = hot + 1
            if lo == nil then lo = b end
            hi = b
        end
    end
    if hot == 0 then return end
    local span = hi - lo + 1

    totals.writes = totals.writes + 1
    bump(counts, slot, hot)
    bump(masks, slot, mask)

    local r = reg_of(slot, offset)
    if r then
        local rs = regs[slot]
        if not rs then rs = {}; regs[slot] = rs end
        local e = rs[r]
        if not e then e = {0, 0, 0, 0}; rs[r] = e end
        e[hot] = e[hot] + 1
    end

    if hot > 1 then
        totals.multi = totals.multi + 1
        local pc = read_pc()
        local a = pcs[slot]
        if not a then a = {}; pcs[slot] = a end
        local b = a[hot]
        if not b then b = {}; a[hot] = b end
        b[pc] = (b[pc] or 0) + 1
        if totals.rows < limit and file then
            file:write(string.format("%d,%s,%08x,%08x,%d,%d,%08x,%08x,%d\n",
                seq, slot, offset, mask, hot, span, data, pc,
                math.floor(mach.time.seconds * 1000000)))
            totals.rows = totals.rows + 1
            if totals.rows % 64 == 0 then file:flush() end
        end
    else
        totals.seen1 = totals.seen1 + 1
        local want = log_all or (sample1 > 0 and (totals.seen1 % sample1) == 1)
        if want and totals.rows < limit and file then
            file:write(string.format("%d,%s,%08x,%08x,%d,%d,%08x,%08x,%d\n",
                seq, slot, offset, mask, hot, span, data, read_pc(),
                math.floor(mach.time.seconds * 1000000)))
            totals.rows = totals.rows + 1
        end
    end
    seq = seq + 1
end

-- ONE tap over the whole program space, filtered in the callback.  The
-- DAFB/VRAM aperture is deliberately kept in scope as a POSITIVE CONTROL:
-- QuickDraw hammers it with LONG stores, so an instrument that cannot see a
-- multi-hot mask at all will show zero there too.
local tap = prog:install_write_tap(0x00000000, 0xffffffff, "periph_strobe_w",
    function(offset, data, mask) on_write(offset, data, mask) end)

-- ⚠ THE RETURNED HANDLE MUST BE KEPT REFERENCED.  install_*_tap returns a
-- memory_passthrough_handler; when Lua garbage-collects it, MAME REMOVES the
-- tap and the capture silently truncates — no error, no warning, just a
-- short trace.  Measured on MAME 0.285 / macqd700, identical 8 s ROM boot,
-- counting VIA1 writes through one narrow write tap:
--     handle stored in a global   -> 17,248 writes
--     handle dropped + collectgarbage() -> 0 writes
-- An earlier revision of this file did not store the handle and reported
-- 2,212 VIA1 writes for the same scenario — i.e. an arbitrary prefix,
-- determined by when the incremental collector happened to run.  Tap RANGE
-- width is NOT the issue: narrow (0x5000_0000..0x50FF_FFFF), two-narrow and
-- whole-space taps all return exactly 17,248 when the handle is held.
_G.__periph_strobe_tap = tap

print(string.format("mame_periph_strobe_capture: taps installed -> %s (pc_state=%s)",
    out_path, pc_state and "yes" or "NO"))

local function write_summary(reason)
    local sf = io.open(sum_path, "w")
    if not sf then return end
    sf:write(string.format("# mame_periph_strobe_capture summary (%s)\n", reason))
    sf:write(string.format("# emulated_seconds=%.3f writes=%d multi_hot=%d rows=%d\n",
        mach.time.seconds, totals.writes, totals.multi, totals.rows))
    sf:write("\n## per-slot hot-byte-lane histogram (writes)\n")
    sf:write("slot,hot1,hot2,hot3,hot4,total\n")
    local names = {}
    for k, _ in pairs(counts) do names[#names + 1] = k end
    table.sort(names)
    for _, k in ipairs(names) do
        local c = counts[k]
        local t = (c[1] or 0) + (c[2] or 0) + (c[3] or 0) + (c[4] or 0)
        sf:write(string.format("%s,%d,%d,%d,%d,%d\n", k,
            c[1] or 0, c[2] or 0, c[3] or 0, c[4] or 0, t))
    end
    sf:write("\n## per-slot mem_mask histogram\n")
    for _, k in ipairs(names) do
        local mk = {}
        for m, _ in pairs(masks[k]) do mk[#mk + 1] = m end
        table.sort(mk)
        for _, m in ipairs(mk) do
            sf:write(string.format("%s mask=%08x n=%d\n", k, m, masks[k][m]))
        end
    end
    sf:write("\n## device-register coverage (peripheral_bus.v register index)\n")
    sf:write("slot,reg,hot1,hot2,hot3,hot4\n")
    local rn = {}
    for k, _ in pairs(regs) do rn[#rn + 1] = k end
    table.sort(rn)
    for _, k in ipairs(rn) do
        local rl = {}
        for r, _ in pairs(regs[k]) do rl[#rl + 1] = r end
        table.sort(rl)
        for _, r in ipairs(rl) do
            local e = regs[k][r]
            sf:write(string.format("%s,%d,%d,%d,%d,%d\n", k, r, e[1], e[2], e[3], e[4]))
        end
    end

    sf:write("\n## multi-hot writer PCs (slot, hot, pc, count)\n")
    local pn = {}
    for k, _ in pairs(pcs) do pn[#pn + 1] = k end
    table.sort(pn)
    for _, k in ipairs(pn) do
        for hot = 2, 4 do
            local b = pcs[k][hot]
            if b then
                local pl = {}
                for p, _ in pairs(b) do pl[#pl + 1] = p end
                table.sort(pl)
                for _, p in ipairs(pl) do
                    sf:write(string.format("%s hot=%d pc=%08x n=%d\n", k, hot, p, b[p]))
                end
            end
        end
    end
    sf:close()
end

-- Optional post-boot activity driver (keyboard/mouse/floppy injection) so the
-- capture covers real driver code, not just the ROM boot path.  See
-- tools/mame_periph_activity.lua.
local activity = os.getenv("MAME_STROBE_ACTIVITY")
if activity and activity ~= "" then dofile(activity) end

local frame = 0
local next_snap = 5
emu.register_frame_done(function()
    frame = frame + 1
    if snap_dir and mach.time.seconds >= next_snap then
        next_snap = next_snap + 5
        pcall(function() mach.video:snapshot() end)
    end
    if stop_sec > 0 and mach.time.seconds >= stop_sec then
        write_summary("time limit")
        if file then file:flush(); file:close(); file = nil end
        mach:exit()
    end
end)

emu.register_stop(function()
    write_summary("stop")
    if file then file:flush(); file:close(); file = nil end
    print(string.format("mame_periph_strobe_capture: stopped, writes=%d multi_hot=%d",
        totals.writes, totals.multi))
end)
