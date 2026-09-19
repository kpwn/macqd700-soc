-- mame_axi_capture.lua  —  Wide CPU-side memory tap for macqd700.
--
-- Captures every CPU-bus access the Q700 ROM makes to anything OTHER
-- than DDR (0x0000_0000..0x3FFF_FFFF) and ROM (0x4000_0000..0x40FF_FFFF).
-- The result is a byte-for-byte reference for AXI lockstep against the
-- RTL-side capture in tb/tb_fpga_top_rom.cpp (see tools/axi_lockstep_diff.py
-- and docs/axi_lockstep.md).
--
-- The tap is installed on `:maincpu.spaces["program"]` via
-- install_read_tap / install_write_tap (verified working in MAME 0.264 by
-- tools/mame_iwm_capture.lua, which uses the same API).  We ride the same
-- boot scenario as that tool: macqd700 with no floppy attached.  The
-- standard "mame-fastdiag,chime-skip" patch stack matches the RTL sim's
-- default to maximise overlap inside the divergence window.
--
-- Output format (one event per line):
--     <seq>,<R|W>,<addr_hex>,<size_bytes>,<data_hex>
--
-- where:
--     <seq>          monotonically increasing 0-based event index
--     <R|W>          direction
--     <addr_hex>     absolute byte address (no '0x' prefix, hex digits only)
--     <size_bytes>   1 / 2 / 4 (BYTE / WORD / LONG) — derived from the mask
--     <data_hex>     access value, hex, no '0x' prefix
--
-- Filter:
--     accept only addresses ≥ 0x5000_0000 OR addresses in the F9xx_xxxx
--     VRAM/DAFB region.  Concretely we drop:
--         0x0000_0000 .. 0x3FFF_FFFF  — RAM/DDR
--         0x4000_0000 .. 0x40FF_FFFF  — ROM
--     Everything else (peripherals, VRAM, DAFB) is in scope.  Mirrors the
--     RTL filter in tb_fpga_top_rom.cpp +axi_lockstep_log= driver.
--
-- ROM-patch handling:
--   Optional patches read from $MAME_AXI_PATCH_FILE are applied to MAME's
--   in-memory `:bootrom` region — same approach as tools/mame_iwm_capture.lua.
--   The wrapping make target (tb-axi-lockstep) writes the canonical
--   "mame-fastdiag,chime-skip" patch list there before launching MAME.
--
-- Usage:
--     MAME_AXI_TRACE_OUT=build/axi_lockstep/axi_mame.csv \
--     MAME_AXI_TRACE_LIMIT=5000 \
--     MAME_AXI_TRACE_SECONDS=2 \
--     mame -rompath roms macqd700 -window -resolution0 320x240 \
--          -nothrottle -seconds_to_run 2 -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_axi_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path  = os.getenv("MAME_AXI_TRACE_OUT")     or "build/mame_runs/axi_q700.csv"
local limit     = tonumber(os.getenv("MAME_AXI_TRACE_LIMIT")   or "5000")
local stop_sec  = tonumber(os.getenv("MAME_AXI_TRACE_SECONDS") or "0")
local patch_path = os.getenv("MAME_AXI_PATCH_FILE")

-- Apply :bootrom patches in-memory (NOT on-disk; checksum still valid).
local rom = mach.memory.regions[":bootrom"]
local patch_count = 0
if patch_path and rom then
    local pf = io.open(patch_path, "r")
    if pf then
        for line in pf:lines() do
            local off, val = line:match("^([0-9a-fA-Fx]+)%s+([0-9a-fA-Fx]+)")
            if off and val then
                rom:write_u8(tonumber(off), tonumber(val))
                patch_count = patch_count + 1
            end
        end
        pf:close()
    end
end

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then
        os.execute(string.format("mkdir -p %q", dir))
    end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_axi_capture v1 — seq,rw,addr,size,data\n")
file:write(string.format("# patches=%d patch_file=%s\n",
                          patch_count, patch_path or "(none)"))
file:flush()

local seq = 0

-- Filter: include 0x5000_0000..0xFFFF_FFFF (peripherals + VRAM + DAFB +
-- everything else), drop 0x0000_0000..0x3FFF_FFFF (DDR) and
-- 0x4000_0000..0x40FF_FFFF (ROM).  Anything in the 0x4100_0000..0x4FFF_FFFF
-- window is technically a ROM mirror per axi_defs.vh, but MAME's macqd700
-- map only places ROM in 0x40800000..0x408FFFFF; mirrors above 0x41000000
-- are not driven in MAME's view.  Be defensive: drop the entire 0x40-0x4F
-- range so any ROM mirror traffic doesn't pollute the CSV.
local function in_scope(addr)
    if addr < 0x40000000 then return false end  -- DDR
    if addr < 0x50000000 then return false end  -- ROM + ROM-mirror window
    return true
end

-- Decode a (offset, mask) pair into a list of {addr, size, data} byte/half/
-- word/longword sub-accesses.  MAME's program-space tap delivers handler-
-- granular accesses; the macqd700 dbus is 32-bit so a typical mask is
-- 0xFF000000 (high byte), 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFFFF0000,
-- 0x0000FFFF, or 0xFFFFFFFF.  We coalesce contiguous mask runs into a single
-- access whose <size> matches what the CPU actually emitted.
local function decode_access(offset, data, mask)
    local results = {}
    -- Find contiguous runs of high bytes in mask, MSB-first to preserve
    -- big-endian order on the m68k bus.  In a 32-bit handler offset is
    -- the byte address of bit [31:24] (the high byte).
    local m = {}
    for byte = 0, 3 do
        m[byte] = ((mask >> ((3 - byte) * 8)) & 0xff) ~= 0
    end
    local i = 0
    while i < 4 do
        if not m[i] then
            i = i + 1
        else
            local j = i
            while j < 4 and m[j] do j = j + 1 end
            local run = j - i
            -- Snap to architectural sizes: 1, 2, 4 bytes only.
            -- A run of 3 is unusual; split as 2 + 1.
            local sizes = {}
            if run == 1 then sizes = {1}
            elseif run == 2 then sizes = {2}
            elseif run == 3 then sizes = {2, 1}
            elseif run == 4 then sizes = {4}
            end
            local pos = i
            for _, sz in ipairs(sizes) do
                local addr = (offset & ~3) + pos
                local val = 0
                for k = 0, sz - 1 do
                    -- Byte (pos+k) of the 32-bit word — bit position is
                    -- (3 - (pos+k)) * 8.
                    local shift = (3 - (pos + k)) * 8
                    local b = (data >> shift) & 0xff
                    val = (val << 8) | b
                end
                results[#results + 1] = {addr = addr, size = sz, data = val}
                pos = pos + sz
            end
            i = j
        end
    end
    return results
end

local function on_event(rw, offset, data, mask)
    if seq >= limit then return end
    if not in_scope(offset) then return end
    local accesses = decode_access(offset, data, mask)
    for _, a in ipairs(accesses) do
        if seq >= limit then break end
        local fmt
        if a.size == 1 then     fmt = "%d,%s,%08x,%d,%02x\n"
        elseif a.size == 2 then fmt = "%d,%s,%08x,%d,%04x\n"
        else                    fmt = "%d,%s,%08x,%d,%08x\n"
        end
        file:write(string.format(fmt, seq, rw, a.addr, a.size, a.data))
        seq = seq + 1
    end
    if seq >= limit then
        file:flush()
        print(string.format("mame_axi_capture: hit limit=%d, exiting", limit))
        if file then file:close(); file = nil end
        mach:exit()
    end
end

-- Wide tap across the full 0x4000_0000..0xFFFF_FFFF upper half so we catch
-- ROM, IO, VRAM, DAFB and any oddball NuBus probes.  in_scope() filters out
-- ROM and DDR.  We can't tap below 0x40000000 because MAME may install
-- handlers at finer granularity; the 0x4000_0000 lower bound covers every
-- ROM mirror plus all peripheral / VRAM / DAFB regions of the macqd700 map.
-- ⚠ THE TAP HANDLE IS NOT RETAINED.  install_*_tap returns a
-- memory_passthrough_handler; if Lua garbage-collects it, MAME REMOVES the tap
-- and the capture silently truncates — no error, just a short CSV, so an
-- AXI-lockstep diff can "pass" on a prefix.  Measured on MAME 0.285/macqd700,
-- identical 8 s ROM boot, one narrow write tap: handle held -> 17,248 VIA1
-- writes; handle dropped + collectgarbage() -> 0.  Assign the return values
-- below to a local/global before trusting any golden trace produced here.
-- See docs/mame_periph_multihot_reachability.md §"Instrument defects found".
prog:install_read_tap(0x40000000, 0xffffffff, "axi_capture_r",
    function(offset, data, mask)
        on_event("R", offset, data, mask)
    end)

prog:install_write_tap(0x40000000, 0xffffffff, "axi_capture_w",
    function(offset, data, mask)
        on_event("W", offset, data, mask)
    end)

print(string.format("mame_axi_capture: tap installed (limit=%d, patches=%d) -> %s",
                    limit, patch_count, out_path))

local frame = 0
emu.register_frame_done(function()
    frame = frame + 1
    if stop_sec > 0 and frame >= stop_sec * 60 then
        print(string.format("mame_axi_capture: %d s elapsed, %d events", stop_sec, seq))
        if file then file:flush(); file:close(); file = nil end
        mach:exit()
    end
end)

emu.register_stop(function()
    if file then
        file:flush()
        file:close()
        file = nil
    end
    print(string.format("mame_axi_capture: stopped, %d events captured", seq))
end)
