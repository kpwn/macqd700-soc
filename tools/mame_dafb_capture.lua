-- mame_dafb_capture.lua  —  DAFB register read/write tap for macqd700.
--
-- Captures the bit-exact byte sequence the Q700 ROM exchanges with the
-- DAFB device during cold boot.  Output is a CSV trace that can be
-- replayed against rtl/mac/video.v in tb_fpga_top_rom (via the
-- +dafb_lockstep_log= flag) and diffed by tools/dafb_lockstep_diff.py.
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
-- Address windows tapped (per src/mame/apple/macquadra700.cpp + dafb.cpp):
--   * 0xF980_0000..0xF980_03FF  DAFB register file (dafb_r/w + swatch_r/w +
--                                ramdac_r/w + clockgen_r/w sub-blocks)
--   * 0x5000_F000..0x5000_F0FF  TurboSCSI register window (mirror 0xfc0000)
--   * 0x5000_F100..0x5000_F101  TurboSCSI DMA handshake (select 0xfc0000)
--   * 0xF900_0000..0xF91F_FFFF  VRAM aperture (CPU writes through DAFB)
--
-- Implementation notes (quirks bisected against MAME 0.264 / macqd700):
-- 1. **One tap pair, wide range.**  Two non-overlapping
--    `install_read_tap` / `install_write_tap` pairs on the same
--    address space cause MAME to silently noop the second pair
--    (only the first set fires).  We mirror the AXI-capture
--    workaround (one tap from 0x4000_0000..0xFFFF_FFFF) and filter
--    inside the callback.
-- 2. **No `emu.register_frame_done` callback alongside a wide tap.**
--    Registering a frame-done callback while a wide-range tap is
--    installed deactivates the tap callbacks entirely (tap installs
--    succeed, but events never deliver).  Use `-seconds_to_run` on
--    the MAME command line instead — the wrapping `make tb-dafb-
--    lockstep` target already passes it.
-- 3. **No function dispatch out of the tap callback.**  Calling
--    even a `local function` upvalue from inside the tap body
--    causes the tap to stop firing under wide-range GC pressure.
--    Inline ALL filter / decode / emit logic.
--
-- ROM-patch handling: optional patches read from $MAME_DAFB_PATCH_FILE
-- are applied to MAME's in-memory `:bootrom` region — same pattern as
-- tools/mame_axi_capture.lua and tools/mame_iwm_capture.lua.  The
-- on-disk ROM is untouched so MAME's checksum still passes.  Default
-- patch set is empty — the standard `mame-fastdiag` stack jumps past
-- the DAFB init, so the unpatched ROM is the one that exercises it.
--
-- Usage:
--     MAME_DAFB_TRACE_OUT=build/dafb_lockstep/dafb_mame.csv \
--     MAME_DAFB_TRACE_LIMIT=8000 \
--     mame -rompath roms macqd700 -window -resolution0 320x240 \
--          -nothrottle -seconds_to_run 15 -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_dafb_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path  = os.getenv("MAME_DAFB_TRACE_OUT")     or "build/mame_runs/dafb_q700.csv"
local limit     = tonumber(os.getenv("MAME_DAFB_TRACE_LIMIT")   or "8000")
local include_vram = (os.getenv("MAME_DAFB_INCLUDE_VRAM") or "0") ~= "0"
local patch_path = os.getenv("MAME_DAFB_PATCH_FILE")

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

local file = assert(io.open(out_path, "w"))
file:write("# mame_dafb_capture v1 — seq,rw,addr,size,data\n")
file:write(string.format("# patches=%d patch_file=%s include_vram=%s\n",
                          patch_count, patch_path or "(none)",
                          include_vram and "1" or "0"))
file:flush()

local seq = 0

-- Read tap: inline filter + emit.  See header for why no function
-- dispatch out of this body.
prog:install_read_tap(0x40000000, 0xffffffff, "dafb_capture_r",
    function(offset, data, mask)
        if seq >= limit then return end
        local b3 = ((mask >> 24) & 0xff) ~= 0
        local b2 = ((mask >> 16) & 0xff) ~= 0
        local b1 = ((mask >>  8) & 0xff) ~= 0
        local b0 = ( mask        & 0xff) ~= 0
        local base = offset & ~3
        local sz, pos = 0, 0
        if b3 and b2 and b1 and b0 then
            sz, pos = 4, 0
        elseif b3 and b2 and not b1 and not b0 then
            sz, pos = 2, 0
        elseif not b3 and not b2 and b1 and b0 then
            sz, pos = 2, 2
        elseif b3 and not b2 and not b1 and not b0 then
            sz, pos = 1, 0
        elseif not b3 and b2 and not b1 and not b0 then
            sz, pos = 1, 1
        elseif not b3 and not b2 and b1 and not b0 then
            sz, pos = 1, 2
        elseif not b3 and not b2 and not b1 and b0 then
            sz, pos = 1, 3
        else
            return
        end
        local addr = base + pos
        local in_scope = false
        if addr >= 0xf9800000 and addr <= 0xf98003ff then
            in_scope = true
        elseif include_vram and addr >= 0xf9000000 and addr <= 0xf91fffff then
            in_scope = true
        elseif addr >= 0x50000000 and addr <= 0x50ffffff then
            local sub = addr & 0x000fffff
            if sub >= 0x0f000 and sub <= 0x0f0ff then in_scope = true
            elseif sub >= 0x0f100 and sub <= 0x0f101 then in_scope = true
            end
        end
        if not in_scope then return end
        local val = 0
        local k = 0
        while k < sz do
            local shift = (3 - (pos + k)) * 8
            local b = (data >> shift) & 0xff
            val = (val << 8) | b
            k = k + 1
        end
        if sz == 1 then
            file:write(string.format("%d,R,%08x,1,%02x\n", seq, addr, val))
        elseif sz == 2 then
            file:write(string.format("%d,R,%08x,2,%04x\n", seq, addr, val))
        else
            file:write(string.format("%d,R,%08x,4,%08x\n", seq, addr, val))
        end
        file:flush()
        seq = seq + 1
    end)

prog:install_write_tap(0x40000000, 0xffffffff, "dafb_capture_w",
    function(offset, data, mask)
        if seq >= limit then return end
        local b3 = ((mask >> 24) & 0xff) ~= 0
        local b2 = ((mask >> 16) & 0xff) ~= 0
        local b1 = ((mask >>  8) & 0xff) ~= 0
        local b0 = ( mask        & 0xff) ~= 0
        local base = offset & ~3
        local sz, pos = 0, 0
        if b3 and b2 and b1 and b0 then
            sz, pos = 4, 0
        elseif b3 and b2 and not b1 and not b0 then
            sz, pos = 2, 0
        elseif not b3 and not b2 and b1 and b0 then
            sz, pos = 2, 2
        elseif b3 and not b2 and not b1 and not b0 then
            sz, pos = 1, 0
        elseif not b3 and b2 and not b1 and not b0 then
            sz, pos = 1, 1
        elseif not b3 and not b2 and b1 and not b0 then
            sz, pos = 1, 2
        elseif not b3 and not b2 and not b1 and b0 then
            sz, pos = 1, 3
        else
            return
        end
        local addr = base + pos
        local in_scope = false
        if addr >= 0xf9800000 and addr <= 0xf98003ff then
            in_scope = true
        elseif include_vram and addr >= 0xf9000000 and addr <= 0xf91fffff then
            in_scope = true
        elseif addr >= 0x50000000 and addr <= 0x50ffffff then
            local sub = addr & 0x000fffff
            if sub >= 0x0f000 and sub <= 0x0f0ff then in_scope = true
            elseif sub >= 0x0f100 and sub <= 0x0f101 then in_scope = true
            end
        end
        if not in_scope then return end
        local val = 0
        local k = 0
        while k < sz do
            local shift = (3 - (pos + k)) * 8
            local b = (data >> shift) & 0xff
            val = (val << 8) | b
            k = k + 1
        end
        if sz == 1 then
            file:write(string.format("%d,W,%08x,1,%02x\n", seq, addr, val))
        elseif sz == 2 then
            file:write(string.format("%d,W,%08x,2,%04x\n", seq, addr, val))
        else
            file:write(string.format("%d,W,%08x,4,%08x\n", seq, addr, val))
        end
        file:flush()
        seq = seq + 1
    end)

print(string.format("mame_dafb_capture: tap installed (limit=%d, patches=%d, vram=%s) -> %s",
                    limit, patch_count, include_vram and "1" or "0", out_path))
print("mame_dafb_capture: stop via MAME -seconds_to_run; no frame_done hook (tap quirk).")
