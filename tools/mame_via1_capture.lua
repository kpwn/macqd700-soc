-- mame_via1_capture.lua — VIA1 register read/write tap for macqd700.
--
-- Captures the bit-exact byte sequence the Q700 ROM exchanges with VIA1
-- during cold boot.  Output is a CSV trace that can be replayed against
-- our RTL via1.v in tb/tb_fpga_top_rom.cpp (peripheral_bus VIA1 master
-- face) and byte-diffed in tools/via1_lockstep_diff.py.
--
-- Output format (one event per line):
--     <seq>,<R|W>,<reg_hex>,<byte_hex>
--
-- where:
--     <seq>           monotonically increasing 0-based event index
--     <R|W>           direction
--     <reg_hex>       VIA1 register selector 0..F  (= ((byte_off & ~mirror) >> 9) & 0xF)
--     <byte_hex>      byte exchanged (high byte of the u16 lane)
--
-- Address-map note (per src/mame/apple/macquadra700.cpp):
--   * macqd700 maps VIA1 at 0x5000_0000..0x5000_1fff with mirror
--     0x00fc_0000.  The Q700 ROM uses the bitwise-OR family of mirrors
--     (e.g. 0x50f0_xxxx, 0x50f4_xxxx, ...).
--   * The handler is u16 wide (rw via_r / via_w returning u16).  MAME passes
--     a WORD offset to the handler; via_r does `offset >>= 8; offset &= 0x0f`.
--     For the byte offset we receive in the lua tap that translates to
--     reg = (byte_offset_within_aperture >> 9) & 0xF.
--   * The byte rides the upper half of the u16 lane (the Q700 driver does
--     `(data & 0xff) | (data << 8)` on read and gates writes by 8/16 bit
--     mask).  In the 32-bit dbus that translates to either the high byte
--     of the high word (mask 0xff00_0000) or the high byte of the low word
--     (mask 0x0000_ff00) depending on which half-word the CPU touches.
--
-- ROM-patch handling:
--   * If $MAME_VIA1_PATCH_FILE is set, the script reads "<off> <val>"
--     pairs (decimal or 0x-hex) and writes them into the :bootrom region
--     at autoboot time.  Same pattern as mame_iwm_capture.lua.
--
-- Usage:
--     MAME_VIA1_TRACE_OUT=build/via1_lockstep/mame_via1.csv \
--     MAME_VIA1_TRACE_LIMIT=4096 \
--     MAME_VIA1_TRACE_SECONDS=4 \
--     mame -rompath roms macqd700 -window -resolution0 320x240 \
--          -nothrottle -seconds_to_run 4 -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_via1_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

-- Current instruction PC, for the informational 5th CSV column.  The
-- via1_lockstep_diff.py only reads columns 0..3, so this never affects
-- the diff — it just lets a control-flow fork be attributed to a PC.
local function cpu_pc()
    local ok, st = pcall(function() return cpu.state end)
    if not ok or not st then return 0 end
    local r = st["CURPC"] or st["PC"]
    if r then return r.value & 0xffffffff end
    return 0
end

local out_path  = os.getenv("MAME_VIA1_TRACE_OUT")    or "build/mame_runs/via1_q700.csv"
local limit     = tonumber(os.getenv("MAME_VIA1_TRACE_LIMIT")   or "4096")
local stop_sec  = tonumber(os.getenv("MAME_VIA1_TRACE_SECONDS") or "0")
local patch_path = os.getenv("MAME_VIA1_PATCH_FILE")

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
file:write("# mame_via1_capture v1 — seq,rw,reg,byte\n")
file:write(string.format("# patches=%d patch_file=%s\n",
                          patch_count, patch_path or "(none)"))
file:flush()

local seq = 0

-- Wide tap across the 0x5000_0000..0x50FF_FFFF I/O window so we catch
-- every mirror-bit combination the ROM picks of base 0x5000_0000 + mask
-- 0x00fc_0000.  Filter inside the callback to the VIA1 aperture
-- (sub-address bits 0x0000..0x1FFF after stripping the mirror mask).
local Q700_IO_MIRROR_MASK = 0x00fc0000

local function on_event(rw, offset, data, mask)
    if seq >= limit then return end
    -- Strip the Q700 mirror mask, then check sub-window.
    local sub = offset & 0x000fffff
    local mac_off = sub & ~Q700_IO_MIRROR_MASK
    if mac_off < 0x0000 or mac_off > 0x1fff then return end
    -- Walk every byte lane in the mask.  In MAME's u16 handler attached to
    -- a 32-bit dbus, the byte rides EITHER the high byte of the high word
    -- (mask 0xff00_0000, lane 3) OR the high byte of the low word
    -- (mask 0x0000_ff00, lane 1) depending on whether the CPU touched the
    -- upper or lower half of the 32-bit word.  We treat both as the same
    -- VIA register — the ROM's via_w/via_r handler ORs the lanes.
    for lane = 0, 3 do
        local mbyte = (mask >> (lane * 8)) & 0xff
        if mbyte ~= 0 then
            -- Compute the register selector from the masked byte's
            -- byte address inside the VIA1 window.
            local byte_addr = (offset & ~3) + (3 - lane)
            local sub_byte  = byte_addr & 0x000fffff
            local mac_byte_off = sub_byte & ~Q700_IO_MIRROR_MASK
            if mac_byte_off >= 0x0000 and mac_byte_off <= 0x1fff then
                -- Reg index from byte offset: (>>9)&0xF — matches our
                -- peripheral_bus.v VIA1 decode (addr[12:9]) and MAME's
                -- via_r `offset >>= 8 & 0x0f` once we account for the
                -- u16 word-offset shift.
                local reg = (mac_byte_off >> 9) & 0xf
                local b = (data >> (lane * 8)) & 0xff
                -- Only emit lanes that actually carry the VIA byte.
                -- MAME's via_w gates with ACCESSING_BITS_0_7 / 8_15.
                -- The 68040 always issues byte-wide MMIO accesses to
                -- VIA, so the high or the low half of the 16-bit lane
                -- is the byte we want.  Filter out the "other" lane in
                -- a 16-bit u16 access where both bytes are the same
                -- (since the read returns `data | (data << 8)`).
                -- Concretely: emit the FIRST lane with a non-zero mbyte
                -- per architectural access (read or write) — duplicates
                -- in the same offset/mask call would appear as two
                -- events for one architectural CPU access.
                file:write(string.format("%d,%s,%x,%02x,%08x\n",
                                          seq, rw, reg, b, cpu_pc()))
                file:flush()
                seq = seq + 1
                if seq >= limit then
                    print(string.format("mame_via1_capture: hit limit=%d, exiting",
                                          limit))
                    if file then file:close(); file = nil end
                    mach:exit()
                    return
                end
                -- The CPU touches one byte per architectural access;
                -- emit only the first lane of any combined mask (e.g.
                -- u16 read where MAME duplicates the byte across two
                -- lanes).  Break after the first match to keep one
                -- event per CPU access.
                break
            end
        end
    end
end

-- ⚠ TWO KNOWN DEFECTS IN THIS TOOL — see docs/mame_periph_multihot_reachability.md.
--
--  1. STRUCTURALLY BLIND TO MULTI-LANE ACCESSES.  The lane loop above `break`s
--     after the first hot byte lane, on the assumption that "the 68040 always
--     issues byte-wide MMIO accesses to VIA".  That assumption is baked into
--     the instrument, so this trace can never falsify it.  It is measurably
--     TRUE for VIA1 (0 multi-hot writes in ~350k captured VIA1 writes across
--     System 7.0.1 and 7.5.3), but it is measurably FALSE for other slots in
--     this same aperture (ORWELL, SONIC, ASC, SCSI-DMA).  Do not reuse this
--     lane loop for any other device without removing the `break`.
--
--  2. THE TAP HANDLE IS NOT RETAINED.  install_*_tap returns a
--     memory_passthrough_handler; if Lua garbage-collects it, MAME REMOVES the
--     tap and the trace silently truncates.  Measured on MAME 0.285/macqd700,
--     identical 8 s ROM boot: handle held -> 17,248 VIA1 writes; handle dropped
--     + collectgarbage() -> 0.  Assign the return values to a module-level
--     local or a global before trusting any golden trace produced here.
--     (Same defect in mame_axi_capture.lua, mame_iwm_capture.lua,
--     mame_asc_capture.lua, mame_dafb_capture.lua.)
prog:install_read_tap(0x50000000, 0x50ffffff, "via1_capture_r",
    function(offset, data, mask)
        on_event("R", offset, data, mask)
    end)

prog:install_write_tap(0x50000000, 0x50ffffff, "via1_capture_w",
    function(offset, data, mask)
        on_event("W", offset, data, mask)
    end)

print(string.format("mame_via1_capture: tap installed (limit=%d, patches=%d) -> %s",
                    limit, patch_count, out_path))

local frame = 0
emu.register_frame_done(function()
    frame = frame + 1
    if stop_sec > 0 and frame >= stop_sec * 60 then
        print(string.format("mame_via1_capture: %d s elapsed, %d events",
                              stop_sec, seq))
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
    print(string.format("mame_via1_capture: stopped, %d events captured", seq))
end)
