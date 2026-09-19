-- mame_iwm_capture.lua  —  IWM/SWIM register read/write tap for macqd700.
--
-- Captures the bit-exact byte sequence the Q700 ROM exchanges with the
-- on-board SWIM (swim1) during cold boot.  Output is a CSV trace that
-- can be replayed against rtl/mac/iwm_stub.v in a Verilator unit
-- testbench (tools/iwm_lockstep_diff.py + tb/tb_iwm_stub.cpp).
--
-- Output format (one event per line):
--     <sim_time_ns>,<R|W>,<reg_hex>,<byte_hex>
--
-- where:
--     <sim_time_ns>   absolute MAME time in nanoseconds (integer)
--     <R|W>           direction
--     <reg_hex>       SWIM register selector 0..F  (= ((addr>>1)>>8) & 0xF)
--     <byte_hex>      byte exchanged (high byte of the u16 lane)
--
-- Address-map note (per src/mame/apple/macquadra700.cpp + swim1.cpp):
--   * macqd700 maps the SWIM at 0x5001_e000..0x5001_ffff with mirror
--     0x00fc_0000.  The Q700 ROM uses the bitwise-OR family of mirrors
--     (e.g. 0x50f1_xxxx, 0x50f5_xxxx, ...).
--   * The handler is u16 wide.  MAME passes the BYTE offset to the
--     state-method handler which then computes (offset >> 8) & 0xF as
--     the register selector — but with a 16-bit handler installed on a
--     32-bit dbus, the byte offset MAME hands the lua tap maps the
--     register selector as: reg = (byte_offset_within_aperture >> 9) & 0xF
--     (verified empirically with the IWM-to-ISM 57,17,57,57 sequence
--     landing on byte offset 0xfe00 == reg 0xf, matching swim1.cpp's
--     `if(offset == 0xf)`).
--   * The byte rides the upper half of the u16 lane (the Q700 driver
--     does `result << 8` on read and `data >> 8` on write).  In the
--     32-bit dbus that translates to the high byte of the high word
--     (mask = 0xff00_0000).
--
-- ROM-patch handling:
--   * If $MAME_IWM_PATCH_FILE is set, the script reads "<off> <val>"
--     pairs (decimal or 0x-hex) and writes them into the :bootrom
--     region at autoboot time, same approach as
--     tools/mame_scc_capture.lua.  This lets us match the
--     rom_patch_sets.h "mame-fastdiag,chime-skip" patch stack our
--     RTL sim uses, without touching the on-disk ROM (MAME's checksum
--     check still passes).
--   * NOTE: the standard "mame-fastdiag" stack actually skips the ROM's
--     floppy probe entirely (we never see SWIM accesses with it
--     applied).  Use the unpatched ROM (or a narrower patch set such
--     as just "chime-skip") if you want the full IWM/SWIM init
--     sequence.
--
-- Usage:
--     MAME_IWM_TRACE_OUT=build/mame_runs/iwm_q700_nodisk.csv \
--     MAME_IWM_TRACE_LIMIT=512 \
--     mame -rompath roms macqd700 -window -resolution0 320x240 \
--          -nothrottle -seconds_to_run 6 -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_iwm_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path = os.getenv("MAME_IWM_TRACE_OUT") or "build/mame_runs/iwm_q700.csv"
local limit    = tonumber(os.getenv("MAME_IWM_TRACE_LIMIT") or "512")
local stop_sec = tonumber(os.getenv("MAME_IWM_TRACE_SECONDS") or "0")
local patch_path = os.getenv("MAME_IWM_PATCH_FILE")

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
file:write("# mame_iwm_capture v2 — ns,rw,reg,byte\n")
file:write(string.format("# patches=%d patch_file=%s\n",
                          patch_count, patch_path or "(none)"))
file:flush()

local count = 0

local function ns_now()
    return math.floor(mach.time:as_double() * 1e9)
end

-- Wide tap across the 0x5000_0000..0x50FF_FFFF I/O window (catches every
-- mirror-bit combination of base 0x5001e000 + mask 0x00fc0000) and
-- filter inside the callback to the SWIM aperture (sub-address bits
-- 0x1e000..0x1ffff).  This catches every mirror variant the ROM picks.

local function on_event(rw, offset, data, mask)
    if count >= limit then return end
    local sub = offset & 0x000fffff
    if sub < 0x1e000 or sub > 0x1ffff then return end
    -- Walk every byte lane in the mask (typically only 0xff000000 is set
    -- because the SWIM byte rides the high byte of the high word).
    for lane = 0, 3 do
        local mbyte = (mask >> (lane * 8)) & 0xff
        if mbyte ~= 0 then
            -- byte_addr is the absolute address of the masked byte.  In
            -- big-endian Q700 memory layout, lane 3 (mask 0xff000000)
            -- is the byte at the lowest address (offset & ~3) + 0; lane 0
            -- (mask 0x000000ff) is at + 3.  The 16-bit register-selection
            -- divides the byte address by 2 then takes (>>8)&0xf.
            local byte_addr = (offset & ~3) + (3 - lane)
            local sub_byte  = byte_addr & 0x000fffff
            local reg = ((sub_byte >> 1) >> 8) & 0xf
            local b = (data >> (lane * 8)) & 0xff
            file:write(string.format("%d,%s,%x,%02x\n", ns_now(), rw, reg, b))
            file:flush()
            count = count + 1
            if count >= limit then
                print(string.format("mame_iwm_capture: hit limit=%d, exiting", limit))
                file:close()
                file = nil
                mach:exit()
                return
            end
        end
    end
end

-- ⚠ THE TAP HANDLE IS NOT RETAINED.  install_*_tap returns a
-- memory_passthrough_handler; if Lua garbage-collects it, MAME REMOVES the tap
-- and the trace silently truncates — no error, just a short file.  Measured on
-- MAME 0.285/macqd700, identical 8 s ROM boot, one narrow write tap: handle
-- held -> 17,248 VIA1 writes; handle dropped + collectgarbage() -> 0.  This
-- specifically weakens tb/vectors/swim_mame_q700_753.csv as evidence of
-- byte-only IWM access: its 869 events may be a truncated prefix.  A dedicated
-- re-measurement (handle retained, no lane `break`) found 0 multi-hot writes in
-- 2,534 IWM writes across System 7.0.1 and 7.5.3 — see
-- docs/mame_periph_multihot_reachability.md.  Assign the return values below to
-- a local/global before trusting any golden trace produced here.
prog:install_read_tap(0x50000000, 0x50ffffff, "iwm_capture_r",
    function(offset, data, mask)
        on_event("R", offset, data, mask)
    end)

prog:install_write_tap(0x50000000, 0x50ffffff, "iwm_capture_w",
    function(offset, data, mask)
        on_event("W", offset, data, mask)
    end)

print(string.format("mame_iwm_capture: tap installed (limit=%d, patches=%d) -> %s",
                    limit, patch_count, out_path))

local frame = 0
emu.register_frame_done(function()
    frame = frame + 1
    if stop_sec > 0 and frame >= stop_sec * 60 then
        print(string.format("mame_iwm_capture: %d s elapsed, %d events", stop_sec, count))
        if file then file:close(); file = nil end
        mach:exit()
    end
end)

emu.register_stop(function()
    if file then
        file:flush()
        file:close()
        file = nil
    end
    print(string.format("mame_iwm_capture: stopped, %d events captured", count))
end)
