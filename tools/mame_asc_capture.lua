-- mame_asc_capture.lua — ASC register read/write tap for macqd700.
--
-- Captures the bit-exact byte sequence the Q700 ROM exchanges with
-- the Apple Sound Chip (asc_easc_device) during cold boot, with
-- per-frame-resolution timestamps so an offline replay tool can
-- drive a Verilator-compiled rtl/mac/asc.v through the same bus
-- traffic and emit a WAV.
--
-- Output format (one event per line, TSV, first non-comment line):
--     <seq>\t<time_ns>\t<R|W>\t<reg_off_hex>\t<byte_hex>
--
-- where:
--     <seq>          monotonically increasing 0-based event index
--     <time_ns>      machine_time at the most recent frame_done (ns)
--     <R|W>          direction
--     <reg_off_hex>  byte offset within the 4 KB ASC window (0x000..0xfff)
--     <byte_hex>     byte exchanged on the live data lane
--
-- Address-map note (per src/mame/apple/macquadra700.cpp:567,589):
--   * macqd700 maps ASC at 0x50014000..0x50015fff with mirror 0x00fc0000.
--   * MAME's CPU dispatcher routes byte accesses to the matching lane;
--     install_*_tap reports `mask` indicating which byte lane is active.
--   * Reg offset = (mac_off - 0x14000) & 0xfff after stripping the mirror
--     (the ASC has internal 4 KB window; software-side aliases beyond
--     the canonical 0x14000..0x15fff are folded onto 0x000..0xfff).
--
-- Two known-pitfalls that bit during bring-up:
--   1. Calling `mach.time:as_double()` from inside the tap callback is
--      ~100× slower than caching it once per frame.  Cache via a
--      `register_frame_done` callback so the per-event tap stays cheap.
--   2. Running MAME with `-wavwrite` AND a tap installed reliably
--      crashes MAME with SIGSEGV after a few hundred ms of emu time
--      (likely a re-entrancy issue between SDL's sound mixer and the
--      Lua engine grabbing emu state from a tap context).  We therefore
--      capture with `-sound none` only — audio comes from feeding our
--      Verilated `Vasc` the captured bus trace via tools/asc_replay_trace.
--
-- Usage:
--     MAME_ASC_TRACE_OUT=/tmp/mame_asc_trace.tsv \
--     MAME_ASC_TRACE_LIMIT=200000 \
--     mame -rompath roms macqd700 -seconds_to_run 1 -nothrottle \
--          -video none -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_asc_capture.lua \
--          -plugins

local mach = manager.machine

local out_path = os.getenv("MAME_ASC_TRACE_OUT") or "/tmp/mame_asc_trace.tsv"
local limit    = tonumber(os.getenv("MAME_ASC_TRACE_LIMIT") or "200000")

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then
        os.execute(string.format("mkdir -p %q", dir))
    end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_asc_capture v2 — seq\\ttime_ns\\trw\\treg_off\\tbyte\n")
file:flush()

local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local Q700_MIRR_MASK = 0x00fc0000
local cur_time_ns    = 0
local seq            = 0

emu.register_frame_done(function()
    cur_time_ns = math.floor(mach.time:as_double() * 1e9)
end)

local function on_event(rw, offset, data, mask)
    if seq >= limit then return end
    -- Strip the Q700 mirror mask, then check sub-window.
    local sub        = offset & 0x000fffff
    local mac_off    = sub & ~Q700_MIRR_MASK
    if mac_off < 0x14000 or mac_off > 0x15fff then return end
    -- Byte lane(s).  MAME emits one tap call per architectural CPU
    -- access; the byte lives in whichever lane has a non-zero mask
    -- byte.  Emit one event per live lane (typically just one).
    for lane = 0, 3 do
        local mbyte = (mask >> (lane * 8)) & 0xff
        if mbyte ~= 0 then
            local byte_addr = (offset & ~3) + (3 - lane)
            local mac_byte_off = (byte_addr & 0x000fffff) & ~Q700_MIRR_MASK
            if mac_byte_off >= 0x14000 and mac_byte_off <= 0x15fff then
                local off = (mac_byte_off - 0x14000) & 0xfff
                local b   = (data >> (lane * 8)) & 0xff
                file:write(string.format("%d\t%d\t%s\t%03x\t%02x\n",
                                          seq, cur_time_ns, rw, off, b))
                seq = seq + 1
                if seq >= limit then
                    print(string.format("mame_asc_capture: hit limit=%d, exiting", limit))
                    if file then file:flush(); file:close(); file = nil end
                    mach:exit()
                    return
                end
                break
            end
        end
    end
end

prog:install_read_tap(0x50000000, 0x50ffffff, "asc_capture_r",
    function(offset, data, mask) on_event("R", offset, data, mask) end)

prog:install_write_tap(0x50000000, 0x50ffffff, "asc_capture_w",
    function(offset, data, mask) on_event("W", offset, data, mask) end)

print(string.format("mame_asc_capture: tap installed (limit=%d) -> %s",
                    limit, out_path))

emu.register_stop(function()
    if file then
        file:flush(); file:close(); file = nil
    end
    print(string.format("mame_asc_capture: stopped, %d events captured", seq))
end)
