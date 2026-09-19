-- mame_scsi96_capture.lua  —  NCR 53C96 / TurboSCSI register tap for macqd700.
--
-- Captures the bit-exact register access sequence the Q700 ROM + Mac OS
-- SCSI Manager exchange with the on-board NCR53C96 through the DAFB
-- TurboSCSI front door.  Output is a CSV trace directly comparable with
-- the RTL-side trace ring in rtl/mac/scsi.v (read out over JTAG).
--
-- Output format (one event per line):
--     <sim_time_ns>,<R|W>,<reg_hex>,<byte_hex>
--
-- where <reg_hex> is:
--     0..f   NCR 53C96 register selector  ((byte_offset & 0xff) >> 4)
--     d      pseudo-DMA shim access (0x5000f100..0x5000f101)
--
-- Address-map note (src/mame/apple/macquadra700.cpp:564-565 and
-- src/mame/apple/dafb.cpp:971-991):
--   * 0x5000f000..0x5000f0ff  -> dafb_device::turboscsi_r/w<0>, which
--     does  m_ncr[0]->read(offset >> 4).  So the 53C96 register index is
--     the byte offset within the 0x100-byte window shifted right by 4.
--   * 0x5000f100..0x5000f101  -> turboscsi_dma_r/w<0> (16-bit pseudo-DMA
--     handshake; this is where DATA IN/OUT payload bytes actually move).
--   * Both entries carry mirror/select 0x00fc0000, i.e. address bits
--     18..23 are don't-care.  Masking the address with 0x0003ffff folds
--     every mirror back onto the canonical 0x0f000..0x0f1ff sub-range.
--
-- The taps are OBSERVERS: install_read_tap sees the value the device
-- already returned, it does not re-issue the read.  So unlike poking the
-- live chip over JTAG, this capture is non-destructive (no FIFO pops, no
-- interrupt clears).
--
-- Usage:
--     MAME_SCSI_TRACE_OUT=/path/out.csv \
--     MAME_SCSI_TRACE_LIMIT=2000000 \
--     xvfb-run -a mame -rompath /tmp/mame_rompath -video soft -sound none \
--          -nothrottle -seconds_to_run 45 -window macqd700 -hard /tmp/hd753.hd \
--          -skip_gameinfo -autoboot_delay 0 \
--          -autoboot_script tools/mame_scsi96_capture.lua

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]

local out_path = os.getenv("MAME_SCSI_TRACE_OUT") or "build/mame_runs/scsi96_q700.csv"
local limit    = tonumber(os.getenv("MAME_SCSI_TRACE_LIMIT") or "2000000")

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then
        os.execute(string.format("mkdir -p %q", dir))
    end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_scsi96_capture v1 - ns,rw,reg,byte  (reg d = pseudo-DMA shim)\n")
file:flush()

local count = 0

local function ns_now()
    return math.floor(mach.time:as_double() * 1e9)
end

local function on_event(rw, offset, data, mask)
    if count >= limit then return end
    local sub = offset & 0x0003ffff
    if sub < 0x0f000 or sub > 0x0f1ff then return end
    for lane = 0, 3 do
        local mbyte = (mask >> (lane * 8)) & 0xff
        if mbyte ~= 0 then
            -- Big-endian 32-bit space: lane 3 (mask 0xff000000) is the
            -- byte at the LOWEST address of the aligned word.
            local byte_addr = (offset & ~3) + (3 - lane)
            local sb  = byte_addr & 0x0003ffff
            local reg
            if sb >= 0x0f100 then
                reg = "d"                        -- pseudo-DMA shim
            else
                reg = string.format("%x", (sb & 0xff) >> 4)
            end
            local b = (data >> (lane * 8)) & 0xff
            file:write(string.format("%d,%s,%s,%02x\n", ns_now(), rw, reg, b))
            count = count + 1
            file:flush()
            if count >= limit then
                print(string.format("mame_scsi96_capture: hit limit=%d, exiting", limit))
                file:flush(); file:close(); file = nil
                mach:exit()
                return
            end
        end
    end
end

-- NARROW taps, one per mirror we care about.
--
-- IMPORTANT (measured 2026-08-07): installing a tap across the whole
-- 0x50000000-0x50ffffff I/O window SEGFAULTS mame 0.285 reproducibly at
-- ~3.034 s of emulated time (2/2 runs).  The identical command line with
-- NO autoboot script runs the full 45 s cleanly (EXIT=0, 198% speed), so
-- the crash is the wide tap, not MAME/the disk image.  The wide tap fires
-- on every VIA/SCC/ASC access too -- millions of Lua closure calls.
-- Narrowing to the 53C96 aperture alone both fixes the crash and captures
-- exactly the same events (verified against the wide-tap prefix).
--
-- The Q700 ROM/driver uses the 0x50f0f000 mirror (measured: every
-- captured access was 0x50f0f0X0).  We install on both that mirror and
-- the canonical 0x5000f000 base so a mirror switch cannot silently drop
-- events.
local tap_bases = { 0x50f0f000, 0x5000f000 }
-- MUST be a GLOBAL: the tap objects returned by install_*_tap are owned by
-- Lua.  If the only reference is a local in the autoboot chunk, the chunk
-- returns, the GC collects the taps, and MAME segfaults calling through the
-- freed callback.  Measured: with a local, mame dies after 701/737/815
-- events (a growing, non-deterministic count -- the use-after-free tell).
_G.scsi96_taps = {}
local taps = _G.scsi96_taps
for i, base in ipairs(tap_bases) do
    taps[#taps+1] = prog:install_read_tap(base, base + 0x1ff,
        string.format("scsi96_capture_r%d", i),
        function(offset, data, mask) on_event("R", offset, data, mask) end)
    taps[#taps+1] = prog:install_write_tap(base, base + 0x1ff,
        string.format("scsi96_capture_w%d", i),
        function(offset, data, mask) on_event("W", offset, data, mask) end)
end

print(string.format("mame_scsi96_capture: %d taps installed (limit=%d) -> %s",
                    #taps, limit, out_path))

local function on_stop()
    if file then
        file:flush(); file:close(); file = nil
    end
    print(string.format("mame_scsi96_capture: stopped, %d events captured", count))
end

if emu.add_machine_stop_notifier then
    emu.add_machine_stop_notifier(on_stop)
else
    emu.register_stop(on_stop)
end
