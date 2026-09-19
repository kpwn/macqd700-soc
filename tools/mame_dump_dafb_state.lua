-- Dump live Q700 DAFB internal state (hres/vres/base/stride/bpp/timing) and
-- a snapshot of VRAM after the boot ROM has fully programmed the display.
--
-- Usage:
--   QT_QPA_PLATFORM=offscreen \
--     MAME_DAFB_OUT=/tmp/dafb_state.txt \
--     MAME_VRAM_OUT=/tmp/vram_dump.bin \
--     mame -rompath /tmp/mame_rompath -video none -sound none -nothrottle \
--          -autoboot_delay 60 \
--          -autoboot_script tools/mame_dump_dafb_state.lua \
--          -seconds_to_run 90 macqd700

local out_path  = os.getenv("MAME_DAFB_OUT")  or "/tmp/dafb_state.txt"
local vram_path = os.getenv("MAME_VRAM_OUT")  or "/tmp/vram_dump.bin"

local cpu = manager.machine.devices[":maincpu"]
local mem = cpu.spaces["program"]

-- DAFB live state lives inside the device member fields.  MAME exposes
-- them via the device tree; pull what we can via memory reads at the
-- DAFB register window ($f9800000) and via the device state if reachable.
local f = assert(io.open(out_path, "wb"))

local function p(line) f:write(line .. "\n") end

p("# MAME Q700 DAFB live state snapshot")

-- DAFB raw register-window dump (first 0x70 bytes covers control + Swatch
-- horizontal/vertical timing).
p("# raw DAFB register window 0xF9800000..0xF98000FF")
for off = 0, 0x100 - 4, 4 do
    local v = mem:read_u32(0xf9800000 + off)
    p(string.format("  +%03x = 0x%08x", off, v))
end

-- Swatch register window 0xF9800100..0xF98001FF (horizontal/vertical
-- timing parameters live here at offsets 0x24..0x64).
p("# Swatch window 0xF9800100..0xF98001FF")
for off = 0, 0x100 - 4, 4 do
    local v = mem:read_u32(0xf9800100 + off)
    p(string.format("  +%03x = 0x%08x", off, v))
end

-- AC842 RAMDAC + clockgen register window 0xF9800200..0xF98003FF.
p("# RAMDAC/clockgen window 0xF9800200..0xF98003FF")
for off = 0, 0x200 - 4, 4 do
    local v = mem:read_u32(0xf9800200 + off)
    if v ~= 0 then
        p(string.format("  +%03x = 0x%08x", 0x200 + off, v))
    end
end

-- Try to peek device member fields directly.  These are typically
-- exposed through the device's `state()` view in MAME but vary by build.
local dev = manager.machine.devices[":dafb"]
if dev then
    p("# :dafb device found")
    -- device:state() is a state interface — try iterating
    local ok, state = pcall(function() return dev.state end)
    if ok and state then
        for k, v in pairs(state.items or {}) do
            p(string.format("  state.%s = %s", tostring(k), tostring(v.value or v)))
        end
    end
end

-- Also dump VRAM aperture.
p(string.format("# VRAM dump → %s", vram_path))
local vf = assert(io.open(vram_path, "wb"))
local chunk = {}
for off = 0, 0x200000 - 1 do
    chunk[#chunk + 1] = string.char(mem:read_u8(0xf9000000 + off))
    if #chunk == 4096 then
        vf:write(table.concat(chunk))
        chunk = {}
    end
end
if #chunk > 0 then vf:write(table.concat(chunk)) end
vf:close()
p(string.format("# VRAM dump complete: %d bytes", 0x200000))

f:close()
print("mame_dump_dafb_state: wrote " .. out_path .. " and " .. vram_path)
manager.machine:exit()
