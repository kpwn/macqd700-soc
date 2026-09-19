-- Dump the active Q700 DAFB VRAM aperture from a MAME run.
--
-- Intended usage:
--   MAME_RTL_VRAM_DUMP=build/mame_runs/q700_vram.bin \
--     mame macqd700 -autoboot_delay 60 -autoboot_script tools/mame_dump_vram.lua
--
-- Optional environment:
--   MAME_RTL_VRAM_BASE=0xf9000000
--   MAME_RTL_VRAM_SIZE=0x200000

local function parse_int(value, fallback)
	if value == nil or value == "" then
		return fallback
	end
	return tonumber(value) or fallback
end

local path = os.getenv("MAME_RTL_VRAM_DUMP") or "build/mame_runs/q700_vram.bin"
local base = parse_int(os.getenv("MAME_RTL_VRAM_BASE"), 0xf9000000)
local size = parse_int(os.getenv("MAME_RTL_VRAM_SIZE"), 0x200000)
local chunk_size = 4096

local cpu = manager.machine.devices[":maincpu"]
local mem = cpu.spaces["program"]
local file = assert(io.open(path, "wb"))
local chunk = {}

for off = 0, size - 1 do
	chunk[#chunk + 1] = string.char(mem:read_u8(base + off))
	if #chunk == chunk_size then
		file:write(table.concat(chunk))
		chunk = {}
	end
end

if #chunk > 0 then
	file:write(table.concat(chunk))
end

file:close()
print(string.format("mame_dump_vram: wrote %s base=0x%08x size=0x%x", path, base, size))
manager.machine:exit()
