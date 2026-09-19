-- Dump a contiguous region of the Q700 CPU address space after boot.
--
-- Usage:
--   MAME_RAM_DUMP=/tmp/q700.ram MAME_RAM_BASE=0 MAME_RAM_SIZE=0x800000 \
--     mame -rompath /tmp/mame_rompath -video none -sound none -nothrottle \
--          -autoboot_delay 60 -autoboot_script tools/mame_dump_ram.lua \
--          -seconds_to_run 90 macqd700 -hard /path/to/disk.hda -ramsize 8M

local function parse_int(value, fallback)
	if value == nil or value == "" then
		return fallback
	end
	return tonumber(value) or fallback
end

local path = os.getenv("MAME_RAM_DUMP") or "/tmp/q700.ram"
local base = parse_int(os.getenv("MAME_RAM_BASE"), 0)
local size = parse_int(os.getenv("MAME_RAM_SIZE"), 0x800000)
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
print(string.format("mame_dump_ram: wrote %s base=0x%08x size=0x%x", path, base, size))
manager.machine:exit()
