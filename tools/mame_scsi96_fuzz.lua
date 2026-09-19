-- mame_scsi96_fuzz.lua — MAME-side executor for the 53C96 differential
-- fuzzer (tools/fuzz/scsi_fuzz.py).  Golden-reference counterpart of
-- tb/tb_scsi_fuzz.cpp; both consume the same script files and emit
-- byte-identical result-log formats.  See docs/scsi_fuzz.md.
--
-- How the chip is driven (proven by probes, 2026-08-18):
--   * The machine boots macqd700; at frame 5 the CPU is parked on a
--     "bra ." in RAM with SR=0x2700, so nothing but this script touches
--     the 53C96 afterwards.
--   * Register accesses go through the CPU program space at the DAFB
--     TurboSCSI aperture 0x50f0f000 (reg = offset>>4), which dispatches
--     dafb_device::turboscsi_r/w -> ncr53c90 read/write with full side
--     effects (mame0285 src/mame/apple/dafb.cpp:971).  Pseudo-DMA beats
--     are byte reads/writes at +0x100; with the reset-default
--     scsi_ctrl=0 the aperture is a blind dma_r()/dma_w() pop/push.
--   * Time is stepped deterministically with a coroutine resumed from
--     emu.register_frame_done; SETTLE = 4 emulated frames (~67 ms),
--     which bounds every 53C96 timer the generator can arm (select
--     timeout is constrained to <= 7 -> <19 ms).
--   * Sync-point internal state (FIFO contents, irq, drq, config regs)
--     is read NON-destructively through the device's save-state items
--     (manager.machine.devices[":scsi:7:ncr53c96"].items + emu.item).
--
-- Usage (invoked by scsi_fuzz.py):
--   SCSI_FUZZ_DIR=<dir with NNN.txt> \
--   mame macqd700 -rompath <roms> ... -hard <fuzzdisk.chd> \
--        -autoboot_script tools/mame_scsi96_fuzz.lua
--
-- Known traps encoded here: print()+flush only (debugger printf is
-- silent headless); tap/item objects must stay referenced from _G.

local mach = manager.machine
local cpu  = mach.devices[":maincpu"]
local prog = cpu.spaces["program"]
local BASE = 0x50f0f000

local fuzz_dir = os.getenv("SCSI_FUZZ_DIR")
if not fuzz_dir then
    print("mame_scsi96_fuzz: SCSI_FUZZ_DIR not set")
    io.stdout:flush()
    mach:exit()
    return
end

local function rreg(r) return prog:read_u8(BASE + r*16) end
local function wreg(r, v) prog:write_u8(BASE + r*16, v & 0xff) end

-- ── pseudo-DMA aperture ──────────────────────────────────────────────
-- macquadra700.cpp:565 maps 0x5000f100..0x5000f101 (SIXTEEN bits wide)
-- through dafb_device::turboscsi_dma_r/w.  Access WIDTH selects which
-- 53C96 entry point runs (dafb.cpp:1012-1024 / :1048-1060):
--   read_u8  -> mem_mask 0xff00 -> ncr->dma_r() << 8      (one byte)
--   read_u16 -> mem_mask 0xffff -> ncr->dma16_swap_r()    (atomic pair)
--   write_u8 -> mem_mask 0xff00 -> ncr->dma_w(data >> 8)
--   write_u16-> mem_mask 0xffff -> ncr->dma16_swap_w()
-- The 16-bit forms are the ones the Q700 ROM's chunk drain actually
-- uses, and they are NOT two byte forms back to back: dma16_r pops both
-- bytes under ONE fifo_pos check (ncr53c90.cpp:1325-1349, incl. the
-- fifo_pos < 2 -> `dma_r() | 0xff00` underflow case) and dma16_w
-- degrades to a SINGLE dma_w when fifo_pos > 14 || tcounter == 1
-- (:1352-1358).  See docs/scsi_fuzz.md.
local function dma_rd() return prog:read_u8(BASE + 0x100) end
local function dma_wr(v) prog:write_u8(BASE + 0x100, v & 0xff) end
local function dma_rd16() return prog:read_u16(BASE + 0x100) end
local function dma_wr16(v) prog:write_u16(BASE + 0x100, v & 0xffff) end

-- ── DAFB TurboSCSI control word (bus 0) ──────────────────────────────
-- dafb_base::map puts the DAFB register file at 0xf9800000 and dafb_w
-- switches on (offset << 2), so m_scsi_ctrl[0] is the u32 at
-- 0xf9800024 (dafb.cpp:487).  bit 7 = DRQ Check Read, bit 8 = DRQ Check
-- Write.  Reset default is 0 = a blind aperture, which is why every
-- hold-off path was dead code before the fuzzer could write this.
local DAFB_SCSI_CTRL = 0xf9800024
local ctrl_val = 0
local function set_ctrl(v)
    ctrl_val = v & 0x1ff
    prog:write_u32(DAFB_SCSI_CTRL, ctrl_val)
end
local function rd_check_on() return (ctrl_val & 0x080) ~= 0 end
local function wr_check_on() return (ctrl_val & 0x100) ~= 0 end

-- ── save-state item access to the chip internals ─────────────────────
local ncr = mach.devices[":scsi:7:ncr53c96"]
if not ncr then
    print("mame_scsi96_fuzz: :scsi:7:ncr53c96 not found")
    io.stdout:flush(); mach:exit(); return
end
local IT = {}
do
    local items = ncr.items
    local function grab(name)
        local idx = items["0/" .. name]
        if not idx then
            print("mame_scsi96_fuzz: missing save item " .. name)
            io.stdout:flush(); mach:exit()
        end
        return emu.item(idx)
    end
    IT.fifo      = grab("fifo")
    IT.fifo_pos  = grab("fifo_pos")
    IT.irq       = grab("irq")
    IT.drq       = grab("drq")
    IT.bus_id    = grab("bus_id")
    IT.sel_to    = grab("select_timeout")
    IT.sync_per  = grab("sync_period")
    IT.sync_off  = grab("sync_offset")
    IT.clk_conv  = grab("clock_conv")
    IT.config    = grab("config")
    IT.config2   = grab("config2")
    IT.config3   = grab("config3")
    IT.command   = grab("command")
    IT.cmd_pos   = grab("command_pos")
    IT.tcount    = grab("tcount")
end
local function item_v(it) return it:read(0) or 0 end

-- ── script list ──────────────────────────────────────────────────────
local scripts = {}
do
    -- MAME's Lua has no dirent; ls via popen (host side sets sane names)
    local p = io.popen("ls '" .. fuzz_dir .. "'")
    for name in p:lines() do
        if name:match("%.txt$") then scripts[#scripts+1] = name end
    end
    p:close()
    table.sort(scripts)
end

-- ── op execution (inside a coroutine; yield() = wait one frame) ──────
local SETTLE_FRAMES  = 4
local DMA_YIELD_MAX  = 120   -- frames to wait for DRQ per DMA beat

-- ── pseudo-DMA op table ──────────────────────────────────────────────
-- word  = 16-bit host access (dma16_swap_r/w) instead of a byte access.
-- paced = wait for DRQ before the beat.  The BLIND (unpaced) forms exist
-- because the Q700 ROM's 16-byte chunk drain issues eight back-to-back
-- `move.w` with no DRQ poll at all; a paced drain stalls at the odd
-- fifo occupancy under LBTM and can never reproduce it.
local DMA_RD_OPS = {
    DR     = {word=false, paced=true },
    DRU    = {word=false, paced=true },
    DRB    = {word=false, paced=false},
    DRUB   = {word=false, paced=false},
    DR16   = {word=true,  paced=true },
    DRU16  = {word=true,  paced=true },
    DRB16  = {word=true,  paced=false},
    DRUB16 = {word=true,  paced=false},
}
local DMA_WR_OPS = {
    DW    = {word=false, paced=true },
    DWB   = {word=false, paced=false},
    DW16  = {word=true,  paced=true },
    DWB16 = {word=true,  paced=false},
}

-- One aperture beat's gating decision.  MUST stay byte-identical in
-- meaning to tb_scsi_fuzz.cpp's beat_allowed():
--   paced -> wait (bounded) for DRQ, give up if it never rises
--   then  -> if the DAFB check for this direction is armed and DRQ is
--            low, the access is HELD OFF (dafb.cpp:1003-1009 rewinds the
--            instruction and returns 0xffff without touching the chip),
--            so the op stops without issuing it.
local function beat_allowed(paced, is_write)
    if paced then
        local waited = 0
        while item_v(IT.drq) == 0 do
            waited = waited + 1
            if waited > DMA_YIELD_MAX then break end
            coroutine.yield()
        end
        if item_v(IT.drq) == 0 then return false end
    end
    -- NOT `is_write and wr_check_on() or rd_check_on()`: Lua's and/or
    -- falls through to the third term whenever the second is false, which
    -- would silently consult the READ check bit on a write beat.
    local check
    if is_write then check = wr_check_on() else check = rd_check_on() end
    if check and item_v(IT.drq) == 0 then return false end
    return true
end

local function do_sync(out, id)
    -- Internal snapshot FIRST (bus reads below mutate chip state);
    -- field set, order and format mirror tb_scsi_fuzz.cpp:emit_sync.
    local fp = item_v(IT.fifo_pos) & 0x1f
    local fifostr = string.format("%d:", fp)
    for i = 0, math.min(fp, 16) - 1 do
        fifostr = fifostr .. string.format("%02x", IT.fifo:read(i) & 0xff)
    end
    local irq  = (item_v(IT.irq) ~= 0) and 1 or 0
    local drq  = (item_v(IT.drq) ~= 0) and 1 or 0
    local cfg = string.format("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
        item_v(IT.bus_id) & 7, item_v(IT.sel_to) & 0xff,
        item_v(IT.sync_per) & 0x1f, item_v(IT.sync_off) & 0xf,
        item_v(IT.clk_conv) & 7, item_v(IT.config) & 0xff,
        item_v(IT.config2) & 0xff, item_v(IT.config3) & 0xff)
    local cmd0 = IT.command:read(0) & 0xff
    local cpos = item_v(IT.cmd_pos) & 3
    local tcnt = item_v(IT.tcount) & 0xffff
    -- Bus reads in fixed order; istatus (destructive) last.
    local st  = rreg(4)
    local sq  = rreg(6)
    local fl  = rreg(7)
    local tlo = rreg(0)
    local thi = rreg(1)
    local is  = rreg(5)
    out:write(string.format(
        "SYNC %s fifo=%s irq=%d drq=%d cfg=%s cmd=%02x:%d tcount=%04x" ..
        " stat=%02x seq=%02x flags=%02x tclo=%02x tchi=%02x istat=%02x\n",
        id, fifostr, irq, drq, cfg, cmd0, cpos, tcnt, st, sq, fl, tlo, thi, is))
end

local function run_script(path, outpath)
    local f = assert(io.open(path, "r"))
    local out = assert(io.open(outpath, "w"))
    for line in f:lines() do
        local s = line:match("^%s*(.-)%s*$")
        if s ~= "" and s:sub(1,1) ~= "#" then
            local toks = {}
            for w in s:gmatch("%S+") do toks[#toks+1] = w end
            local op = toks[1]
            if op == "W" then
                wreg(tonumber(toks[2], 16), tonumber(toks[3], 16))
            elseif op == "RC" or op == "RU" then
                local r = tonumber(toks[2], 16)
                out:write(string.format("%s %x=%02x\n", op, r, rreg(r)))
            elseif DMA_RD_OPS[op] then
                local kind = DMA_RD_OPS[op]
                local n = tonumber(toks[2], 16)
                local data = {}
                for i = 1, n do
                    if not beat_allowed(kind.paced, false) then break end
                    if kind.word then data[#data+1] = dma_rd16()
                    else              data[#data+1] = dma_rd() end
                end
                local hex = {}
                local fmt = kind.word and "%04x" or "%02x"
                for i = 1, #data do hex[i] = string.format(fmt, data[i]) end
                -- to=0 always: MAME's aperture cannot fail to terminate,
                -- so a to=1 on the RTL side is by construction a
                -- divergence (see tb_scsi_fuzz.cpp's header).
                out:write(string.format("%s %x got=%x data=%s to=0\n",
                    op, n, #data, table.concat(hex)))
            elseif DMA_WR_OPS[op] then
                local kind = DMA_WR_OPS[op]
                local n = tonumber(toks[2], 16)
                local put = 0
                for i = 1, n do
                    local b = tonumber(toks[2 + i], 16)
                    if b == nil then break end
                    if not beat_allowed(kind.paced, true) then break end
                    if kind.word then dma_wr16(b) else dma_wr(b) end
                    put = put + 1
                end
                out:write(string.format("%s %x put=%x to=0\n", op, n, put))
            elseif op == "CTRL" then
                set_ctrl(tonumber(toks[2], 16))
            elseif op == "GAP" or op == "SDGAP" then
                -- RTL-only timing knobs: no emulated time passes here.
            elseif op == "SETTLE" then
                for i = 1, SETTLE_FRAMES do coroutine.yield() end
            elseif op == "SYNC" then
                do_sync(out, toks[2])
            elseif op == "END" then
                break
            else
                print("mame_scsi96_fuzz: unknown op " .. tostring(op))
            end
        end
    end
    f:close()
    out:close()
end

-- ── main coroutine ───────────────────────────────────────────────────
local co = coroutine.create(function()
    for i = 1, 5 do coroutine.yield() end
    -- Park the CPU: bra-. in RAM, interrupts masked.  Verified: the
    -- overlay has cleared by frame 5 and the ROM has not touched the
    -- 53C96 yet (all registers read back power-on values).
    prog:write_u16(0x2000, 0x60FE)
    cpu.state["SR"].value = 0x2700
    cpu.state["PC"].value = 0x2000
    coroutine.yield()
    if prog:read_u16(0x2000) ~= 0x60FE or cpu.state["PC"].value > 0x2004 then
        print("mame_scsi96_fuzz: CPU park FAILED")
        io.stdout:flush(); mach:exit(); return
    end
    for _, name in ipairs(scripts) do
        local base = name:sub(1, #name - 4)
        run_script(fuzz_dir .. "/" .. name,
                   fuzz_dir .. "/" .. base .. ".mame.log")
    end
    print(string.format("mame_scsi96_fuzz: %d scripts executed", #scripts))
    io.stdout:flush()
    mach:exit()
end)

-- MUST live in _G: locals of the autoboot chunk get GC'd (measured
-- use-after-free crashes in mame_scsi96_capture.lua's history).
_G.scsi96_fuzz_frame_cb = function()
    if coroutine.status(co) ~= "dead" then
        local ok, err = coroutine.resume(co)
        if not ok then
            print("mame_scsi96_fuzz: LUA ERROR: " .. tostring(err))
            io.stdout:flush()
            mach:exit()
        end
    end
end
emu.register_frame_done(_G.scsi96_fuzz_frame_cb)
