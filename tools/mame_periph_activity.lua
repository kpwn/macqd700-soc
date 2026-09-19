-- mame_periph_activity.lua — post-boot driver-activity driver for macqd700.
--
-- Loaded by tools/mame_periph_strobe_capture.lua when $MAME_STROBE_ACTIVITY
-- points at it.  Its job is to make a MAME System 7 session exercise real
-- driver code paths beyond the ROM boot, so a reachability capture is not
-- limited to what the ROM alone touches:
--
--   ADB / VIA1   continuous mouse motion + desktop clicks (ADB transactions
--                run through VIA1's shift register)
--   SCSI + VIA   Cmd-Shift-3 screen shots — the Finder writes a "Picture N"
--                file, i.e. the OS *disk-write* path (the path the SCSI-DMA
--                shim finding in commit eba8aa2 was about)
--   IWM / SWIM   a floppy insert; the .Sony driver polls and reads the disk
--   SCC          left to the OS (AppleTalk/serial init runs on its own)
--
-- Timings are in emulated seconds and assume the machine reaches the Finder
-- by ~25 s.  Everything is best-effort and wrapped in pcall so a failure to
-- find a device/field never aborts the capture.
--
-- Env:
--   MAME_ACTIVITY_FLOPPY   path to a floppy image to insert at T_FLOPPY
--   MAME_ACTIVITY_START    emulated second to begin (default 25)

local amach   = manager.machine
local a_start = tonumber(os.getenv("MAME_ACTIVITY_START") or "25")
local a_flop  = os.getenv("MAME_ACTIVITY_FLOPPY")

local function field(port_tag, field_name)
    local p = amach.ioport.ports[port_tag]
    if not p then return nil end
    return p.fields[field_name]
end

-- Held-key bookkeeping: MAME lua fields expose :set_value(v) for override.
local function hold(f, on)
    if not f then return end
    pcall(function()
        if on then f:set_value(1) else f:clear_value() end
    end)
end

local K_CMD   = field(":macadb:KEY3", "Command / Open Apple")
local K_SHIFT = field(":macadb:KEY3", "Shift")
local K_3     = field(":macadb:KEY1", "3  #")
local M_BTN   = field(":macadb:MOUSE0", "Mouse Button 0")
local M_X     = field(":macadb:MOUSE1", "Mouse X")
local M_Y     = field(":macadb:MOUSE2", "Mouse Y")

-- A tiny time-ordered script.  Each entry: {at_seconds, function}
local script = {}
local function at(t, fn) script[#script + 1] = { t = a_start + t, fn = fn, done = false } end

-- Screen shot #1 (Cmd-Shift-3): press, hold ~0.25 s, release.
at(0.0, function() hold(K_CMD, true); hold(K_SHIFT, true) end)
at(0.2, function() hold(K_3, true) end)
at(0.4, function() hold(K_3, false) end)
at(0.6, function() hold(K_CMD, false); hold(K_SHIFT, false) end)

-- Screen shot #2, a few seconds later (second disk write, different file).
at(6.0, function() hold(K_CMD, true); hold(K_SHIFT, true) end)
at(6.2, function() hold(K_3, true) end)
at(6.4, function() hold(K_3, false) end)
at(6.6, function() hold(K_CMD, false); hold(K_SHIFT, false) end)

-- Desktop click (Finder redraw + ADB button traffic).
at(12.0, function() hold(M_BTN, true) end)
at(12.2, function() hold(M_BTN, false) end)
at(13.0, function() hold(M_BTN, true) end)
at(13.2, function() hold(M_BTN, false) end)

-- Floppy insert/eject cycles — each insert wakes the .Sony driver, which
-- drives SWIM/IWM registers hard (seek, read, format-probe).  Verified
-- working: the Finder puts up "This is not a Macintosh disk" each time.
local function flop_dev()
    for _, img in pairs(amach.images) do
        if img.instance_name and img.instance_name:find("flop") then return img end
    end
    return nil
end
for cyc = 0, 3 do
    at(15.0 + cyc * 8.0, function()
        if not a_flop or a_flop == "" then return end
        local d = flop_dev()
        if d then pcall(function() d:load(a_flop) end) end
    end)
    at(20.0 + cyc * 8.0, function()
        local d = flop_dev()
        if d then pcall(function() d:unload() end) end
    end)
end

-- Screen shot #3 after the floppy dialog.
at(30.0, function() hold(K_CMD, true); hold(K_SHIFT, true) end)
at(30.2, function() hold(K_3, true) end)
at(30.4, function() hold(K_3, false) end)
at(30.6, function() hold(K_CMD, false); hold(K_SHIFT, false) end)

-- Continuous mouse motion from a_start onward: a small dither every frame,
-- which keeps the ADB (and hence VIA1) transaction stream busy.
local mdir = 3
emu.register_frame_done(function()
    local now = amach.time.seconds
    if now < a_start then return end
    if M_X and M_Y then
        pcall(function()
            M_X:set_value(mdir & 0xff)
            M_Y:set_value((256 - mdir) & 0xff)
        end)
        mdir = -mdir
    end
    for _, s in ipairs(script) do
        if not s.done and now >= s.t then
            s.done = true
            local ok, err = pcall(s.fn)
            if not ok then print("activity: step failed: " .. tostring(err)) end
        end
    end
end)

print(string.format("mame_periph_activity: armed (start=%.1fs floppy=%s)",
    a_start, a_flop or "(none)"))
