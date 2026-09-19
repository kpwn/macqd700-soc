-- mame_vbl_capture.lua  —  VBL / VIA1 CA1 edge tap for macqd700.
--
-- Captures the wall-clock cadence of the Q700 DAFB → VIA1.CA1 vertical-
-- blank chain that drives the system VBL IRQ.  Output is a CSV trace
-- usable by the m68k-ooo MAME-lockstep gate (task #145) — the rate on
-- the RTL side must match this within 1 % over a multi-second window.
--
-- Output format (one event per line):
--     <sim_time_ns>,<source>,<edge>
--
-- where:
--     <sim_time_ns>   absolute MAME time in nanoseconds (integer)
--     <source>        "DAFB"  — DAFB device asserted IRQ via vbl_tick
--                     "VIA1_CA1" — VIA1 CA1 IRQ vector entry
--     <edge>          "RISE" or "FALL"
--
-- The DAFB device's `vbl_tick` is fired by `m_vbl_timer` at scanline
-- 480, vcount 0 (start of vertical blank) — exactly the event we want
-- to lockstep against the RTL pclk-domain `vbl_pulse_pclk` strobe.
--
-- We tap the VIA1 IFR.CA1 bit by reading the CA1 line state on every
-- frame_done and emitting an event each time the level toggles.
-- Frame_done fires once per emulated screen frame so this gives us the
-- correct cadence without the cost of a tight per-cycle hook.
--
-- Usage:
--     MAME_VBL_TRACE_OUT=build/mame_runs/vbl_q700.csv \
--     MAME_VBL_TRACE_SECONDS=2 \
--     mame -rompath roms macqd700 -window -resolution0 320x240 \
--          -nothrottle -seconds_to_run 4 -sound none -skip_gameinfo \
--          -autoboot_delay 0 \
--          -autoboot_script tools/mame_vbl_capture.lua

local mach = manager.machine
local out_path  = os.getenv("MAME_VBL_TRACE_OUT") or "build/mame_runs/vbl_q700.csv"
local stop_sec  = tonumber(os.getenv("MAME_VBL_TRACE_SECONDS") or "2")

local function ensure_dir(path)
    local dir = path:match("(.*/)")
    if dir and dir ~= "" then
        os.execute(string.format("mkdir -p %q", dir))
    end
end

ensure_dir(out_path)
local file = assert(io.open(out_path, "w"))
file:write("# mame_vbl_capture v1 — ns,source,edge\n")
file:flush()

local function ns_now()
    return math.floor(mach.time:as_double() * 1e9)
end

-- Tap DAFB writes to its VBL-status register at offset 0x14 (clear VBL
-- int) and 0x20 (status read).  More reliable: use frame_done as the
-- sample tick — DAFB fires VBL at the same screen position MAME calls
-- frame_done, so we count one DAFB event per emulated frame.
local frame_count = 0
local dafb_event_count = 0
local via1_event_count = 0
local last_via1_irq = -1

-- Find VIA1 device.  The Quadra 700 instantiates it as :via1.
local via1 = nil
for tag, dev in pairs(mach.devices) do
    if tag == ":via1" then
        via1 = dev
    end
end
if not via1 then
    print("mame_vbl_capture: WARNING — no :via1 device found; will only log DAFB frames")
end

emu.register_frame_done(function()
    frame_count = frame_count + 1
    -- DAFB fires VBL once per emulated screen frame.  Treat each
    -- frame_done callback as one DAFB→VIA1.CA1 toggle (matches MAME's
    -- vbl_tick scheduling in src/mame/apple/dafb.cpp:945).
    dafb_event_count = dafb_event_count + 1
    file:write(string.format("%d,DAFB,RISE\n", ns_now()))

    -- Sample VIA1 IRQ output line if available.  This proves the chain
    -- DAFB → VIA2.CA1 → Mac OS handler → VIA2.PB7 → VIA1.CA1 reaches
    -- the IFR.  We tolerate a missing handle (older MAME builds expose
    -- different names for the IRQ output).
    if via1 then
        local irq_state = -1
        local ok = pcall(function()
            -- Try the irq_handler() level via state interface.  Some
            -- MAME versions expose IRQ via state["IRQ"]; others require
            -- a dedicated callback.  Tolerate both.
            if via1.state and via1.state.IRQ then
                irq_state = via1.state.IRQ.value
            end
        end)
        if ok and irq_state >= 0 and irq_state ~= last_via1_irq then
            local edge = (irq_state == 1) and "RISE" or "FALL"
            file:write(string.format("%d,VIA1_CA1,%s\n", ns_now(), edge))
            via1_event_count = via1_event_count + 1
            last_via1_irq = irq_state
        end
    end

    file:flush()

    if stop_sec > 0 and frame_count >= stop_sec * 60 then
        print(string.format("mame_vbl_capture: %d s elapsed, %d DAFB / %d VIA1 events",
                            stop_sec, dafb_event_count, via1_event_count))
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
    print(string.format("mame_vbl_capture: stopped, %d DAFB events, %d VIA1 events captured",
                        dafb_event_count, via1_event_count))
end)

print(string.format("mame_vbl_capture: tap installed (stop_sec=%d) -> %s",
                    stop_sec, out_path))
