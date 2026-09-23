// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// rtc.v — off-chip Mac RTC (real-time clock) peripheral
//
// Classic Macintosh machines wire a small custom RTC chip (Apple "58F1",
// aka the 343S0042 RTC+PRAM IC) to VIA1 via three GPIO pins:
//
//   rtcEnb  — active-low chip select (PB2)
//   rtcClk  — clock                  (PB1)
//   rtcData — bidir data             (PB0)
//
// The bit-level protocol (Apple's "IIgs-compat" mode — Mac uses the
// 58321 format originally, later the 343S0042):
//
//   1. Mac drives rtcEnb low.
//   2. Mac shifts in an 8-bit command MSB-first on rtcClk falling edges.
//      Command byte encoding (we implement the 4-byte seconds flavour):
//         bit 7 = R/W (1 = read, 0 = write)
//         bits 6..2 = register address
//         bits 1..0 = register index within group
//      "Read Seconds" commands: 0x81, 0x85, 0x89, 0x8D  — each returns
//      one byte of the 32-bit seconds counter, least-significant byte
//      first.  This matches MAME macrtc.cpp and the 343-0042 register
//      byte order.
//      "Write Seconds" commands: 0x01, 0x05, 0x09, 0x0D.
//   3. If write: Mac continues to shift in 8 bits of data.
//      If read:  rtc drives rtcData on each rtcClk falling edge.
//      Extended PRAM commands use a second address byte, then either an
//      8-bit read response or an 8-bit write data byte.
//   4. Mac raises rtcEnb to end the transaction.
//
// The rtcEnb falling edge resets the shift FSM, the first 8 bits are the
// command, and the command decoder determines whether one normal data byte
// or the extended address/data sequence follows.  Multi-byte seconds reads
// are issued by the CPU as 4 separate normal transactions with addresses
// 0x81, 0x85, 0x89, 0x8D; that matches the actual Apple driver behaviour
// in the Mac ROM.
//
// Internal state:
//   - seconds [31:0] — Apple epoch (1904-01-01 00:00:00 GMT).  Ticks
//     once every SEC_DIV phi2 pulses (SEC_DIV = 1_000_000 for 1 Hz at
//     1 MHz phi2).  Parametrised for simulation: tb can pass a low
//     divisor so the test runs in fewer cycles.
//   - cko — 1 Hz clock output to VIA1 CA2.  It resets high and toggles
//     every half-second; the seconds register advances on the rising edge,
//     matching MAME's rtc3430042_device.
//   - pram[0:255] — 256 bytes of "parameter RAM" (user settings).
//   - write_protect / test_mode — deterministic control bits the ROM
//     can probe without introducing wall-clock or NVRAM dependence.
//   - +rtc_mame_state — simulation-only reset mode that matches MAME's
//     clean-NVRAM default: zero-filled PRAM.  This is useful for MAME/RTL
//     lockstep, while the default reset keeps the known-good PRAM image
//     used by standalone RTL boot probes.
//   - +rtc_init_seconds=<n> — simulation-only initial seconds counter,
//     used with a fixed MAME RTC date for deterministic lockstep runs.
//   - +rtc_trace — optional simulation-only access log for command/data
//     traffic.  Useful when correlating ROM-visible PRAM traffic with
//     the VIA1 bit-bang path.
//
// Fast sim mode:
//   - Verilator/iverilog runs can pass `+rtc_fast` on the command line
//     to divide the effective SEC_DIV by SEC_DIV_FAST_RATIO (default
//     1000).  A ROM-boot sim that otherwise waits 50M cycles for one
//     RTC second completes in 50k.  Gated by an `initial` block +
//     `$test$plusargs`, so it is simulation-only — synthesis never
//     sees the check (synth treats $test$plusargs as false, leaving
//     the register at 0).
//
// Latency / semantics:
//   - Shift registers clocked by falling edge of rtcClk.  This matches
//     MAME macrtc.cpp: host write data must be valid before the falling
//     edge, and read data becomes valid after that same edge.

module rtc #(
    // Number of phi2 pulses per real-time second.  Default assumes a
    // 1 MHz phi2 — for simulation an obviously shorter value (e.g.
    // 1_000) makes second ticks observable in a reasonable test length.
    parameter integer SEC_DIV = 32'd1_000_000,
    // In +rtc_fast mode, SEC_DIV is divided by this ratio to shorten
    // the sim time to first RTC tick.  1000× default is enough to
    // collapse a one-second wait from 1M phi2 pulses to 1k.
    parameter integer SEC_DIV_FAST_RATIO = 32'd1_000
) (
    input  wire        clk,
    input  wire        rst,

    // 1 MHz timebase (shared with VIA1)
    input  wire        phi2_tick,

    // RTC wires from VIA1 PB0..PB2
    input  wire        rtc_enb,       // active-HIGH idle, LOW during transaction
    input  wire        rtc_clk,
    input  wire        rtc_data_o,    // from VIA1 (CPU drives)
    input  wire        rtc_data_oe,   // DDRB[0]: 1 = CPU driving

    // Explicit "zap the PRAM" strobe — the RTL equivalent of a real
    // Macintosh's Cmd-Opt-P-R.  PRAM models battery-backed storage and
    // therefore deliberately survives `rst`; this synchronous input is
    // the ONLY way to force all 256 bytes back to their power-on image.
    // Hold high for >=1 clk. A rising edge starts a 256-clock sweep.
    // Serial transactions must start after pram_busy falls.
    input  wire        pram_clear,
    output wire        pram_busy,

    // ── External PRAM snapshot / restore port ─────────────────────────
    //
    // A byte-wide back door onto the same 256-byte array, used ONLY by
    // rtl/soc/pram_sd.v to persist PRAM to the SD card and restore it —
    // both strictly manual JTAG operations.  It is NOT reachable from the
    // Mac's address map: the 68k still sees PRAM exclusively through the
    // bit-banged RTC command protocol above.
    //
    //   pram_ext_rdata  one-clock synchronous read (read-before-write).
    //   pram_ext_we     write strobe, accepted only when !pram_busy.
    //
    // Tie pram_ext_we to 1'b0 and leave the rest unconnected if unused.
    // See rtl/soc/pram_cdc.v for the core_clk <-> pb_clk handshake that
    // keeps addr/wdata stable across this port.
    input  wire [7:0]  pram_ext_addr,
    input  wire        pram_ext_we,
    input  wire [7:0]  pram_ext_wdata,
    output wire [7:0]  pram_ext_rdata,

    output reg         cko,            // clock output to VIA1 CA2
    output reg         rtc_data_i     // to VIA1 pb_in[0]
);

    // ── Real-time seconds counter ─────────────────────────────────────
    //
    // `fast_mode` is a simulation-only register: set to 1 by $test$plusargs
    // if the caller passes `+rtc_fast`.  Synthesis sees the initial value
    // (0) and treats $test$plusargs as false, so the effective divisor is
    // unchanged.  In sim with +rtc_fast, SEC_DIV is divided by
    // SEC_DIV_FAST_RATIO at first-evaluation and the seconds register
    // advances 1000× faster.
    reg        fast_mode;
    reg        mame_state_mode;
    reg        trace_mode;
    reg [31:0] init_seconds;
    initial begin
        fast_mode = 1'b0;
        // mame_state_mode = 1 is the DEFAULT in sim: PRAM zero-fill matches
        // MAME's macqd700 PRAM defaults.  The populated "SCBI"-magic image
        // (`pram_reset_byte`) is opt-in via +rtc_populated_pram, sim only.
        //
        // SYNTHESIS RESETS PRAM TO ZERO TOO — see pram_reset_value() below,
        // which is `ifdef SYNTHESIS -> unconditional 8'h00.  So hardware and
        // MAME boot from the SAME (empty) PRAM, and this flag has no effect
        // on a real bitstream.  A previous revision of this comment claimed
        // the synthesis path defaulted to the POPULATED image; that was
        // false, it contradicted pram_reset_value() twenty lines down, and
        // it cost a wrong root-cause hypothesis on 2026-08-02 (someone read
        // the comment, concluded HW and MAME ran different PRAM, and built a
        // theory on it).  Do not restate the selection rule here — point at
        // pram_reset_value(), which is the only place that decides.
        //
        // Background on why zero is the contract: the populated image's
        // bytes 0xF8-0xFB ('SCBI') hit a ROM checksum trampoline at
        // 0x40846CE6 that MAME's empty PRAM doesn't, branching the boot into
        // the storage-descriptor path and wedging earlier than MAME.  See
        // the `m68k-mame-rtl-bisect` skill for how that was found.
        mame_state_mode = 1'b1;
        trace_mode = 1'b0;
        init_seconds = 32'd0;
`ifndef SYNTHESIS
        if ($test$plusargs("rtc_fast")) fast_mode = 1'b1;
        // Legacy +rtc_mame_state alias: kept for explicitness.  Either
        // plusarg keeps mame_state_mode=1 (which is now the default).
        if ($test$plusargs("rtc_mame_state")) mame_state_mode = 1'b1;
        // Opt-in to the populated SCBI-magic PRAM image.
        if ($test$plusargs("rtc_populated_pram")) mame_state_mode = 1'b0;
        if (!$value$plusargs("rtc_init_seconds=%d", init_seconds))
            init_seconds = 32'd0;
        if ($test$plusargs("rtc_trace")) trace_mode = 1'b1;
`endif
    end

    // Per-tick divisor: the real SEC_DIV (1 Hz) or the fast-sim one.
    wire [31:0] sec_div_raw = fast_mode
                              ? (SEC_DIV / SEC_DIV_FAST_RATIO)
                              : SEC_DIV[31:0];
    wire [31:0] sec_div_eff = (sec_div_raw == 32'd0) ? 32'd1 : sec_div_raw;

    wire [31:0] half_div_raw = sec_div_eff >> 1;
    wire [31:0] half_div_eff = (half_div_raw == 32'd0) ? 32'd1 : half_div_raw;

    reg [31:0] seconds;
    reg [31:0] half_div_cnt;
    reg        seconds_wr_pending;
    reg [1:0]  seconds_wr_idx;
    reg [7:0]  seconds_wr_data;
    reg        write_protect;
    reg        test_mode;
    always @(posedge clk) begin
        if (rst) begin
            seconds     <= init_seconds;
            half_div_cnt <= 32'd0;
            cko         <= 1'b1;
        end else if (seconds_wr_pending) begin
            case (seconds_wr_idx)
                2'd0: seconds[7:0]   <= seconds_wr_data;
                2'd1: seconds[15:8]  <= seconds_wr_data;
                2'd2: seconds[23:16] <= seconds_wr_data;
                2'd3: seconds[31:24] <= seconds_wr_data;
                default: ;
            endcase
            half_div_cnt <= 32'd0;
        end else if (phi2_tick) begin
            if (half_div_cnt >= half_div_eff - 32'd1) begin
                half_div_cnt <= 32'd0;
                cko          <= ~cko;
                if (!cko)
                    seconds <= seconds + 32'd1;
            end else begin
                half_div_cnt <= half_div_cnt + 32'd1;
            end
        end
    end

    // ── PRAM (256 bytes) — BATTERY-BACKED, NOT RESET ──────────────────
    //
    // On a real Macintosh the PRAM lives in the RTC IC and is kept alive
    // by a lithium cell: it holds the user's settings (display depth,
    // boot device, sound volume, AppleTalk node) across power cycles and
    // across every warm reset.  We model that here:
    //
    //   * The `initial` block below is the CONFIGURATION-TIME image —
    //     i.e. the state the array powers up in at bitstream load (or, in
    //     sim, at DUT construction).  That is the analogue of a freshly
    //     installed battery.
    //   * `rst` deliberately does NOT touch the array.  A warm reset of
    //     the machine must leave user settings intact, exactly like
    //     silicon.  (Everything else in this module — the shift FSM, the
    //     seconds counter, write_protect, test_mode — still resets.)
    //   * `pram_clear` is the explicit escape hatch (Cmd-Opt-P-R), the
    //     only input that rewrites the array from pram_reset_value().
    //
    // The image is deterministic and gives the ROM a small, stable PRAM
    // profile instead of X/host-dependent NVRAM contents.  The non-zero
    // bytes mirror the command-visible portion of a known-good Q700 PRAM
    // image; all other bytes start at zero and remain writable.
    (* ram_style = "block" *) reg [7:0] pram [0:255];
    integer idx;

    function [7:0] pram_reset_byte(input [7:0] addr);
        case (addr)
            8'h02: pram_reset_byte = 8'h4F;
            8'h03: pram_reset_byte = 8'h48;
            8'h08: pram_reset_byte = 8'h13;
            8'h09: pram_reset_byte = 8'h88;
            8'h0B: pram_reset_byte = 8'h4C;
            8'h0C: pram_reset_byte = 8'h4E;
            8'h0D: pram_reset_byte = 8'h75;
            8'h0E: pram_reset_byte = 8'h4D;
            8'h0F: pram_reset_byte = 8'h63;
            8'h10: pram_reset_byte = 8'hA8;
            8'h14: pram_reset_byte = 8'hCC;
            8'h15: pram_reset_byte = 8'h0A;
            8'h16: pram_reset_byte = 8'hCC;
            8'h17: pram_reset_byte = 8'h0A;
            8'h1D: pram_reset_byte = 8'h02;
            8'h1E: pram_reset_byte = 8'h63;
            8'h47: pram_reset_byte = 8'h33;
            8'h48: pram_reset_byte = 8'h80;
            8'h49: pram_reset_byte = 8'hC9;
            8'h4A: pram_reset_byte = 8'hC9;
            8'h4B: pram_reset_byte = 8'h06;
            8'h4C: pram_reset_byte = 8'h01;
            8'h77: pram_reset_byte = 8'h01;
            8'h78: pram_reset_byte = 8'hFF;
            8'h79: pram_reset_byte = 8'hFF;
            8'h7A: pram_reset_byte = 8'hFF;
            8'h7B: pram_reset_byte = 8'hDF;
            8'hF0: pram_reset_byte = 8'h86;
            8'hF1: pram_reset_byte = 8'h86;
            8'hF8: pram_reset_byte = 8'h53; // "SCBI" storage descriptor
            8'hF9: pram_reset_byte = 8'h43;
            8'hFA: pram_reset_byte = 8'h42;
            8'hFB: pram_reset_byte = 8'h49;
            8'hFF: pram_reset_byte = 8'h01;
            default: pram_reset_byte = 8'h00;
        endcase
    endfunction

	    function [7:0] pram_reset_value(input [7:0] addr);
	        begin
	            // Default-zero PRAM matches MAME's macqd700 boot path and is
	            // the silicon contract enforced for both sim and synthesis.
	            // Opt-in to the populated SCBI-magic image via
	            // +rtc_populated_pram (sim only — SYNTHESIS path always
	            // resets to zero so a real bitstream boots through the same
	            // ROM checksum trampoline as MAME instead of branching off
	            // at 0x40846CE6 into the storage-descriptor path).
`ifdef SYNTHESIS
	            pram_reset_value = 8'h00;
`else
	            pram_reset_value = mame_state_mode ? 8'h00 : pram_reset_byte(addr);
`endif
	        end
	    endfunction

    // Configuration-time / power-on image.  This is the ONLY unconditional
    // initialisation of the array — see the battery-backed note above.
    // Under SYNTHESIS pram_reset_value() is unconditionally 8'h00, so a
    // real bitstream still powers on with all-zero PRAM (unchanged
    // behaviour); the populated SCBI image is a sim-only opt-in.
    initial for (idx = 0; idx < 256; idx = idx + 1) pram[idx] = pram_reset_value(idx[7:0]);

    // No array reset: explicit clear walks port A independently of rst.
    reg clear_active = 1'b0;
    reg clear_q = 1'b0;
    reg [7:0] clear_addr = 8'h00;
    wire clear_start = pram_clear && !clear_q;
    assign pram_busy = pram_clear || clear_active;
    always @(posedge clk) begin
        clear_q <= pram_clear;
        if (clear_start) begin
            clear_active <= 1'b1;
            clear_addr <= 8'h00;
        end else if (clear_active) begin
            clear_addr <= clear_addr + 8'd1;
            if (clear_addr == 8'hff) clear_active <= 1'b0;
        end
    end

    // ── Bit-level shift FSM ───────────────────────────────────────────
    //
    // Edge detection on rtc_clk and rtc_enb.
    reg rtc_clk_q, rtc_enb_q;
    wire rtc_clk_rise = rtc_clk && !rtc_clk_q;
    wire rtc_clk_fall = !rtc_clk && rtc_clk_q;
    wire rtc_enb_fall = !rtc_enb && rtc_enb_q;
    wire rtc_enb_rise = rtc_enb && !rtc_enb_q;
    wire host_data_bit = rtc_data_oe ? rtc_data_o : 1'b1;
    wire drive_read_data = (state == S_DATA) && is_read;

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_CMD  = 2'd1;
    localparam [1:0] S_DATA = 2'd2;
    localparam [1:0] S_XP   = 2'd3;

    reg [1:0]  state;
    reg [3:0]  bit_cnt;      // counts 0..7
    reg [7:0]  cmd_byte;     // captured command
    reg [7:0]  shift_out;    // data to clock out on reads
    reg [7:0]  shift_in;     // data being clocked in on writes
    reg [7:0]  selected_idle_cnt;
    reg        is_read;      // latched from cmd_byte[7]
    reg        is_extended;  // extended PRAM command in progress
    reg        xp_addr_done; // extended PRAM address byte received
    reg [7:0]  xp_addr;      // extended PRAM byte address
    reg [7:0]  xp_addr_byte; // second command byte for extended PRAM

    function is_extended_cmd(input [7:0] c);
        is_extended_cmd = ((c & 8'h78) == 8'h38);
    endfunction

    function is_pram_reg(input [4:0] reg_idx);
        is_pram_reg = ((reg_idx >= 5'd8 && reg_idx <= 5'd11) ||
                       (reg_idx >= 5'd16));
    endfunction

    // Decode command address to seconds byte, PRAM byte, or control bit.
    // MAME macrtc.cpp indexes the 32-bit seconds register as bytes
    // 0..3, low byte first; command indexes 4..7 alias back to 0..3.
    // The 343-0042-B write-protect command accepts register 13 with any
    // low command bits (0x34..0x37 on writes); we provide deterministic
    // readback on the matching read commands for testability.
    function [7:0] read_byte_for_cmd(input [7:0] c);
        case (c[6:2])
            5'd0, 5'd4: read_byte_for_cmd = seconds[7:0];
            5'd1, 5'd5: read_byte_for_cmd = seconds[15:8];
            5'd2, 5'd6: read_byte_for_cmd = seconds[23:16];
            5'd3, 5'd7: read_byte_for_cmd = seconds[31:24];
            5'd12: read_byte_for_cmd = {test_mode, 7'b0};
            5'd13: read_byte_for_cmd = {write_protect, 7'b0};
            default: read_byte_for_cmd = 8'h00; // PRAM uses its synchronous port
        endcase
    endfunction

    function [7:0] seconds_index_for_cmd(input [7:0] c);
        seconds_index_for_cmd = {6'b0, c[3:2]};
    endfunction

    function [7:0] xp_addr_for(input [7:0] c, input [7:0] addr_byte);
        xp_addr_for = {c[2:0], addr_byte[6:2]};
    endfunction

    wire [7:0] incoming_cmd = {cmd_byte[6:0], host_data_bit};
    wire normal_pram_read = state == S_CMD && bit_cnt == 4'd7 &&
        cmd_byte[6] && !is_extended_cmd(incoming_cmd) &&
        is_pram_reg(incoming_cmd[6:2]);
    wire extended_pram_read = state == S_XP && is_extended &&
        !xp_addr_done && bit_cnt == 4'd7 && is_read;
    wire serial_read = !rst && !pram_busy && !rtc_enb && rtc_clk_fall &&
        (normal_pram_read || extended_pram_read);
    wire serial_write = !rst && !pram_busy && rtc_enb_rise &&
        !is_read && bit_cnt == 4'd8 &&
        ((state == S_DATA && !is_extended && !write_protect &&
          is_pram_reg(cmd_byte[6:2])) || (state == S_XP && is_extended));
    wire [7:0] serial_addr = serial_read ?
        (extended_pram_read ? xp_addr_for(cmd_byte, {xp_addr_byte[6:0], host_data_bit}) :
                             {3'b000, incoming_cmd[6:2]}) :
        (is_extended ? xp_addr : {3'b000, cmd_byte[6:2]});
    wire ext_write = pram_ext_we && !pram_busy;
    wire port_a_write = clear_active ||
        (serial_write && !(ext_write && serial_addr == pram_ext_addr));
    wire [7:0] port_a_addr = clear_active ? clear_addr : serial_addr;
    reg [7:0] serial_ram_data, ext_ram_data;
    reg serial_read_pending;

    // Common-clock READ_FIRST ports; inhibit simultaneous same-address writes.
    // Keep memory outputs separate from resettable protocol registers so Vivado
    // can absorb them into the block RAM. The slow serial wire hides the latency.
    always @(posedge clk) begin
        if (clear_active || serial_read || serial_write) begin
            if (port_a_write)
                pram[port_a_addr] <= clear_active ? pram_reset_value(clear_addr) : shift_in;
            serial_ram_data <= pram[port_a_addr];
        end
    end
    always @(posedge clk) begin
        if (ext_write) pram[pram_ext_addr] <= pram_ext_wdata;
        ext_ram_data <= pram[pram_ext_addr];
    end
    assign pram_ext_rdata = ext_ram_data;

    always @(posedge clk) begin
        if (rst) begin
            rtc_clk_q  <= 1'b0;
            rtc_enb_q  <= 1'b1;    // idle high
            state      <= S_IDLE;
            bit_cnt    <= 4'd0;
            cmd_byte   <= 8'h00;
            shift_out  <= 8'h00;
            shift_in   <= 8'h00;
            selected_idle_cnt <= 8'h00;
            is_read    <= 1'b0;
            is_extended <= 1'b0;
            xp_addr_done <= 1'b0;
            xp_addr     <= 8'h00;
            xp_addr_byte <= 8'h00;
            // MAME's rtc3430042 data_r() returns the device data_out latch.
            // That latch resets low and is cleared on chip-select edges.
            rtc_data_i <= 1'b0;
            seconds_wr_pending <= 1'b0;
            seconds_wr_idx     <= 2'd0;
            seconds_wr_data    <= 8'h00;
            write_protect      <= 1'b0;
            test_mode          <= 1'b0;
            serial_read_pending <= 1'b0;
            // NOTE: `pram` is intentionally NOT reset here.  It models the
            // battery-backed PRAM of a real Macintosh RTC IC, so user
            // settings (display depth, boot device, volume, AppleTalk
            // node) must survive a warm reset the way they do in silicon.
            // Its power-on image comes from the configuration-time
            // `initial` above; the only way to clear it is the explicit
            // `pram_clear` sweep handled independently of this block.
        end else begin
            serial_read_pending <= serial_read;
            if (serial_read_pending) shift_out <= serial_ram_data;
`ifndef SYNTHESIS
            if (serial_read_pending && trace_mode)
                $display("rtc: PRAM read data=0x%02x", serial_ram_data);
`endif
            rtc_clk_q <= rtc_clk;
            rtc_enb_q <= rtc_enb;
            seconds_wr_pending <= 1'b0;

            if (rtc_enb || rtc_clk_rise || rtc_clk_fall) begin
                selected_idle_cnt <= 8'h00;
            end else if (selected_idle_cnt != 8'hff) begin
                selected_idle_cnt <= selected_idle_cnt + 8'd1;
            end
            if (!rtc_enb && selected_idle_cnt == 8'hff && state != S_DATA) begin
                state   <= S_IDLE;
                bit_cnt <= 4'd0;
            end

            // Transaction start: rtcEnb falling edge.
            if (rtc_enb_fall) begin
                state    <= S_CMD;
                bit_cnt  <= 4'd0;
                cmd_byte <= 8'h00;
                shift_in <= 8'h00;
                is_read  <= 1'b0;
                is_extended <= 1'b0;
                xp_addr_done <= 1'b0;
                xp_addr  <= 8'h00;
                xp_addr_byte <= 8'h00;
                selected_idle_cnt <= 8'h00;
                rtc_data_i <= 1'b0;
            end

            // Transaction end: rtcEnb rising edge.  If the transaction
            // was a write with a complete data byte, commit it.
            if (rtc_enb_rise) begin
                if (state == S_DATA && !is_read && !is_extended && bit_cnt == 4'd8) begin
                    if (cmd_byte[6:2] == 5'd13) begin
                        write_protect <= shift_in[7];
`ifndef SYNTHESIS
                        if (trace_mode)
                            $display("rtc: write control wp cmd=0x%02x data=0x%02x wp=%0d",
                                     cmd_byte, shift_in, shift_in[7]);
`endif
                    end else if (!write_protect) begin
                        if (cmd_byte[6:2] == 5'd12) begin
                            // MAME's test bit is a latch, not a clock reset.
                            test_mode <= shift_in[7];
`ifndef SYNTHESIS
                            if (trace_mode)
                                $display("rtc: write control test cmd=0x%02x data=0x%02x test=%0d",
                                         cmd_byte, shift_in, shift_in[7]);
`endif
                        end else if (cmd_byte[6:2] < 5'd8) begin
                            seconds_wr_pending <= 1'b1;
                            seconds_wr_idx     <= cmd_byte[3:2];
                            seconds_wr_data    <= shift_in;
`ifndef SYNTHESIS
                            if (trace_mode)
                                $display("rtc: write seconds cmd=0x%02x idx=%0d data=0x%02x",
                                         cmd_byte, seconds_index_for_cmd(cmd_byte), shift_in);
`endif
                        end else if (is_pram_reg(cmd_byte[6:2])) begin
`ifndef SYNTHESIS
                            if (trace_mode)
                                $display("rtc: write pram cmd=0x%02x addr=0x%02x data=0x%02x",
                                         cmd_byte, {3'b000, cmd_byte[6:2]}, shift_in);
`endif
                        end
                    end else begin
`ifndef SYNTHESIS
                        if (trace_mode)
                            $display("rtc: blocked write cmd=0x%02x data=0x%02x", cmd_byte, shift_in);
`endif
                    end
                end else if (state == S_XP && !is_read && is_extended && bit_cnt == 4'd8) begin
                    // MAME macrtc.cpp:268-273 (RTC_STATE_XPWRITE branch)
                    // commits the byte UNCONDITIONALLY — extended-PRAM
                    // writes are NOT gated by the write-protect bit.  The
                    // WP check at macrtc.cpp:280 lives inside the
                    // RTC_STATE_WRITE branch only.  Mac OS parks network
                    // / AppleTalk settings in extended PRAM even with WP
                    // engaged, so gating here would diverge from real
                    // silicon and from MAME.
`ifndef SYNTHESIS
                    if (trace_mode)
                        $display("rtc: write xpram cmd=0x%02x addr=0x%02x data=0x%02x",
                                 cmd_byte, xp_addr, shift_in);
`endif
                end
                state   <= S_IDLE;
                bit_cnt <= 4'd0;
                rtc_data_i <= 1'b0;
            end

            // Clock in from CPU (command and write-data) on falling edge.
            // The Q700 ROM can leave rtcEnb low while idle before starting
            // the PRAM helper, so there may be no clean select falling edge.
            // In that stale-selected case, capture only the first command bit
            // on the first rising clock edge, then resume normal falling-edge
            // sampling for the rest of the transaction.
            if (!rtc_enb && state == S_IDLE &&
                selected_idle_cnt == 8'hff && rtc_clk_rise) begin
                state    <= S_CMD;
                bit_cnt  <= 4'd1;
                cmd_byte <= {7'h00, host_data_bit};
                shift_in <= 8'h00;
                is_read  <= 1'b0;
                is_extended <= 1'b0;
                xp_addr_done <= 1'b0;
                xp_addr  <= 8'h00;
                xp_addr_byte <= 8'h00;
                rtc_data_i <= 1'b0;
            end else if (rtc_clk_fall && !rtc_enb) begin
                if (state == S_IDLE) begin
                    state    <= S_CMD;
                    bit_cnt  <= 4'd1;
                    cmd_byte <= {7'h00, host_data_bit};
                    shift_in <= 8'h00;
                    is_read  <= 1'b0;
                    is_extended <= 1'b0;
                    xp_addr_done <= 1'b0;
                    xp_addr  <= 8'h00;
                    xp_addr_byte <= 8'h00;
                    rtc_data_i <= 1'b0;
                end else if (state == S_CMD) begin
                    cmd_byte <= {cmd_byte[6:0], host_data_bit};
                    if (bit_cnt == 4'd7) begin
                        // Command byte now in cmd_byte (shifted this cycle).
                        // MSB of {cmd_byte[6:0], host_data_bit} == cmd_byte[6].
                        // Hoisted out of concat-index for Vivado XST compat.
                        is_read <= cmd_byte[6];
                        bit_cnt <= 4'd0;
                        is_extended <= is_extended_cmd({cmd_byte[6:0], host_data_bit});
                        if (is_extended_cmd({cmd_byte[6:0], host_data_bit})) begin
                            state <= S_XP;
                            shift_in <= 8'h00;
`ifndef SYNTHESIS
                            if (trace_mode)
                                $display("rtc: cmd xpram_%s cmd=0x%02x sector=%0d",
                                         cmd_byte[6] ? "read" : "write",
                                         {cmd_byte[6:0], host_data_bit},
                                         {cmd_byte[1:0], host_data_bit});
`endif
                        end else begin
                            state   <= S_DATA;
                            // Read: pre-load shift_out with the answer.
                            if (cmd_byte[6]) begin
                                shift_out <= read_byte_for_cmd({cmd_byte[6:0], host_data_bit});
`ifndef SYNTHESIS
                                if (trace_mode && !normal_pram_read)
                                    $display("rtc: cmd read cmd=0x%02x reg=%0d data=0x%02x",
                                             {cmd_byte[6:0], host_data_bit},
                                             (({cmd_byte[6:0], host_data_bit} >> 2) & 8'h1f),
                                             read_byte_for_cmd({cmd_byte[6:0], host_data_bit}));
`endif
                            end
                        end
                    end else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end else if (state == S_DATA && !is_read) begin
                    // Write data from CPU
                    shift_in <= {shift_in[6:0], host_data_bit};
                    if (bit_cnt < 4'd8)
                        bit_cnt <= bit_cnt + 4'd1;
                end else if (state == S_XP && is_extended && !xp_addr_done) begin
                    xp_addr_byte <= {xp_addr_byte[6:0], host_data_bit};
                    if (bit_cnt == 4'd7) begin
                        xp_addr <= xp_addr_for(cmd_byte, {xp_addr_byte[6:0], host_data_bit});
                        xp_addr_done <= 1'b1;
                        bit_cnt <= 4'd0;
                        if (is_read) begin
                            // PRAM read completes through serial_read_pending.
                            state <= S_DATA;
`ifndef SYNTHESIS
                            if (trace_mode)
                                $display("rtc: read xpram cmd=0x%02x addr=0x%02x",
                                         cmd_byte,
                                         xp_addr_for(cmd_byte, {xp_addr_byte[6:0], host_data_bit}));
`endif
                        end else begin
                            shift_in <= 8'h00;
                        end
                    end else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end else if (state == S_XP && is_extended && xp_addr_done && !is_read) begin
                    shift_in <= {shift_in[6:0], host_data_bit};
                    if (bit_cnt < 4'd8)
                        bit_cnt <= bit_cnt + 4'd1;
                end
            end

            // Clock out to CPU on falling edge; host samples after the
            // transition, matching MAME macrtc.cpp and the 343-0042 docs.
            if (rtc_clk_fall && !rtc_enb && state == S_DATA && is_read) begin
                rtc_data_i <= shift_out[7];
                shift_out  <= {shift_out[6:0], 1'b0};
                if (bit_cnt < 4'd8)
                    bit_cnt <= bit_cnt + 4'd1;
            end

            // When CPU is driving the line (DDRB[0]=1) we mirror it back
            // so the VIA read path is self-consistent (host usually only
            // listens while it tristates PB0 — DDRB[0]=0).
            if (rtc_data_oe && !drive_read_data)
                rtc_data_i <= rtc_data_o;
            else if (state != S_DATA || !is_read)
                rtc_data_i <= 1'b0;
            // Clear aborts the serial transaction, not the seconds counter or
            // control registers. The host must start a new transaction afterward.
            if (pram_busy) begin
                state <= S_IDLE;
                bit_cnt <= 4'd0;
                selected_idle_cnt <= 8'h00;
                serial_read_pending <= 1'b0;
                rtc_data_i <= 1'b0;
            end
        end
    end

endmodule
