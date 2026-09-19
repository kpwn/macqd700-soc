// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// adb_pic_modem.v - Quadra 700 ADB modem PIC wrapper
//
// Wraps the PIC1654S core (pic16c5x.v) running the real 342s0440-b
// firmware.  Port mapping mirrors MAME's adbmodem_device exactly
// (src/mame/apple/adbmodem.cpp):
//
//   portb_w: RB2 -> VIA1 CB1 (clock), RB3 -> VIA1 CB2 (data),
//            RB4 -> ADB-IRQ (inverted, write_irq(BIT^1)).
//   portb_r: returns m_via_data << 3 — ONLY bit 3 carries the VIA1
//            CB2 data; every other bit reads 0.
//   porta_r: returns (m_via_state & 3) | (m_adb_in << 3) — PORTA[1:0]
//            is the 68k's ADB state command (VIA1 ORB[5:4]), PORTA[3]
//            is the ADB bus line; all other bits read 0.
//   porta_w: RA2 ^ 1 drives the ADB bus open-drain line.

module adb_pic_modem (
    input  wire clk,
    input  wire rst,
    input  wire phi2_tick,
    output wire cb1_out,
    output wire cb2_out,
    output wire cb2_oe,
    input  wire cb2_in,
    input  wire adb_bus_in,
    input  wire rtcc_in,
    output wire pic_adb_out,
    // ADB state command from the 68k — VIA1 ORB[5:4].  MAME's
    // macquadra700 via_out_b does set_via_state((ORB & 0x30) >> 4);
    // the PIC reads it on PORTA[1:0] (adbmodem.cpp porta_r).
    input  wire [1:0] adb_state,
    output wire adb_irq_pending
);
    wire [7:0] porta_out;
    wire [7:0] portb_out;
    wire [7:0] porta_dir;
    wire [7:0] portb_dir;
    wire [8:0] dbg_pc;
    wire [7:0] dbg_w;
    wire       cyc_done;

    // PORTA: bits[1:0] = ADB state from the 68k; bit[3] = ADB bus
    // input from the ADB PHY; all other bits 0.  Matches
    // adbmodem.cpp porta_r = (m_via_state & 3) | (m_adb_in << 3).
    wire [7:0] porta_in = {4'b0000, adb_bus_in, 1'b0, adb_state};
    // PORTB: only bit 3 carries the VIA1 CB2 data; rest 0.  Matches
    // adbmodem.cpp portb_r = (m_via_data << 3).
    wire [7:0] portb_in = {4'b0000, cb2_in, 3'b000};

    pic16c5x #(
        .PROGHEX("rtl/mac/adb_pic_fw.hex")
    ) u_pic16c5x (
        .clk      (clk),
        .rst      (rst),
        .cyc_en   (phi2_tick),
        .rtcc_in  (rtcc_in),
        .porta_in (porta_in),
        .portb_in (portb_in),
        .porta_out(porta_out),
        .portb_out(portb_out),
        .porta_dir(porta_dir),
        .portb_dir(portb_dir),
        .dbg_pc   (dbg_pc),
        .dbg_w    (dbg_w),
        .cyc_done (cyc_done)
    );

    // PIC1654S I/O is open-drain (TRIS is not used by the chip; the ADB
    // firmware never executes a TRIS instruction either — see
    // pic16c5x.v port-read comment).  Pins are pulled LOW when the
    // latch bit is 0 and float (pulled high externally) when the latch
    // bit is 1.  Model this by driving 0 on the pin only while the
    // latch is 0; let the shared CB2 wire idle-high otherwise.
    //   * CB1 is unidirectional PIC→via1, so the open-drain effect is
    //     transparent: the latch value goes onto the wire directly.
    //   * CB2 is bidirectional — via1 drives during shift-out, PIC
    //     drives during shift-in.  The shared mux in
    //     fpga_top_peripherals.vh prefers via1 when via1_cb2_oe=1,
    //     else honours pic_cb2_oe.
    //   * RB4 is the IRQ to via1 PB3 (inverted) — same open-drain
    //     latch wiring; MAME passes BIT(latch,4)^1 to write_irq with
    //     no TRIS gating.
    assign cb1_out         = portb_out[2];
    assign cb2_out         = 1'b0;
    assign cb2_oe          = ~portb_out[3];
    assign adb_irq_pending = ~portb_out[4];
    assign pic_adb_out     = ~porta_out[2];

    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] _unused_porta_out = porta_out;
    wire [7:0] _unused_porta_dir = porta_dir;
    wire [7:0] _unused_portb_dir = portb_dir;
    wire [8:0] _unused_dbg_pc    = dbg_pc;
    wire [7:0] _unused_dbg_w     = dbg_w;
    wire       _unused_cyc_done  = cyc_done;
    /* verilator lint_on UNUSEDSIGNAL */
endmodule
