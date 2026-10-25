// Convert successive USB boot-keyboard snapshots into ADB make/break events.
// The caller crosses complete reports into clk first, then holds report_valid
// until report_ready. Event valid/data remain stable until event_ready.
// A disconnect is a report with connected=0, releasing every held key.
`default_nettype none
module usb_hid_adb_keyboard (
    input  wire        clk,
    input  wire        rst,
    input  wire        report_valid,
    output wire        report_ready,
    input  wire        connected,
    input  wire [7:0]  modifiers,
    input  wire [47:0] usages,
    output reg         event_valid,
    output reg  [7:0]  event_byte,
    input  wire        event_ready
);
    reg active;
    reg [1:0] phase;
    reg [2:0] slot;
    reg [7:0] old_mod, new_mod;
    reg [47:0] old_keys, new_keys;
    assign report_ready = !active;

    function [7:0] key_at;
        input [47:0] keys;
        input [2:0] index;
        begin
            case (index)
                3'd0: key_at = keys[47:40];
                3'd1: key_at = keys[39:32];
                3'd2: key_at = keys[31:24];
                3'd3: key_at = keys[23:16];
                3'd4: key_at = keys[15:8];
                default: key_at = keys[7:0];
            endcase
        end
    endfunction

    function contains_key;
        input [47:0] keys;
        input [7:0] usage;
        integer k;
        begin
            contains_key = 1'b0;
            for (k = 0; k < 6; k = k + 1)
                if (key_at(keys, k[2:0]) == usage) contains_key = 1'b1;
        end
    endfunction

    function earlier_key;
        input [47:0] keys;
        input [2:0] index;
        input [7:0] usage;
        integer k;
        begin
            earlier_key = 1'b0;
            for (k = 0; k < 6; k = k + 1)
                if (k < index && key_at(keys, k[2:0]) == usage)
                    earlier_key = 1'b1;
        end
    endfunction

    function rollover;
        input [47:0] keys;
        integer k;
        begin
            rollover = 1'b0;
            for (k = 0; k < 6; k = k + 1)
                if (key_at(keys, k[2:0]) >= 8'h01 &&
                    key_at(keys, k[2:0]) <= 8'h03) rollover = 1'b1;
        end
    endfunction

    // Standard ANSI Mac virtual keycodes, matching the existing web/JTAG
    // ADB injector. 0xFF marks HID usages this keyboard does not emit.
    function [7:0] adb_code;
        input [7:0] usage;
        begin
            case (usage)
                8'h04: adb_code=8'h00; 8'h05: adb_code=8'h0b; // A B
                8'h06: adb_code=8'h08; 8'h07: adb_code=8'h02; // C D
                8'h08: adb_code=8'h0e; 8'h09: adb_code=8'h03; // E F
                8'h0a: adb_code=8'h05; 8'h0b: adb_code=8'h04; // G H
                8'h0c: adb_code=8'h22; 8'h0d: adb_code=8'h26; // I J
                8'h0e: adb_code=8'h28; 8'h0f: adb_code=8'h25; // K L
                8'h10: adb_code=8'h2e; 8'h11: adb_code=8'h2d; // M N
                8'h12: adb_code=8'h1f; 8'h13: adb_code=8'h23; // O P
                8'h14: adb_code=8'h0c; 8'h15: adb_code=8'h0f; // Q R
                8'h16: adb_code=8'h01; 8'h17: adb_code=8'h11; // S T
                8'h18: adb_code=8'h20; 8'h19: adb_code=8'h09; // U V
                8'h1a: adb_code=8'h0d; 8'h1b: adb_code=8'h07; // W X
                8'h1c: adb_code=8'h10; 8'h1d: adb_code=8'h06; // Y Z
                8'h1e: adb_code=8'h12; 8'h1f: adb_code=8'h13; // 1 2
                8'h20: adb_code=8'h14; 8'h21: adb_code=8'h15; // 3 4
                8'h22: adb_code=8'h17; 8'h23: adb_code=8'h16; // 5 6
                8'h24: adb_code=8'h1a; 8'h25: adb_code=8'h1c; // 7 8
                8'h26: adb_code=8'h19; 8'h27: adb_code=8'h1d; // 9 0
                8'h28: adb_code=8'h24; 8'h29: adb_code=8'h35; // Return Esc
                8'h2a: adb_code=8'h33; 8'h2b: adb_code=8'h30; // Delete Tab
                8'h2c: adb_code=8'h31; 8'h2d: adb_code=8'h1b; // Space -
                8'h2e: adb_code=8'h18; 8'h2f: adb_code=8'h21; // = [
                8'h30: adb_code=8'h1e; 8'h31: adb_code=8'h2a; // ] backslash
                8'h33: adb_code=8'h29; 8'h34: adb_code=8'h27; // ; '
                8'h35: adb_code=8'h32; 8'h36: adb_code=8'h2b; // ` ,
                8'h37: adb_code=8'h2f; 8'h38: adb_code=8'h2c; // . /
                8'h39: adb_code=8'h39; // Caps Lock
                8'h4f: adb_code=8'h7c; 8'h50: adb_code=8'h7b; // right left
                8'h51: adb_code=8'h7d; 8'h52: adb_code=8'h7e; // down up
                default: adb_code=8'hff;
            endcase
        end
    endfunction

    function [7:0] modifier_code;
        input [2:0] index;
        begin
            case (index)
                3'd0: modifier_code=8'h3b; // left control
                3'd1: modifier_code=8'h38; // left shift
                3'd2: modifier_code=8'h3a; // left option
                3'd3: modifier_code=8'h37; // left command
                3'd4: modifier_code=8'h3e; // right control
                3'd5: modifier_code=8'h3c; // right shift
                3'd6: modifier_code=8'h3d; // right option
                default: modifier_code=8'h36; // right command
            endcase
        end
    endfunction

    reg [7:0] usage, code;
    always @(*) begin
        usage = 8'h00;
        code = 8'hff;
        event_valid = 1'b0;
        event_byte = 8'h00;
        if (active) begin
            case (phase)
                2'd0: begin // release keys absent from the new snapshot
                    usage = key_at(old_keys, slot);
                    code = adb_code(usage);
                    event_valid = usage >= 8'h04 && code != 8'hff &&
                                  !contains_key(new_keys, usage) &&
                                  !earlier_key(old_keys, slot, usage);
                    event_byte = {1'b1, code[6:0]};
                end
                2'd1: begin // release modifiers before pressing new ones
                    event_valid = old_mod[slot] && !new_mod[slot];
                    event_byte = 8'h80 | modifier_code(slot);
                end
                2'd2: begin
                    event_valid = new_mod[slot] && !old_mod[slot];
                    event_byte = modifier_code(slot);
                end
                default: begin // press newly present keys
                    usage = key_at(new_keys, slot);
                    code = adb_code(usage);
                    event_valid = usage >= 8'h04 && code != 8'hff &&
                                  !contains_key(old_keys, usage) &&
                                  !earlier_key(new_keys, slot, usage);
                    event_byte = code;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            active <= 1'b0;
            phase <= 2'd0;
            slot <= 3'd0;
            old_mod <= 8'd0;
            new_mod <= 8'd0;
            old_keys <= 48'd0;
            new_keys <= 48'd0;
        end else if (!active) begin
            if (report_valid && (!connected || !rollover(usages))) begin
                new_mod <= connected ? modifiers : 8'd0;
                new_keys <= connected ? usages : 48'd0;
                phase <= 2'd0;
                slot <= 3'd0;
                active <= 1'b1;
            end
        end else if (!event_valid || event_ready) begin
            if (slot == ((phase == 2'd1 || phase == 2'd2) ? 3'd7 : 3'd5)) begin
                slot <= 3'd0;
                if (phase == 2'd3) begin
                    old_mod <= new_mod;
                    old_keys <= new_keys;
                    active <= 1'b0;
                end else phase <= phase + 2'd1;
            end else slot <= slot + 3'd1;
        end
    end
endmodule
`default_nettype wire
