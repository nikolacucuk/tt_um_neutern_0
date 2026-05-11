`default_nettype none


`ifndef YOSYS
import tile_pkg::*;
`endif
module neuron_exec
  (
    input  wire execute_valid,
    input  wire [TAG_W-1:0]             exec_tag,
    input  wire [RF_FLAT_W-1:0]         rf_state_flat,
    input  wire signed [WEIGHT_W-1:0]   current_weight_value,
    input  wire [15:0]                  instr_word,
    input  wire [TAG_W-1:0]             last_tag_r,
    input  wire cmp_ge_r,
    input  wire cmp_eq_r,
    input  wire spike_flag_r,
    input  wire emit_pending_in,
    input  wire [DATA_W-1:0]            emit_data_in,
    output logic [RF_FLAT_W-1:0]        rf_next_flat,
    output logic [TAG_W-1:0]            last_tag_next,
    output logic cmp_ge_next,
    output logic cmp_eq_next,
    output logic spike_flag_next,
    output logic emit_pending_next,
    output logic [DATA_W-1:0]           emit_data_next
);
    // RF_COUNT and OP_* opcodes are imported from tile_pkg via tile_flit_types.vh.

    integer rf_idx;

    logic signed [7:0] tmp_rf [0:RF_COUNT-1];
    logic [4:0] instr_op;
    logic [2:0] instr_rd;
    logic [2:0] instr_k;
    logic signed [7:0] instr_imm4;
    // Tag gating: when rb[0]=1 (instr_word[2]) and exec_tag=0 (data event),
    // the instruction is skipped.  Barrier events (tag=1) trigger
    // integrate/threshold/emit.
    wire tag_gate_skip_c = instr_word[2] && !exec_tag;
    logic signed [8:0] add_tmp;  // shared 9b arithmetic path for all sat_i8 uses

    function automatic signed [7:0] sat_i8(input signed [8:0] value);
        begin
            if (value > 9'sd127) begin
                sat_i8 = 8'sd127;
            end else if (value < -9'sd128) begin
                sat_i8 = -8'sd128;
            end else begin
                sat_i8 = value[7:0];
            end
        end
    endfunction

    function automatic signed [7:0] imm4_from_fields(
        input sign_bit,
        input [2:0] k_bits
    );
        logic [3:0] raw;
        begin
            raw = {sign_bit, k_bits};
            imm4_from_fields = {{4{raw[3]}}, raw};
        end
    endfunction

    // sat_reg: saturate 9-bit signed arithmetic result to RF_REG_W-bit signed range.
    // Ensures write-back values fit in the compact 4-bit context registers.
    function automatic signed [7:0] sat_reg(input signed [8:0] value);
        localparam signed [8:0] REG_MAX = (9'(1) << (RF_REG_W - 1)) - 9'sd1;
        localparam signed [8:0] REG_MIN = -(9'(1) << (RF_REG_W - 1));
        begin
            if (value > REG_MAX) sat_reg = 8'(REG_MAX);
            else if (value < REG_MIN) sat_reg = 8'(REG_MIN);
            else sat_reg = value[7:0];
        end
    endfunction

    // pack_emit_data: pack spike status and tag hint into DATA_W-bit emit payload.
    // Layout: [DATA_W-1]=1 (valid marker), [DATA_W-2]=spike_flag, [1:0]=tag[1:0]
    function automatic [DATA_W-1:0] pack_emit_data(
        input [1:0] tag,
        input spike
    );
        begin
            pack_emit_data = {1'b1, spike, tag[0], 1'b0};
        end
    endfunction

    always_comb begin
        // Sign-extend each RF_REG_W-bit register slot to 8-bit for the shared ALU.
        for (rf_idx = 0; rf_idx < RF_COUNT; rf_idx = rf_idx + 1) begin
            tmp_rf[rf_idx] = {{(8-RF_REG_W){rf_state_flat[rf_idx*RF_REG_W + RF_REG_W-1]}},
                              rf_state_flat[rf_idx*RF_REG_W +: RF_REG_W]};
        end

        rf_next_flat = rf_state_flat;
        instr_op = OP_LDI;
        instr_rd = 3'd0;
        instr_k = 3'd0;
        instr_imm4 = 8'sd0;
        add_tmp = 9'sd0;

        last_tag_next = last_tag_r;
        cmp_ge_next = cmp_ge_r;
        cmp_eq_next = cmp_eq_r;
        spike_flag_next = spike_flag_r;
        emit_pending_next = emit_pending_in;
        emit_data_next = emit_data_in;
        if (execute_valid && !tag_gate_skip_c) begin
            // Compact 12-bit instruction format: [11:7]=op, [6:4]=rd, [3]=sign, [2:0]=k
            instr_op = instr_word[11:7];
            instr_rd = instr_word[6:4];
            instr_k = instr_word[2:0];
            instr_imm4 = imm4_from_fields(instr_word[3], instr_k);

            case (instr_op)
                OP_LDI: begin
                    if (instr_rd < 3'(RF_COUNT)) tmp_rf[instr_rd] = instr_imm4;
                end
                OP_RECV: begin
                    last_tag_next = exec_tag;
                end
                OP_ACCUM_W: begin
                    // Sign-extend weight to 9 bits; shared sat path.
                    add_tmp = $signed(tmp_rf[1]) + 9'(signed'(current_weight_value));
                    tmp_rf[1] = sat_i8(add_tmp);
                end
                // 6'd3 reserved (OP_LEAK removed — no timestamps)
                OP_INTEG: begin
                    add_tmp = $signed(tmp_rf[0]) + $signed(tmp_rf[1]);
                    tmp_rf[0] = sat_i8(add_tmp);
                    if (instr_k[2]) tmp_rf[1] = 8'sd0; // auto-clear accumulator after integration
                end
                OP_SPIKE_IF_GE: begin
                    spike_flag_next = ($signed(tmp_rf[0]) >= $signed(tmp_rf[2]));
                end
                OP_RESET: begin
                    if (spike_flag_next) begin
                        case (instr_k[1:0])
                            2'b00, 2'b11: tmp_rf[0] = 8'sd0;
                            2'b01: begin
                                add_tmp = $signed(tmp_rf[0]) - $signed(tmp_rf[2]);
                                tmp_rf[0] = sat_i8(add_tmp);
                            end
                            2'b10: begin
                                if ($signed(tmp_rf[0]) > $signed(tmp_rf[2])) begin
                                    tmp_rf[0] = tmp_rf[2];
                                end
                            end
                        endcase
                    end
                end
                // 6'd7 reserved (OP_REFRACT removed with rf[3])
                OP_EMIT: begin
                    if (!emit_pending_next) begin
                        emit_pending_next = 1'b1;
                        emit_data_next = pack_emit_data(instr_k[1:0], spike_flag_next);
                    end
                end
                default: begin
                end
            endcase
        end

        // Write-back: saturate 8-bit ALU result to RF_REG_W-bit range and pack.
        // Use a temporary to avoid function-return indexing (Yosys/formal compat).
        begin : writeback
            logic signed [7:0] sat_tmp;
            for (rf_idx = 0; rf_idx < RF_COUNT; rf_idx = rf_idx + 1) begin
                sat_tmp = sat_reg({tmp_rf[rf_idx][7], tmp_rf[rf_idx]});
                rf_next_flat[rf_idx*RF_REG_W +: RF_REG_W] = sat_tmp[RF_REG_W-1:0];
            end
        end
    end
endmodule

`default_nettype wire
