// SPDX-License-Identifier: Apache-2.0
`default_nettype none
`include "tile_flit_types.vh"

// =============================================================================
// neuron_exec_formal.sv
//
// Comprehensive formal harness for neuron_exec (purely combinational ISA unit).
//
// Assertion style: always @(*) with assert()/cover() inside, matching the
// verified Yosys 0.64 pattern from logical_neuron_context_bank_formal.
//
// Tasks (one ifdef per SBY task):
//   FORMAL_IDLE_HOLD       -- A1: execute_valid=0 -> pure pass-through
//   FORMAL_TAG_GATE_SKIP   -- A2: tag gate suppresses data events
//   FORMAL_ACCUM_SHADOW    -- A3: OP_ACCUM_W saturated accumulation
//   FORMAL_INTEG_VM        -- A4/A5: OP_INTEG vm integration + accum clear
//   FORMAL_SPIKE_FLAG      -- A6: OP_SPIKE_IF_GE threshold comparison
//   FORMAL_RESET_MODES     -- A7-A9: OP_RESET all four modes
//   FORMAL_EMIT_ONE_SHOT   -- A10/A11: OP_EMIT one-shot + format
//   FORMAL_LDI_LOAD        -- A12: OP_LDI register load and isolation
//   FORMAL_RECV_TAG        -- A13: OP_RECV tag capture
// =============================================================================

module neuron_exec_formal;

    // -- Concrete widths from tile_pkg (imported via tile_flit_types.vh) ------
    localparam integer RFW  = RF_REG_W;   // 4
    localparam integer RFC  = RF_COUNT;   // 3
    localparam integer RFFW = RF_FLAT_W;  // 12
    localparam integer TWG  = TAG_W;      // 1
    localparam integer WTW  = WEIGHT_W;   // 4
    localparam integer DTW  = DATA_W;     // 4

    // 5-bit opcode values extracted from instr_word[11:7]
    localparam [4:0] OP5_LDI   = OP_LDI[4:0];
    localparam [4:0] OP5_RECV  = OP_RECV[4:0];
    localparam [4:0] OP5_ACCUM = OP_ACCUM_W[4:0];
    localparam [4:0] OP5_INTEG = OP_INTEG[4:0];
    localparam [4:0] OP5_SPIKE = OP_SPIKE_IF_GE[4:0];
    localparam [4:0] OP5_RESET = OP_RESET[4:0];
    localparam [4:0] OP5_EMIT  = OP_EMIT[4:0];

    // -- Free inputs to DUT ---------------------------------------------------
    logic                   execute_valid;
    logic [0:0]             exec_tag;
    logic [RFFW-1:0]        rf_state_flat;
    logic signed [WTW-1:0]  current_weight_value;
    logic [15:0]             instr_word;
    logic [0:0]             last_tag_r;
    logic                   cmp_ge_r, cmp_eq_r, spike_flag_r;
    logic                   emit_pending_in;
    logic [DTW-1:0]         emit_data_in;

    // -- DUT outputs ----------------------------------------------------------
    logic [RFFW-1:0]        rf_next_flat;
    logic [0:0]             last_tag_next;
    logic                   cmp_ge_next, cmp_eq_next, spike_flag_next;
    logic                   emit_pending_next;
    logic [DTW-1:0]         emit_data_next;

    // -- DUT ------------------------------------------------------------------
    neuron_exec dut (
        .execute_valid        (execute_valid),
        .exec_tag             (exec_tag),
        .rf_state_flat        (rf_state_flat),
        .current_weight_value (current_weight_value),
        .instr_word           (instr_word),
        .last_tag_r           (last_tag_r),
        .cmp_ge_r             (cmp_ge_r),
        .cmp_eq_r             (cmp_eq_r),
        .spike_flag_r         (spike_flag_r),
        .emit_pending_in      (emit_pending_in),
        .emit_data_in         (emit_data_in),
        .rf_next_flat         (rf_next_flat),
        .last_tag_next        (last_tag_next),
        .cmp_ge_next          (cmp_ge_next),
        .cmp_eq_next          (cmp_eq_next),
        .spike_flag_next      (spike_flag_next),
        .emit_pending_next    (emit_pending_next),
        .emit_data_next       (emit_data_next)
    );

    // -- Decode fields -------------------------------------------------------
    wire [4:0] op5           = instr_word[11:7];
    wire [2:0] rd            = instr_word[6:4];
    wire [2:0] k             = instr_word[2:0];
    wire [7:0] imm4          = {{4{instr_word[3]}}, instr_word[2:0]};
    wire       tag_gate_skip = instr_word[2] & ~exec_tag[0];
    wire       real_execute  = execute_valid & ~tag_gate_skip;

    // -- 9-bit sign-extended inputs -------------------------------------------
    wire [8:0] r0_ext = {{5{rf_state_flat[0*RFW+RFW-1]}}, rf_state_flat[0*RFW +: RFW]};
    wire [8:0] r1_ext = {{5{rf_state_flat[1*RFW+RFW-1]}}, rf_state_flat[1*RFW +: RFW]};
    wire [8:0] r2_ext = {{5{rf_state_flat[2*RFW+RFW-1]}}, rf_state_flat[2*RFW +: RFW]};
    wire [8:0] w_ext  = {{5{current_weight_value[WTW-1]}},  current_weight_value};

    // -- 9-bit arithmetic sums ------------------------------------------------
    wire [8:0] accum_sum9 = $signed(r1_ext) + $signed(w_ext);
    wire [8:0] integ_sum9 = $signed(r0_ext) + $signed(r1_ext);
    wire [8:0] sub_sum9   = $signed(r0_ext) - $signed(r2_ext);

    // -- Saturated 4-bit expected results (signed saturation to [-8..+7]) ----
    wire [RFW-1:0] exp_accum_sat = ($signed(accum_sum9) > $signed(9'd7))   ? 4'h7 :
                                   ($signed(accum_sum9) < $signed(-9'd8))  ? 4'h8 :
                                   accum_sum9[RFW-1:0];
    wire [RFW-1:0] exp_integ_sat = ($signed(integ_sum9) > $signed(9'd7))   ? 4'h7 :
                                   ($signed(integ_sum9) < $signed(-9'd8))  ? 4'h8 :
                                   integ_sum9[RFW-1:0];
    wire [RFW-1:0] exp_sub_sat   = ($signed(sub_sum9)   > $signed(9'd7))   ? 4'h7 :
                                   ($signed(sub_sum9)   < $signed(-9'd8))  ? 4'h8 :
                                   sub_sum9[RFW-1:0];
    wire           exp_spike      = ($signed(r0_ext) >= $signed(r2_ext)) ? 1'b1 : 1'b0;
    wire [RFW-1:0] exp_clamp_vm  = ($signed(r0_ext) >  $signed(r2_ext))
                                   ? rf_state_flat[2*RFW +: RFW]
                                   : rf_state_flat[0*RFW +: RFW];

    // =========================================================================
    // A1: execute_valid=0 -> pure pass-through
    // =========================================================================
`ifdef FORMAL_IDLE_HOLD
    always @(*) begin
        if (!execute_valid) begin
            a_idle_rf:        assert (rf_next_flat == rf_state_flat);
            a_idle_last_tag:  assert (last_tag_next == last_tag_r);
            a_idle_cmp_ge:    assert (cmp_ge_next == cmp_ge_r);
            a_idle_cmp_eq:    assert (cmp_eq_next == cmp_eq_r);
            a_idle_spike:     assert (spike_flag_next == spike_flag_r);
            a_idle_emit_pend: assert (emit_pending_next == emit_pending_in);
            a_idle_emit_data: assert (emit_data_next == emit_data_in);
        end
    end
`endif

    // =========================================================================
    // A2: tag gate skip -> same as not-execute
    // =========================================================================
`ifdef FORMAL_TAG_GATE_SKIP
    always @(*) begin
        if (execute_valid && tag_gate_skip) begin
            a_tg_rf:       assert (rf_next_flat == rf_state_flat);
            a_tg_last_tag: assert (last_tag_next == last_tag_r);
            a_tg_spike:    assert (spike_flag_next == spike_flag_r);
            a_tg_cmp_ge:   assert (cmp_ge_next == cmp_ge_r);
            a_tg_cmp_eq:   assert (cmp_eq_next == cmp_eq_r);
            a_tg_emit_p:   assert (emit_pending_next == emit_pending_in);
            a_tg_emit_d:   assert (emit_data_next == emit_data_in);
        end
        if (execute_valid && instr_word[2] && exec_tag[0]) begin
            a_barrier_no_skip: assert (!tag_gate_skip);
        end
    end
    always @(*) begin
        c_tag_gate:     cover (execute_valid && tag_gate_skip);
        c_barrier_exec: cover (execute_valid && instr_word[2] && exec_tag[0]);
    end
`endif

    // =========================================================================
    // A3: OP_ACCUM_W -- saturated accumulation
    // =========================================================================
`ifdef FORMAL_ACCUM_SHADOW
    always @(*) begin
        if (real_execute && (op5 == OP5_ACCUM)) begin
            a_accum_rf1:        assert (rf_next_flat[1*RFW +: RFW] == exp_accum_sat);
            a_accum_rf0_stable: assert (rf_next_flat[0*RFW +: RFW] == rf_state_flat[0*RFW +: RFW]);
            a_accum_rf2_stable: assert (rf_next_flat[2*RFW +: RFW] == rf_state_flat[2*RFW +: RFW]);
            a_accum_spike:      assert (spike_flag_next == spike_flag_r);
            a_accum_cmp_ge:     assert (cmp_ge_next == cmp_ge_r);
            a_accum_cmp_eq:     assert (cmp_eq_next == cmp_eq_r);
            a_accum_last_tag:   assert (last_tag_next == last_tag_r);
            if (exp_accum_sat == 4'h7) begin
                a_accum_sat_max: assert (rf_next_flat[1*RFW +: RFW] == 4'h7);
            end
            if (exp_accum_sat == 4'h8) begin
                a_accum_sat_min: assert (rf_next_flat[1*RFW +: RFW] == 4'h8);
            end
        end
    end
    always @(*) begin
        c_accum_fires:     cover (real_execute && (op5 == OP5_ACCUM));
        c_accum_sat_upper: cover (real_execute && (op5 == OP5_ACCUM) && exp_accum_sat == 4'h7);
        c_accum_sat_lower: cover (real_execute && (op5 == OP5_ACCUM) && exp_accum_sat == 4'h8);
    end
`endif

    // =========================================================================
    // A4/A5: OP_INTEG -- vm = sat(vm + accum), optional accum clear
    // =========================================================================
`ifdef FORMAL_INTEG_VM
    always @(*) begin
        if (real_execute && (op5 == OP5_INTEG)) begin
            a_integ_vm:           assert (rf_next_flat[0*RFW +: RFW] == exp_integ_sat);
            a_integ_theta_stable: assert (rf_next_flat[2*RFW +: RFW] == rf_state_flat[2*RFW +: RFW]);
            a_integ_flags_spike:  assert (spike_flag_next == spike_flag_r);
            a_integ_flags_cmpge:  assert (cmp_ge_next == cmp_ge_r);
            a_integ_last_tag:     assert (last_tag_next == last_tag_r);
            if (k[2]) begin
                a_integ_accum_clear: assert (rf_next_flat[1*RFW +: RFW] == {RFW{1'b0}});
            end
            if (!k[2]) begin
                a_integ_accum_stable: assert (rf_next_flat[1*RFW +: RFW] == rf_state_flat[1*RFW +: RFW]);
            end
        end
    end
    always @(*) begin
        c_integ_fires:      cover (real_execute && (op5 == OP5_INTEG));
        c_integ_acc_clear:  cover (real_execute && (op5 == OP5_INTEG) && k[2]);
        c_integ_sat_pos:    cover (real_execute && (op5 == OP5_INTEG) && exp_integ_sat == 4'h7);
    end
`endif

    // =========================================================================
    // A6: OP_SPIKE_IF_GE -- spike_flag and cmp_ge = (vm >= threshold)
    // =========================================================================
`ifdef FORMAL_SPIKE_FLAG
    always @(*) begin
        if (real_execute && (op5 == OP5_SPIKE)) begin
            a_spike_flag:     assert (spike_flag_next == exp_spike);
            a_spike_cmp_ge:   assert (cmp_ge_next == cmp_ge_r); // RTL default passthrough
            a_spike_rf:       assert (rf_next_flat == rf_state_flat);
            a_spike_emit_p:   assert (emit_pending_next == emit_pending_in);
            a_spike_emit_d:   assert (emit_data_next == emit_data_in);
            a_spike_last_tag: assert (last_tag_next == last_tag_r);
        end
    end
    always @(*) begin
        c_spike_true:  cover (real_execute && (op5 == OP5_SPIKE) && exp_spike);
        c_spike_false: cover (real_execute && (op5 == OP5_SPIKE) && !exp_spike);
    end
`endif

    // =========================================================================
    // A7-A9: OP_RESET -- four reset modes (conditioned on spike_flag_r)
    // =========================================================================
`ifdef FORMAL_RESET_MODES
    always @(*) begin
        if (real_execute && (op5 == OP5_RESET)) begin
            a_reset_accum_stable: assert (rf_next_flat[1*RFW +: RFW] == rf_state_flat[1*RFW +: RFW]);
            a_reset_theta_stable: assert (rf_next_flat[2*RFW +: RFW] == rf_state_flat[2*RFW +: RFW]);
            a_reset_last_tag:     assert (last_tag_next == last_tag_r);
            a_reset_spike_pass:   assert (spike_flag_next == spike_flag_r);
            if (!spike_flag_r) begin
                a_reset_no_spike: assert (rf_next_flat == rf_state_flat);
            end
            if (spike_flag_r && (k[1:0] == 2'b00 || k[1:0] == 2'b11)) begin
                a_reset_zero: assert (rf_next_flat[0*RFW +: RFW] == {RFW{1'b0}});
            end
            if (spike_flag_r && (k[1:0] == 2'b01)) begin
                a_reset_sub: assert (rf_next_flat[0*RFW +: RFW] == exp_sub_sat);
            end
            if (spike_flag_r && (k[1:0] == 2'b10)) begin
                a_reset_clamp: assert (rf_next_flat[0*RFW +: RFW] == exp_clamp_vm);
            end
        end
    end
    always @(*) begin
        c_reset_spike:      cover (real_execute && (op5 == OP5_RESET) && spike_flag_r);
        c_reset_no_spike:   cover (real_execute && (op5 == OP5_RESET) && !spike_flag_r);
        c_reset_sub:        cover (real_execute && (op5 == OP5_RESET) && spike_flag_r && (k[1:0] == 2'b01));
        c_reset_clamp:      cover (real_execute && (op5 == OP5_RESET) && spike_flag_r && (k[1:0] == 2'b10));
    end
`endif

    // =========================================================================
    // A10/A11: OP_EMIT -- one-shot and data format
    // =========================================================================
`ifdef FORMAL_EMIT_ONE_SHOT
    always @(*) begin
        if (real_execute && (op5 == OP5_EMIT)) begin
            a_emit_rf:     assert (rf_next_flat == rf_state_flat);
            a_emit_spike:  assert (spike_flag_next == spike_flag_r);
            a_emit_cmpge:  assert (cmp_ge_next == cmp_ge_r);
            a_emit_ltag:   assert (last_tag_next == last_tag_r);
            if (emit_pending_in) begin
                a_emit_noop_p: assert (emit_pending_next == 1'b1);
                a_emit_noop_d: assert (emit_data_next == emit_data_in);
            end
            if (!emit_pending_in) begin
                a_emit_sets_pend:  assert (emit_pending_next == 1'b1);
                a_emit_data_fmt:   assert (emit_data_next == {1'b1, spike_flag_r, k[0], 1'b0});
            end
        end
    end
    always @(*) begin
        c_emit_first:    cover (real_execute && (op5 == OP5_EMIT) && !emit_pending_in);
        c_emit_oneshot:  cover (real_execute && (op5 == OP5_EMIT) && emit_pending_in);
    end
`endif

    // =========================================================================
    // A12: OP_LDI -- load immediate into register, isolation
    // =========================================================================
`ifdef FORMAL_LDI_LOAD
    always @(*) begin
        if (real_execute && (op5 == OP5_LDI)) begin
            if (rd < 3'(RFC)) begin
                a_ldi_loads: assert (rf_next_flat[rd*RFW +: RFW] == imm4[RFW-1:0]);
            end
            if (rd != 3'd0) begin
                a_ldi_rf0_iso: assert (rf_next_flat[0*RFW +: RFW] == rf_state_flat[0*RFW +: RFW]);
            end
            if (rd != 3'd1) begin
                a_ldi_rf1_iso: assert (rf_next_flat[1*RFW +: RFW] == rf_state_flat[1*RFW +: RFW]);
            end
            if (rd != 3'd2) begin
                a_ldi_rf2_iso: assert (rf_next_flat[2*RFW +: RFW] == rf_state_flat[2*RFW +: RFW]);
            end
            if (rd >= 3'(RFC)) begin
                a_ldi_oob:   assert (rf_next_flat == rf_state_flat);
            end
            a_ldi_spike:  assert (spike_flag_next == spike_flag_r);
            a_ldi_cmpge:  assert (cmp_ge_next == cmp_ge_r);
            a_ldi_ltag:   assert (last_tag_next == last_tag_r);
            a_ldi_emitp:  assert (emit_pending_next == emit_pending_in);
            a_ldi_emitd:  assert (emit_data_next == emit_data_in);
        end
    end
    always @(*) begin
        c_ldi_vm:    cover (real_execute && (op5 == OP5_LDI) && (rd == 3'd0));
        c_ldi_theta: cover (real_execute && (op5 == OP5_LDI) && (rd == 3'd2));
        c_ldi_neg:   cover (real_execute && (op5 == OP5_LDI) && (rd < 3'(RFC)) && imm4[7]);
    end
`endif

    // =========================================================================
    // A13: OP_RECV -- capture event tag
    // =========================================================================
`ifdef FORMAL_RECV_TAG
    always @(*) begin
        if (real_execute && (op5 == OP5_RECV)) begin
            a_recv_ltag: assert (last_tag_next == exec_tag);
            a_recv_rf:   assert (rf_next_flat == rf_state_flat);
            a_recv_spk:  assert (spike_flag_next == spike_flag_r);
            a_recv_cmpg: assert (cmp_ge_next == cmp_ge_r);
            a_recv_cmpe: assert (cmp_eq_next == cmp_eq_r);
            a_recv_emp:  assert (emit_pending_next == emit_pending_in);
            a_recv_emd:  assert (emit_data_next == emit_data_in);
        end
    end
    always @(*) begin
        c_recv_data:    cover (real_execute && (op5 == OP5_RECV) && !exec_tag[0]);
        c_recv_barrier: cover (real_execute && (op5 == OP5_RECV) && exec_tag[0]);
    end
`endif

    // -- Global covers (always active) ----------------------------------------
    always @(*) begin
        c_any_spike:    cover (real_execute && (op5 == OP5_SPIKE) && exp_spike);
        c_any_emit:     cover (real_execute && (op5 == OP5_EMIT) && !emit_pending_in);
        c_any_tag_gate: cover (execute_valid && tag_gate_skip);
    end

endmodule

`default_nettype wire
