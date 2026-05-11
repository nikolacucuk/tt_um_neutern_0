// SPDX-License-Identifier: Apache-2.0
`default_nettype none
`include "tile_flit_types.vh"

// =============================================================================
// logical_neuron_context_bank_formal.sv
//
// Comprehensive formal verification harness for logical_neuron_context_bank.
//
// Scope: leaf-level proof of ALL behavioral contracts derived from the
//        architecture document (docs/logical_neuron_context_bank_architecture.md).
//
// Instantiation: NEURONS_PER_TILE = 4 (concrete; reduces solver state space)
//
// Strategy: shadow register file mirrors the DUT's write priority exactly.
//   shadow_valid[i]  — tracks ctx_valid_r[i]
//   shadow_ctx[i]    — tracks ctx_r[i] (16-bit packed context word)
//
// The "shadow consistency" assertions (G9) prove that the DUT read outputs
// match the shadow at all times. This implies ALL of the following contracts:
//   - reset/quiescence            (G2)
//   - graph_state_clear priority  (G3)
//   - soft_reset priority/payload (G4)
//   - commit payload identity     (G5)
//   - ena=0 blocks commit         (G6)
//   - write isolation             (G8)
//
// Additional direct assertions cover:
//   - mem_stall invariant         (G1)
//   - dual-port same-index        (G7)
//
// Cover properties verify reachability of key states.
// =============================================================================

module logical_neuron_context_bank_formal;

    // ─── Concrete parameters ──────────────────────────────────────────────────
    // RF_FLAT_W and TAG_W come from tile_pkg via tile_flit_types.vh above.
    localparam int unsigned N         = 4;           // NEURONS_PER_TILE
    localparam int unsigned IDX_W     = 2;           // $clog2(4) = 2
    localparam int unsigned RF_W      = RF_FLAT_W;  // 12 from tile_pkg
    localparam int unsigned TAG_W_L   = TAG_W;      // 1 from tile_pkg
    // CTX layout: {spike_flag[1], cmp_eq[1], cmp_ge[1], last_tag[TAG_W_L], rf[RF_W]}
    localparam int unsigned CTX_W     = RF_W + TAG_W_L + 3;  // = 16
    // Bit positions within a packed ctx word (matches RTL pack_ctx / UNPACK_CTX)
    localparam int unsigned RF_HI     = RF_W - 1;           // [11:0]
    localparam int unsigned TAG_HI    = RF_W + TAG_W_L - 1; // [12:12]
    localparam int unsigned TAG_LO    = RF_W;
    localparam int unsigned GE_BIT    = RF_W + TAG_W_L;     // [13]
    localparam int unsigned EQ_BIT    = RF_W + TAG_W_L + 1; // [14]
    localparam int unsigned SF_BIT    = RF_W + TAG_W_L + 2; // [15]
    // Default RF: threshold=7, accum=0, vm=0  →  {4'h7, 4'h0, 4'h0} = 12'h700
    localparam logic [RF_W-1:0] DEF_RF = {4'h7, 4'h0, 4'h0};

    // ─── Clock / Reset ────────────────────────────────────────────────────────
    logic clk;
    logic rst_n;

    // ─── DUT port signals (all driven by the solver, unconstrained) ───────────
    logic                  ena;
    logic                  graph_state_clear;
    logic                  soft_reset_valid;
    logic [IDX_W-1:0]      soft_reset_idx;
    logic [RF_W-1:0]       init_rf_flat;
    logic                  commit_valid;
    logic [IDX_W-1:0]      commit_idx;
    logic [RF_W-1:0]       commit_rf_state_flat;
    logic [TAG_W_L-1:0]    commit_last_tag;
    logic                  commit_cmp_ge;
    logic                  commit_cmp_eq;
    logic                  commit_spike_flag;
    // Port A (read)
    logic [IDX_W-1:0]      read_idx_a;
    logic [RF_W-1:0]       read_rf_state_flat_a;
    logic [TAG_W_L-1:0]    read_last_tag_a;
    logic                  read_cmp_ge_a;
    logic                  read_cmp_eq_a;
    logic                  read_spike_flag_a;
    // Port B (read)
    logic [IDX_W-1:0]      read_idx_b;
    logic [RF_W-1:0]       read_rf_state_flat_b;
    logic [TAG_W_L-1:0]    read_last_tag_b;
    logic                  read_cmp_ge_b;
    logic                  read_cmp_eq_b;
    logic                  read_spike_flag_b;
    // Stall
    logic                  mem_stall;

    // ─── DUT ──────────────────────────────────────────────────────────────────
    logical_neuron_context_bank #(
        .NEURONS_PER_TILE(N)
    ) dut (
        .clk                  (clk),
        .rst_n                (rst_n),
        .ena                  (ena),
        .graph_state_clear    (graph_state_clear),
        .soft_reset_valid     (soft_reset_valid),
        .soft_reset_idx       (soft_reset_idx),
        .init_rf_flat         (init_rf_flat),
        .commit_valid         (commit_valid),
        .commit_idx           (commit_idx),
        .commit_rf_state_flat (commit_rf_state_flat),
        .commit_last_tag      (commit_last_tag),
        .commit_cmp_ge        (commit_cmp_ge),
        .commit_cmp_eq        (commit_cmp_eq),
        .commit_spike_flag    (commit_spike_flag),
        .read_idx_a           (read_idx_a),
        .read_rf_state_flat_a (read_rf_state_flat_a),
        .read_last_tag_a      (read_last_tag_a),
        .read_cmp_ge_a        (read_cmp_ge_a),
        .read_cmp_eq_a        (read_cmp_eq_a),
        .read_spike_flag_a    (read_spike_flag_a),
        .read_idx_b           (read_idx_b),
        .read_rf_state_flat_b (read_rf_state_flat_b),
        .read_last_tag_b      (read_last_tag_b),
        .read_cmp_ge_b        (read_cmp_ge_b),
        .read_cmp_eq_b        (read_cmp_eq_b),
        .read_spike_flag_b    (read_spike_flag_b),
        .mem_stall            (mem_stall)
    );

    // =========================================================================
    // Shadow register file
    //
    // Mirrors the DUT write sequencer with the same priority:
    //   1. graph_state_clear  → shadow_valid <= 0
    //   2. soft_reset_valid   → shadow_ctx[soft_reset_idx] <= packed(init_rf, 0-flags)
    //                           shadow_valid[soft_reset_idx] <= 1
    //   3. ena && commit_valid → shadow_ctx[commit_idx] <= packed(commit_fields)
    //                            shadow_valid[commit_idx] <= 1
    //
    // The shadow is defined to be identical to the DUT specification. Any
    // divergence between shadow expectation and DUT output is a bug.
    // =========================================================================

    logic [N-1:0]      shadow_valid;
    logic [CTX_W-1:0]  shadow_ctx [0:N-1];

    // Pack helper — matches RTL pack_ctx():
    //   {spike_flag, cmp_eq, cmp_ge, last_tag, rf_state_flat}
    function automatic [CTX_W-1:0] f_pack (
        input logic [RF_W-1:0]    rf,
        input logic [TAG_W_L-1:0] tag,
        input logic               cmp_ge,
        input logic               cmp_eq,
        input logic               spike_flag
    );
        f_pack = {spike_flag, cmp_eq, cmp_ge, tag, rf};
    endfunction

    // Shadow validity sequencer
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shadow_valid <= '0;
        end else begin
            if (graph_state_clear) begin
                shadow_valid <= '0;
            end else if (soft_reset_valid) begin
                shadow_valid[soft_reset_idx] <= 1'b1;
            end else if (ena && commit_valid) begin
                shadow_valid[commit_idx] <= 1'b1;
            end
        end
    end

    // Shadow context data sequencer (no reset needed — valid gate protects reads)
    always_ff @(posedge clk) begin
        if (soft_reset_valid && !graph_state_clear) begin
            shadow_ctx[soft_reset_idx] <=
                f_pack(init_rf_flat, '0, 1'b0, 1'b0, 1'b0);
        end else if (ena && commit_valid && !soft_reset_valid && !graph_state_clear) begin
            shadow_ctx[commit_idx] <=
                f_pack(commit_rf_state_flat, commit_last_tag,
                       commit_cmp_ge, commit_cmp_eq, commit_spike_flag);
        end
    end

    // =========================================================================
    // Formal environment
    // =========================================================================
`ifdef FORMAL

    // Initialise from reset so the prover starts in a known-clean state.
    initial assume(!rst_n);

    // =========================================================================
    // Shadow expected-output signals for each neuron index (0..3).
    // Derived combinatorially from shadow_valid/shadow_ctx.
    // These are referenced inside the assertion always block below.
    // =========================================================================

    logic [RF_W-1:0]    exp_rf  [0:N-1];
    logic [TAG_W_L-1:0] exp_tag [0:N-1];
    logic               exp_ge  [0:N-1];
    logic               exp_eq  [0:N-1];
    logic               exp_sf  [0:N-1];

    genvar gi;
    generate
        for (gi = 0; gi < N; gi++) begin : g_exp
            assign exp_rf[gi]  = shadow_valid[gi] ? shadow_ctx[gi][RF_HI:0]
                                                  : DEF_RF;
            assign exp_tag[gi] = shadow_valid[gi] ? shadow_ctx[gi][TAG_HI:TAG_LO]
                                                  : {TAG_W_L{1'b0}};
            assign exp_ge[gi]  = shadow_valid[gi] ? shadow_ctx[gi][GE_BIT]  : 1'b0;
            assign exp_eq[gi]  = shadow_valid[gi] ? shadow_ctx[gi][EQ_BIT]  : 1'b0;
            assign exp_sf[gi]  = shadow_valid[gi] ? shadow_ctx[gi][SF_BIT]  : 1'b0;
        end
    endgenerate

    // =========================================================================
    // G1.  mem_stall invariant — holds at all times including reset.
    // =========================================================================

    always @(*) begin
        ap_mem_stall_always_zero: assert (mem_stall == 1'b0);
    end

    // =========================================================================
    // All sequential formal assertions are collected in a single clocked block
    // to avoid SVA clock-specification issues with Yosys read_verilog.
    //
    // Layout:
    //   G2  Reset quiescence
    //   G7  Dual-port same-index consistency
    //   G9  Shadow state consistency (strongest; subsumes G2-G6, G8 isolation)
    //
    // NOTE: G3-G6 (GSC/soft_reset/commit/ena specific assertions) are fully
    // subsumed by G9 (shadow) and are not duplicated here because a naive
    // one-cycle readback property fails when a second write happens to the
    // same index in the immediately following cycle — valid RTL behavior that
    // is NOT a bug. G9's shadow correctly tracks all such state transitions.
    // =========================================================================

    always @(posedge clk) begin

        // ─── G2. Reset quiescence ─────────────────────────────────────────────
        // While rst_n=0, ctx_valid_r is 0 → reads return DEFAULT_RF + no flags.
        if (!rst_n) begin
            ap_rst_a_rf:    assert (read_rf_state_flat_a == DEF_RF);
            ap_rst_a_ge:    assert (read_cmp_ge_a        == 1'b0);
            ap_rst_a_eq:    assert (read_cmp_eq_a        == 1'b0);
            ap_rst_a_sf:    assert (read_spike_flag_a    == 1'b0);
            ap_rst_a_tag:   assert (read_last_tag_a      == 1'b0);
            ap_rst_b_rf:    assert (read_rf_state_flat_b == DEF_RF);
            ap_rst_b_ge:    assert (read_cmp_ge_b        == 1'b0);
            ap_rst_b_eq:    assert (read_cmp_eq_b        == 1'b0);
            ap_rst_b_sf:    assert (read_spike_flag_b    == 1'b0);
            ap_rst_b_tag:   assert (read_last_tag_b      == 1'b0);
        end

        if (rst_n) begin

            // ─── G7. Dual-port same-index consistency ─────────────────────────
            // Combinatorial property: proven in any cycle.
            if (read_idx_a == read_idx_b) begin
                ap_dual_rf:  assert (read_rf_state_flat_a == read_rf_state_flat_b);
                ap_dual_tag: assert (read_last_tag_a      == read_last_tag_b);
                ap_dual_ge:  assert (read_cmp_ge_a        == read_cmp_ge_b);
                ap_dual_eq:  assert (read_cmp_eq_a        == read_cmp_eq_b);
                ap_dual_sf:  assert (read_spike_flag_a    == read_spike_flag_b);
            end

            // ─── G9. Shadow state consistency ────────────────────────────────
            // For each neuron index, both ports must return the shadow-expected
            // values at all times. The shadow tracks the same write priority
            // as the DUT (GSC > soft_reset > ena&&commit), so divergence means
            // an RTL bug. This set proves:
            //   - GSC invalidates all entries (G3)
            //   - soft_reset stores init_rf_flat with flags=0 (G4)
            //   - commit stores all 5 fields correctly (G5)
            //   - ena=0 blocks commit (G6)
            //   - writes are isolated: index K write does not affect index J≠K (G8)

            // Neuron 0
            if (read_idx_a == 2'd0) begin
                ap_s_a_rf_0:  assert (read_rf_state_flat_a == exp_rf[0]);
                ap_s_a_tag_0: assert (read_last_tag_a      == exp_tag[0]);
                ap_s_a_ge_0:  assert (read_cmp_ge_a        == exp_ge[0]);
                ap_s_a_eq_0:  assert (read_cmp_eq_a        == exp_eq[0]);
                ap_s_a_sf_0:  assert (read_spike_flag_a    == exp_sf[0]);
            end
            if (read_idx_b == 2'd0) begin
                ap_s_b_rf_0:  assert (read_rf_state_flat_b == exp_rf[0]);
                ap_s_b_tag_0: assert (read_last_tag_b      == exp_tag[0]);
                ap_s_b_ge_0:  assert (read_cmp_ge_b        == exp_ge[0]);
                ap_s_b_eq_0:  assert (read_cmp_eq_b        == exp_eq[0]);
                ap_s_b_sf_0:  assert (read_spike_flag_b    == exp_sf[0]);
            end
            // Neuron 1
            if (read_idx_a == 2'd1) begin
                ap_s_a_rf_1:  assert (read_rf_state_flat_a == exp_rf[1]);
                ap_s_a_tag_1: assert (read_last_tag_a      == exp_tag[1]);
                ap_s_a_ge_1:  assert (read_cmp_ge_a        == exp_ge[1]);
                ap_s_a_eq_1:  assert (read_cmp_eq_a        == exp_eq[1]);
                ap_s_a_sf_1:  assert (read_spike_flag_a    == exp_sf[1]);
            end
            if (read_idx_b == 2'd1) begin
                ap_s_b_rf_1:  assert (read_rf_state_flat_b == exp_rf[1]);
                ap_s_b_tag_1: assert (read_last_tag_b      == exp_tag[1]);
                ap_s_b_ge_1:  assert (read_cmp_ge_b        == exp_ge[1]);
                ap_s_b_eq_1:  assert (read_cmp_eq_b        == exp_eq[1]);
                ap_s_b_sf_1:  assert (read_spike_flag_b    == exp_sf[1]);
            end
            // Neuron 2
            if (read_idx_a == 2'd2) begin
                ap_s_a_rf_2:  assert (read_rf_state_flat_a == exp_rf[2]);
                ap_s_a_tag_2: assert (read_last_tag_a      == exp_tag[2]);
                ap_s_a_ge_2:  assert (read_cmp_ge_a        == exp_ge[2]);
                ap_s_a_eq_2:  assert (read_cmp_eq_a        == exp_eq[2]);
                ap_s_a_sf_2:  assert (read_spike_flag_a    == exp_sf[2]);
            end
            if (read_idx_b == 2'd2) begin
                ap_s_b_rf_2:  assert (read_rf_state_flat_b == exp_rf[2]);
                ap_s_b_tag_2: assert (read_last_tag_b      == exp_tag[2]);
                ap_s_b_ge_2:  assert (read_cmp_ge_b        == exp_ge[2]);
                ap_s_b_eq_2:  assert (read_cmp_eq_b        == exp_eq[2]);
                ap_s_b_sf_2:  assert (read_spike_flag_b    == exp_sf[2]);
            end
            // Neuron 3
            if (read_idx_a == 2'd3) begin
                ap_s_a_rf_3:  assert (read_rf_state_flat_a == exp_rf[3]);
                ap_s_a_tag_3: assert (read_last_tag_a      == exp_tag[3]);
                ap_s_a_ge_3:  assert (read_cmp_ge_a        == exp_ge[3]);
                ap_s_a_eq_3:  assert (read_cmp_eq_a        == exp_eq[3]);
                ap_s_a_sf_3:  assert (read_spike_flag_a    == exp_sf[3]);
            end
            if (read_idx_b == 2'd3) begin
                ap_s_b_rf_3:  assert (read_rf_state_flat_b == exp_rf[3]);
                ap_s_b_tag_3: assert (read_last_tag_b      == exp_tag[3]);
                ap_s_b_ge_3:  assert (read_cmp_ge_b        == exp_ge[3]);
                ap_s_b_eq_3:  assert (read_cmp_eq_b        == exp_eq[3]);
                ap_s_b_sf_3:  assert (read_spike_flag_b    == exp_sf[3]);
            end

        end  // if (rst_n)

    end  // always @(posedge clk)

    // =========================================================================
    // Cover properties — reachability / sanity traces
    // =========================================================================

    always @(posedge clk) begin
        if (rst_n) begin
            // Port A reads a non-default RF value.
            cv_port_a_nondefault:
                cover (read_rf_state_flat_a != DEF_RF);

            // spike_flag is set on port A.
            cv_spike_flag_set_a:
                cover (read_spike_flag_a == 1'b1);

            // Both ports reading non-default data from different indices.
            cv_both_ports_nondefault_diff_idx:
                cover (read_rf_state_flat_a != DEF_RF &&
                       read_rf_state_flat_b != DEF_RF &&
                       read_idx_a != read_idx_b);

            // GSC then re-initialization.
            cv_reinit_after_gsc:
                cover ($past(graph_state_clear) && soft_reset_valid);

            // Commit followed immediately by GSC.
            cv_commit_then_gsc:
                cover ($past(ena && commit_valid && !soft_reset_valid &&
                             !graph_state_clear) &&
                       graph_state_clear);

            // All three spike/comparison flags set simultaneously.
            cv_all_flags_set_a:
                cover (read_cmp_ge_a == 1'b1 && read_cmp_eq_a == 1'b1 &&
                       read_spike_flag_a == 1'b1);
        end
    end

`endif  // FORMAL

endmodule

`default_nettype wire
