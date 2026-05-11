// SPDX-License-Identifier: Apache-2.0
`default_nettype none
`include "tile_flit_types.vh"

// =============================================================================
// neuron_compute_core_formal.sv
//
// Sequential formal harness for neuron_compute_core (via flat-port wrapper).
//
// Uses always @(posedge clk) with assert()/cover() patterns.
// DUT: neuron_compute_core_flat (Yosys-compatible, behaviorally identical to
//      neuron_compute_core — same logic with flat ports instead of rv_if).
//
// Tasks (one ifdef per SBY task):
//   FORMAL_DISABLED_IDLE      — B1: rst/ena=0 → outputs quiescent
//   FORMAL_CLEAR_BLOCKS_START — B2: graph_state_clear → pipeline clears
//   FORMAL_ZERO_LEN_NO_FETCH  — B3: zero-len dispatch never issues ucode_read
//   FORMAL_ZERO_LEN_RESULT    — B4: zero-len dispatch → result valid next cycle
//   FORMAL_RESULT_HOLD        — B5: result_valid held until ready
//   FORMAL_BUSY_BLOCKS_START  — B6: busy → worker_start_ready=0
//   FORMAL_UCODE_PAYLOAD_ID   — B7: ucode_read logical_idx matches dispatch
//   FORMAL_CTX_PASSTHROUGH    — B8: zero-len result ctx == dispatch ctx
//   FORMAL_SINGLE_STEP_LDI    — B9: single-instruction dispatch completes
// =============================================================================

module neuron_compute_core_formal;

    // ── Concrete parameters ──────────────────────────────────────────────────
    localparam integer LIDX_W   = 2;
    localparam integer START_W  = NEURON_WORKER_START_W;
    localparam integer UREQ_W   = NEURON_UCODE_REQ_W;
    localparam integer URSP_W   = NEURON_UCODE_RSP_W;
    localparam integer RSLT_W   = NEURON_WORKER_RESULT_W;
    localparam integer CTX_W    = NEURON_EXEC_CTX_W;
    localparam integer EMIT_W   = NEURON_EMIT_W;
    localparam integer PIIDX_W  = PROG_IDX_W;

    // ── Clock / Reset ────────────────────────────────────────────────────────
    logic clk;
    logic rst_n;

    // ── Free stimulus signals ────────────────────────────────────────────────
    logic                   ena;
    logic                   graph_state_clear;

    // worker_start: env → DUT
    logic                   ws_valid;
    logic [START_W-1:0]     ws_payload;

    // ucode_rsp: env → DUT
    logic                   ur_valid;
    logic [URSP_W-1:0]      ur_payload;

    // env drives ready on DUT tx channels
    logic                   ureq_ready;
    logic                   wr_ready;

    // ── DUT output wires ─────────────────────────────────────────────────────
    wire                    ws_ready;
    wire                    ureq_valid;
    wire [UREQ_W-1:0]       ureq_payload;
    wire                    ur_ready;
    wire                    wr_valid;
    wire [RSLT_W-1:0]       wr_payload;

    // ── DUT (flat-port behavioral equivalent) ────────────────────────────────
    neuron_compute_core_flat #(
        .LOGICAL_IDX_W(LIDX_W)
    ) dut (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ena                    (ena),
        .graph_state_clear      (graph_state_clear),
        .worker_start_valid     (ws_valid),
        .worker_start_payload   (ws_payload),
        .worker_start_ready     (ws_ready),
        .ucode_read_valid       (ureq_valid),
        .ucode_read_payload     (ureq_payload),
        .ucode_read_ready       (ureq_ready),
        .ucode_rsp_valid        (ur_valid),
        .ucode_rsp_payload      (ur_payload),
        .ucode_rsp_ready        (ur_ready),
        .worker_result_valid    (wr_valid),
        .worker_result_payload  (wr_payload),
        .worker_result_ready    (wr_ready)
    );

    // ── Derived control signals ──────────────────────────────────────────────
    wire run_enabled = rst_n && ena && !graph_state_clear;

    // ── Decode start payload fields ──────────────────────────────────────────
    wire [PIIDX_W-1:0]      ws_ucode_len  = ws_payload[PIIDX_W-1:0];
    wire [PIIDX_W-1:0]      ws_ucode_ptr  = ws_payload[2*PIIDX_W-1:PIIDX_W];
    wire [CTX_W-1:0]        ws_ctx        = ws_payload[START_W-NEURON_IDX_W-1 -: CTX_W];
    wire [NEURON_IDX_W-1:0] ws_lidx       = ws_payload[START_W-1 -: NEURON_IDX_W];

    // ── Decode result payload fields ─────────────────────────────────────────
    wire [CTX_W-1:0]         wr_ctx   = wr_payload[RSLT_W-NEURON_IDX_W-1 -: CTX_W];
    wire [NEURON_IDX_W-1:0]  wr_lidx  = wr_payload[RSLT_W-1 -: NEURON_IDX_W];

    // ── Decode ucode_read request ────────────────────────────────────────────
    wire [NEURON_IDX_W-1:0]  ureq_lidx = ureq_payload[UREQ_W-1 -: NEURON_IDX_W];

    // ── Shadow: track state of last accepted dispatch ────────────────────────
    logic [NEURON_IDX_W-1:0] shadow_lidx;
    logic [CTX_W-1:0]        shadow_ctx;
    logic                    shadow_busy;
    logic                    shadow_zero_len;

    always_ff @(posedge clk) begin
        if (!rst_n || graph_state_clear) begin
            shadow_busy     <= 1'b0;
            shadow_lidx     <= '0;
            shadow_ctx      <= '0;
            shadow_zero_len <= 1'b0;
        end else if (ws_valid && ws_ready) begin
            shadow_busy     <= (ws_ucode_len != PIIDX_W'(0));
            shadow_lidx     <= ws_lidx[LIDX_W-1:0];
            shadow_ctx      <= ws_ctx;
            shadow_zero_len <= (ws_ucode_len == PIIDX_W'(0));
        end else if (wr_valid && wr_ready) begin
            shadow_busy     <= 1'b0;
            shadow_zero_len <= 1'b0;
        end
    end

    // ── Formal initialization guard ─────────────────────────────────────────
    // f_past_valid: true after at least one reset clock has fired.
    // Gates all clocked assertions to prevent false fires from unconstrained
    // initial FF state (Yosys BMC starts FFs unconstrained / anyinit).
    logic f_past_valid;
    initial f_past_valid = 1'b0;
    always_ff @(posedge clk) begin
        if (!rst_n) f_past_valid <= 1'b1;
    end

        // =========================================================================
    // B1: disabled or in reset → all outputs quiescent
    // =========================================================================
`ifdef FORMAL_DISABLED_IDLE
    // Combinatorial: run_enabled=0 -> ws_ready, ureq_valid, ur_ready all 0.
    // (These are gated by run_enabled in the RTL always_comb.)
    always @(*) begin
        if (!run_enabled) begin
            b1_ws_ready:  assert (!ws_ready);
            b1_ureq:      assert (!ureq_valid);
            b1_ur_ready:  assert (!ur_ready);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_disabled: cover (!rst_n);
        c_running:  cover (run_enabled);
    end
`endif


    // =========================================================================
    // B2: graph_state_clear → pipeline clears
    // =========================================================================
`ifdef FORMAL_CLEAR_BLOCKS_START
    // WHILE gsc=1: run_enabled=0 -> ws_ready=0, ureq_valid=0, ur_ready=0.
    // This is a combinatorial consequence of run_enabled gating.
    // (Same as B1 but specifically exercising the gsc path.)
    always @(*) begin
        if (rst_n && graph_state_clear) begin
            b2_ws_ready_during_gsc: assert (!ws_ready);
            b2_ureq_during_gsc:     assert (!ureq_valid);
            b2_ur_ready_during_gsc: assert (!ur_ready);
        end
    end
    // AFTER gsc: wr_valid cleared (one clock after gsc=1 with rst_n=1).
    // Use $past to check that gsc fired in the PREVIOUS clock.
    always @(posedge clk) begin
        if (f_past_valid && $past(graph_state_clear) && $past(rst_n) && rst_n) begin
            b2_wr_clear: assert (!wr_valid);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_clear_fires: cover (graph_state_clear && rst_n && ena);
    end
`endif

    // =========================================================================
    // B3: zero-len dispatch never issues ucode_read
    // =========================================================================
`ifdef FORMAL_ZERO_LEN_NO_FETCH
    always @(posedge clk) begin
        if (f_past_valid && $past(ws_valid && ws_ready) && ($past(ws_ucode_len) == PIIDX_W'(0))) begin
            b3_no_fetch: assert (!ureq_valid);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_zero_len_dispatch: cover (ws_valid && ws_ready && (ws_ucode_len == PIIDX_W'(0)));
    end
`endif

    // =========================================================================
    // B4: zero-len dispatch → result valid the SAME cycle
    // =========================================================================
`ifdef FORMAL_ZERO_LEN_RESULT
    always @(posedge clk) begin
        if (f_past_valid && $past(ws_valid && ws_ready) && ($past(ws_ucode_len) == PIIDX_W'(0))) begin
            b4_result_valid: assert (wr_valid);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_zero_len_result: cover (wr_valid && shadow_zero_len);
    end
`endif

    // =========================================================================
    // B5: result_valid held until ready accepted (no spurious drops)
    // =========================================================================
`ifdef FORMAL_RESULT_HOLD
    always @(posedge clk) begin
        // result_valid_r at step N is cleared by inputs at step N-1:
        //   !rst_n(N-1), graph_state_clear(N-1), or wr_ready(N-1) handshake.
        // Guard with PAST versions of all three clearing conditions.
        if (f_past_valid && $past(wr_valid) && !$past(wr_ready) && $past(rst_n)
                && !$past(graph_state_clear)) begin
            b5_result_hold: assert (wr_valid);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_result_backpressure: cover (wr_valid && !wr_ready);
    end
`endif

    // =========================================================================
    // B6: busy → worker_start_ready=0
    // =========================================================================
`ifdef FORMAL_BUSY_BLOCKS_START
    // We observe the DUT's internal busy_r through wr_valid (which clears when
    // not busy) and ws_ready. When a dispatch was just accepted, ws_ready=0
    // until the result is sent.
    always @(posedge clk) begin
        if (f_past_valid && $past(ws_valid && ws_ready) && ($past(ws_ucode_len) != PIIDX_W'(0))) begin
            // After a non-zero-len dispatch, DUT must NOT accept another immediately.
            b6_busy_blocks: assert (!ws_ready);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_busy_blocks: cover (shadow_busy && !ws_ready);
    end
`endif

    // =========================================================================
    // B7: ucode_read logical_idx matches the dispatched logical_idx
    // =========================================================================
`ifdef FORMAL_UCODE_PAYLOAD_ID
    always @(posedge clk) begin
        if (f_past_valid && ureq_valid && shadow_busy) begin
            b7_lidx: assert (ureq_lidx == shadow_lidx[NEURON_IDX_W-1:0]);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_fetch_fires: cover (ureq_valid);
    end
`endif

    // =========================================================================
    // B8: zero-len result ctx == dispatch ctx
    // =========================================================================
`ifdef FORMAL_CTX_PASSTHROUGH
    always @(posedge clk) begin
        if (f_past_valid && wr_valid && shadow_zero_len) begin
            b8_ctx_pass: assert (wr_ctx == shadow_ctx);
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_zero_result_seen: cover (wr_valid && shadow_zero_len);
    end
`endif

    // =========================================================================
    // B9: single-instruction dispatch reaches result (cover trace)
    // =========================================================================
`ifdef FORMAL_SINGLE_STEP_LDI
    // Cover: a dispatch with len=1 and ucode arrives and produces wr_valid.
    logic saw_dispatch_len1;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            saw_dispatch_len1 <= 1'b0;
        end else if (ws_valid && ws_ready && (ws_ucode_len == PIIDX_W'(1))) begin
            saw_dispatch_len1 <= 1'b1;
        end else if (wr_valid && wr_ready) begin
            saw_dispatch_len1 <= 1'b0;
        end
    end
    initial assume (!rst_n);
    always @(*) begin
        c_single_step_result: cover (wr_valid && saw_dispatch_len1);
    end
`endif

    // ── Global covers ────────────────────────────────────────────────────────
    always @(*) begin
        c_run_enabled:  cover (run_enabled);
        c_result_fires: cover (wr_valid);
    end

endmodule

`default_nettype wire
