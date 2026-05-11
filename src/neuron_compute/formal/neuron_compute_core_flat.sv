// SPDX-License-Identifier: Apache-2.0
// FORMAL-ONLY: flat-port variant of neuron_compute_core for Yosys compatibility.
//
// Yosys 0.64 does not support SystemVerilog interface ports (rv_if.rx/tx).
// This module is an exact behavioral copy of neuron_compute_core with the
// four rv_if ports replaced by flat valid/ready/payload triples.
//
// It is NOT synthesized — only used by the formal harness.
`default_nettype none
`include "tile_flit_types.vh"

module neuron_compute_core_flat #(
    parameter int unsigned LOGICAL_IDX_W = 1
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,

    // worker_start (DUT = rx: input valid/payload, output ready)
    input  wire                              worker_start_valid,
    input  wire [NEURON_WORKER_START_W-1:0]  worker_start_payload,
    output wire                              worker_start_ready,

    // ucode_read (DUT = tx: output valid/payload, input ready)
    output wire                              ucode_read_valid,
    output wire [NEURON_UCODE_REQ_W-1:0]    ucode_read_payload,
    input  wire                              ucode_read_ready,

    // ucode_rsp (DUT = rx: input valid/payload, output ready)
    input  wire                              ucode_rsp_valid,
    input  wire [NEURON_UCODE_RSP_W-1:0]    ucode_rsp_payload,
    output wire                              ucode_rsp_ready,

    // worker_result (DUT = tx: output valid/payload, input ready)
    output wire                              worker_result_valid,
    output wire [NEURON_WORKER_RESULT_W-1:0] worker_result_payload,
    input  wire                              worker_result_ready
);
    localparam int unsigned START_W     = NEURON_WORKER_START_W;
    localparam int unsigned UCODE_REQ_W = NEURON_UCODE_REQ_W;
    localparam int unsigned UCODE_RSP_W = NEURON_UCODE_RSP_W;
    localparam int unsigned RESULT_W    = NEURON_WORKER_RESULT_W;

    // ── Flat interface signal aliases (matching neuron_compute_core internals) ──
    wire worker_start_valid_c         = worker_start_valid;
    wire [START_W-1:0] worker_start_payload_bits_c = worker_start_payload;
    wire worker_result_ready_c        = worker_result_ready;
    wire ucode_read_ready_c           = ucode_read_ready;
    wire ucode_rsp_valid_c            = ucode_rsp_valid;
    wire [UCODE_RSP_W-1:0] ucode_rsp_payload_bits_c = ucode_rsp_payload;

    neuron_worker_start_t  worker_start_payload_c;
    neuron_worker_result_t worker_result_payload_c;
    neuron_ucode_req_t     ucode_read_payload_c;
    neuron_ucode_rsp_t     ucode_rsp_payload_c;

    logic worker_start_ready_c;
    logic ucode_read_valid_c;
    logic [UCODE_REQ_W-1:0] ucode_read_payload_bits_c;
    logic ucode_rsp_ready_c;
    logic worker_result_valid_c;
    logic [RESULT_W-1:0] worker_result_payload_bits_c;

    assign worker_start_ready   = worker_start_ready_c;
    assign ucode_read_valid     = ucode_read_valid_c;
    assign ucode_read_payload   = ucode_read_payload_bits_c;
    assign ucode_rsp_ready      = ucode_rsp_ready_c;
    assign worker_result_valid  = worker_result_valid_c;
    assign worker_result_payload = worker_result_payload_bits_c;

    assign worker_start_payload_c        = neuron_worker_start_t'(worker_start_payload_bits_c);
    assign worker_result_payload_bits_c  = worker_result_payload_c;
    assign ucode_read_payload_bits_c     = ucode_read_payload_c;
    assign ucode_rsp_payload_c           = neuron_ucode_rsp_t'(ucode_rsp_payload_bits_c);

    // ── Result valid register ────────────────────────────────────────────────
    logic result_valid_r;
    assign worker_result_valid_c = result_valid_r;

    // ── Working registers ────────────────────────────────────────────────────
    reg busy_r;
    reg [LOGICAL_IDX_W-1:0] logical_idx_r;
    reg [4:0] work_event_sid_r;
    reg [1:0] work_event_tag_r;
    reg signed [7:0] work_weight_value_r;
    reg [4:0] work_pc_r;
    reg [5:0] work_remaining_r;
    reg ucode_req_pending_r;
    reg [2:0] ucode_wait_cycles_r;
    reg pfetch_valid_r;
    reg [NEURON_UCODE_RSP_W-1:0] pfetch_word_r;
    reg exec_commit_pending_r;
    reg exec_last_step_r;

    // ── Execution-state registers ────────────────────────────────────────────
    reg [RF_FLAT_W-1:0] exec_rf_state_flat_r;
    reg [1:0] exec_last_tag_r;
    reg exec_cmp_ge_r;
    reg exec_cmp_eq_r;
    reg exec_spike_flag_r;
    reg exec_emit_valid_r;
    reg [DATA_W-1:0] exec_emit_data_r;

    // ── Result payload assigns ───────────────────────────────────────────────
    assign worker_result_payload_c.logical_idx       = NEURON_IDX_W'(logical_idx_r);
    assign worker_result_payload_c.ctx.rf_state_flat = exec_rf_state_flat_r;
    assign worker_result_payload_c.ctx.last_tag      = exec_last_tag_r;
    assign worker_result_payload_c.ctx.cmp_ge        = exec_cmp_ge_r;
    assign worker_result_payload_c.ctx.cmp_eq        = exec_cmp_eq_r;
    assign worker_result_payload_c.ctx.spike_flag    = exec_spike_flag_r;
    assign worker_result_payload_c.emit.valid        = exec_emit_valid_r;
    assign worker_result_payload_c.emit.data         = exec_emit_data_r;

    // ── Control wires ────────────────────────────────────────────────────────
    wire exec_step_c;
    wire exec_last_step_c;
    wire start_fire_c;
    wire exec_capture_c;
    wire ucode_retry_due_c;
    wire [4:0] start_pc_c;
    wire run_enabled_c;
    wire start_fetch_c;
    wire [RF_FLAT_W-1:0] rf_next_flat_c;
    wire [1:0] last_tag_next_c;
    wire cmp_ge_next_c;
    wire cmp_eq_next_c;
    wire spike_flag_next_c;
    wire emit_pending_next_c;
    wire [DATA_W-1:0] emit_data_next_c;
    wire [NEURON_UCODE_RSP_W-1:0] exec_instr_word_c;
    localparam logic [2:0] UcodeReadRetryCycles = 3'd4;

    assign run_enabled_c        = rst_n && ena && !graph_state_clear;
    assign ucode_rsp_ready_c    = run_enabled_c;
    assign worker_start_ready_c = run_enabled_c && !busy_r && !result_valid_r;
    assign start_fire_c         = worker_start_valid_c && worker_start_ready_c;
    assign start_fetch_c        = start_fire_c && (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
    assign start_pc_c           = worker_start_payload_c.ucode_ptr;
    assign exec_capture_c       = run_enabled_c && busy_r && ucode_req_pending_r &&
                                  (ucode_rsp_valid_c || pfetch_valid_r);
    assign exec_instr_word_c    = pfetch_valid_r ? pfetch_word_r : ucode_rsp_payload_c.word;
    assign exec_step_c          = exec_capture_c;
    assign exec_last_step_c     = exec_step_c && (work_remaining_r <= 6'd1);
    assign ucode_retry_due_c    = (ucode_wait_cycles_r >= UcodeReadRetryCycles);

    always_comb begin
        ucode_read_valid_c = 1'b0;
        ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
        ucode_read_payload_c.word_index  = work_pc_r;

        if (start_fetch_c) begin
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = worker_start_payload_c.logical_idx;
            ucode_read_payload_c.word_index  = start_pc_c;
        end else if (exec_capture_c && !exec_last_step_c) begin
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
            ucode_read_payload_c.word_index  = work_pc_r + PROG_IDX_W'(1);
        end else if (run_enabled_c &&
                     busy_r &&
                     !exec_commit_pending_r &&
                     !pfetch_valid_r &&
                     !ucode_rsp_valid_c &&
                     (!ucode_req_pending_r || ucode_retry_due_c)) begin
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
            ucode_read_payload_c.word_index  = work_pc_r;
        end
    end

    neuron_exec u_exec (
        .execute_valid        (exec_step_c),
        .exec_tag             (work_event_tag_r),
        .rf_state_flat        (exec_rf_state_flat_r),
        .current_weight_value (work_weight_value_r),
        .instr_word           (exec_instr_word_c),
        .last_tag_r           (exec_last_tag_r),
        .cmp_ge_r             (exec_cmp_ge_r),
        .cmp_eq_r             (exec_cmp_eq_r),
        .spike_flag_r         (exec_spike_flag_r),
        .emit_pending_in      (exec_emit_valid_r),
        .emit_data_in         (exec_emit_data_r),
        .rf_next_flat         (rf_next_flat_c),
        .last_tag_next        (last_tag_next_c),
        .cmp_ge_next          (cmp_ge_next_c),
        .cmp_eq_next          (cmp_eq_next_c),
        .spike_flag_next      (spike_flag_next_c),
        .emit_pending_next    (emit_pending_next_c),
        .emit_data_next       (emit_data_next_c)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy_r              <= 1'b0;
            ucode_req_pending_r <= 1'b0;
            ucode_wait_cycles_r <= '0;
            pfetch_valid_r      <= 1'b0;
            pfetch_word_r       <= '0;
            exec_commit_pending_r <= 1'b0;
            exec_last_step_r    <= 1'b0;
            result_valid_r      <= 1'b0;
            exec_emit_valid_r   <= 1'b0;
            exec_last_tag_r     <= '0;
            exec_cmp_ge_r       <= 1'b0;
            exec_cmp_eq_r       <= 1'b0;
            exec_spike_flag_r   <= 1'b0;
            exec_emit_data_r    <= '0;
            work_event_sid_r    <= '0;
            work_event_tag_r    <= '0;
            work_weight_value_r <= '0;
            work_pc_r           <= '0;
            work_remaining_r    <= '0;
            logical_idx_r       <= '0;
        end else begin
            if (graph_state_clear) begin
                busy_r              <= 1'b0;
                ucode_req_pending_r <= 1'b0;
                ucode_wait_cycles_r <= '0;
                pfetch_valid_r      <= 1'b0;
                exec_commit_pending_r <= 1'b0;
                exec_last_step_r    <= 1'b0;
                result_valid_r      <= 1'b0;
                exec_emit_valid_r   <= 1'b0;
            end else if (ena) begin
                if (result_valid_r && worker_result_ready_c) begin
                    result_valid_r    <= 1'b0;
                    exec_emit_valid_r <= 1'b0;
                end

                if (exec_commit_pending_r && ucode_rsp_valid_c && !pfetch_valid_r) begin
                    pfetch_valid_r <= 1'b1;
                    pfetch_word_r  <= ucode_rsp_payload_c.word;
                end
                if (exec_capture_c && pfetch_valid_r) begin
                    pfetch_valid_r <= 1'b0;
                end

                if (exec_commit_pending_r) begin
                    exec_commit_pending_r <= 1'b0;
                    if (exec_last_step_r) begin
                        busy_r         <= 1'b0;
                        result_valid_r <= 1'b1;
                    end else begin
                        work_pc_r        <= work_pc_r + 5'd1;
                        work_remaining_r <= work_remaining_r - 6'd1;
                        ucode_req_pending_r <= 1'b1;
                    end
                end

                if (start_fire_c) begin
                    busy_r              <= (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
                    ucode_req_pending_r <= (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
                    ucode_wait_cycles_r <= '0;
                    pfetch_valid_r      <= 1'b0;
                    logical_idx_r       <= LOGICAL_IDX_W'(worker_start_payload_c.logical_idx);
                    exec_rf_state_flat_r <= worker_start_payload_c.ctx.rf_state_flat;
                    exec_last_tag_r     <= worker_start_payload_c.ctx.last_tag;
                    exec_cmp_ge_r       <= worker_start_payload_c.ctx.cmp_ge;
                    exec_cmp_eq_r       <= worker_start_payload_c.ctx.cmp_eq;
                    exec_spike_flag_r   <= worker_start_payload_c.ctx.spike_flag;
                    exec_emit_valid_r   <= 1'b0;
                    exec_emit_data_r    <= '0;
                    work_event_tag_r    <= worker_start_payload_c.in_event.tag;
                    work_weight_value_r <= worker_start_payload_c.in_event.weight;
                    work_pc_r           <= start_pc_c;
                    work_remaining_r    <= {1'b0, worker_start_payload_c.ucode_len};
                    if (worker_start_payload_c.ucode_len == PROG_IDX_W'(0)) begin
                        result_valid_r <= 1'b1;
                    end
                end else if (exec_capture_c) begin
                    ucode_req_pending_r   <= 1'b0;
                    ucode_wait_cycles_r   <= '0;
                    exec_commit_pending_r <= 1'b1;
                    exec_last_step_r      <= exec_last_step_c;
                    exec_rf_state_flat_r  <= rf_next_flat_c;
                    exec_last_tag_r       <= last_tag_next_c;
                    exec_cmp_ge_r         <= cmp_ge_next_c;
                    exec_cmp_eq_r         <= cmp_eq_next_c;
                    exec_spike_flag_r     <= spike_flag_next_c;
                    exec_emit_valid_r     <= emit_pending_next_c;
                    exec_emit_data_r      <= emit_data_next_c;
                end else if (busy_r && !exec_commit_pending_r && !pfetch_valid_r) begin
                    if ((!ucode_req_pending_r || ucode_retry_due_c) &&
                        !ucode_rsp_valid_c) begin
                        ucode_req_pending_r <= 1'b1;
                        ucode_wait_cycles_r <= '0;
                    end else if (ucode_req_pending_r &&
                                 !ucode_retry_due_c &&
                                 !ucode_rsp_valid_c) begin
                        ucode_wait_cycles_r <= ucode_wait_cycles_r + 3'd1;
                    end
                end
            end
        end
    end
endmodule

`default_nettype wire
