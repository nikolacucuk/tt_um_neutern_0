`default_nettype none

(* keep_hierarchy = "no" *)
`ifndef YOSYS
import tile_pkg::DATA_W;
import tile_pkg::NEURON_IDX_W;
import tile_pkg::NEURON_UCODE_REQ_W;
import tile_pkg::NEURON_UCODE_RSP_W;
import tile_pkg::PROG_IDX_W;
import tile_pkg::RF_FLAT_W;
import tile_pkg::TAG_W;
import tile_pkg::WEIGHT_W;
import tile_pkg::neuron_ucode_req_t;
import tile_pkg::neuron_ucode_rsp_t;
import tile_pkg::neuron_worker_result_t;
import tile_pkg::neuron_worker_start_t;
`endif
module neuron_compute_core
  #(
    parameter int unsigned LOGICAL_IDX_W = 1
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,
    // Worker dispatch: tile_top drives a neuron_worker_start_t payload.
    rv_if.rx                           worker_start,
    // Ucode fetch: request goes to tile_top's shared-bank adapter; response
    // returns one 16-bit instruction word.
    rv_if.tx                           ucode_read,
    rv_if.rx                           ucode_rsp,
    // Worker result: this module drives a neuron_worker_result_t payload.
    rv_if.tx                           worker_result
);
    // Widths derived from tile_pkg constants via tile_flit_types.vh import.
    // NEURON_WORKER_START_W  = NEURON_IDX_W + NEURON_EXEC_CTX_W + NEURON_WORKER_EVENT_W + 2*PROG_IDX_W
    // NEURON_WORKER_RESULT_W = NEURON_IDX_W + NEURON_EXEC_CTX_W + NEURON_EMIT_W
    localparam int unsigned START_W     = NEURON_WORKER_START_W;
    localparam int unsigned UCODE_REQ_W = NEURON_UCODE_REQ_W;
    localparam int unsigned UCODE_RSP_W = NEURON_UCODE_RSP_W;
    localparam int unsigned RESULT_W    = NEURON_WORKER_RESULT_W;

    neuron_worker_start_t  worker_start_payload_c;
    neuron_worker_result_t worker_result_payload_c;
    neuron_ucode_req_t     ucode_read_payload_c;
    neuron_ucode_rsp_t     ucode_rsp_payload_c;

    logic worker_start_valid_c;
    logic worker_start_ready_c;
    logic [START_W-1:0] worker_start_payload_bits_c;
    logic ucode_read_valid_c;
    logic ucode_read_ready_c;
    logic [UCODE_REQ_W-1:0] ucode_read_payload_bits_c;
    logic ucode_rsp_valid_c;
    logic ucode_rsp_ready_c;
    logic [UCODE_RSP_W-1:0] ucode_rsp_payload_bits_c;
    logic worker_result_valid_c;
    logic worker_result_ready_c;
    logic [RESULT_W-1:0] worker_result_payload_bits_c;

    assign worker_start_valid_c = worker_start.valid;
    assign worker_start_payload_bits_c = worker_start.rv_payload;
    assign worker_start.ready = worker_start_ready_c;

    assign ucode_read.valid = ucode_read_valid_c;
    assign ucode_read.rv_payload = ucode_read_payload_bits_c;
    assign ucode_read_ready_c = ucode_read.ready;

    assign ucode_rsp_valid_c = ucode_rsp.valid;
    assign ucode_rsp_payload_bits_c = ucode_rsp.rv_payload;
    assign ucode_rsp.ready = ucode_rsp_ready_c;

    assign worker_result.valid = worker_result_valid_c;
    assign worker_result.rv_payload = worker_result_payload_bits_c;
    assign worker_result_ready_c = worker_result.ready;

    assign worker_start_payload_c = neuron_worker_start_t'(worker_start_payload_bits_c);
    assign worker_result_payload_bits_c = worker_result_payload_c;
    assign ucode_read_payload_bits_c = ucode_read_payload_c;
    assign ucode_rsp_payload_c = neuron_ucode_rsp_t'(ucode_rsp_payload_bits_c);

    // ── Result valid register + output connects ───────────────────────────
    // Context outputs are driven directly from exec_*_r (stable while
    // result_valid_r is high because busy_r is cleared at that point).
    // Action outputs (weight_commit, emit) are exec_*_r FFs, cleared on
    // the result handshake.
    logic result_valid_r;
    assign worker_result_valid_c = result_valid_r;

    // ── Working registers ───────────────────────────────────────────────────
    reg busy_r;
    reg [LOGICAL_IDX_W-1:0] logical_idx_r;
    // Event-input registers: fixed for the duration of one dispatch.
    reg [4:0] work_event_sid_r;
    reg [TAG_W-1:0] work_event_tag_r;
    // Timestamps removed: neutern does not use event-time ordering.
    reg signed [WEIGHT_W-1:0] work_weight_value_r;
    // Instruction-pointer / sequencing registers.
    reg [4:0] work_pc_r;
    reg [5:0] work_remaining_r;
    reg ucode_req_pending_r;
    reg [2:0] ucode_wait_cycles_r;
    // 1-entry prefetch buffer: loaded during exec_capture_c for PC+1.
    // Drained on the following exec_commit_pending_r→work cycle instead
    // of issuing a new fetch request, hiding the ucode read latency for
    // all but the first instruction of each dispatch.
    reg pfetch_valid_r;
    reg [NEURON_UCODE_RSP_W-1:0] pfetch_word_r;
    reg exec_commit_pending_r;
    reg exec_last_step_r;
    // ── Execution-state registers (merged work+exec, single RF) ──────────
    // Neuron_exec reads these at exec_step_c and the result is clocked back
    // in at exec_capture_c.  Timestamps removed — saves 4×6=24 FFs.
    reg [RF_FLAT_W-1:0] exec_rf_state_flat_r;
    reg [TAG_W-1:0] exec_last_tag_r;
    reg exec_cmp_ge_r;
    reg exec_cmp_eq_r;
    reg exec_spike_flag_r;
    reg exec_emit_valid_r;
    reg [DATA_W-1:0] exec_emit_data_r;
    // Result output assigns — after all exec_*_r declarations (Slang forward-ref)
    assign worker_result_payload_c.logical_idx          = NEURON_IDX_W'(logical_idx_r);
    assign worker_result_payload_c.ctx.rf_state_flat    = exec_rf_state_flat_r;
    assign worker_result_payload_c.ctx.last_tag         = exec_last_tag_r;
    assign worker_result_payload_c.ctx.cmp_ge           = exec_cmp_ge_r;
    assign worker_result_payload_c.ctx.cmp_eq           = exec_cmp_eq_r;
    assign worker_result_payload_c.ctx.spike_flag       = exec_spike_flag_r;
    assign worker_result_payload_c.emit.valid           = exec_emit_valid_r;
    assign worker_result_payload_c.emit.data            = exec_emit_data_r;

    wire exec_step_c;
    wire exec_last_step_c;
    wire start_fire_c;
    wire exec_capture_c;
    wire ucode_retry_due_c;
    wire [4:0] start_pc_c;
    wire run_enabled_c;
    wire start_fetch_c;
    wire [RF_FLAT_W-1:0] rf_next_flat_c;
    wire [TAG_W-1:0] last_tag_next_c;
    wire cmp_ge_next_c;
    wire cmp_eq_next_c;
    wire spike_flag_next_c;
    wire emit_pending_next_c;
    wire [DATA_W-1:0] emit_data_next_c;
    wire [NEURON_UCODE_RSP_W-1:0] exec_instr_word_c;
    localparam logic [2:0] UcodeReadRetryCycles = 3'd4;

    assign run_enabled_c = rst_n && ena && !graph_state_clear;
    assign ucode_rsp_ready_c = run_enabled_c;  // driven here, after run_enabled_c declared
    assign worker_start_ready_c = run_enabled_c && !busy_r && !result_valid_r;
    assign start_fire_c   = worker_start_valid_c && worker_start_ready_c;
    assign start_fetch_c  = start_fire_c && (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
    assign start_pc_c     = worker_start_payload_c.ucode_ptr;
    // exec_capture fires when a ucode word is available — either from the
    // live read port or from the prefetch buffer (priority: prefetch first).
    assign exec_capture_c = run_enabled_c && busy_r && ucode_req_pending_r &&
                            (ucode_rsp_valid_c || pfetch_valid_r);
    assign exec_instr_word_c = pfetch_valid_r ? pfetch_word_r : ucode_rsp_payload_c.word;
    assign exec_step_c = exec_capture_c;
    assign exec_last_step_c = exec_step_c && (work_remaining_r <= 6'd1);
    assign ucode_retry_due_c = (ucode_wait_cycles_r >= UcodeReadRetryCycles);

    always_comb begin
        ucode_read_valid_c = 1'b0;
        ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
        ucode_read_payload_c.word_index = work_pc_r;

        if (start_fetch_c) begin
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = worker_start_payload_c.logical_idx;
            ucode_read_payload_c.word_index = start_pc_c;
        end else if (exec_capture_c && !exec_last_step_c) begin
            // Prefetch PC+1 while the current word is being captured.
            // PC has not incremented yet so work_pc_r+1 is the next address.
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
            ucode_read_payload_c.word_index = work_pc_r + PROG_IDX_W'(1);
        end else if (run_enabled_c &&
                     busy_r &&
                     !exec_commit_pending_r &&
                     !pfetch_valid_r &&
                     !ucode_rsp_valid_c &&
                     (!ucode_req_pending_r || ucode_retry_due_c)) begin
            // Normal fetch: only when prefetch buffer is empty.
            ucode_read_valid_c = 1'b1;
            ucode_read_payload_c.logical_idx = NEURON_IDX_W'(logical_idx_r);
            ucode_read_payload_c.word_index = work_pc_r;
        end
    end

    neuron_exec u_exec (
        .execute_valid(exec_step_c),
        .exec_tag(work_event_tag_r),
        .rf_state_flat(exec_rf_state_flat_r),
        .current_weight_value(work_weight_value_r),
        .instr_word({{(16-NEURON_UCODE_RSP_W){1'b0}}, exec_instr_word_c}),
        .last_tag_r(exec_last_tag_r),
        .cmp_ge_r(exec_cmp_ge_r),
        .cmp_eq_r(exec_cmp_eq_r),
        .spike_flag_r(exec_spike_flag_r),
        .emit_pending_in(exec_emit_valid_r),
        .emit_data_in(exec_emit_data_r),
        .rf_next_flat(rf_next_flat_c),
        .last_tag_next(last_tag_next_c),
        .cmp_ge_next(cmp_ge_next_c),
        .cmp_eq_next(cmp_eq_next_c),
        .spike_flag_next(spike_flag_next_c),
        .emit_pending_next(emit_pending_next_c),
        .emit_data_next(emit_data_next_c)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy_r <= 1'b0;
            ucode_req_pending_r <= 1'b0;
            ucode_wait_cycles_r <= '0;
            pfetch_valid_r <= 1'b0;
            pfetch_word_r <= '0;
            exec_commit_pending_r <= 1'b0;
            exec_last_step_r <= 1'b0;
            result_valid_r <= 1'b0;
            exec_emit_valid_r <= 1'b0;
            exec_last_tag_r <= '0;
            exec_cmp_ge_r <= 1'b0;
            exec_cmp_eq_r <= 1'b0;
            exec_spike_flag_r <= 1'b0;
            exec_emit_data_r <= '0;
            work_event_sid_r <= '0;
            work_event_tag_r <= '0;
            work_weight_value_r <= '0;
            work_pc_r <= '0;
            work_remaining_r <= '0;
            logical_idx_r <= '0;
        end else begin
            if (graph_state_clear) begin
                // Minimum set of FFs required for correct re-start after a
                // graph flush.  Data registers (exec_*, result_data)
                // are written unconditionally by start_fire_c / exec_capture_c
                // before they are read, so they do not need clearing here.
                // Only reset architectural-safety control bits.
                busy_r <= 1'b0;
                ucode_req_pending_r <= 1'b0;
                ucode_wait_cycles_r <= '0;
                pfetch_valid_r <= 1'b0;
                exec_commit_pending_r <= 1'b0;
                exec_last_step_r <= 1'b0;
                result_valid_r <= 1'b0;
                exec_emit_valid_r <= 1'b0;
            end else if (ena) begin
                if (result_valid_r && worker_result_ready_c) begin
                    result_valid_r <= 1'b0;
                    exec_emit_valid_r <= 1'b0;
                end

                // Capture incoming ucode word into prefetch buffer when a
                // word arrives during exec_commit_pending_r (the cycle after
                // exec_capture_c, when the prefetch request was issued).
                // Also clear the prefetch buffer when it gets drained by
                // exec_capture_c.
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
                        // Result context (exec_*_r) already holds the final
                        // values; just assert result_valid_r.  No copy needed.
                        busy_r <= 1'b0;
                        result_valid_r <= 1'b1;
                    end else begin
                        // exec_*_r already updated by exec_capture_c; just
                        // advance PC and re-arm the ucode fetch.
                        work_pc_r <= work_pc_r + 5'd1;
                        work_remaining_r <= work_remaining_r - 6'd1;
                        // Prefetch buffer drains on the next exec_capture_c;
                        // re-arm ucode_req_pending_r so the capture logic fires.
                        ucode_req_pending_r <= 1'b1;
                    end
                end

                if (start_fire_c) begin
                    busy_r <= (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
                    ucode_req_pending_r <= (worker_start_payload_c.ucode_len != PROG_IDX_W'(0));
                    ucode_wait_cycles_r <= '0;
                    pfetch_valid_r <= 1'b0;
                    logical_idx_r <= LOGICAL_IDX_W'(worker_start_payload_c.logical_idx);
                    // Pre-load exec_*_r from context.
                    exec_rf_state_flat_r <= worker_start_payload_c.ctx.rf_state_flat;
                    exec_last_tag_r <= worker_start_payload_c.ctx.last_tag;
                    exec_cmp_ge_r <= worker_start_payload_c.ctx.cmp_ge;
                    exec_cmp_eq_r <= worker_start_payload_c.ctx.cmp_eq;
                    exec_spike_flag_r <= worker_start_payload_c.ctx.spike_flag;
                    exec_emit_valid_r <= 1'b0;
                    exec_emit_data_r <= '0;
                    work_event_tag_r <= worker_start_payload_c.in_event.tag;
                    work_weight_value_r <= worker_start_payload_c.in_event.weight;
                    work_pc_r <= start_pc_c;
                    work_remaining_r <= {1'b0, worker_start_payload_c.ucode_len};
                    if (worker_start_payload_c.ucode_len == PROG_IDX_W'(0)) begin
                        // Zero-ucode: exec_*_r pre-loaded above; just fire result.
                        result_valid_r <= 1'b1;
                    end
                end else if (exec_capture_c) begin
                    ucode_req_pending_r <= 1'b0;
                    ucode_wait_cycles_r <= '0;
                    exec_commit_pending_r <= 1'b1;
                    exec_last_step_r <= exec_last_step_c;
                    exec_rf_state_flat_r <= rf_next_flat_c;
                    exec_last_tag_r <= last_tag_next_c;
                    exec_cmp_ge_r <= cmp_ge_next_c;
                    exec_cmp_eq_r <= cmp_eq_next_c;
                    exec_spike_flag_r <= spike_flag_next_c;
                    exec_emit_valid_r <= emit_pending_next_c;
                    exec_emit_data_r <= emit_data_next_c;
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
