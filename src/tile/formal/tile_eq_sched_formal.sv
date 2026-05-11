`default_nettype none

module tile_eq_sched_formal #(
    parameter int unsigned NEURONS_PER_TILE = 4,
    parameter int unsigned WORKER_CORES_PER_TILE = 1,
    parameter int unsigned FANOUT_POOL_DEPTH = 4,
    parameter int unsigned FANOUT_QUEUE_DEPTH = 4
) (
    input logic clk,
    input logic rst_n
);
`ifndef YOSYS
    import tile_pkg::EVENT_TIME_W;
    import tile_pkg::NEURON_EMIT_W;
    import tile_pkg::NEURON_EXEC_CTX_W;
    import tile_pkg::NEURON_IDX_W;
    import tile_pkg::TAG_W;
    import tile_pkg::WEIGHT_W;
    import tile_pkg::context_commit_t;
`endif

    localparam int unsigned NIDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE);
    localparam int unsigned WIDX_W =
        (WORKER_CORES_PER_TILE <= 1) ? 1 : $clog2(WORKER_CORES_PER_TILE);
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);

    logic ena;
    logic tile_graph_state_clear;
    logic soft_reset_valid_in;
    logic [NIDX_W-1:0] soft_reset_idx_in;

    logic logical_spike_pending_valid;
    logic [NIDX_W-1:0] logical_spike_pending_idx;
    logic logical_spike_full;
    logic signed [WEIGHT_W-1:0] logical_spike_head_read_weight;
    logic [TAG_W-1:0] logical_spike_head_read_tag;
    logic [EVENT_TIME_W-1:0] logical_spike_head_read_time;

    logic dequeue_valid;
    logic [NIDX_W-1:0] dequeue_idx;
    logic [NIDX_W-1:0] head_read_idx;

    logic [NIDX_W-1:0] state_dispatch_read_idx;
    logic [4:0] state_dispatch_ucode_ptr;
    logic [4:0] state_dispatch_ucode_len;
    logic [FANOUT_ADDR_W-1:0] state_dispatch_fanout_len;
    logic noc_egress_en;

    logic [WORKER_CORES_PER_TILE-1:0] worker_start_valid;
    logic [WORKER_CORES_PER_TILE-1:0] worker_start_ready;
    logic [TAG_W-1:0] worker_start_event_tag;
    logic [EVENT_TIME_W-1:0] worker_start_event_time;
    logic signed [WEIGHT_W-1:0] worker_start_event_weight;
    logic [NIDX_W-1:0] start_dispatch_logical_idx;
    logic start_dispatch_valid_out;
    logic [WIDX_W-1:0] start_dispatch_worker_idx_out;
    logic worker_start_fire;

    logic [WORKER_CORES_PER_TILE-1:0] worker_result_valid;
    logic [WORKER_CORES_PER_TILE-1:0] worker_result_ready;
    logic [WORKER_CORES_PER_TILE*NIDX_W-1:0] worker_result_logical_idx;
    logic [WORKER_CORES_PER_TILE*NEURON_EXEC_CTX_W-1:0] worker_result_ctx;
    logic [WORKER_CORES_PER_TILE*NEURON_EMIT_W-1:0] worker_result_emit;

    logic worker_commit_valid;
    logic worker_commit_ready;
    logic [NIDX_W-1:0] worker_commit_logical_idx;
    logic [WIDX_W-1:0] worker_commit_idx;
    logic [WORKER_CORES_PER_TILE-1:0] worker_has_fanout_snapshot;

    rv_if #(.WIDTH($bits(context_commit_t))) context_commit();

    logic host_reserve_current;
    logic fanout_queue_can_accept;
    logic [((FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH + 1))-1:0]
        fanout_queue_count_live;

    logic [WIDX_W-1:0] formal_worker_rr_ptr_r;
    logic [NEURONS_PER_TILE-1:0] formal_logical_event_inflight_r;
    logic [NEURONS_PER_TILE-1:0] formal_in_ready_fifo_r;
    logic formal_ready_push;
    logic [NIDX_W-1:0] formal_ready_push_idx;
    logic formal_dispatch_fire;
    logic [NIDX_W-1:0] formal_dispatch_idx;
    logic formal_prev_pending_valid_r;
    logic [NIDX_W-1:0] formal_prev_pending_idx_r;
    logic formal_start_dispatch_pending_r;
    logic formal_start_dispatch_valid_r;
    logic [NIDX_W-1:0] formal_start_dispatch_logical_idx_r;
    logic formal_ready_fifo_full;
    logic [NEURONS_PER_TILE-1:0] formal_ready_push_mask_r;
    logic formal_pop_reservation_valid_r;
    logic [NIDX_W-1:0] formal_pop_reservation_idx_r;

    tile_dispatch_scheduler #(
        .NEURONS_PER_TILE      (NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE (WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH     (FANOUT_POOL_DEPTH),
        .FANOUT_QUEUE_DEPTH    (FANOUT_QUEUE_DEPTH),
        .LOCAL_Z               (0)
    ) dut (
        .clk                               (clk),
        .rst_n                             (rst_n),
        .ena                               (ena),
        .tile_graph_state_clear            (tile_graph_state_clear),
        .soft_reset_valid_in               (soft_reset_valid_in),
        .soft_reset_idx_in                 (soft_reset_idx_in),
        .soft_reset_valid                  (),
        .soft_reset_idx                    (),
        .logical_spike_pending_valid       (logical_spike_pending_valid),
        .logical_spike_pending_idx         (logical_spike_pending_idx),
        .logical_spike_full                (logical_spike_full),
        .logical_spike_head_read_weight    (logical_spike_head_read_weight),
        .logical_spike_head_read_tag       (logical_spike_head_read_tag),
        .logical_spike_head_read_time      (logical_spike_head_read_time),
        .dequeue_valid                     (dequeue_valid),
        .dequeue_idx                       (dequeue_idx),
        .head_read_idx                     (head_read_idx),
        .state_dispatch_read_idx           (state_dispatch_read_idx),
        .state_dispatch_ucode_ptr          (state_dispatch_ucode_ptr),
        .state_dispatch_ucode_len          (state_dispatch_ucode_len),
        .state_dispatch_fanout_len         (state_dispatch_fanout_len),
        .noc_egress_en                     (noc_egress_en),
        .worker_start_valid                (worker_start_valid),
        .worker_start_ready                (worker_start_ready),
        .worker_start_event_tag            (worker_start_event_tag),
        .worker_start_event_time           (worker_start_event_time),
        .worker_start_event_weight         (worker_start_event_weight),
        .start_dispatch_logical_idx        (start_dispatch_logical_idx),
        .start_dispatch_valid_out          (start_dispatch_valid_out),
        .start_dispatch_worker_idx_out     (start_dispatch_worker_idx_out),
        .worker_start_fire                 (worker_start_fire),
        .worker_result_valid               (worker_result_valid),
        .worker_result_ready               (worker_result_ready),
        .worker_result_logical_idx         (worker_result_logical_idx),
        .worker_result_ctx                 (worker_result_ctx),
        .worker_result_emit                (worker_result_emit),
        .worker_commit_valid               (worker_commit_valid),
        .worker_commit_ready               (worker_commit_ready),
        .worker_commit_logical_idx         (worker_commit_logical_idx),
        .worker_commit_idx                 (worker_commit_idx),
        .worker_has_fanout_snapshot        (worker_has_fanout_snapshot),
        .context_commit                    (context_commit),
        .host_reserve_current              (host_reserve_current),
        .fanout_queue_can_accept           (fanout_queue_can_accept),
        .fanout_queue_count_live           (fanout_queue_count_live),
        .formal_worker_rr_ptr_r            (formal_worker_rr_ptr_r),
        .formal_logical_event_inflight_r   (formal_logical_event_inflight_r),
        .formal_in_ready_fifo_r            (formal_in_ready_fifo_r),
        .formal_ready_push                 (formal_ready_push),
        .formal_ready_push_idx             (formal_ready_push_idx),
        .formal_dispatch_fire              (formal_dispatch_fire),
        .formal_dispatch_idx               (formal_dispatch_idx),
        .formal_prev_pending_valid_r       (formal_prev_pending_valid_r),
        .formal_prev_pending_idx_r         (formal_prev_pending_idx_r),
        .formal_start_dispatch_pending_r   (formal_start_dispatch_pending_r),
        .formal_start_dispatch_valid_r     (formal_start_dispatch_valid_r),
        .formal_start_dispatch_logical_idx_r(formal_start_dispatch_logical_idx_r),
        .formal_ready_fifo_full            (formal_ready_fifo_full),
        .formal_ready_push_mask_r          (formal_ready_push_mask_r),
        .formal_pop_reservation_valid_r    (formal_pop_reservation_valid_r),
        .formal_pop_reservation_idx_r      (formal_pop_reservation_idx_r)
    );

    function automatic [NIDX_W-1:0] lowest_set_idx(
        input logic [NEURONS_PER_TILE-1:0] mask,
        output logic found
    );
        integer k;
        begin
            found = 1'b0;
            lowest_set_idx = '0;
            for (k = 0; k < NEURONS_PER_TILE; k = k + 1) begin
                if (!found && mask[k]) begin
                    found = 1'b1;
                    lowest_set_idx = NIDX_W'(k);
                end
            end
        end
    endfunction

    logic [NEURONS_PER_TILE-1:0] pending_r;
    logic [NEURONS_PER_TILE-1:0] push_mask_c;
    logic [NIDX_W-1:0] push_idx_c;
    logic push_valid_c;
    logic pending_found_c;

    logic signed [WEIGHT_W-1:0] queue_weight_r [0:NEURONS_PER_TILE-1];
    logic [TAG_W-1:0] queue_tag_r [0:NEURONS_PER_TILE-1];
    logic [EVENT_TIME_W-1:0] queue_time_r [0:NEURONS_PER_TILE-1];

    (* anyseq *) logic f_push_valid;
    (* anyseq *) logic [NIDX_W-1:0] f_push_idx;
    (* anyseq *) logic signed [WEIGHT_W-1:0] f_push_weight;

    logic worker_pending_r;
    logic [NIDX_W-1:0] worker_pending_idx_r;

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;

    logic f_first_cycle = 1'b1;
    always_ff @(posedge clk) f_first_cycle <= 1'b0;

    always_ff @(posedge clk) begin
        if (f_first_cycle) assume(!rst_n);
        if (!rst_n) begin
            assume(!ena);
            assume(!f_push_valid);
        end else begin
            assume(ena);
        end
        assume(!tile_graph_state_clear);
        assume(!soft_reset_valid_in);
    end
`endif

    always_comb begin
        push_valid_c = 1'b0;
        push_idx_c = '0;
        push_mask_c = '0;
        if (f_push_valid && !pending_r[f_push_idx] && !formal_logical_event_inflight_r[f_push_idx]) begin
            push_valid_c = 1'b1;
            push_idx_c = f_push_idx;
            push_mask_c[f_push_idx] = 1'b1;
        end
    end

    always_comb begin
        logical_spike_pending_idx = lowest_set_idx(pending_r, pending_found_c);
        logical_spike_pending_valid = pending_found_c;
        logical_spike_full = &pending_r;

        logical_spike_head_read_weight = queue_weight_r[head_read_idx];
        logical_spike_head_read_tag = queue_tag_r[head_read_idx];
        logical_spike_head_read_time = queue_time_r[head_read_idx];

        state_dispatch_ucode_ptr = '0;
        state_dispatch_ucode_len = 5'd1;
        state_dispatch_fanout_len = '0;
        noc_egress_en = 1'b0;

        host_reserve_current = 1'b0;
        fanout_queue_can_accept = 1'b1;
        fanout_queue_count_live = '0;

        worker_start_ready = '1;
        worker_result_valid = worker_pending_r;
        worker_result_logical_idx = {NIDX_W{1'b0}};
        worker_result_logical_idx[NIDX_W-1:0] = worker_pending_idx_r;
        worker_result_ctx = '0;
        worker_result_emit = '0;
        context_commit.ready = 1'b1;

        soft_reset_idx_in = '0;
    end

    integer qi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_r <= '0;
            worker_pending_r <= 1'b0;
            worker_pending_idx_r <= '0;
            for (qi = 0; qi < NEURONS_PER_TILE; qi = qi + 1) begin
                queue_weight_r[qi] <= '0;
                queue_tag_r[qi] <= '0;
                queue_time_r[qi] <= '0;
            end
        end else if (ena) begin
            if (push_valid_c) begin
                pending_r[push_idx_c] <= 1'b1;
                queue_weight_r[push_idx_c] <= f_push_weight;
                queue_tag_r[push_idx_c] <= TAG_W'(push_idx_c);
                queue_time_r[push_idx_c] <= '0;
            end

            if (dequeue_valid) begin
                pending_r[dequeue_idx] <= 1'b0;
            end

            worker_pending_r <= worker_start_fire;
            if (worker_start_fire) begin
                worker_pending_idx_r <= start_dispatch_logical_idx;
            end
        end
    end

`ifdef FORMAL
    // Scheduler must only dequeue entries that are actually pending.
    always_ff @(posedge clk) begin
        if (f_past_valid && $past(rst_n) && $past(ena) && $past(dequeue_valid)) begin
            assert($past(pending_r[$past(dequeue_idx)]));
        end
    end

    // Scoreboard consistency: pending and inflight must never overlap.
    genvar gi;
    generate
        for (gi = 0; gi < NEURONS_PER_TILE; gi = gi + 1) begin : g_no_overlap
            always_ff @(posedge clk) begin
                if (rst_n && ena) begin
                    assert(!(pending_r[gi] && formal_logical_event_inflight_r[gi]));
                end
            end
        end
    endgenerate

`ifdef COUPLED_DISPATCH_ENQUEUE
    // Any persistent pending entry is dispatched in bounded time.
    logic [3:0] pending_age_r [0:NEURONS_PER_TILE-1];
    integer ai;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (ai = 0; ai < NEURONS_PER_TILE; ai = ai + 1) begin
                pending_age_r[ai] <= '0;
            end
        end else if (ena) begin
            for (ai = 0; ai < NEURONS_PER_TILE; ai = ai + 1) begin
                if (!pending_r[ai] || formal_logical_event_inflight_r[ai]) begin
                    pending_age_r[ai] <= '0;
                end else if (pending_age_r[ai] != 4'hf) begin
                    pending_age_r[ai] <= pending_age_r[ai] + 4'd1;
                end
                assert(pending_age_r[ai] < 4'd10);
            end
        end
    end
`endif

`ifdef COUPLED_COMMITS_BOUNDED
    // Worker result handoff must commit quickly once produced.
    logic [2:0] commit_age_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            commit_age_r <= '0;
        end else if (ena) begin
            if (worker_result_valid[0] && !worker_result_ready[0]) begin
                commit_age_r <= commit_age_r + 3'd1;
            end else begin
                commit_age_r <= '0;
            end
            assert(commit_age_r < 3'd4);
        end
    end
`endif

`ifdef HEAD_DATA_FRESHNESS
    // Dispatched payload must match the queue head payload selected by index.
    always_ff @(posedge clk) begin
        if (rst_n && ena && formal_dispatch_fire) begin
            assert(worker_start_event_weight == queue_weight_r[formal_dispatch_idx]);
            assert(worker_start_event_tag == queue_tag_r[formal_dispatch_idx]);
            assert(worker_start_event_time == queue_time_r[formal_dispatch_idx]);
        end
    end
`endif

`ifdef REFILL_RACE_HEAD_VALID
    // If dispatch-valid is held due to start backpressure, idx must stay stable.
    always_ff @(posedge clk) begin
        if (f_past_valid && $past(rst_n) && $past(ena) &&
            $past(formal_start_dispatch_valid_r) && !$past(worker_start_ready[0])) begin
            assert(formal_start_dispatch_valid_r);
            assert(formal_start_dispatch_logical_idx_r ==
                   $past(formal_start_dispatch_logical_idx_r));
        end
    end
`endif

`ifdef COUNT_NONZERO_TRACKS
    // If model still has pending work, scheduler must advertise pending state.
    always_ff @(posedge clk) begin
        if (rst_n && ena && (pending_r != '0)) begin
            assert(logical_spike_pending_valid || (formal_logical_event_inflight_r != '0));
        end
    end
`endif
`endif

endmodule

`default_nettype wire
