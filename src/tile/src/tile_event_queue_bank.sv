`default_nettype none

`ifndef YOSYS
import tile_pkg::DATA_W;
import tile_pkg::tile_queue_event_t;
`endif

// -----------------------------------------------------------------------------
// tile_event_queue_bank - shared FF-backed FIFO for tile_queue_event_t spikes
//
// ONE coldfoot_mem_bytelane_sync instance serves all WORKER_CORES_PER_TILE
// independent logical FIFOs. The three byte lanes therefore become three
// shared memory across all workers. The address
// space is partitioned as:
//
//   addr = { worker_idx[WIDX_W-1:0], fifo_offset[PTR_W-1:0] }
//
// Element width is $bits(tile_queue_event_t) = 24 bits, stored as exactly
// three byte lanes:
//   byte[0] = { neuron_y[2:0], neuron_x[4:0] } via packed layout
//   byte[1] = { event_time[5:0], neuron_y[4:3] }
//   byte[2] = weight[7:0]
//
// Exposed depth tuning knob:
//   FIFO_DEPTH_PER_WORKER selects the logical FIFO depth per worker.
//   Example: FIFO_DEPTH_PER_WORKER = 64 -> total depth = 4 x 64 = 256.
//
// Smallest-area GF180 macro tier:
//   All paths use FF-backed coldfoot_mem_bytelane_sync.
//   Example ASIC-lean setting: FIFO_DEPTH_PER_WORKER = 2 -> total logical
//   depth = 4 x 2 = 8, shared among all workers.
//
// Worker service timing note:
//   Once a worker pops a spike from this queue, the dispatch->commit->fanout
//   path takes 3 to 7 cycles in the no-stall case, depending on the neuron's
//   ucode length.
//
// Single-port arbitration: one memory operation (read-refill OR write-push)
// per cycle.  A round-robin grant selects which worker's pending operation
// goes first when multiple workers compete.  Per-worker head_cache registers
// hide the 1-cycle FF read latency.
//
//   ASCII (shared FF memory, 4 workers):
//
//   worker 0 enqueue -->\
//   worker 1 enqueue -->+ round-robin    +-- mem addr={w,offset} --> head_cache[w]
//   worker 2 enqueue -->+ arbiter -------+                              |
//   worker 3 enqueue -->/                +-- push write                  |
//              refills ----------------------------------------- dequeue_valid/data
// -----------------------------------------------------------------------------
(* keep_hierarchy = "no" *)
module tile_event_queue_bank #(
    parameter int unsigned WORKER_CORES_PER_TILE   = 4,
    // Entries per worker logical FIFO. Must be a power of two.
    parameter int unsigned FIFO_DEPTH_PER_WORKER   = 64,
    // Keep this explicit to avoid frontend issues with $bits(type)
    // in ANSI parameter defaults on some formal lanes.
    parameter int unsigned EVENT_W                 = 24,
    parameter int unsigned MEM_STYLE_HINT          = 3
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    // Scalar soft-reset: pulse clears every worker spike FIFO simultaneously.
    input  wire soft_reset_valid,

    // ── Per-worker enqueue (tile_queue_event_t) ──────────────────────────────
    input  wire [WORKER_CORES_PER_TILE-1:0]                          enqueue_valid,
    input  wire [EVENT_W*WORKER_CORES_PER_TILE-1:0]                  enqueue_data,
    output logic [WORKER_CORES_PER_TILE-1:0]                         enqueue_ready,

    // ── Per-worker dequeue (tile_queue_event_t) ──────────────────────────────
    input  wire [WORKER_CORES_PER_TILE-1:0]                          dequeue_ready,
    output logic [WORKER_CORES_PER_TILE-1:0]                         dequeue_valid,
    output logic [EVENT_W*WORKER_CORES_PER_TILE-1:0]                 dequeue_data,

    // not_full[i]: true iff worker i's logical FIFO has slack to accept a
    // new push *if asked*.  This is independent of `enqueue_valid` — unlike
    // `enqueue_ready[i]`, which only asserts on the cycle a push is firing.
    // Consumers that need a "can-this-queue-accept-an-element" predicate
    // (e.g. tile_top's `logical_event_full_c` feeding back into
    // tile_ingress's spike-enqueue gate) must read `not_full`, not
    // `enqueue_ready`; otherwise they form a combinational
    // valid-depends-on-ready loop that latches at zero and deadlocks
    // (the 2026-05 multi_edge_return_addressing wedge).
    output logic [WORKER_CORES_PER_TILE-1:0]                         not_full,

    // mem_stall: asserted while the shared memory is handling an arbitrated
    // operation that is not yet complete for a given worker.
    output logic mem_stall
);
    localparam int unsigned W          = WORKER_CORES_PER_TILE;
    localparam int unsigned WIDX_W     = (W <= 1) ? 1 : $clog2(W);
    localparam int unsigned ELEM_W     = EVENT_W;
    localparam int unsigned BYTE_LANES = (ELEM_W + 7) / 8;             // ceil to byte boundary
    localparam int unsigned PAD_W      = BYTE_LANES * 8;               // always a multiple of 8
    localparam int unsigned PTR_W      = (FIFO_DEPTH_PER_WORKER <= 1) ? 1
                                         : $clog2(FIFO_DEPTH_PER_WORKER);
    localparam int unsigned CNT_W      = $clog2(FIFO_DEPTH_PER_WORKER + 1);
    // Total memory depth = all worker regions concatenated.
    localparam int unsigned MEM_DEPTH = W * FIFO_DEPTH_PER_WORKER;
    localparam int unsigned MEM_ADDR_W = (MEM_DEPTH <= 1) ? 1 : $clog2(MEM_DEPTH);

    // ── Shared memory signals ───────────────────────────────────────────────
    logic [MEM_ADDR_W-1:0] mem_rd_addr;
    logic [PAD_W-1:0]       mem_rd_data;
    logic                   mem_wr_en;
    logic [MEM_ADDR_W-1:0] mem_wr_addr;
    logic [PAD_W-1:0]       mem_wr_data;
    logic                   mem_wait;

    coldfoot_mem_bytelane_sync #(
        .DATA_W    (PAD_W),
        .DEPTH     (MEM_DEPTH),
        .BYTE_LANES(BYTE_LANES),
        .STYLE_HINT(MEM_STYLE_HINT)
    ) u_mem (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (1'b1),
        .rd_addr    (mem_rd_addr),
        .rd_data    (mem_rd_data),
        .wr_en      (mem_wr_en),
        .wr_addr    (mem_wr_addr),
        .wr_data    (mem_wr_data),
        .wr_byte_en ({BYTE_LANES{1'b1}}),
        .waitrequest(mem_wait)
    );

    // ── Per-worker FIFO state ─────────────────────────────────────────────────
    logic [PTR_W-1:0]  head_r  [0:W-1];
    logic [PTR_W-1:0]  tail_r  [0:W-1];
    logic [CNT_W-1:0]  count_r [0:W-1];
    logic [ELEM_W-1:0] cache_r [0:W-1];
    logic              cache_valid_r [0:W-1];
    // Refill FSM per worker: 0=idle  1=addr_issued  2=data_ready
    logic [1:0]        refill_st_r [0:W-1];
    // Which worker's refill read was last issued to memory.
    logic [WIDX_W-1:0] refill_owner_r;

    // ── Arbitration: round-robin grant register ───────────────────────────────
    // push_grant cycles through workers that have a pending push; if none,
    // refill_grant gives priority to the oldest pending refill.
    logic [WIDX_W-1:0] rr_grant_r;   // next candidate for push grants

    // ── Per-worker combinational helpers ─────────────────────────────────────
    logic full_c  [0:W-1];
    logic empty_c [0:W-1];
    logic pop_c   [0:W-1];
    tile_queue_event_t enq_c [0:W-1];

    // Which worker wins the memory port this cycle (one read OR one write).
    logic               arb_push_valid_c;
    logic [WIDX_W-1:0] arb_push_winner_c;
    logic               arb_refill_valid_c;
    logic [WIDX_W-1:0] arb_refill_winner_c;
    // Refill takes priority over push (empty head_cache stalls dequeue).
    logic               mem_op_is_read_c;
    logic [WIDX_W-1:0] mem_op_worker_c;
    logic               mem_read_fire_c;
    logic               mem_push_fire_c;

    // ── Build per-worker metadata ─────────────────────────────────────────────
    always_comb begin
        for (int i = 0; i < W; i++) begin
            enq_c[i]   = tile_queue_event_t'(enqueue_data[i*ELEM_W +: ELEM_W]);
            full_c[i]  = (count_r[i] == CNT_W'(FIFO_DEPTH_PER_WORKER));
            empty_c[i] = (count_r[i] == '0);

            // Pop: only when cache is valid and FIFO is idle and no flush.
            pop_c[i] = dequeue_ready[i] && cache_valid_r[i]
                       && (refill_st_r[i] == 2'd0) && !soft_reset_valid && ena;

            enqueue_ready[i] = mem_push_fire_c && (arb_push_winner_c == WIDX_W'(i));
            // `not_full` mirrors `full_c` (the internal headroom check)
            // without the firing/arbitration gate so producers can use it
            // as a stable "is there slack?" predicate.  Gated on
            // `!soft_reset_valid` so a soft-reset window reads as
            // not-yet-accepting (matches the `enqueue_ready=0` behaviour
            // during reset).
            not_full[i]      = !full_c[i] && !soft_reset_valid;
            dequeue_valid[i] = cache_valid_r[i] && (refill_st_r[i] == 2'd0)
                               && !soft_reset_valid;
            dequeue_data[i*ELEM_W +: ELEM_W] = cache_r[i];
        end
    end

    // ── Arbitration: find push winner (round-robin starting at rr_grant_r) ───
    always_comb begin
        int arb_idx;
        arb_push_valid_c  = 1'b0;
        arb_push_winner_c = '0;
        for (int arb_k = 0; arb_k < W; arb_k = arb_k + 1) begin
            arb_idx = int'(rr_grant_r) + arb_k;
            if (arb_idx >= W)
                arb_idx = arb_idx - W;
            if (!arb_push_valid_c
                && enqueue_valid[arb_idx] && !full_c[arb_idx] && !mem_wait && !soft_reset_valid && ena) begin
                arb_push_valid_c  = 1'b1;
                arb_push_winner_c = WIDX_W'(arb_idx);
            end
        end

        // Refill winner: any worker in state addr_issued (waiting on mem data).
        arb_refill_valid_c  = 1'b0;
        arb_refill_winner_c = refill_owner_r;
        for (int arb_k = 0; arb_k < W; arb_k = arb_k + 1) begin
            if (!arb_refill_valid_c && (refill_st_r[arb_k] == 2'd1)) begin
                arb_refill_valid_c  = 1'b1;
                arb_refill_winner_c = WIDX_W'(arb_k);
            end
        end

        // Choose memory operation: refill read if any pending, else push write.
        mem_op_is_read_c = arb_refill_valid_c;
        mem_op_worker_c  = arb_refill_valid_c ? arb_refill_winner_c : arb_push_winner_c;

        // Read address: worker region base + head pointer.
        // Write address: worker region base + tail pointer.
        mem_rd_addr = MEM_ADDR_W'({mem_op_worker_c, head_r[mem_op_worker_c]});
        mem_read_fire_c = arb_refill_valid_c && !mem_wait && !soft_reset_valid && ena;
        mem_push_fire_c = arb_push_valid_c && !arb_refill_valid_c
                   && !mem_wait && !soft_reset_valid && ena;
        mem_wr_en   = mem_push_fire_c;
        mem_wr_addr = MEM_ADDR_W'({arb_push_winner_c, tail_r[arb_push_winner_c]});
        mem_wr_data = PAD_W'(enq_c[arb_push_winner_c]);
    end

    // mem_stall: any worker has a pending refill not yet committed to cache.
    always_comb begin
        mem_stall = mem_wait;
        for (int i = 0; i < W; i++) mem_stall |= (refill_st_r[i] != 2'd0);
    end

    // ── Sequential: per-worker FIFO control + refill pipeline ────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_grant_r    <= '0;
            refill_owner_r <= '0;
            for (int i = 0; i < W; i++) begin
                head_r[i]        <= '0;
                tail_r[i]        <= '0;
                count_r[i]       <= '0;
                cache_r[i]       <= '0;
                cache_valid_r[i] <= 1'b0;
                refill_st_r[i]   <= 2'd0;
            end
        end else begin
            if (soft_reset_valid) begin
                for (int i = 0; i < W; i++) begin
                    head_r[i]        <= '0;
                    tail_r[i]        <= '0;
                    count_r[i]       <= '0;
                    cache_r[i]       <= '0;
                    cache_valid_r[i] <= 1'b0;
                    refill_st_r[i]   <= 2'd0;
                end
            end else if (ena) begin

                // Advance round-robin when a push was granted.
                if (mem_push_fire_c)
                    rr_grant_r <= WIDX_W'((int'(arb_push_winner_c) + 1) % W);

                // ── Refill pipeline ───────────────────────────────────────────
                // State 1 (addr_issued): mem will produce data next cycle.
                // State 2 (data_ready): latch mem_rd_data into cache.
                for (int i = 0; i < W; i++) begin
                    case (refill_st_r[i])
                        2'd1: begin
                            if (mem_read_fire_c && (int'(arb_refill_winner_c) == i)) begin
                                refill_owner_r <= WIDX_W'(i);
                                refill_st_r[i] <= 2'd2;
                            end
                        end
                        2'd2: begin
                            cache_r[i]       <= mem_rd_data[ELEM_W-1:0];
                            cache_valid_r[i] <= 1'b1;
                            refill_st_r[i]   <= 2'd0;
                        end
                        default: ;
                    endcase
                end

                // ── Per-worker pop / push ─────────────────────────────────────
                for (int i = 0; i < W; i++) begin
                    if (pop_c[i]) begin
                        cache_valid_r[i] <= 1'b0;
                        head_r[i]        <= head_r[i] + PTR_W'(1);
                        count_r[i]       <= count_r[i] - CNT_W'(1)
                                            + ((arb_push_valid_c
                                                && mem_push_fire_c
                                                && (int'(arb_push_winner_c) == i))
                                                ? CNT_W'(1) : '0);
                        if (count_r[i] > CNT_W'(1)) begin
                            // More entries remain → start refill.
                            refill_st_r[i] <= 2'd1;
                        end else if (arb_push_valid_c
                                     && mem_push_fire_c
                                     && (int'(arb_push_winner_c) == i)) begin
                            // count==1 + simultaneous push: bypass into cache.
                            cache_r[i]       <= enq_c[i];
                            cache_valid_r[i] <= 1'b1;
                        end
                    end

                    if (arb_push_valid_c
                        && mem_push_fire_c
                        && (int'(arb_push_winner_c) == i)) begin
                        tail_r[i] <= tail_r[i] + PTR_W'(1);
                        if (!pop_c[i]) begin
                            count_r[i] <= count_r[i] + CNT_W'(1);
                            if (empty_c[i]) begin
                                // Empty FIFO: bypass push data into cache.
                                cache_r[i]       <= enq_c[i];
                                cache_valid_r[i] <= 1'b1;
                            end
                        end
                    end
                end

            end
        end
    end

endmodule

`default_nettype wire
