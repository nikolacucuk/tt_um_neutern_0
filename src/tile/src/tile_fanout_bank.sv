`default_nettype none


// -----------------------------------------------------------------------------
// tile_fanout_bank - shared FF-backed FIFO for compact tile_fanout_spike_t output spikes
//
// One coldfoot_mem_bytelane_sync (byte-padded to tile_fanout_spike_t width)
// is shared across ALL workers (FF-backed coldfoot_mem_bytelane_sync).
// across all workers on the GF180 macro path.
// Address layout: { worker_idx[WIDX_W-1:0], fifo_offset[PTR_W-1:0] }
// Total depth = WORKER_CORES_PER_TILE * FIFO_DEPTH_PER_WORKER.
//
//   tile_fanout_spike_t is a four-tile staging payload:
//     queue_spike[23:0] + tile_y[0] + tile_x[0] = 26 bits
//   This maps to four byte lanes after padding.  The general tile_out_spike_t
//   remains available for wider meshes and protocol-facing payloads.
//
// Exposed depth tuning knob:
//   FIFO_DEPTH_PER_WORKER selects the logical FIFO depth per worker.
//   Example: FIFO_DEPTH_PER_WORKER = 64 -> total depth = 4 x 64 = 256.
//
// Smallest-area GF180 macro tier:
//   All paths use FF-backed coldfoot_mem_bytelane_sync.
//
// NOTE: enqueue/dequeue are currently stubbed to 0 in tile_top pending
//       tile_fanout_executor migration.  See follow-up PR.
//
// Arbitration:
//   Read port  - round-robin among workers needing refill (after dequeue).
//   Write port - round-robin among workers wanting to push (enqueue).
//   Both ports operate simultaneously each cycle (1r1w different addresses).
//
//   ASCII (shared FF memory, 4 workers):
//
//   enqueue_valid[w]/data[w] --> RR write arbiter --> mem write port
//   mem read port --> RR read arbiter --> per-worker head_cache
//   per-worker head_cache --> dequeue_valid[w]/data[w]
// -----------------------------------------------------------------------------
(* keep_hierarchy = "no" *)
`ifndef YOSYS
import tile_pkg::*;
`endif
module tile_fanout_bank
  #(
    parameter int unsigned WORKER_CORES_PER_TILE  = 4,
    // Entries per worker logical FIFO. Must be a power of two.
    parameter int unsigned FIFO_DEPTH_PER_WORKER  = 64,
    parameter int unsigned MEM_STYLE_HINT         = 3
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    // graph_state_clear: synchronous clear — flush all worker FIFOs.
    input  wire graph_state_clear,

    // ── Per-worker enqueue (tile_fanout_spike_t) ─────────────────────────────
    input  wire [WORKER_CORES_PER_TILE-1:0]                              enqueue_valid,
    input  wire [FANOUT_SPIKE_W*WORKER_CORES_PER_TILE-1:0]              enqueue_data,
    output logic [WORKER_CORES_PER_TILE-1:0]                             enqueue_ready,

    // ── Per-worker dequeue (tile_fanout_spike_t) ─────────────────────────────
    input  wire [WORKER_CORES_PER_TILE-1:0]                              dequeue_ready,
    output logic [WORKER_CORES_PER_TILE-1:0]                             dequeue_valid,
    output logic [FANOUT_SPIKE_W*WORKER_CORES_PER_TILE-1:0]             dequeue_data,

    // mem_stall: STYLE_HINT=3 always 0; kept for interface compatibility.
    output logic mem_stall
);
    localparam int unsigned W          = WORKER_CORES_PER_TILE;
    localparam int unsigned ELEM_W     = FANOUT_SPIKE_W;  // = $bits(tile_fanout_spike_t)
    localparam int unsigned PAD_W      = ((ELEM_W + 7) / 8) * 8;
    localparam int unsigned BYTE_LANES = PAD_W / 8;
    localparam int unsigned PTR_W      = (FIFO_DEPTH_PER_WORKER <= 1) ? 1
                                         : $clog2(FIFO_DEPTH_PER_WORKER);
    localparam int unsigned CNT_W      = $clog2(FIFO_DEPTH_PER_WORKER + 1);
    localparam int unsigned WIDX_W     = (W <= 1) ? 1 : $clog2(W);
    localparam int unsigned MEM_DEPTH = W * FIFO_DEPTH_PER_WORKER;
    localparam int unsigned MEM_ADDR_W = $clog2(MEM_DEPTH);

    // ── Shared FF memory ──────────────────────────────────────────────────────
    logic [MEM_ADDR_W-1:0] mem_rd_addr_c;
    logic [PAD_W-1:0]       mem_rd_data;
    logic                   mem_wr_en_c;
    logic [MEM_ADDR_W-1:0] mem_wr_addr_c;
    logic [PAD_W-1:0]       mem_wr_data_c;

    coldfoot_mem_bytelane_sync #(
        .DATA_W    (PAD_W),
        .DEPTH     (MEM_DEPTH),
        .BYTE_LANES(BYTE_LANES),
        .STYLE_HINT(MEM_STYLE_HINT)
    ) u_mem (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (1'b1),
        .rd_addr    (mem_rd_addr_c),
        .rd_data    (mem_rd_data),
        .wr_en      (mem_wr_en_c),
        .wr_addr    (mem_wr_addr_c),
        .wr_data    (mem_wr_data_c),
        .wr_byte_en ({BYTE_LANES{1'b1}}),
        .waitrequest(mem_stall)
    );

    // mem_stall: waitrequest is always 0 for STYLE_HINT=3 (synchronous macros).
    assign mem_stall = 1'b0;

    // ── Per-worker FIFO state ─────────────────────────────────────────────────
    logic [PTR_W-1:0]  head_r         [0:W-1];
    logic [PTR_W-1:0]  tail_r         [0:W-1];
    logic [CNT_W-1:0]  count_r        [0:W-1];
    logic [ELEM_W-1:0] cache_r        [0:W-1];
    logic              cache_valid_r  [0:W-1];
    // Refill FSM per worker: 2'd0=idle  2'd1=addr_issued  2'd2=data_ready
    logic [1:0]        refill_st_r    [0:W-1];

    // Which worker's refill read is currently in the memory pipeline (1-cycle).
    logic [WIDX_W-1:0] rd_inflight_r;
    logic              rd_inflight_valid_r;

    // Round-robin write grant pointer and read grant pointer.
    logic [WIDX_W-1:0] rr_wr_r;
    logic [WIDX_W-1:0] rr_rd_r;

    // ── Per-worker combinational ─────────────────────────────────────────────
    logic              full_c         [0:W-1];
    logic              empty_c        [0:W-1];
    logic              pop_c          [0:W-1];
    logic              push_want_c    [0:W-1];  // worker wants to push
    logic              refill_want_c  [0:W-1];  // worker needs refill read
    tile_fanout_spike_t enq_c         [0:W-1];

    // Write and read grant signals.
    logic [WIDX_W-1:0] wr_grant_c;
    logic              wr_grant_valid_c;
    logic              wr_fire_c;
    logic [WIDX_W-1:0] rd_grant_c;
    logic              rd_grant_valid_c;

    // ── Combinational: per-worker wants ──────────────────────────────────────
    integer i;
    integer w;
    always_comb begin
        for (w = 0; w < W; w++) begin
            enq_c[w]         = tile_fanout_spike_t'(enqueue_data[w*ELEM_W +: ELEM_W]);
            full_c[w]        = (count_r[w] == CNT_W'(FIFO_DEPTH_PER_WORKER));
            empty_c[w]       = (count_r[w] == '0);
            pop_c[w]         = dequeue_ready[w] && cache_valid_r[w]
                               && (refill_st_r[w] == 2'd0)
                               && !graph_state_clear && ena;
            push_want_c[w]   = enqueue_valid[w] && !full_c[w]
                               && !graph_state_clear && ena;
            // Worker needs refill when: idle, cache invalid, count > 0.
            refill_want_c[w] = !cache_valid_r[w] && (count_r[w] > '0)
                               && (refill_st_r[w] == 2'd0)
                               && !graph_state_clear;
            enqueue_ready[w] = wr_fire_c && (wr_grant_c == WIDX_W'(w));
            dequeue_valid[w] = cache_valid_r[w] && (refill_st_r[w] == 2'd0)
                               && !graph_state_clear;
            dequeue_data[w*ELEM_W +: ELEM_W] = cache_r[w];
        end
    end

    // ── Write arbitration: round-robin among workers wanting to push ──────────
    always_comb begin : wr_arb
        wr_grant_c       = '0;
        wr_grant_valid_c = 1'b0;
        for (i = 0; i < W; i++) begin
            w = (int'(rr_wr_r) + i) % W;
            if (push_want_c[w] && !wr_grant_valid_c) begin
                wr_grant_c       = WIDX_W'(w);
                wr_grant_valid_c = 1'b1;
            end
        end
        wr_fire_c      = wr_grant_valid_c && !mem_stall && !graph_state_clear && ena;
        mem_wr_en_c   = wr_fire_c;
        mem_wr_addr_c = MEM_ADDR_W'({wr_grant_c, tail_r[wr_grant_c]});
        mem_wr_data_c = PAD_W'(enq_c[wr_grant_c]);
    end

    // ── Read arbitration: round-robin among workers needing refill ────────────
    // Refill reads take priority; only issue a new read when the pipeline
    // is not already occupied (rd_inflight_valid_r == 0).
    always_comb begin : rd_arb
        rd_grant_c       = '0;
        rd_grant_valid_c = 1'b0;
        if (!rd_inflight_valid_r) begin
            for (i = 0; i < W; i++) begin
                w = (int'(rr_rd_r) + i) % W;
                if (refill_want_c[w] && !rd_grant_valid_c) begin
                    rd_grant_c       = WIDX_W'(w);
                    rd_grant_valid_c = 1'b1;
                end
            end
        end
        mem_rd_addr_c = MEM_ADDR_W'({rd_grant_c, head_r[rd_grant_c]});
    end

    // ── Sequential: per-worker FIFO state and shared arb registers ───────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_wr_r             <= '0;
            rr_rd_r             <= '0;
            rd_inflight_r       <= '0;
            rd_inflight_valid_r <= 1'b0;
            for (w = 0; w < W; w++) begin
                head_r[w]        <= '0;
                tail_r[w]        <= '0;
                count_r[w]       <= '0;
                cache_r[w]       <= '0;
                cache_valid_r[w] <= 1'b0;
                refill_st_r[w]   <= 2'd0;
            end
        end else begin
            // ── Advance write RR on every granted push ──────────────────────
            if (wr_fire_c) begin
                rr_wr_r <= (rr_wr_r == WIDX_W'(W-1)) ? '0 : rr_wr_r + WIDX_W'(1);
            end

            // ── Track read in-flight ────────────────────────────────────────
            if (rd_grant_valid_c) begin
                rd_inflight_r       <= rd_grant_c;
                rd_inflight_valid_r <= 1'b1;
                rr_rd_r <= (rd_grant_c == WIDX_W'(W-1)) ? '0 : rd_grant_c + WIDX_W'(1);
            end else begin
                rd_inflight_valid_r <= 1'b0;
            end

            // ── Per-worker: graph_state_clear reset ──────────────────────────
            for (w = 0; w < W; w++) begin
                if (graph_state_clear) begin
                    head_r[w]        <= '0;
                    tail_r[w]        <= '0;
                    count_r[w]       <= '0;
                    cache_r[w]       <= '0;
                    cache_valid_r[w] <= 1'b0;
                    refill_st_r[w]   <= 2'd0;
                end else if (ena) begin
                    if (rd_grant_valid_c && (rd_grant_c == WIDX_W'(w))) begin
                        refill_st_r[w] <= 2'd1;
                    end

                    // ── Refill FSM ─────────────────────────────────────────
                    if (rd_inflight_valid_r && (rd_inflight_r == WIDX_W'(w))) begin
                        // Memory data arrives; load into cache.
                        cache_r[w]       <= ELEM_W'(mem_rd_data);
                        cache_valid_r[w] <= 1'b1;
                        refill_st_r[w]   <= 2'd0;
                    end
                    // Note: refill_st_r transitions to 2'd1 are managed by
                    // rd_grant_valid_c; we clear it when data arrives above.
                    // The grant fires when refill_want_c[w] is high.

                    // ── Pop ───────────────────────────────────────────────
                    if (pop_c[w]) begin
                        cache_valid_r[w] <= 1'b0;
                        head_r[w]        <= head_r[w] + PTR_W'(1);
                        count_r[w]       <= count_r[w] - CNT_W'(1)
                                           + ((wr_grant_c == WIDX_W'(w) && wr_fire_c)
                                              ? CNT_W'(1) : '0);
                        // Simultaneous pop+push bypass when count==1.
                        if ((count_r[w] == CNT_W'(1)) &&
                            wr_grant_c == WIDX_W'(w) &&
                            wr_fire_c) begin
                            cache_r[w]       <= enq_c[w];
                            cache_valid_r[w] <= 1'b1;
                        end
                        // Trigger refill on next cycle via refill_want_c.
                        if (count_r[w] > CNT_W'(1))
                            refill_st_r[w] <= 2'd1;
                    end

                    // ── Push ─────────────────────────────────────────────
                    if (wr_grant_c == WIDX_W'(w) && wr_fire_c) begin
                        tail_r[w] <= tail_r[w] + PTR_W'(1);
                        if (!pop_c[w]) begin
                            count_r[w] <= count_r[w] + CNT_W'(1);
                            if (empty_c[w]) begin
                                cache_r[w]       <= enq_c[w];
                                cache_valid_r[w] <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

    // ── Legacy: for-generate stub (interface unchanged) ──────────────────────
    // NOTE: The generate loop below is retained for the old FIFO registers
    // approach. In this shared-memory design all logic is at module scope above.
    // The generate block is gone; per-worker logic is in the always_ff above.

endmodule

`default_nettype wire
