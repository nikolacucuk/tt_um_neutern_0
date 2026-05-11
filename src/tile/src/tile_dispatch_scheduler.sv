`default_nettype none

`include "tile_flit_types.vh"
// tile_dispatch_scheduler
// ---------------------------------------------------------------------------
// Per-tile logical-spike ready-FIFO scheduler.  Replaces the original
// O(NEURONS_PER_TILE) combinational mask-scan ("find_next_ready_candidate")
// with a true FIFO of ready neuron indices, keyed off edge-triggered
// enqueue from the pending-mask and a commit-side re-enqueue hook.
//
// Invariants preserved from the previous design (these encode the
// per-neuron chronological-order contract the mesh depends on for correct
// LIF dynamics — do NOT weaken):
//
//   1. At most one spike per neuron inflight at a time. Tracked via
//      `logical_event_inflight_r[NEURONS_PER_TILE]`.  Do NOT shard
//      per-worker.
//   2. No duplicate enqueue into the ready FIFO.  Tracked via
//      `in_ready_fifo_r[NEURONS_PER_TILE]`.
//   3. Worker commit gate unchanged:
//        worker_commit_ready = !host_reserve_current && fanout_queue_can_accept
//      (the fanout_queue_can_accept term only applies when the committing
//      worker is spiking AND owns a fanout).  This is the MSG_OUTPUT wedge
//      invariant from docs/debugging_msg_output_wedge.md — keep exact.
//   4. `logical_event_inflight_r[N]` clears on worker_commit_valid
//      (independent of ready).  Side effects (context write, host/fanout
//      push) still gate on valid && ready.  This is the 2026-04-20 patch
//      from the MSG_OUTPUT wedge investigation.
//
// Resource motivation: the prior O(N) scan was the biggest single LUT
// consumer in the tile (~2.9k LUTs x 4 tiles for N=128).  An explicit
// FIFO + 128-wide priority encoder is O(log N) depth and ~10x smaller.
(* keep_hierarchy = "no" *)
module tile_dispatch_scheduler #(
    parameter int unsigned NEURONS_PER_TILE = 2,
    parameter int unsigned WORKER_CORES_PER_TILE = 1,
    parameter int unsigned FANOUT_POOL_DEPTH = 256,
    // FANOUT_QUEUE_DEPTH must match the depth in tile_fanout_executor.  It
    // is consumed here for credit-based dispatch throttling: a new
    // has_fanout dispatch can only transition pending -> valid when
    //   fanout_queue_count_live + popcount(worker_has_fanout_r) < DEPTH
    // holds.  That invariant, maintained by the throttle, guarantees
    // worker_commit_ready never stalls on fanout_queue-full — which
    // breaks the fanout_queue <-> event_queue back-pressure cycle that
    // wedges the compute pipeline.
    parameter int unsigned FANOUT_QUEUE_DEPTH = 4,
    parameter int unsigned SYNAPSE_SID_W = 5,
    // Unused by this module, but required in the parameter scope so
    // Vivado can elaborate the `local_core_id_for_index` and
    // `local_target_valid_for_core_id` helpers brought in by the
    // `\`include "tile_pkg.svh"` directive below.  Those helpers are
    // invoked only by the other three tile submodules; this module
    // calls `onehot_for_idx` only.  Default 0 avoids forcing every
    // instantiation site to pass it.
    parameter int unsigned LOCAL_Z = 0
) (
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               ena,
    input  wire                               tile_graph_state_clear,

    // Scalar soft-reset (from state_bank directly; replaces N-wide
    // soft_reset_pulse + internal priority encoder).
    input  wire                               soft_reset_valid_in,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                              soft_reset_idx_in,
    output wire                               soft_reset_valid,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                              soft_reset_idx,

    // --- Spike queue bank: scalar pending/full + head-read handshake ---
    input  wire                               logical_spike_pending_valid,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              logical_spike_pending_idx,
    input  wire                               logical_spike_full,
    input  wire signed [WEIGHT_W-1:0]            logical_spike_head_read_weight,
    input  wire [TAG_W-1:0]                      logical_spike_head_read_tag,
    input  wire [EVENT_TIME_W-1:0]               logical_spike_head_read_time,
    output wire                               dequeue_valid,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              dequeue_idx,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              head_read_idx,

    // --- State bank dispatch-read port (drives state_bank.dispatch_read_idx) ---
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              state_dispatch_read_idx,
    input  wire [4:0]                         state_dispatch_ucode_ptr,
    input  wire [4:0]                         state_dispatch_ucode_len,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                              state_dispatch_fanout_len,
    input  wire                               noc_egress_en,

    // --- Worker start interface ---
    output wire [WORKER_CORES_PER_TILE-1:0]   worker_start_valid,
    input  wire [WORKER_CORES_PER_TILE-1:0]   worker_start_ready,
    output wire [TAG_W-1:0]                   worker_start_event_tag,
    output wire [EVENT_TIME_W-1:0]            worker_start_event_time,
    output wire signed [WEIGHT_W-1:0]         worker_start_event_weight,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              start_dispatch_logical_idx,
    output wire                               start_dispatch_valid_out,
    output wire [((WORKER_CORES_PER_TILE <= 1) ? 1 :
                  $clog2(WORKER_CORES_PER_TILE))-1:0]
                                              start_dispatch_worker_idx_out,
    output wire                               worker_start_fire,

    // --- Worker result bundle → commit arbitration / context-commit ---
    input  wire [WORKER_CORES_PER_TILE-1:0]   worker_result_valid,
    output wire [WORKER_CORES_PER_TILE-1:0]   worker_result_ready,
    // Flat-packed worker result vectors (Yosys/synthesis compatible).
    // Internal generate block unpacks these back into per-worker arrays.
    input  wire [((WORKER_CORES_PER_TILE <= 1) ? 1 : WORKER_CORES_PER_TILE) *
                 (((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE)))-1:0]
                                              worker_result_logical_idx,
    input  wire [((WORKER_CORES_PER_TILE <= 1) ? 1 : WORKER_CORES_PER_TILE) *
                 NEURON_EXEC_CTX_W-1:0]
                                              worker_result_ctx,
    input  wire [((WORKER_CORES_PER_TILE <= 1) ? 1 : WORKER_CORES_PER_TILE) *
                 NEURON_EMIT_W-1:0]
                                              worker_result_emit,

    // --- Commit outputs (to fanout_executor and host_io) ---
    output wire                               worker_commit_valid,
    output wire                               worker_commit_ready,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              worker_commit_logical_idx,
    output wire [((WORKER_CORES_PER_TILE <= 1) ? 1 :
                  $clog2(WORKER_CORES_PER_TILE))-1:0]
                                              worker_commit_idx,
    output wire [WORKER_CORES_PER_TILE-1:0]   worker_has_fanout_snapshot,

    // --- Context commit to logical_neuron_context_bank (via tile_top) ---
    // rv_if carrying context_commit_t = { idx, ctx }.
    // rv_if.valid = context_commit_valid; tile_top ties ready = 1.
    rv_if.tx                                  context_commit,

    // --- Commit-gate inputs from ingress (host_reserve) and fanout_executor ---
    input  wire                               host_reserve_current,
    input  wire                               fanout_queue_can_accept,
    // Live occupancy of the fanout_queue, used for credit-based dispatch
    // throttling below.  See FANOUT_QUEUE_DEPTH parameter comment.
    input  wire [((FANOUT_QUEUE_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_QUEUE_DEPTH + 1))-1:0]
                                              fanout_queue_count_live
`ifdef FORMAL
    ,
    // Formal-only passthrough of internal state.  Yosys's built-in Verilog
    // frontend does not reliably resolve hierarchical references into a DUT
    // for BMC, so we expose the bits the tile_dispatch_scheduler_formal
    // harness proves invariants over as ordinary output ports.
    output wire [((WORKER_CORES_PER_TILE <= 1) ? 1 :
                  $clog2(WORKER_CORES_PER_TILE))-1:0]
                                              formal_worker_rr_ptr_r,
    output wire [NEURONS_PER_TILE-1:0]        formal_logical_event_inflight_r,
    output wire [NEURONS_PER_TILE-1:0]        formal_in_ready_fifo_r,
    output wire                               formal_ready_push,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              formal_ready_push_idx,
    output wire                               formal_dispatch_fire,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              formal_dispatch_idx,
    output wire                               formal_prev_pending_valid_r,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              formal_prev_pending_idx_r,
    output wire                               formal_start_dispatch_pending_r,
    output wire                               formal_start_dispatch_valid_r,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              formal_start_dispatch_logical_idx_r,
    // FIFO-full marker used by NO_ORPHAN to account for back-pressure.
    // When `ready_count_r == READY_FIFO_DEPTH`, pushes are gated off
    // and a pending-but-eligible neuron may not be in (inflight /
    // in_ready_fifo / being_pushed / in_pipeline) this cycle — it is
    // correctly waiting for a FIFO slot to open.  Needed once
    // READY_FIFO_DEPTH dropped below NEURONS_PER_TILE in Wave F.
    output wire                               formal_ready_fifo_full,
    // Decoded one-hot of the registered ready-push token decided one cycle ago.
    // Stage-A/B pipeline on ready_push moves the in_ready_fifo_r
    // update one cycle later than the decision; NO_ORPHAN needs to
    // observe this "in-flight push" window to remain complete.
    output wire [NEURONS_PER_TILE-1:0]        formal_ready_push_mask_r,
    // Registered pop reservation used to keep a just-popped neuron out
    // of the next enqueue-selection cycle without same-cycle feedback.
    output wire                               formal_pop_reservation_valid_r,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                              formal_pop_reservation_idx_r
`endif
);

    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE);
    localparam int unsigned WORKER_IDX_W =
        (WORKER_CORES_PER_TILE <= 1) ? 1 : $clog2(WORKER_CORES_PER_TILE);
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    // neuron_exec_ctx_t = 48 bits, neuron_emit_t = 15 bits.
    localparam int unsigned EXEC_CTX_W = NEURON_EXEC_CTX_W;  // = $bits(neuron_exec_ctx_t) = 48
    localparam int unsigned EMIT_W     = NEURON_EMIT_W;      // = $bits(neuron_emit_t) = 15

    // ---------------------------------------------------------------------
    // READY_FIFO_DEPTH rationale
    // ---------------------------------------------------------------------
    // The ready FIFO is a staging buffer between "pending-and-eligible"
    // (masked by the dedup bitmap `in_ready_fifo_r`) and the worker
    // dispatch.  Its depth is NOT a capacity on how many neurons can be
    // pending — that is bounded by NEURONS_PER_TILE via the pending mask
    // held in `tile_event_queue_bank`.  Depth here is only the staging
    // slots between "picked up off the pending mask" and "latched into a
    // worker".
    //
    // Sizing: WORKER_CORES_PER_TILE work items can be in-flight at once
    // (logical_event_inflight_r gate); push rate is 1/cycle (commit-requeue or
    // edge-push, mutex); pop rate is 1/cycle (worker_start_fire).  So
    // the steady-state occupancy is bounded by the worker count.  A
    // depth of 4×WORKER_CORES_PER_TILE is comfortable head-room for
    // transient spike bursts.  If the FIFO ever fills, pushes back off
    // (`ready_push_c` is gated on `ready_count_r < DEPTH`) and the
    // pending neuron waits one more cycle — correctness is preserved
    // because `enqueue_candidate_mask_c` is level-sensitive.
    //
    // An early draft set DEPTH = NEURONS_PER_TILE, which correctly
    // eliminates overflow but creates a 128:1 mux at the read head
    // (`ready_fifo_r[ready_rd_ptr_r]`) — O(N) combinational depth,
    // defeating the whole point of the ready-FIFO redesign.  The small
    // depth keeps that mux at log2(DEPTH) LUT levels.
    // ---------------------------------------------------------------------
    localparam int unsigned READY_FIFO_DEPTH =
        (WORKER_CORES_PER_TILE * 4 < 8) ? 8 : WORKER_CORES_PER_TILE * 4;
    localparam int unsigned READY_FIFO_PTR_W =
        (READY_FIFO_DEPTH <= 1) ? 1 : $clog2(READY_FIFO_DEPTH);
    localparam int unsigned READY_FIFO_COUNT_W =
        $clog2(READY_FIFO_DEPTH + 1);

    // ---------------------------------------------------------------------
    // Shared helpers (onehot_for_idx etc.).  MUST be included at module
    // scope so Vivado accepts the module-parameter references inside the
    // function bodies.
    // ---------------------------------------------------------------------
    `include "tile_pkg.svh"

    // ---------------------------------------------------------------------
    // Small combinational helpers.
    // ---------------------------------------------------------------------

    // 4-wide (typical) priority encoder over worker_ready mask, rotated by
    // start_idx to implement fair round-robin.  WORKER_CORES_PER_TILE is
    // small (4 typical, max 8) so an unrolled scan is cheaper than any
    // structured encoder.
    function automatic [WORKER_IDX_W-1:0] find_worker_rr(
        input logic [WORKER_CORES_PER_TILE-1:0] ready_mask,
        input logic [WORKER_IDX_W-1:0]          start_idx,
        output logic                            found_fw
    );
        integer k_fw;
        integer probe;
        begin
            found_fw = 1'b0;
            find_worker_rr = start_idx;
            for (k_fw = 0; k_fw < WORKER_CORES_PER_TILE; k_fw = k_fw + 1) begin
                probe = (start_idx + k_fw) % WORKER_CORES_PER_TILE;
                if (!found_fw && ready_mask[probe[WORKER_IDX_W-1:0]]) begin
                    found_fw = 1'b1;
                    find_worker_rr = probe[WORKER_IDX_W-1:0];
                end
            end
        end
    endfunction

    // Increment a FIFO pointer with wrap at READY_FIFO_DEPTH.
    function automatic [READY_FIFO_PTR_W-1:0] next_ready_ptr(
        input logic [READY_FIFO_PTR_W-1:0] ptr
    );
        begin
            if (READY_FIFO_DEPTH <= 1) begin
                next_ready_ptr = '0;
            end else if (ptr == READY_FIFO_PTR_W'(READY_FIFO_DEPTH - 1)) begin
                next_ready_ptr = '0;
            end else begin
                next_ready_ptr = ptr + READY_FIFO_PTR_W'(1);
            end
        end
    endfunction

    // Lowest-set-bit priority encoder over an NEURONS_PER_TILE-wide mask.
    // Synthesis tools (Vivado, Yosys) recognise this pattern and map to
    // LUT6-chain or carry-chain OR; O(log N) depth.
    function automatic [NEURON_IDX_W-1:0] lowest_set_neuron(
        input  logic [NEURONS_PER_TILE-1:0] mask,
        output logic                        found_ls
    );
        integer k_ls;
        begin
            found_ls = 1'b0;
            lowest_set_neuron = '0;
            for (k_ls = 0; k_ls < NEURONS_PER_TILE; k_ls = k_ls + 1) begin
                if (!found_ls && mask[k_ls]) begin
                    found_ls = 1'b1;
                    lowest_set_neuron = k_ls[NEURON_IDX_W-1:0];
                end
            end
        end
    endfunction

    // ---------------------------------------------------------------------
    // Registers owned by this module.
    // ---------------------------------------------------------------------
    // Yosys-friendly shim: unpack flat-packed vectors into per-worker arrays.
    logic [NEURON_IDX_W-1:0] worker_result_logical_idx_u [0:WORKER_CORES_PER_TILE-1];
    neuron_exec_ctx_t        worker_result_ctx_u         [0:WORKER_CORES_PER_TILE-1];
    neuron_emit_t            worker_result_emit_u        [0:WORKER_CORES_PER_TILE-1];
    integer                  unpack_wr_i;
    integer                  w;
    integer                  k_rst;

    always_comb begin
        for (unpack_wr_i = 0; unpack_wr_i < WORKER_CORES_PER_TILE; unpack_wr_i = unpack_wr_i + 1) begin
            worker_result_logical_idx_u[unpack_wr_i] =
                worker_result_logical_idx[unpack_wr_i*NEURON_IDX_W +: NEURON_IDX_W];
            worker_result_ctx_u[unpack_wr_i] =
                neuron_exec_ctx_t'(worker_result_ctx[unpack_wr_i*EXEC_CTX_W +: EXEC_CTX_W]);
            worker_result_emit_u[unpack_wr_i] =
                neuron_emit_t'(worker_result_emit[unpack_wr_i*EMIT_W +: EMIT_W]);
        end
    end


    // Start-dispatch pipeline (two-cycle: capture -> latch event).
    logic                            start_dispatch_valid_r;
    logic                            start_dispatch_pending_r;
    logic [WORKER_IDX_W-1:0]         start_dispatch_worker_idx_r;
    logic [NEURON_IDX_W-1:0]         start_dispatch_logical_idx_r;
    logic [TAG_W-1:0]                start_dispatch_event_tag_r;
    logic [EVENT_TIME_W-1:0]         start_dispatch_event_time_r;
    logic signed [WEIGHT_W-1:0]      start_dispatch_weight_r;
    logic                            start_dispatch_has_fanout_r;

    // Per-neuron state.  3 bits x N total.
    logic [NEURONS_PER_TILE-1:0]     logical_event_inflight_r;              // invariant 1
    logic [NEURONS_PER_TILE-1:0]     in_ready_fifo_r;         // invariant 2
    logic [NEURONS_PER_TILE-1:0]     pending_mask_prev_r;  // kept for wiring; always 0 or one-hot

    // Ready FIFO proper.
    logic [NEURON_IDX_W-1:0]         ready_fifo_r [0:READY_FIFO_DEPTH-1];
    logic [READY_FIFO_PTR_W-1:0]     ready_rd_ptr_r;
    logic [READY_FIFO_PTR_W-1:0]     ready_wr_ptr_r;
    logic [READY_FIFO_COUNT_W-1:0]   ready_count_r;

    // Worker round-robin.
    logic [WORKER_IDX_W-1:0]         worker_rr_ptr_r;
    logic [WORKER_IDX_W-1:0]         worker_commit_rr_ptr_r;
    logic [WORKER_CORES_PER_TILE-1:0] worker_has_fanout_r;

    // Registered soft-reset one-hot → idx pipeline.  Combinational encoder
    // computes _c; the _r versions drive the module outputs with 1 cycle
    // of latency to break the 128-wide priority encoder off the critical
    // path that feeds into state_bank / context_bank BRAM write ports.
    logic                            soft_reset_valid_c;
    logic [NEURON_IDX_W-1:0]         soft_reset_idx_c;
    logic                            soft_reset_valid_r;
    logic [NEURON_IDX_W-1:0]         soft_reset_idx_r;

    // Observability counter: how often the stuck-escape path fired an
    // over-credit dispatch.  Under healthy operation this stays at 0.
    // Exposed via verilator's public_flat_rd so cocotb probes can read
    // it at end-of-test and characterise the throttle's coverage.
    logic [15:0] fanout_stuck_escape_total_r
        /* verilator public_flat_rd */;

    // ---------------------------------------------------------------------
    // Lint: full_mask / ucode fields read only for future lane-gating.
    // ---------------------------------------------------------------------
    /* verilator lint_off UNUSED */
    wire _unused_ok = &{1'b0,
                        state_dispatch_ucode_ptr, state_dispatch_ucode_len,
                        logical_spike_full, 1'b0};
    /* verilator lint_on UNUSED */

    // ---------------------------------------------------------------------
    // Dispatch step (O(1), no scan).
    // ---------------------------------------------------------------------
    logic                            ready_head_valid_c;
    logic [NEURON_IDX_W-1:0]         ready_head_idx_c;
    logic                            ready_head_stale_c;
    logic                            ready_pop_c;
    logic                            start_dispatch_issue_c;
    logic                            worker_start_fire_c;

    logic [WORKER_CORES_PER_TILE-1:0] worker_ready_mask_c;
    logic [WORKER_IDX_W-1:0]         worker_start_idx_c;
    logic                            worker_start_worker_valid_c;

    logic [WORKER_CORES_PER_TILE-1:0] worker_start_valid_c;

    // ---------------------------------------------------------------------
    // Credit-based dispatch throttling (Option A of the MSG_OUTPUT wedge
    // structural fix).  The fanout_queue can fill to DEPTH entries; each
    // entry represents one neuron's fanout-burst in flight.  In-flight
    // workers holding worker_has_fanout_r[w]=1 are reservations that will
    // (if the result spikes) land a push into fanout_queue at commit time.
    //
    // Invariant maintained by the gate below:
    //   fanout_queue_count_live + popcount(worker_has_fanout_r) <= DEPTH
    //
    // A new has_fanout dispatch can only transition pending -> valid when
    // the sum stays within DEPTH.  Non-has_fanout dispatches are never
    // throttled since they can never push to fanout_queue.  On commit we
    // now clear worker_has_fanout_r[w] so the popcount is accurate: the
    // reservation is released when the worker either (a) pushes the job
    // into fanout_queue (where fanout_queue_count_live increments in the
    // same cycle, preserving the sum), or (b) commits a non-spike result
    // (where the reservation just evaporates without a push).
    // ---------------------------------------------------------------------
    localparam int unsigned FANOUT_CREDIT_W =
        (FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH + 1);

    logic [FANOUT_CREDIT_W-1:0]      worker_fanout_reservations_c;
    logic [FANOUT_CREDIT_W-1:0]      fanout_occupancy_c;
    logic                            fanout_credit_allow_c;
    logic                            pending_has_fanout_c;
    logic                            pending_advance_c;
    // Escape valve: when the throttle has fully blocked dispatch AND no
    // workers are in flight AND fanout_queue is at its cap, the only way
    // for fanout_queue to ever drain is for some event_queue target to
    // get drained by a dispatch — which the throttle is currently
    // preventing.  Detect the stuck state and allow one over-credit
    // dispatch to break the cycle.  The matching drop-on-overflow at
    // fanout_queue_push_c (in tile_fanout_executor) ensures the
    // speculative dispatch's push, if it fires, is silently dropped
    // rather than overflowing the queue.  Dropped pushes are counted
    // for observability (a rare lossy path, only reached under
    // pathological cascades with all neurons on a single tile).
    logic                            fanout_stuck_c;
    integer                          dispatch_scan_idx;
    logic                            dispatch_found_worker;

    always_comb begin

        // Head view.
        ready_head_valid_c = (ready_count_r != READY_FIFO_COUNT_W'(0));
        ready_head_idx_c   = ready_fifo_r[ready_rd_ptr_r];

        // Mask worker_ready with "pipeline busy on this worker".
        for (dispatch_scan_idx = 0; dispatch_scan_idx < WORKER_CORES_PER_TILE; dispatch_scan_idx = dispatch_scan_idx + 1) begin
            worker_ready_mask_c[dispatch_scan_idx] = worker_start_ready[dispatch_scan_idx];
        end
        if (start_dispatch_valid_r || start_dispatch_pending_r) begin
            worker_ready_mask_c[start_dispatch_worker_idx_r] = 1'b0;
        end

        worker_start_idx_c = find_worker_rr(worker_ready_mask_c,
                                            worker_rr_ptr_r,
                                            dispatch_found_worker);
        worker_start_worker_valid_c = dispatch_found_worker;

        // Head is "stale" if it shouldn't actually be issued (e.g. pending
        // cleared by soft-reset, or inflight race).  Drop-without-issue.
        ready_head_stale_c =
            ready_head_valid_c &&
            !start_dispatch_valid_r &&
            !start_dispatch_pending_r &&
                        (!(logical_spike_pending_valid &&
                             (logical_spike_pending_idx == ready_head_idx_c)) ||
              logical_event_inflight_r[ready_head_idx_c]);

        start_dispatch_issue_c =
            !start_dispatch_valid_r && !start_dispatch_pending_r &&
             ready_head_valid_c && !ready_head_stale_c &&
             logical_spike_pending_valid &&
             (logical_spike_pending_idx == ready_head_idx_c) &&
             !logical_event_inflight_r[ready_head_idx_c] &&
             worker_start_worker_valid_c;

        // FIFO pops on either a stale drop or a successful issue.
        ready_pop_c =
            !start_dispatch_valid_r && !start_dispatch_pending_r &&
             ready_head_valid_c &&
             (ready_head_stale_c || start_dispatch_issue_c);

        // worker_start_fire = the stage-2 latched dispatch sees its
        // nominated worker become ready.
        worker_start_fire_c =
            start_dispatch_valid_r &&
            worker_start_ready[start_dispatch_worker_idx_r];

        for (dispatch_scan_idx = 0; dispatch_scan_idx < WORKER_CORES_PER_TILE; dispatch_scan_idx = dispatch_scan_idx + 1) begin
            worker_start_valid_c[dispatch_scan_idx] =
                start_dispatch_valid_r &&
                (start_dispatch_worker_idx_r == dispatch_scan_idx[WORKER_IDX_W-1:0]);
        end
    end

    // ---------------------------------------------------------------------
    // Enqueue source selection.  One push per cycle maximum.  Commit-path
    // re-enqueue takes priority over pending-edge enqueue (keeps an
    // actively-worked neuron moving even if the pending set is fanned out).
    // ---------------------------------------------------------------------
    logic                            commit_requeue_c;
    logic [NEURON_IDX_W-1:0]         commit_requeue_idx_c;

    logic [NEURONS_PER_TILE-1:0]     pending_edge_mask_c;  // kept for formal compat; always one-hot or 0
    logic [NEURONS_PER_TILE-1:0]     enqueue_candidate_mask_c; // not used for priority encode; see below
    logic                            pending_edge_c;        // scalar edge detect
    logic                            edge_push_valid_c;
    logic [NEURON_IDX_W-1:0]         edge_push_idx_c;

    // prev-pending snapshot (replaces pending_mask_prev_r[N])
    logic                            prev_pending_valid_r;
    logic [NEURON_IDX_W-1:0]         prev_pending_idx_r;

    logic                            ready_push_c;
    logic [NEURON_IDX_W-1:0]         ready_push_idx_c;
    logic [NEURONS_PER_TILE-1:0]     pending_push_mask_c;
    logic [NEURONS_PER_TILE-1:0]     pop_reservation_mask_c;
    logic [READY_FIFO_COUNT_W-1:0]   effective_ready_count_c;

    // Stage-A / Stage-B pipeline on the ready-FIFO push.
    //
    // Previously `ready_push_c` / `ready_push_idx_c` were consumed in the
    // same always_ff block that mutated `in_ready_fifo_r` and `ready_fifo_r`.
    // `ready_push_idx_c` is the output of a NEURONS_PER_TILE-wide lowest-set
    // priority encoder over `enqueue_candidate_mask_c`, which itself is a
    // combinational function of `logical_event_pending_mask`,
    // `logical_event_inflight_r`, `in_ready_fifo_r`, and a large
    // graph_state_clear-gated control cone.  That combinational chain
    // (~25 cascaded LUT6s driving an indexed write into in_ready_fifo_r)
    // was the post-fix-A worst synth path.
    //
    // Stage A (this cycle): compute `ready_push_c` / `ready_push_idx_c`
    // as before and register only the narrow token into `_r`.  Stage B
    // (next cycle): locally decode that token when mutating the FIFO /
    // bitmap -- the index decoder is driven from a flop Q instead of the
    // encoder output, without carrying a full N-bit push mask as state.
    //
    // Invariant implications:
    //   * The candidate mask on cycle N+1 must also exclude
    //     `pending_push_mask_c`, because the push decided on cycle N is not
    //     yet reflected in `in_ready_fifo_r` at the start of cycle N+1
    //     (it lands end-of-N+1).  Without this exclusion the same neuron
    //     could be picked twice.
    //   * The commit-requeue gate must exclude `pending_push_mask_c` for
    //     the same reason.
    //   * The FIFO-full guard must account for the pending push:
    //     `(ready_count_r + ready_push_valid_r) < READY_FIFO_DEPTH`.
    //   * First-dispatch latency increases by 1 cycle (push decided
    //     N -> ready_head visible N+2).  `ready_count_r` still reflects
    //     committed entries so the head-read handshake is unchanged.
    logic                            ready_push_valid_r;
    logic [NEURON_IDX_W-1:0]         ready_push_idx_r;

    // One-cycle reservation for the FIFO head that was just popped into
    // the start-dispatch pipeline.  Enqueue selection uses this stable
    // token rather than feeding current-cycle ready_pop_c back into the
    // next-push priority encoder.
    logic                            pop_reservation_valid_r;
    logic [NEURON_IDX_W-1:0]         pop_reservation_idx_r;

    // ---------------------------------------------------------------------
    // Commit step.
    // ---------------------------------------------------------------------
    logic [NEURON_IDX_W-1:0]         worker_commit_logical_idx_c;
    logic [WORKER_IDX_W-1:0]         worker_commit_idx_c;
    logic                            worker_commit_valid_c;
    logic                            worker_commit_ready_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_result_ready_c;

    logic                            context_commit_valid_c;
    logic [NEURON_IDX_W-1:0]         context_commit_idx_c;
    neuron_exec_ctx_t                context_commit_ctx_c;
    integer                          commit_scan_idx;
    logic                            commit_found_commit;

    always_comb begin

        worker_commit_idx_c = find_worker_rr(worker_result_valid,
                                             worker_commit_rr_ptr_r,
                                             commit_found_commit);
        worker_commit_valid_c = commit_found_commit;
        worker_commit_logical_idx_c = '0;
        if (worker_commit_valid_c) begin
            worker_commit_logical_idx_c =
                worker_result_logical_idx_u[worker_commit_idx_c];
        end

        // Commit gate.  The fanout_queue_can_accept term that used to
        // stall spike+fanout commits has been removed: the credit-based
        // dispatch throttle above already prevents fanout_queue overflow
        // in the common case, and the stuck-escape path explicitly
        // allows over-credit dispatches with drop-on-overflow at
        // fanout_executor's push gate.  Keeping the stall here would
        // re-introduce the exact deadlock the throttle is meant to
        // eliminate (commits waiting on queue drain, which waits on
        // target event_queue drain, which waits on commits...).
        worker_commit_ready_c = 1'b0;
        if (ena && worker_commit_valid_c) begin
            worker_commit_ready_c = !host_reserve_current;
        end

        context_commit_valid_c = 1'b0;
        context_commit_idx_c = '0;
        context_commit_ctx_c = '0;
        for (commit_scan_idx = 0; commit_scan_idx < WORKER_CORES_PER_TILE; commit_scan_idx = commit_scan_idx + 1) begin
            worker_result_ready_c[commit_scan_idx] = 1'b0;
        end

        if (worker_commit_valid_c && worker_commit_ready_c) begin
            worker_result_ready_c[worker_commit_idx_c] = 1'b1;
            context_commit_valid_c = 1'b1;
            context_commit_idx_c = worker_commit_logical_idx_c;
            context_commit_ctx_c = worker_result_ctx_u[worker_commit_idx_c];
        end
    end

    // ---------------------------------------------------------------------
    // Enqueue source selection, continued.  Dedup observes the registered
    // dispatch pipeline and registered push/pop tokens, so the current
    // FIFO pop does not feed the current enqueue choice.
    // ---------------------------------------------------------------------
    logic                            enqueue_found_edge;
    logic [NEURONS_PER_TILE-1:0]     enqueue_in_pipeline_mask;
    logic [NEURONS_PER_TILE-1:0]     stb_pop_clear_mask;
    logic [NEURONS_PER_TILE-1:0]     stb_keep_mask;

    always_comb begin

        // Decode narrow registered tokens locally.  These masks are not
        // persistent state, which keeps reset and placement fanout small.
        pending_push_mask_c = '0;
        if (ready_push_valid_r) begin
            pending_push_mask_c[ready_push_idx_r] = 1'b1;
        end

        pop_reservation_mask_c = '0;
        if (pop_reservation_valid_r) begin
            pop_reservation_mask_c[pop_reservation_idx_r] = 1'b1;
        end

        // Mark the neuron currently riding the start-dispatch two-cycle
        // pipeline (between FIFO-pop and worker_start_fire).  It is not
        // yet in logical_event_inflight_r but must not be re-enqueued.
        enqueue_in_pipeline_mask = '0;
        if (start_dispatch_pending_r || start_dispatch_valid_r) begin
            enqueue_in_pipeline_mask[start_dispatch_logical_idx_r] = 1'b1;
        end

        // Commit re-enqueue: on a successful commit, if the neuron still
        // has queued events (pending now valid for that neuron) and isn't
        // already in the FIFO, push it.
        commit_requeue_c     = 1'b0;
        commit_requeue_idx_c = '0;
        if (worker_commit_valid_c && worker_commit_ready_c) begin
            if (logical_spike_pending_valid &&
                (logical_spike_pending_idx == worker_commit_logical_idx_c) &&
                !in_ready_fifo_r[worker_commit_logical_idx_c] &&
                !pending_push_mask_c[worker_commit_logical_idx_c] &&
                !pop_reservation_mask_c[worker_commit_logical_idx_c] &&
                !enqueue_in_pipeline_mask[worker_commit_logical_idx_c]) begin
                commit_requeue_c     = 1'b1;
                commit_requeue_idx_c = worker_commit_logical_idx_c;
            end
        end

        // Scalar edge detect: the head neuron changed (or FIFO went empty→full).
        // prev_pending_{valid,idx}_r are updated in the sequential block.
        pending_edge_c =
            logical_spike_pending_valid &&
            (!prev_pending_valid_r || (logical_spike_pending_idx != prev_pending_idx_r));

        // Scalar candidate check: new head eligible iff not inflight/in-fifo/etc.
        edge_push_valid_c =
            pending_edge_c &&
            !logical_event_inflight_r[logical_spike_pending_idx] &&
            !in_ready_fifo_r[logical_spike_pending_idx] &&
            !pending_push_mask_c[logical_spike_pending_idx] &&
            !pop_reservation_mask_c[logical_spike_pending_idx] &&
            !enqueue_in_pipeline_mask[logical_spike_pending_idx];
        edge_push_idx_c = logical_spike_pending_idx;

        // Keep formal-visible masks consistent (one-hot or zero).
        pending_edge_mask_c      = '0;
        enqueue_candidate_mask_c = '0;
        if (pending_edge_c) begin
            pending_edge_mask_c[logical_spike_pending_idx] = 1'b1;
        end
        if (edge_push_valid_c) begin
            enqueue_candidate_mask_c[logical_spike_pending_idx] = 1'b1;
        end

        // Priority: commit re-enqueue > edge enqueue.  Push only if the
        // FIFO has room — which, given the dedup invariant, is guaranteed
        // whenever at least one eligible source exists (since |fifo| <=
        // count of distinct neurons not inflight, and we don't push
        // duplicates).  The explicit check is belt-and-braces.
        //
        // Effective count = committed entries + pending Stage-A push that
        // hasn't landed yet.  Without the `+ ready_push_valid_r` term a
        // back-to-back decide-push on a nearly-full FIFO could overflow.
        effective_ready_count_c =
            ready_count_r +
            READY_FIFO_COUNT_W'(ready_push_valid_r);
        ready_push_c     = 1'b0;
        ready_push_idx_c = '0;
        if (commit_requeue_c &&
            (effective_ready_count_c <
             READY_FIFO_DEPTH[READY_FIFO_COUNT_W-1:0])) begin
            ready_push_c     = 1'b1;
            ready_push_idx_c = commit_requeue_idx_c;
        end else if (edge_push_valid_c &&
                     (effective_ready_count_c <
                      READY_FIFO_DEPTH[READY_FIFO_COUNT_W-1:0])) begin
            ready_push_c     = 1'b1;
            ready_push_idx_c = edge_push_idx_c;
        end

    end

    // ---------------------------------------------------------------------
    // Soft-reset pulse → one-hot decoder.  Picks the lowest set bit.
    // Combinational `_c` here feeds a 1-cycle register (`_r`) that drives
    // the module output.  soft_reset_pulse is one-hot in practice (state
    // bank sets one bit per CSR write), so the priority tree is really
    // just a position encoder.  Registering the output prevents the 128→7
    // combinational reduction from landing on the critical path between
    // soft_reset_pulse_reg (state_bank) and reset_init_rf_flat (BRAM
    // write in state_bank / context_bank).
    // ---------------------------------------------------------------------
    // Soft-reset: state_bank now provides scalar (valid, idx) directly.
    // No priority encoder needed; just wire through for registration.
    always_comb begin
        soft_reset_valid_c = soft_reset_valid_in;
        soft_reset_idx_c   = soft_reset_idx_in;
    end

    // ---------------------------------------------------------------------
    // Credit-based dispatch throttle — combinational.
    // ---------------------------------------------------------------------
    always_comb begin
        // Sum in-flight has_fanout reservations across all workers.  Small
        // popcount (WORKER_CORES_PER_TILE is typically 2..4).
        worker_fanout_reservations_c = '0;
        for (w = 0; w < WORKER_CORES_PER_TILE; w++) begin
            if (worker_has_fanout_r[w]) begin
                worker_fanout_reservations_c =
                    worker_fanout_reservations_c + FANOUT_CREDIT_W'(1);
            end
        end

        fanout_occupancy_c =
            fanout_queue_count_live + worker_fanout_reservations_c;

        // Would THIS dispatch need a fanout_queue slot?  Determined from
        // the same signals used to set start_dispatch_has_fanout_r below.
        pending_has_fanout_c =
            (state_dispatch_fanout_len != FANOUT_ADDR_W'(0)) &&
            noc_egress_en;

        // Allow the transition unless we would exceed DEPTH after adding
        // this reservation.  No-fanout dispatches are always allowed.
        fanout_credit_allow_c =
            !pending_has_fanout_c ||
            (fanout_occupancy_c <
             FANOUT_CREDIT_W'(FANOUT_QUEUE_DEPTH));

        // Stuck detection: fanout_queue is at cap AND no workers are
        // in flight AND we would otherwise block this dispatch.  In
        // that state the fanout_queue cannot drain without a new
        // dispatch (which would drain a target event_queue), so we
        // must allow one over-credit dispatch and rely on
        // drop-on-overflow at the push site to handle the overshoot.
        fanout_stuck_c =
            pending_has_fanout_c &&
            !fanout_credit_allow_c &&
            (worker_fanout_reservations_c == FANOUT_CREDIT_W'(0)) &&
            (fanout_queue_count_live ==
             FANOUT_CREDIT_W'(FANOUT_QUEUE_DEPTH));

        pending_advance_c =
            start_dispatch_pending_r &&
            !start_dispatch_valid_r &&
            (fanout_credit_allow_c || fanout_stuck_c);
    end

    // ---------------------------------------------------------------------
    // Registered updates.
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n || tile_graph_state_clear) begin
            start_dispatch_valid_r       <= 1'b0;
            start_dispatch_pending_r     <= 1'b0;
            start_dispatch_worker_idx_r  <= '0;
            start_dispatch_logical_idx_r <= '0;
            start_dispatch_event_tag_r   <= '0;
            start_dispatch_event_time_r  <= '0;
            start_dispatch_weight_r      <= '0;
            start_dispatch_has_fanout_r  <= 1'b0;
            logical_event_inflight_r                   <= '0;
            in_ready_fifo_r              <= '0;
            pending_mask_prev_r          <= '0;
            prev_pending_valid_r         <= 1'b0;
            prev_pending_idx_r           <= '0;
            ready_rd_ptr_r               <= '0;
            ready_wr_ptr_r               <= '0;
            ready_count_r                <= '0;
            ready_push_valid_r           <= 1'b0;
            ready_push_idx_r             <= '0;
            pop_reservation_valid_r      <= 1'b0;
            pop_reservation_idx_r        <= '0;
            worker_rr_ptr_r              <= '0;
            worker_commit_rr_ptr_r       <= '0;
            worker_has_fanout_r          <= '0;
            soft_reset_valid_r           <= 1'b0;
            soft_reset_idx_r             <= '0;
            fanout_stuck_escape_total_r  <= 16'd0;
            for (k_rst = 0; k_rst < READY_FIFO_DEPTH; k_rst++) begin
                ready_fifo_r[k_rst] <= '0;
            end
        end else if (ena) begin
            // Scalar pending snapshot (replaces N-wide pending_mask_prev_r).
            prev_pending_valid_r <= logical_spike_pending_valid;
            prev_pending_idx_r   <= logical_spike_pending_idx;
            // Legacy compat: keep pending_mask_prev_r as one-hot of head.
            pending_mask_prev_r  <= '0;
            if (logical_spike_pending_valid)
                pending_mask_prev_r[logical_spike_pending_idx] <= 1'b1;

            // Soft-reset output pipeline: 1-cycle register from encoder.
            soft_reset_valid_r  <= soft_reset_valid_c;
            soft_reset_idx_r    <= soft_reset_idx_c;

            // Inflight bitmap: scalar soft-reset clear.
            if (soft_reset_valid_r) begin
                logical_event_inflight_r[soft_reset_idx_r] <= 1'b0;
            end
            ready_push_valid_r <= ready_push_c;
            ready_push_idx_r   <= ready_push_idx_c;
            pop_reservation_valid_r <= ready_pop_c && start_dispatch_issue_c;
            pop_reservation_idx_r   <= ready_head_idx_c;

            // --- Stage B: apply the previously-registered push ---
            // Stage-B in_ready_fifo_r: scalar soft-reset clear replaces
            // N-wide AND with ~soft_reset_pulse.
            begin : stage_b_ready_fifo_update
                stb_pop_clear_mask = '0;
                if (ready_pop_c) begin
                    stb_pop_clear_mask[ready_head_idx_c] = 1'b1;
                end
                stb_keep_mask = in_ready_fifo_r & ~stb_pop_clear_mask;
                if (soft_reset_valid_r) begin
                    stb_keep_mask[soft_reset_idx_r] = 1'b0;
                end
                in_ready_fifo_r <= stb_keep_mask | pending_push_mask_c;
            end
            if (ready_pop_c) begin
                ready_rd_ptr_r <= next_ready_ptr(ready_rd_ptr_r);
            end
            if (ready_push_valid_r) begin
                ready_fifo_r[ready_wr_ptr_r] <= ready_push_idx_r;
                ready_wr_ptr_r <= next_ready_ptr(ready_wr_ptr_r);
            end
            // Count update (push/pop can happen same cycle; net delta).
            // Push side uses the Stage-B flop so count increments align
            // with the actual ready_fifo_r write.  Pop side stays same-
            // cycle as ready_pop_c.
            if (ready_push_valid_r && !ready_pop_c) begin
                ready_count_r <= ready_count_r + READY_FIFO_COUNT_W'(1);
            end else if (!ready_push_valid_r && ready_pop_c) begin
                ready_count_r <= ready_count_r - READY_FIFO_COUNT_W'(1);
            end

            // --- Start-dispatch pipeline ---
            if (start_dispatch_issue_c) begin
                start_dispatch_pending_r     <= 1'b1;
                start_dispatch_worker_idx_r  <= worker_start_idx_c;
                start_dispatch_logical_idx_r <= ready_head_idx_c;
            end
            // Gated on fanout credit: if this dispatch would push a
            // has_fanout reservation that exceeds the fanout_queue slack,
            // stay in pending_r for another cycle and re-check next
            // cycle.  The fanout_len read is stable across cycles since
            // state_dispatch_read_idx keeps pointing at logical_idx_r
            // while pending_r=1, so no additional re-issue is needed.
            if (pending_advance_c) begin
                start_dispatch_event_tag_r  <= logical_spike_head_read_tag;
                start_dispatch_event_time_r <= logical_spike_head_read_time;
                start_dispatch_weight_r     <= logical_spike_head_read_weight;
                start_dispatch_valid_r      <= 1'b1;
                start_dispatch_pending_r    <= 1'b0;
                start_dispatch_has_fanout_r <= pending_has_fanout_c;
                // Count stuck-escape fires: the transition only takes
                // the over-credit path when the throttle would have
                // denied it but the stuck-escape override kicked in.
                if (!fanout_credit_allow_c && fanout_stuck_c) begin
                    fanout_stuck_escape_total_r <=
                        fanout_stuck_escape_total_r + 16'd1;
                end
            end

            // Inflight bitmap: scalar soft-reset clear (invariant carry-over).
            if (soft_reset_valid_r) begin
                logical_event_inflight_r[soft_reset_idx_r] <= 1'b0;
            end
            // Set on worker_start_fire (dispatch latched into a worker).
            if (worker_start_fire_c) begin
                worker_has_fanout_r[start_dispatch_worker_idx_r] <=
                    start_dispatch_has_fanout_r;
                logical_event_inflight_r[start_dispatch_logical_idx_r] <= 1'b1;
                worker_rr_ptr_r <= start_dispatch_worker_idx_r +
                                    WORKER_IDX_W'(1);
                start_dispatch_valid_r   <= 1'b0;
                start_dispatch_pending_r <= 1'b0;
            end
            // Clear inflight on worker_commit_valid (not ready-gated) —
            // invariant 4.  See docs/debugging_msg_output_wedge.md.
            if (worker_commit_valid_c) begin
                logical_event_inflight_r[worker_commit_logical_idx_c] <= 1'b0;
            end

            // Release the fanout-queue credit reservation only on a true
            // commit (valid && ready).  Clearing on valid alone would
            // race the push path: if worker_commit_ready_c ever stalls
            // a cycle, has_fanout_r would drop before the push fires,
            // and fanout_queue_push_c (which ANDs worker_has_fanout)
            // would silently drop the spike.  Pairing the clear with
            // the push firing keeps both edges atomic.
            if (worker_commit_valid_c && worker_commit_ready_c) begin
                worker_has_fanout_r[worker_commit_idx_c] <= 1'b0;
            end

            // Commit RR pointer advances only on a true commit.
            if (worker_commit_valid_c && worker_commit_ready_c) begin
                worker_commit_rr_ptr_r <= worker_commit_idx_c +
                                           WORKER_IDX_W'(1);
            end
        end
    end

    // ---------------------------------------------------------------------
    // Output wiring
    // ---------------------------------------------------------------------
    assign soft_reset_valid              = soft_reset_valid_r;
    assign soft_reset_idx                = soft_reset_idx_r;

    assign worker_start_valid            = worker_start_valid_c;
    assign worker_start_event_tag        = start_dispatch_event_tag_r;
    assign worker_start_event_time       = start_dispatch_event_time_r;
    assign worker_start_event_weight     = start_dispatch_weight_r;
    assign start_dispatch_logical_idx    = start_dispatch_logical_idx_r;
    assign start_dispatch_valid_out      = start_dispatch_valid_r;
    assign start_dispatch_worker_idx_out = start_dispatch_worker_idx_r;
    assign worker_start_fire             = worker_start_fire_c;

    assign dequeue_valid                 = worker_start_fire_c;
    assign dequeue_idx                   = start_dispatch_logical_idx_r;
    assign head_read_idx                 = ready_head_idx_c;

    assign state_dispatch_read_idx =
        (start_dispatch_valid_r || start_dispatch_pending_r)
            ? start_dispatch_logical_idx_r
            : ready_head_idx_c;

    assign worker_result_ready           = worker_result_ready_c;

    assign worker_commit_valid           = worker_commit_valid_c;
    assign worker_commit_ready           = worker_commit_ready_c;
    assign worker_commit_logical_idx     = worker_commit_logical_idx_c;
    assign worker_commit_idx             = worker_commit_idx_c;
    assign worker_has_fanout_snapshot    = worker_has_fanout_r;

    // Pack context_commit_t struct and drive rv_if.
    context_commit_t context_commit_payload_c;
    assign context_commit_payload_c.idx = context_commit_idx_c;
    assign context_commit_payload_c.ctx = context_commit_ctx_c;
    assign context_commit.valid      = context_commit_valid_c;
    assign context_commit.rv_payload = context_commit_payload_c;
    // context_commit.ready driven by tile_top (always 1, context_bank has no back-pressure)

`ifdef FORMAL
    assign formal_worker_rr_ptr_r             = worker_rr_ptr_r;
    assign formal_logical_event_inflight_r    = logical_event_inflight_r;
    assign formal_in_ready_fifo_r             = in_ready_fifo_r;
    assign formal_ready_push                  = ready_push_c;
    assign formal_ready_push_idx              = ready_push_idx_c;
    assign formal_dispatch_fire               = worker_start_fire_c;
    assign formal_dispatch_idx                = start_dispatch_logical_idx_r;
    assign formal_prev_pending_valid_r        = prev_pending_valid_r;
    assign formal_prev_pending_idx_r          = prev_pending_idx_r;
    assign formal_start_dispatch_pending_r    = start_dispatch_pending_r;
    assign formal_start_dispatch_valid_r      = start_dispatch_valid_r;
    assign formal_start_dispatch_logical_idx_r= start_dispatch_logical_idx_r;
    assign formal_ready_fifo_full             =
        (ready_count_r == READY_FIFO_COUNT_W'(READY_FIFO_DEPTH));
    assign formal_ready_push_mask_r           = pending_push_mask_c;
    assign formal_pop_reservation_valid_r     = pop_reservation_valid_r;
    assign formal_pop_reservation_idx_r       = pop_reservation_idx_r;
`endif

endmodule

`default_nettype wire
