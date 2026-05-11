`default_nettype none

// tile_fanout_executor
// ---------------------------------------------------------------------------
// Fanout walker for emitted spikes.  A worker commit that produced a spike is
// queued here, then the executor walks that neuron's outbound fanout entries
// one at a time.  Each resolved entry either injects a weighted local spike
// back into tile_top's per-worker spike queues or emits a packet to the NoC
// child lane.
//
//   worker commit --> fanout_queue --> route read --> route response
//                                               |          |
//                                               |          +--> remote NoC packet
//                                               +----------+--> local spike enqueue
//
// Local-route retry path:
//   If a local destination is full, the current fanout route is snapshotted
//   into fanout_retry_* and replayed later.  This preserves spike causality
//   without holding the shared fanout queue head forever.
//
// Area note for large 1x1024 tiles:
//   Local-loopback pending state is scalar {valid, addr, broadcast} rather
//   than an N-bit destination mask, avoiding a 1024-bit pending register and
//   N-wide accept/conflict gates.
(* keep_hierarchy = "no" *)
`ifndef YOSYS
import tile_pkg::CMD_MESSAGE;
import tile_pkg::CORE_ID_W;
import tile_pkg::EVENT_TIME_W;
import tile_pkg::MESSAGE_CORE_BCAST;
import tile_pkg::MSG_SID_W;
import tile_pkg::MSG_SPIKE;
import tile_pkg::NEURON_IDX_W;
import tile_pkg::PKT_PLANE_DATA;
import tile_pkg::TAG_W;
import tile_pkg::TILE_COORD_W;
import tile_pkg::WEIGHT_W;
import tile_pkg::fanout_read_rsp_t;
import tile_pkg::message_packet_t;
import tile_pkg::neuron_emit_t;
import tile_pkg::packet_meta_with_plane;
import tile_pkg::tile_event_t;
`endif
module tile_fanout_executor
  #(
    parameter int unsigned LOCAL_Z = 0,
    parameter int unsigned NEURONS_PER_TILE = 2,
    parameter int unsigned WORKER_CORES_PER_TILE = 1,
    parameter int unsigned FANOUT_POOL_DEPTH = 256,
    parameter int unsigned FANOUT_QUEUE_DEPTH = 4,
    parameter int unsigned MESSAGE_W = 83,
    // Routing coordinate widths — must match tile_fanout_pool and tile_top.
    // Defaults of 8 preserve backward-compatible behavior.
    parameter int unsigned FANOUT_DST_X_W   = 8,
    parameter int unsigned FANOUT_DST_Y_W   = 8,
    parameter int unsigned FANOUT_CORE_ID_W = 8
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       ena,
    input  wire [TILE_COORD_W-1:0]         tile_x_coord,
    input  wire [TILE_COORD_W-1:0]         tile_y_coord,
    input  wire                       tile_graph_state_clear,

    // --- Worker commit handshake (from dispatch/scheduler-inline) ---
    input  wire                       worker_commit_valid,
    input  wire                       worker_commit_ready,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                      worker_commit_logical_idx,
    input  wire [((WORKER_CORES_PER_TILE <= 1) ? 1 :
                  $clog2(WORKER_CORES_PER_TILE))-1:0]
                                      worker_commit_idx,
    input  wire                       worker_has_fanout,          // worker_has_fanout_r[commit_idx]
    input  neuron_emit_t              worker_result_emit,

    // --- Gate back to scheduler (worker_commit_ready gating) ---
    output wire                       fanout_queue_can_accept,

    // --- Credit-based dispatch throttling (expose live occupancy) ---
    // The scheduler combines this with `popcount(worker_has_fanout_r)` to
    // gate `pending_r -> valid_r` transitions so a new has_fanout dispatch
    // only fires when the sum is strictly less than FANOUT_QUEUE_DEPTH.
    // This moves the commit-side back-pressure upstream of the compute,
    // breaking the fanout_queue <-> spike_queue_bank deadlock cycle.
    output wire [((FANOUT_QUEUE_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_QUEUE_DEPTH + 1))-1:0]
                                      fanout_queue_count_live,

    // --- Debug / telemetry: fanout_fire edge for the top-level counter ---
    output wire                       fanout_fire_dbg,

    // --- State bank fanout-read port ---
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                      fanout_read_idx,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                      state_fanout_ptr,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                      state_fanout_len,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                      state_commit_fanout_ptr,

    // --- Fanout pool port 1 (route walk) drive ---
    output wire                       fanout_bank_p1_read_en,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                      fanout_bank_p1_neuron_idx,
    output wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                      fanout_bank_p1_index,
    // Fanout pool read response — structured rv_if (fanout_read_rsp_t payload).
    // rv_if.valid = read_rsp_valid; rv_if.ready tied 1 in tile_top.
    rv_if.rx                          fanout_rsp,

    // --- NoC egress (child lane) ---
    rv_if.tx                          noc_child,

    // --- Local loopback to spike queue bank (mux stays in top) ---
    // fanout_ev.valid  = a local-route entry is ready to enqueue.
    // fanout_ev.ready  = driven by tile_top as !logical_spike_full.
    //                    The executor also reads logical_spike_full directly
    //                    for its internal lane-accept / dispatch-bypass gating,
    //                    so ready is informational here but keeps the rv_if
    //                    contract intact for the receiver side.
    // fanout_ev.rv_payload = tile_event_t { neuron_idx, weight, tag, time }.
    rv_if.tx                          fanout_ev,

    // --- Ingress conflict mux ---
    input  wire                       ingress_spike_enqueue_fire,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                      ingress_spike_target_idx,

    // --- Dispatch-view inputs (for fanout_lane_accept) ---
    input  wire                       start_dispatch_valid,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                      start_dispatch_logical_idx,
    input  wire                       worker_start_ready_for_dispatch,

    // --- Spike queue full (scalar) ---
    input  wire                       logical_spike_full

`ifdef FORMAL
    // --- FORMAL-only observer ports ---
    // These expose internal retry-path and pending-mask registers so a
    // formal harness can check invariants without relying on cross-hier
    // yosys access (which may synthesize a duplicate register under
    // keep_hierarchy + flatten).  Guarded by `FORMAL` so silicon builds
    // never see these.
    , output wire                      formal_retry_valid_obs
    , output wire                      formal_stall_defer_obs
    , output wire                      formal_retry_start_obs
    , output wire                      formal_local_pending_valid_obs
    , output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                    $clog2(NEURONS_PER_TILE))-1:0]
                                       formal_local_pending_addr_obs
    , output wire                      formal_local_pending_bcast_obs
    , output wire                      formal_fanout_active_obs
    , output wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                    $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       formal_route_idx_obs
`endif
);

    /* verilator lint_off VARHIDDEN */
    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE); /* verilator lint_on VARHIDDEN */
    localparam int unsigned WORKER_IDX_W =
        (WORKER_CORES_PER_TILE <= 1) ? 1 : $clog2(WORKER_CORES_PER_TILE);
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    localparam int unsigned FANOUT_QUEUE_PTR_W =
        (FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH);
    localparam int unsigned FANOUT_QUEUE_COUNT_W =
        $clog2(FANOUT_QUEUE_DEPTH + 1);
    localparam int unsigned FANOUT_RETRY_BYPASS_W =
        (FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH + 1);

    // DIRECT_WEIGHT_ADDR_MODE: fanout table is indexed directly by logical_idx.

    logic noc_child_valid;
    logic [MESSAGE_W-1:0] noc_child_payload;
    wire noc_child_ready = noc_child.ready;
    assign noc_child.valid = noc_child_valid;
    assign noc_child.rv_payload = noc_child_payload;

    // ---------------------------------------------------------------------
    // Shared helpers (local_core_id_for_index, local_target_valid_for_core_id,
    // onehot_for_idx).  MUST be included at module scope so Vivado accepts
    // the module-parameter references inside the function bodies.
    // ---------------------------------------------------------------------
    `include "tile_pkg.svh"

    function automatic logic route_exists_from(
        input logic [FANOUT_ADDR_W-1:0] ref_bidx,
        input logic [FANOUT_ADDR_W-1:0] ref_flen
    );
        begin
            route_exists_from = (ref_flen != FANOUT_ADDR_W'(0));
        end
    endfunction

    function automatic [FANOUT_ADDR_W-1:0] first_route_index_from(
        input logic [FANOUT_ADDR_W-1:0] frif_bidx
    );
        begin
            first_route_index_from = frif_bidx;
        end
    endfunction

    function automatic logic route_exists_after(
        input logic [FANOUT_ADDR_W-1:0] rea_cidx,
        input logic [FANOUT_ADDR_W-1:0] rea_bidx,
        input logic [FANOUT_ADDR_W-1:0] rea_flen
    );
        logic [FANOUT_ADDR_W-1:0] end_idx;
        begin
            end_idx = rea_bidx + rea_flen;
            route_exists_after = (rea_cidx + FANOUT_ADDR_W'(1)) < end_idx;
        end
    endfunction

    function automatic [FANOUT_ADDR_W-1:0] next_route_index(
        input logic [FANOUT_ADDR_W-1:0] nri_cidx
    );
        begin
            next_route_index = nri_cidx + FANOUT_ADDR_W'(1);
        end
    endfunction

    function automatic [FANOUT_QUEUE_PTR_W-1:0] next_fanout_queue_ptr(
        input logic [FANOUT_QUEUE_PTR_W-1:0] ptr
    );
        begin
            if (FANOUT_QUEUE_DEPTH <= 1) begin
                next_fanout_queue_ptr = '0;
            end else if (ptr == FANOUT_QUEUE_PTR_W'(FANOUT_QUEUE_DEPTH - 1)) begin
                next_fanout_queue_ptr = '0;
            end else begin
                next_fanout_queue_ptr = ptr + FANOUT_QUEUE_PTR_W'(1);
            end
        end
    endfunction

    function automatic message_packet_t make_weighted_spike_packet(
        input logic [NEURON_IDX_W-1:0] logical_idx,
        input logic signed [WEIGHT_W-1:0] spike_weight,
        input logic [TAG_W-1:0]        tag,
        input logic                    do_broadcast
        // event_time removed: neutern does not use temporal event ordering
    );
        message_packet_t packet;
        logic [CORE_ID_W-1:0] local_core_id;
        begin
            packet = '0;
            local_core_id = CORE_ID_W'(local_core_id_for_index(logical_idx));
            // Route/header fields are wrapped later from the fanout route
            // lookup right before NoC egress; this packet carries only the
            // spike's local payload fields until then.
            packet.kind     = MSG_SPIKE;
            packet.cmd_kind = CMD_MESSAGE;
            packet.broadcast = do_broadcast;
            packet.dst_x = '0;
            packet.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            packet.core_id  = local_core_id;
            packet.sid      = local_core_id[MSG_SID_W-1:0];
            packet.tag      = tag;
            packet.weight   = spike_weight[WEIGHT_W-1:0];  // 4-bit weight
            packet.meta     = packet_meta_with_plane(8'h00, PKT_PLANE_DATA);
            make_weighted_spike_packet = packet;
        end
    endfunction

    // ---------------------------------------------------------------------
    // Registers owned by this module
    // ---------------------------------------------------------------------
    logic fanout_active_r;
    logic [NEURON_IDX_W-1:0] fanout_logical_idx_r;
    logic [FANOUT_ADDR_W-1:0] fanout_route_idx_r;
    logic [MESSAGE_W-1:0] fanout_packet_bits_r;

    logic fanout_retry_valid_r;
    logic fanout_retry_turn_r;
    logic [FANOUT_RETRY_BYPASS_W-1:0] fanout_retry_bypass_r;
    logic [NEURON_IDX_W-1:0] fanout_retry_logical_idx_r;
    logic [FANOUT_ADDR_W-1:0] fanout_retry_route_idx_r;
    logic [MESSAGE_W-1:0] fanout_retry_packet_bits_r;
    logic fanout_retry_route_data_valid_r;
    logic [FANOUT_DST_X_W-1:0]  fanout_retry_dst_x_r;
    logic [FANOUT_DST_Y_W-1:0]  fanout_retry_dst_y_r;
    logic [FANOUT_CORE_ID_W-1:0] fanout_retry_core_id_r;
    logic signed [WEIGHT_W-1:0] fanout_retry_weight_r;
    logic fanout_retry_route_valid_r;
    logic fanout_retry_local_pending_valid_r;
    logic [NEURON_IDX_W-1:0] fanout_retry_local_pending_addr_r;
    logic fanout_retry_local_pending_bcast_r;

    logic fanout_route_data_valid_r;
    logic [FANOUT_DST_X_W-1:0]  fanout_route_dst_x_r;
    logic [FANOUT_DST_Y_W-1:0]  fanout_route_dst_y_r;
    logic [FANOUT_CORE_ID_W-1:0] fanout_route_core_id_r;
    logic signed [WEIGHT_W-1:0] fanout_route_weight_r;
    logic fanout_route_valid_r;

    logic route_fanout_read_pending_r;
    logic fanout_local_pending_valid_r;
    logic [NEURON_IDX_W-1:0] fanout_local_pending_addr_r;
    logic fanout_local_pending_bcast_r;
    logic fanout_local_enqueue_ready_r;

    // Registered fanout-pool read-request stage.  Previously the BRAM
    // ENARDEN and address pins were driven combinationally by a long
    // chain (classification -> queue/pending -> next-route math ->
    // port mux).  That path was the worst synth path; routing it
    // through flops turns the BRAM port pins into simple Q outputs.
    // Costs +1 cycle per route issued (handshake via
    // route_fanout_read_pending_r already tolerates arbitrary BRAM
    // read latency, so no FSM change is required).
    logic                          fanout_bank_p1_read_en_r;
    logic [((NEURONS_PER_TILE <= 1) ? 1 :
            $clog2(NEURONS_PER_TILE))-1:0]
                                   fanout_bank_p1_neuron_idx_r;
    logic [((FANOUT_POOL_DEPTH <= 1) ? 1 :
            $clog2(FANOUT_POOL_DEPTH))-1:0]
                                   fanout_bank_p1_index_r;

    // Fanout read-response interface (port-1).
    // Unpack fanout_rsp rv_if payload into named fields.
    fanout_read_rsp_t fanout_rsp_data_c;
    assign fanout_rsp.ready          = 1'b1;  // pool response always consumed
    assign fanout_rsp_data_c = fanout_read_rsp_t'(fanout_rsp.rv_payload);

    wire fanout_read_rsp_valid_c     = fanout_rsp.valid;
    wire fanout_read_rsp_meta_valid_c = fanout_rsp_data_c.meta_valid;
    wire signed [WEIGHT_W-1:0] fanout_read_rsp_weight_c  = fanout_rsp_data_c.weight;
    wire [FANOUT_DST_X_W-1:0]  fanout_read_rsp_dst_x_c   = FANOUT_DST_X_W'(fanout_rsp_data_c.dst_x);
    wire [FANOUT_DST_Y_W-1:0]  fanout_read_rsp_dst_y_c   = FANOUT_DST_Y_W'(fanout_rsp_data_c.dst_y);
    wire [FANOUT_CORE_ID_W-1:0] fanout_read_rsp_core_id_c = FANOUT_CORE_ID_W'(fanout_rsp_data_c.core_id);

    // Shared queue entries; each entry tracks one in-flight fanout burst
    // for any logical neuron in the tile.
    logic [NEURON_IDX_W-1:0] fanout_queue_logical_idx_r [0:FANOUT_QUEUE_DEPTH-1];
    logic [TAG_W-1:0] fanout_queue_tag_r [0:FANOUT_QUEUE_DEPTH-1];
    logic [EVENT_TIME_W-1:0] fanout_queue_event_time_r [0:FANOUT_QUEUE_DEPTH-1];
    logic [FANOUT_ADDR_W-1:0] fanout_queue_base_r [0:FANOUT_QUEUE_DEPTH-1];
    logic [FANOUT_QUEUE_PTR_W-1:0] fanout_queue_rd_ptr_r;
    logic [FANOUT_QUEUE_PTR_W-1:0] fanout_queue_wr_ptr_r;
    logic [FANOUT_QUEUE_COUNT_W-1:0] fanout_queue_count_r;
    logic [FANOUT_ADDR_W-1:0] fanout_queue_head_base_r;

    // Observability: total count of drop-on-overflow events.  Exposed
    // via verilator's public_flat_rd so cocotb probes can read at
    // end-of-test.  Under the throttle's normal operating regime this
    // stays at 0; nonzero means the stuck-escape path fired AND the
    // newly-dispatched worker committed with a spike during a full-
    // queue window.
    logic [15:0] fanout_queue_drop_total_r
        /* verilator public_flat_rd */;

    // ---------------------------------------------------------------------
    // Combinational views of the queue head
    // ---------------------------------------------------------------------
    logic fanout_queue_valid_c;
    logic fanout_queue_full_c;
    logic [NEURON_IDX_W-1:0] fanout_queue_head_logical_idx_c;
    logic [TAG_W-1:0] fanout_queue_head_tag_c;
    logic [EVENT_TIME_W-1:0] fanout_queue_head_event_time_c;

    assign fanout_queue_valid_c = (fanout_queue_count_r != FANOUT_QUEUE_COUNT_W'(0));
    assign fanout_queue_full_c =
        (fanout_queue_count_r == FANOUT_QUEUE_COUNT_W'(FANOUT_QUEUE_DEPTH));
    assign fanout_queue_head_logical_idx_c = fanout_queue_logical_idx_r[fanout_queue_rd_ptr_r];
    assign fanout_queue_head_tag_c = fanout_queue_tag_r[fanout_queue_rd_ptr_r];
    assign fanout_queue_head_event_time_c = fanout_queue_event_time_r[fanout_queue_rd_ptr_r];

    wire [FANOUT_ADDR_W-1:0] fanout_queue_head_direct_idx_c =
        FANOUT_ADDR_W'(fanout_queue_head_logical_idx_c);
    wire [FANOUT_ADDR_W-1:0] fanout_queue_push_base_c =
        FANOUT_ADDR_W'(worker_commit_logical_idx);
    wire [FANOUT_ADDR_W-1:0] fanout_queue_head_base_for_issue_c =
        fanout_queue_head_direct_idx_c;

    // ---------------------------------------------------------------------
    // Lane / conflict / enqueue combinational (scalar, replaces N-wide masks)
    // ---------------------------------------------------------------------
    // Scalar lane-accept: current pending addr can accept if queue isn't full
    // OR if dispatch is simultaneously consuming that same neuron (bypass).
    logic fanout_dispatch_accept_c;
    logic fanout_ingress_conflict_c;
    logic fanout_lane_accept_c;
    logic fanout_local_enqueue_valid_c;
    logic fanout_local_enqueue_ready_c;
    // Next-state for pending addr/valid after one accepted enqueue.
    logic fanout_local_pending_next_valid_c;
    logic [NEURON_IDX_W-1:0] fanout_local_pending_next_addr_c;

    always_comb begin
        // Dispatch bypass: if dispatch fires for the same neuron, that
        // neuron is being dequeued simultaneously, so it can accept.
        fanout_dispatch_accept_c =
            start_dispatch_valid &&
            worker_start_ready_for_dispatch &&
            (start_dispatch_logical_idx == fanout_local_pending_addr_r);

        // Ingress conflict: ingress is also enqueuing the same neuron
        // this cycle — defer by one cycle.
        fanout_ingress_conflict_c =
            ingress_spike_enqueue_fire &&
            (ingress_spike_target_idx == fanout_local_pending_addr_r);

        // Lane accept for the current pending neuron.
        fanout_lane_accept_c =
            (!logical_spike_full || fanout_dispatch_accept_c) &&
            !fanout_ingress_conflict_c;

        // Enqueue fires when there is a pending neuron and the lane accepts.
        fanout_local_enqueue_valid_c =
            fanout_local_pending_valid_r && fanout_lane_accept_c;
        fanout_local_enqueue_ready_c = fanout_local_enqueue_valid_c;

        // Next-state for the pending counter.
        if (fanout_local_enqueue_valid_c && fanout_local_pending_bcast_r) begin
            // Broadcast: advance to next neuron, or done if at last.
            if (fanout_local_pending_addr_r ==
                    NEURON_IDX_W'(NEURONS_PER_TILE - 1)) begin
                fanout_local_pending_next_valid_c = 1'b0;
                fanout_local_pending_next_addr_c  = '0;
            end else begin
                fanout_local_pending_next_valid_c = 1'b1;
                fanout_local_pending_next_addr_c  =
                    fanout_local_pending_addr_r + NEURON_IDX_W'(1);
            end
        end else if (fanout_local_enqueue_valid_c) begin
            // Unicast: done after one accepted enqueue.
            fanout_local_pending_next_valid_c = 1'b0;
            fanout_local_pending_next_addr_c  = '0;
        end else begin
            // No change.
            fanout_local_pending_next_valid_c = fanout_local_pending_valid_r;
            fanout_local_pending_next_addr_c  = fanout_local_pending_addr_r;
        end
    end

    // ---------------------------------------------------------------------
    // Fanout local-route view (scalar; replaces N-wide fanout_local_target_mask_c)
    // ---------------------------------------------------------------------
    logic fanout_local_route_c;
    logic fanout_local_has_target_c;
    logic fanout_local_target_is_bcast_c;
    logic [NEURON_IDX_W-1:0] fanout_local_target_idx_c;  // unicast idx; 0 for bcast

    assign fanout_local_route_c =
        fanout_active_r &&
        fanout_route_data_valid_r &&
        fanout_route_valid_r &&
        (fanout_route_dst_x_r == FANOUT_DST_X_W'(tile_x_coord)) &&
        (fanout_route_dst_y_r == FANOUT_DST_Y_W'(tile_y_coord));

    always_comb begin
        fanout_local_target_idx_c = '0;
        if (CORE_ID_W'(fanout_route_core_id_r) == MESSAGE_CORE_BCAST) begin
            fanout_local_target_is_bcast_c = 1'b1;
            fanout_local_has_target_c      = 1'b1;
        end else begin
            fanout_local_target_is_bcast_c = 1'b0;
            fanout_local_has_target_c =
                local_target_valid_for_core_id(
                    CORE_ID_W'(fanout_route_core_id_r),
                    fanout_local_target_idx_c
                );
        end
    end

    logic fanout_local_drop_c;
    logic fanout_local_blocked_c;
    logic fanout_retry_stall_defer_c;

    assign fanout_local_drop_c = fanout_local_route_c && !fanout_local_has_target_c;
    assign fanout_local_blocked_c =
        fanout_local_route_c &&
        fanout_local_has_target_c &&
        fanout_local_pending_valid_r &&
        !fanout_local_enqueue_ready_c;
    assign fanout_retry_stall_defer_c =
        fanout_active_r &&
        fanout_local_blocked_c &&
        !fanout_retry_valid_r;

    // ---------------------------------------------------------------------
    // Fanout queue pop/push/retry control (originally @ top 1255-1278)
    // ---------------------------------------------------------------------
    wire [FANOUT_ADDR_W-1:0] fanout_base_idx_c =
        FANOUT_ADDR_W'(fanout_logical_idx_r);
    wire [FANOUT_ADDR_W-1:0] fanout_len_c =
        FANOUT_ADDR_W'(1);
    wire [FANOUT_ADDR_W-1:0] fanout_first_index_c =
        first_route_index_from(fanout_base_idx_c);
    wire [FANOUT_ADDR_W-1:0] fanout_next_index_c = next_route_index(fanout_route_idx_r);
    wire fanout_next_valid_c = route_exists_after(fanout_route_idx_r, fanout_base_idx_c, fanout_len_c);

    logic fanout_queue_push_c;
    logic fanout_queue_pop_c;
    logic fanout_queue_can_accept_c;
    logic start_fanout_issue_c;
    logic route_fanout_read_issue_c;
    logic fanout_retry_force_c;
    logic fanout_retry_start_c;
    logic fanout_fire_c;
    logic fanout_route_invalid_skip_c;

    assign fanout_retry_force_c =
        fanout_retry_valid_r &&
        (fanout_retry_bypass_r == FANOUT_RETRY_BYPASS_W'(0));
    assign fanout_retry_start_c =
        !fanout_active_r &&
        fanout_retry_valid_r &&
        (fanout_retry_force_c ||
         !fanout_queue_valid_c ||
         (!fanout_queue_full_c && fanout_retry_turn_r));
    assign fanout_queue_pop_c =
        !fanout_active_r &&
        fanout_queue_valid_c &&
        (!fanout_retry_valid_r ||
         (!fanout_retry_force_c &&
          (fanout_queue_full_c || !fanout_retry_turn_r)));
    assign fanout_queue_can_accept_c = !fanout_queue_full_c || fanout_queue_pop_c;
    // Drop-on-overflow: an over-credit dispatch authorised by the
    // scheduler's stuck-escape path can reach commit while fanout_queue
    // is at its cap.  If so, the push is silently dropped — the spike's
    // downstream fanout is lost, but forward progress is restored and
    // the lost_fanout counter (below) is incremented for observability.
    // Under the throttle's normal operating regime (no stuck-escape)
    // fanout_queue_full_c is unreachable at the push instant so no drop
    // fires.
    logic fanout_queue_push_want_c;
    logic fanout_queue_push_drop_c;
    assign fanout_queue_push_want_c =
        worker_commit_valid &&
        worker_commit_ready &&
        worker_result_emit.valid &&
        worker_result_emit.data[0] &&
        worker_has_fanout;
    assign fanout_queue_push_drop_c =
        fanout_queue_push_want_c && fanout_queue_full_c && !fanout_queue_pop_c;
    assign fanout_queue_push_c =
        fanout_queue_push_want_c && !fanout_queue_push_drop_c;
    assign start_fanout_issue_c = fanout_queue_pop_c;
    assign route_fanout_read_issue_c = start_fanout_issue_c || (fanout_fire_c && fanout_next_valid_c);

    assign fanout_queue_can_accept = fanout_queue_can_accept_c;
    assign fanout_queue_count_live = fanout_queue_count_r;
    assign fanout_fire_dbg = fanout_fire_c;

    // ---------------------------------------------------------------------
    // NoC egress / local-inject combinational (originally @ top 1280-1304)
    // ---------------------------------------------------------------------
    always_comb begin
        message_packet_t fanout_packet_c;
        fanout_packet_c = message_packet_t'(fanout_packet_bits_r);
        // Tile->NoC boundary wrap: fill destination route/header fields from
        // the fanout route-bank response.
        fanout_packet_c.dst_x = TILE_COORD_W'(fanout_route_dst_x_r);
        fanout_packet_c.dst_y = TILE_COORD_W'(fanout_route_dst_y_r);
        fanout_packet_c.core_id = CORE_ID_W'(fanout_route_core_id_r);
        noc_child_valid =
            fanout_active_r &&
            fanout_route_data_valid_r &&
            fanout_route_valid_r &&
            !fanout_local_route_c;
        noc_child_payload = fanout_packet_c;
    end

    assign fanout_route_invalid_skip_c =
        fanout_active_r &&
        fanout_route_data_valid_r &&
        !fanout_route_valid_r;
    assign fanout_fire_c =
        fanout_route_invalid_skip_c ? 1'b1 :
        (fanout_local_route_c ? (fanout_local_drop_c ||
                                 (fanout_local_enqueue_ready_r &&
                                  !fanout_local_pending_next_valid_c))
                              : (noc_child_valid && noc_child_ready));

    // ---------------------------------------------------------------------
    // State bank / fanout pool drive (originally @ top 1024-1055, port-1)
    // ---------------------------------------------------------------------
    assign fanout_read_idx = fanout_logical_idx_r;
    // Drive BRAM port pins from flops -- see declaration block for the
    // timing-closure rationale.  The clocked update that computes the
    // next-cycle request lives in the main always_ff below.
    assign fanout_bank_p1_read_en    = fanout_bank_p1_read_en_r;
    assign fanout_bank_p1_neuron_idx = fanout_bank_p1_neuron_idx_r;
    assign fanout_bank_p1_index      = fanout_bank_p1_index_r;

    // Combinational next-cycle values fed into the read-request flops.
    logic [((NEURONS_PER_TILE <= 1) ? 1 :
            $clog2(NEURONS_PER_TILE))-1:0]
                                   fanout_bank_p1_neuron_idx_nxt_c;
    logic [((FANOUT_POOL_DEPTH <= 1) ? 1 :
            $clog2(FANOUT_POOL_DEPTH))-1:0]
                                   fanout_bank_p1_index_nxt_c;
    always_comb begin
        fanout_bank_p1_neuron_idx_nxt_c =
            start_fanout_issue_c ? fanout_queue_head_logical_idx_c
                                 : fanout_logical_idx_r;
        fanout_bank_p1_index_nxt_c =
            start_fanout_issue_c
                ? first_route_index_from(fanout_queue_head_base_for_issue_c)
                : (fanout_next_valid_c
                    ? fanout_next_index_c
                    : fanout_route_idx_r);
    end

    // ---------------------------------------------------------------------
    // Local loopback rv_if assignments (replaces 7 flat output ports)
    // ---------------------------------------------------------------------
    // The packet-view of `fanout_packet_bits_r` gives us the tag
    // that should be injected into the selected per-worker spike queue.  This
    // mirrors the `fanout_local_packet_c` usage in the original
    // enqueue mux (tile_top.sv lines 1206-1211).
    message_packet_t fanout_local_packet_view_c;
    message_packet_t fanout_packet_tmp;
    always_comb begin
        fanout_local_packet_view_c = message_packet_t'(fanout_packet_bits_r);
        // fanout_packet_tmp: base is the current packet with weight field updated
        // from the fanout read response.  Computed combinationally; written to
        // fanout_packet_bits_r via non-blocking assignment in always_ff.
        fanout_packet_tmp = message_packet_t'(fanout_packet_bits_r);
        if (route_fanout_read_pending_r && fanout_read_rsp_valid_c)
            fanout_packet_tmp.weight = WEIGHT_W'(fanout_read_rsp_weight_c);
    end

    assign fanout_ev.valid      = fanout_local_enqueue_valid_c;
    assign fanout_ev.rv_payload = tile_event_t'({
        NEURON_IDX_W'(fanout_local_pending_addr_r),
        fanout_route_weight_r,
        fanout_local_packet_view_c.tag,
        EVENT_TIME_W'(0)   // event_time unused in neutern
    });

    // ---------------------------------------------------------------------
    // Register update block
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n || tile_graph_state_clear) begin
            fanout_active_r <= 1'b0;
            fanout_retry_valid_r <= 1'b0;
            fanout_retry_turn_r <= 1'b0;
            fanout_retry_bypass_r <= '0;
            fanout_local_pending_valid_r  <= 1'b0;
            fanout_local_pending_addr_r   <= '0;
            fanout_local_pending_bcast_r  <= 1'b0;
            fanout_local_enqueue_ready_r  <= 1'b0;
            fanout_logical_idx_r <= '0;
            fanout_retry_logical_idx_r <= '0;
            fanout_route_idx_r <= '0;
            fanout_retry_route_idx_r <= '0;
            // fanout_packet_bits_r / fanout_retry_packet_bits_r intentionally
            // left un-reset: all readers gate on fanout_active_r /
            // fanout_route_*_valid_r / fanout_retry_valid_r, which are
            // synchronously cleared in this same block.  Resetting the wide
            // packet registers was producing a nasty classify→packet-reset
            // timing path during synthesis.
            route_fanout_read_pending_r <= 1'b0;
            fanout_bank_p1_read_en_r    <= 1'b0;
            fanout_bank_p1_neuron_idx_r <= '0;
            fanout_bank_p1_index_r      <= '0;
            fanout_route_data_valid_r <= 1'b0;
            fanout_retry_route_data_valid_r <= 1'b0;
            fanout_route_dst_x_r <= '0;
            fanout_route_dst_y_r <= '0;
            fanout_route_core_id_r <= '0;
            fanout_route_weight_r <= WEIGHT_W'(0);
            fanout_route_valid_r <= 1'b0;
            fanout_retry_dst_x_r <= '0;
            fanout_retry_dst_y_r <= '0;
            fanout_retry_core_id_r <= '0;
            fanout_retry_weight_r <= WEIGHT_W'(0);
            fanout_retry_route_valid_r <= 1'b0;
            fanout_retry_local_pending_valid_r <= '0;
            fanout_retry_local_pending_addr_r  <= '0;
            fanout_retry_local_pending_bcast_r <= '0;
            fanout_queue_rd_ptr_r <= '0;
            fanout_queue_wr_ptr_r <= '0;
            fanout_queue_count_r <= '0;
            fanout_queue_head_base_r <= '0;
            fanout_queue_drop_total_r <= 16'd0;
            for (int fanout_fifo_idx = 0; fanout_fifo_idx < FANOUT_QUEUE_DEPTH; fanout_fifo_idx++) begin
                fanout_queue_logical_idx_r[fanout_fifo_idx] <= '0;
                fanout_queue_tag_r[fanout_fifo_idx] <= '0;
                fanout_queue_event_time_r[fanout_fifo_idx] <= '0;
            end
        end else if (ena) begin
            fanout_local_enqueue_ready_r <= fanout_local_enqueue_ready_c;

            // Registered fanout-pool read-request stage.  The BRAM port
            // pins now lag one cycle behind route_fanout_read_issue_c;
            // route_fanout_read_pending_r continues to guard the
            // response-latching branch below so no additional handshake
            // change is required.
            fanout_bank_p1_read_en_r    <= route_fanout_read_issue_c;
            fanout_bank_p1_neuron_idx_r <= fanout_bank_p1_neuron_idx_nxt_c;
            fanout_bank_p1_index_r      <= fanout_bank_p1_index_nxt_c;

            // Observability: tally drop-on-overflow events.
            if (fanout_queue_push_drop_c) begin
                fanout_queue_drop_total_r <=
                    fanout_queue_drop_total_r + 16'd1;
            end

            if (route_fanout_read_pending_r && fanout_read_rsp_valid_c) begin
                route_fanout_read_pending_r <= 1'b0;
                fanout_route_data_valid_r <= 1'b1;
                fanout_route_dst_x_r <= FANOUT_DST_X_W'(tile_x_coord);
                fanout_route_dst_y_r <= FANOUT_DST_Y_W'(tile_y_coord);
                fanout_route_core_id_r <=
                    FANOUT_CORE_ID_W'(local_core_id_for_index(fanout_logical_idx_r));
                fanout_route_weight_r <= fanout_read_rsp_weight_c;
                fanout_route_valid_r <= fanout_read_rsp_valid_c;
                // fanout_packet_tmp is updated by always_comb above.
                fanout_packet_bits_r <= fanout_packet_tmp;
            end

            if (fanout_queue_pop_c) begin
                fanout_queue_rd_ptr_r <= next_fanout_queue_ptr(fanout_queue_rd_ptr_r);
                if (fanout_retry_valid_r) begin
                    fanout_retry_turn_r <= 1'b1;
                    if (fanout_retry_bypass_r != FANOUT_RETRY_BYPASS_W'(0)) begin
                        fanout_retry_bypass_r <= fanout_retry_bypass_r - FANOUT_RETRY_BYPASS_W'(1);
                    end
                end
            end

            if (fanout_queue_push_c) begin
                fanout_queue_logical_idx_r[fanout_queue_wr_ptr_r] <= worker_commit_logical_idx;
                fanout_queue_tag_r[fanout_queue_wr_ptr_r] <=
                    TAG_W'(0);  // tag: neutern has no temporal ordering, data field is only 4 bits
                fanout_queue_event_time_r[fanout_queue_wr_ptr_r] <=
                    EVENT_TIME_W'(0);  // event_time eliminated from emit struct
                fanout_queue_base_r[fanout_queue_wr_ptr_r] <= fanout_queue_push_base_c;
                fanout_queue_wr_ptr_r <= next_fanout_queue_ptr(fanout_queue_wr_ptr_r);
            end

            case ({fanout_queue_push_c, fanout_queue_pop_c})
                2'b10: begin
                    if (!fanout_queue_full_c) begin
                        fanout_queue_count_r <= fanout_queue_count_r + FANOUT_QUEUE_COUNT_W'(1);
                    end
                end
                2'b01: fanout_queue_count_r <= fanout_queue_count_r - FANOUT_QUEUE_COUNT_W'(1);
                default: begin
                end
            endcase

            if (fanout_queue_pop_c) begin
                if ((fanout_queue_count_r == FANOUT_QUEUE_COUNT_W'(1)) && fanout_queue_push_c) begin
                    fanout_queue_head_base_r <= fanout_queue_push_base_c;
                end else if (fanout_queue_count_r == FANOUT_QUEUE_COUNT_W'(1)) begin
                    fanout_queue_head_base_r <= '0;
                end else begin
                    fanout_queue_head_base_r <=
                        fanout_queue_base_r[next_fanout_queue_ptr(fanout_queue_rd_ptr_r)];
                end
            end else if (fanout_queue_push_c && !fanout_queue_valid_c) begin
                fanout_queue_head_base_r <= fanout_queue_push_base_c;
            end

            if (fanout_retry_stall_defer_c) begin
                fanout_retry_valid_r <= 1'b1;
                fanout_retry_turn_r <= 1'b0;
                fanout_retry_bypass_r <= FANOUT_RETRY_BYPASS_W'(FANOUT_QUEUE_DEPTH);
                fanout_retry_logical_idx_r <= fanout_logical_idx_r;
                fanout_retry_route_idx_r <= fanout_route_idx_r;
                fanout_retry_packet_bits_r <= fanout_packet_bits_r;
                fanout_retry_route_data_valid_r <= fanout_route_data_valid_r;
                fanout_retry_dst_x_r <= fanout_route_dst_x_r;
                fanout_retry_dst_y_r <= fanout_route_dst_y_r;
                fanout_retry_core_id_r <= fanout_route_core_id_r;
                fanout_retry_weight_r <= fanout_route_weight_r;
                fanout_retry_route_valid_r <= fanout_route_valid_r;
                fanout_retry_local_pending_valid_r <= fanout_local_pending_valid_r;
                fanout_retry_local_pending_addr_r  <= fanout_local_pending_addr_r;
                fanout_retry_local_pending_bcast_r <= fanout_local_pending_bcast_r;
                fanout_active_r <= 1'b0;
                fanout_local_pending_valid_r <= 1'b0;
                fanout_local_pending_addr_r  <= '0;
                fanout_local_pending_bcast_r <= 1'b0;
                fanout_route_data_valid_r <= 1'b0;
                fanout_route_valid_r <= 1'b0;
                fanout_route_idx_r <= '0;
                fanout_packet_bits_r <= '0;
            end

            if (fanout_retry_start_c) begin
                fanout_active_r <= 1'b1;
                fanout_retry_valid_r <= 1'b0;
                fanout_retry_turn_r <= 1'b0;
                fanout_retry_bypass_r <= '0;
                fanout_logical_idx_r <= fanout_retry_logical_idx_r;
                fanout_route_idx_r <= fanout_retry_route_idx_r;
                fanout_packet_bits_r <= fanout_retry_packet_bits_r;
                fanout_route_data_valid_r <= fanout_retry_route_data_valid_r;
                fanout_route_dst_x_r <= fanout_retry_dst_x_r;
                fanout_route_dst_y_r <= fanout_retry_dst_y_r;
                fanout_route_core_id_r <= fanout_retry_core_id_r;
                fanout_route_weight_r <= fanout_retry_weight_r;
                fanout_route_valid_r <= fanout_retry_route_valid_r;
                fanout_local_pending_valid_r <= fanout_retry_local_pending_valid_r;
                fanout_local_pending_addr_r  <= fanout_retry_local_pending_addr_r;
                fanout_local_pending_bcast_r <= fanout_retry_local_pending_bcast_r;
                route_fanout_read_pending_r <= 1'b0;
            end

            if (start_fanout_issue_c) begin
                fanout_active_r <= 1'b1;
                fanout_retry_turn_r <= fanout_retry_valid_r;
                fanout_logical_idx_r <= fanout_queue_head_logical_idx_c;
                fanout_local_pending_valid_r <= 1'b0;
                fanout_local_pending_addr_r  <= '0;
                fanout_local_pending_bcast_r <= 1'b0;
                fanout_route_idx_r <=
                    first_route_index_from(fanout_queue_head_base_for_issue_c);
                fanout_packet_bits_r <=
                    make_weighted_spike_packet(
                        fanout_queue_head_logical_idx_c,
                        '0,
                        fanout_queue_head_tag_c,
                        1'b0
                        // event_time removed from packet
                    );
                route_fanout_read_pending_r <= 1'b1;
                fanout_route_data_valid_r <= 1'b0;
            end

            // Set pending when a local route is first detected.
            // For broadcast: start at addr 0 and iterate up to N-1.
            // For unicast: set addr to the decoded target index.
            if (fanout_local_route_c &&
                fanout_route_valid_r &&
                !fanout_local_pending_valid_r) begin
                fanout_local_pending_valid_r <= fanout_local_has_target_c;
                fanout_local_pending_bcast_r <= fanout_local_target_is_bcast_c;
                fanout_local_pending_addr_r  <= fanout_local_target_is_bcast_c
                                               ? '0
                                               : fanout_local_target_idx_c;
            end

            // On accepted enqueue: advance counter (handled by comb; latch result).
            if (fanout_local_route_c &&
                fanout_local_enqueue_ready_c &&
                !fanout_fire_c) begin
                fanout_local_pending_valid_r <= fanout_local_pending_next_valid_c;
                fanout_local_pending_addr_r  <= fanout_local_pending_next_addr_c;
            end

            if (fanout_fire_c) begin
                fanout_local_pending_valid_r <= 1'b0;
                fanout_local_pending_addr_r  <= '0;
                fanout_local_pending_bcast_r <= 1'b0;
                if (fanout_next_valid_c) begin
                    fanout_route_idx_r <= fanout_next_index_c;
                    route_fanout_read_pending_r <= 1'b1;
                    fanout_route_data_valid_r <= 1'b0;
                end else begin
                    fanout_active_r <= 1'b0;
                    fanout_route_idx_r <= '0;
                    fanout_packet_bits_r <= '0;
                    fanout_route_data_valid_r <= 1'b0;
                    fanout_route_valid_r <= 1'b0;
                end
            end
        end
    end

`ifdef FORMAL
    assign formal_retry_valid_obs              = fanout_retry_valid_r;
    assign formal_stall_defer_obs              = fanout_retry_stall_defer_c;
    assign formal_retry_start_obs              = fanout_retry_start_c;
    assign formal_local_pending_valid_obs      = fanout_local_pending_valid_r;
    assign formal_local_pending_addr_obs       = fanout_local_pending_addr_r;
    assign formal_local_pending_bcast_obs      = fanout_local_pending_bcast_r;
    assign formal_fanout_active_obs            = fanout_active_r;
    assign formal_route_idx_obs                = fanout_route_idx_r;
`endif

endmodule

`default_nettype wire
