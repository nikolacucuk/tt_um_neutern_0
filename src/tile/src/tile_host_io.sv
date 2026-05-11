`default_nettype none

// tile_host_io
// ---------------------------------------------------------------------------
// Host-side tile output path.  It arbitrates immediate ingress responses,
// asynchronous ucode/weight/route reads, dump frames, and runtime MSG_OUTPUT
// spikes into one registered rv_host_out stream.
//
//   ingress response -----> host_pending_r -----> rv_host_out
//   ucode/fanout read ----> host_pending_r -----> rv_host_out
//   dump FSM -------------> host_pending_r -----> rv_host_out
//   worker emit ----------> output FIFO --------> rv_host_out
//
// Host egress policy:
//   * host_pending_r has priority over runtime emit FIFO entries
//   * dump/read slots block new host read issues until their response returns
//   * runtime emits are one-shot guarded against a stuck worker_commit_valid
(* keep_hierarchy = "no" *)
`ifndef YOSYS
/* verilator lint_off IMPORTSTAR */
import tile_pkg::*;
/* verilator lint_on IMPORTSTAR */
`endif
module tile_host_io
  #(
    parameter int unsigned LOCAL_Z = 0,
    parameter int unsigned NEURONS_PER_TILE = 2,
    parameter int unsigned WORKER_CORES_PER_TILE = 1,
    parameter int unsigned FANOUT_POOL_DEPTH = 256,
    parameter int unsigned MESSAGE_W = 83,
    parameter int unsigned HOST_OUTPUT_FIFO_DEPTH = 4,
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
    input  wire                       graph_state_clear,

    // rv_host_out stream
    rv_if.tx                          rv_host_out,

    // From tile_ingress: host-response trigger + classified packet snapshot.
    input  wire                       current_host_response_valid,
    input  message_packet_t           current_host_response_payload,
    input  wire                       ucode_host_read_issue,
    input  wire                       weight_host_read_issue,
    input  wire                       route_host_read_issue,
    input  wire                       dump_neuron_issue,
    input  message_packet_t           current_packet,
    input  wire                       host_reserve_current,

    // Registered view back to ingress for consume gating.
    output wire                       host_slot_free_r_out,
    output wire                       dump_active,

    // Ucode bank port 0 (shared host/dump read).  The mux selects between
    // worker 0's ucode read lane and the aux (host/dump) request — driven by
    // this module as (port0_ucode_aux_req, port0_ucode_aux_is_dump); the
    // core's bank-read mux reads those and drives the bank port-0
    // neuron_idx / word_index.
    output wire                       port0_ucode_aux_req,
    output wire                       port0_ucode_aux_is_dump,
    output wire [4:0]                 port0_ucode_aux_word_index,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                       port0_ucode_aux_neuron_idx,
    // Registered "an aux (host or dump) read is about to land on port 0
    // this cycle" — used by the core's mux to suppress the worker-0 lane.
    output wire                       port0_ucode_aux_in_flight,
    input  wire                       ucode_rsp_valid_port0,
    input  wire [15:0]                ucode_rsp_word_port0,

    // Selected target neuron (from ingress classify) — needed for port0
    // host-read neuron_idx / for fanout-pool port-0 weight/route lookups.
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                       selected_target_idx,

    // Fanout pool port 0 — request/response for host/dump reads.  The core
    // uses `dump_fanout_read_issue` to select dump_target vs selected_target,
    // and selects between `state_target_fanout_ptr` (host) and
    // `state_dump_fanout_ptr` (dump) for the index.  Outputs to bank are
    // driven on port 0 by the core mux.
    output wire                       fanout_bank_port0_read_en,
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                       fanout_bank_port0_neuron_idx,
    output wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       fanout_bank_port0_index,
    // Fanout pool read response — structured rv_if (fanout_read_rsp_t payload).
    rv_if.rx                           fanout_rsp,

    // State bank dump-read port.  This module drives `dump_target_idx`
    // (= dump_target_idx_r) to the state bank's dump_read_idx input.
    output wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                       dump_target_idx,
    input  wire [23:0]                state_dump_init_rf_flat,
    input  wire [4:0]                 state_dump_ucode_ptr,
    input  wire [4:0]                 state_dump_ucode_len,
    input  wire [7:0]                 state_dump_neuron_flags,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       state_dump_fanout_ptr,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       state_dump_fanout_len,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 :
                  $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       state_target_fanout_ptr,
    input  wire [NEURONS_PER_TILE-1:0] host_egress_en_bus,
    input  wire [NEURONS_PER_TILE-1:0] noc_egress_en_bus,
    // ctx_read_last_time_b removed: timestamps eliminated

    // Worker commit + emit lanes (from dispatch/fanout-inline).
    input  wire                       worker_commit_valid,
    input  wire                       worker_commit_ready,
    input  wire [((WORKER_CORES_PER_TILE <= 1) ? 1 :
                  $clog2(WORKER_CORES_PER_TILE))-1:0]
                                       worker_commit_idx,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 :
                  $clog2(NEURONS_PER_TILE))-1:0]
                                       worker_commit_logical_idx,
    input  neuron_emit_t              worker_result_emit
);

    /* verilator lint_off VARHIDDEN */
    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE); /* verilator lint_on VARHIDDEN */
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    localparam int unsigned HOST_OUTPUT_FIFO_PTR_W =
        (HOST_OUTPUT_FIFO_DEPTH <= 1) ? 1 : $clog2(HOST_OUTPUT_FIFO_DEPTH);
    localparam int unsigned HOST_OUTPUT_FIFO_COUNT_W =
        $clog2(HOST_OUTPUT_FIFO_DEPTH + 1);

    localparam logic [2:0] DUMP_SECTION_STATE  = 3'd0;
    localparam logic [2:0] DUMP_SECTION_UCODE  = 3'd1;
    localparam logic [2:0] DUMP_SECTION_FANOUT = 3'd2;
    localparam logic [4:0] DUMP_STATE_FRAME_COUNT = 5'd4;
    localparam logic [4:0] DUMP_UCODE_FRAME_COUNT = 5'd16;

    localparam logic [3:0] DUMP_STATE_IDLE         = 4'd0;
    localparam logic [3:0] DUMP_STATE_STATE        = 4'd1;
    localparam logic [3:0] DUMP_STATE_UCODE_ISSUE  = 4'd2;
    localparam logic [3:0] DUMP_STATE_UCODE_WAIT   = 4'd3;
    localparam logic [3:0] DUMP_STATE_FANOUT_ISSUE = 4'd4;
    localparam logic [3:0] DUMP_STATE_FANOUT_WAIT  = 4'd5;
    localparam logic [3:0] DUMP_STATE_EMIT         = 4'd6;
    localparam logic [3:0] DUMP_STATE_BRAM_WAIT    = 4'd7;

    wire rv_host_out_ready = rv_host_out.ready;
    logic rv_host_out_valid;
    logic [MESSAGE_W-1:0] rv_host_out_payload;
    assign rv_host_out.valid = rv_host_out_valid;
    assign rv_host_out.rv_payload = rv_host_out_payload;

    // ---------------------------------------------------------------------
    // Shared helpers.  MUST be included at module scope so Vivado accepts
    // the module-parameter references inside the function bodies.
    // ---------------------------------------------------------------------
    `include "tile_pkg.svh"

    function automatic [7:0] fanout_count_low8(input logic [FANOUT_ADDR_W-1:0] fcl8_val);
        begin
            fanout_count_low8 = 8'(fcl8_val);
        end
    endfunction

    function automatic [4:0] fanout_count_low5(input logic [FANOUT_ADDR_W-1:0] fcl5_val);
        begin
            fanout_count_low5 = 5'(fcl5_val);
        end
    endfunction

    function automatic [7:0] fanout_base_low8(input logic [FANOUT_ADDR_W-1:0] fbl8_val);
        begin
            fanout_base_low8 = 8'(fbl8_val);
        end
    endfunction

    function automatic [15:0] pack_dump_coord(
        input logic pdc_valid,
        input logic [7:0] coord_x,
        input logic [7:0] coord_y
    );
        begin
            pack_dump_coord = {coord_y[3:0], coord_x[3:0], pdc_valid, 7'd0};
        end
    endfunction

    function automatic [HOST_OUTPUT_FIFO_PTR_W-1:0] next_host_output_fifo_ptr(
        input logic [HOST_OUTPUT_FIFO_PTR_W-1:0] ptr
    );
        begin
            if (HOST_OUTPUT_FIFO_DEPTH <= 1) begin
                next_host_output_fifo_ptr = '0;
            end else if (ptr == HOST_OUTPUT_FIFO_PTR_W'(HOST_OUTPUT_FIFO_DEPTH - 1)) begin
                next_host_output_fifo_ptr = '0;
            end else begin
                next_host_output_fifo_ptr = ptr + HOST_OUTPUT_FIFO_PTR_W'(1);
            end
        end
    endfunction

    function automatic message_packet_t make_read_rsp_packet(
        input message_packet_t request
    );
        message_packet_t pkt_rsp;
        begin
            pkt_rsp = request;
            pkt_rsp.kind = MSG_READ_RSP;
            pkt_rsp.broadcast = 1'b0;
            pkt_rsp.dst_x = '0;  // single-tile: no routing needed
            pkt_rsp.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            pkt_rsp.core_id = request.core_id;
            make_read_rsp_packet = pkt_rsp;
        end
    endfunction

    function automatic message_packet_t make_ucode_host_response_packet(
        input message_packet_t request,
        input logic [15:0] word
    );
        message_packet_t pkt_ucode;
        begin
            pkt_ucode = make_read_rsp_packet(request);
            pkt_ucode.data = word[7:0];
            // data_hi removed from message_packet_t (DATA_W narrowed, neutern lean)
            make_ucode_host_response_packet = pkt_ucode;
        end
    endfunction

    function automatic message_packet_t make_route_host_response_packet(
        input message_packet_t request,
        input logic [FANOUT_DST_X_W-1:0]  dst_x,
        input logic [FANOUT_DST_Y_W-1:0]  dst_y,
        input logic [FANOUT_CORE_ID_W-1:0] core_id,
        input logic rhr_valid
    );
        message_packet_t pkt_route;
        begin
            pkt_route = make_read_rsp_packet(request);
            pkt_route.data = dst_x;
            // data_hi removed from message_packet_t (neutern lean profile)
            pkt_route.core_id = CORE_ID_W'(core_id);
            pkt_route.meta = {(META_W-1)'(0), rhr_valid};
            make_route_host_response_packet = pkt_route;
        end
    endfunction

    function automatic message_packet_t make_weight_host_response_packet(
        input message_packet_t request,
        input logic signed [7:0] weight
    );
        message_packet_t pkt_weight;
        begin
            pkt_weight = make_read_rsp_packet(request);
            pkt_weight.data = weight[7:0];
            // data_hi removed from message_packet_t (neutern lean profile)
            make_weight_host_response_packet = pkt_weight;
        end
    endfunction

    function automatic message_packet_t make_dump_rsp_packet(
        input message_packet_t request,
        input logic [2:0] section,
        input logic [4:0] frame_index,
        input logic [31:0] payload
    );
        message_packet_t pkt_dump;
        begin
            pkt_dump = make_read_rsp_packet(request);
            pkt_dump.cmd_kind = CMD_DUMP;
            pkt_dump.prog_index = frame_index;
            pkt_dump.addr = {1'b0, section};
            pkt_dump.data = payload[7:0];
            // data_hi removed from message_packet_t (neutern lean profile)
            pkt_dump.meta = payload[23:16];
            // event_time removed from message_packet_t (neutern profile)
            pkt_dump.tag = payload[TAG_W-1:0];  // use tag-width bits
            pkt_dump.sid = '0;
            pkt_dump.weight = '0;
            make_dump_rsp_packet = pkt_dump;
        end
    endfunction

    // NOTE: Only called with MSG_OUTPUT (unicast to host, broadcast=0).
    function automatic message_packet_t make_runtime_packet(
        input logic [NEURON_IDX_W-1:0] logical_idx,
        input logic [3:0]              kind,
        input logic [7:0]              emit_data
        // event_time removed: neutern does not use temporal event ordering
    );
        message_packet_t pkt_rt;
        logic [CORE_ID_W-1:0] local_core_id;
        begin
            pkt_rt = '0;
            local_core_id = CORE_ID_W'(local_core_id_for_index(logical_idx));
            pkt_rt.kind     = kind;
            pkt_rt.cmd_kind = CMD_MESSAGE;
            pkt_rt.broadcast = (kind != MSG_OUTPUT);
            pkt_rt.dst_x = '0;
            pkt_rt.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            pkt_rt.core_id  = local_core_id;
            pkt_rt.data     = emit_data;
            pkt_rt.sid      = local_core_id[MSG_SID_W-1:0];
            pkt_rt.tag      = emit_data[TAG_W+4:5];  // bits [5+TAG_W-1:5]
            pkt_rt.weight   = '0;
            pkt_rt.meta     = packet_meta_with_plane(8'h00, PKT_PLANE_DATA);
            make_runtime_packet = pkt_rt;
        end
    endfunction

    // ---------------------------------------------------------------------
    // Registers
    // ---------------------------------------------------------------------
    (* ram_style = "distributed" *)
    logic [MESSAGE_W-1:0] host_output_fifo_r [0:HOST_OUTPUT_FIFO_DEPTH-1];
    logic [HOST_OUTPUT_FIFO_PTR_W-1:0]   host_output_rd_ptr_r;
    logic [HOST_OUTPUT_FIFO_PTR_W-1:0]   host_output_wr_ptr_r;
    logic [HOST_OUTPUT_FIFO_COUNT_W-1:0] host_output_count_r;

    logic                  rv_host_out_valid_r;
    logic [MESSAGE_W-1:0]  rv_host_out_payload_r;
    logic                  rv_host_out_from_pending_r;

    logic                  host_pending_valid_r;
    logic [MESSAGE_W-1:0]  host_pending_payload_r;
    logic                  host_slot_free_r;

    logic                  ucode_host_read_pending_r;
    logic [MESSAGE_W-1:0]  ucode_host_read_request_r;
    logic                  weight_host_read_pending_r;
    logic [MESSAGE_W-1:0]  weight_host_read_request_r;
    logic                  route_host_read_pending_r;
    logic [MESSAGE_W-1:0]  route_host_read_request_r;

    logic                  dump_active_r;
    logic [3:0]            dump_state_r;
    logic [MESSAGE_W-1:0]  dump_request_r;
    logic [NEURON_IDX_W-1:0] dump_target_idx_r;
    logic [4:0]            dump_frame_idx_r;
    logic [2:0]            dump_lane_idx_r;
    logic [31:0]           dump_payload_r;
    logic [2:0]            dump_section_r;

    // Port-0 ucode aux pipeline — tracks whether the port-0 bank read is an
    // aux (host/dump) request and whether that aux request is a dump read.
    logic port0_ucode_aux_stage0_valid_r;
    logic port0_ucode_aux_stage1_valid_r;
    logic port0_ucode_aux_stage2_valid_r;
    logic port0_ucode_aux_stage0_is_dump_r;
    logic port0_ucode_aux_stage1_is_dump_r;
    logic port0_ucode_aux_stage2_is_dump_r;

    // ---------------------------------------------------------------------
    // Combinational views
    // ---------------------------------------------------------------------
    logic host_output_fifo_valid_c;
    logic host_output_fifo_full_c;
    logic host_output_fifo_pop_c;
    logic host_output_commit_push_c;
    logic host_output_commit_push_armed_c;
    logic host_output_fifo_drop_oldest_c;
    logic [MESSAGE_W-1:0] host_output_fifo_head_payload_c;

    // ---------------------------------------------------------------------
    // Emit-push one-shot latch (2026-04-21 fix, flagged by
    // `tile_host_io_formal::emit_one_shot`).
    //
    // `host_output_commit_push_c` is a pure combinational AND across
    // five input lanes.  If every lane sits stable-high across two
    // consecutive cycles (same worker_commit_idx, same emit data, etc.)
    // the AND fires in both cycles and the same payload lands in the
    // host FIFO twice.  The scheduler's current single-cycle handshake
    // contract makes that unreachable in the shipped design, but nothing
    // inside this module *enforces* the contract — so any future
    // sibling-module change (stalled worker, multi-cycle ready, arbiter
    // re-tries) would silently duplicate a spike packet into
    // rv_host_out.
    //
    // `host_output_commit_pushed_r` latches "I have already pushed a
    // packet for the currently-held worker_commit_idx under this
    // valid-high window."  It clears when worker_commit_valid drops
    // (a new window starts on the next rise).  `worker_commit_idx` is
    // carried along so back-to-back commits of *different* workers
    // (legal: the scheduler's RR can retire worker A, then arbitrate
    // immediately to worker B without valid dropping) are not
    // spuriously suppressed.
    //
    // Effect on normal operation: none.  Under the existing
    // single-cycle handshake, the latch rises exactly once per
    // transaction and clears before the next one, so the guard never
    // triggers a real push.  It only activates if some future bug
    // reintroduces the stuck-high scenario.
    // ---------------------------------------------------------------------
    logic host_output_commit_pushed_r;
    logic [((WORKER_CORES_PER_TILE <= 1) ? 1 :
             $clog2(WORKER_CORES_PER_TILE))-1:0]
                                      host_output_commit_pushed_idx_r;

    logic dump_ucode_read_issue_c;
    logic dump_fanout_read_issue_c;

    logic port0_ucode_aux_req_c;
    logic port0_ucode_aux_is_dump_c;
    logic [4:0] port0_ucode_aux_word_index_c;
    logic [NEURON_IDX_W-1:0] port0_ucode_aux_neuron_idx_c;
    logic port0_ucode_host_valid_c;
    logic port0_ucode_dump_valid_c;

    logic [15:0] ucode_host_read_word_c;

    logic fanout_bank_port0_read_en_c;
    logic [NEURON_IDX_W-1:0] fanout_bank_port0_neuron_idx_c;
    logic [FANOUT_ADDR_W-1:0] fanout_bank_port0_index_c;

    logic signed [7:0] weight_host_read_value_c;
    logic [FANOUT_DST_X_W-1:0]  route_host_read_dst_x_c;
    logic [FANOUT_DST_Y_W-1:0]  route_host_read_dst_y_c;
    logic [FANOUT_CORE_ID_W-1:0] route_host_read_core_id_c;
    logic       route_host_rsp_valid_c;

    logic [MESSAGE_W-1:0] ucode_host_response_payload_c;
    logic [MESSAGE_W-1:0] weight_host_response_payload_c;
    logic [MESSAGE_W-1:0] route_host_response_payload_c;
    logic [MESSAGE_W-1:0] dump_response_payload_c;

    // ---------------------------------------------------------------------
    // Host output FIFO / rv_host_out assigns.
    // ---------------------------------------------------------------------
    assign host_output_fifo_valid_c =
        (host_output_count_r != HOST_OUTPUT_FIFO_COUNT_W'(0));
    assign host_output_fifo_full_c =
        (host_output_count_r == HOST_OUTPUT_FIFO_COUNT_W'(HOST_OUTPUT_FIFO_DEPTH));
    assign host_output_fifo_head_payload_c =
        host_output_fifo_r[host_output_rd_ptr_r];
    assign host_output_fifo_pop_c =
        ena &&
        rv_host_out_valid_r &&
        rv_host_out_ready &&
        !rv_host_out_from_pending_r;
    assign host_output_fifo_drop_oldest_c =
        host_output_commit_push_c &&
        host_output_fifo_full_c &&
        !host_output_fifo_pop_c;
    // Edge guard: suppress the push when we have already pushed for
    // this worker during the current worker_commit_valid window.  See
    // the host_output_commit_pushed_r declaration above for rationale.
    assign host_output_commit_push_armed_c =
        !(host_output_commit_pushed_r &&
          (host_output_commit_pushed_idx_r == worker_commit_idx));

    assign host_output_commit_push_c =
        worker_commit_valid &&
        worker_commit_ready &&
        host_output_commit_push_armed_c &&
        worker_result_emit.valid &&
        host_egress_en_bus[worker_commit_logical_idx] &&
        worker_result_emit.data[0];

    assign rv_host_out_valid   = rv_host_out_valid_r;
    assign rv_host_out_payload = rv_host_out_payload_r;

    assign host_slot_free_r_out = host_slot_free_r;
    assign dump_active          = dump_active_r;

    // Compact ingress no longer reserves host slots for bulk loader writes;
    // keep the port for scheduler/host path API stability.
    wire unused_host_reserve_current = host_reserve_current;

    // ---------------------------------------------------------------------
    // Port-0 ucode aux (host/dump) request + 3-stage aux-read pipeline.
    // The mux in the core picks aux vs worker0 lane based on
    // port0_ucode_aux_req; we drive the aux request fields here.
    // ---------------------------------------------------------------------
    always_comb begin
        dump_ucode_read_issue_c  = 1'b0;
        dump_fanout_read_issue_c = 1'b0;
        if (dump_active_r) begin
            case (dump_state_r)
                DUMP_STATE_UCODE_ISSUE:  dump_ucode_read_issue_c = 1'b1;
                DUMP_STATE_FANOUT_ISSUE: dump_fanout_read_issue_c = 1'b1;
                default: begin
                end
            endcase
        end
    end

    assign port0_ucode_aux_req_c      = ucode_host_read_issue || dump_ucode_read_issue_c;
    assign port0_ucode_aux_is_dump_c  = dump_ucode_read_issue_c;
    assign port0_ucode_aux_neuron_idx_c =
        port0_ucode_aux_is_dump_c ? dump_target_idx_r : selected_target_idx;
    assign port0_ucode_aux_word_index_c =
        port0_ucode_aux_is_dump_c
            ? {dump_frame_idx_r[3:0], dump_lane_idx_r[0]}
            : current_packet.prog_index;

    assign port0_ucode_aux_req        = port0_ucode_aux_req_c;
    assign port0_ucode_aux_is_dump    = port0_ucode_aux_is_dump_c;
    assign port0_ucode_aux_neuron_idx = port0_ucode_aux_neuron_idx_c;
    assign port0_ucode_aux_word_index = port0_ucode_aux_word_index_c;
    assign port0_ucode_aux_in_flight  = port0_ucode_aux_stage2_valid_r;

    // Ucode read-response interface (port-0):
    //   valid = ucode read response beat present
    wire ucode_read_rsp_valid_c = ucode_rsp_valid_port0;
    wire [15:0] ucode_read_rsp_word_c = ucode_rsp_word_port0;

    assign ucode_host_read_word_c = ucode_read_rsp_word_c;
    assign port0_ucode_host_valid_c =
        ucode_read_rsp_valid_c &&
        port0_ucode_aux_stage2_valid_r &&
        !port0_ucode_aux_stage2_is_dump_r;
    assign port0_ucode_dump_valid_c =
        ucode_read_rsp_valid_c &&
        port0_ucode_aux_stage2_valid_r &&
        port0_ucode_aux_stage2_is_dump_r;

    // ---------------------------------------------------------------------
    // Fanout-bank port-0 host/dump routing.  Enable asserts on any of the
    // 3 triggers (weight/route/dump).  Index selects the right base + offset.
    // ---------------------------------------------------------------------
    assign fanout_bank_port0_read_en_c =
        weight_host_read_issue || route_host_read_issue || dump_fanout_read_issue_c;
    assign fanout_bank_port0_neuron_idx_c =
        dump_fanout_read_issue_c ? dump_target_idx_r : selected_target_idx;
    assign fanout_bank_port0_index_c =
        dump_fanout_read_issue_c
            ? (state_dump_fanout_ptr + FANOUT_ADDR_W'(dump_frame_idx_r))
            : (state_target_fanout_ptr +
               FANOUT_ADDR_W'(current_packet.prog_index));

    assign fanout_bank_port0_read_en    = fanout_bank_port0_read_en_c;
    assign fanout_bank_port0_neuron_idx = fanout_bank_port0_neuron_idx_c;
    assign fanout_bank_port0_index      = fanout_bank_port0_index_c;

    // Fanout read-response interface (single-lane):
    //   valid = read response beat present
    //   meta_valid = route-entry valid bit for this beat
    fanout_read_rsp_t fanout_rsp_data_c;
    assign fanout_rsp.ready           = 1'b1;  // pool response always consumed
    assign fanout_rsp_data_c = fanout_read_rsp_t'(fanout_rsp.rv_payload);

    wire fanout_read_rsp_valid_c      = fanout_rsp.valid;
    wire fanout_read_rsp_meta_valid_c = fanout_rsp_data_c.meta_valid;
    wire signed [WEIGHT_W-1:0] fanout_read_rsp_weight_c  = fanout_rsp_data_c.weight;
    wire [FANOUT_DST_X_W-1:0]  fanout_read_rsp_dst_x_c   = FANOUT_DST_X_W'(fanout_rsp_data_c.dst_x);
    wire [FANOUT_DST_Y_W-1:0]  fanout_read_rsp_dst_y_c   = FANOUT_DST_Y_W'(fanout_rsp_data_c.dst_y);
    wire [FANOUT_CORE_ID_W-1:0] fanout_read_rsp_core_id_c = FANOUT_CORE_ID_W'(fanout_rsp_data_c.core_id);

    assign weight_host_read_value_c  = fanout_read_rsp_weight_c;
    assign route_host_read_dst_x_c   = fanout_read_rsp_dst_x_c;
    assign route_host_read_dst_y_c   = fanout_read_rsp_dst_y_c;
    assign route_host_read_core_id_c = fanout_read_rsp_core_id_c;
    assign route_host_rsp_valid_c    =
        fanout_read_rsp_valid_c && fanout_read_rsp_meta_valid_c;

    assign dump_target_idx = dump_target_idx_r;

    // ---------------------------------------------------------------------
    // Ucode/weight/route/dump response payload combinational builders.
    // ---------------------------------------------------------------------
    always_comb begin
        ucode_host_response_payload_c = make_ucode_host_response_packet(
            message_packet_t'(ucode_host_read_request_r),
            ucode_host_read_word_c
        );
    end

    always_comb begin
        route_host_response_payload_c = make_route_host_response_packet(
            message_packet_t'(route_host_read_request_r),
            route_host_read_dst_x_c,
            route_host_read_dst_y_c,
            route_host_read_core_id_c,
            route_host_rsp_valid_c
        );
    end

    always_comb begin
        weight_host_response_payload_c = make_weight_host_response_packet(
            message_packet_t'(weight_host_read_request_r),
            weight_host_read_value_c
        );
    end

    always_comb begin
        dump_response_payload_c = make_dump_rsp_packet(
            message_packet_t'(dump_request_r),
            dump_section_r,
            dump_frame_idx_r,
            dump_payload_r
        );
    end

    // ---------------------------------------------------------------------
    // Register update (reset + ena-gated sequential).  Mirrors the
    // behaviour of the original tile_top always_ff block for
    // the host_*/dump_* registers.
    // ---------------------------------------------------------------------
    logic [31:0] dump_csr_payload_c;
    logic [23:0] dump_init_rf_c;
    always_ff @(posedge clk) begin
        if (!rst_n || graph_state_clear) begin
            host_pending_valid_r <= 1'b0;
            host_pending_payload_r <= '0;
            host_slot_free_r <= 1'b1;
            rv_host_out_valid_r <= 1'b0;
            rv_host_out_payload_r <= '0;
            rv_host_out_from_pending_r <= 1'b0;
            ucode_host_read_pending_r <= 1'b0;
            ucode_host_read_request_r <= '0;
            port0_ucode_aux_stage0_valid_r <= 1'b0;
            port0_ucode_aux_stage1_valid_r <= 1'b0;
            port0_ucode_aux_stage2_valid_r <= 1'b0;
            port0_ucode_aux_stage0_is_dump_r <= 1'b0;
            port0_ucode_aux_stage1_is_dump_r <= 1'b0;
            port0_ucode_aux_stage2_is_dump_r <= 1'b0;
            weight_host_read_pending_r <= 1'b0;
            weight_host_read_request_r <= '0;
            route_host_read_pending_r <= 1'b0;
            route_host_read_request_r <= '0;
            dump_active_r <= 1'b0;
            dump_state_r <= DUMP_STATE_IDLE;
            dump_request_r <= '0;
            dump_target_idx_r <= '0;
            dump_frame_idx_r <= '0;
            dump_lane_idx_r <= '0;
            dump_payload_r <= '0;
            dump_section_r <= DUMP_SECTION_STATE;
            host_output_rd_ptr_r <= '0;
            host_output_wr_ptr_r <= '0;
            host_output_count_r <= '0;
            host_output_commit_pushed_r <= 1'b0;
            host_output_commit_pushed_idx_r <= '0;
            // Intentionally leave host_output_fifo_r uninitialized on reset:
            // rd_ptr/wr_ptr/count all start at 0 so no stale entry is ever
            // read.  Clearing the memory in parallel blocks LUTRAM inference
            // (Synth 8-7186), forcing the storage into ~380 flops per tile.
        end else if (ena) begin
            // Emit-push one-shot bookkeeping.  Clears when no commit is
            // in flight (end of window); sets when a push actually fires
            // and remembers which worker it was for.  See the
            // declaration site for the full rationale.
            if (!worker_commit_valid) begin
                host_output_commit_pushed_r <= 1'b0;
            end else if (host_output_commit_push_c) begin
                host_output_commit_pushed_r     <= 1'b1;
                host_output_commit_pushed_idx_r <= worker_commit_idx;
            end

            host_slot_free_r <=
                (!host_pending_valid_r || !rv_host_out_valid_r || rv_host_out_ready) &&
                !dump_active_r &&
                !ucode_host_read_pending_r &&
                !weight_host_read_pending_r &&
                !route_host_read_pending_r;

            if (rv_host_out_valid_r && rv_host_out_ready) begin
                rv_host_out_valid_r <= 1'b0;
                rv_host_out_payload_r <= '0;
                rv_host_out_from_pending_r <= 1'b0;
            end else if (!rv_host_out_valid_r) begin
                rv_host_out_valid_r <= host_pending_valid_r || host_output_fifo_valid_c;
                rv_host_out_payload_r <=
                    host_pending_valid_r ? host_pending_payload_r : host_output_fifo_head_payload_c;
                rv_host_out_from_pending_r <= host_pending_valid_r;
            end

            port0_ucode_aux_stage2_valid_r <= port0_ucode_aux_stage1_valid_r;
            port0_ucode_aux_stage1_valid_r <= port0_ucode_aux_stage0_valid_r;
            // Original: stage0 <= bank_read_en[0] && aux_req.  Since
            // bank_read_en[0] == aux_req || worker0_valid, the AND with
            // aux_req collapses to aux_req.  Semantically equivalent.
            port0_ucode_aux_stage0_valid_r <= port0_ucode_aux_req_c;
            port0_ucode_aux_stage2_is_dump_r <= port0_ucode_aux_stage1_is_dump_r;
            port0_ucode_aux_stage1_is_dump_r <= port0_ucode_aux_stage0_is_dump_r;
            port0_ucode_aux_stage0_is_dump_r <= port0_ucode_aux_is_dump_c;

            if (ucode_host_read_pending_r && port0_ucode_host_valid_c) begin
                ucode_host_read_pending_r <= 1'b0;
            end

            if (weight_host_read_pending_r && fanout_read_rsp_valid_c) begin
                weight_host_read_pending_r <= 1'b0;
            end

            if (route_host_read_pending_r && fanout_read_rsp_valid_c) begin
                route_host_read_pending_r <= 1'b0;
            end

            if (dump_neuron_issue) begin
                dump_active_r <= 1'b1;
                dump_state_r <= DUMP_STATE_BRAM_WAIT;
                dump_request_r <= current_packet;
                dump_target_idx_r <= selected_target_idx;
                dump_frame_idx_r <= 5'd0;
                dump_lane_idx_r <= 3'd0;
                dump_payload_r <= 32'd0;
                dump_section_r <= DUMP_SECTION_STATE;
            end

            if (dump_active_r) begin
                case (dump_state_r)
                    DUMP_STATE_BRAM_WAIT: begin
                        dump_state_r <= DUMP_STATE_STATE;
                    end
                    DUMP_STATE_STATE: begin
                        if (!host_pending_valid_r || rv_host_out_ready) begin
                            dump_init_rf_c = state_dump_init_rf_flat;
                            case (dump_frame_idx_r)
                                5'd0: dump_csr_payload_c = {
                                    fanout_count_low8(state_dump_fanout_len),
                                    {3'b000, state_dump_ucode_len},
                                    {3'b000, state_dump_ucode_ptr},
                                    {3'b000, noc_egress_en_bus[dump_target_idx_r],
                                     host_egress_en_bus[dump_target_idx_r], 3'b000}
                                };
                                5'd1: dump_csr_payload_c = {
                                    dump_init_rf_c[23:16],  // rf[2]=AUX
                                    8'h00,                  // last_time eliminated
                                    dump_init_rf_c[15:8],
                                    dump_init_rf_c[7:0]
                                };
                                5'd2: dump_csr_payload_c = {
                                    16'd0,
                                    fanout_base_low8(state_dump_fanout_ptr),
                                    state_dump_neuron_flags
                                };
                                default: dump_csr_payload_c = 32'd0;  // rf[3] eliminated, no extra byte
                            endcase
                            host_pending_valid_r <= 1'b1;
                            host_pending_payload_r <= make_dump_rsp_packet(
                                message_packet_t'(dump_request_r),
                                DUMP_SECTION_STATE,
                                dump_frame_idx_r,
                                dump_csr_payload_c
                            );
                            if (dump_frame_idx_r == (DUMP_STATE_FRAME_COUNT - 5'd1)) begin
                                dump_section_r <= DUMP_SECTION_UCODE;
                                dump_frame_idx_r <= 5'd0;
                                dump_lane_idx_r <= 3'd0;
                                dump_payload_r <= 32'd0;
                                dump_state_r <= DUMP_STATE_UCODE_ISSUE;
                            end else begin
                                dump_frame_idx_r <= dump_frame_idx_r + 5'd1;
                            end
                        end
                    end
                    DUMP_STATE_UCODE_ISSUE: begin
                        if (dump_lane_idx_r == 3'd0) begin
                            dump_payload_r <= 32'd0;
                        end
                        dump_state_r <= DUMP_STATE_UCODE_WAIT;
                    end
                    DUMP_STATE_UCODE_WAIT: begin
                        if (port0_ucode_dump_valid_c) begin
                            if (dump_lane_idx_r[0]) begin
                                dump_payload_r[31:16] <= ucode_host_read_word_c;
                                dump_section_r <= DUMP_SECTION_UCODE;
                                dump_lane_idx_r <= 3'd0;
                                dump_state_r <= DUMP_STATE_EMIT;
                            end else begin
                                dump_payload_r[15:0] <= ucode_host_read_word_c;
                                dump_lane_idx_r <= 3'd1;
                                dump_state_r <= DUMP_STATE_UCODE_ISSUE;
                            end
                        end
                    end
                    DUMP_STATE_FANOUT_ISSUE: begin
                        dump_payload_r <= 32'd0;
                        dump_state_r <= DUMP_STATE_FANOUT_WAIT;
                    end
                    DUMP_STATE_FANOUT_WAIT: begin
                        if (fanout_read_rsp_valid_c) begin
                            dump_payload_r <= {
                                8'(signed'(fanout_read_rsp_weight_c)),
                                fanout_read_rsp_core_id_c,
                                fanout_read_rsp_dst_y_c,
                                fanout_read_rsp_dst_x_c
                            };
                            dump_section_r <= DUMP_SECTION_FANOUT;
                            dump_state_r <= DUMP_STATE_EMIT;
                        end
                    end
                    DUMP_STATE_EMIT: begin
                        if (!host_pending_valid_r || rv_host_out_ready) begin
                            logic [4:0] dump_ucode_frame_count_c;
                            dump_ucode_frame_count_c =
                                (state_dump_ucode_len + 5'd1) >> 1;
                            host_pending_valid_r <= 1'b1;
                            host_pending_payload_r <= dump_response_payload_c;
                            dump_payload_r <= 32'd0;
                            case (dump_section_r)
                                DUMP_SECTION_UCODE: begin
                                    if (dump_ucode_frame_count_c == 5'd0 ||
                                        (dump_frame_idx_r + 5'd1) >= dump_ucode_frame_count_c) begin
                                        dump_frame_idx_r <= 5'd0;
                                        dump_lane_idx_r <= 3'd0;
                                        dump_section_r <= DUMP_SECTION_FANOUT;
                                        dump_state_r <= DUMP_STATE_FANOUT_ISSUE;
                                    end else begin
                                        dump_frame_idx_r <= dump_frame_idx_r + 5'd1;
                                        dump_lane_idx_r <= 3'd0;
                                        dump_state_r <= DUMP_STATE_UCODE_ISSUE;
                                    end
                                end
                                DUMP_SECTION_FANOUT: begin
                                    if ((dump_frame_idx_r + 5'd1) >=
                                        fanout_count_low5(state_dump_fanout_len)) begin
                                        dump_active_r <= 1'b0;
                                        dump_state_r <= DUMP_STATE_IDLE;
                                        dump_frame_idx_r <= 5'd0;
                                        dump_lane_idx_r <= 3'd0;
                                    end else begin
                                        dump_frame_idx_r <= dump_frame_idx_r + 5'd1;
                                        dump_lane_idx_r <= 3'd0;
                                        dump_state_r <= DUMP_STATE_FANOUT_ISSUE;
                                    end
                                end
                                default: begin
                                    dump_active_r <= 1'b0;
                                    dump_state_r <= DUMP_STATE_IDLE;
                                    dump_frame_idx_r <= 5'd0;
                                    dump_lane_idx_r <= 3'd0;
                                end
                            endcase
                        end
                    end
                    default: begin
                        dump_active_r <= 1'b0;
                        dump_state_r <= DUMP_STATE_IDLE;
                        dump_frame_idx_r <= 5'd0;
                        dump_lane_idx_r <= 3'd0;
                        dump_payload_r <= 32'd0;
                    end
                endcase
            end

            if (rv_host_out_valid_r && rv_host_out_ready && rv_host_out_from_pending_r) begin
                host_pending_valid_r <= 1'b0;
            end

            if (host_output_fifo_pop_c || host_output_fifo_drop_oldest_c) begin
                host_output_rd_ptr_r <= next_host_output_fifo_ptr(host_output_rd_ptr_r);
            end

            case ({host_output_commit_push_c, host_output_fifo_pop_c})
                2'b10: begin
                    if (!host_output_fifo_full_c) begin
                        host_output_count_r <= host_output_count_r + HOST_OUTPUT_FIFO_COUNT_W'(1);
                    end
                end
                2'b01: host_output_count_r <= host_output_count_r - HOST_OUTPUT_FIFO_COUNT_W'(1);
                default: begin
                end
            endcase

            if (ucode_host_read_pending_r && port0_ucode_host_valid_c) begin
                host_pending_valid_r <= 1'b1;
                host_pending_payload_r <= ucode_host_response_payload_c;
            end

            if (weight_host_read_pending_r && fanout_read_rsp_valid_c) begin
                host_pending_valid_r <= 1'b1;
                host_pending_payload_r <= weight_host_response_payload_c;
            end

            if (route_host_read_pending_r && fanout_read_rsp_valid_c) begin
                host_pending_valid_r <= 1'b1;
                host_pending_payload_r <= route_host_response_payload_c;
            end

            if (current_host_response_valid) begin
                host_pending_valid_r <= 1'b1;
                host_pending_payload_r <= current_host_response_payload;
            end

            if (ucode_host_read_issue) begin
                ucode_host_read_pending_r <= 1'b1;
                ucode_host_read_request_r <= current_packet;
            end

            if (weight_host_read_issue) begin
                weight_host_read_pending_r <= 1'b1;
                weight_host_read_request_r <= current_packet;
            end

            if (route_host_read_issue) begin
                route_host_read_pending_r <= 1'b1;
                route_host_read_request_r <= current_packet;
            end

            if (worker_commit_valid && worker_commit_ready) begin
                if (host_output_commit_push_c) begin
                    host_output_fifo_r[host_output_wr_ptr_r] <=
                        make_runtime_packet(
                            worker_commit_logical_idx,
                            MSG_OUTPUT,
                            worker_result_emit.data
                            // event_time removed
                        );
                    host_output_wr_ptr_r <= next_host_output_fifo_ptr(host_output_wr_ptr_r);
                end
            end
        end else begin
            host_slot_free_r <= 1'b1;
            rv_host_out_valid_r <= 1'b0;
            rv_host_out_payload_r <= '0;
            rv_host_out_from_pending_r <= 1'b0;
        end
    end

`ifdef FORMAL
    // -------------------------------------------------------------------
    // HOST_SLOT_LIVENESS — catches the class of bug where
    // `host_slot_free_r` latches low without an identifiable back-
    // pressure source.  If this fires, the tile silently refuses every
    // incoming MSG_WRITE packet from the loader (ingress consume for
    // CMD_ROUTE / CMD_WEIGHT / CMD_UCODE is gated on
    // `host_slot_free_r_out`), and the loader's watchdog trips with
    // `ERROR_IDLE_TIMEOUT` at whatever phase the burst is currently in
    // — the exact symptom seen on live hardware at progress=7680,
    // phase=PHASE_ROUTE_WORD_HI.
    //
    // The legitimate back-pressure sources (matching lines 645-650):
    //   * a host read (ucode / weight / route) is pending a bank
    //     response,
    //   * the dump FSM is running,
    //   * a pending response is waiting for rv_host_out_ready.
    //
    // The register at line 645 samples the source expression at each
    // posedge, so `host_slot_free_r` lags the source by one cycle.
    // Property: if all sources are clear for two consecutive cycles
    // (sampled at $past(...) and current), then `host_slot_free_r`
    // MUST be high on the current cycle.  A counter-example would mean
    // the register lost a clear edge — a missed reset, a mismatched
    // source enumeration, or a combinational term that stops tracking
    // the register update.
    logic host_slot_backpressure_source_c;
    always_comb begin
        host_slot_backpressure_source_c =
            dump_active_r                           ||
            ucode_host_read_pending_r               ||
            weight_host_read_pending_r              ||
            route_host_read_pending_r               ||
            (host_pending_valid_r &&
             rv_host_out_valid_r  &&
             !rv_host_out_ready);
    end

    // Pre-/-registered view so the assertion below can reference the
    // prior cycle's source state without triggering $past warnings
    // under harnesses that BMC below cycle 2.
    logic host_slot_backpressure_source_r;
    always_ff @(posedge clk) begin
        if (!rst_n || graph_state_clear) begin
            host_slot_backpressure_source_r <= 1'b0;
        end else if (ena) begin
            host_slot_backpressure_source_r <= host_slot_backpressure_source_c;
        end
    end

    // f_reset_seen starts at 0 (inline initialiser → INIT=0 attribute;
    // setundef -anyseq does not override driven registers with INIT attrs).
    // It goes high after the first posedge with !rst_n, guaranteeing that
    // the liveness assertion only fires after all synchronous FFs have
    // been through at least one active-low reset cycle.
    logic f_reset_seen = 1'b0;
    always @(posedge clk) begin
        if (!rst_n) f_reset_seen <= 1'b1;
    end

    always @(posedge clk) begin
        if (f_reset_seen && rst_n && ena && !graph_state_clear) begin
            if (!host_slot_backpressure_source_r &&
                !host_slot_backpressure_source_c) begin
                host_slot_liveness_assert:
                    assert (host_slot_free_r);
            end
        end
    end
`endif

endmodule

`default_nettype wire
