`default_nettype none

`include "tile_flit_types.vh"

// tile_ingress
// ---------------------------------------------------------------------------
// Compact tile ingress for the 1x1024-neuron stream path.  This module owns
// the rv_in skid queue, compact packet decode, local spike enqueue request,
// graph-clear pulse, bank-write muxes, and immediate host-response payloads.
//
//   rv_in --> q0/q1/q2/q3 --> classify_r --> consume/decode
//                                      |            |
//                                      |            +--> CSR/ucode/fanout writes
//                                      |            +--> host response payload
//                                      +--> spike enqueue {idx, weight, time}
//
// Compact stream contract:
//   payload_is_spike = 0: configuration/header beat, consumes locally and arms
//                         the stream (also carries graph-clear bit)
//   payload_is_spike = 1: spike event, decoded into local neuron index and
//                         enqueued only after a header has been accepted
(* keep_hierarchy = "no" *)
module tile_ingress #(
    parameter int unsigned LOCAL_Z = 0,
    parameter int unsigned NEURONS_PER_TILE = 2,
    parameter int unsigned FANOUT_POOL_DEPTH = 256,
    parameter int unsigned INGRESS_QUEUE_DEPTH = 4,
    parameter int unsigned MESSAGE_W = 83
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       ena,
    input  wire [TILE_COORD_W-1:0]         tile_x_coord,
    input  wire [TILE_COORD_W-1:0]         tile_y_coord,

    // rv_in stream
    rv_if.rx                          rv_in,

    // Registered classified packet view (drives downstream consumers).
    output message_packet_t           current_packet,
    output logic                      current_packet_valid,
    output logic                      current_local_tile_match,
    output logic                      current_core_broadcast,
    output logic                      current_has_target,
    output logic                      packet_from_loader,

    // Consume decoder outputs
    output logic                      current_packet_consume,
    output logic                      ingress_consume,
    output logic                      host_reserve_current,

    // Back-pressure / state from downstream consumers
    input  wire                       host_slot_free_r,
    input  wire                       dump_active_r,

    // Spike-queue enqueue firing (MSG_INPUT/MSG_SPIKE)
    output logic                      ingress_spike_enqueue_fire,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                      ingress_spike_target_idx,
    output logic                      ingress_spike_is_bcast,
    output logic                      ingress_spike_enqueue_ready,

    // Scalar back-pressure view from tile_top's selected per-worker spike FIFO.
    input  wire                       logical_spike_full,

    // Graph-state-clear
    output logic                      packet_graph_state_clear,
    output logic                      tile_graph_state_clear,

    // Ucode / weight / route / dump host-read triggers
    output logic                      ucode_host_read_issue,
    output logic                      weight_host_read_issue,
    output logic                      route_host_read_issue,
    output logic                      dump_neuron_issue,

    // Weight / route write enables (muxed back into fanout_write_* outputs)
    output logic                      weight_write_en,
    output logic                      route_write_en,

    // Host-response construction for host_io consumer
    output logic                      current_host_response_valid,
    output message_packet_t           current_host_response_payload,

    // Data needed for read-response path (from state bank)
    input  wire [7:0]                 state_target_csr_read_data,
    input  wire [15:0]                calc_out_packets,
    input  wire [15:0]                calc_in_packets,

    // Selected target neuron index (derived from classify).  Drives downstream
    // state bank read / ucode+fanout port0 read idx selection.
    output logic [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0]
                                       selected_target_idx,

    // Ingress -> bank write mux outputs
    // ctrl/csr path
    output logic                       ctrl_write_en_mux,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                       ctrl_write_neuron_idx_mux,
    output logic                       ctrl_write_broadcast_mux,
    output logic [3:0]                 ctrl_write_addr_mux,
    output logic [7:0]                 ctrl_write_data_mux,
    // ucode write/prog path
    input  wire                        ucode_prog_end_en,
    output logic                       ucode_write_en_mux,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                       ucode_write_neuron_idx_mux,
    output logic                       ucode_write_broadcast_mux,
    output logic [4:0]                 ucode_write_index_mux,
    output logic [NEURON_UCODE_RSP_W-1:0]  ucode_write_word_mux,
    output logic                       ucode_prog_en_mux,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                       ucode_prog_neuron_idx_mux,
    output logic                       ucode_prog_broadcast_mux,
    output logic [4:0]                 ucode_prog_ptr_mux,
    output logic [4:0]                 ucode_prog_len_mux,
    // Fanout prog (meta) path
    output logic                       fanout_prog_ptr_en_mux,
    output logic                       fanout_prog_len_en_mux,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 :
                   $clog2(NEURONS_PER_TILE))-1:0]
                                       fanout_prog_neuron_idx_mux,
    output logic                       fanout_prog_broadcast_mux,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       fanout_prog_ptr_mux,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       fanout_prog_len_mux,
    // Fanout write path — structured rv_if output (replaces 11 flat mux ports).
    // rv_if.valid = write_en; fanout_write_addr carries the table address.
    rv_if.tx                               fanout_write,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0]
                                           fanout_write_addr,

    // State-bank fanout pointer for the selected target (used by the
    // CMD_WEIGHT / CMD_ROUTE / CMD_UCODE write-index computations).
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0]
                                       state_target_fanout_ptr
);

    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE);
    localparam int unsigned SPIKE_COORD_IDX_W = 2 * NEURON_LOCAL_W;
    localparam int unsigned SPIKE_IDX_CMP_W = SPIKE_COORD_IDX_W + 1;
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    localparam int unsigned FANOUT_CORE_ID_W = CORE_ID_W;  // 8 bits — matches tile_fanout_pool default

    // Internal signals for fanout write path (driven in always_comb, then
    // packed into fanout_write rv_if + fanout_write_addr at module outputs).
    logic                        fanout_write_en_mux;
    logic                        fanout_write_mask_valid_mux;
    logic [FANOUT_ADDR_W-1:0]    fanout_write_index_mux;
    logic signed [WEIGHT_W-1:0]       fanout_write_weight_mux;
    logic [TILE_COORD_W-1:0]         fanout_write_dst_x_mux;
    logic [TILE_COORD_W-1:0]         fanout_write_dst_y_mux;
    logic [CORE_ID_W-1:0]            fanout_write_core_id_mux;
    logic [1:0]                      fanout_write_meta_mux;
    logic                        fanout_write_valid_mux;
    logic                        fanout_write_weight_only_mux;
    logic                        fanout_write_route_only_mux;
    localparam int unsigned MIN_PACKET_W = MESSAGE_PKT_MIN_W;
    localparam bit HAS_INGRESS_Q1 = (INGRESS_QUEUE_DEPTH > 1);
    localparam bit HAS_INGRESS_Q2 = (INGRESS_QUEUE_DEPTH > 2);
    localparam bit HAS_INGRESS_Q3 = (INGRESS_QUEUE_DEPTH > 3);
    localparam logic [3:0] TILE_DEBUG_ADDR_CALC = 4'hE;
    localparam logic [4:0] TILE_DEBUG_CALC_OUT_PACKETS = 5'd0;
    localparam logic [4:0] TILE_DEBUG_CALC_IN_PACKETS = 5'd1;
    // Maximum valid prog_index for CMD_ROUTE read requests.  Capped at 31
    // because prog_index is a 5-bit field; the actual pool depth may be
    // larger.  Replaces the legacy hard-coded 5'd24 which reflected the
    // old per-neuron fixed fanout of 24 before the shared-pool refactor.
    localparam logic [4:0] ROUTE_PROG_IDX_MAX =
        (FANOUT_POOL_DEPTH < 32) ? FANOUT_POOL_DEPTH[4:0] : 5'd31;

    wire rv_in_valid = rv_in.valid;
    wire [MESSAGE_W-1:0] rv_in_payload = rv_in.rv_payload;
    logic rv_in_ready;
    assign rv_in.ready = rv_in_ready;

    // ---------------------------------------------------------------------
    // Shared helpers.  MUST be included at module scope (not file scope)
    // so the function bodies can see NEURONS_PER_TILE / LOCAL_Z /
    // NEURON_IDX_W — Vivado correctly rejects the file-scope form.
    // ---------------------------------------------------------------------
    `include "tile_pkg.svh"

    // ---------------------------------------------------------------------
    // Packet-factory helpers (used only by host-response payload build).
    // These are direct copies of the make_* helpers in tile_top
    // but scoped to this module.  Duplication is acceptable per the
    // decomposition contract; a later `tile_packet_factory.svh` pass will
    // consolidate.
    // ---------------------------------------------------------------------
    function automatic message_packet_t make_status_packet(
        input message_packet_t req_stat,
        input logic [7:0] status_code
    );
        message_packet_t pkt_status;
        begin
            pkt_status = req_stat;
            pkt_status.kind = MSG_STATUS;
            pkt_status.broadcast = 1'b0;
            pkt_status.dst_x = '0;  // single-tile: no routing needed
            pkt_status.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            pkt_status.core_id = req_stat.core_id;
            pkt_status.data = status_code;
            // data_hi removed
            pkt_status.meta = packet_meta_with_plane(8'h00, PKT_PLANE_CMD);
            make_status_packet = pkt_status;
        end
    endfunction

    function automatic message_packet_t make_tile_pong_packet(
        input message_packet_t req_pong
    );
        message_packet_t pkt_pong;
        begin
            pkt_pong = req_pong;
            pkt_pong.kind = MSG_PONG;
            pkt_pong.broadcast = 1'b0;
            pkt_pong.dst_x = '0;  // single-tile: no routing needed
            pkt_pong.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            pkt_pong.core_id = CORE_ID_W'(LOCAL_Z);
            pkt_pong.prog_index = 5'd0;
            pkt_pong.addr = 4'd0;
            pkt_pong.data = HWINFO_MAGIC_LO;
            // data_hi removed; only HWINFO low nibble returned
            pkt_pong.sid = '0;
            pkt_pong.tag = '0;
            // event_time removed: neutern does not use temporal event ordering
            pkt_pong.weight = '0;
            pkt_pong.meta = packet_meta_with_plane(
                PONG_META_TILE | TILE_HEALTH_ALIVE | TILE_HEALTH_ENABLED,
                PKT_PLANE_CMD
            );
            make_tile_pong_packet = pkt_pong;
        end
    endfunction

    function automatic message_packet_t make_read_rsp_packet(
        input message_packet_t req_rsp
    );
        message_packet_t pkt_rsp;
        begin
            pkt_rsp = req_rsp;
            pkt_rsp.kind = MSG_READ_RSP;
            pkt_rsp.broadcast = 1'b0;
            pkt_rsp.dst_x = '0;  // single-tile: no routing needed
            pkt_rsp.dst_y = '0;
            // src_x / src_y removed: single-tile neutern profile
            pkt_rsp.core_id = req_rsp.core_id;
            make_read_rsp_packet = pkt_rsp;
        end
    endfunction

    function automatic message_packet_t make_local_read_response_packet(
        input message_packet_t in_packet,
        input logic [15:0] in_calc_out_packets,
        input logic [15:0] in_calc_in_packets,
        input logic [7:0] in_csr_read_data
    );
        message_packet_t pkt;
        begin
            pkt = '0;
            case (in_packet.cmd_kind)
                CMD_CSR: begin
                    pkt = make_read_rsp_packet(in_packet);
                    pkt.data = DATA_W'(in_csr_read_data);
                    // data_hi removed
                end
                CMD_DEBUG: begin
                    if (in_packet.addr != TILE_DEBUG_ADDR_CALC) begin
                        pkt = make_status_packet(in_packet, MSG_STATUS_BAD_ADDR);
                    end else begin
                        pkt = make_read_rsp_packet(in_packet);
                        unique case (in_packet.prog_index)
                            TILE_DEBUG_CALC_OUT_PACKETS: begin
                                pkt.data = in_calc_out_packets[3:0]; // low nibble only
                                // data_hi removed; high byte dropped
                            end
                            TILE_DEBUG_CALC_IN_PACKETS: begin
                                pkt.data = in_calc_in_packets[3:0]; // low nibble only
                            end
                            default: begin
                                pkt =
                                    make_status_packet(in_packet, MSG_STATUS_BAD_ADDR);
                            end
                        endcase
                    end
                end
                CMD_UCODE:   pkt = '0;
                CMD_WEIGHT:  pkt = '0;
                CMD_SYNAPSE: begin
                    pkt = make_status_packet(in_packet, MSG_STATUS_BAD_KIND);
                end
                CMD_DUMP:    pkt = '0;
                CMD_ROUTE: begin
                    if (in_packet.prog_index >= ROUTE_PROG_IDX_MAX) begin
                        pkt = make_status_packet(in_packet, MSG_STATUS_BAD_ADDR);
                    end else begin
                        pkt = '0;
                    end
                end
                default: begin
                    pkt = make_status_packet(in_packet, MSG_STATUS_BAD_KIND);
                end
            endcase
            make_local_read_response_packet = pkt;
        end
    endfunction

    // ---------------------------------------------------------------------
    // Shift queue + classify registers
    // ---------------------------------------------------------------------
    logic ingress_pending_r;
    logic [MESSAGE_W-1:0] ingress_payload_r;
    logic ingress_q1_valid_r;
    logic ingress_q2_valid_r;
    logic ingress_q3_valid_r;
    logic [MESSAGE_W-1:0] ingress_q1_payload_r;
    logic [MESSAGE_W-1:0] ingress_q2_payload_r;
    logic [MESSAGE_W-1:0] ingress_q3_payload_r;
    logic next_ingress_pending_c;
    logic [MESSAGE_W-1:0] next_ingress_payload_c;
    logic next_ingress_q1_valid_c;
    logic next_ingress_q2_valid_c;
    logic next_ingress_q3_valid_c;
    logic [MESSAGE_W-1:0] next_ingress_q1_payload_c;
    logic [MESSAGE_W-1:0] next_ingress_q2_payload_c;
    logic [MESSAGE_W-1:0] next_ingress_q3_payload_c;

    logic classify_valid_r;
    logic classify_local_tile_match_r;
    logic classify_core_broadcast_r;
    logic classify_has_target_r;
    logic [NEURON_IDX_W-1:0] classify_target_idx_r;
    logic [MESSAGE_W-1:0] classify_payload_r;

    logic classify_load_c;
    logic ingress_accept_c;

    logic ingress_classify_local_tile_match_c;
    logic ingress_classify_core_broadcast_c;
    logic ingress_classify_has_target_c;
    logic [NEURON_IDX_W-1:0] ingress_classify_target_idx_c;
    logic [SPIKE_COORD_IDX_W-1:0] ingress_spike_flat_idx_c;
    logic [CORE_ID_W-1:0] current_min_core_id_c;
    logic min_cfg_loaded_r;

    message_packet_t current_packet_c;
    message_packet_min_t ingress_packet_min_c;
    message_packet_min_t current_packet_min_c;

    // Bottom-25 view of the rv_in payload, used by the local
    // tile-to-tile spike-enqueue path that carries
    // {payload_is_spike, queue_spike{neuron_x, neuron_y, weight,
    // event_time}} packed into the lowest message_packet_t fields.
    // The host gateway uses the FULL message_packet_t layout (kind,
    // cmd_kind, dst, addr, prog_index, ...), so anything that needs
    // to dispatch on the full layout must read `current_packet_c`,
    // *not* derive everything from the compact view.  Pre-refactor
    // this module simply did `current_packet_c = classify_payload_r`
    // (full cast); the optimization branch replaced that with a
    // compact-only synthesis that overwrote `kind`/`cmd_kind` and
    // broke every host-issued MSG_WRITE / CMD_ROUTE / CMD_WEIGHT /
    // CMD_DEBUG / MSG_PROG_*.  Reverted to the full cast so
    // downstream consumers (route/weight write_en, ucode programs,
    // CSR writes, host-response classifier) see actual packet
    // fields.  The spike-enqueue path keeps using
    // `current_packet_min_c.payload.spike.*` for the queue_spike
    // payload — that's still where the compact target/weight/time
    // live for tile-to-tile MSG_SPIKE traffic.
    assign ingress_packet_min_c = ingress_payload_r[MIN_PACKET_W-1:0];
    assign current_packet_min_c = classify_payload_r[MIN_PACKET_W-1:0];

    always_comb begin
        current_packet_c = message_packet_t'(classify_payload_r);
        current_min_core_id_c = tile_spike_flat_core_id(current_packet_min_c.payload);
    end

    assign current_packet = current_packet_c;
    assign current_packet_valid = classify_valid_r && ena;
    assign current_local_tile_match = classify_local_tile_match_r;
    assign current_core_broadcast = classify_core_broadcast_r;
    assign current_has_target = classify_has_target_r;
    assign selected_target_idx = classify_target_idx_r;
    assign packet_from_loader = 1'b0;

    assign classify_load_c =
        ena &&
        ingress_pending_r &&
        (!classify_valid_r || ingress_consume);

    assign ingress_accept_c = rv_in_valid && rv_in_ready;

    // ---------------------------------------------------------------------
    // Classify combinational (tile match / core broadcast / has_target)
    //
    // Restored to use the full `message_packet_t` (`broadcast`, `dst_x`,
    // `dst_y`, `core_id`) instead of the compact-stream view.  The
    // optimization-branch refactor swapped this for a payload_is_spike-
    // only classify path, which broke every host-issued packet (host
    // sends full message_packet_t — broadcast/dst/core_id are the
    // authoritative fields, the bottom-25 compact view is unrelated to
    // these for non-tile-to-tile traffic).
    // ---------------------------------------------------------------------
    message_packet_t ingress_packet_c;
    assign ingress_packet_c = message_packet_t'(ingress_payload_r);

    always_comb begin
        ingress_classify_target_idx_c       = '0;
        ingress_classify_local_tile_match_c =
            ingress_packet_c.broadcast ||
            ((TILE_COORD_W'(ingress_packet_c.dst_x) == TILE_COORD_W'(tile_x_coord)) &&
             (TILE_COORD_W'(ingress_packet_c.dst_y) == TILE_COORD_W'(tile_y_coord)));
        ingress_classify_core_broadcast_c =
            (ingress_packet_c.core_id == MESSAGE_CORE_BCAST);
        ingress_classify_has_target_c       = ingress_classify_core_broadcast_c;
        ingress_spike_flat_idx_c =
            tile_spike_flat_idx(ingress_packet_min_c.payload);
        if (!ingress_classify_core_broadcast_c) begin
            // Map the full packet's `core_id` (8-bit) to a tile-local
            // neuron index via the standard helper, accounting for
            // LOCAL_Z (this tile's neuron-Z offset within the chip).
            ingress_classify_has_target_c =
                local_target_valid_for_core_id(
                    ingress_packet_c.core_id,
                    ingress_classify_target_idx_c
                );
        end
    end

    // ---------------------------------------------------------------------
    // Spike-enqueue-ready / target calculation
    // Scalar spike back-pressure: tile_top maps the target neuron to a worker
    // FIFO and returns the selected FIFO's full flag.
    // For broadcast, tile_top owns the per-neuron counter; ingress just
    // signals `ingress_spike_is_bcast` and lets top serialize.
    // ---------------------------------------------------------------------
    logic ingress_spike_enqueue_ready_calc_c;
    always_comb begin
        ingress_spike_enqueue_ready_calc_c = 1'b1;
        if (current_core_broadcast) begin
            ingress_spike_is_bcast    = 1'b1;
            ingress_spike_target_idx  = '0;       // tile_top counter starts from 0
            if (logical_spike_full) begin
                ingress_spike_enqueue_ready_calc_c = 1'b0;
            end
        end else begin
            ingress_spike_is_bcast    = 1'b0;
            ingress_spike_target_idx  = selected_target_idx;
            if (current_has_target && logical_spike_full) begin
                ingress_spike_enqueue_ready_calc_c = 1'b0;
            end
        end

        // Block spike enqueue if the per-neuron config hasn't been
        // loaded yet — same intent as before, but keyed off kind so it
        // catches host-originated MSG_INPUT (where compact
        // `payload_is_spike` is unset).
        if (((current_packet_c.kind == MSG_INPUT) ||
             (current_packet_c.kind == MSG_SPIKE)) && !min_cfg_loaded_r) begin
            ingress_spike_enqueue_ready_calc_c = 1'b0;
        end

        ingress_spike_enqueue_ready = ingress_spike_enqueue_ready_calc_c;
    end

    // Spike-enqueue fires on full-packet kind alone — the redundant
    // `payload_is_spike` AND was a compact-stream artefact that
    // suppressed every host-originated MSG_INPUT.
    // (data_hi removed from message_packet_t in neutern lean profile.)
    assign ingress_spike_enqueue_fire =
        current_packet_valid &&
        current_packet_consume &&
        current_local_tile_match &&
        current_has_target &&
        ((current_packet_c.kind == MSG_INPUT) || (current_packet_c.kind == MSG_SPIKE));

    // ---------------------------------------------------------------------
    // Graph-state-clear pulse + fanout metadata write indicators
    // ---------------------------------------------------------------------
    logic packet_fanout_meta_ptr_write_c;
    logic packet_fanout_meta_len_write_c;
    logic min_packet_graph_state_clear_c;
    logic [FANOUT_ADDR_W-1:0] packet_absolute_fanout_index_c;

    // Graph-clear pulse fires on the gateway's clear-broadcaster packet
    // (MSG_WRITE + CMD_DEBUG + addr=0xF + broadcast).  The compact-stream
    // path used `payload.cfg.neuron_op[0]` here, which doesn't correspond
    // to anything for full host packets — restored to the full-packet
    // discriminant.
    assign min_packet_graph_state_clear_c =
        current_packet_valid &&
        current_packet_consume &&
        current_local_tile_match &&
        (current_packet_c.kind     == MSG_WRITE) &&
        (current_packet_c.cmd_kind == CMD_DEBUG) &&
        (current_packet_c.addr     == 4'hF);

    assign packet_graph_state_clear = min_packet_graph_state_clear_c;
    assign packet_fanout_meta_ptr_write_c =
        current_packet_valid &&
        current_local_tile_match &&
        current_has_target &&
        (current_packet_c.kind == MSG_WRITE) &&
        (current_packet_c.cmd_kind == CMD_DEBUG) &&
        (current_packet_c.addr == 4'hD);
    assign packet_fanout_meta_len_write_c =
        current_packet_valid &&
        current_local_tile_match &&
        current_has_target &&
        (current_packet_c.kind == MSG_WRITE) &&
        (current_packet_c.cmd_kind == CMD_DEBUG) &&
        (current_packet_c.addr == 4'hD);
    assign packet_absolute_fanout_index_c =
        FANOUT_ADDR_W'({current_packet_c.addr, current_packet_c.prog_index});
    assign tile_graph_state_clear = packet_graph_state_clear;

    // ---------------------------------------------------------------------
    // Bank-write mux
    // ---------------------------------------------------------------------
    always_comb begin
        ctrl_write_en_mux =
            current_packet_valid &&
            current_local_tile_match &&
            current_has_target &&
            (current_packet_c.kind == MSG_WRITE) &&
            (current_packet_c.cmd_kind == CMD_CSR);
        ctrl_write_neuron_idx_mux = selected_target_idx;
        ctrl_write_broadcast_mux  = current_core_broadcast;
        ctrl_write_addr_mux = current_packet_c.addr;
        ctrl_write_data_mux = current_packet_c.data;
        ucode_write_en_mux =
            (current_packet_valid &&
             current_local_tile_match &&
             current_has_target &&
             (current_packet_c.kind == MSG_PROG_WORD) &&
             (current_packet_c.cmd_kind == CMD_UCODE));
        ucode_write_neuron_idx_mux = selected_target_idx;
        ucode_write_broadcast_mux  = current_core_broadcast;
        ucode_write_index_mux = current_packet_c.prog_index;
        ucode_write_word_mux = {current_packet_c.weight, current_packet_c.addr, current_packet_c.data};
        ucode_prog_en_mux =
            ucode_prog_end_en ||
            (current_packet_valid &&
             current_local_tile_match &&
             current_has_target &&
             (current_packet_c.kind == MSG_PROG_END) &&
             (current_packet_c.cmd_kind == CMD_UCODE));
        ucode_prog_neuron_idx_mux = selected_target_idx;
        ucode_prog_broadcast_mux  = current_core_broadcast;
        ucode_prog_ptr_mux = current_packet_c.prog_index[4:0];
        ucode_prog_len_mux = {1'b0, current_packet_c.data};  // DATA_W=4, zero-extend to 5
        fanout_prog_ptr_en_mux = packet_fanout_meta_ptr_write_c;
        fanout_prog_len_en_mux = packet_fanout_meta_len_write_c;
        fanout_prog_neuron_idx_mux = selected_target_idx;
        fanout_prog_broadcast_mux  = current_core_broadcast;
        fanout_prog_ptr_mux = FANOUT_ADDR_W'({
            current_packet_c.sid,
            current_packet_c.tag,
            current_packet_c.weight
        });
        fanout_prog_len_mux = FANOUT_ADDR_W'(current_packet_c.data);
        fanout_write_en_mux = 1'b0;
        fanout_write_mask_valid_mux =
            current_core_broadcast || current_has_target;
        fanout_write_index_mux =
            state_target_fanout_ptr +
            FANOUT_ADDR_W'(current_packet_c.prog_index);
        fanout_write_weight_mux = WEIGHT_W'(signed'(current_packet_c.data));
        fanout_write_dst_x_mux = TILE_COORD_W'(current_packet_c.data);
        fanout_write_dst_y_mux = TILE_COORD_W'(current_packet_c.addr);
        fanout_write_core_id_mux = FANOUT_CORE_ID_W'({current_packet_c.tag, current_packet_c.meta[5:0]});
        fanout_write_meta_mux = 2'b01;  // route installs always valid
        fanout_write_valid_mux = 1'b1;                    // route installs always valid
        fanout_write_weight_only_mux = 1'b0;
        fanout_write_route_only_mux = 1'b0;

        if (weight_write_en) begin
            fanout_write_en_mux = 1'b1;
            fanout_write_index_mux = packet_absolute_fanout_index_c;
            fanout_write_weight_only_mux = 1'b1;
        end

        if (route_write_en) begin
            fanout_write_en_mux = 1'b1;
            fanout_write_index_mux = packet_absolute_fanout_index_c;
            fanout_write_route_only_mux = 1'b1;
        end
    end

    // Kept identical to top-level: ucode_write_en_c (the extra write-en
    // source coming from CMD_UCODE write packets) is logically folded into
    // ucode_write_en_mux above; top-level no longer needs to OR.  The
    // `ucode_write_en_c` name used to appear as an OR term but was always
    // the same expression as the branch in ucode_write_en_mux_c, so this
    // is semantically identical.

    // ---------------------------------------------------------------------
    // rv_in_ready
    // ---------------------------------------------------------------------
    always_comb begin
        rv_in_ready =
            ena && (
                !ingress_pending_r ||
                classify_load_c ||
                (HAS_INGRESS_Q1 && !ingress_q1_valid_r) ||
                (HAS_INGRESS_Q2 && !ingress_q2_valid_r) ||
                (HAS_INGRESS_Q3 && !ingress_q3_valid_r)
            );
    end

    // ---------------------------------------------------------------------
    // Current host-response-payload construction (valid + payload; consumed
    // by host_io register stage).
    // ---------------------------------------------------------------------
    always_comb begin
        message_packet_t packet_tmp;
        packet_tmp = '0;
        current_host_response_valid = 1'b0;
        current_host_response_payload = '0;

        if (current_packet_valid && current_packet_consume && !packet_from_loader) begin
            if (current_packet_c.kind == MSG_PING) begin
                current_host_response_valid = 1'b1;
                current_host_response_payload = make_tile_pong_packet(current_packet_c);
            end else if (!current_has_target &&
                         current_local_tile_match &&
                         ((current_packet_c.kind == MSG_READ) ||
                          (current_packet_c.kind == MSG_WRITE) ||
                          (current_packet_c.kind == MSG_PROG_WORD) ||
                          (current_packet_c.kind == MSG_PROG_BEGIN) ||
                          (current_packet_c.kind == MSG_PROG_END)) &&
                         !((current_packet_c.kind == MSG_WRITE) &&
                           (current_packet_c.cmd_kind == CMD_DEBUG) &&
                           (current_packet_c.addr == 4'hF))) begin
                current_host_response_valid = 1'b1;
                current_host_response_payload =
                    make_status_packet(current_packet_c, MSG_STATUS_BAD_CORE);
            end else if ((current_packet_c.kind == MSG_READ) &&
                         current_local_tile_match &&
                         !current_core_broadcast &&
                         current_has_target) begin
                current_host_response_valid =
                    (current_packet_c.cmd_kind != CMD_UCODE) &&
                    (current_packet_c.cmd_kind != CMD_WEIGHT) &&
                    (current_packet_c.cmd_kind != CMD_SYNAPSE) &&
                    (current_packet_c.cmd_kind != CMD_DUMP) &&
                                        !((current_packet_c.cmd_kind == CMD_ROUTE) &&
                                            (current_packet_c.prog_index < ROUTE_PROG_IDX_MAX));
                current_host_response_payload = make_local_read_response_packet(
                    current_packet_c,
                    calc_out_packets,
                    calc_in_packets,
                    state_target_csr_read_data
                );
            end else if ((current_packet_c.kind == MSG_WRITE) &&
                         ((current_packet_c.cmd_kind == CMD_CSR) ||
                          (current_packet_c.cmd_kind == CMD_WEIGHT) ||
                          (current_packet_c.cmd_kind == CMD_ROUTE) ||
                          ((current_packet_c.cmd_kind == CMD_DEBUG) &&
                           (current_packet_c.addr == 4'hD)) ||
                          ((current_packet_c.cmd_kind == CMD_DEBUG) &&
                           (current_packet_c.addr == 4'hF))) &&
                         current_local_tile_match &&
                         (current_has_target ||
                          ((current_packet_c.cmd_kind == CMD_DEBUG) &&
                           (current_packet_c.addr == 4'hF)))) begin
                // src_x / src_y removed: single-tile neutern — no loader detection.
                // Loader-originated writes are not expected on this TT interface.
                if (1'b0) begin  // loader sentinel never fires without src_x
                    current_host_response_valid = 1'b0;
                end else begin
                    packet_tmp = make_status_packet(current_packet_c, MSG_STATUS_OK);
                    current_host_response_valid = 1'b1;
                    current_host_response_payload = packet_tmp;
                end
            end else if (((current_packet_c.kind == MSG_PROG_WORD) ||
                          (current_packet_c.kind == MSG_PROG_END)) &&
                         (current_packet_c.cmd_kind == CMD_UCODE) &&
                         current_local_tile_match &&
                         current_has_target) begin
                // Same: no loader suppression in neutern single-tile profile.
                if (1'b0) begin
                    current_host_response_valid = 1'b0;
                end else begin
                    packet_tmp = make_status_packet(current_packet_c, MSG_STATUS_OK);
                    current_host_response_valid = 1'b1;
                    current_host_response_payload = packet_tmp;
                end
            end else if (((current_packet_c.kind == MSG_WRITE) ||
                          (current_packet_c.kind == MSG_PROG_WORD) ||
                          (current_packet_c.kind == MSG_PROG_BEGIN) ||
                          (current_packet_c.kind == MSG_PROG_END)) &&
                         current_local_tile_match &&
                         current_has_target) begin
                packet_tmp = make_status_packet(current_packet_c, MSG_STATUS_BAD_KIND);
                current_host_response_valid = 1'b1;
                current_host_response_payload = packet_tmp;
            end
        end
    end

    // ---------------------------------------------------------------------
    // 4-stage shift queue next-state
    // ---------------------------------------------------------------------
    always_comb begin
        next_ingress_pending_c = ingress_pending_r;
        next_ingress_payload_c = ingress_payload_r;
        next_ingress_q1_valid_c = ingress_q1_valid_r;
        next_ingress_q2_valid_c = ingress_q2_valid_r;
        next_ingress_q3_valid_c = ingress_q3_valid_r;
        next_ingress_q1_payload_c = ingress_q1_payload_r;
        next_ingress_q2_payload_c = ingress_q2_payload_r;
        next_ingress_q3_payload_c = ingress_q3_payload_r;

        if (classify_load_c) begin
            if (HAS_INGRESS_Q1 && next_ingress_q1_valid_c) begin
                next_ingress_pending_c = 1'b1;
                next_ingress_payload_c = next_ingress_q1_payload_c;
                if (HAS_INGRESS_Q2) begin
                    next_ingress_q1_valid_c = next_ingress_q2_valid_c;
                    next_ingress_q1_payload_c = next_ingress_q2_payload_c;
                    if (HAS_INGRESS_Q3) begin
                        next_ingress_q2_valid_c = next_ingress_q3_valid_c;
                        next_ingress_q2_payload_c = next_ingress_q3_payload_c;
                        next_ingress_q3_valid_c = 1'b0;
                        next_ingress_q3_payload_c = '0;
                    end else begin
                        next_ingress_q2_valid_c = 1'b0;
                        next_ingress_q2_payload_c = '0;
                    end
                end else begin
                    next_ingress_q1_valid_c = 1'b0;
                    next_ingress_q1_payload_c = '0;
                end
            end else begin
                next_ingress_pending_c = 1'b0;
                next_ingress_payload_c = '0;
            end
        end

        if (ingress_accept_c) begin
            if (!next_ingress_pending_c) begin
                next_ingress_pending_c = 1'b1;
                next_ingress_payload_c = rv_in_payload;
            end else if (HAS_INGRESS_Q1 && !next_ingress_q1_valid_c) begin
                next_ingress_q1_valid_c = 1'b1;
                next_ingress_q1_payload_c = rv_in_payload;
            end else if (HAS_INGRESS_Q2 && !next_ingress_q2_valid_c) begin
                next_ingress_q2_valid_c = 1'b1;
                next_ingress_q2_payload_c = rv_in_payload;
            end else if (HAS_INGRESS_Q3 && !next_ingress_q3_valid_c) begin
                next_ingress_q3_valid_c = 1'b1;
                next_ingress_q3_payload_c = rv_in_payload;
            end
        end
    end

    // ---------------------------------------------------------------------
    // Packet consume decoder (sets current_packet_consume, host_reserve,
    // per-bank read-issue triggers, weight/route write enables).
    // ---------------------------------------------------------------------
    always_comb begin
        current_packet_consume = 1'b0;
        ingress_consume = 1'b0;
        host_reserve_current = 1'b0;
        ucode_host_read_issue = 1'b0;
        weight_host_read_issue = 1'b0;
        route_host_read_issue = 1'b0;
        dump_neuron_issue = 1'b0;
        weight_write_en = 1'b0;
        route_write_en = 1'b0;

        // Consume decoder dispatches on kind:
        //  - Spike-bearing kinds (MSG_INPUT / MSG_SPIKE) need the
        //    event-queue ready signal before consume can fire.
        //  - All other kinds (MSG_WRITE / MSG_PROG_* / MSG_READ /
        //    MSG_PING / MSG_MCAST etc.) consume on local match — the
        //    downstream bank-write / host-response logic decides
        //    what to do with them.
        if (current_packet_valid && current_local_tile_match) begin
            if ((current_packet_c.kind != MSG_INPUT) &&
                (current_packet_c.kind != MSG_SPIKE)) begin
                current_packet_consume = 1'b1;
            end else if (!min_cfg_loaded_r) begin
                // Drop spikes until a header/config packet arms the stream.
                current_packet_consume = 1'b1;
            end else if (current_has_target && ingress_spike_enqueue_ready) begin
                current_packet_consume = 1'b1;
            end else if (!current_has_target) begin
                current_packet_consume = 1'b1;
            end
        end

        // Drive the fanout-bank write enables on the matching consume.
        // This was lost in the optimization-branch refactor (the two
        // signals were left wired to '0 unconditionally), which silently
        // dropped every CMD_ROUTE / CMD_WEIGHT write the loader emitted
        // and prevented the route table from ever being populated.
        // Restored from the pre-refactor decoder; gating identical to
        // the consume rules above (valid + local + has_target).
        weight_write_en =
            current_packet_valid &&
            current_packet_consume &&
            current_local_tile_match &&
            current_has_target &&
            (current_packet_c.kind == MSG_WRITE) &&
            (current_packet_c.cmd_kind == CMD_WEIGHT);
        route_write_en =
            current_packet_valid &&
            current_packet_consume &&
            current_local_tile_match &&
            current_has_target &&
            (current_packet_c.kind == MSG_WRITE) &&
            (current_packet_c.cmd_kind == CMD_ROUTE);

        ingress_consume = current_packet_consume;
    end

    // Downstream back-pressure inputs kept in the port contract for symmetry
    // with the decomposed host path. Compact stream consume currently drains
    // without host-slot reservation, so these are intentional no-ops here.
    wire unused_downstream_backpressure = &{host_slot_free_r, dump_active_r};

    // ---------------------------------------------------------------------
    // Register update
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (!rst_n || tile_graph_state_clear) begin
            ingress_pending_r <= 1'b0;
            ingress_payload_r <= '0;
            ingress_q1_valid_r <= 1'b0;
            ingress_q2_valid_r <= 1'b0;
            ingress_q3_valid_r <= 1'b0;
            ingress_q1_payload_r <= '0;
            ingress_q2_payload_r <= '0;
            ingress_q3_payload_r <= '0;
            classify_valid_r <= 1'b0;
            classify_local_tile_match_r <= 1'b0;
            classify_core_broadcast_r <= 1'b0;
            classify_has_target_r <= 1'b0;
            classify_target_idx_r <= '0;
            classify_payload_r <= '0;
            min_cfg_loaded_r <= 1'b0;
        end else if (ena) begin
            if (classify_load_c) begin
                classify_valid_r <= 1'b1;
                classify_local_tile_match_r <= ingress_classify_local_tile_match_c;
                classify_core_broadcast_r <= ingress_classify_core_broadcast_c;
                classify_has_target_r <= ingress_classify_has_target_c;
                classify_target_idx_r <= ingress_classify_target_idx_c;
                classify_payload_r <= ingress_payload_r;
            end else if (ingress_consume) begin
                classify_valid_r <= 1'b0;
                classify_local_tile_match_r <= 1'b0;
                classify_core_broadcast_r <= 1'b0;
                classify_has_target_r <= 1'b0;
                classify_target_idx_r <= '0;
                classify_payload_r <= '0;
            end

            // `min_cfg_loaded_r` arms once any config-bearing packet
            // (anything that isn't MSG_INPUT / MSG_SPIKE) lands locally
            // — keys off kind, not the compact `payload_is_spike` bit
            // (which doesn't apply to host-originated full packets).
            if (current_packet_valid &&
                current_packet_consume &&
                current_local_tile_match &&
                (current_packet_c.kind != MSG_INPUT) &&
                (current_packet_c.kind != MSG_SPIKE)) begin
                min_cfg_loaded_r <= 1'b1;
            end

            ingress_pending_r <= next_ingress_pending_c;
            ingress_payload_r <= next_ingress_payload_c;
            ingress_q1_valid_r <= next_ingress_q1_valid_c;
            ingress_q2_valid_r <= next_ingress_q2_valid_c;
            ingress_q3_valid_r <= next_ingress_q3_valid_c;
            ingress_q1_payload_r <= next_ingress_q1_payload_c;
            ingress_q2_payload_r <= next_ingress_q2_payload_c;
            ingress_q3_payload_r <= next_ingress_q3_payload_c;
        end
    end

    // ── fanout_write rv_if output packing ─────────────────────────────────────
    // Pack internal mux results into fanout_write_req_t struct and drive rv_if.
    fanout_write_req_t fw_payload_c;
    assign fw_payload_c.mask_valid  = fanout_write_mask_valid_mux;
    assign fw_payload_c.weight      = fanout_write_weight_mux;
    assign fw_payload_c.dst_x       = fanout_write_dst_x_mux;
    assign fw_payload_c.dst_y       = fanout_write_dst_y_mux;
    assign fw_payload_c.core_id     = fanout_write_core_id_mux;
    assign fw_payload_c.meta        = fanout_write_meta_mux;
    assign fw_payload_c.valid       = fanout_write_valid_mux;
    assign fw_payload_c.weight_only = fanout_write_weight_only_mux;
    assign fw_payload_c.route_only  = fanout_write_route_only_mux;

    assign fanout_write.valid      = fanout_write_en_mux;
    assign fanout_write.rv_payload = fw_payload_c;
    // fanout_write.ready driven by tile_fanout_pool (always 1, no back-pressure)

    assign fanout_write_addr = fanout_write_index_mux;

endmodule

`default_nettype wire
