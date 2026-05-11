`default_nettype none

(* keep_hierarchy = "no" *)
`ifndef YOSYS
/* verilator lint_off IMPORTSTAR */
import tile_pkg::*;
/* verilator lint_on IMPORTSTAR */
`endif
module tile_top
  #(
    parameter int unsigned TILE_COORD_X = 0,
    parameter int unsigned TILE_COORD_Y = 0,
    parameter int unsigned LOCAL_Z = 0,
    parameter int unsigned NEURONS_PER_TILE = 1,
    parameter int unsigned WORKER_CORES_PER_TILE = NEURONS_PER_TILE,
    parameter int unsigned FANOUT_POOL_DEPTH = 256,
    // Fanout is fixed to one shared read lane (1r1w register-file behavior).
    // Dual-read fanout mode is removed to avoid accidental area blowups
    // for deep fanout pools.
    // Shared tile-bank memory style hint:
    // 0=auto, 1=distributed, 2=block, 3=macro-intent (ASIC path).
    parameter int unsigned TILE_BANK_MEM_STYLE = 2,
    // Shared logical-event pool sizing hint.  The event queue bank uses
    // one global pool sized as WORKER_CORES_PER_TILE * LOGICAL_EVENT_QUEUE_DEPTH.
    parameter int unsigned LOGICAL_EVENT_QUEUE_DEPTH = 4,
    parameter int unsigned INGRESS_QUEUE_DEPTH = 4,
    // Fanout route-lane stored widths.  Defaults to 8 b per field to
    // match the protocol; GF180/SoC wrappers narrow these from the
    // top-level mesh/worker parameters (e.g. 2/2/2 for up to a 4x4 mesh
    // with 4 workers) so the fanout pool stores only sparse outbound
    // route bits.  External ports stay 8 b and upper bits are zero-filled
    // on read.
    parameter int unsigned FANOUT_DST_X_W   = 8,
    parameter int unsigned FANOUT_DST_Y_W   = 8,
    parameter int unsigned FANOUT_CORE_ID_W = 8,
    // Host output FIFO depth.
    parameter int unsigned HOST_OUTPUT_FIFO_DEPTH = 4,
    // Fanout queue depth (in-flight spike-emission slots between commit and NOC egress).
    parameter int unsigned FANOUT_QUEUE_DEPTH = 4,
    parameter int unsigned MESSAGE_W = 53
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  ena,
    rv_if                        rv_in,
    rv_if                        rv_out
`ifdef FORMAL
    ,
    output wire                  rv_in_ready_obs,
    output wire                  rv_out_valid_obs,
    output wire [MESSAGE_W-1:0]  rv_out_payload_obs,
    output wire                  rv_host_out_valid_obs,
    output wire [MESSAGE_W-1:0]  rv_host_out_payload_obs,
    output wire                  rv_noc_out_valid_obs,
    output wire [MESSAGE_W-1:0]  rv_noc_out_payload_obs,
    output wire                  rv_host_out_ready_obs,
    output wire                  rv_noc_out_ready_obs,
    output wire                  tile_graph_state_clear_r_obs
`endif
);
    // Tile coordinates used internally by sub-module instances.  Previously
    // plumbed in as ports from tile_top_packed; now derived directly from
    // the TILE_COORD_X / TILE_COORD_Y parameters after the wrapper collapse.
    wire [TILE_COORD_W-1:0] tile_x_coord = TILE_COORD_W'(TILE_COORD_X);
    wire [TILE_COORD_W-1:0] tile_y_coord = TILE_COORD_W'(TILE_COORD_Y);

    // Internal ready/valid channels.
    rv_if #(.WIDTH(MESSAGE_W)) rv_host_out_if();
    rv_if #(.WIDTH(MESSAGE_W)) rv_noc_out_if();
    rv_if #(.WIDTH(MESSAGE_W)) noc_child_if();

    wire                  rv_in_valid         = rv_in.valid;
    wire [MESSAGE_W-1:0]  rv_in_payload       = rv_in.rv_payload;
    wire                  rv_in_ready         = rv_in.ready;

    // Split host / NOC egress busses are produced by the body logic below
    // and muxed onto the single rv_out channel.  host wins on conflict so
    // host responses cannot be starved by locally-sourced NOC traffic.
    //
    // The mux passes the FULL message_packet_t through.  Pre-fix this
    // truncated each branch to message_packet_min_t (25 bits zero-
    // extended to MESSAGE_W), discarding the top kind/cmd_kind/dst/
    // src/core_id/prog_index/addr fields.  Every fanout-emitted spike
    // therefore left tile X with kind=0, cmd=0, dst=(0,0), src=(0,0) —
    // which routed straight back to tile (0,0) and produced the
    // multi_edge_return_addressing self-loop (acc count climbing
    // unboundedly while every emitted packet had the same all-zero
    // top half).
    logic                 rv_noc_out_ready;
    wire                  rv_noc_out_valid    = rv_noc_out_if.valid;
    wire [MESSAGE_W-1:0]  rv_noc_out_payload  = rv_noc_out_if.rv_payload;
    logic                 rv_host_out_ready;
    wire                  rv_host_out_valid   = rv_host_out_if.valid;
    wire [MESSAGE_W-1:0]  rv_host_out_payload = rv_host_out_if.rv_payload;

    assign rv_out.valid                       = rv_host_out_valid || rv_noc_out_valid;
    assign rv_out.rv_payload =
        rv_host_out_valid ? rv_host_out_payload : rv_noc_out_payload;
    assign rv_host_out_ready                  = rv_out.ready;
    assign rv_noc_out_ready                   = rv_out.ready && !rv_host_out_valid;
    assign rv_host_out_if.ready               = rv_host_out_ready;
    assign rv_noc_out_if.ready                = rv_noc_out_ready;

`ifdef FORMAL
    assign rv_in_ready_obs         = rv_in_ready;
    assign rv_out_valid_obs        = rv_out.valid;
    assign rv_out_payload_obs      = rv_out.rv_payload;
    assign rv_host_out_valid_obs   = rv_host_out_valid;
    assign rv_host_out_payload_obs = rv_host_out_payload;
    assign rv_noc_out_valid_obs    = rv_noc_out_valid;
    assign rv_noc_out_payload_obs  = rv_noc_out_payload;
    assign rv_host_out_ready_obs              = rv_host_out_ready;
    assign rv_noc_out_ready_obs               = rv_noc_out_ready;
`endif

    // Calc scaffolding removed; tie legacy debug readback counters to zero so
    // the existing TILE_DEBUG_ADDR_CALC readback path still compiles cleanly.
    wire [15:0] calc_out_packets = 16'd0;
    wire [15:0] calc_in_packets  = 16'd0;

    // IS_ASIC_LEAN controls compact parameter settings (reduced ucode words,
    // shared bank, zero-init reset).  All memory instances use FF arrays
    // regardless of TILE_BANK_MEM_STYLE.
    localparam bit IS_ASIC_LEAN = (TILE_BANK_MEM_STYLE == 3);

    localparam int unsigned UCODE_WORDS_PER_NEURON_CFG  = IS_ASIC_LEAN ?  8 : 16;
    localparam int unsigned UCODE_SHARED_BANK_CFG       = IS_ASIC_LEAN ?  1 :  0;
    localparam int unsigned ASIC_ZERO_INIT_RESET_CFG    = IS_ASIC_LEAN ?  1 :  0;
    localparam bit CONTEXT_SPLIT_LAYOUT_CFG             = 1'b1;
    localparam bit DUAL_READ_CONTEXT_CFG                = IS_ASIC_LEAN ? 1'b0 : 1'b1;
    // FANOUT section in state_bank row:
    //   LEAN: drop — fanout_ptr already in COMMIT, fanout_len in DISPATCH.
    //   Non-lean: keep for dump readback.
    localparam bit DROP_REDUNDANT_FANOUT_BRAM_CFG       = IS_ASIC_LEAN ? 1'b1 : 1'b0;
    localparam bit STATE_ROW_NO_FANOUT_BYTES_CFG        = IS_ASIC_LEAN ? 1'b1 : 1'b0;
    localparam bit DROP_DISPATCH_BRAM_ON_COMPACT_CFG    = 1'b1;
    // Output spike FIFO depth per worker.  Decoupled from FANOUT_POOL_DEPTH;
    // currently disabled (disconnected from the NoC path).
    // compact 28-bit tile_fanout_spike_t payload padded to four byte lanes.
    // Shared single FIFO, depth=4 (matches NEURONS_PER_TILE).
    localparam int unsigned FANOUT_SPIKE_FIFO_DEPTH_CFG = 4;

    /* verilator lint_off VARHIDDEN */
    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE); /* verilator lint_on VARHIDDEN */
    localparam int unsigned WORKER_IDX_W =
        (WORKER_CORES_PER_TILE <= 1) ? 1 : $clog2(WORKER_CORES_PER_TILE);
    localparam int unsigned SYNAPSE_SID_W = 5;
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    localparam bit HAS_INGRESS_Q1 = (INGRESS_QUEUE_DEPTH > 1);
    localparam bit HAS_INGRESS_Q2 = (INGRESS_QUEUE_DEPTH > 2);
    localparam bit HAS_INGRESS_Q3 = (INGRESS_QUEUE_DEPTH > 3);
    // HOST_OUTPUT_FIFO_DEPTH and FANOUT_QUEUE_DEPTH are now top-level
    // parameters (see parameter block above); only the derived widths
    // remain as localparams.
    localparam int unsigned HOST_OUTPUT_FIFO_PTR_W =
        (HOST_OUTPUT_FIFO_DEPTH <= 1) ? 1 : $clog2(HOST_OUTPUT_FIFO_DEPTH);
    localparam int unsigned HOST_OUTPUT_FIFO_COUNT_W =
        $clog2(HOST_OUTPUT_FIFO_DEPTH + 1);
    localparam int unsigned FANOUT_QUEUE_PTR_W =
        (FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH);
    localparam int unsigned FANOUT_QUEUE_COUNT_W =
        $clog2(FANOUT_QUEUE_DEPTH + 1);
    localparam int unsigned FANOUT_RETRY_BYPASS_W =
        (FANOUT_QUEUE_DEPTH <= 1) ? 1 : $clog2(FANOUT_QUEUE_DEPTH + 1);
    // Ready FIFO depth is capped so it doesn't scale to NEURONS_PER_TILE.
    // The actual FIFO logic lives in tile_dispatch_scheduler which already
    // applies the same cap; these localparams are kept here for documentation
    // consistency.  Steady-state occupancy is bounded by WORKER_CORES_PER_TILE;
    // a depth of 4× with a floor of 8 absorbs transient spike bursts cleanly.
    localparam int unsigned READY_FIFO_DEPTH =
        (WORKER_CORES_PER_TILE * 4 < 8) ? 8 : WORKER_CORES_PER_TILE * 4;
    localparam int unsigned READY_FIFO_PTR_W =
        (READY_FIFO_DEPTH <= 1) ? 1 : $clog2(READY_FIFO_DEPTH);
    localparam int unsigned READY_FIFO_COUNT_W =
        $clog2(READY_FIFO_DEPTH + 1);

    // Classified-packet view exposed by tile_ingress (regs live inside the
    // sub-module; these are just the outputs we wire into the inline logic
    // for dispatch / fanout / host_io and the bank instances).
    message_packet_t         current_packet_c;
    logic                    packet_from_loader_c;
    logic                    current_packet_consume_c;
    logic                    ingress_consume_c;
    logic                    current_packet_valid_c;
    logic                    current_host_response_valid_c;
    message_packet_t         current_host_response_payload_c;
    logic                    noc_tile_valid_unused_c;

    logic [NEURON_IDX_W-1:0] ingress_event_target_idx_c;
    logic                    ingress_event_is_bcast_c;
    // Broadcast spike serialization counter in tile_top: when ingress
    // fires a broadcast event, this counter walks 0..N-1 enqueuing each
    // neuron.  The ingress packet has already been consumed; the counter
    // drives the enqueue path independently.
    logic                    ingress_bcast_in_progress_r;
    logic [NEURON_IDX_W-1:0] ingress_bcast_addr_r;
    logic                    current_local_tile_match_c;
    logic                    current_core_broadcast_c;
    logic                    current_has_target_c;
    logic                    ingress_event_enqueue_ready_c;
    logic                    ingress_event_enqueue_fire_c;
    logic                    tile_graph_state_clear_c;
    // One-cycle-registered view of tile_graph_state_clear_c.  The
    // combinational clear sourced from u_ingress has a fan-out of ~290
    // into workers + scheduler + bank resets; routing that wire from a
    // raw classify-payload decode put a deep LUT cone on the worst
    // synth path.  Registering here turns the clear distribution into
    // a simple flop -> sync-reset fan-out, with a 1-cycle delay to
    // downstream reset points (acceptable because clear is a config
    // event, not a dataflow signal).  The ingress module itself still
    // self-clears on the combinational `_c` — only downstream consumers
    // use `_r`.
    logic                    tile_graph_state_clear_r;
    logic                    packet_graph_state_clear_c;
    logic [NEURON_IDX_W-1:0] selected_target_idx_c;

    // host_pending_*/host_slot_free now live inside u_host_io.  Only the
    // combinational "reserve current packet slot" signal is wired through
    // ingress -> host_io -> dispatch.
    logic                    host_slot_free_r;
    logic                    host_reserve_current_c;

    // DEBUG 2026-04-18: count CSR_CTRL writes that actually fire at this
    // tile, so the runtime test can tell whether the bundle loader's
    // DIRECT_WRITE_CTRL packet ever reached the ctrl_bank write-enable.
    // `csr_ctrl_writes_total_r` bumps on every clock where
    // `ctrl_write_en_mux_c && (ctrl_write_addr_mux_c == CSR_CTRL)` — i.e.
    // the mux driving the logical_neuron ctrl bank would be writing a
    // CSR_CTRL value (any neuron bit set).  Snapshot reads this via
    // `tile.csr_ctrl_writes_total_r.value`.
    //
    // These counters are pure observability — the cocotb test at
    // `hw/SoC/test/test_coldfoot_runtime.py` already guards the reads
    // with `hasattr(tile, "csr_ctrl_writes_total_r")`, so defining
    // `TILE_ASIC_LEAN` on the ASIC synthesis path removes them
    // (≈290 flops + 17 × 16-bit adders / tile) without breaking sim.


    // host_output_fifo / rv_host_out registers now live inside u_host_io.

    // Driven by u_fanout_executor; consumed by u_noc_egress.child.ready.
    wire                 noc_child_valid_c   = noc_child_if.valid;
    wire [MESSAGE_W-1:0] noc_child_payload_c = noc_child_if.rv_payload;
    wire                 noc_child_ready_c   = noc_child_if.ready;
    // Exported from u_fanout_executor purely for the top-level fanout-fire
    // debug counter.
    wire                 fanout_fire_c;
    // soft_reset_pulse_c: legacy tie-to-zero for any remaining consumers.
    // Real soft-reset is now scalar (soft_reset_valid_c, soft_reset_idx_c)
    // from logical_neuron_state_bank via u_dispatch_scheduler.
    logic [NEURONS_PER_TILE-1:0] soft_reset_pulse_c; // driven by u_state_bank (always 0 in scalar refactor)
    logic [NEURONS_PER_TILE-1:0] host_egress_en_bus_c;
    logic [NEURONS_PER_TILE-1:0] noc_egress_en_bus_c;
    // Single-bit mux for the scheduler: selects the current-dispatch neuron's
    // noc_egress_en bit.  Variable-select of a bus cannot be done inline in a
    // port connection, so we wire it here.
    wire                         start_dispatch_valid_r;
    wire [WORKER_IDX_W-1:0]      start_dispatch_worker_idx_r;
    wire [NEURON_IDX_W-1:0]      start_dispatch_logical_idx_r;
    wire                         noc_egress_en_c;
    assign noc_egress_en_c = noc_egress_en_bus_c[start_dispatch_logical_idx_r];
    logic [7:0] state_target_csr_read_data_c;
    logic [FANOUT_ADDR_W-1:0] state_target_fanout_ptr_c;
    logic [FANOUT_ADDR_W-1:0] state_target_fanout_len_c;
    logic [4:0] state_dispatch_ucode_ptr_c;
    logic [4:0] state_dispatch_ucode_len_c;
    logic [FANOUT_ADDR_W-1:0] state_dispatch_fanout_len_c;
    logic [FANOUT_ADDR_W-1:0] state_fanout_ptr_c;
    logic [FANOUT_ADDR_W-1:0] state_fanout_len_c;
    logic [FANOUT_ADDR_W-1:0] state_commit_fanout_ptr_c;
    logic [23:0] state_dump_init_rf_flat_c;
    logic [4:0] state_dump_ucode_ptr_c;
    logic [4:0] state_dump_ucode_len_c;
    logic [7:0] state_dump_neuron_flags_c;
    logic [FANOUT_ADDR_W-1:0] state_dump_fanout_ptr_c;
    logic [FANOUT_ADDR_W-1:0] state_dump_fanout_len_c;
    logic [23:0] state_reset_init_rf_flat_c;
    // state_reset_last_event_time_c removed: timestamps eliminated
    // state_bank → scheduler (combinational soft-reset output from state_bank)
    wire  soft_reset_valid_sbank_c;
    wire  [NEURON_IDX_W-1:0] soft_reset_idx_sbank_c;
    // scheduler → context_bank (registered 1-cycle pipeline output)
    wire  soft_reset_valid_c;
    wire  [NEURON_IDX_W-1:0] soft_reset_idx_c;
    wire  [NEURON_IDX_W-1:0] state_dispatch_read_idx_c;

    logic [23:0] ctx_read_rf_state_flat_a_c;
    logic [TAG_W-1:0]          ctx_read_last_tag_a_c;
    // ctx_read_last_time_a_c removed: timestamps eliminated
    logic ctx_read_cmp_ge_a_c;
    logic ctx_read_cmp_eq_a_c;
    logic ctx_read_spike_flag_a_c;
    logic [23:0] ctx_read_rf_state_flat_b_c;
    logic [TAG_W-1:0]          ctx_read_last_tag_b_c;
    // ctx_read_last_time_b_c removed: timestamps eliminated
    logic ctx_read_cmp_ge_b_c;
    logic ctx_read_cmp_eq_b_c;
    logic ctx_read_spike_flag_b_c;
    // context-bank last_tag ports now match TAG_W directly.

    // Packing wires: assemble struct views of context-bank read port A and
    // the scheduler's event fields for the compute core connections.
    neuron_exec_ctx_t ctx_read_a_ctx_c;
    assign ctx_read_a_ctx_c.rf_state_flat = ctx_read_rf_state_flat_a_c;
    assign ctx_read_a_ctx_c.last_tag      = ctx_read_last_tag_a_c;
    // ctx_read_a_ctx_c.last_time removed
    assign ctx_read_a_ctx_c.cmp_ge        = ctx_read_cmp_ge_a_c;
    assign ctx_read_a_ctx_c.cmp_eq        = ctx_read_cmp_eq_a_c;
    assign ctx_read_a_ctx_c.spike_flag    = ctx_read_spike_flag_a_c;

    wire  [TAG_W-1:0]             worker_start_event_tag_c;
    wire  [EVENT_TIME_W-1:0]      worker_start_event_time_c;
    wire  signed [WEIGHT_W-1:0]   worker_start_event_weight_c;
    neuron_worker_event_t worker_start_event_c;
    assign worker_start_event_c.tag    = worker_start_event_tag_c;
    // spike_time removed from neuron_worker_event_t (neutern has no timestamps)
    assign worker_start_event_c.weight = worker_start_event_weight_c;

    logic ucode_prog_end_en_c;
    logic ucode_host_read_issue_c;
    logic route_host_read_issue_c;
    logic weight_host_read_issue_c;
    logic dump_neuron_issue_c;
    // Dump state (active, target idx) comes out of u_host_io and is used
    // only for debug counters + bank-wire naming in this file.
    wire dump_active_r;
    wire [NEURON_IDX_W-1:0] dump_target_idx_r;
    wire ctx_dispatch_allowed_c;
    wire [NEURON_IDX_W-1:0] ctx_read_idx_a_c;
    wire [NEURON_IDX_W-1:0] ctx_read_idx_b_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_ucode_read_valid_c;
    logic [NEURON_IDX_W-1:0] worker_ucode_read_logical_idx_c [0:WORKER_CORES_PER_TILE-1];
    logic [4:0] worker_ucode_read_word_index_c [0:WORKER_CORES_PER_TILE-1];
    logic [WORKER_CORES_PER_TILE-1:0] worker_ucode_read_word_valid_c;
    logic [15:0] worker_ucode_read_word_c [0:WORKER_CORES_PER_TILE-1];
    // rv_if channels for the shared ucode bank single read port.
    rv_if #(.WIDTH(NEURON_UCODE_REQ_W)) ucode_bank_req_if();
    rv_if #(.WIDTH(NEURON_UCODE_RSP_W)) ucode_bank_rsp_if();
    // Port-0 ucode aux request fields now come out of u_host_io.
    wire port0_ucode_aux_req_c;
    wire port0_ucode_aux_is_dump_c;
    wire [4:0] port0_ucode_aux_word_index_c;
    wire [NEURON_IDX_W-1:0] port0_ucode_aux_neuron_idx_c;
    // Registered view of "port-0 aux read is about to land" — gates the
    // worker-0 lane on the core's bank-port-0 mux below.
    wire port0_ucode_host_or_dump_in_flight_c;
    // Ucode bank port-0 mux helper aliases from u_host_io.
    wire [NEURON_IDX_W-1:0] host_io_port0_ucode_aux_neuron_idx_w =
        port0_ucode_aux_neuron_idx_c;
    wire [4:0] host_io_port0_ucode_aux_word_index_w =
        port0_ucode_aux_word_index_c;

    // Fanout bank port-drive wires from u_host_io (port0) and
    // u_fanout_executor (port1).
    wire fanout_bank_port0_read_en_w;
    wire [NEURON_IDX_W-1:0] fanout_bank_port0_neuron_idx_w;
    wire [FANOUT_ADDR_W-1:0] fanout_bank_port0_index_w;
    wire fanout_bank_port1_read_en_w;
    wire [NEURON_IDX_W-1:0] fanout_bank_port1_neuron_idx_w;
    wire [FANOUT_ADDR_W-1:0] fanout_bank_port1_index_w;

    // State bank fanout-read idx from u_fanout_executor.
    wire [NEURON_IDX_W-1:0] fanout_read_idx_w;

    logic weight_write_en_c;
    logic route_write_en_c;
    logic fanout_bank_read_en_c;
    logic [FANOUT_ADDR_W-1:0] fanout_bank_read_index_c;
    logic signed [7:0] fanout_bank_read_weight_c;
    logic fanout_bank_read_meta_c;  // 1-bit: valid+epoch gate from u_fanout_pool
    // start_dispatch_* regs / ready-FIFO / worker RR / logical_event_inflight
    // / worker_has_fanout all moved into u_dispatch_scheduler.  The core
    // observes only the registered views exported by the scheduler.

    assign ctx_dispatch_allowed_c = DUAL_READ_CONTEXT_CFG || !dump_active_r;
    assign ctx_read_idx_a_c = (!DUAL_READ_CONTEXT_CFG && dump_active_r)
        ? dump_target_idx_r
        : start_dispatch_logical_idx_r;
    assign ctx_read_idx_b_c = DUAL_READ_CONTEXT_CFG
        ? dump_target_idx_r
        : ctx_read_idx_a_c;
    // fanout_bank_read_* driven by u_fanout_pool via fanout_rsp_if rv_if (see below).
    // fanout_bank_read_en_c: legacy stub; tied low — pool read goes through shared arbiter.
    wire _unused_fanout_bank_read_en = fanout_bank_read_en_c;

    // ── Fanout write rv_if: tile_ingress → tile_fanout_pool ──────────────────
    rv_if #(.WIDTH(FANOUT_WRITE_REQ_W)) fanout_write_if();
    logic [FANOUT_ADDR_W-1:0]           fanout_write_addr_c;   // companion table address

    // ── Fanout pool read response rv_if: tile_fanout_pool → tile_top demux ───
    // Single response channel from pool; tile_top routes to host or executor.
    rv_if #(.WIDTH(FANOUT_READ_RSP_W)) fanout_pool_rsp_if();   // from pool
    rv_if #(.WIDTH(FANOUT_READ_RSP_W)) fanout_rsp_exec_if();   // to executor
    rv_if #(.WIDTH(FANOUT_READ_RSP_W)) fanout_rsp_host_if();   // to host_io

    // ── Context commit rv_if: tile_dispatch_scheduler → tile_top ─────────────
    rv_if #(.WIDTH(CONTEXT_COMMIT_W))  context_commit_if();

    // Shared fanout single-read arbitration state (Section 8.3).
    logic shared_fanout_req_host_pending_r;
    logic shared_fanout_req_exec_pending_r;
    logic [FANOUT_ADDR_W-1:0] shared_fanout_req_host_index_r;
    logic [FANOUT_ADDR_W-1:0] shared_fanout_req_exec_index_r;
    logic shared_fanout_read_inflight_r;
    logic shared_fanout_read_owner_host_r;
    logic shared_fanout_issue_host_c;
    logic shared_fanout_issue_exec_c;
    logic [FANOUT_ADDR_W-1:0] shared_fanout_issue_index_c;
    logic shared_fanout_resp_valid_c;
    // Fanout-executor state + queue were moved into u_fanout_executor.
    // The core only observes fanout_queue_can_accept_c (back-pressure to
    // the still-inline worker_commit_ready gate).
    logic fanout_queue_can_accept_c;
    // Live fanout_queue occupancy for credit-based dispatch throttling in
    // tile_dispatch_scheduler (MSG_OUTPUT wedge structural fix).
    logic [FANOUT_QUEUE_COUNT_W-1:0] fanout_queue_count_live_c;

    // Ucode read-response interface aliases (bank port-0 -> host_io).
    wire ucode_rsp_valid_port0_c = ucode_bank_rsp_if.valid;
    wire [15:0] ucode_rsp_word_port0_c =
        {4'b0, ucode_bank_rsp_if.rv_payload};  // payload is NEURON_UCODE_RSP_W bits

    // fanout_pool_rsp_if is the single read-response channel from u_fanout_pool.
    // Route to executor or host_io based on who owns the in-flight read.
    assign shared_fanout_resp_valid_c = fanout_pool_rsp_if.valid;

    // fanout_pool read-response rv_if demux: executor has priority path.
    assign fanout_rsp_exec_if.valid      = fanout_pool_rsp_if.valid && !shared_fanout_read_owner_host_r;
    assign fanout_rsp_exec_if.rv_payload = fanout_pool_rsp_if.rv_payload;
    assign fanout_rsp_host_if.valid      = fanout_pool_rsp_if.valid && shared_fanout_read_owner_host_r;
    assign fanout_rsp_host_if.rv_payload = fanout_pool_rsp_if.rv_payload;
    // Consumers always accept responses (no back-pressure on fanout reads).
    assign fanout_pool_rsp_if.ready = 1'b1;
    // fanout_rsp_exec_if.ready and fanout_rsp_host_if.ready driven inside
    // the respective submodules (rv_if.rx contract).

    // Shared fanout read-port arbitration.
    // Executor route-walk requests have priority over host/dump reads.
    integer w;
    always_comb begin
        shared_fanout_issue_host_c   = 1'b0;
        shared_fanout_issue_exec_c   = 1'b0;
        shared_fanout_issue_index_c  = '0;

        if (!shared_fanout_read_inflight_r) begin
            if (shared_fanout_req_exec_pending_r || fanout_bank_port1_read_en_w) begin
                shared_fanout_issue_exec_c = 1'b1;
                if (shared_fanout_req_exec_pending_r) begin
                    shared_fanout_issue_index_c = shared_fanout_req_exec_index_r;
                end else begin
                    shared_fanout_issue_index_c = fanout_bank_port1_index_w;
                end
            end else if (shared_fanout_req_host_pending_r || fanout_bank_port0_read_en_w) begin
                shared_fanout_issue_host_c = 1'b1;
                if (shared_fanout_req_host_pending_r) begin
                    shared_fanout_issue_index_c = shared_fanout_req_host_index_r;
                end else begin
                    shared_fanout_issue_index_c = fanout_bank_port0_index_w;
                end
            end
        end
    end

    // (shared_fanout_resp_valid_c is assigned from fanout_pool_rsp_if.valid above)

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shared_fanout_req_host_pending_r <= 1'b0;
            shared_fanout_req_exec_pending_r <= 1'b0;
            shared_fanout_req_host_index_r <= '0;
            shared_fanout_req_exec_index_r <= '0;
            shared_fanout_read_inflight_r <= 1'b0;
            shared_fanout_read_owner_host_r <= 1'b0;
        end else if (ena) begin
            if (tile_graph_state_clear_r) begin
                shared_fanout_req_host_pending_r <= 1'b0;
                shared_fanout_req_exec_pending_r <= 1'b0;
                shared_fanout_read_inflight_r <= 1'b0;
                shared_fanout_read_owner_host_r <= 1'b0;
            end else begin
                if (fanout_bank_port0_read_en_w &&
                    !(shared_fanout_issue_host_c && !shared_fanout_req_host_pending_r)) begin
                    shared_fanout_req_host_pending_r <= 1'b1;
                    shared_fanout_req_host_index_r <= fanout_bank_port0_index_w;
                end

                if (fanout_bank_port1_read_en_w &&
                    !(shared_fanout_issue_exec_c && !shared_fanout_req_exec_pending_r)) begin
                    shared_fanout_req_exec_pending_r <= 1'b1;
                    shared_fanout_req_exec_index_r <= fanout_bank_port1_index_w;
                end

                if (shared_fanout_issue_host_c && shared_fanout_req_host_pending_r) begin
                    shared_fanout_req_host_pending_r <= 1'b0;
                end
                if (shared_fanout_issue_exec_c && shared_fanout_req_exec_pending_r) begin
                    shared_fanout_req_exec_pending_r <= 1'b0;
                end

                if (shared_fanout_issue_host_c || shared_fanout_issue_exec_c) begin
                    shared_fanout_read_inflight_r <= 1'b1;
                    shared_fanout_read_owner_host_r <= shared_fanout_issue_host_c;
                end

                if (shared_fanout_resp_valid_c) begin
                    shared_fanout_read_inflight_r <= 1'b0;
                end
            end
        end
    end

    // rv_if channel: tile_fanout_executor → tile_top local-loopback.
    // fanout_executor drives valid + tile_event_t payload; tile_top drives
    // ready = !logical_event_full_c (assigned after full is known).
    rv_if #(.WIDTH(TILE_EVENT_W)) fanout_ev_if();

    // rv_if channel: enqueue mux → per-worker event queues.
    rv_if #(.WIDTH(TILE_EVENT_W)) enqueue_ev_if();

    // ── Per-worker event queue signals (tile_spike_t FIFO interface) ──────────
    // Worker assignment: lower WORKER_IDX_W bits of neuron_idx select the FIFO.
    localparam int unsigned EQ_ELEM_W = TILE_QUEUE_EVENT_W;
    logic [WORKER_CORES_PER_TILE-1:0]                          eq_enqueue_valid;
    logic [EQ_ELEM_W*WORKER_CORES_PER_TILE-1:0]                eq_enqueue_data;
    logic [WORKER_CORES_PER_TILE-1:0]                          eq_enqueue_ready;
    // eq_not_full[i]: per-worker "queue has slack" predicate from the
    // bank.  Independent of `eq_enqueue_valid`, so this is what the
    // ingress-side fullness gate must read — see the comment on
    // `logical_event_full_c` below.
    logic [WORKER_CORES_PER_TILE-1:0]                          eq_not_full;
    logic [WORKER_CORES_PER_TILE-1:0]                          eq_dequeue_ready;
    logic [WORKER_CORES_PER_TILE-1:0]                          eq_dequeue_valid;
    logic [EQ_ELEM_W*WORKER_CORES_PER_TILE-1:0]                eq_dequeue_data;

    // Scalar event-queue signals derived from per-worker FIFOs.
    // pending_valid = any worker FIFO has data; full = target worker FIFO full.
    logic                    logical_event_pending_valid_c;
    logic [NEURON_IDX_W-1:0] logical_event_pending_idx_c;
    logic                    logical_event_full_c;
    logic signed [WEIGHT_W-1:0] logical_event_head_read_weight_c;
    logic [TAG_W-1:0]           logical_event_head_read_tag_c;  // stub: tile_spike_t has no tag
    logic [EVENT_TIME_W-1:0]    logical_event_head_read_time_c;

    // Declarations hoisted here to avoid Slang forward-reference errors.
    tile_event_t enqueue_ev_payload_c;
    wire  worker_start_fire_c;

    // Worker index assigned to an incoming spike (bottom WORKER_IDX_W bits).
    // Derived from enqueue_ev_payload_c.neuron_idx so all three enqueue paths
    // (unicast ingress, broadcast serializer, fanout loopback) are handled.
    logic [WORKER_IDX_W-1:0] enq_worker_sel_c;

    // Enqueue mux: route tile_queue_event_t to the selected worker FIFO
    tile_queue_event_t enqueue_spike_c;
    tile_queue_event_t sched_head_c;
    always_comb begin
        // Build tile_spike_t from enqueue_ev_payload_c (already muxed for all
        // three paths: unicast/broadcast/fanout-loopback).
        enqueue_spike_c.neuron_x   = NEURON_LOCAL_W'(enqueue_ev_payload_c.neuron_idx[NEURON_LOCAL_W-1:0]);
        enqueue_spike_c.neuron_y   = NEURON_LOCAL_W'(enqueue_ev_payload_c.neuron_idx[2*NEURON_LOCAL_W-1:NEURON_LOCAL_W]);
        enqueue_spike_c.weight     = enqueue_ev_payload_c.weight;
        enqueue_spike_c.event_time = enqueue_ev_payload_c.event_time;

        // With WORKER_CORES_PER_TILE==1 the forced WORKER_IDX_W==1 means
        // neuron_idx[0] alternates 0/1 for even/odd neurons, but worker
        // indices only span 0..0.  Indexing eq_not_full[1] is out-of-bounds
        // and propagates 'x' in simulation, causing Verilator DIDNOTCONVERGE.
        // Use constant 0 for the single-worker configuration; the general
        // multi-worker path retains the bit-slice.
        enq_worker_sel_c = (WORKER_CORES_PER_TILE == 1)
            ? WORKER_IDX_W'(0)
            : WORKER_IDX_W'(enqueue_ev_payload_c.neuron_idx[WORKER_IDX_W-1:0]);

        eq_enqueue_valid = '0;
        eq_enqueue_data  = '0;
        for (w = 0; w < WORKER_CORES_PER_TILE; w++) begin
            if (w == int'(enq_worker_sel_c)) begin
                eq_enqueue_valid[w] = enqueue_ev_if.valid;
                eq_enqueue_data[w*EQ_ELEM_W +: EQ_ELEM_W] = enqueue_spike_c;
            end
        end

        // Dequeue: scheduler fires pop on the worker it is dispatching
        eq_dequeue_ready = '0;
        eq_dequeue_ready[start_dispatch_worker_idx_r] = worker_start_fire_c;

        // Present scheduled worker's FIFO head to the scheduler
        sched_head_c = tile_queue_event_t'(
            eq_dequeue_data[start_dispatch_worker_idx_r*EQ_ELEM_W +: EQ_ELEM_W]);

        logical_event_pending_valid_c    = |eq_dequeue_valid;
        logical_event_pending_idx_c      = NEURON_IDX_W'(
            {sched_head_c.neuron_y, sched_head_c.neuron_x});
        // logical_event_full_c feeds tile_ingress's
        // `ingress_spike_enqueue_ready_calc_c` predicate, which gates
        // `current_packet_consume`, which gates `enqueue_ev_if.valid`
        // — i.e. valid depends on this signal.  Therefore this signal
        // MUST NOT depend on valid, which rules out the bank's
        // `eq_enqueue_ready` (a "currently firing" handshake output
        // gated on `enqueue_valid`).  We use `eq_not_full` instead,
        // which the bank derives from its internal occupancy state
        // alone.  Pre-fix this read `!eq_enqueue_ready[X]` and formed
        // a combinational valid-depends-on-ready loop that latched at
        // zero — the 2026-05 multi_edge_return_addressing wedge.  See
        // tile_event_queue_bank_handshake_formal for the in-bank
        // liveness proofs that pin this fix as a tile_top integration
        // issue rather than a bank bug.
        logical_event_full_c             = !eq_not_full[enq_worker_sel_c];
        logical_event_head_read_weight_c = sched_head_c.weight;
        logical_event_head_read_tag_c    = '0;  // not carried in tile_spike_t
        logical_event_head_read_time_c   = sched_head_c.event_time;

        // Back-pressure to ingress: ready iff the target worker has slack.
        // Same rationale as logical_event_full_c — `enqueue_ev_if.valid`
        // depends combinationally on this signal, so we read `eq_not_full`
        // (occupancy-only) rather than `eq_enqueue_ready` (firing-gated).
        enqueue_ev_if.ready = eq_not_full[enq_worker_sel_c];
    end

    logic [WORKER_CORES_PER_TILE-1:0] worker_start_valid_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_start_ready_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_start_ready_for_dispatch_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_result_valid_c;
    logic [WORKER_CORES_PER_TILE-1:0] worker_result_ready_c;
    wire  [WORKER_CORES_PER_TILE-1:0] worker_has_fanout_r;
    logic [NEURON_IDX_W-1:0]         worker_result_logical_idx_c [0:WORKER_CORES_PER_TILE-1];
    // Worker result payloads — grouped into struct arrays for readability.
    neuron_exec_ctx_t     worker_result_ctx_c  [0:WORKER_CORES_PER_TILE-1];
    neuron_emit_t          worker_result_emit_c [0:WORKER_CORES_PER_TILE-1];

    // Flat-packed versions for tile_dispatch_scheduler (Yosys-compatible).
    wire [WORKER_CORES_PER_TILE*NEURON_IDX_W-1:0] worker_result_logical_idx_packed;
    wire [WORKER_CORES_PER_TILE*NEURON_EXEC_CTX_W-1:0] worker_result_ctx_packed;
    wire [WORKER_CORES_PER_TILE*NEURON_EMIT_W-1:0]     worker_result_emit_packed;
    // WORKER_CORES_PER_TILE = 1: direct assigns, no generate loop needed.
    assign worker_result_logical_idx_packed = worker_result_logical_idx_c[0];
    assign worker_result_ctx_packed         = worker_result_ctx_c[0];
    assign worker_result_emit_packed        = worker_result_emit_c[0];

    wire  [NEURON_IDX_W-1:0] worker_start_logical_idx_c;   // unused now
    wire  [WORKER_IDX_W-1:0] worker_start_idx_c;           // unused now
    // worker_start_fire_c declared earlier (hoisted above always_comb that uses it).
    wire  _sched_dequeue_valid_alias;
    wire  [NEURON_IDX_W-1:0] _sched_dequeue_idx_alias;
    wire  [NEURON_IDX_W-1:0] worker_commit_logical_idx_c;
    wire  [WORKER_IDX_W-1:0] worker_commit_idx_c;
    wire  worker_commit_valid_c;
    wire  worker_commit_ready_c;

    // context_commit_if (rv_if) declared in the rv_if instance block above.
    // Unpack helper: drives context_bank ports from the rv_if payload.
    context_commit_t context_commit_data_c;
    assign context_commit_data_c = context_commit_t'(context_commit_if.rv_payload);
    assign context_commit_if.ready = 1'b1;  // context_bank has no back-pressure
    logic ctrl_write_en_mux_c;
    logic [NEURON_IDX_W-1:0] ctrl_write_neuron_idx_mux_c;
    logic ctrl_write_broadcast_mux_c;
    logic [3:0] ctrl_write_addr_mux_c;
    logic [7:0] ctrl_write_data_mux_c;
    logic ucode_write_en_mux_c;
    logic [NEURON_IDX_W-1:0] ucode_write_neuron_idx_mux_c;
    logic ucode_write_broadcast_mux_c;
    logic [4:0] ucode_write_index_mux_c;
    logic [15:0] ucode_write_word_mux_c;
    logic ucode_prog_en_mux_c;
    logic [NEURON_IDX_W-1:0] ucode_prog_neuron_idx_mux_c;
    logic ucode_prog_broadcast_mux_c;
    logic [4:0] ucode_prog_ptr_mux_c;
    logic [4:0] ucode_prog_len_mux_c;
    logic fanout_prog_ptr_en_mux_c;
    logic fanout_prog_len_en_mux_c;
    logic [NEURON_IDX_W-1:0] fanout_prog_neuron_idx_mux_c;
    logic fanout_prog_broadcast_mux_c;
    logic [FANOUT_ADDR_W-1:0] fanout_prog_ptr_mux_c;
    logic [FANOUT_ADDR_W-1:0] fanout_prog_len_mux_c;
    // fanout_write_if (rv_if) and fanout_write_addr_c declared above near
    // the rv_if instance block.  The fanout write address is now the sole
    // companion signal carried alongside the fanout_write_if rv_if channel.

    // All fanout-executor state / combinational signals moved into
    // u_fanout_executor (see ai/prompts/tile_decomposition.md, Module 3).
    // Only the signals crossing the module boundary remain declared in
    // tile_top; see the block above.
    // Dump FSM localparams moved to tile_host_io.
    // Helper functions (local_core_id_for_index, local_target_valid_for_core_id,
    // onehot_for_idx, next_host_output_fifo_ptr, next_fanout_queue_ptr,
    // next_ready_fifo_ptr, neuron_mask_for_bit, worker_mask_for_bit,
    // find_next_ready_candidate, route_exists_from, first_route_index_from,
    // route_exists_after, next_route_index, find_next_worker_rr,
    // fanout_count_low{8,5}, fanout_base_low8, pack_dump_coord, make_*_packet,
    // make_local_read_response_packet) previously defined here were moved
    // into tile_ingress, tile_host_io, tile_fanout_executor, and
    // tile_dispatch_scheduler as part of the Phase 2 decomposition. They
    // are no longer referenced from this file.

    // ingress_packet_c / classify_* regs / classify_load_c now live inside
    // tile_ingress (see u_ingress instantiation later).  current_packet_c,
    // current_*_c etc. are driven out of tile_ingress.

    // ucode_prog_end_en_c has been dead since the CMD_UCODE write/prog path
    // collapsed; tie to 0 but keep driving the tile_ingress input port.
    assign ucode_prog_end_en_c = 1'b0;
    assign worker_start_ready_for_dispatch_c =
        worker_start_ready_c & {WORKER_CORES_PER_TILE{ctx_dispatch_allowed_c}};

    // host_slot_free / host_output_fifo / rv_host_out assigns moved to
    // u_host_io; the module exposes host_slot_free_r_out which is wired
    // through to u_ingress.  fanout_queue_* assigns moved into u_fanout_executor.

`ifndef LIBRELANE
    initial begin
        if (WORKER_CORES_PER_TILE < 1) begin
            $error("tile_top: WORKER_CORES_PER_TILE must be at least 1");
        end
        if (NEURONS_PER_TILE < 1) begin
            $error("tile_top: NEURONS_PER_TILE must be at least 1");
        end
        if (LOGICAL_EVENT_QUEUE_DEPTH < 1) begin
            $error("tile_top: LOGICAL_EVENT_QUEUE_DEPTH must be at least 1");
        end
        if (WORKER_CORES_PER_TILE > NEURONS_PER_TILE) begin
            $error("tile_top: WORKER_CORES_PER_TILE (%0d) must not exceed NEURONS_PER_TILE (%0d)",
                   WORKER_CORES_PER_TILE, NEURONS_PER_TILE);
        end
    end
`endif

    // Ucode bank port-0 mux: host_io aux request has priority over worker 0.
    // WORKER_CORES_PER_TILE = 1, so there is exactly one worker port.
    //
    // Request: valid = aux_req OR worker read; payload selected by aux_req.
    // Response: worker-0 valid is suppressed when host_io owns the in-flight
    //           pipeline slot (port0_ucode_host_or_dump_in_flight_c).
    neuron_ucode_req_t ucode_bank_req_payload_c;

    assign ucode_bank_req_if.valid = port0_ucode_aux_req_c || worker_ucode_read_valid_c[0];
    assign ucode_bank_req_payload_c = '{
        logical_idx: NEURON_IDX_W'(port0_ucode_aux_req_c
            ? host_io_port0_ucode_aux_neuron_idx_w
            : worker_ucode_read_logical_idx_c[0]),
        word_index: port0_ucode_aux_req_c
            ? host_io_port0_ucode_aux_word_index_w
            : worker_ucode_read_word_index_c[0]
    };
    assign ucode_bank_req_if.rv_payload =
        NEURON_UCODE_REQ_W'(ucode_bank_req_payload_c);

    // Worker-0 response: suppress when host_io's aux pipeline owns the slot.
    assign worker_ucode_read_word_valid_c[0] =
        ucode_bank_rsp_if.valid && !port0_ucode_host_or_dump_in_flight_c;
    assign worker_ucode_read_word_c[0] =
        {4'b0, ucode_bank_rsp_if.rv_payload};  // payload is NEURON_UCODE_RSP_W bits

    // fanout_bank_read_en_c / fanout_bank_read_index_c: previously driven the
    // old random-access fanout pool.  The pool no longer exists; these are now
    // stubbed above.  Shared-fanout arbitration is preserved for the follow-up
    // fanout_executor migration.
    assign fanout_bank_read_en_c    = 1'b0;
    assign fanout_bank_read_index_c = '0;

    // fanout_local_* / retry / lane-accept combinational logic moved into
    // u_fanout_executor.  This module now exposes fanout_local_route_c +
    // enqueue_mask_c / enqueue_ready_c / weight / tag / time, which are
    // consumed by the logical_event_enqueue mux below.
    //
    // packet_graph_state_clear_c / tile_graph_state_clear_c / soft_reset_valid_sbank_c
    // and the bank-write mux are driven by u_ingress.  rv_in_ready is
    // also an ingress output.

    // Logical-event enqueue mux — drives rv_if enqueue_ev_if.
    // fanout_ev_if.ready = !full is also assigned here so the executor's
    // rv_if ready signal is driven correctly.
    // Broadcast spikes: ingress fires once per packet (unicast enqueue for
    // the *first* neuron); the bcast counter below handles neurons 1..N-1.
    // enqueue_ev_payload_c declared earlier (hoisted above always_comb that uses it).
    always_comb begin
        enqueue_ev_if.valid      = 1'b0;
        enqueue_ev_payload_c     = '0;
        fanout_ev_if.ready       = !logical_event_full_c;

        if (ingress_bcast_in_progress_r) begin
            // Broadcast serialization: enqueue current counter address.
            enqueue_ev_if.valid         = 1'b1;
            enqueue_ev_payload_c.neuron_idx  = NEURON_IDX_W'(ingress_bcast_addr_r);
            enqueue_ev_payload_c.weight  = current_packet_c.weight;
            enqueue_ev_payload_c.tag     = current_packet_c.tag;
            enqueue_ev_payload_c.event_time = '0;  // removed from message_packet_t (neutern)
        end else if (ingress_event_enqueue_fire_c && !ingress_event_is_bcast_c) begin
            // Unicast ingress enqueue.
            enqueue_ev_if.valid         = 1'b1;
            enqueue_ev_payload_c.neuron_idx  = NEURON_IDX_W'(ingress_event_target_idx_c);
            enqueue_ev_payload_c.weight  = current_packet_c.weight;
            enqueue_ev_payload_c.tag     = current_packet_c.tag;
            enqueue_ev_payload_c.event_time = '0;  // removed from message_packet_t (neutern)
        end else if (ingress_event_enqueue_fire_c && ingress_event_is_bcast_c) begin
            // First beat of broadcast: enqueue neuron 0 this cycle.
            enqueue_ev_if.valid         = 1'b1;
            enqueue_ev_payload_c.neuron_idx  = '0;
            enqueue_ev_payload_c.weight  = current_packet_c.weight;
            enqueue_ev_payload_c.tag     = current_packet_c.tag;
            enqueue_ev_payload_c.event_time = '0;  // removed from message_packet_t (neutern)
        end else if (fanout_ev_if.valid) begin
            // Fanout local loopback: relay fanout_ev payload directly.
            enqueue_ev_if.valid      = 1'b1;
            enqueue_ev_payload_c     = tile_event_t'(fanout_ev_if.rv_payload);
        end
    end
    assign enqueue_ev_if.rv_payload = enqueue_ev_payload_c;

    // Broadcast serialization counter: started when ingress fires a bcast
    // spike, walks addr 1..N-1 (addr 0 already enqueued above), clears when
    // addr reaches N-1.
    always_ff @(posedge clk) begin
        if (!rst_n || tile_graph_state_clear_r) begin
            ingress_bcast_in_progress_r <= 1'b0;
            ingress_bcast_addr_r        <= '0;
        end else if (ena) begin
            if (ingress_event_enqueue_fire_c && ingress_event_is_bcast_c &&
                    (NEURONS_PER_TILE > 1)) begin
                // Start serialization from addr 1 (addr 0 enqueued this cycle).
                ingress_bcast_in_progress_r <= 1'b1;
                ingress_bcast_addr_r        <= NEURON_IDX_W'(1);
            end else if (ingress_bcast_in_progress_r && !logical_event_full_c) begin
                if (ingress_bcast_addr_r == NEURON_IDX_W'(NEURONS_PER_TILE - 1)) begin
                    ingress_bcast_in_progress_r <= 1'b0;
                    ingress_bcast_addr_r        <= '0;
                end else begin
                    ingress_bcast_addr_r <= ingress_bcast_addr_r + NEURON_IDX_W'(1);
                end
            end
        end
    end

    // Scheduler drives worker_start_event_* directly (see u_dispatch_scheduler
    // wiring below).  worker_start_event_*_c are assigned from the scheduler's
    // outputs for the compute core connections.

    // current_host_response_valid_c / _payload_c are driven by u_ingress.

    // Host/dump response payload builders moved to u_host_io.

    // Fanout queue pop/push/retry control + noc_child_* mux + fanout_fire_c
    // all moved to u_fanout_executor.  noc_child_{valid,payload}_c are now
    // driven out of that module; noc_child_ready_c is the acknowledgement
    // from u_noc_egress and flows back in as an input.
    //
    // Packet consume decoder lives in u_ingress; the dump-active-gated
    // ucode/fanout read-issue signals live in u_host_io.  `dump_fanout_read_issue_c`
    // and `dump_ucode_read_issue_c` are driven by u_host_io outputs.
    //
    // Dispatch ready-FIFO + worker RR + start_dispatch pipeline + commit
    // arbitration + context_commit fan-out all moved into u_dispatch_scheduler.

    tile_ingress #(
        .LOCAL_Z(LOCAL_Z),
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .INGRESS_QUEUE_DEPTH(INGRESS_QUEUE_DEPTH),
        .MESSAGE_W(MESSAGE_W)
    ) u_ingress (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .tile_x_coord(tile_x_coord),
        .tile_y_coord(tile_y_coord),
        .rv_in(rv_in),
        .current_packet(current_packet_c),
        .current_packet_valid(current_packet_valid_c),
        .current_local_tile_match(current_local_tile_match_c),
        .current_core_broadcast(current_core_broadcast_c),
        .current_has_target(current_has_target_c),
        .packet_from_loader(packet_from_loader_c),
        .current_packet_consume(current_packet_consume_c),
        .ingress_consume(ingress_consume_c),
        .host_reserve_current(host_reserve_current_c),
        .host_slot_free_r(host_slot_free_r),
        .dump_active_r(dump_active_r),
        .ingress_spike_enqueue_fire(ingress_event_enqueue_fire_c),
        .ingress_spike_target_idx(ingress_event_target_idx_c),
        .ingress_spike_is_bcast(ingress_event_is_bcast_c),
        .ingress_spike_enqueue_ready(ingress_event_enqueue_ready_c),
        .logical_spike_full(logical_event_full_c),
        .packet_graph_state_clear(packet_graph_state_clear_c),
        .tile_graph_state_clear(tile_graph_state_clear_c),
        .ucode_host_read_issue(ucode_host_read_issue_c),
        .weight_host_read_issue(weight_host_read_issue_c),
        .route_host_read_issue(route_host_read_issue_c),
        .dump_neuron_issue(dump_neuron_issue_c),
        .weight_write_en(weight_write_en_c),
        .route_write_en(route_write_en_c),
        .current_host_response_valid(current_host_response_valid_c),
        .current_host_response_payload(current_host_response_payload_c),
        .state_target_csr_read_data(state_target_csr_read_data_c),
        .calc_out_packets(calc_out_packets),
        .calc_in_packets(calc_in_packets),
        .selected_target_idx(selected_target_idx_c),
        .ctrl_write_en_mux(ctrl_write_en_mux_c),
        .ctrl_write_neuron_idx_mux(ctrl_write_neuron_idx_mux_c),
        .ctrl_write_broadcast_mux(ctrl_write_broadcast_mux_c),
        .ctrl_write_addr_mux(ctrl_write_addr_mux_c),
        .ctrl_write_data_mux(ctrl_write_data_mux_c),
        .ucode_prog_end_en(ucode_prog_end_en_c),
        .ucode_write_en_mux(ucode_write_en_mux_c),
        .ucode_write_neuron_idx_mux(ucode_write_neuron_idx_mux_c),
        .ucode_write_broadcast_mux(ucode_write_broadcast_mux_c),
        .ucode_write_index_mux(ucode_write_index_mux_c),
        .ucode_write_word_mux(ucode_write_word_mux_c),
        .ucode_prog_en_mux(ucode_prog_en_mux_c),
        .ucode_prog_neuron_idx_mux(ucode_prog_neuron_idx_mux_c),
        .ucode_prog_broadcast_mux(ucode_prog_broadcast_mux_c),
        .ucode_prog_ptr_mux(ucode_prog_ptr_mux_c),
        .ucode_prog_len_mux(ucode_prog_len_mux_c),
        .fanout_prog_ptr_en_mux(fanout_prog_ptr_en_mux_c),
        .fanout_prog_len_en_mux(fanout_prog_len_en_mux_c),
        .fanout_prog_neuron_idx_mux(fanout_prog_neuron_idx_mux_c),
        .fanout_prog_broadcast_mux(fanout_prog_broadcast_mux_c),
        .fanout_prog_ptr_mux(fanout_prog_ptr_mux_c),
        .fanout_prog_len_mux(fanout_prog_len_mux_c),
        .fanout_write(fanout_write_if),
        .fanout_write_addr(fanout_write_addr_c),
        .state_target_fanout_ptr(state_target_fanout_ptr_c)
    );

    tile_host_io #(
        .LOCAL_Z(LOCAL_Z),
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE(WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .MESSAGE_W(MESSAGE_W),
        .HOST_OUTPUT_FIFO_DEPTH(HOST_OUTPUT_FIFO_DEPTH),
        .FANOUT_DST_X_W(FANOUT_DST_X_W),
        .FANOUT_DST_Y_W(FANOUT_DST_Y_W),
        .FANOUT_CORE_ID_W(FANOUT_CORE_ID_W)
    ) u_host_io (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .tile_x_coord(tile_x_coord),
        .tile_y_coord(tile_y_coord),
        .graph_state_clear(tile_graph_state_clear_r),

        .rv_host_out(rv_host_out_if),

        .current_host_response_valid(current_host_response_valid_c),
        .current_host_response_payload(current_host_response_payload_c),
        .ucode_host_read_issue(ucode_host_read_issue_c),
        .weight_host_read_issue(weight_host_read_issue_c),
        .route_host_read_issue(route_host_read_issue_c),
        .dump_neuron_issue(dump_neuron_issue_c),
        .current_packet(current_packet_c),
        .host_reserve_current(host_reserve_current_c),

        .host_slot_free_r_out(host_slot_free_r),
        .dump_active(dump_active_r),

        .port0_ucode_aux_req(port0_ucode_aux_req_c),
        .port0_ucode_aux_is_dump(port0_ucode_aux_is_dump_c),
        .port0_ucode_aux_word_index(port0_ucode_aux_word_index_c),
        .port0_ucode_aux_neuron_idx(port0_ucode_aux_neuron_idx_c),
        .port0_ucode_aux_in_flight(port0_ucode_host_or_dump_in_flight_c),
        .ucode_rsp_valid_port0(ucode_rsp_valid_port0_c),
        .ucode_rsp_word_port0(ucode_rsp_word_port0_c),

        .selected_target_idx(selected_target_idx_c),

        .fanout_bank_port0_read_en(fanout_bank_port0_read_en_w),
        .fanout_bank_port0_neuron_idx(fanout_bank_port0_neuron_idx_w),
        .fanout_bank_port0_index(fanout_bank_port0_index_w),
        .fanout_rsp(fanout_rsp_host_if),

        .dump_target_idx(dump_target_idx_r),
        .state_dump_init_rf_flat(state_dump_init_rf_flat_c),
        .state_dump_ucode_ptr(state_dump_ucode_ptr_c),
        .state_dump_ucode_len(state_dump_ucode_len_c),
        .state_dump_neuron_flags(state_dump_neuron_flags_c),
        .state_dump_fanout_ptr(state_dump_fanout_ptr_c),
        .state_dump_fanout_len(state_dump_fanout_len_c),
        .state_target_fanout_ptr(state_target_fanout_ptr_c),
        .host_egress_en_bus(host_egress_en_bus_c),
        .noc_egress_en_bus(noc_egress_en_bus_c),
        // ctx_read_last_time_b removed: timestamps eliminated

        .worker_commit_valid(worker_commit_valid_c),
        .worker_commit_ready(worker_commit_ready_c),
        .worker_commit_idx(worker_commit_idx_c),
        .worker_commit_logical_idx(worker_commit_logical_idx_c),
        .worker_result_emit(worker_result_emit_c[worker_commit_idx_c])
    );

    // Stall aggregation — always 0 for FF-backed memories.
    logic ctx_mem_stall_c;
    logic state_mem_stall_c;
    logic event_mem_stall_c;
    logic fanout_mem_stall_c;
    logic fanout_pool_stall_c;
    wire  mem_stall_c = ctx_mem_stall_c | state_mem_stall_c
                      | event_mem_stall_c | fanout_mem_stall_c
                      | fanout_pool_stall_c;

    tile_fanout_executor #(
        .LOCAL_Z(LOCAL_Z),
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE(WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .FANOUT_QUEUE_DEPTH(FANOUT_QUEUE_DEPTH),
        .MESSAGE_W(MESSAGE_W),
        .FANOUT_DST_X_W(FANOUT_DST_X_W),
        .FANOUT_DST_Y_W(FANOUT_DST_Y_W),
        .FANOUT_CORE_ID_W(FANOUT_CORE_ID_W)
    ) u_fanout_executor (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .tile_x_coord(tile_x_coord),
        .tile_y_coord(tile_y_coord),
        .tile_graph_state_clear(tile_graph_state_clear_r),

        .worker_commit_valid(worker_commit_valid_c),
        .worker_commit_ready(worker_commit_ready_c),
        .worker_commit_logical_idx(worker_commit_logical_idx_c),
        .worker_commit_idx(worker_commit_idx_c),
        .worker_has_fanout(worker_has_fanout_r[worker_commit_idx_c]),
        .worker_result_emit(worker_result_emit_c[worker_commit_idx_c]),

        .fanout_queue_can_accept(fanout_queue_can_accept_c),
        .fanout_queue_count_live(fanout_queue_count_live_c),
        .fanout_fire_dbg(fanout_fire_c),

        .fanout_read_idx(fanout_read_idx_w),
        .state_fanout_ptr(state_fanout_ptr_c),
        .state_fanout_len(state_fanout_len_c),
        .state_commit_fanout_ptr(state_commit_fanout_ptr_c),

        .fanout_bank_p1_read_en(fanout_bank_port1_read_en_w),
        .fanout_bank_p1_neuron_idx(fanout_bank_port1_neuron_idx_w),
        .fanout_bank_p1_index(fanout_bank_port1_index_w),
        .fanout_rsp(fanout_rsp_exec_if),

        .noc_child(noc_child_if),

        .fanout_ev(fanout_ev_if),

        .ingress_spike_enqueue_fire(ingress_event_enqueue_fire_c),
        .ingress_spike_target_idx(ingress_event_target_idx_c),

        .start_dispatch_valid(start_dispatch_valid_r),
        .start_dispatch_logical_idx(start_dispatch_logical_idx_r),
        .worker_start_ready_for_dispatch(
            worker_start_ready_for_dispatch_c[start_dispatch_worker_idx_r] && !mem_stall_c
        ),

        .logical_spike_full(logical_event_full_c)
`ifdef FORMAL
        ,
        .formal_retry_valid_obs        (),
        .formal_stall_defer_obs        (),
        .formal_retry_start_obs        (),
        .formal_local_pending_valid_obs(),
        .formal_local_pending_addr_obs (),
        .formal_local_pending_bcast_obs(),
        .formal_fanout_active_obs      (),
        .formal_route_idx_obs          ()
`endif
    );

    tile_dispatch_scheduler #(
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE(WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .FANOUT_QUEUE_DEPTH(FANOUT_QUEUE_DEPTH),
        .SYNAPSE_SID_W(SYNAPSE_SID_W),
        .LOCAL_Z(LOCAL_Z)
    ) u_dispatch_scheduler (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .tile_graph_state_clear(tile_graph_state_clear_r),

        .soft_reset_valid_in(soft_reset_valid_sbank_c),
        .soft_reset_idx_in(soft_reset_idx_sbank_c),
        .soft_reset_valid(soft_reset_valid_c),
        .soft_reset_idx(soft_reset_idx_c),

        .logical_spike_pending_valid(logical_event_pending_valid_c),
        .logical_spike_pending_idx(logical_event_pending_idx_c),
        .logical_spike_full(logical_event_full_c),
        .logical_spike_head_read_weight(logical_event_head_read_weight_c),
        .logical_spike_head_read_tag(logical_event_head_read_tag_c),
        .logical_spike_head_read_time(logical_event_head_read_time_c),
        .dequeue_valid(_sched_dequeue_valid_alias),
        .dequeue_idx(_sched_dequeue_idx_alias),
        .head_read_idx(worker_start_logical_idx_c),

        .state_dispatch_read_idx(state_dispatch_read_idx_c),
        .state_dispatch_ucode_ptr(state_dispatch_ucode_ptr_c),
        .state_dispatch_ucode_len(state_dispatch_ucode_len_c),
        .state_dispatch_fanout_len(state_dispatch_fanout_len_c),
        .noc_egress_en(noc_egress_en_c),

        .worker_start_valid(worker_start_valid_c),
        .worker_start_ready(worker_start_ready_for_dispatch_c),
        .worker_start_event_tag(worker_start_event_tag_c),
        .worker_start_event_time(worker_start_event_time_c),
        .worker_start_event_weight(worker_start_event_weight_c),
        .start_dispatch_logical_idx(start_dispatch_logical_idx_r),
        .start_dispatch_valid_out(start_dispatch_valid_r),
        .start_dispatch_worker_idx_out(start_dispatch_worker_idx_r),
        .worker_start_fire(worker_start_fire_c),

        .worker_result_valid(worker_result_valid_c),
        .worker_result_ready(worker_result_ready_c),
        .worker_result_logical_idx(worker_result_logical_idx_packed),
        .worker_result_ctx(worker_result_ctx_packed),
        .worker_result_emit(worker_result_emit_packed),

        .worker_commit_valid(worker_commit_valid_c),
        .worker_commit_ready(worker_commit_ready_c),
        .worker_commit_logical_idx(worker_commit_logical_idx_c),
        .worker_commit_idx(worker_commit_idx_c),
        .worker_has_fanout_snapshot(worker_has_fanout_r),

        .context_commit(context_commit_if),

        .host_reserve_current(host_reserve_current_c),
        .fanout_queue_can_accept(fanout_queue_can_accept_c),
        .fanout_queue_count_live(fanout_queue_count_live_c)
`ifdef FORMAL
        ,
        .formal_worker_rr_ptr_r            (),
        .formal_logical_event_inflight_r   (),
        .formal_in_ready_fifo_r            (),
        .formal_ready_push                 (),
        .formal_ready_push_idx             (),
        .formal_dispatch_fire              (),
        .formal_dispatch_idx               (),
        .formal_prev_pending_valid_r       (),
        .formal_prev_pending_idx_r         (),
        .formal_start_dispatch_pending_r   (),
        .formal_start_dispatch_valid_r     (),
        .formal_start_dispatch_logical_idx_r(),
        .formal_ready_fifo_full            (),
        .formal_ready_push_mask_r          (),
        .formal_pop_reservation_valid_r    (),
        .formal_pop_reservation_idx_r      ()
`endif
    );

    logical_neuron_state_bank #(
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .MEM_STYLE_HINT(TILE_BANK_MEM_STYLE),
        .UCODE_SHARED_BANK(UCODE_SHARED_BANK_CFG),
        .ASIC_ZERO_INIT_RESET(ASIC_ZERO_INIT_RESET_CFG),
        .DROP_REDUNDANT_FANOUT_BRAM(DROP_REDUNDANT_FANOUT_BRAM_CFG),
        .STATE_ROW_NO_FANOUT_BYTES(STATE_ROW_NO_FANOUT_BYTES_CFG),
        .DROP_DISPATCH_BRAM_ON_COMPACT(DROP_DISPATCH_BRAM_ON_COMPACT_CFG),
        .SHARED_NEURON_CONFIG(IS_ASIC_LEAN ? 1 : 0)
    ) u_state_bank (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .graph_state_clear(tile_graph_state_clear_r),
        .csr_write_en(ctrl_write_en_mux_c),
        .csr_write_addr(ctrl_write_neuron_idx_mux_c),
        .csr_write_broadcast(ctrl_write_broadcast_mux_c),
        .csr_addr(ctrl_write_addr_mux_c),
        .csr_data(ctrl_write_data_mux_c),
        .target_read_idx(selected_target_idx_c),
        .target_csr_read_data(state_target_csr_read_data_c),
        .target_fanout_ptr(state_target_fanout_ptr_c),
        .target_fanout_len(state_target_fanout_len_c),
        .dispatch_read_idx(state_dispatch_read_idx_c),
        .dispatch_ucode_ptr(state_dispatch_ucode_ptr_c),
        .dispatch_ucode_len(state_dispatch_ucode_len_c),
        .dispatch_fanout_len(state_dispatch_fanout_len_c),
        .fanout_read_idx(fanout_read_idx_w),
        .fanout_read_ptr(state_fanout_ptr_c),
        .fanout_read_len(state_fanout_len_c),
        .commit_read_idx(worker_commit_logical_idx_c),
        .commit_fanout_ptr(state_commit_fanout_ptr_c),
        .dump_read_idx(dump_target_idx_r),
        .dump_init_rf_flat(state_dump_init_rf_flat_c),
        .dump_ucode_ptr(state_dump_ucode_ptr_c),
        .dump_ucode_len(state_dump_ucode_len_c),
        .dump_neuron_flags(state_dump_neuron_flags_c),
        .dump_fanout_ptr(state_dump_fanout_ptr_c),
        .dump_fanout_len(state_dump_fanout_len_c),
        .mem_stall(state_mem_stall_c),
        .reset_read_idx(soft_reset_idx_c),
        .reset_init_rf_flat(state_reset_init_rf_flat_c),
        // reset_last_event_time removed: timestamps eliminated
        .soft_reset_pulse(soft_reset_pulse_c),
        .host_egress_en_bus(host_egress_en_bus_c),
        .noc_egress_en_bus(noc_egress_en_bus_c),
        .ucode_prog_en(ucode_prog_en_mux_c),
        .ucode_prog_addr(ucode_prog_neuron_idx_mux_c),
        .ucode_prog_ptr(ucode_prog_ptr_mux_c),
        .ucode_prog_len(ucode_prog_len_mux_c),
        .fanout_prog_ptr_en(fanout_prog_ptr_en_mux_c),
        .fanout_prog_len_en(fanout_prog_len_en_mux_c),
        .fanout_prog_addr(fanout_prog_neuron_idx_mux_c),
        .fanout_prog_ptr(fanout_prog_ptr_mux_c),
        .fanout_prog_len(fanout_prog_len_mux_c),
        .soft_reset_valid(soft_reset_valid_sbank_c),
        .soft_reset_idx(soft_reset_idx_sbank_c)
    );

    logical_neuron_context_bank #(
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .MEM_STYLE_HINT(TILE_BANK_MEM_STYLE),
        .CONTEXT_SPLIT_LAYOUT(CONTEXT_SPLIT_LAYOUT_CFG),
        .DUAL_READ_CONTEXT(DUAL_READ_CONTEXT_CFG)
    ) u_context_bank (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .graph_state_clear(tile_graph_state_clear_r),
        .soft_reset_valid(soft_reset_valid_c),
        .soft_reset_idx(soft_reset_idx_c),
        .init_rf_flat(state_reset_init_rf_flat_c),
        // init_last_time removed: timestamps eliminated
        .commit_valid(context_commit_if.valid),
        .commit_idx(context_commit_data_c.idx),
        .commit_rf_state_flat(context_commit_data_c.ctx.rf_state_flat),
        .commit_last_tag(context_commit_data_c.ctx.last_tag),
        // commit_last_time removed: timestamps eliminated
        .commit_cmp_ge(context_commit_data_c.ctx.cmp_ge),
        .commit_cmp_eq(context_commit_data_c.ctx.cmp_eq),
        .commit_spike_flag(context_commit_data_c.ctx.spike_flag),
        .read_idx_a(ctx_read_idx_a_c),
        .read_rf_state_flat_a(ctx_read_rf_state_flat_a_c),
        .read_last_tag_a(ctx_read_last_tag_a_c),
        // read_last_time_a removed: timestamps eliminated
        .read_cmp_ge_a(ctx_read_cmp_ge_a_c),
        .read_cmp_eq_a(ctx_read_cmp_eq_a_c),
        .read_spike_flag_a(ctx_read_spike_flag_a_c),
        .read_idx_b(ctx_read_idx_b_c),
        .read_rf_state_flat_b(ctx_read_rf_state_flat_b_c),
        .read_last_tag_b(ctx_read_last_tag_b_c),
        // read_last_time_b removed: timestamps eliminated
        .read_cmp_ge_b(ctx_read_cmp_ge_b_c),
        .read_cmp_eq_b(ctx_read_cmp_eq_b_c),
        .read_spike_flag_b(ctx_read_spike_flag_b_c),
        .mem_stall(ctx_mem_stall_c)
    );

    tile_event_queue_bank #(
        .WORKER_CORES_PER_TILE(WORKER_CORES_PER_TILE),
        .FIFO_DEPTH_PER_WORKER(LOGICAL_EVENT_QUEUE_DEPTH),
        .MEM_STYLE_HINT       (TILE_BANK_MEM_STYLE),
        .EVENT_W              (EQ_ELEM_W)
    ) u_event_queue_bank (
        .clk            (clk),
        .rst_n          (rst_n),
        .ena            (ena),
        .soft_reset_valid(soft_reset_valid_sbank_c),
        .enqueue_valid  (eq_enqueue_valid),
        .enqueue_data   (eq_enqueue_data),
        .enqueue_ready  (eq_enqueue_ready),
        .dequeue_ready  (eq_dequeue_ready),
        .dequeue_valid  (eq_dequeue_valid),
        .dequeue_data   (eq_dequeue_data),
        .not_full       (eq_not_full),
        .mem_stall      (event_mem_stall_c)
    );

    // ── Fanout pool: flat per-synapse connection table ─────────────────────
    // Stores FANOUT_POOL_DEPTH entries of { weight, dst_x, dst_y, core_id,
    // meta } (5 bytes each).  The executor (u_fanout_executor) reads entries
    // as it walks per-neuron fanout ranges; the host programs entries via
    // MSG_WRITE CMD_WEIGHT / CMD_ROUTE packets that arrive through u_ingress.
    //
    // The single physical read port is pre-arbitrated here by the
    // shared_fanout_issue_* combinational signals: executor reads have
    // priority over host/dump reads.  The response is demultiplexed to
    // fanout_rsp_valid_host_c / fanout_rsp_valid_exec_c in the block below.
    tile_fanout_pool #(
        .FANOUT_POOL_DEPTH(FANOUT_POOL_DEPTH),
        .MEM_STYLE_HINT   (TILE_BANK_MEM_STYLE),
        // Pass mesh address widths so the pool packs routing bits tightly
        // (e.g. 2+2+2=6 bits for the 4x4/4-worker area target) instead of
        // always allocating 3 full 8-bit coordinate bytes.
        .FANOUT_DST_X_W   (FANOUT_DST_X_W),
        .FANOUT_DST_Y_W   (FANOUT_DST_Y_W),
        .FANOUT_CORE_ID_W (FANOUT_CORE_ID_W)
    ) u_fanout_pool (
        .clk                (clk),
        .rst_n              (rst_n),
        .ena                (ena),
        .graph_state_clear  (tile_graph_state_clear_r),
        // Write port — rv_if carrying fanout_write_req_t + companion address
        .fanout_write_if    (fanout_write_if),
        .write_index        (fanout_write_addr_c),
        // Single pre-arbitrated read port (executor priority over host/dump)
        .read_en            (shared_fanout_issue_host_c | shared_fanout_issue_exec_c),
        .read_index         (shared_fanout_issue_index_c),
        // Read response → fanout_pool_rsp_if rv_if (demuxed to exec/host above)
        .fanout_rsp         (fanout_pool_rsp_if),
        .mem_stall          (fanout_pool_stall_c)
    );

    logical_neuron_ucode_bank #(
        .NEURONS_PER_TILE(NEURONS_PER_TILE),
        .UCODE_WORDS_PER_NEURON(UCODE_WORDS_PER_NEURON_CFG),
        .UCODE_SHARED_BANK(UCODE_SHARED_BANK_CFG)
    ) u_ucode_bank (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .write_en(ucode_write_en_mux_c),
        .write_neuron_idx(ucode_write_neuron_idx_mux_c),
        .write_broadcast(ucode_write_broadcast_mux_c),
        .write_word_index(ucode_write_index_mux_c),
        .write_word(ucode_write_word_mux_c),
        .read_port(ucode_bank_req_if),
        .read_rsp_port(ucode_bank_rsp_if)
    );

    // Fanout mem stall: always 0 — FF-backed memories have no wait state.
    assign fanout_mem_stall_c = 1'b0;

    tile_noc_egress #(
        .PAYLOAD_W(MESSAGE_W)
    ) u_noc_egress (
        .clk(clk),
        .rst_n(rst_n),
        .ena(ena),
        .graph_state_clear(tile_graph_state_clear_r),
        .child(noc_child_if),
        .rv_out(rv_noc_out_if),
        .tile_valid_dbg(noc_tile_valid_unused_c)
`ifdef FORMAL
        ,
        .tile_count_obs()
`endif
    );

    // ── Compute core interface channels ───────────────────────────────────────
    rv_if #(.WIDTH(NEURON_WORKER_START_W))  worker_start_if();
    rv_if #(.WIDTH(NEURON_UCODE_REQ_W))     ucode_read_if();
    rv_if #(.WIDTH(NEURON_UCODE_RSP_W))     ucode_rsp_if();
    rv_if #(.WIDTH(NEURON_WORKER_RESULT_W)) worker_result_if();

    // ── Shared fanout staging bank (single FIFO, depth=4, all neurons) ────────
    // One tile_fanout_bank replaces the previous per-neuron array.  Depth=4
    // matches NEURONS_PER_TILE.  Enqueue/dequeue wired when fanout_executor
    // integration is complete; held at 0 for now.
    logic                    fo_enq_ready_w;
    logic                    fo_deq_valid_w;
    logic [FANOUT_SPIKE_W-1:0] fo_deq_data_w;
    logic                    fo_stall_w;

    tile_fanout_bank #(
        .WORKER_CORES_PER_TILE(1),
        .FIFO_DEPTH_PER_WORKER(FANOUT_SPIKE_FIFO_DEPTH_CFG),
        .MEM_STYLE_HINT       (TILE_BANK_MEM_STYLE)
    ) u_fanout_bank (
        .clk(clk), .rst_n(rst_n), .ena(ena), .graph_state_clear(tile_graph_state_clear_r),
        .enqueue_valid(1'b0), .enqueue_data('0), .enqueue_ready(fo_enq_ready_w),
        .dequeue_ready(1'b0), .dequeue_valid(fo_deq_valid_w), .dequeue_data(fo_deq_data_w),
        .mem_stall(fo_stall_w)
    );

    // ── Compute core (single worker, WORKER_CORES_PER_TILE=1) ─────────────────
    // Worker start payload
    neuron_worker_start_t w_start_payload_c;
    assign w_start_payload_c.logical_idx = start_dispatch_logical_idx_r;
    assign w_start_payload_c.ctx         = ctx_read_a_ctx_c;
    assign w_start_payload_c.in_event    = worker_start_event_c;
    assign w_start_payload_c.ucode_ptr   = state_dispatch_ucode_ptr_c;
    assign w_start_payload_c.ucode_len   = state_dispatch_ucode_len_c;
    assign worker_start_if.valid          = worker_start_valid_c[0];
    assign worker_start_if.rv_payload     = w_start_payload_c;
    assign worker_start_ready_c[0]        = worker_start_if.ready;

    // Ucode fetch: core drives valid+payload; tile drives ready=1
    assign worker_ucode_read_valid_c[0]          = ucode_read_if.valid;
    neuron_ucode_req_t w_ucode_req_c;
    assign w_ucode_req_c = neuron_ucode_req_t'(ucode_read_if.rv_payload);
    assign worker_ucode_read_logical_idx_c[0]    = w_ucode_req_c.logical_idx;
    assign worker_ucode_read_word_index_c[0]     = w_ucode_req_c.word_index;
    assign ucode_read_if.ready = 1'b1;

    // Ucode response: tile drives valid+payload; core drives ready internally
    assign ucode_rsp_if.valid      = worker_ucode_read_word_valid_c[0];
    assign ucode_rsp_if.rv_payload = NEURON_UCODE_RSP_W'(worker_ucode_read_word_c[0]);

    // Worker result: core drives valid+payload; tile drives ready
    assign worker_result_valid_c[0]      = worker_result_if.valid;
    assign worker_result_if.ready        = worker_result_ready_c[0];
    neuron_worker_result_t w_result_c;
    assign w_result_c = neuron_worker_result_t'(worker_result_if.rv_payload);
    assign worker_result_logical_idx_c[0] = w_result_c.logical_idx;
    assign worker_result_ctx_c[0]         = w_result_c.ctx;
    assign worker_result_emit_c[0]        = w_result_c.emit;

    (* keep_hierarchy = "no" *) neuron_compute_core #(
        .LOGICAL_IDX_W(NEURON_IDX_W)
    ) u_compute (
        .clk              (clk),
        .rst_n            (rst_n),
        .ena              (ena),
        .graph_state_clear(tile_graph_state_clear_r),
        .worker_start (worker_start_if),
        .ucode_read   (ucode_read_if),
        .ucode_rsp    (ucode_rsp_if),
        .worker_result(worker_result_if)
    );





    // start_dispatch_*_r, logical_event_inflight_r, ready FIFO, worker RR,
    // worker_has_fanout_r register reset/update all live inside
    // u_dispatch_scheduler.
    // Ingress shift/classify regs live in u_ingress.
    // host_*/dump_* regs live in u_host_io.
    // fanout_*_r / fanout_queue_*_r / route_fanout_read_pending_r live in
    // u_fanout_executor.

    // Debug counter: increments every cycle where the ctrl-bank mux would
    // fire a CSR_CTRL write.  Snapshot reads this to confirm loader packets
    // actually reach the ctrl_write_en signal.
    // tile_graph_state_clear_r pipeline flop.  Async-reset + synchronous
    // clear to match the reset style of the debug counters below, but
    // timing-wise this is just a single flop between the ingress decode
    // and all ~290 downstream consumers.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tile_graph_state_clear_r <= 1'b0;
        end else if (ena) begin
            tile_graph_state_clear_r <= tile_graph_state_clear_c;
        end
    end

`ifdef FORMAL
    assign tile_graph_state_clear_r_obs = tile_graph_state_clear_r;
`endif

endmodule

`default_nettype wire
