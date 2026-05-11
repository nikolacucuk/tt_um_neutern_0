`ifndef TILE_FLIT_TYPES_VH
`define TILE_FLIT_TYPES_VH

import tile_pkg::*;

// -----------------------------------------------------------------------------
// tile_flit_types.vh - shared protocol and tile-local payload structs
//
// This header consolidates the maintained shared packet/flit/type definitions
// that are consumed across the mesh, tile, host-gateway, and worker paths.
//
// All protocol constants and width definitions now live in tile_pkg.sv.
// This file contains only typedefs and pure helper functions that depend on
// those package constants.
// -----------------------------------------------------------------------------

function automatic logic [META_W-1:0] packet_meta_with_plane(
    input logic [META_W-1:0] meta,
    input logic [1:0]        plane
);
    begin
        packet_meta_with_plane = {plane, meta[5:0]};
    end
endfunction

typedef struct packed {
    logic [MSG_KIND_W-1:0]    kind;
    logic [CMD_KIND_W-1:0]    cmd_kind;
    logic                     broadcast;
    logic [TILE_COORD_W-1:0]  dst_x;
    logic [TILE_COORD_W-1:0]  dst_y;
    // src_x / src_y removed: single-tile neutern profile - no mesh routing
    logic [CORE_ID_W-1:0]     core_id;
    logic [PROG_IDX_W-1:0]    prog_index;
    logic [MSG_ADDR_W-1:0]    addr;
    logic [DATA_W-1:0]        data;
    // data_hi removed: DATA_W narrowed to 4 b; ucode high nibble now in weight field
    logic [MSG_SID_W-1:0]     sid;
    logic [TAG_W-1:0]         tag;
    // event_time removed: neutern does not use temporal event ordering
    logic signed [WEIGHT_W-1:0] weight;  // was weight_code; now full WEIGHT_W-bit weight
    logic [META_W-1:0]        meta;
} message_packet_t;

// Local event-queue payload: 24 bits packed into exactly three byte lanes.
// Lane[15:0]  = { event_time[5:0], neuron_y[4:0], neuron_x[4:0] }
// Lane[23:16] = weight[7:0]
typedef struct packed {
    logic signed [WEIGHT_W-1:0] weight;
    logic [EVENT_TIME_W-1:0]    event_time;
    logic [NEURON_LOCAL_W-1:0]  neuron_y;
    logic [NEURON_LOCAL_W-1:0]  neuron_x;
} tile_queue_event_t;

// Routed spike payload: 2-D local neuron coordinates plus full tile coordinates.
// This is the general mesh/protocol shape and remains intentionally wider than
// tile-local FIFO payloads so it can represent larger meshes.
typedef struct packed {
    tile_queue_event_t          queue_spike;
    logic [TILE_COORD_W-1:0]    tile_x;
    logic [TILE_COORD_W-1:0]    tile_y;
} tile_spike_t;

typedef tile_spike_t tile_out_spike_t;

// Compact fanout staging payload for the ASIC tile profile.  Tile coordinates
// use TILE_COORD_W bits matching the protocol coordinate width.
typedef struct packed {
    tile_queue_event_t          queue_spike;
    logic [TILE_COORD_W-1:0]    tile_y;
    logic [TILE_COORD_W-1:0]    tile_x;
} tile_fanout_spike_t;

// (1 + TILE_SPIKE_W)-bit shared payload bus: payload_is_spike=1 - spike is valid.
typedef struct packed {
    logic          payload_is_spike;
    tile_spike_t   payload;
} message_packet_min_t;

// Flattens local (y, x) into the maintained 2-D tile-local neuron index.
function automatic logic [2*NEURON_LOCAL_W-1:0] tile_spike_flat_idx(input tile_spike_t spike);
    begin
        tile_spike_flat_idx = {
            spike.queue_spike.neuron_y,
            spike.queue_spike.neuron_x
        };
    end
endfunction

// Legacy compact-packet helper: preserves the low 8 bits of the flattened
// local neuron index for the message_packet_t.core_id shadow field.
function automatic logic [CORE_ID_W-1:0] tile_spike_flat_core_id(input tile_spike_t spike);
    logic [2*NEURON_LOCAL_W-1:0] flat_idx;
    begin
        flat_idx = tile_spike_flat_idx(spike);
        tile_spike_flat_core_id = CORE_ID_W'(flat_idx);
    end
endfunction

// -----------------------------------------------------------------------------
// tile-local worker / queue payload structs
// -----------------------------------------------------------------------------

// Tile-local worker/event payload: flattened neuron index plus worker tag.
typedef struct packed {
    logic [NEURON_IDX_W-1:0]    neuron_idx;
    logic signed [WEIGHT_W-1:0] weight;
    logic [TAG_W-1:0]           tag;
    logic [EVENT_TIME_W-1:0]    event_time;
} tile_event_t;

typedef struct packed {
    logic [RF_FLAT_W-1:0]       rf_state_flat;
    logic [TAG_W-1:0]           last_tag;
    // last_time removed: neutern does not use timestamps
    logic                       cmp_ge;
    logic                       cmp_eq;
    logic                       spike_flag;
} neuron_exec_ctx_t;

typedef struct packed {
    logic [TAG_W-1:0]           tag;
    // spike_time removed: neutern does not use timestamps
    logic signed [WEIGHT_W-1:0] weight;
} neuron_worker_event_t;

typedef struct packed {
    logic                       valid;
    logic [DATA_W-1:0]          data;
    // event_time removed: neutern does not use timestamps
} neuron_emit_t;

// Ready/valid payload from the tile dispatcher into one compute worker.
// The logical index is package-width so the same payload can cross tile-local
// wrappers; narrower workers cast it down to their configured LOGICAL_IDX_W.
typedef struct packed {
    logic [NEURON_IDX_W-1:0]    logical_idx;
    neuron_exec_ctx_t           ctx;
    neuron_worker_event_t       in_event;
    logic [PROG_IDX_W-1:0]      ucode_ptr;
    logic [PROG_IDX_W-1:0]      ucode_len;
} neuron_worker_start_t;

// Ready/valid payload from a compute worker to the shared ucode bank adapter.
typedef struct packed {
    logic [NEURON_IDX_W-1:0]    logical_idx;
    logic [PROG_IDX_W-1:0]      word_index;
} neuron_ucode_req_t;

// Ready/valid payload returned by the shared ucode bank adapter.
typedef struct packed {
    logic [NEURON_UCODE_RSP_W-1:0]  word;  // 12-bit compact instruction
} neuron_ucode_rsp_t;

// Ready/valid payload returned by one compute worker for commit arbitration.
typedef struct packed {
    logic [NEURON_IDX_W-1:0]    logical_idx;
    neuron_exec_ctx_t           ctx;
    neuron_emit_t               emit;
} neuron_worker_result_t;

// -----------------------------------------------------------------------------
// Fanout pool rv_if interface structs
// -----------------------------------------------------------------------------
//
//   ASCII datapath sketch (fanout pool write/read):
//
//   tile_ingress                     tile_fanout_pool
//   ----------------  rv_if          ------------------
//   -fanout_write.tx------------------fanout_write.rx  -
//   -fanout_write_  -  [addr]        - write_index     -
//   -  addr out    -------------------                 -
//   ----------------                 - read port       -
//                                    -  rv_if          -
//   tile_fanout_executor             - fanout_rsp.tx ------ tile_top demux
//   ----------------  rv_if          -                 -     ---- executor
//   -fanout_rsp.rx  ------------------                 -     ---- host_io
//   ----------------                 ------------------

// Fanout pool write request payload.  rv_if.valid = write_en; table write
// address (fanout_write_addr) is carried as a companion output alongside rv_if.
typedef struct packed {
    logic                            mask_valid;   // pool entry write gating (neuron matched)
    logic signed [WEIGHT_W-1:0]      weight;       // synaptic weight (two's-complement)
    logic [TILE_COORD_W-1:0]         dst_x;        // route: destination tile X
    logic [TILE_COORD_W-1:0]         dst_y;        // route: destination tile Y
    logic [CORE_ID_W-1:0]            core_id;      // route: destination core
    logic [1:0]                      meta;         // metadata: {valid_flag, epoch_override}
    logic                            valid;        // table entry valid flag
    logic                            weight_only;  // partial write: weight lane only
    logic                            route_only;   // partial write: route/meta lanes only
} fanout_write_req_t;  // = FANOUT_WRITE_REQ_W bits

// Fanout pool read response (1-cycle latency after read_en).
typedef struct packed {
    logic                            meta_valid;  // epoch-gated entry-valid flag
    logic signed [WEIGHT_W-1:0]      weight;      // synaptic weight
    logic [TILE_COORD_W-1:0]         dst_x;       // destination tile X
    logic [TILE_COORD_W-1:0]         dst_y;       // destination tile Y
    logic [CORE_ID_W-1:0]            core_id;     // destination core
} fanout_read_rsp_t;  // = FANOUT_READ_RSP_W bits

// Context bank commit payload.  rv_if.valid = context_commit_valid.
// tile_top unpacks this struct to drive the logical_neuron_context_bank ports.
typedef struct packed {
    logic [NEURON_IDX_W-1:0]    idx;  // target neuron index
    neuron_exec_ctx_t           ctx;  // execution context to commit
} context_commit_t;  // = CONTEXT_COMMIT_W (NEURON_IDX_W + NEURON_EXEC_CTX_W)

`endif // TILE_FLIT_TYPES_VH
