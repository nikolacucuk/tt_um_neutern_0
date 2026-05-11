// tile_pkg.sv
// ─────────────────────────────────────────────────────────────────────────────
// Architectural parameter package — single source of truth for all field
// widths used in message/flit structures, tile-internal payload types, and
// neuron execution state.
//
// Usage
// ─────
// tile_flit_types.vh imports this package unconditionally:
//
//   import tile_pkg::*;
//
// Because both headers are included at compilation-unit scope (before any
// module keyword), the import lands at compilation-unit scope and the
// parameter names are visible everywhere those headers are included.
// Additional explicit `import tile_pkg::*` inside a module is harmless.
//
// Dependency order
// ────────────────
// This file MUST be compiled before any file that includes tile_flit_types.vh.
// In FuseSoC, list tile_pkg.sv in a fileset that appears
// before the include_headers fileset in base.core.
//
// Content policy
// ──────────────
// Parameters only — no typedefs, no functions, no interfaces.
// Derived structs that use these parameters live in tile_flit_types.vh so that
// the struct layout is visible to include-file
// consumers without requiring a package import at the module level.
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │  tile_pkg params (this file)                                      │
//  │          ↓ import tile_pkg::*                                     │
//  │  tile_flit_types.vh ─ message_packet_t, tile_spike_t, tile_cfg_t, │
//  │                       flit_header_t, message_packet_min_t,         │
//  │                       tile_event_t, neuron_exec_ctx_t,             │
//  │                       neuron_worker_event_t,                       │
//  │                       neuron_weight_commit_t, neuron_emit_t        │
//  │          ↓ `include                                                 │
//  │  tile_top, tile_fanout_executor, neuron_compute_core, ...          │
//  └─────────────────────────────────────────────────────────────────────┘
// ─────────────────────────────────────────────────────────────────────────────

package tile_pkg;

    // ── Flit transport header constants ─────────────────────────────────────
    //
    // These constants define the host/mesh flit framing contract.  They are
    // consumed by tile_flit_types.vh and by router code that classifies traffic into
    // command-vs-data planes and virtual channels.
    //
    // FLIT_WIDTH
    //   Total physical flit payload width used on the mesh datapath.
    //
    // FLIT_MAX_BODY_FLITS
    //   Maximum number of body flits that may follow a header flit.  The header
    //   length field is 4 bits today, so the largest encodable body count is 15.
    //
    // FLIT_PLANE_W / FLIT_CLASS_W / FLIT_LENGTH_W / FLIT_TAG_W
    //   Canonical bit widths for the flit header fields.  These are kept in the
    //   package so header structs and any decoding logic stay synchronized.
    //
    // FLIT_INLINE_PAYLOAD_W
    //   Inline payload space carried directly in the header flit.
    localparam int unsigned FLIT_WIDTH           = 64;
    localparam logic [3:0]  FLIT_MAX_BODY_FLITS  = 4'd15;
    localparam int unsigned FLIT_PLANE_W         = 2;
    localparam int unsigned FLIT_CLASS_W         = 4;
    localparam int unsigned FLIT_LENGTH_W        = 4;
    localparam int unsigned FLIT_TAG_W           = 3;
    localparam int unsigned FLIT_INLINE_PAYLOAD_W = 16;

    // ── Flit plane encodings ────────────────────────────────────────────────
    //
    // Plane selection partitions mesh traffic into command and data classes.
    // The current implementation only actively uses CMD and DATA.  The reserved
    // encodings are kept stable so traces, manifests, and future protocol growth
    // do not silently reassign existing values.
    localparam logic [FLIT_PLANE_W-1:0] PLANE_CMD        = 2'd0;
    localparam logic [FLIT_PLANE_W-1:0] PLANE_DATA       = 2'd1;
    localparam logic [FLIT_PLANE_W-1:0] PLANE_RSVD_CTRL  = 2'd2;
    localparam logic [FLIT_PLANE_W-1:0] PLANE_RSVD_SPARE = 2'd3;

    // ── Command-plane class IDs ─────────────────────────────────────────────
    //
    // These 4-bit class IDs are carried in flit_header_t.class_id when the flit
    // plane is PLANE_CMD.  They define the high-level command namespace routed
    // through the mesh packet core.
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_CSR_REQ   = 4'h0;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_CSR_RSP   = 4'h1;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_STATUS    = 4'h2;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_PING      = 4'h3;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_PONG      = 4'h4;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_HWINFO    = 4'h5;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_MANIFEST  = 4'h6;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_BUNDLE    = 4'h7;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_PROG      = 4'h8;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_TRACE     = 4'h9;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_TELEMETRY = 4'hA;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_TRANSPORT = 4'hB;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_ROUTE     = 4'hC;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_RSVD_0    = 4'hD;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_RSVD_1    = 4'hE;
    localparam logic [FLIT_CLASS_W-1:0] CMD_CLASS_RSVD_2    = 4'hF;

    // ── Data-plane class IDs ────────────────────────────────────────────────
    //
    // These 4-bit class IDs are carried in flit_header_t.class_id when the flit
    // plane is PLANE_DATA.  They describe the data payload interpretation for a
    // body-flit stream or inline payload.
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_SPIKE    = 4'h0;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_SCALAR8  = 4'h1;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_SCALAR16 = 4'h2;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_VECTOR8  = 4'h3;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_VECTOR16 = 4'h4;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_INPUT    = 4'h5;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_OUTPUT   = 4'h6;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_RSVD_0   = 4'h7;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_RSVD_1   = 4'h8;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_RSVD_2   = 4'h9;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_RSVD_3   = 4'hA;
    localparam logic [FLIT_CLASS_W-1:0] DATA_CLASS_RSVD_4   = 4'hB;

    // ── Flit virtual-channel assignments ────────────────────────────────────
    //
    // The mesh currently exposes three maintained VCs:
    //   VC_CMD_REQ  — command requests
    //   VC_CMD_RSP  — command responses / observability drain
    //   VC_DATA     — data-plane payload traffic
    // The fourth encoding remains reserved for compatibility.
    localparam logic [1:0] VC_CMD_REQ       = 2'd0;
    localparam logic [1:0] VC_CMD_RSP       = 2'd1;
    localparam logic [1:0] VC_DATA          = 2'd2;
    localparam logic [1:0] VC_DATA_RSP_RSVD = 2'd3;
    localparam int unsigned NUM_FLIT_VCS    = 3;

    // ── Mesh / routing coordinates ────────────────────────────────────────────
    //
    // TILE_COORD_W  bits for a tile X or Y address in message packets and spike
    //   payloads.   5 b supports up to 32 tiles per axis.  Matching field in
    //   message_packet_t.dst_x/y/src_x/y and tile_spike_t.tile_x/y.
    //
    // FLIT_COORD_W  full NoC routing coordinate width in flit_header_t.
    //   Kept at 8 b to match the host-facing UART/JTAG framing; upper bits are
    //   zero for meshes smaller than 256 nodes per axis.
    localparam int unsigned TILE_COORD_W  = 1;               // → single tile (neutern: 1 tile)
    // Wire-level flit routing coord.  Kept at 8 b so flit_header_t fits in
    // FLIT_WIDTH=64 b exactly (2+4+4+1+1+1+3+8+8+8+8+16 = 64).  Earlier
    // 2*TILE_COORD_W=10 widened the struct to 72 b, which silently lost the
    // upper 2 b of each coord on the 64-b wire and shifted every header
    // field's decode position.
    localparam int unsigned FLIT_COORD_W  = 8;               // up to 256 tiles per axis

    // ── Neuron geometry ───────────────────────────────────────────────────────
    //
    // NEURON_LOCAL_W  bits for one local neuron coordinate (X or Y) within a
    //   tile.  1 b supports up to 2 positions per axis (up to 4 neurons per tile).
    //   Neutern profile: 2x2 = 4 neurons, 1-bit x and y coords (values 0..1).
    //   Used in tile_spike_t.neuron_x/y.
    //
    // NEURON_IDX_W  flat neuron index derived from 2 * NEURON_LOCAL_W.
    //   Used in tile_event_t.neuron_idx.
    localparam int unsigned NEURON_LOCAL_W = 1;                 // → 2 positions per axis (2x2=4 neurons)
    localparam int unsigned NEURON_IDX_W   = 2*NEURON_LOCAL_W;  // flat neuron index = 2 bits

    // ── Core / worker identity ────────────────────────────────────────────────
    //
    // CORE_ID_W  logical core/worker ID field in the message protocol (3 b).
    //   Valid neuron IDs: 0..3 (4 neurons).  Broadcast sentinel = 3'b111 = 7
    //   is distinct from any valid neuron ID (0-3), avoiding false-broadcast.
    //   Neutern: 3 bits (was 4) saves 1 bit per field; broadcast = 3'h7.
    localparam int unsigned CORE_ID_W     = 3;

    // ── Synaptic signal widths ────────────────────────────────────────────────
    //
    // WEIGHT_W    signed synaptic weight (two's-complement, 4 b = range -8..+7).
    //   Neutern profile: 4-bit weight matching neutern_spike_t.weight.
    //   Used in tile_spike_t.weight, tile_event_t.weight, etc.
    //
    // SID_W       tile-internal synapse stream ID (minimal for neutern).
    //   Used in neuron_exec_ctx_t.last_sid, neuron_worker_event_t.sid, etc.
    //
    // MSG_SID_W   message-protocol SID field (minimal, unused in neutern path).
    localparam int unsigned WEIGHT_W      = 4;   // 4-bit signed weight (neutern_spike_t)
    localparam int unsigned SID_W         = 5;   // tile-internal stream ID (unchanged)
    localparam int unsigned MSG_SID_W     = 1;   // protocol-level SID (minimal for neutern)

    // ── Temporal / event fields ───────────────────────────────────────────────
    //
    // EVENT_TIME_W  event timestamp width.  Neutern does not use event-time
    //   ordering; set to 1 (minimum valid) so internal structs stay syntactically
    //   valid while consuming negligible area.
    //
    // TAG_W  lightweight tag (minimal for neutern; 1 b = 2 tag states).
    localparam int unsigned EVENT_TIME_W  = 1;   // unused in neutern; minimum valid
    localparam int unsigned TAG_W         = 1;   // minimal tag

    // Derived: tile_queue_event_t (weight + event_time + neuron_y + neuron_x)
    // + TILE_COORD_W tile_x + TILE_COORD_W tile_y in tile_fanout_spike_t.
    // Used in port width declarations to avoid $bits(typedef) Yosys parse errors.
    localparam int unsigned TILE_QUEUE_EVENT_W = WEIGHT_W + EVENT_TIME_W + 2 * NEURON_LOCAL_W;
    localparam int unsigned FANOUT_SPIKE_W     = TILE_QUEUE_EVENT_W + 2 * TILE_COORD_W;  // tile_x + tile_y

    // ── Message protocol frame fields ─────────────────────────────────────────
    //
    // MSG_KIND_W    primary 4-bit message opcode (MSG_WRITE … MSG_MCAST).
    // CMD_KIND_W    3-bit command sub-opcode namespace (CMD_CSR … CMD_DUMP).
    // MSG_ADDR_W    4-bit register/CSR address within a command domain.
    // PROG_IDX_W    5-bit programming word index for MSG_PROG_* sequences.
    // DATA_W        4-bit primary data nibble (neutern: data_hi removed).
    // WEIGHT_CODE_W neutern: 4-bit field matches WEIGHT_W (carries actual weight).
    // META_W        8-bit per-message metadata / flags byte.
    localparam int unsigned MSG_KIND_W    = 4;
    localparam int unsigned CMD_KIND_W    = 3;
    localparam int unsigned MSG_ADDR_W    = 4;
    localparam int unsigned PROG_IDX_W    = 5;
    localparam int unsigned DATA_W        = 4;   // 4-bit data field (neutern lean: no data_hi)
    localparam int unsigned WEIGHT_CODE_W = WEIGHT_W;  // = 4; now carries actual weight
    localparam int unsigned META_W        = 8;

    // ── Message command domains ─────────────────────────────────────────────
    //
    // cmd_kind namespaces the register and memory operations carried by
    // MSG_WRITE / MSG_READ style packets.
    localparam logic [CMD_KIND_W-1:0] CMD_CSR     = 3'b000;
    localparam logic [CMD_KIND_W-1:0] CMD_WEIGHT  = 3'b001;
    localparam logic [CMD_KIND_W-1:0] CMD_UCODE   = 3'b010;
    localparam logic [CMD_KIND_W-1:0] CMD_MESSAGE = 3'b011;
    localparam logic [CMD_KIND_W-1:0] CMD_SYNAPSE = 3'b100;
    localparam logic [CMD_KIND_W-1:0] CMD_DEBUG   = 3'b101;
    localparam logic [CMD_KIND_W-1:0] CMD_ROUTE   = 3'b110;
    localparam logic [CMD_KIND_W-1:0] CMD_DUMP    = 3'b111;

    // ── Neuron microcode opcodes ────────────────────────────────────────────
    //
    // These encodings define the current maintained ISA implemented by the
    // logical-neuron execution lane.
    localparam int unsigned NEURON_OP_W   = 6;  // forward-declared here; also appears in the width section below
    localparam logic [NEURON_OP_W-1:0] OP_LDI         = 6'd0;
    localparam logic [NEURON_OP_W-1:0] OP_RECV        = 6'd1;
    localparam logic [NEURON_OP_W-1:0] OP_ACCUM_W     = 6'd2;
    // 6'd3 reserved (was OP_LEAK — removed; no timestamps in neutern)
    localparam logic [NEURON_OP_W-1:0] OP_INTEG       = 6'd4;
    localparam logic [NEURON_OP_W-1:0] OP_SPIKE_IF_GE = 6'd5;
    localparam logic [NEURON_OP_W-1:0] OP_RESET       = 6'd6;
    // 6'd7 reserved (was OP_REFRACT — removed with rf[3])
    localparam logic [NEURON_OP_W-1:0] OP_EMIT        = 6'd8;
    // 6'd9, 6'd10 reserved (were OP_TDEC/OP_TINC — removed; no timestamps)
    localparam logic [NEURON_OP_W-1:0] OP_STDP_LITE   = 6'd11;

    // ── Message opcodes ─────────────────────────────────────────────────────
    //
    // kind is the primary packet opcode understood across the host, gateway,
    // mesh, and tile blocks.
    localparam logic [MSG_KIND_W-1:0] MSG_WRITE      = 4'd0;
    localparam logic [MSG_KIND_W-1:0] MSG_READ       = 4'd1;
    localparam logic [MSG_KIND_W-1:0] MSG_READ_RSP   = 4'd2;
    localparam logic [MSG_KIND_W-1:0] MSG_PROG_BEGIN = 4'd3;
    localparam logic [MSG_KIND_W-1:0] MSG_PROG_WORD  = 4'd4;
    localparam logic [MSG_KIND_W-1:0] MSG_PROG_END   = 4'd5;
    localparam logic [MSG_KIND_W-1:0] MSG_STATUS     = 4'd6;
    localparam logic [MSG_KIND_W-1:0] MSG_OUTPUT     = 4'd7;
    localparam logic [MSG_KIND_W-1:0] MSG_PING       = 4'd8;
    localparam logic [MSG_KIND_W-1:0] MSG_PONG       = 4'd9;
    localparam logic [MSG_KIND_W-1:0] MSG_INPUT      = 4'd10;
    localparam logic [MSG_KIND_W-1:0] MSG_SPIKE      = 4'd11;
    localparam logic [MSG_KIND_W-1:0] MSG_TRACE      = 4'd12;
    localparam logic [MSG_KIND_W-1:0] MSG_TELEMETRY  = 4'd13;
    localparam logic [MSG_KIND_W-1:0] MSG_MCAST      = 4'd14;

    // NUM_VCS is the logical message-protocol request/response split used by
    // the packet routers.  This is distinct from NUM_FLIT_VCS above because the
    // message protocol currently exposes only request vs response classes.
    localparam int unsigned NUM_VCS = 2;

    // ── Hardware-info query keys and fixed response values ──────────────────
    //
    // HWINFO_* selectors choose the sub-field returned by a HWINFO read.  The
    // *_MAGIC / *_PROTOCOL / *_FEATURE_* constants define the maintained wire
    // protocol identity exposed to software.
    localparam logic [3:0] HWINFO_MAGIC    = 4'hB;
    localparam logic [3:0] HWINFO_PROTOCOL = 4'hC;
    localparam logic [3:0] HWINFO_MESH     = 4'hD;
    localparam logic [3:0] HWINFO_FEATURES = 4'hE;
    localparam logic [3:0] HWINFO_BUILD    = 4'hF;

    localparam logic [7:0] HWINFO_MAGIC_LO       = 8'h43;
    localparam logic [7:0] HWINFO_MAGIC_HI       = 8'h46;
    localparam logic [7:0] HWINFO_PROTOCOL_MAJOR = 8'h01;
    localparam logic [7:0] HWINFO_PROTOCOL_MINOR = 8'h00;
    localparam logic [7:0] HWINFO_FEATURE_PING   = 8'h01;
    localparam logic [7:0] HWINFO_FEATURE_HWINFO = 8'h02;
    localparam logic [7:0] HWINFO_FEATURE_UART   = 8'h04;
    localparam logic [7:0] HWINFO_FEATURE_JTAG   = 8'h08;
    localparam logic [7:0] HWINFO_FEATURE_DEBUG  = 8'h10;
    localparam logic [7:0] HWINFO_FEATURE_TRACE  = 8'h20;
    localparam logic [7:0] HWINFO_FEATURE_ETH    = 8'h40;
    localparam logic [7:0] HWINFO_BUILD_ID       = 8'h00;
    localparam logic [7:0] PONG_META_SOC         = 8'h01;
    localparam logic [7:0] PONG_META_TILE        = 8'h20;
    localparam logic [7:0] TILE_HEALTH_ALIVE     = 8'h01;
    localparam logic [7:0] TILE_HEALTH_ENABLED   = 8'h02;

    // ── Packet metadata plane bits ──────────────────────────────────────────
    //
    // message_packet_t.meta[7:6] carries the packet plane classification.  These
    // values intentionally mirror the flit-plane numbering used by the mesh.
    localparam logic [1:0] PKT_PLANE_CMD        = 2'd0;
    localparam logic [1:0] PKT_PLANE_DATA       = 2'd1;
    localparam logic [1:0] PKT_PLANE_RSVD_CTRL  = 2'd2;
    localparam logic [1:0] PKT_PLANE_RSVD_SPARE = 2'd3;

    // ── Message status / sentinel IDs ───────────────────────────────────────
    //
    // MSG_STATUS_* values are returned in MSG_STATUS packets.  MESSAGE_* values
    // below define reserved coordinate/core IDs used by bundle loading and
    // broadcast targeting.
    localparam logic [7:0] MSG_STATUS_OK        = 8'h00;
    localparam logic [7:0] MSG_STATUS_BAD_ADDR  = 8'h01;
    localparam logic [7:0] MSG_STATUS_BAD_KIND  = 8'h02;
    localparam logic [7:0] MSG_STATUS_BAD_CORE  = 8'h03;
    localparam logic [7:0] MSG_STATUS_NOT_LOCAL = 8'h04;

    // MESSAGE_COORD_LOADER tags packets emitted by the host gateway's loader
    // and clear-broadcaster paths. It is assigned into message_packet_t.src_x/y,
    // which are TILE_COORD_W bits wide; declaring the constant at the field
    // width avoids silent truncation of the older 8'hFE value (which used to
    // collapse to 5'h1E on the TILE_COORD_W=5 branch).  All-ones is unreachable
    // for any legal mesh coord so it remains an unambiguous sentinel.
    localparam logic [TILE_COORD_W-1:0] MESSAGE_COORD_LOADER = '1;  // all-ones sentinel (1'b1)
    localparam logic [CORE_ID_W-1:0]   MESSAGE_CORE_BCAST   = '1;  // all-ones = 3'h7 (> max neuron ID 3)
    localparam logic [CORE_ID_W-1:0]   MESSAGE_CORE_ID_V1   = '0;

    // ── Tile CSR addresses ──────────────────────────────────────────────────
    //
    // These are the maintained CSR offsets within CMD_CSR.
    localparam logic [MSG_ADDR_W-1:0] CSR_CTRL          = 4'h0;
    localparam logic [MSG_ADDR_W-1:0] CSR_UCODE_PTR     = 4'h1;
    localparam logic [MSG_ADDR_W-1:0] CSR_UCODE_LEN     = 4'h2;
    localparam logic [MSG_ADDR_W-1:0] CSR_VEC_BASE_01   = 4'h3;
    localparam logic [MSG_ADDR_W-1:0] CSR_VEC_BASE_23   = 4'h4;
    localparam logic [MSG_ADDR_W-1:0] CSR_INIT_VI       = 4'h5;
    localparam logic [MSG_ADDR_W-1:0] CSR_INIT_TR       = 4'h6;
    localparam logic [MSG_ADDR_W-1:0] CSR_INIT_T01      = 4'h7;
    localparam logic [MSG_ADDR_W-1:0] CSR_INIT_WAUX     = 4'h8;
    localparam logic [MSG_ADDR_W-1:0] CSR_NEURON_META   = 4'h9;
    localparam logic [MSG_ADDR_W-1:0] CSR_RESET_TRIGGER = 4'hA;

    // CSR bit definitions are kept here so software-visible programming fields
    // and RTL decoding logic derive from the same source of truth.
    localparam int unsigned CSR_CTRL_SOFT_RESET_BIT          = 0;
    localparam int unsigned CSR_CTRL_CLEAR_OUT_BIT           = 1;
    localparam int unsigned CSR_CTRL_CLEAR_FIFO_BIT          = 2;
    localparam int unsigned CSR_CTRL_HOST_EGRESS_BIT         = 3;
    localparam int unsigned CSR_CTRL_NOC_EGRESS_BIT          = 4;
    localparam int unsigned CSR_RESET_TRIGGER_SOFT_RESET_BIT = 0;

    // ── Trace / manifest constants ──────────────────────────────────────────
    //
    // TRACE_OBS_* identifies the observed path in trace packets.  MANIFEST_* is
    // the fixed image header used by the maintained bundle/manifest flow.
    localparam logic [7:0]  TRACE_OBS_HOST_TO_TILE = 8'h01;
    localparam logic [7:0]  TRACE_OBS_TILE_TO_TILE = 8'h02;
    localparam logic [7:0]  TRACE_OBS_TILE_TO_HOST = 8'h03;
    localparam logic [15:0] MANIFEST_MAGIC         = 16'h4346;
    localparam logic [15:0] MANIFEST_VERSION       = 16'h0001;
    localparam logic [15:0] MANIFEST_FLAG_VALID    = 16'h0001;

    // ── Neuron execution ISA ──────────────────────────────────────────────────
    //
    // RF_COUNT    number of 8-bit execution registers per neuron worker (4).
    // RF_REG_W    width of each register (8 b).
    // RF_FLAT_W   total flattened register-file width (RF_COUNT × RF_REG_W = 32).
    //   Used in neuron_exec_ctx_t.rf_state_flat.
    //
    // NEURON_OP_W  microcode opcode width (6 b covers 64 opcodes).
    //   Used in tile_cfg_t.neuron_op.
    //
    // CFG_DATA_W   config immediate / data byte in tile_cfg_t.
    // RF narrowed to 3 registers: rf[0]=vm, rf[1]=accum, rf[2]=threshold.
    // rf[3]=refractory removed along with OP_REFRACT (no timestamp support).
    // RF_REG_W=4: registers are 4-bit (matching WEIGHT_W) for area minimisation.
    localparam int unsigned RF_COUNT      = 3;
    localparam int unsigned RF_REG_W      = 4;   // 4-bit per register (matches WEIGHT_W)
    localparam int unsigned RF_FLAT_W     = RF_COUNT * RF_REG_W;  // = 12
    // NEURON_OP_W declared earlier (before first use); see top of opcode section.
    localparam int unsigned CFG_DATA_W    = 8;

    // ── Derived composite widths ──────────────────────────────────────────────
    //
    // TILE_SPIKE_W  total bit-width of tile_spike_t (neutern = 9 b):
    //   neuron_x(1) + neuron_y(1) + tile_x(1) + tile_y(1) + weight(4) + event_time(1) = 9
    localparam int unsigned TILE_SPIKE_W =
        NEURON_LOCAL_W * 2 + TILE_COORD_W * 2 + WEIGHT_W + EVENT_TIME_W;

    // TILE_CFG_RSVD_W  reserved padding field in tile_cfg_t.
    //   tile_cfg_t and tile_spike_t share a packed union in message_packet_min_t,
    //   so they must be the same total width (TILE_SPIKE_W).  The reserved field
    //   absorbs the difference:
    //     TILE_SPIKE_W − NEURON_OP_W − CFG_DATA_W = 34 − 6 − 8 = 20
    localparam int unsigned TILE_CFG_RSVD_W =
        TILE_SPIKE_W - NEURON_OP_W - CFG_DATA_W;                  // = 20

    // ── $bits() replacement constants for Yosys SV compatibility ─────────────
    // Yosys cannot evaluate $bits(typedef_name) in module contexts.
    // These constants allow replacing $bits(T) with a package constant.

    // tile_event_t: neuron_idx(10) + weight(8) + tag(2) + event_time(6) = 26
    localparam int unsigned TILE_EVENT_W = NEURON_IDX_W + WEIGHT_W + TAG_W + EVENT_TIME_W;

    // message_packet_min_t: payload_is_spike(1) + TILE_SPIKE_W(34) = 35
    localparam int unsigned MESSAGE_PKT_MIN_W = 1 + TILE_SPIKE_W;

    // neuron_exec_ctx_t: rf_state_flat(RF_FLAT_W=12) + last_tag(TAG_W=1)
    //                    + cmp_ge(1) + cmp_eq(1) + spike_flag(1) = 16
    //   EVENT_TIME_W removed: neutern does not use temporal context.
    localparam int unsigned NEURON_EXEC_CTX_W = RF_FLAT_W + TAG_W + 3;

    // neuron_worker_event_t: tag(TAG_W=1) + weight(WEIGHT_W=4) = 5  (spike_time removed)
    localparam int unsigned NEURON_WORKER_EVENT_W = TAG_W + WEIGHT_W;

    // neuron_worker_start_t: logical_idx(10) + ctx(48) + in_event(21) + 2*PROG_IDX_W(5) = 89
    localparam int unsigned NEURON_WORKER_START_W =
        NEURON_IDX_W + NEURON_EXEC_CTX_W + NEURON_WORKER_EVENT_W + 2 * PROG_IDX_W;

    // neuron_ucode_req_t: logical_idx(10) + word_index(5) = 15
    localparam int unsigned NEURON_UCODE_REQ_W = NEURON_IDX_W + PROG_IDX_W;

    // neuron_ucode_rsp_t: word[11:0] = 12  (compact: {weight_nibble, addr_nibble, data_nibble})
    // data_hi removed; ucode instruction word packed as {weight[3:0], addr[3:0], data[3:0]}.
    // Instruction decode: [11:7]=op(5b), [6:4]=rd(3b), [3]=sign, [2:0]=k(3b).
    localparam int unsigned NEURON_UCODE_RSP_W = 12;

    // neuron_weight_commit_t: valid(1) + sid(5) + value(8) = 14
    localparam int unsigned NEURON_WEIGHT_COMMIT_W = 1 + SID_W + WEIGHT_W;

    // neuron_emit_t: valid(1) + data(DATA_W=4) = 5  (event_time removed)
    localparam int unsigned NEURON_EMIT_W = 1 + DATA_W;

    // neuron_worker_result_t: logical_idx(10) + ctx(48) + weight_commit(14) + emit(15) = 87
    localparam int unsigned NEURON_WORKER_RESULT_W =
        NEURON_IDX_W + NEURON_EXEC_CTX_W + NEURON_EMIT_W;  // weight_commit removed (STDP disabled)

    // ── rv_if interface payload widths ────────────────────────────────────────
    //
    // These constants replace $bits(typedef_name) for Yosys compatibility and
    // allow parameterized rv_if WIDTH declarations referencing struct sizes.
    //
    // fanout_write_req_t:
    //   mask_valid(1) + weight(WEIGHT_W=4) + dst_x(TILE_COORD_W=1) + dst_y(TILE_COORD_W=1)
    //   + core_id(CORE_ID_W=3) + meta(2) + valid(1) + weight_only(1) + route_only(1) = 15
    localparam int unsigned FANOUT_WRITE_REQ_W = 1 + WEIGHT_W + TILE_COORD_W + TILE_COORD_W +
                                                 CORE_ID_W + 2 + 1 + 1 + 1;
    //
    // fanout_read_rsp_t:
    //   meta_valid(1) + weight(WEIGHT_W=4) + dst_x(TILE_COORD_W=1) + dst_y(TILE_COORD_W=1)
    //   + core_id(CORE_ID_W=3) = 10
    localparam int unsigned FANOUT_READ_RSP_W = 1 + WEIGHT_W + TILE_COORD_W + TILE_COORD_W + CORE_ID_W;
    //
    // context_commit_t: idx(NEURON_IDX_W) + ctx(NEURON_EXEC_CTX_W)
    localparam int unsigned CONTEXT_COMMIT_W = NEURON_IDX_W + NEURON_EXEC_CTX_W;

endpackage
