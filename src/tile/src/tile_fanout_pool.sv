`default_nettype none

`ifndef YOSYS
import tile_pkg::CORE_ID_W;
import tile_pkg::DATA_W;
import tile_pkg::TILE_COORD_W;
import tile_pkg::WEIGHT_W;
import tile_pkg::fanout_read_rsp_t;
import tile_pkg::fanout_write_req_t;
`endif

// -----------------------------------------------------------------------------
// tile_fanout_pool - FF-backed per-synapse connection table
//
// Stores FANOUT_POOL_DEPTH entries.  Each entry holds the outbound route
// and weight for one synapse.  Entry layout is parameter-driven:
//
//   Byte 0           : weight[7:0]  (signed, two's-complement)
//   Bytes [1..ROUTE_BYTES] : FANOUT_DST_X_W + FANOUT_DST_Y_W + FANOUT_CORE_ID_W
//                            routing bits packed LSB-first (dst_x || dst_y || core_id)
//   Meta             : bit[0]=valid, bit[1]=epoch (graph-state-clear).  When
//                      the route byte has at least two spare bits, meta is
//                      packed into those spare bits instead of allocating a
//                      dedicated byte lane.
//
// Write ports accept [7:0] protocol-width inputs; the lower
// FANOUT_DST_X/Y/CORE_ID_W bits are extracted before packing into the table.
// Read ports return exactly [FANOUT_DST_X/Y/CORE_ID_W-1:0] bits;
// callers zero-extend to protocol width (8 b) when building packet headers.
//
// Address space: one flat pool of FANOUT_POOL_DEPTH entries.
// Per-neuron fanout ranges (ptr+len) live in logical_neuron_state_bank and
// are managed by the runtime; this module is address-agnostic.
//
// Single physical read port - caller (tile_top shared arbiter) pre-arbitrates
// between host/dump reads (port 0) and executor route-walk reads (port 1).
// Read latency: 1 clock cycle.
//
// Write control:
//   write_weight_only = 1 : update weight byte only
//   write_route_only  = 1 : update route/meta byte(s) only
//   neither              : full entry write
//   The epoch bit is stamped automatically from graph_epoch_r at write time.
//
// graph_state_clear epoch mechanism:
//   graph_epoch_r (1 FF) flips on every graph_state_clear assertion.
//   Written entries carry the epoch at write time in meta[1].
//   On reads, read_rsp_meta_valid is gated by both the stored valid bit
//   (meta[0]) AND an epoch match against the CURRENT graph_epoch_r.
//   This invalidates all pre-clear entries without re-writing all rows.
//
// Entry bytes = 1 (weight) + ROUTE_BYTES + META_BYTES
// ROUTE_BYTES = ceil((FANOUT_DST_X_W + FANOUT_DST_Y_W + FANOUT_CORE_ID_W) / 8)
// META_BYTES  = 0 when valid/epoch fit in route-byte padding, else 1
//
//   4x4 mesh, 4 workers (X_W=2, Y_W=2, C_W=2): ROUTE_BYTES=1, META_BYTES=0,
//
//   Default (X_W=8, Y_W=8, C_W=8): ROUTE_BYTES=3, META_BYTES=1,
//     ENTRY_BYTES=5 (backward-compatible)
//
// ASCII schematic:
//
//  write_en ----------------------\
//  write_{weight,dst_x,dst_y,      +--> Write mux --> mem write port
//         core_id,meta} ----------/
//  write_{weight,route}_only -----/
//                                      [FANOUT_POOL_DEPTH x 5B]
//  read_en --------------------------> mem read port
//  read_index ----------------------->
//          1 cycle --> read_rsp_{valid,weight,dst_x,dst_y,core_id,meta_valid}
// -----------------------------------------------------------------------------
(* keep_hierarchy = "no" *)
module tile_fanout_pool #(
    parameter int unsigned FANOUT_POOL_DEPTH  = 256,
    parameter int unsigned MEM_STYLE_HINT     = 0,
    // Routing coordinate widths - set to actual mesh address widths to shrink
    // the SRAM entry from 5 fixed bytes down to (1 + ROUTE_BYTES + META_BYTES)
    // bytes.  The 4x4/4-worker profile uses 2/2/2, packing valid/epoch into
    // route-byte padding for a 2-byte row.
    parameter int unsigned FANOUT_DST_X_W    = 8,
    parameter int unsigned FANOUT_DST_Y_W    = 8,
    parameter int unsigned FANOUT_CORE_ID_W  = 8
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,

    // Write port — structured rv_if carrying fanout_write_req_t payload.
    // rv_if.valid = write_en; write_index carries the table address separately.
    rv_if.rx                                                                     fanout_write_if,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] write_index,

    // Single pre-arbitrated read port
    input  wire                                                           read_en,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] read_index,

    // Read response (1-cycle latency) — structured rv_if carrying fanout_read_rsp_t.
    // rv_if.valid = read_rsp_valid (pipelined read_en).
    rv_if.tx                       fanout_rsp,

    output logic                  mem_stall
);

    // Local parameters
    localparam int unsigned FANOUT_ADDR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);

    // Route bits: pack dst_x || dst_y || core_id (LSB-first) into minimal
    // bytes.  A minimum of 1 byte is always allocated.
    localparam int unsigned ROUTE_BITS  = FANOUT_DST_X_W + FANOUT_DST_Y_W + FANOUT_CORE_ID_W;
    localparam int unsigned ROUTE_BYTES = (ROUTE_BITS == 0) ? 1 : (ROUTE_BITS + 7) / 8;

    // Entry row layout:
    //   Byte 0           : weight[7:0]  (signed)
    //   Bytes [1..ROUTE_BYTES] : routing bits packed LSB-first
    //     bits [FANOUT_DST_X_W-1:0]                         = dst_x
    //     bits [FANOUT_DST_X_W+FANOUT_DST_Y_W-1:FANOUT_DST_X_W] = dst_y
    //     bits [ROUTE_BITS-1:FANOUT_DST_X_W+FANOUT_DST_Y_W] = core_id
    //     bits [ROUTE_BITS +: 2] = {epoch, valid} when spare route-byte bits exist
    //   Optional meta byte     : bit[0]=valid, bit[1]=epoch, bits[7:2]=reserved
    localparam int unsigned ROW_OFF_WEIGHT = 0;
    localparam int unsigned ROW_OFF_ROUTE  = 1;

    localparam bit PACK_META_IN_ROUTE = ((ROUTE_BITS + 2) <= (ROUTE_BYTES * 8));
    localparam int unsigned META_BYTES = PACK_META_IN_ROUTE ? 0 : 1;
    localparam int unsigned ROW_OFF_META = PACK_META_IN_ROUTE ? ROW_OFF_ROUTE : (1 + ROUTE_BYTES);

    // Bit offsets of each field within the packed ROUTE_BYTES region
    localparam int unsigned ROFF_DST_X   = 0;
    localparam int unsigned ROFF_DST_Y   = FANOUT_DST_X_W;
    localparam int unsigned ROFF_CORE_ID = FANOUT_DST_X_W + FANOUT_DST_Y_W;

    localparam int unsigned ENTRY_BYTES = 1 + ROUTE_BYTES + META_BYTES;
    localparam int unsigned ENTRY_W     = ENTRY_BYTES * 8;

    localparam int unsigned ROUTE_META_VALID_BIT = PACK_META_IN_ROUTE ? ROUTE_BITS : 0;
    localparam int unsigned ROUTE_META_EPOCH_BIT = PACK_META_IN_ROUTE ? (ROUTE_BITS + 1) : 1;

    function automatic logic byte_fits_width(
        input logic [7:0] value,
        input int unsigned width
    );
        begin
            byte_fits_width = (width >= 8) || ((value >> width) == 8'd0);
        end
    endfunction

    // Unpack fanout_write_if rv_if payload into named fields.
    // fanout_write_if.ready is tied high (pool always accepts writes).
    fanout_write_req_t fw_c;
    assign fw_c = fanout_write_req_t'(fanout_write_if.rv_payload);
    assign fanout_write_if.ready = 1'b1;

    // Convenience aliases matching original port names (used in write logic).
    wire       write_en         = fanout_write_if.valid;
    wire       write_mask_valid = fw_c.mask_valid;
    wire signed [WEIGHT_W-1:0] write_weight    = fw_c.weight;
    wire [TILE_COORD_W-1:0] write_dst_x      = fw_c.dst_x;
    wire [TILE_COORD_W-1:0] write_dst_y      = fw_c.dst_y;
    wire [CORE_ID_W-1:0] write_core_id    = fw_c.core_id;
    wire [1:0] write_meta       = fw_c.meta;
    wire       write_valid      = fw_c.valid;
    wire       write_weight_only = fw_c.weight_only;
    wire       write_route_only  = fw_c.route_only;

    // A compact route field must not silently alias a protocol-width value.
    // In particular, the legacy 8'hff broadcast core-id cannot become worker
    // 3 in a 2-bit core-id profile; the runtime should expand such broadcasts
    // into explicit sparse fanout entries instead.
    wire write_route_fields_fit_c =
        byte_fits_width({{(8-TILE_COORD_W){1'b0}}, write_dst_x}, FANOUT_DST_X_W) &&
        byte_fits_width({{(8-TILE_COORD_W){1'b0}}, write_dst_y}, FANOUT_DST_Y_W) &&
        byte_fits_width({{(8-CORE_ID_W){1'b0}}, write_core_id}, FANOUT_CORE_ID_W);
    wire write_entry_valid_c = write_valid && write_route_fields_fit_c;

    // Epoch register
    // Flips on every graph_state_clear; used to invalidate all prior entries
    // without re-writing all rows.
    logic graph_epoch_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) graph_epoch_r <= 1'b0;
        else if (ena && graph_state_clear) graph_epoch_r <= ~graph_epoch_r;
    end

    // Memory wires
    logic [ENTRY_W-1:0]     mem_wr_data_c;
    logic [ENTRY_BYTES-1:0] mem_wr_mask_c;
    logic                   mem_wr_en_c;
    logic [FANOUT_ADDR_W-1:0] mem_wr_addr_c;
    logic [FANOUT_ADDR_W-1:0] mem_rd_addr_c;
    logic [ENTRY_W-1:0]     mem_rd_data;
    logic                   mem_waitreq;
    // route_pack_c removed from always_comb (read-back within always_comb creates
    // false combinational loops in Yosys).  Fields are now written directly into
    // mem_wr_data_c at their absolute byte-lane bit offsets.

    // Write logic
    //
    //  write_weight_only=0, write_route_only=0 -> full write
    //  write_weight_only=1                     -> weight only (byte 0)
    //  write_route_only=1                      -> route/meta only
    //
    // When meta is packed into route padding, route-only writes update both
    // route and valid/epoch bits.  In the split-meta layout, the legacy
    // dedicated meta byte remains full-write-only.
    always_comb begin
        mem_wr_en_c   = 1'b0;
        mem_wr_addr_c = write_index;
        mem_wr_data_c = '0;
        mem_wr_mask_c = '0;

        if (write_en && write_mask_valid) begin
            mem_wr_en_c = 1'b1;

            if (!write_route_only) begin
                // Weight lane (byte 0)
                mem_wr_mask_c[ROW_OFF_WEIGHT] = 1'b1;
                mem_wr_data_c[ROW_OFF_WEIGHT*8 +: 8] = 8'(signed'(write_weight));
            end

            if (!write_weight_only) begin
                // Route byte(s): write dst_x || dst_y || core_id directly into
                // mem_wr_data_c at absolute bit offsets (ROW_OFF_ROUTE*8 + field_offset).
                // Avoids the intermediate route_pack_c read-back that Yosys interpreted
                // as a combinational loop.
                if (FANOUT_DST_X_W > 0)
                    mem_wr_data_c[ROW_OFF_ROUTE*8 + ROFF_DST_X +: FANOUT_DST_X_W] =
                        FANOUT_DST_X_W'(write_dst_x);
                if (FANOUT_DST_Y_W > 0)
                    mem_wr_data_c[ROW_OFF_ROUTE*8 + ROFF_DST_Y +: FANOUT_DST_Y_W] =
                        FANOUT_DST_Y_W'(write_dst_y);
                if (FANOUT_CORE_ID_W > 0)
                    mem_wr_data_c[ROW_OFF_ROUTE*8 + ROFF_CORE_ID +: FANOUT_CORE_ID_W] =
                        FANOUT_CORE_ID_W'(write_core_id);
                if (PACK_META_IN_ROUTE) begin
                    mem_wr_data_c[ROW_OFF_ROUTE*8 + ROUTE_META_VALID_BIT] = write_entry_valid_c;
                    mem_wr_data_c[ROW_OFF_ROUTE*8 + ROUTE_META_EPOCH_BIT] = graph_epoch_r;
                end
                for (int rb = 0; rb < ROUTE_BYTES; rb = rb + 1) begin
                    mem_wr_mask_c[ROW_OFF_ROUTE + rb] = 1'b1;
                end
            end

            if (!PACK_META_IN_ROUTE && !write_weight_only && !write_route_only) begin
                // Split-meta full write - update dedicated meta lane.
                mem_wr_mask_c[ROW_OFF_META] = 1'b1;
                mem_wr_data_c[ROW_OFF_META*8 +: 8] = {
                    6'b0,                 // upper 6 bits unused (meta now 2-bit)
                    graph_epoch_r,        // bit 1: epoch stamp
                    write_entry_valid_c    // bit 0: valid only if route fits
                };
            end
        end
    end

    // Read address
    assign mem_rd_addr_c = read_index;

    // FF register file instance
    coldfoot_mem_bytelane_sync #(
        .DATA_W    (ENTRY_W),
        .DEPTH     (FANOUT_POOL_DEPTH),
        .BYTE_LANES(ENTRY_BYTES),
        .STYLE_HINT(MEM_STYLE_HINT)
    ) u_pool_mem (
        .clk        (clk),
        .rst_n      (rst_n),
        .ena        (1'b1),
        .rd_addr    (mem_rd_addr_c),
        .rd_data    (mem_rd_data),
        .wr_en      (mem_wr_en_c),
        .wr_addr    (mem_wr_addr_c),
        .wr_data    (mem_wr_data_c),
        .wr_byte_en (mem_wr_mask_c),
        .waitrequest(mem_waitreq)
    );

    // Read pipeline
    // mem_rd_data is valid 1 cycle after mem_rd_addr_c is presented.  Pipeline-register the read_en to gate the response valid.
    logic read_en_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) read_en_r <= 1'b0;
        else if (ena) read_en_r <= read_en;
    end

    // Read response decode
    // Use CURRENT graph_epoch_r (not pipelined) so a graph_state_clear that
    // fires between read issue and response correctly invalidates the entry.
    fanout_read_rsp_t fanout_rsp_payload_c;
    logic             fanout_rsp_valid_c;
    logic [ROUTE_BYTES*8-1:0] route_unpack_c;
    always_comb begin
        route_unpack_c = mem_rd_data[ROW_OFF_ROUTE*8 +: ROUTE_BYTES*8];

        fanout_rsp_valid_c                 = read_en_r;
        fanout_rsp_payload_c.weight        = WEIGHT_W'(signed'(mem_rd_data[ROW_OFF_WEIGHT*8 +: 8]));
        // Unpack routing bits at parameter-derived width.
        fanout_rsp_payload_c.dst_x    = TILE_COORD_W'(route_unpack_c[ROFF_DST_X   +: FANOUT_DST_X_W]);
        fanout_rsp_payload_c.dst_y    = TILE_COORD_W'(route_unpack_c[ROFF_DST_Y   +: FANOUT_DST_Y_W]);
        fanout_rsp_payload_c.core_id  = CORE_ID_W'(route_unpack_c[ROFF_CORE_ID +: FANOUT_CORE_ID_W]);
        // Valid only when hardware valid bit (meta[0]) is set AND the stored
        // epoch (meta[1]) matches the current graph epoch register.
        if (PACK_META_IN_ROUTE) begin
            fanout_rsp_payload_c.meta_valid =
                route_unpack_c[ROUTE_META_VALID_BIT] &&
                (route_unpack_c[ROUTE_META_EPOCH_BIT] == graph_epoch_r);
        end else begin
            fanout_rsp_payload_c.meta_valid =
                mem_rd_data[ROW_OFF_META*8 + 0] &&
                (mem_rd_data[ROW_OFF_META*8 + 1] == graph_epoch_r);
        end
    end

    assign fanout_rsp.valid      = fanout_rsp_valid_c;
    assign fanout_rsp.rv_payload = fanout_rsp_payload_c;
    // fanout_rsp.ready driven by tile_top (response consumer always accepts)

    assign mem_stall = mem_waitreq;  // always 0 (FF path)

endmodule

`default_nettype wire
