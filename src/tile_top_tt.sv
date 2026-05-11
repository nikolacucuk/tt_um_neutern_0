// =============================================================================
// tile_top_tt.sv — TinyTapeout flat-port adapter for tile_top
// =============================================================================
// Wraps tile_top with flat (non-interface) ports so that the Verilog-only
// TinyTapeout top module (project.v) can instantiate it without needing
// SystemVerilog interface syntax.  The rv_if channels are created internally
// and are never exposed as ports.
//
// BOUNDARY PROTOCOL: neutern_spike_t  (see tt_um_neutern_pkg.sv)
//   Wire byte: { weight[3:0], 2'b00, neuron_y[0], neuron_x[0] }  =  8-bit TT wire
//   Active payload: 6 bits (2×2 grid, 1-bit per coord).  Bits [3:2] reserved=0.
//   event_time is excluded; one flit per TT cycle — no serialisation.
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │  host (TT)      tile_top_tt            tile_top (MESSAGE_W = 40)    │
//  │  8-bit flit ──► expand ─────────────► message_packet_t → rv_in     │
//  │  8-bit flit ◄── extract ◄───────────  message_packet_t ← rv_out    │
//  └──────────────────────────────────────────────────────────────────────┘
//
// INGRESS expansion  (neutern_spike_t → message_packet_t)
//   kind       = MSG_INPUT
//   dst_x/y    = TILE_COORD_X / TILE_COORD_Y
//   core_id    = flat_idx = neuron_y * NEURONS_PER_ROW + neuron_x
//   data       = {{4{weight[3]}}, weight[3:0]}  (sign-extended to 8 bits)
//   event_time = 0                               (not carried in neutern_spike_t)
//   meta       = PKT_PLANE_DATA
//   all other fields = 0
//
// EGRESS contraction  (message_packet_t MSG_OUTPUT → neutern_spike_t)
//   weight     = out_pkt.data[3:0]
//   neuron_y   = out_pkt.core_id / NEURONS_PER_ROW  (constant-parameter divide)
//   neuron_x   = out_pkt.core_id % NEURONS_PER_ROW
//
// NOTE: spikes are dropped by tile_ingress until a non-spike configuration
//       packet (MSG_WRITE / MSG_PROG_* etc.) has armed the stream.  The host
//       must configure the tile (CSR writes, ucode, fanout table) before
//       injecting neutern_spike_t flits.
//
// TT pin mapping:
//   rv_in_payload[7:0]  ← ui_in[7:0]   (host → tile, neutern_spike_t)
//   rv_in_valid         ← uio_in[0]
//   rv_out_ready        ← uio_in[1]
//   rv_in_ready         → uio_out[0]
//   rv_out_valid        → uio_out[1]
//   rv_out_payload[7:0] → uo_out[7:0]  (tile → host, neutern_spike_t)
// =============================================================================

`default_nettype none


`ifndef YOSYS
import tile_pkg::CORE_ID_W;
import tile_pkg::DATA_W;
import tile_pkg::MSG_INPUT;
import tile_pkg::PKT_PLANE_DATA;
import tile_pkg::TILE_COORD_W;
import tile_pkg::message_packet_t;
import tile_pkg::packet_meta_with_plane;
`endif
module tile_top_tt
  #(
    parameter int unsigned TILE_COORD_X          = 0,
    parameter int unsigned TILE_COORD_Y          = 0,
    parameter int unsigned LOCAL_Z               = 0,
    parameter int unsigned NEURONS_PER_TILE      = 4,
    parameter int unsigned WORKER_CORES_PER_TILE = 1,
    parameter int unsigned FANOUT_POOL_DEPTH     = 4,
    // X-axis neuron count; used to convert flat core_id ↔ (neuron_y, neuron_x).
    // Must equal the number of neurons per row in the logical grid.
    parameter int unsigned NEURONS_PER_ROW       = 2,
    // Internal tile protocol width; must match tile_top default (message_packet_t = 40 bits).
    parameter int unsigned MESSAGE_W             = 40
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ena,
    // rv_in  (host → tile) — neutern_spike_t boundary flit
    input  wire       rv_in_valid,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [7:0] rv_in_payload,    // { weight[3:0], 2'b00, neuron_y[0], neuron_x[0] }
    /* verilator lint_on UNUSEDSIGNAL */
    output wire       rv_in_ready,
    // rv_out (tile → host) — neutern_spike_t boundary flit
    output wire       rv_out_valid,
    output wire [7:0] rv_out_payload,   // { weight[3:0], 2'b00, neuron_y[0], neuron_x[0] }
    input  wire       rv_out_ready
);

    // ── Internal rv_if channels (MESSAGE_W = 40 bits wide) ───────────────────
    rv_if #(.WIDTH(MESSAGE_W)) u_rv_in  ();
    rv_if #(.WIDTH(MESSAGE_W)) u_rv_out ();

    // ── Ingress: neutern_spike_t → message_packet_t ───────────────────────────
    // Decompose the 8-bit input flit: { weight[3:0], 2'b00, neuron_y[0], neuron_x[0] }
    // Bits [3:2] are reserved/zero; only 1-bit coords are used (2×2 grid).
    wire [3:0] in_weight   = rv_in_payload[7:4];   // 4-bit signed synaptic weight
    wire [0:0] in_neuron_y = rv_in_payload[1];     // 1-bit Y coordinate (0..1)
    wire [0:0] in_neuron_x = rv_in_payload[0];     // 1-bit X coordinate (0..1)

    // Flat neuron index: core_id = neuron_y * NEURONS_PER_ROW + neuron_x.
    // Sized to CORE_ID_W bits; the multiply result fits because max flat_idx
    // = (2^1-1)*2 + 1 = 3 which requires only 2 bits (CORE_ID_W=3 ≥ 2).
    wire [CORE_ID_W-1:0] in_core_id =
        CORE_ID_W'(in_neuron_y * NEURONS_PER_ROW + in_neuron_x);

    // rv_in_payload[3:2] are protocol-reserved zeros (2×2 grid, 1-bit coords).
    // Explicitly consumed here to satisfy UNUSEDSIGNAL without suppression.
    wire _unused_rv_in = &{rv_in_payload[3:2]};

    // Sign-extend 4-bit weight to the 4-bit message_packet_t.data field (DATA_W=4).
    wire [DATA_W-1:0] in_data = in_weight;

    // Build the full message_packet_t for tile_top rv_in.
    message_packet_t in_pkt;
    always_comb begin
        in_pkt            = '0;
        in_pkt.kind       = MSG_INPUT;
        in_pkt.broadcast  = 1'b0;
        in_pkt.dst_x      = TILE_COORD_W'(TILE_COORD_X);
        in_pkt.dst_y      = TILE_COORD_W'(TILE_COORD_Y);
        // src_x/src_y removed from message_packet_t (single-tile profile)
        in_pkt.core_id    = CORE_ID_W'(in_core_id);
        in_pkt.data       = in_data;
        // event_time removed from message_packet_t (neutern profile)
        in_pkt.tag        = '0;
        in_pkt.meta       = packet_meta_with_plane(8'h00, PKT_PLANE_DATA);
    end

    // Drive the internal rv_in channel with valid + the expanded packet.
    assign u_rv_in.valid      = rv_in_valid;
    assign u_rv_in.rv_payload = in_pkt;
    assign rv_in_ready        = u_rv_in.ready;

    // ── Egress: message_packet_t → neutern_spike_t ───────────────────────────
    // Receive the full MESSAGE_W-bit output packet from tile_top.
    // Only the fields used for output pin mapping are accessed; remaining
    // struct fields are explicitly consumed via _unused_out_pkt to satisfy
    // UNUSEDSIGNAL without suppression.
    message_packet_t out_pkt;
    assign out_pkt = message_packet_t'(u_rv_out.rv_payload);

    // Consume message_packet_t fields not mapped to rv_out_payload pins.
    // (kind, broadcast, dst_x, dst_y, tag, meta, event_time are all protocol-
    //  internal; only core_id and data drive the egress flit.)
    wire _unused_out_pkt = &{out_pkt.kind, out_pkt.broadcast,
                              out_pkt.dst_x, out_pkt.dst_y,
                              out_pkt.tag, out_pkt.meta};

    // Recover (neuron_y, neuron_x) from the flat core_id in the output packet.
    // core_id = LOCAL_Z + flat_idx, and with LOCAL_Z=0: core_id = flat_idx.
    // Invert: neuron_y = core_id / NEURONS_PER_ROW,  neuron_x = core_id % NEURONS_PER_ROW.
    // With NEURONS_PER_ROW=2: division by power-of-2 → single bit select.
    logic [0:0] out_neuron_y_c;
    logic [0:0] out_neuron_x_c;
    always_comb begin
        out_neuron_y_c = 1'(out_pkt.core_id / 8'(NEURONS_PER_ROW));
        out_neuron_x_c = 1'(out_pkt.core_id -
                            (8'(out_neuron_y_c) * 8'(NEURONS_PER_ROW)));
    end

    // Pack output flit: { weight[3:0], 2'b00, neuron_y[0], neuron_x[0] }
    // Bits [3:2] = reserved zero padding (2x2 grid, 1-bit coords).
    assign rv_out_valid   = u_rv_out.valid;
    assign rv_out_payload = {out_pkt.data[3:0], 2'b00, out_neuron_y_c, out_neuron_x_c};
    assign u_rv_out.ready = rv_out_ready;

    // ── tile_top core ─────────────────────────────────────────────────────────
    tile_top #(
        .TILE_COORD_X          (TILE_COORD_X),
        .TILE_COORD_Y          (TILE_COORD_Y),
        .LOCAL_Z               (LOCAL_Z),
        .NEURONS_PER_TILE      (NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE (WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH     (FANOUT_POOL_DEPTH),
        .MESSAGE_W             (MESSAGE_W),
        .TILE_BANK_MEM_STYLE   (0)  // STYLE_AUTO: inferred RTL (sky130A compatible)
    ) u_tile_top (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .ena                   (ena),
        .rv_in                 (u_rv_in),
        .rv_out                (u_rv_out)
    );

endmodule

`default_nettype wire
