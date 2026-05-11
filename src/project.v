/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_neutern_0 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ---------------------------------------------------------------------------
  // Flit type at this boundary: neutern_spike_t  (NEUTERN_FLIT_W = 6 bits)
  //   Defined in tt_um_neutern_pkg.sv as:
  //   { weight[3:0], neuron_y[0], neuron_x[0] }
  //
  //   Bits [7:4] = weight   — 4-bit signed synaptic weight (-8..+7)
  //   Bits [3:2] = unused   — reserved, host drives 0 / tile drives 0
  //   Bit  [1]   = neuron_y — Y coordinate within tile (0..1)
  //   Bit  [0]   = neuron_x — X coordinate within tile (0..1)
  //
  // Single-cycle, single-byte transfer: ui_in[7:0] carries the full flit.
  // event_time is NOT included; timing is managed at the host driver level.
  //
  // Grid: 2×2 = 4 neurons, 1 tile (see tt_um_neutern_pkg.sv for full params).
  // NOTE: tile_top uses MESSAGE_W=53 internally (message_packet_t, see
  //       tile_flit_types.vh); tile_top_tt expands neutern_spike_t →
  //       message_packet_t on ingress, and extracts neutern_spike_t from
  //       message_packet_t on egress (MSG_OUTPUT).
  //
  // Pin usage summary
  // ---------------------------------------------------------------------------
  // Inputs (ui_in):
  //   ui_in[7:4]   USED   → weight[3:0]          (4-bit signed synaptic weight)
  //   ui_in[3:2]   UNUSED  reserved, ignored
  //   ui_in[1]     USED   → neuron_y[0]           (Y coordinate 0..1)
  //   ui_in[0]     USED   → neuron_x[0]           (X coordinate 0..1)
  //
  // Dedicated outputs (uo_out):
  //   uo_out[7:4]  USED   ← weight[3:0]          (4-bit signed synaptic weight)
  //   uo_out[3:2]  UNUSED  driven 0
  //   uo_out[1]    USED   ← neuron_y[0]           (Y coordinate 0..1)
  //   uo_out[0]    USED   ← neuron_x[0]           (X coordinate 0..1)
  //
  // Bidirectional (uio), configured as:
  //   uio_in[0]    USED   → rv_in.valid           (host asserts flit byte valid)
  //   uio_in[1]    USED   → rv_out.ready          (host asserts downstream ready)
  //   uio_in[7:2]  UNUSED  input, tied off via _unused
  //   uio_out[0]   USED   ← rv_in.ready           (tile asserts ready to receive)
  //   uio_out[1]   USED   ← rv_out.valid          (tile asserts output byte valid)
  //   uio_out[7:2] UNUSED  driven low (direction=input, uio_oe[7:2]=0)
  //   uio_oe[1:0]  = 1    (output direction for rv_in.ready, rv_out.valid)
  //   uio_oe[7:2]  = 0    (input direction, pins not used)
  //
  // Control inputs:
  //   ena          UNUSED  always 1 in TT; not connected to tile
  //   clk          USED   → tile clk
  //   rst_n        USED   → tile rst_n
  // ---------------------------------------------------------------------------

  wire        rv_in_ready_w;
  wire        rv_out_valid_w;
  wire [7:0]  rv_out_payload_w;  // {weight[3:0], 2'b00, neuron_y[0], neuron_x[0]} (2x2 grid)

  // Suppress unused-input warnings for uio_in[7:2] and ena
  wire _unused = &{ena, uio_in[7:2], 1'b0};

  assign uio_oe  = 8'b00000011;                              // [1:0]=output, [7:2]=input(unused)
  assign uio_out = {6'b0, rv_out_valid_w, rv_in_ready_w};    // [7:2]=0(unused) [1]=rv_out.valid [0]=rv_in.ready
  assign uo_out  = rv_out_payload_w;

  // Parameter values mirror tt_um_neutern_pkg constants (kept in sync manually).
  // Literals used here so project.v is self-contained for Yosys port-checking.
  tile_top_tt #(
      .NEURONS_PER_TILE      (4),   // tt_um_neutern_pkg::NEUTERN_NEURONS_PER_TILE
      .WORKER_CORES_PER_TILE (1),   // tt_um_neutern_pkg::NEUTERN_WORKER_CORES
      .FANOUT_POOL_DEPTH     (4),   // tt_um_neutern_pkg::NEUTERN_FANOUT_POOL_DEPTH
      .NEURONS_PER_ROW       (2)    // tt_um_neutern_pkg::NEUTERN_NEURONS_X
  ) u_tile (
      .clk                   (clk),
      .rst_n                 (rst_n),
      .ena                   (ena),
      // rv_in: host → tile
      .rv_in_valid           (uio_in[0]),
      .rv_in_payload         (ui_in[7:0]),
      .rv_in_ready           (rv_in_ready_w),
      // rv_out: tile → host
      .rv_out_valid          (rv_out_valid_w),
      .rv_out_payload        (rv_out_payload_w),
      .rv_out_ready          (uio_in[1])
  );

endmodule
