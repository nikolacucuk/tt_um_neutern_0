// =============================================================================
// tt_um_neutern_pkg.sv — Project-specific parameters for tt_um_neutern_0
// =============================================================================
// Defines the architectural constants that describe this TinyTapeout instance:
//   - A single tile with a 2×2 neuron grid (4 neurons total).
//   - The host-facing ready/valid flit type is neutern_spike_t (defined below),
//     a project-local 6-bit spike payload mapped to the 8-bit TT byte boundary.
//
// Relationship to tile_pkg.sv
// ───────────────────────────
// tile_pkg.sv is the global mesh/SoC parameter set.  This package defines
// project-specific overrides (narrower fields, smaller grid) that apply at
// the tt_um_neutern_0 boundary.  tile_top and its sub-modules are
// instantiated with these values via tile_top_tt.sv.
//
// neutern_spike_t layout (used at the TT host↔tile boundary):
// ┌────────────────┬────────────┬────────────┐
// │  weight[3:0]   │ neuron_y   │ neuron_x   │
// │   4 bits       │  1 bit     │  1 bit     │
// └────────────────┴────────────┴────────────┘
//  Total: NEUTERN_FLIT_W = 6 bits, zero-padded to 8-bit TT wire:
//    ui_in[7:4] = weight[3:0]
//    ui_in[3:2] = reserved (ignored on input, driven 0 on output)
//    ui_in[1]   = neuron_y[0]
//    ui_in[0]   = neuron_x[0]
//
// Note on NEUTERN_NEURON_LOCAL_W = 1:
//   1 bit per axis → 2 addressable positions per axis (values 0..1).
//   At NEUTERN_NEURONS_X = 2 / NEUTERN_NEURONS_Y = 2 (2×2 grid), all
//   positions 0 and 1 are addressable.  $clog2(2) = 1 bit.
//
// Note on NEUTERN_WEIGHT_W = 4:
//   4-bit signed weight → range -8..+7 (two's complement).
// =============================================================================

package tt_um_neutern_pkg;

    // ── Grid geometry ─────────────────────────────────────────────────────────
    // Single tile, 2×2 neuron layout.
    localparam int unsigned NEUTERN_MESH_TILES_X  = 1;   // tiles in X direction
    localparam int unsigned NEUTERN_MESH_TILES_Y  = 1;   // tiles in Y direction
    localparam int unsigned NEUTERN_NEURONS_X     = 2;   // neurons per tile along X
    localparam int unsigned NEUTERN_NEURONS_Y     = 2;   // neurons per tile along Y
    localparam int unsigned NEUTERN_NEURONS_PER_TILE =
        NEUTERN_NEURONS_X * NEUTERN_NEURONS_Y;           // = 4
    localparam int unsigned NEUTERN_FANOUT_POOL_DEPTH = 4; // one fanout entry per neuron

    // ── Flit field widths ─────────────────────────────────────────────────────
    localparam int unsigned NEUTERN_NEURON_LOCAL_W = 1;  // bits per neuron axis coord (0..1, $clog2(2)=1)
    localparam int unsigned NEUTERN_WEIGHT_W       = 4;  // signed synaptic weight (4-bit, range -8..+7)

    // ── Derived flit width ────────────────────────────────────────────────────
    // neutern_spike_t = { weight[3:0], neuron_y[0], neuron_x[0] }
    localparam int unsigned NEUTERN_FLIT_W =
        NEUTERN_WEIGHT_W +
        NEUTERN_NEURON_LOCAL_W + NEUTERN_NEURON_LOCAL_W;  // = 6

    // ── tile_top instantiation parameters ─────────────────────────────────────
    // Passed into tile_top_tt / tile_top in project.v.
    localparam int unsigned NEUTERN_TILE_COORD_X        = 0;
    localparam int unsigned NEUTERN_TILE_COORD_Y        = 0;
    localparam int unsigned NEUTERN_LOCAL_Z             = 0;
    localparam int unsigned NEUTERN_WORKER_CORES        = 1;  // shared worker core

    // ── Project-local spike flit type ────────────────────────────────────────
    // 6-bit spike (zero-padded to 8-bit TT wire): 4-bit signed weight + 1-bit Y + 1-bit X.
    // Wire mapping: {weight[3:0], 2'b00, neuron_y[0], neuron_x[0]}.
    // Does NOT include event_time (removed to fit the TT boundary).
    typedef struct packed {
        logic signed [NEUTERN_WEIGHT_W-1:0]       weight;    // [7:4] signed weight (-8..+7)
        logic        [NEUTERN_NEURON_LOCAL_W-1:0] neuron_y; // [1]   Y coordinate (0..1)
        logic        [NEUTERN_NEURON_LOCAL_W-1:0] neuron_x; // [0]   X coordinate (0..1)
    } neutern_spike_t;

endpackage
