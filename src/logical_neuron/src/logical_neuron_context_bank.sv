`default_nettype none

`include "tile_flit_types.vh"

// ─────────────────────────────────────────────────────────────────────────────
// logical_neuron_context_bank — direct FF register file for neuron context
//
// For NEURONS_PER_TILE=4 on sky130A the upstream coldfoot_mem_bytelane_sync
// was a pure FF array anyway.  This rewrite removes the memory abstraction
// entirely and uses a flat ctx_r[0:N-1] array with combinatorial dual-port
// read, saving ~212 FFs vs the phase-sequenced implementation.
//
// Changes (area optimisations):
//   RF narrowed: RF_W=24 (rf[3]=refractory removed with OP_REFRACT)
//   Time removed: last_time field dropped (no timestamp ordering in neutern)
//   CTX_W = 24(RF) + 1(tag) + 3(flags) = 28 bits per neuron
//   4 neurons: 4×28 = 112 FFs  (was 4×37 = 148 FFs)
//
// Row layout (MSB→LSB, 28 bits):
//   [27]    spike_flag
//   [26]    cmp_eq
//   [25]    cmp_ge
//   [24]    last_tag   (TAG_W=1)
//   [11:0]  rf_state_flat (RF_FLAT_W=12, 3 × 4-bit regs: rf[2]=threshold, rf[1]=accum, rf[0]=vm)
//
// mem_stall is always 0 — reads are combinatorial.
// ─────────────────────────────────────────────────────────────────────────────
(* keep_hierarchy = "no" *)
module logical_neuron_context_bank #(
    parameter int unsigned NEURONS_PER_TILE = 1,
    // Legacy parameters kept for API compatibility; all ignored in this impl.
    parameter int unsigned MEM_STYLE_HINT   = 3,
    parameter bit CONTEXT_SPLIT_LAYOUT      = 1'b0,
    parameter bit DUAL_READ_CONTEXT         = 1'b1
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,
    input  wire soft_reset_valid,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] soft_reset_idx,
    input  wire [RF_FLAT_W-1:0]   init_rf_flat,
    input  wire commit_valid,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] commit_idx,
    input  wire [RF_FLAT_W-1:0]   commit_rf_state_flat,
    input  wire [TAG_W-1:0]       commit_last_tag,
    // commit_last_time removed: neutern does not use timestamps
    input  wire                   commit_cmp_ge,
    input  wire                   commit_cmp_eq,
    input  wire                   commit_spike_flag,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] read_idx_a,
    output logic [RF_FLAT_W-1:0]   read_rf_state_flat_a,
    output logic [TAG_W-1:0]       read_last_tag_a,
    // read_last_time_a removed: neutern does not use timestamps
    output logic                   read_cmp_ge_a,
    output logic                   read_cmp_eq_a,
    output logic                   read_spike_flag_a,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] read_idx_b,
    output logic [RF_FLAT_W-1:0]   read_rf_state_flat_b,
    output logic [TAG_W-1:0]       read_last_tag_b,
    // read_last_time_b removed: neutern does not use timestamps
    output logic                   read_cmp_ge_b,
    output logic                   read_cmp_eq_b,
    output logic                   read_spike_flag_b,
    // mem_stall: always 0 — combinatorial reads never stall.
    output logic mem_stall
);
    // ── Constants ─────────────────────────────────────────────────────────────
    localparam int unsigned NEURON_IDX_W = (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE);
    localparam logic [RF_FLAT_W-1:0] DEFAULT_RF = {4'h7, 4'h0, 4'h0};  // rf[2]=threshold=7, rf[1]=accum=0, rf[0]=vm=0
    localparam int unsigned RF_W         = RF_FLAT_W;  // = 12 (3 x 4-bit regs)
    // CTX layout: [RF_FLAT_W-1:0]=rf_state_flat, [RF_FLAT_W]=last_tag, [RF_FLAT_W+1]=cmp_ge,
    //             [RF_FLAT_W+2]=cmp_eq, [RF_FLAT_W+3]=spike_flag
    //   = 12+1+1+1+1 = 16 bits (NEURON_EXEC_CTX_W=16)
    // Context row: rf(24) + tag(TAG_W=1) + 3 flags = 28b  (last_time and rf[3] removed)
    localparam int unsigned CTX_W = RF_W + TAG_W + 3;

    // ── Direct FF context array ───────────────────────────────────────────────
    logic [CTX_W-1:0]              ctx_r     [0:NEURONS_PER_TILE-1];
    logic [NEURONS_PER_TILE-1:0]   ctx_valid_r;  // 1 = entry has been written since last clear

    // ── Pack helper ───────────────────────────────────────────────────────────
    function automatic [CTX_W-1:0] pack_ctx(
        input logic [RF_FLAT_W-1:0]    rf,
        input logic [TAG_W-1:0]        tag,
        // last_time removed
        input logic                    cmp_ge,
        input logic                    cmp_eq,
        input logic                    spike_flag
    );
        pack_ctx = {spike_flag, cmp_eq, cmp_ge, tag, rf};
    endfunction

    // ── Write sequencer ───────────────────────────────────────────────────────
    // Priority: soft_reset > ena&&commit.  graph_state_clear invalidates all.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctx_valid_r <= '0;
        end else begin
            if (graph_state_clear) begin
                // Invalidate all entries; neurons will be soft-reset before use.
                ctx_valid_r <= '0;
            end else if (soft_reset_valid) begin
                ctx_r[soft_reset_idx]     <= pack_ctx(init_rf_flat, '0,
                                                      1'b0, 1'b0, 1'b0);
                ctx_valid_r[soft_reset_idx] <= 1'b1;
            end else if (ena && commit_valid) begin
                ctx_r[commit_idx]     <= pack_ctx(commit_rf_state_flat,
                                                  commit_last_tag,
                                                  commit_cmp_ge, commit_cmp_eq,
                                                  commit_spike_flag);
                ctx_valid_r[commit_idx] <= 1'b1;
            end
        end
    end

    // ── Combinatorial dual-port read ─────────────────────────────────────────
    // No stall, no registered output stage.
    assign mem_stall = 1'b0;

    // Unpack macro (used twice for A and B).
    `define UNPACK_CTX(row, rf, tag, ge, eq, sf) \
        rf  = row[RF_W-1:0]; \
        tag = row[RF_W + TAG_W - 1 : RF_W]; \
        ge  = row[RF_W + TAG_W]; \
        eq  = row[RF_W + TAG_W + 1]; \
        sf  = row[RF_W + TAG_W + 2];

    always_comb begin
        // Port A
        if (!ctx_valid_r[read_idx_a]) begin
            read_rf_state_flat_a = DEFAULT_RF;
            read_last_tag_a      = '0;
            read_cmp_ge_a        = 1'b0;
            read_cmp_eq_a        = 1'b0;
            read_spike_flag_a    = 1'b0;
        end else begin
            `UNPACK_CTX(ctx_r[read_idx_a],
                        read_rf_state_flat_a, read_last_tag_a,
                        read_cmp_ge_a, read_cmp_eq_a, read_spike_flag_a)
        end
        // Port B
        if (!ctx_valid_r[read_idx_b]) begin
            read_rf_state_flat_b = DEFAULT_RF;
            read_last_tag_b      = '0;
            read_cmp_ge_b        = 1'b0;
            read_cmp_eq_b        = 1'b0;
            read_spike_flag_b    = 1'b0;
        end else begin
            `UNPACK_CTX(ctx_r[read_idx_b],
                        read_rf_state_flat_b, read_last_tag_b,
                        read_cmp_ge_b, read_cmp_eq_b, read_spike_flag_b)
        end
    end

    `undef UNPACK_CTX

endmodule

`default_nettype wire
