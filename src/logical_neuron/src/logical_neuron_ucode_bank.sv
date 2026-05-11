`default_nettype none

`ifndef YOSYS
import tile_pkg::NEURON_IDX_W;
import tile_pkg::NEURON_UCODE_RSP_W;
import tile_pkg::neuron_ucode_req_t;
`endif
// -----------------------------------------------------------------------------
// logical_neuron_ucode_bank
//
// Single-port, 3-cycle-latency ucode word store for logical neurons.
//
//   Write port : synchronous, byte-wide 16-bit words, per-neuron or shared.
//   Read  port : rv_if handshake; request payload = neuron_ucode_req_t,
//                response payload = neuron_ucode_rsp_t (3 cycles later).
//
// Pipeline sketch:
//
//   Cycle N   : read_port.valid asserted, address captured into stage-0 reg
//   Cycle N+1 : array read with write-first bypass -> stage-1 reg
//   Cycle N+2 : final word registered -> read_rsp_port.valid asserted
//
//  Stage 0 (addr latch) -> Stage 1 (array/bypass) -> Stage 2 (word out)
// -----------------------------------------------------------------------------
(* keep_hierarchy = "no" *)
module logical_neuron_ucode_bank #(
    parameter int unsigned NEURONS_PER_TILE      = 1,
    // Storage capacity in 16-bit words per neuron.  Runtime software uses
    // at most 16 words; increase here if future programs need more.
    parameter int unsigned UCODE_WORDS_PER_NEURON = 16,
    // When set, collapse per-neuron storage into one shared bank of
    // UCODE_WORDS_PER_NEURON words.  All neurons run the same microprogram.
    // Keep at 0 (default) for independent per-neuron ucode images.
    parameter int unsigned UCODE_SHARED_BANK = 0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    // ---- Write port -------------------------------------------------------
    input  wire write_en,
    input  wire [(NEURONS_PER_TILE <= 1) ? 0 : $clog2(NEURONS_PER_TILE)-1:0] write_neuron_idx,
    input  wire write_broadcast,          // 1 = write all neurons (shared / soft-reset)
    input  wire [4:0]  write_word_index,
    input  wire [NEURON_UCODE_RSP_W-1:0] write_word,
    // ---- Read port: rv_if, single port, 3-cycle latency -------------------
    //   Request  payload : neuron_ucode_req_t  { logical_idx, word_index }
    //   Response payload : neuron_ucode_rsp_t  { word }
    rv_if.rx  read_port,
    rv_if.tx  read_rsp_port
);
    // ---- Local parameters -------------------------------------------------
    /* verilator lint_off VARHIDDEN */
    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE); /* verilator lint_on VARHIDDEN */
    localparam int unsigned UCODE_NEURON_BANKS =
        (UCODE_SHARED_BANK != 0) ? 1 : NEURONS_PER_TILE;
    localparam int unsigned UCODE_BANK_IDX_W =
        (UCODE_NEURON_BANKS <= 1) ? 1 : $clog2(UCODE_NEURON_BANKS);
    localparam int unsigned UCODE_WORD_IDX_W =
        (UCODE_WORDS_PER_NEURON <= 1) ? 1 : $clog2(UCODE_WORDS_PER_NEURON);
    localparam int unsigned UCODE_MEM_DEPTH = UCODE_NEURON_BANKS * UCODE_WORDS_PER_NEURON;
    localparam int unsigned UCODE_ADDR_W =
        (UCODE_MEM_DEPTH <= 1) ? 1 : $clog2(UCODE_MEM_DEPTH);

    // ---- Address helper function ------------------------------------------
    // Maps (neuron_idx, word_idx) to flat storage address.
    // Shared-bank mode: neuron_idx is ignored (all map to row 0..N-1).
    function automatic [UCODE_ADDR_W-1:0] ucode_addr_for(
        input logic [NEURON_IDX_W-1:0] neuron_idx,
        input logic [4:0] word_idx
    );
        logic [UCODE_ADDR_W-1:0]     addr;
        logic [UCODE_BANK_IDX_W-1:0] neuron_idx_eff;
        logic [UCODE_WORD_IDX_W-1:0] word_idx_masked;
        begin
            neuron_idx_eff  = (UCODE_SHARED_BANK != 0)
                              ? {UCODE_BANK_IDX_W{1'b0}}
                              : UCODE_BANK_IDX_W'(neuron_idx);
            // Mask the 5-bit CSR word index to the configured depth.
            word_idx_masked = word_idx[UCODE_WORD_IDX_W-1:0];
            addr = UCODE_ADDR_W'(neuron_idx_eff) * UCODE_ADDR_W'(UCODE_WORDS_PER_NEURON);
            addr = addr + UCODE_ADDR_W'(word_idx_masked);
            ucode_addr_for = addr;
        end
    endfunction

    // ---- Write path -------------------------------------------------------
    // Per-neuron mode  : write_neuron_idx selects the target row.
    // Shared-bank mode : neuron_idx is masked to 0 inside ucode_addr_for.
    logic write_fire_c;
    logic [UCODE_ADDR_W-1:0] write_addr_c;

    always_comb begin
        write_fire_c = write_en;
        write_addr_c = ucode_addr_for(
            NEURON_IDX_W'(write_neuron_idx), write_word_index);
    end

    // ---- Storage array ----------------------------------------------------
    (* ram_style = "block" *) logic [NEURON_UCODE_RSP_W-1:0] ucode_words_r [0:UCODE_MEM_DEPTH-1];

    initial begin
        for (int init_idx = 0; init_idx < UCODE_MEM_DEPTH; init_idx++)
            ucode_words_r[init_idx] = '0;
    end

    always_ff @(posedge clk) begin
        if (rst_n && ena && write_fire_c)
            ucode_words_r[write_addr_c] <= write_word;
    end

    // ---- Read port: 3-stage pipeline, single port -------------------------
    // Stage 0 : latch incoming read address.
    // Stage 1 : read array + write-first bypass for same-cycle collision.
    // Stage 2 : register final word, assert read_rsp_port.valid.

    neuron_ucode_req_t       read_req_c;
    logic [UCODE_ADDR_W-1:0] read_addr_c;
    logic                    read_valid_stage0_r;
    logic                    read_valid_stage1_r;
    logic                    read_valid_stage2_r;
    logic [NEURON_UCODE_RSP_W-1:0] read_word_stage1_r;
    logic [NEURON_UCODE_RSP_W-1:0] read_word_stage2_r;
    logic [UCODE_ADDR_W-1:0] read_addr_stage0_r;

    // Bank always accepts; caller may keep valid high for consecutive reads.
    assign read_port.ready = 1'b1;
    assign read_req_c      = neuron_ucode_req_t'(read_port.rv_payload);
    assign read_addr_c     = ucode_addr_for(
        NEURON_IDX_W'(read_req_c.logical_idx), read_req_c.word_index);

    initial begin
        read_addr_stage0_r  = '0;
        read_valid_stage0_r = 1'b0;
        read_valid_stage1_r = 1'b0;
        read_valid_stage2_r = 1'b0;
        read_word_stage1_r  = '0;
        read_word_stage2_r  = '0;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            read_addr_stage0_r  <= '0;
            read_valid_stage0_r <= 1'b0;
            read_valid_stage1_r <= 1'b0;
            read_valid_stage2_r <= 1'b0;
            read_word_stage1_r  <= '0;
            read_word_stage2_r  <= '0;
        end else if (ena) begin
            // Stage 0: latch incoming request address
            read_valid_stage0_r <= read_port.valid;
            if (read_port.valid)
                read_addr_stage0_r <= read_addr_c;

            // Stage 1: read array with write-first bypass
            read_valid_stage1_r <= read_valid_stage0_r;
            if (read_valid_stage0_r) begin
                if (write_fire_c && (write_addr_c == read_addr_stage0_r))
                    read_word_stage1_r <= write_word;
                else
                    read_word_stage1_r <= ucode_words_r[read_addr_stage0_r];
            end

            // Stage 2: register final output word
            read_valid_stage2_r <= read_valid_stage1_r;
            if (read_valid_stage1_r)
                read_word_stage2_r <= read_word_stage1_r;
        end
    end

    // Response: ready from tile_top is not back-pressured (pipeline fires
    // unconditionally once valid is asserted).
    assign read_rsp_port.valid      = read_valid_stage2_r;
    // rv_payload width = $bits(neuron_ucode_rsp_t) = NEURON_UCODE_RSP_W bits.
    assign read_rsp_port.rv_payload = read_word_stage2_r;

endmodule

`default_nettype wire
