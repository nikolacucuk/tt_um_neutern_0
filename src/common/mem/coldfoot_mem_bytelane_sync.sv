`default_nettype none

// coldfoot_mem_bytelane_sync — byte-enabled synchronous simple-dual-port
// FF register file.
//
// STYLE_HINT is accepted for interface compatibility but ignored.
// All instances synthesize to flip-flop arrays with byte-enable write
// and write-first bypass semantics.
//
// Constraints
// -----------
//   * DATA_W must be a multiple of 8.
//   * DEPTH >= 1.
//   * Read latency is 1 clock cycle (registered output).
//   * Write-through bypass: if wr_en is asserted to the same address as
//     rd_addr in the same cycle, the written data is forwarded to rd_data
//     one cycle later (write-first semantics).
//   * waitrequest is always 0.

module coldfoot_mem_bytelane_sync #(
    /* verilator lint_off VARHIDDEN */
    parameter int unsigned DATA_W     = 8, /* verilator lint_on VARHIDDEN */
    parameter int unsigned DEPTH      = 16,
    parameter int unsigned BYTE_LANES = (DATA_W + 7) / 8,
    parameter int unsigned ADDR_W     = (DEPTH <= 1) ? 1 : $clog2(DEPTH),
    parameter int unsigned STYLE_HINT = 0
) (
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   ena,
    input  wire  [ADDR_W-1:0]     rd_addr,
    output logic [DATA_W-1:0]     rd_data,
    input  wire                   wr_en,
    input  wire  [ADDR_W-1:0]     wr_addr,
    input  wire  [DATA_W-1:0]     wr_data,
    input  wire  [BYTE_LANES-1:0] wr_byte_en,
    output logic                  waitrequest
);
    assign waitrequest = 1'b0;

    // FF array with byte-enable merge.
    logic [DATA_W-1:0] mem [0:DEPTH-1];
    logic [DATA_W-1:0] merged_wr_data_c;
    integer byte_idx;
    integer init_idx;

    always_comb begin
        merged_wr_data_c = mem[wr_addr];
        for (byte_idx = 0; byte_idx < BYTE_LANES; byte_idx = byte_idx + 1) begin
            if (wr_byte_en[byte_idx]) begin
                merged_wr_data_c[byte_idx*8 +: 8] = wr_data[byte_idx*8 +: 8];
            end
        end
    end

    initial begin
        for (init_idx = 0; init_idx < DEPTH; init_idx++) begin
            mem[init_idx] = '0;
        end
        rd_data = '0;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_data <= '0;
        end else if (ena) begin
            if (wr_en) begin
                mem[wr_addr] <= merged_wr_data_c;
            end
            if (wr_en && (wr_addr == rd_addr)) begin
                rd_data <= merged_wr_data_c;
            end else begin
                rd_data <= mem[rd_addr];
            end
        end
    end

endmodule

`default_nettype wire
