`default_nettype none

// tile_mem_1r1w_sync — synchronous 1-read/1-write FF register file.
// STYLE_HINT is accepted for interface compatibility but ignored;
// all instances synthesize to flip-flop arrays with write-first bypass.

module tile_mem_1r1w_sync #(
    parameter int unsigned DATA_W = 8,
    parameter int unsigned DEPTH = 16,
    parameter int unsigned ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH),
    parameter int unsigned STYLE_HINT = 0
) (
    input  wire                   clk,
    input  wire                   rd_en,
    input  wire [ADDR_W-1:0]      rd_addr,
    output logic [DATA_W-1:0]     rd_data,
    input  wire                   wr_en,
    input  wire [ADDR_W-1:0]      wr_addr,
    input  wire [DATA_W-1:0]      wr_data
);
    // FF register file — write-first bypass (simultaneous same-address
    // read/write returns the written data one cycle later).
    logic [DATA_W-1:0] mem [0:DEPTH-1];
    integer i;

    initial begin
        for (i = 0; i < DEPTH; i++) mem[i] = '0;
        rd_data = '0;
    end

    always_ff @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
        if (rd_en) begin
            if (wr_en && (wr_addr == rd_addr)) begin
                rd_data <= wr_data;
            end else begin
                rd_data <= mem[rd_addr];
            end
        end
    end

endmodule

