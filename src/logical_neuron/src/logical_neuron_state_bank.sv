`default_nettype none

`include "tile_flit_types.vh"

module logical_neuron_state_bank_packed_bram #(
    parameter int unsigned DATA_W = 8,
    parameter int unsigned DEPTH = 16,
    parameter int unsigned BYTE_LANES = (DATA_W + 7) / 8
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         ena,
    input  wire [((DEPTH <= 1) ? 1 : $clog2(DEPTH))-1:0] rd_addr,
    output logic [DATA_W-1:0]           rd_data,
    input  wire                         wr_en,
    input  wire [((DEPTH <= 1) ? 1 : $clog2(DEPTH))-1:0] wr_addr,
    input  wire [DATA_W-1:0]            wr_data,
    input  wire [BYTE_LANES-1:0]        wr_byte_en
);
    logic [DATA_W-1:0] rd_data_raw;
    logic [DATA_W-1:0] rd_data_bypass_c;
    integer byte_idx;
    integer init_idx;

    (* ram_style = "block" *) logic [DATA_W-1:0] mem [0:DEPTH-1];
    logic [DATA_W-1:0] merged_wr_data_c;

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

(* keep_hierarchy = "no" *)
module logical_neuron_state_bank #(
    parameter int unsigned NEURONS_PER_TILE = 1,
    // Named constant for the default outbound-connections-per-neuron
    // multiplier so the magic 24 is documented and not scattered across
    // callers.  Actual pool size is overridden at instantiation time
    // (e.g. chip_core passes FANOUT_POOL_DEPTH=128 explicitly).
    parameter int unsigned DEFAULT_FANOUT_PER_NEURON = 24,
    parameter int unsigned FANOUT_POOL_DEPTH = NEURONS_PER_TILE * DEFAULT_FANOUT_PER_NEURON,
    // Memory style for the five BRAM sub-memories (dispatch / fanout /
    // dump / target / reset).  Applies to coldfoot_mem_bytelane_sync.
    //   0 = AUTO (flip-flop array — compact FF storage, all async reads)
    //   1 = DISTRIBUTED (same as 0 for this module)
    //   2 = BLOCK (Xilinx xpm_memory_sdpram BRAMs)
    //   3 = (ignored, maps to FF path)
    // All styles map to the FF path in this implementation.
    parameter int unsigned MEM_STYLE_HINT = 2,
    // When nonzero, all neurons share one ucode bank (start address = 0).
    // Eliminates the per-neuron ucode_ptr byte from the dispatch and dump
    // BRAM rows and hardwires dispatch_ucode_ptr / dump_ucode_ptr to 5'b0,
    // reducing BRAM storage by 2 × NEURONS_PER_TILE bytes and removing the
    // per-neuron ucode_ptr FF array on compact-FF paths.
    parameter int unsigned UCODE_SHARED_BANK = 0,
    // When nonzero, all neurons share one init-config register (8 bytes).
    // Stores V_MEM_INIT, SYN_INIT, AUX_INIT, UCODE_LEN; eliminates per-neuron
    // copies of these fields from the unified FF memory.  Combined with
    // UCODE_SHARED_BANK=1, removes 4 bytes×N from the state row.
    // For NEURONS_PER_TILE=4: saves 4×4=16B=128 FFs in the BRAM path.
    parameter int unsigned SHARED_NEURON_CONFIG = 0,
    // When nonzero, the per-neuron reset-state BRAM is omitted.  Soft-reset
    // always restores neurons to all-zero initial state.  Saves
    // NEURONS_PER_TILE × RESET_BRAM_BYTES bytes of storage.
    parameter int unsigned ASIC_ZERO_INIT_RESET = 0,
    // Section 8.7.1 switch: drop the redundant fanout BRAM image.
    // Runtime fanout ptr/len already comes from dedicated async FF arrays.
    parameter bit DROP_REDUNDANT_FANOUT_BRAM = 1'b0,
    // Section 8.7.2 switch: remove fanout ptr/len byte duplication from
    // dump/target BRAM rows. Fanout ptr/len outputs remain sourced from FFs.
    parameter bit STATE_ROW_NO_FANOUT_BYTES = 1'b0,
    // Section 8.7.3 switch: drop dispatch BRAM when compact-FF paths already
    // provide authoritative dispatch outputs (STYLE 2 path is unchanged).
    parameter bit DROP_DISPATCH_BRAM_ON_COMPACT = 1'b0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,
    input  wire csr_write_en,
    // Neuron index for CSR write (replaces N-wide csr_write_mask).
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] csr_write_addr,
    // Broadcast flag: when set with CSR_RESET_TRIGGER, all neurons are added
    // to the soft-reset drain queue (no effect on other CSR addresses).
    input  wire csr_write_broadcast,
    input  wire [3:0] csr_addr,
    input  wire [7:0] csr_data,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] target_read_idx,
    output logic [7:0] target_csr_read_data,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] target_fanout_ptr,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] target_fanout_len,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] dispatch_read_idx,
    output logic [4:0] dispatch_ucode_ptr,
    output logic [4:0] dispatch_ucode_len,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] dispatch_fanout_len,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] fanout_read_idx,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] fanout_read_ptr,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] fanout_read_len,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] commit_read_idx,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] commit_fanout_ptr,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] dump_read_idx,
    output logic [23:0] dump_init_rf_flat,
    output logic [4:0] dump_ucode_ptr,
    output logic [4:0] dump_ucode_len,
    output logic [7:0] dump_neuron_flags,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] dump_fanout_ptr,
    output logic [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] dump_fanout_len,
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] reset_read_idx,
    output logic [23:0] reset_init_rf_flat,
    // reset_last_event_time removed: neutern does not use timestamps
    output logic [NEURONS_PER_TILE-1:0] soft_reset_pulse,  // LEGACY: removed in scalar refactor — tied to 0
    output logic soft_reset_valid,
    output logic [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] soft_reset_idx,
    output logic [NEURONS_PER_TILE-1:0] host_egress_en_bus,
    output logic [NEURONS_PER_TILE-1:0] noc_egress_en_bus,
    input  wire ucode_prog_en,
    // ucode_prog_addr replaces N-wide ucode_prog_mask.
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] ucode_prog_addr,
    input  wire [4:0] ucode_prog_ptr,
    input  wire [4:0] ucode_prog_len,
    input  wire fanout_prog_ptr_en,
    input  wire fanout_prog_len_en,
    // fanout_prog_addr replaces N-wide fanout_prog_mask.
    input  wire [((NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE))-1:0] fanout_prog_addr,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] fanout_prog_ptr,
    input  wire [((FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH))-1:0] fanout_prog_len,
    // mem_stall: OR of waitrequest from all internal BRAM instances.
    // Always 0 (FF path).
    output logic mem_stall
);
    localparam int unsigned NEURON_IDX_W =
        (NEURONS_PER_TILE <= 1) ? 1 : $clog2(NEURONS_PER_TILE);
    localparam int unsigned FANOUT_PTR_W =
        (FANOUT_POOL_DEPTH <= 1) ? 1 : $clog2(FANOUT_POOL_DEPTH);
    localparam int unsigned FANOUT_PTR_BYTES = (FANOUT_PTR_W + 7) / 8;

    // All state-bank storage uses FF-backed coldfoot_mem_bytelane_sync instances.
    // USE_BRAM=1 and USE_COMPACT_FF=0 are permanently fixed; generate-if blocks
    // that conditioned on these flags have been inlined to their active branch.

    // When UCODE_SHARED_BANK!=0 the per-neuron ucode_ptr byte is omitted from
    // both the dispatch and dump BRAM rows, saving one byte per row per memory.
    localparam int unsigned UCODE_PTR_BRAM_BYTES = (UCODE_SHARED_BANK != 0) ? 0 : 1;
    localparam int unsigned DISPATCH_BRAM_BYTES = 1 + UCODE_PTR_BRAM_BYTES + FANOUT_PTR_BYTES;
    localparam int unsigned DISPATCH_BRAM_W = DISPATCH_BRAM_BYTES * 8;
    // When DROP_REDUNDANT_FANOUT_BRAM=1 the FANOUT section is removed from the row;
    // fanout_ptr and fanout_len are extracted from the COMMIT and DISPATCH sections
    // respectively during the phase-1 read (both values already live there).
    // This saves 2×FANOUT_PTR_BYTES per neuron row (4 bytes for Z=512).
    // FANOUT_BRAM_BYTES_EFF: actual bytes contributed to the row (0 when DROP=1).
    // FANOUT_BRAM_BYTES: always sized for 2×FANOUT_PTR_BYTES so the else branch
    // in always_comb field extraction stays type-valid under Slang analysis even
    // when DROP_REDUNDANT_FANOUT_BRAM=1 (dead branch, but still type-checked).
    localparam int unsigned FANOUT_BRAM_BYTES_EFF = DROP_REDUNDANT_FANOUT_BRAM
                                                    ? 0 : (2 * FANOUT_PTR_BYTES);
    localparam int unsigned FANOUT_BRAM_BYTES     = 2 * FANOUT_PTR_BYTES;
    localparam int unsigned FANOUT_BRAM_W         = FANOUT_BRAM_BYTES * 8;
    localparam int unsigned DUMP_FANOUT_BRAM_BYTES =
        STATE_ROW_NO_FANOUT_BYTES ? 0 : (2 * FANOUT_PTR_BYTES);
    // DUMP_BRAM_BYTES now 5 per row (V_MEM, SYN, AUX, UCODE_LEN, FLAGS) with UCODE_PTR_BRAM_BYTES.
    localparam int unsigned DUMP_BRAM_BYTES = 5 + UCODE_PTR_BRAM_BYTES + DUMP_FANOUT_BRAM_BYTES;
    localparam int unsigned DUMP_BRAM_W = DUMP_BRAM_BYTES * 8;

    // DISPATCH row layout (ucode_ptr byte present only when UCODE_PTR_BRAM_BYTES==1):
    localparam int unsigned DISPATCH_BYTE_UCODE_PTR  = 0;   // valid only when UCODE_PTR_BRAM_BYTES==1
    localparam int unsigned DISPATCH_BYTE_UCODE_LEN  = UCODE_PTR_BRAM_BYTES;
    localparam int unsigned DISPATCH_BYTE_FANOUT_LEN = 1 + UCODE_PTR_BRAM_BYTES;

    localparam int unsigned FANOUT_BYTE_PTR = 0;
    localparam int unsigned FANOUT_BYTE_LEN = FANOUT_PTR_BYTES;

    // DUMP row layout (ucode_ptr byte present only when UCODE_PTR_BRAM_BYTES==1):
    // rf[3]=refractory byte removed; AUX renumbered to byte 2.
    localparam int unsigned DUMP_BYTE_V_MEM      = 0;
    localparam int unsigned DUMP_BYTE_SYN        = 1;
    localparam int unsigned DUMP_BYTE_AUX        = 2;  // was byte 3; rf[3]/REFR byte removed
    localparam int unsigned DUMP_BYTE_UCODE_PTR  = 3;   // valid only when UCODE_PTR_BRAM_BYTES==1
    localparam int unsigned DUMP_BYTE_UCODE_LEN  = 3 + UCODE_PTR_BRAM_BYTES;
    localparam int unsigned DUMP_BYTE_FLAGS      = 4 + UCODE_PTR_BRAM_BYTES;
    // Offsets below are only meaningful when STATE_ROW_NO_FANOUT_BYTES==0.
    localparam int unsigned DUMP_BYTE_FANOUT_PTR = 5 + UCODE_PTR_BRAM_BYTES;
    localparam int unsigned DUMP_BYTE_FANOUT_LEN = DUMP_BYTE_FANOUT_PTR + FANOUT_PTR_BYTES;

    // TARGET = DUMP + no extra last_time (timestamps removed).
    localparam int unsigned TARGET_BRAM_BYTES = DUMP_BRAM_BYTES;
    localparam int unsigned TARGET_BRAM_W = TARGET_BRAM_BYTES * 8;

    // RESET row: V_MEM, SYN, AUX only (rf[3]/REFR and last_time both removed).
    localparam int unsigned RESET_BYTE_V_MEM = 0;
    localparam int unsigned RESET_BYTE_SYN = 1;
    localparam int unsigned RESET_BYTE_AUX = 2;
    localparam int unsigned RESET_BRAM_BYTES = 3;
    localparam int unsigned RESET_BRAM_W = RESET_BRAM_BYTES * 8;

    // USE_BRAM and USE_COMPACT_FF are permanently fixed for this implementation
    // (all memory styles use the unified coldfoot_mem_bytelane_sync FF path).
    localparam bit USE_BRAM       = 1'b1;  // always use the unified FF memory
    localparam bit USE_COMPACT_FF = 1'b0;  // compact-FF arrays disabled; BRAM path active

    logic state_epoch_r;
    logic [NEURONS_PER_TILE-1:0] state_epoch_seen_r;
    logic csr_selected_valid_c;
    logic [NEURON_IDX_W-1:0] csr_selected_idx_c;
    logic ucode_prog_selected_valid_c;
    logic [NEURON_IDX_W-1:0] ucode_prog_selected_idx_c;
    logic fanout_prog_selected_valid_c;
    logic [NEURON_IDX_W-1:0] fanout_prog_selected_idx_c;
    logic csr_write_hit_c;
    // CSR_RESET_TRIGGER drain queue.  A broadcast write to this addr
    // ORs `csr_write_mask` into `reset_drain_mask_r`; a one-hot pulse
    // is then emitted per cycle (lowest bit first) on `soft_reset_pulse`
    // until the mask is empty.  This keeps the downstream one-hot
    // contract (tile_dispatch_scheduler / context_bank single write port)
    // intact while letting the host trigger a tile-wide reset with one
    // packet instead of one packet per neuron.
    logic [NEURONS_PER_TILE-1:0] reset_drain_mask_r;
    logic [NEURONS_PER_TILE-1:0] reset_drain_pulse_c;
    logic                        reset_drain_pulse_valid_c;
    logic                        reset_drain_emit_c;
    logic [NEURON_IDX_W-1:0]     drain_idx_c;
    logic                        drain_valid_c;
    logic                        soft_reset_valid_next_c;
    logic [NEURON_IDX_W-1:0]     soft_reset_idx_next_c;
    logic [NEURONS_PER_TILE-1:0] reset_trigger_mask_or_c;
    logic [NEURONS_PER_TILE-1:0] soft_reset_pulse_next_c;
    logic [NEURONS_PER_TILE-1:0] state_epoch_write_mask_c;
    logic ucode_ptr_wr_en_c;
    logic [NEURON_IDX_W-1:0] ucode_ptr_wr_idx_c;
    logic [4:0] ucode_ptr_wr_data_c;
    logic ucode_len_wr_en_c;
    logic [NEURON_IDX_W-1:0] ucode_len_wr_idx_c;
    logic [4:0] ucode_len_wr_data_c;
    logic fanout_ptr_wr_en_c;
    logic [NEURON_IDX_W-1:0] fanout_ptr_wr_idx_c;
    logic [FANOUT_PTR_W-1:0] fanout_ptr_wr_data_c;
    logic fanout_len_wr_en_c;
    logic [NEURON_IDX_W-1:0] fanout_len_wr_idx_c;
    logic [FANOUT_PTR_W-1:0] fanout_len_wr_data_c;
    integer idx;
    integer d;
    integer b;
    integer byte_idx;

    logic dispatch_bram_wr_en_c;
    logic [NEURON_IDX_W-1:0] dispatch_bram_wr_addr_c;
    logic [DISPATCH_BRAM_W-1:0] dispatch_bram_wr_data_c;
    logic [DISPATCH_BRAM_BYTES-1:0] dispatch_bram_wr_mask_c;
    logic [DISPATCH_BRAM_W-1:0] dispatch_bram_rd_data;

    logic fanout_bram_wr_en_c;
    logic [NEURON_IDX_W-1:0] fanout_bram_wr_addr_c;
    logic [FANOUT_BRAM_W-1:0] fanout_bram_wr_data_c;
    logic [FANOUT_BRAM_BYTES-1:0] fanout_bram_wr_mask_c;
    logic [FANOUT_BRAM_W-1:0] fanout_bram_rd_data;
    // Compact fanout: only valid when DROP_REDUNDANT_FANOUT_BRAM=1.
    // ptr extracted from COMMIT section, len from DISPATCH section at phase-1 latch time.
    logic [FANOUT_PTR_BYTES*8-1:0] fanout_compact_ptr;
    logic [FANOUT_PTR_BYTES*8-1:0] fanout_compact_len;

    logic dump_bram_wr_en_c;
    logic [NEURON_IDX_W-1:0] dump_bram_wr_addr_c;
    logic [DUMP_BRAM_W-1:0] dump_bram_wr_data_c;
    logic [DUMP_BRAM_BYTES-1:0] dump_bram_wr_mask_c;
    logic [DUMP_BRAM_W-1:0] dump_bram_rd_data;

    logic target_bram_wr_en_c;
    logic [NEURON_IDX_W-1:0] target_bram_wr_addr_c;
    logic [TARGET_BRAM_W-1:0] target_bram_wr_data_c;
    logic [TARGET_BRAM_BYTES-1:0] target_bram_wr_mask_c;
    logic [TARGET_BRAM_W-1:0] target_bram_rd_data;

    logic reset_bram_wr_en_c;
    logic [NEURON_IDX_W-1:0] reset_bram_wr_addr_c;
    logic [RESET_BRAM_W-1:0] reset_bram_wr_data_c;
    logic [RESET_BRAM_BYTES-1:0] reset_bram_wr_mask_c;
    logic [RESET_BRAM_W-1:0] reset_bram_rd_data;

    logic [FANOUT_PTR_BYTES*8-1:0] commit_ptr_bram_rd_data;

    logic dispatch_epoch_valid_r;
    logic fanout_epoch_valid_r;
    logic dump_epoch_valid_r;
    logic target_epoch_valid_r;
    logic reset_epoch_valid_r;
    logic commit_epoch_valid_r;
    logic [FANOUT_PTR_BYTES*8-1:0] fanout_ptr_wr_padded_c;
    logic [FANOUT_PTR_BYTES*8-1:0] fanout_len_wr_padded_c;

    function automatic [NEURON_IDX_W-1:0] first_hot_idx_removed_placeholder(
        input logic [0:0] dummy, output logic found
    );
        begin first_hot_idx_removed_placeholder = '0; found = 1'b0; end
    endfunction

    always_comb begin
        fanout_ptr_wr_padded_c = '0;
        fanout_ptr_wr_padded_c[FANOUT_PTR_W-1:0] = fanout_ptr_wr_data_c;
        fanout_len_wr_padded_c = '0;
        fanout_len_wr_padded_c[FANOUT_PTR_W-1:0] = fanout_len_wr_data_c;
    end

    always_comb begin
        // Direct address decode — three priority encoders removed.
        // csr_write_addr is already the neuron index from the caller.
        csr_selected_valid_c         = csr_write_en;
        csr_selected_idx_c           = csr_write_addr;
        ucode_prog_selected_valid_c  = ucode_prog_en;
        ucode_prog_selected_idx_c    = ucode_prog_addr;
        fanout_prog_selected_valid_c = (fanout_prog_ptr_en || fanout_prog_len_en);
        fanout_prog_selected_idx_c   = fanout_prog_addr;
        csr_write_hit_c              = csr_write_en;

        // Drain: find lowest set bit index (scalar, no N-wide one-hot bus).
        drain_idx_c   = '0;
        drain_valid_c = 1'b0;
        for (int d = 0; d < NEURONS_PER_TILE; d = d + 1) begin
            if (!drain_valid_c && reset_drain_mask_r[d]) begin
                drain_idx_c   = NEURON_IDX_W'(d);
                drain_valid_c = 1'b1;
            end
        end

        // Drain emit: fire when we have a pending drain and CSR_CTRL didn't preempt.
        reset_drain_emit_c = drain_valid_c &&
                             !(csr_write_hit_c && (csr_addr == CSR_CTRL));

        // Scalar soft-reset output: CSR_CTRL per-neuron reset takes priority over drain.
        soft_reset_valid_next_c = 1'b0;
        soft_reset_idx_next_c   = '0;
        if (csr_write_hit_c && (csr_addr == CSR_CTRL) && csr_data[CSR_CTRL_SOFT_RESET_BIT]) begin
            soft_reset_valid_next_c = 1'b1;
            soft_reset_idx_next_c   = csr_selected_idx_c;
        end else if (reset_drain_emit_c) begin
            soft_reset_valid_next_c = 1'b1;
            soft_reset_idx_next_c   = drain_idx_c;
        end

        // state_epoch_write_mask_c removed — writes are done via direct
        // index in the sequential block below.
        if (csr_write_hit_c) begin
            // epoch write for CSR path handled in sequential block (direct index).
        end
        // (ucode/fanout prog epoch writes also handled in sequential block)

        ucode_ptr_wr_en_c = 1'b0;
        ucode_ptr_wr_idx_c = '0;
        ucode_ptr_wr_data_c = '0;
        // When UCODE_SHARED_BANK!=0, ucode_ptr is always 0 for every neuron.
        // Skip all writes so the per-neuron ptr storage can be optimised away.
        if (UCODE_SHARED_BANK == 0) begin
            if (ucode_prog_en && ucode_prog_selected_valid_c) begin
                ucode_ptr_wr_en_c = 1'b1;
                ucode_ptr_wr_idx_c = ucode_prog_selected_idx_c;
                ucode_ptr_wr_data_c = ucode_prog_ptr;
            end else if (csr_write_hit_c && (csr_addr == CSR_UCODE_PTR)) begin
                ucode_ptr_wr_en_c = 1'b1;
                ucode_ptr_wr_idx_c = csr_selected_idx_c;
                ucode_ptr_wr_data_c = csr_data[4:0];
            end
        end

        ucode_len_wr_en_c = 1'b0;
        ucode_len_wr_idx_c = '0;
        ucode_len_wr_data_c = '0;
        if (ucode_prog_en && ucode_prog_selected_valid_c) begin
            ucode_len_wr_en_c = 1'b1;
            ucode_len_wr_idx_c = ucode_prog_selected_idx_c;
            ucode_len_wr_data_c = ucode_prog_len;
        end else if (csr_write_hit_c && (csr_addr == CSR_UCODE_LEN)) begin
            ucode_len_wr_en_c = 1'b1;
            ucode_len_wr_idx_c = csr_selected_idx_c;
            ucode_len_wr_data_c = csr_data[4:0];
        end

        fanout_ptr_wr_en_c = 1'b0;
        fanout_ptr_wr_idx_c = '0;
        fanout_ptr_wr_data_c = '0;
        if (fanout_prog_ptr_en && fanout_prog_selected_valid_c) begin
            fanout_ptr_wr_en_c = 1'b1;
            fanout_ptr_wr_idx_c = fanout_prog_selected_idx_c;
            fanout_ptr_wr_data_c = fanout_prog_ptr;
        end else if (csr_write_hit_c && (csr_addr == CSR_VEC_BASE_23)) begin
            fanout_ptr_wr_en_c = 1'b1;
            fanout_ptr_wr_idx_c = csr_selected_idx_c;
            fanout_ptr_wr_data_c = FANOUT_PTR_W'(csr_data);
        end

        fanout_len_wr_en_c = 1'b0;
        fanout_len_wr_idx_c = '0;
        fanout_len_wr_data_c = '0;
        if (fanout_prog_len_en && fanout_prog_selected_valid_c) begin
            fanout_len_wr_en_c = 1'b1;
            fanout_len_wr_idx_c = fanout_prog_selected_idx_c;
            fanout_len_wr_data_c = fanout_prog_len;
        end
    end

    always_comb begin
        dispatch_bram_wr_en_c = 1'b0;
        dispatch_bram_wr_addr_c = '0;
        dispatch_bram_wr_data_c = '0;
        dispatch_bram_wr_mask_c = '0;
        // Section 8.7.3: dispatch-BRAM write (USE_COMPACT_FF=0, so always active).
        if (ucode_ptr_wr_en_c || ucode_len_wr_en_c) begin
            dispatch_bram_wr_en_c = 1'b1;
            dispatch_bram_wr_addr_c = ucode_ptr_wr_en_c ? ucode_ptr_wr_idx_c : ucode_len_wr_idx_c;
            if (ucode_ptr_wr_en_c) begin
                dispatch_bram_wr_mask_c[DISPATCH_BYTE_UCODE_PTR] = 1'b1;
                dispatch_bram_wr_data_c[DISPATCH_BYTE_UCODE_PTR*8 +: 8] = {3'b000, ucode_ptr_wr_data_c};
            end
            if (ucode_len_wr_en_c) begin
                dispatch_bram_wr_mask_c[DISPATCH_BYTE_UCODE_LEN] = 1'b1;
                dispatch_bram_wr_data_c[DISPATCH_BYTE_UCODE_LEN*8 +: 8] = {3'b000, ucode_len_wr_data_c};
            end
        end else if (fanout_len_wr_en_c) begin
            dispatch_bram_wr_en_c = 1'b1;
            dispatch_bram_wr_addr_c = fanout_len_wr_idx_c;
            for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                dispatch_bram_wr_mask_c[DISPATCH_BYTE_FANOUT_LEN + byte_idx] = 1'b1;
                dispatch_bram_wr_data_c[(DISPATCH_BYTE_FANOUT_LEN + byte_idx)*8 +: 8] =
                    fanout_len_wr_padded_c[byte_idx*8 +: 8];
            end
        end
    end

    always_comb begin
        fanout_bram_wr_en_c = 1'b0;
        fanout_bram_wr_addr_c = '0;
        fanout_bram_wr_data_c = '0;
        fanout_bram_wr_mask_c = '0;
        // Section 8.7.1: optional removal of redundant fanout BRAM image.
        if (!DROP_REDUNDANT_FANOUT_BRAM) begin
            if (fanout_ptr_wr_en_c || fanout_len_wr_en_c) begin
                fanout_bram_wr_en_c = 1'b1;
                fanout_bram_wr_addr_c = fanout_ptr_wr_en_c ? fanout_ptr_wr_idx_c : fanout_len_wr_idx_c;
                if (fanout_ptr_wr_en_c) begin
                    for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                        fanout_bram_wr_mask_c[FANOUT_BYTE_PTR + byte_idx] = 1'b1;
                        fanout_bram_wr_data_c[(FANOUT_BYTE_PTR + byte_idx)*8 +: 8] =
                            fanout_ptr_wr_padded_c[byte_idx*8 +: 8];
                    end
                end
                if (fanout_len_wr_en_c) begin
                    for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                        fanout_bram_wr_mask_c[FANOUT_BYTE_LEN + byte_idx] = 1'b1;
                        fanout_bram_wr_data_c[(FANOUT_BYTE_LEN + byte_idx)*8 +: 8] =
                            fanout_len_wr_padded_c[byte_idx*8 +: 8];
                    end
                end
            end
        end
    end

    always_comb begin
        dump_bram_wr_en_c = 1'b0;
        dump_bram_wr_addr_c = '0;
        dump_bram_wr_data_c = '0;
        dump_bram_wr_mask_c = '0;
        if (ucode_ptr_wr_en_c || ucode_len_wr_en_c) begin
            dump_bram_wr_en_c = 1'b1;
            dump_bram_wr_addr_c = ucode_ptr_wr_en_c ? ucode_ptr_wr_idx_c : ucode_len_wr_idx_c;
            if (ucode_ptr_wr_en_c) begin
                dump_bram_wr_mask_c[DUMP_BYTE_UCODE_PTR] = 1'b1;
                dump_bram_wr_data_c[DUMP_BYTE_UCODE_PTR*8 +: 8] = {3'b000, ucode_ptr_wr_data_c};
            end
            if (ucode_len_wr_en_c) begin
                dump_bram_wr_mask_c[DUMP_BYTE_UCODE_LEN] = 1'b1;
                dump_bram_wr_data_c[DUMP_BYTE_UCODE_LEN*8 +: 8] = {3'b000, ucode_len_wr_data_c};
            end
        end else if (!STATE_ROW_NO_FANOUT_BYTES && (fanout_ptr_wr_en_c || fanout_len_wr_en_c)) begin
            dump_bram_wr_en_c = 1'b1;
            dump_bram_wr_addr_c = fanout_ptr_wr_en_c ? fanout_ptr_wr_idx_c : fanout_len_wr_idx_c;
            if (fanout_ptr_wr_en_c) begin
                for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                    dump_bram_wr_mask_c[DUMP_BYTE_FANOUT_PTR + byte_idx] = 1'b1;
                    dump_bram_wr_data_c[(DUMP_BYTE_FANOUT_PTR + byte_idx)*8 +: 8] =
                        fanout_ptr_wr_padded_c[byte_idx*8 +: 8];
                end
            end
            if (fanout_len_wr_en_c) begin
                for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                    dump_bram_wr_mask_c[DUMP_BYTE_FANOUT_LEN + byte_idx] = 1'b1;
                    dump_bram_wr_data_c[(DUMP_BYTE_FANOUT_LEN + byte_idx)*8 +: 8] =
                        fanout_len_wr_padded_c[byte_idx*8 +: 8];
                end
            end
        end else if (csr_write_hit_c) begin
            dump_bram_wr_addr_c = csr_selected_idx_c;
            case (csr_addr)
                CSR_INIT_VI, CSR_INIT_TR, CSR_INIT_WAUX, CSR_VEC_BASE_01:
                    dump_bram_wr_en_c = 1'b1;
                default:
                    dump_bram_wr_en_c = 1'b0;
            endcase
            case (csr_addr)
                CSR_INIT_VI: begin
                    dump_bram_wr_mask_c[DUMP_BYTE_V_MEM] = 1'b1;
                    dump_bram_wr_data_c[DUMP_BYTE_V_MEM*8 +: 8] = csr_data;
                end
                CSR_INIT_TR: begin
                    dump_bram_wr_mask_c[DUMP_BYTE_SYN] = 1'b1;
                    dump_bram_wr_data_c[DUMP_BYTE_SYN*8 +: 8] = csr_data;
                end
                // CSR_INIT_T01 removed (rf[3]=refractory eliminated)
                CSR_INIT_WAUX: begin
                    dump_bram_wr_mask_c[DUMP_BYTE_AUX] = 1'b1;
                    dump_bram_wr_data_c[DUMP_BYTE_AUX*8 +: 8] = csr_data;
                end
                CSR_VEC_BASE_01: begin
                    dump_bram_wr_mask_c[DUMP_BYTE_FLAGS] = 1'b1;
                    dump_bram_wr_data_c[DUMP_BYTE_FLAGS*8 +: 8] = csr_data;
                end
                default: begin
                end
            endcase
        end
    end

    always_comb begin
        target_bram_wr_en_c = 1'b0;
        target_bram_wr_addr_c = '0;
        target_bram_wr_data_c = '0;
        target_bram_wr_mask_c = '0;
        if (ucode_ptr_wr_en_c || ucode_len_wr_en_c) begin
            target_bram_wr_en_c = 1'b1;
            target_bram_wr_addr_c = ucode_ptr_wr_en_c ? ucode_ptr_wr_idx_c : ucode_len_wr_idx_c;
            if (ucode_ptr_wr_en_c) begin
                target_bram_wr_mask_c[DUMP_BYTE_UCODE_PTR] = 1'b1;
                target_bram_wr_data_c[DUMP_BYTE_UCODE_PTR*8 +: 8] = {3'b000, ucode_ptr_wr_data_c};
            end
            if (ucode_len_wr_en_c) begin
                target_bram_wr_mask_c[DUMP_BYTE_UCODE_LEN] = 1'b1;
                target_bram_wr_data_c[DUMP_BYTE_UCODE_LEN*8 +: 8] = {3'b000, ucode_len_wr_data_c};
            end
        end else if (!STATE_ROW_NO_FANOUT_BYTES && (fanout_ptr_wr_en_c || fanout_len_wr_en_c)) begin
            target_bram_wr_en_c = 1'b1;
            target_bram_wr_addr_c = fanout_ptr_wr_en_c ? fanout_ptr_wr_idx_c : fanout_len_wr_idx_c;
            if (fanout_ptr_wr_en_c) begin
                for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                    target_bram_wr_mask_c[DUMP_BYTE_FANOUT_PTR + byte_idx] = 1'b1;
                    target_bram_wr_data_c[(DUMP_BYTE_FANOUT_PTR + byte_idx)*8 +: 8] =
                        fanout_ptr_wr_padded_c[byte_idx*8 +: 8];
                end
            end
            if (fanout_len_wr_en_c) begin
                for (int byte_idx = 0; byte_idx < FANOUT_PTR_BYTES; byte_idx++) begin
                    target_bram_wr_mask_c[DUMP_BYTE_FANOUT_LEN + byte_idx] = 1'b1;
                    target_bram_wr_data_c[(DUMP_BYTE_FANOUT_LEN + byte_idx)*8 +: 8] =
                        fanout_len_wr_padded_c[byte_idx*8 +: 8];
                end
            end
        end else if (csr_write_hit_c) begin
            target_bram_wr_addr_c = csr_selected_idx_c;
            case (csr_addr)
                CSR_INIT_VI, CSR_INIT_TR, CSR_INIT_WAUX, CSR_VEC_BASE_01:
                    target_bram_wr_en_c = 1'b1;
                default:
                    target_bram_wr_en_c = 1'b0;
            endcase
            case (csr_addr)
                CSR_INIT_VI: begin
                    target_bram_wr_mask_c[DUMP_BYTE_V_MEM] = 1'b1;
                    target_bram_wr_data_c[DUMP_BYTE_V_MEM*8 +: 8] = csr_data;
                end
                CSR_INIT_TR: begin
                    target_bram_wr_mask_c[DUMP_BYTE_SYN] = 1'b1;
                    target_bram_wr_data_c[DUMP_BYTE_SYN*8 +: 8] = csr_data;
                end
                // CSR_INIT_T01 removed (rf[3] eliminated)
                CSR_INIT_WAUX: begin
                    target_bram_wr_mask_c[DUMP_BYTE_AUX] = 1'b1;
                    target_bram_wr_data_c[DUMP_BYTE_AUX*8 +: 8] = csr_data;
                end
                CSR_VEC_BASE_01: begin
                    target_bram_wr_mask_c[DUMP_BYTE_FLAGS] = 1'b1;
                    target_bram_wr_data_c[DUMP_BYTE_FLAGS*8 +: 8] = csr_data;
                end
                // CSR_NEURON_META removed (last_time eliminated)
                default: begin
                end
            endcase
        end
    end

    always_comb begin
        reset_bram_wr_en_c = 1'b0;
        reset_bram_wr_addr_c = '0;
        reset_bram_wr_data_c = '0;
        reset_bram_wr_mask_c = '0;
        if (csr_write_hit_c) begin
            reset_bram_wr_addr_c = csr_selected_idx_c;
            case (csr_addr)
                CSR_INIT_VI, CSR_INIT_TR, CSR_INIT_WAUX:
                    reset_bram_wr_en_c = 1'b1;
                default:
                    reset_bram_wr_en_c = 1'b0;
            endcase
            case (csr_addr)
                CSR_INIT_VI: begin
                    reset_bram_wr_mask_c[RESET_BYTE_V_MEM] = 1'b1;
                    reset_bram_wr_data_c[RESET_BYTE_V_MEM*8 +: 8] = csr_data;
                end
                CSR_INIT_TR: begin
                    reset_bram_wr_mask_c[RESET_BYTE_SYN] = 1'b1;
                    reset_bram_wr_data_c[RESET_BYTE_SYN*8 +: 8] = csr_data;
                end
                // CSR_INIT_T01 removed (rf[3] eliminated)
                CSR_INIT_WAUX: begin
                    reset_bram_wr_mask_c[RESET_BYTE_AUX] = 1'b1;
                    reset_bram_wr_data_c[RESET_BYTE_AUX*8 +: 8] = csr_data;
                end
                // CSR_NEURON_META removed (last_time eliminated)
                default: begin
                end
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Unified FF register file — single coldfoot_mem_bytelane_sync instance replaces
    // the six separate BRAM sub-memories (dispatch / fanout / commit_ptr /
    // dump / target / reset).
    //
    // Row layout (bytes packed from offset 0):
    //   DISPATCH_BRAM_BYTES    dispatch fields
    //   FANOUT_BRAM_BYTES      fanout ptr/len fields
    //   FANOUT_PTR_BYTES       commit_ptr field
    //   TARGET_BRAM_BYTES      dump+target fields (TARGET ⊇ DUMP; dump reads
    //                          first DUMP_BRAM_BYTES bytes of this section)
    //   EFFECTIVE_RESET_BYTES  reset init fields (0 when ASIC_ZERO_INIT_RESET!=0)
    //
    // Key optimisation: DUMP and TARGET are stored in the SAME byte region.
    // TARGET = DUMP + last_time (one extra byte).  Dump reads extract
    // DUMP_BRAM_BYTES from the section; target reads extract TARGET_BRAM_BYTES.
    // Write logic for both ports writes to ROW_OFF_TARGET, eliminating the
    // 10-byte duplicate DUMP section from the old 6-BRAM layout.
    //
    // Area (GF180 OCD, NEURONS_PER_TILE=512, STYLE_HINT=3, ASIC config):
    //   DISPATCH(3) + FANOUT(4) + COMMIT(2) + TARGET(11) = 20 bytes

    // ---------------------------------------------------------------

    // Row byte offsets.
    localparam int unsigned ROW_OFF_DISPATCH   = 0;
    localparam int unsigned ROW_OFF_FANOUT     = ROW_OFF_DISPATCH + DISPATCH_BRAM_BYTES;
    // ROW_OFF_COMMIT_PTR uses FANOUT_BRAM_BYTES_EFF (0 when DROP=1) so the row shrinks.
    localparam int unsigned ROW_OFF_COMMIT_PTR = ROW_OFF_FANOUT   + FANOUT_BRAM_BYTES_EFF;
    // DUMP and TARGET share the same byte range in the row.
    // TARGET is DUMP + one extra last_time byte, so target_bram_rd_data
    // is TARGET_BRAM_BYTES wide; dump_bram_rd_data is the first DUMP_BRAM_BYTES.
    localparam int unsigned ROW_OFF_TARGET     = ROW_OFF_COMMIT_PTR + FANOUT_PTR_BYTES;
    localparam int unsigned ROW_OFF_DUMP       = ROW_OFF_TARGET;   // alias — same section
    // Reset section: only allocated when ASIC_ZERO_INIT_RESET==0.
    localparam int unsigned EFFECTIVE_RESET_BRAM_BYTES =
        (ASIC_ZERO_INIT_RESET != 0) ? 0 : RESET_BRAM_BYTES;
    localparam int unsigned ROW_OFF_RESET      = ROW_OFF_TARGET   + TARGET_BRAM_BYTES;
    localparam int unsigned STATE_ROW_BYTES    = ROW_OFF_RESET    + EFFECTIVE_RESET_BRAM_BYTES;
    localparam int unsigned STATE_ROW_W        = STATE_ROW_BYTES * 8;

    // ── State storage (USE_BRAM=1 always: FF-backed coldfoot_mem_bytelane_sync) ─

        // ── Shared memory signals ───────────────────────────────────────────
        logic [STATE_ROW_W-1:0]  mem_rd_data;
        logic                    mem_waitreq;

        // Read-phase counter.
        //
        // When DROP_REDUNDANT_FANOUT_BRAM=1, the COMMIT phase is eliminated:
        // in LEAN mode the fanout executor uses the neuron index directly
        // commit_fanout_ptr is unused (DIRECT mode only).
        // Dropping it reduces mem_stall from 4 to 3 cycles per dispatch
        // (25% throughput improvement).
        //
        //   4-phase (DROP=1, ASIC_ZERO_INIT_RESET!=0): dispatch→fanout→dump→target
        //   5-phase (DROP=1, ASIC_ZERO_INIT_RESET==0): dispatch→fanout→dump→target→reset
        //   5-phase (DROP=0, ASIC_ZERO_INIT_RESET!=0): dispatch→fanout→commit→dump→target
        //   6-phase (DROP=0, ASIC_ZERO_INIT_RESET==0): dispatch→fanout→commit→dump→target→reset
        localparam int unsigned NUM_PHASES = DROP_REDUNDANT_FANOUT_BRAM
            ? ((ASIC_ZERO_INIT_RESET != 0) ? 4 : 5)
            : ((ASIC_ZERO_INIT_RESET != 0) ? 5 : 6);
        localparam int unsigned PHASE_W    = 3;
        logic [PHASE_W-1:0] rd_phase_r;

        // Registered per-read-port results (extracted after mem read lands).
        logic [DISPATCH_BRAM_W-1:0]         dispatch_bram_rd_data_r;
        logic [FANOUT_BRAM_W-1:0]           fanout_bram_rd_data_r;  // unused when DROP=1
        logic [FANOUT_PTR_BYTES*8-1:0]      commit_ptr_bram_rd_data_r;
        // Compact path (DROP_REDUNDANT_FANOUT_BRAM=1): ptr and len extracted from
        // COMMIT and DISPATCH byte offsets of the phase-1 fanout_read_idx row read.
        logic [FANOUT_PTR_BYTES*8-1:0]      fanout_compact_ptr_r;
        logic [FANOUT_PTR_BYTES*8-1:0]      fanout_compact_len_r;
        logic [DUMP_BRAM_W-1:0]             dump_bram_rd_data_r;
        logic [TARGET_BRAM_W-1:0]           target_bram_rd_data_r;
        logic [RESET_BRAM_W-1:0]            reset_bram_rd_data_r;

        // Combinational read address: select by current phase.
        logic [NEURON_IDX_W-1:0] mem_rd_addr_c;
        always_comb begin
            case (rd_phase_r)
                3'd0: mem_rd_addr_c = dispatch_read_idx;
                3'd1: mem_rd_addr_c = fanout_read_idx;
                // Phase 2: commit read (non-drop) or dump read (drop — COMMIT phase removed).
                3'd2: mem_rd_addr_c = DROP_REDUNDANT_FANOUT_BRAM ? dump_read_idx   : commit_read_idx;
                // Phase 3: dump read (non-drop) or target read (drop).
                3'd3: mem_rd_addr_c = DROP_REDUNDANT_FANOUT_BRAM ? target_read_idx : dump_read_idx;
                // Phase 4: target read (non-drop) or reset read (drop+ASIC_ZERO_INIT=0 only;
                //           unreachable when DROP_REDUNDANT_FANOUT_BRAM=1 && ASIC_ZERO_INIT!=0).
                3'd4: mem_rd_addr_c = DROP_REDUNDANT_FANOUT_BRAM ? reset_read_idx  : target_read_idx;
                // Phase 5: reset read (6-phase non-drop+ASIC_ZERO_INIT=0 only).
                3'd5: mem_rd_addr_c = reset_read_idx;
                default: mem_rd_addr_c = dispatch_read_idx;
            endcase
        end

        // Write path: merged row with byte-enables for each BRAM write.
        // Write is issued every cycle that a sub-BRAM write is pending;
        // the byte-enable masks ensure only the relevant fields are updated.
        logic [STATE_ROW_W-1:0]   mem_wr_data_c;
        logic [STATE_ROW_BYTES-1:0] mem_wr_mask_c;
        logic                     mem_wr_en_c;
        logic [NEURON_IDX_W-1:0]  mem_wr_addr_c;

        always_comb begin : mem_write_merge
            mem_wr_data_c = '0;
            mem_wr_mask_c = '0;
            mem_wr_en_c   = 1'b0;
            mem_wr_addr_c = '0;

            // dispatch write
            if (dispatch_bram_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = dispatch_bram_wr_addr_c;
                for (int b = 0; b < DISPATCH_BRAM_BYTES; b++) begin
                    if (dispatch_bram_wr_mask_c[b]) begin
                        mem_wr_mask_c[ROW_OFF_DISPATCH + b] = 1'b1;
                        mem_wr_data_c[(ROW_OFF_DISPATCH + b)*8 +: 8] =
                            dispatch_bram_wr_data_c[b*8 +: 8];
                    end
                end
            end
            // fanout write
            if (fanout_bram_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = fanout_bram_wr_addr_c;
                for (int b = 0; b < FANOUT_BRAM_BYTES; b++) begin
                    if (fanout_bram_wr_mask_c[b]) begin
                        mem_wr_mask_c[ROW_OFF_FANOUT + b] = 1'b1;
                        mem_wr_data_c[(ROW_OFF_FANOUT + b)*8 +: 8] =
                            fanout_bram_wr_data_c[b*8 +: 8];
                    end
                end
            end
            // commit_ptr write (fanout_ptr_wr_en_c)
            if (fanout_ptr_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = fanout_ptr_wr_idx_c;
                for (int b = 0; b < FANOUT_PTR_BYTES; b++) begin
                    mem_wr_mask_c[ROW_OFF_COMMIT_PTR + b] = 1'b1;
                    mem_wr_data_c[(ROW_OFF_COMMIT_PTR + b)*8 +: 8] =
                        fanout_ptr_wr_padded_c[b*8 +: 8];
                end
            end
            // dump write
            if (dump_bram_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = dump_bram_wr_addr_c;
                for (int b = 0; b < DUMP_BRAM_BYTES; b++) begin
                    if (dump_bram_wr_mask_c[b]) begin
                        mem_wr_mask_c[ROW_OFF_DUMP + b] = 1'b1;
                        mem_wr_data_c[(ROW_OFF_DUMP + b)*8 +: 8] =
                            dump_bram_wr_data_c[b*8 +: 8];
                    end
                end
            end
            // target write
            if (target_bram_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = target_bram_wr_addr_c;
                for (int b = 0; b < TARGET_BRAM_BYTES; b++) begin
                    if (target_bram_wr_mask_c[b]) begin
                        mem_wr_mask_c[ROW_OFF_TARGET + b] = 1'b1;
                        mem_wr_data_c[(ROW_OFF_TARGET + b)*8 +: 8] =
                            target_bram_wr_data_c[b*8 +: 8];
                    end
                end
            end
            // reset write
            if (reset_bram_wr_en_c) begin
                mem_wr_en_c   = 1'b1;
                mem_wr_addr_c = reset_bram_wr_addr_c;
                for (int b = 0; b < RESET_BRAM_BYTES; b++) begin
                    if (reset_bram_wr_mask_c[b]) begin
                        mem_wr_mask_c[ROW_OFF_RESET + b] = 1'b1;
                        mem_wr_data_c[(ROW_OFF_RESET + b)*8 +: 8] =
                            reset_bram_wr_data_c[b*8 +: 8];
                    end
                end
            end
        end

        coldfoot_mem_bytelane_sync #(
            .DATA_W    (STATE_ROW_W),
            .DEPTH     (NEURONS_PER_TILE),
            .BYTE_LANES(STATE_ROW_BYTES),
            .STYLE_HINT(MEM_STYLE_HINT)
        ) u_state_mem (
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

        // ── Phase sequencer + result latching ────────────────────────────────
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                rd_phase_r                <= '0;
                dispatch_bram_rd_data_r   <= '0;
                fanout_bram_rd_data_r     <= '0;
                commit_ptr_bram_rd_data_r <= '0;
                dump_bram_rd_data_r       <= '0;
                target_bram_rd_data_r     <= '0;
                reset_bram_rd_data_r      <= '0;
                fanout_compact_ptr_r      <= '0;
                fanout_compact_len_r      <= '0;
            end else begin
                // Advance phase every cycle (regardless of writes).
                rd_phase_r <= (rd_phase_r == PHASE_W'(NUM_PHASES - 1))
                              ? '0 : rd_phase_r + PHASE_W'(1);

                // Latch mem output into the port whose read was issued
                // on the PREVIOUS cycle (1-cycle FF latency).
                case (rd_phase_r)
                    // Phase N: mem_rd_data holds the result of the read
                    // issued in phase N-1.
                    3'd1: dispatch_bram_rd_data_r   <=
                              mem_rd_data[ROW_OFF_DISPATCH*8  +: DISPATCH_BRAM_W];
                    3'd2: if (!DROP_REDUNDANT_FANOUT_BRAM) begin
                              fanout_bram_rd_data_r <=
                                  mem_rd_data[ROW_OFF_FANOUT*8 +: FANOUT_BRAM_W];
                          end else begin
                              // Compact path: FANOUT section removed from row.
                              // Extract ptr from COMMIT section, len from DISPATCH section
                              // of the fanout_read_idx row that was read in phase 1.
                              fanout_compact_ptr_r <=
                                  mem_rd_data[ROW_OFF_COMMIT_PTR*8 +: FANOUT_PTR_BYTES*8];
                              fanout_compact_len_r <=
                                  mem_rd_data[(ROW_OFF_DISPATCH + DISPATCH_BYTE_FANOUT_LEN)*8
                                               +: FANOUT_PTR_BYTES*8];
                          end
                    // Phase 3: latch commit_ptr (non-drop) or dump data (drop — COMMIT removed).
                    3'd3: if (DROP_REDUNDANT_FANOUT_BRAM) begin
                              dump_bram_rd_data_r <=
                                  mem_rd_data[ROW_OFF_DUMP*8 +: DUMP_BRAM_W];
                          end else begin
                              commit_ptr_bram_rd_data_r <=
                                  mem_rd_data[ROW_OFF_COMMIT_PTR*8 +: FANOUT_PTR_BYTES*8];
                          end
                    // Phase 4: latch dump (non-drop) or target (drop+ASIC_ZERO_INIT=0 only;
                    //           unreachable in 4-phase drop mode where ASIC_ZERO_INIT!=0).
                    3'd4: if (DROP_REDUNDANT_FANOUT_BRAM) begin
                              // 5-phase drop (ASIC_ZERO_INIT_RESET=0): target data from phase 3 read.
                              target_bram_rd_data_r <=
                                  mem_rd_data[ROW_OFF_TARGET*8 +: TARGET_BRAM_W];
                          end else begin
                              dump_bram_rd_data_r <=
                                  mem_rd_data[ROW_OFF_DUMP*8 +: DUMP_BRAM_W];
                          end
                    // Phase 5 (6-phase non-drop mode only): latch target from phase 4 read.
                    3'd5: target_bram_rd_data_r <=
                              mem_rd_data[ROW_OFF_TARGET*8 +: TARGET_BRAM_W];
                    // Phase 0:
                    //   4-phase drop (ASIC_ZERO_INIT!=0): latch target from phase 3 read.
                    //   5-phase drop (ASIC_ZERO_INIT==0): latch reset from phase 4 read.
                    //   5-phase non-drop (ASIC_ZERO_INIT!=0): latch target from phase 4 read.
                    //   6-phase non-drop (ASIC_ZERO_INIT==0): latch reset from phase 5 read.
                    3'd0: if (ASIC_ZERO_INIT_RESET != 0) begin
                              target_bram_rd_data_r <= mem_rd_data[ROW_OFF_TARGET*8 +: TARGET_BRAM_W];
                          end else begin
                              reset_bram_rd_data_r  <= mem_rd_data[ROW_OFF_RESET*8  +: RESET_BRAM_W];
                          end
                    default: ;
                endcase
            end
        end

        // ── Connect registered results to the module-scope rd_data wires ─────
        assign dispatch_bram_rd_data   = dispatch_bram_rd_data_r;
        assign fanout_bram_rd_data     = fanout_bram_rd_data_r;
        assign commit_ptr_bram_rd_data = commit_ptr_bram_rd_data_r;
        assign dump_bram_rd_data       = dump_bram_rd_data_r;
        assign target_bram_rd_data     = target_bram_rd_data_r;
        assign reset_bram_rd_data      = reset_bram_rd_data_r;
        assign fanout_compact_ptr      = fanout_compact_ptr_r;
        assign fanout_compact_len      = fanout_compact_len_r;

        // mem_stall: asserted during phases 1–5 (phase 0 = dispatch, which
        // is the hot path; consumers that only need dispatch can ignore stall
        // during the remaining phases, but the global stall gate in tile_top
        // halts all dispatch until the round completes).
        assign mem_stall = (rd_phase_r != 3'd0);  // mem_waitreq always 0

    // Fanout pointer/length storage in FF register file through fanout/target/dump
    // BRAM rows plus a dedicated commit_ptr BRAM read port.

    // ---------------------------------------------------------------
    // Compact FF read outputs — used by the output always_comb below.
    // Declared at module scope so the output mux can reference them
    // regardless of whether gen_compact_ff was elaborated.
    // When USE_COMPACT_FF=0 (STYLE=2 Xilinx) these are tied to '0 and
    // the output mux selects BRAM data instead.
    // ---------------------------------------------------------------
    logic [4:0] cff_ucode_ptr_dispatch;
    logic [4:0] cff_ucode_len_dispatch;
    logic [4:0] cff_ucode_ptr_dump;
    logic [4:0] cff_ucode_len_dump;
    logic [4:0] cff_ucode_ptr_target;
    logic [4:0] cff_ucode_len_target;
    logic [7:0] cff_init_v_target;
    logic [7:0] cff_init_syn_target;
    logic [7:0] cff_init_aux_target;
    logic [7:0] cff_flags_target;
    logic [7:0] cff_init_v_dump;
    logic [7:0] cff_init_syn_dump;
    logic [7:0] cff_init_aux_dump;
    logic [7:0] cff_flags_dump;
    logic [7:0] cff_init_v_reset;
    logic [7:0] cff_init_syn_reset;
    logic [7:0] cff_init_aux_reset;
    // Compact FF storage — active when USE_COMPACT_FF=1.
    //
    // For STYLE 0/1 (pure FF path): ALL fields are stored here and the
    // BRAM outputs above are '0 stubs.
    //
    // For STYLE 2 (BLOCK path): not instantiated — all reads come from
    // the BRAM outputs.
    //
    // ---------------------------------------------------------------
    if (USE_COMPACT_FF) begin : gen_compact_ff

        logic [4:0] ucode_ptr_mem_r [0:NEURONS_PER_TILE-1];
        logic [4:0] ucode_len_mem_r [0:NEURONS_PER_TILE-1];

        // Init / flags — only needed for pure FF path (STYLE 0/1).
        logic [7:0] init_v_mem_r     [0:NEURONS_PER_TILE-1];
        logic [7:0] init_syn_r       [0:NEURONS_PER_TILE-1];
        // init_refr_r removed: rf[3]=refractory eliminated
        logic [7:0] init_aux_r       [0:NEURONS_PER_TILE-1];
        logic [7:0] neuron_flags_r   [0:NEURONS_PER_TILE-1];
        // last_event_time_r removed: timestamps eliminated
        integer ucode_reset_idx;

        always_ff @(posedge clk) begin
            if (!rst_n) begin
                for (ucode_reset_idx = 0;
                     ucode_reset_idx < NEURONS_PER_TILE;
                     ucode_reset_idx = ucode_reset_idx + 1) begin
                    // When UCODE_SHARED_BANK!=0 the ptr array is never written;
                    // gating the reset lets synthesis eliminate the flip-flops.
                    if (UCODE_SHARED_BANK == 0)
                        ucode_ptr_mem_r[ucode_reset_idx]   <= 5'd0;
                        ucode_len_mem_r[ucode_reset_idx]   <= 5'd0;
                        init_v_mem_r[ucode_reset_idx]      <= 8'd0;
                        init_syn_r[ucode_reset_idx]        <= 8'd0;
                        // init_refr removed
                        init_aux_r[ucode_reset_idx]        <= 8'd0;
                        neuron_flags_r[ucode_reset_idx]    <= 8'd0;
                        // last_event_time removed
                end
            end else if (ena) begin
                if (ucode_ptr_wr_en_c) begin
                    ucode_ptr_mem_r[ucode_ptr_wr_idx_c] <= ucode_ptr_wr_data_c;
                end
                if (ucode_len_wr_en_c) begin
                    ucode_len_mem_r[ucode_len_wr_idx_c] <= ucode_len_wr_data_c;
                end
                if (csr_write_hit_c) begin
                    case (csr_addr)
                        CSR_INIT_VI:    init_v_mem_r[csr_selected_idx_c]      <= csr_data;
                        CSR_INIT_TR:    init_syn_r[csr_selected_idx_c]        <= csr_data;
                        // CSR_INIT_T01: removed (rf[3] eliminated)
                        CSR_INIT_WAUX:  init_aux_r[csr_selected_idx_c]        <= csr_data;
                        CSR_VEC_BASE_01: neuron_flags_r[csr_selected_idx_c]   <= csr_data;
                        // CSR_NEURON_META: removed (timestamps eliminated)
                        default: begin end
                    endcase
                end
            end
        end

        // Drive module-scope cff_* signals from local FF arrays.
        always_comb begin
            // When UCODE_SHARED_BANK!=0 the ptr array is never written and
            // synthesis will optimise it away; expose constant 0 directly.
            cff_ucode_ptr_dispatch  = (UCODE_SHARED_BANK != 0) ? 5'd0 : ucode_ptr_mem_r[dispatch_read_idx];
            cff_ucode_len_dispatch  = ucode_len_mem_r[dispatch_read_idx];
            cff_ucode_ptr_dump      = (UCODE_SHARED_BANK != 0) ? 5'd0 : ucode_ptr_mem_r[dump_read_idx];
            cff_ucode_len_dump      = ucode_len_mem_r[dump_read_idx];
            cff_ucode_ptr_target    = (UCODE_SHARED_BANK != 0) ? 5'd0 : ucode_ptr_mem_r[target_read_idx];
            cff_ucode_len_target    = ucode_len_mem_r[target_read_idx];
            cff_init_v_target       = init_v_mem_r[target_read_idx];
            cff_init_syn_target     = init_syn_r[target_read_idx];
            // cff_init_refr_target removed
            cff_init_aux_target     = init_aux_r[target_read_idx];
            cff_flags_target        = neuron_flags_r[target_read_idx];
            // cff_last_time_target removed
            cff_init_v_dump         = init_v_mem_r[dump_read_idx];
            cff_init_syn_dump       = init_syn_r[dump_read_idx];
            // cff_init_refr_dump removed
            cff_init_aux_dump       = init_aux_r[dump_read_idx];
            cff_flags_dump          = neuron_flags_r[dump_read_idx];
            cff_init_v_reset        = init_v_mem_r[reset_read_idx];
            cff_init_syn_reset      = init_syn_r[reset_read_idx];
            // cff_init_refr_reset removed
            cff_init_aux_reset      = init_aux_r[reset_read_idx];
            // cff_last_time_reset removed
        end

    end else begin : gen_compact_ff_stub

        // STYLE=2: tie cff_* outputs to '0; output mux uses BRAM data.
        always_comb begin
            cff_ucode_ptr_dispatch  = 5'd0;
            cff_ucode_len_dispatch  = 5'd0;
            cff_ucode_ptr_dump      = 5'd0;
            cff_ucode_len_dump      = 5'd0;
            cff_ucode_ptr_target    = 5'd0;
            cff_ucode_len_target    = 5'd0;
            cff_init_v_target       = 8'd0;
            cff_init_syn_target     = 8'd0;
            // cff_init_refr_target removed
            cff_init_aux_target     = 8'd0;
            cff_flags_target        = 8'd0;
            // cff_last_time_target removed
            cff_init_v_dump         = 8'd0;
            cff_init_syn_dump       = 8'd0;
            // cff_init_refr_dump removed
            cff_init_aux_dump       = 8'd0;
            cff_flags_dump          = 8'd0;
            cff_init_v_reset        = 8'd0;
            cff_init_syn_reset      = 8'd0;
            // cff_init_refr_reset removed
            cff_init_aux_reset      = 8'd0;
            // cff_last_time_reset removed
        end

    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            dispatch_epoch_valid_r <= 1'b0;
            fanout_epoch_valid_r <= 1'b0;
            dump_epoch_valid_r <= 1'b0;
            target_epoch_valid_r <= 1'b0;
            reset_epoch_valid_r <= 1'b0;
            commit_epoch_valid_r <= 1'b0;
        end else if (ena) begin
            dispatch_epoch_valid_r <= (state_epoch_seen_r[dispatch_read_idx] == state_epoch_r);
            fanout_epoch_valid_r <= (state_epoch_seen_r[fanout_read_idx] == state_epoch_r);
            dump_epoch_valid_r <= (state_epoch_seen_r[dump_read_idx] == state_epoch_r);
            target_epoch_valid_r <= (state_epoch_seen_r[target_read_idx] == state_epoch_r);
            reset_epoch_valid_r <= (state_epoch_seen_r[reset_read_idx] == state_epoch_r);
            commit_epoch_valid_r <= (state_epoch_seen_r[commit_read_idx] == state_epoch_r);
        end
    end

    always_comb begin
        if (!target_epoch_valid_r) begin
            target_csr_read_data = (csr_addr == CSR_CTRL) ? 8'b0001_0000 : 8'h00;
            target_fanout_ptr = '0;
            target_fanout_len = '0;
        end else begin
            case (csr_addr)
                CSR_CTRL: target_csr_read_data = {
                    3'b000,
                    noc_egress_en_bus[target_read_idx],
                    host_egress_en_bus[target_read_idx],
                    3'b000
                };
                // ucode_ptr/len: compact FF arrays when USE_COMPACT_FF, else
                // BRAM bytes on the Xilinx path (STYLE=2).
                CSR_UCODE_PTR: target_csr_read_data = (UCODE_SHARED_BANK != 0) ? 8'b0 :
                    (USE_COMPACT_FF ? {3'b000, cff_ucode_ptr_target}
                                   : target_bram_rd_data[DUMP_BYTE_UCODE_PTR*8 +: 8]);
                CSR_UCODE_LEN: target_csr_read_data = USE_COMPACT_FF
                    ? {3'b000, cff_ucode_len_target}
                    : target_bram_rd_data[DUMP_BYTE_UCODE_LEN*8 +: 8];
                // Flags, init values, last_event_time: compact FFs when
                // !USE_BRAM (STYLE 0/1), BRAM bytes when USE_BRAM (STYLE 2/3).
                CSR_VEC_BASE_01: target_csr_read_data = !USE_BRAM
                    ? cff_flags_target
                    : target_bram_rd_data[DUMP_BYTE_FLAGS*8 +: 8];
                // CSR_VEC_BASE_23 returns the low byte of fanout_ptr.
                CSR_VEC_BASE_23: target_csr_read_data =
                    target_bram_rd_data[DUMP_BYTE_FANOUT_PTR*8 +: 8];
                CSR_INIT_VI: target_csr_read_data = !USE_BRAM
                    ? cff_init_v_target
                    : target_bram_rd_data[DUMP_BYTE_V_MEM*8 +: 8];
                CSR_INIT_TR: target_csr_read_data = !USE_BRAM
                    ? cff_init_syn_target
                    : target_bram_rd_data[DUMP_BYTE_SYN*8 +: 8];
                CSR_INIT_T01: target_csr_read_data = 8'h00;  // rf[3] eliminated
                CSR_INIT_WAUX: target_csr_read_data = !USE_BRAM
                    ? cff_init_aux_target
                    : target_bram_rd_data[DUMP_BYTE_AUX*8 +: 8];
                CSR_NEURON_META: target_csr_read_data = 8'h00;  // last_time eliminated
                default: target_csr_read_data = 8'h00;
            endcase
            if (STATE_ROW_NO_FANOUT_BYTES) begin
                target_fanout_ptr = '0;
                target_fanout_len = '0;
            end else begin
                target_fanout_ptr =
                    FANOUT_PTR_W'(target_bram_rd_data[DUMP_BYTE_FANOUT_PTR*8 +: FANOUT_PTR_W]);
                target_fanout_len =
                    FANOUT_PTR_W'(target_bram_rd_data[DUMP_BYTE_FANOUT_LEN*8 +: FANOUT_PTR_W]);
            end
        end

        if (!dispatch_epoch_valid_r) begin
            dispatch_ucode_ptr  = 5'd0;
            dispatch_ucode_len  = 5'd0;
            dispatch_fanout_len = '0;
        end else begin
            // ucode: compact FF on USE_COMPACT_FF paths; BRAM byte on STYLE=2.
            dispatch_ucode_ptr = (UCODE_SHARED_BANK != 0) ? 5'b0 :
                (USE_COMPACT_FF ? cff_ucode_ptr_dispatch
                               : dispatch_bram_rd_data[DISPATCH_BYTE_UCODE_PTR*8 +: 5]);
            dispatch_ucode_len = USE_COMPACT_FF
                ? cff_ucode_len_dispatch
                : dispatch_bram_rd_data[DISPATCH_BYTE_UCODE_LEN*8 +: 5];
            dispatch_fanout_len =
                FANOUT_PTR_W'(dispatch_bram_rd_data[DISPATCH_BYTE_FANOUT_LEN*8 +: FANOUT_PTR_W]);
        end

        if (!fanout_epoch_valid_r) begin
            fanout_read_ptr = '0;
            fanout_read_len = '0;
        end else if (DROP_REDUNDANT_FANOUT_BRAM) begin
            // Compact path: ptr/len extracted from COMMIT/DISPATCH during phase-1 latch.
            fanout_read_ptr = FANOUT_PTR_W'(fanout_compact_ptr[FANOUT_PTR_W-1:0]);
            fanout_read_len = FANOUT_PTR_W'(fanout_compact_len[FANOUT_PTR_W-1:0]);
        end else begin
            fanout_read_ptr =
                FANOUT_PTR_W'(fanout_bram_rd_data[FANOUT_BYTE_PTR*8 +: FANOUT_PTR_W]);
            fanout_read_len =
                FANOUT_PTR_W'(fanout_bram_rd_data[FANOUT_BYTE_LEN*8 +: FANOUT_PTR_W]);
        end

        commit_fanout_ptr = commit_epoch_valid_r
            ? FANOUT_PTR_W'(commit_ptr_bram_rd_data[FANOUT_PTR_W-1:0])
            : '0;

        if (!dump_epoch_valid_r) begin
            dump_init_rf_flat = '0;
            dump_ucode_ptr    = 5'd0;
            dump_ucode_len    = 5'd0;
            dump_neuron_flags = 8'd0;
            dump_fanout_ptr   = '0;
            dump_fanout_len   = '0;
        end else begin
            dump_init_rf_flat = '0;
            if (!USE_BRAM) begin
                dump_init_rf_flat[7:0]   = cff_init_v_dump;
                dump_init_rf_flat[15:8]  = cff_init_syn_dump;
                dump_init_rf_flat[23:16] = cff_init_aux_dump;
            end else begin
                dump_init_rf_flat[7:0]   = dump_bram_rd_data[DUMP_BYTE_V_MEM*8  +: 8];
                dump_init_rf_flat[15:8]  = dump_bram_rd_data[DUMP_BYTE_SYN*8    +: 8];
                dump_init_rf_flat[23:16] = dump_bram_rd_data[DUMP_BYTE_AUX*8    +: 8];
            end
            // ucode: compact FFs when USE_COMPACT_FF, BRAM when STYLE=2 only.
            dump_ucode_ptr = (UCODE_SHARED_BANK != 0) ? 5'b0 :
                (USE_COMPACT_FF ? cff_ucode_ptr_dump
                               : dump_bram_rd_data[DUMP_BYTE_UCODE_PTR*8 +: 5]);
            dump_ucode_len = USE_COMPACT_FF
                ? cff_ucode_len_dump
                : dump_bram_rd_data[DUMP_BYTE_UCODE_LEN*8 +: 5];
            // flags: compact FFs for STYLE 0/1, BRAM for STYLE 2/3.
            dump_neuron_flags = !USE_BRAM
                ? cff_flags_dump
                : dump_bram_rd_data[DUMP_BYTE_FLAGS*8 +: 8];
            if (STATE_ROW_NO_FANOUT_BYTES) begin
                dump_fanout_ptr = '0;
                dump_fanout_len = '0;
            end else begin
                dump_fanout_ptr =
                    FANOUT_PTR_W'(dump_bram_rd_data[DUMP_BYTE_FANOUT_PTR*8 +: FANOUT_PTR_W]);
                dump_fanout_len =
                    FANOUT_PTR_W'(dump_bram_rd_data[DUMP_BYTE_FANOUT_LEN*8 +: FANOUT_PTR_W]);
            end
        end

        // When ASIC_ZERO_INIT_RESET!=0 the reset BRAM is not present;
        // soft-reset always restores neurons to all-zero initial state.
        if (ASIC_ZERO_INIT_RESET != 0 || !reset_epoch_valid_r) begin
            reset_init_rf_flat    = '0;
        end else begin
            reset_init_rf_flat = '0;
            if (!USE_BRAM) begin
                reset_init_rf_flat[7:0]   = cff_init_v_reset;
                reset_init_rf_flat[15:8]  = cff_init_syn_reset;
                reset_init_rf_flat[23:16] = cff_init_aux_reset;
            end else begin
                reset_init_rf_flat[7:0]   = reset_bram_rd_data[RESET_BYTE_V_MEM*8    +: 8];
                reset_init_rf_flat[15:8]  = reset_bram_rd_data[RESET_BYTE_SYN*8      +: 8];
                reset_init_rf_flat[23:16] = reset_bram_rd_data[RESET_BYTE_AUX*8      +: 8];
            end
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state_epoch_r <= 1'b0;
            state_epoch_seen_r <= '1;
            // Scalar soft-reset outputs (replaces N-wide soft_reset_pulse FF).
            soft_reset_pulse <= '0;   // legacy tie-off: always 0
            soft_reset_valid <= 1'b0;
            soft_reset_idx   <= '0;
            host_egress_en_bus <= '0;
            noc_egress_en_bus <= {NEURONS_PER_TILE{1'b1}};
            reset_drain_mask_r <= '0;
        end else begin
            // Scalar soft-reset register.
            soft_reset_pulse <= '0;   // legacy tie-off
            soft_reset_valid <= soft_reset_valid_next_c;
            soft_reset_idx   <= soft_reset_idx_next_c;

            if (graph_state_clear) begin
                state_epoch_r <= ~state_epoch_r;
                host_egress_en_bus <= '0;
                noc_egress_en_bus <= {NEURONS_PER_TILE{1'b1}};
                reset_drain_mask_r <= '0;
            end else if (ena) begin
                // Direct-index epoch updates (replaces N-wide mask loop).
                if (csr_write_hit_c) begin
                    state_epoch_seen_r[csr_selected_idx_c] <= state_epoch_r;
                end
                if (ucode_prog_en && ucode_prog_selected_valid_c) begin
                    state_epoch_seen_r[ucode_prog_selected_idx_c] <= state_epoch_r;
                end
                if ((fanout_prog_ptr_en || fanout_prog_len_en) && fanout_prog_selected_valid_c) begin
                    state_epoch_seen_r[fanout_prog_selected_idx_c] <= state_epoch_r;
                end
                if (csr_write_hit_c && (csr_addr == CSR_CTRL)) begin
                    host_egress_en_bus[csr_selected_idx_c] <= csr_data[CSR_CTRL_HOST_EGRESS_BIT];
                    noc_egress_en_bus[csr_selected_idx_c]  <= csr_data[CSR_CTRL_NOC_EGRESS_BIT];
                end
                // Drain queue: scalar add (unicast or broadcast) then
                // clear the emitted bit using its index.
                if (csr_write_hit_c &&
                    (csr_addr == CSR_RESET_TRIGGER) &&
                    csr_data[CSR_RESET_TRIGGER_SOFT_RESET_BIT]) begin
                    if (csr_write_broadcast) begin
                        reset_drain_mask_r <= '1;
                    end else begin
                        reset_drain_mask_r[csr_write_addr] <= 1'b1;
                    end
                end
                if (reset_drain_emit_c) begin
                    reset_drain_mask_r[drain_idx_c] <= 1'b0;
                end
            end
        end
    end

endmodule

`default_nettype wire
