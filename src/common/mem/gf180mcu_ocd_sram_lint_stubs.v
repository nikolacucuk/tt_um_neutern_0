// gf180mcu_ocd_sram_lint_stubs.v
// ---------------------------------------------------------------------------
// Black-box stub declarations for the GF180MCU OCD 3.3V SRAM macros that
// `tile_mem_1r1w_sync.sv` instantiates inside its `STYLE_HINT >=
// STYLE_MACRO_INTENT` generate paths.
//
// These stubs are intentionally empty (port list only, no body).  They
// give Verilator's lint reference-resolution pass something to point at
// for the macro names — the generate condition `if (GF180_OCD_256_ENABLED)`
// is `1'b0` in lint flows (the `COLDFOOT_GF180_OCD_SRAM_SP_MACRO` define
// is not set), so the elaborated design never actually drives signals
// into these modules.  But Verilator resolves module references before
// it elides constant-false generate branches, so without these stubs
// every lint target that pulls in tile_mem_1r1w_sync errors out with
// "Can't resolve module reference: 'gf180mcu_ocd_ip_sram__sram256x8m8wm1'".
//
// The "real" macro views live under `.tmp/gf180mcu_ocd_ip_sram/` (vendor
// IP downloaded by tooling, not committed) and are picked up explicitly
// by PnR / hardening flows that drive STYLE_HINT>=3 paths and define
// `COLDFOOT_GF180_OCD_SRAM_SP_MACRO`.  This file is a lint-only convenience
// — it must NOT be on the synthesis or PnR paths, where the real models
// are required.
// ---------------------------------------------------------------------------

`ifndef GF180MCU_OCD_SRAM_LINT_STUBS_V
`define GF180MCU_OCD_SRAM_LINT_STUBS_V

`timescale 1ns/1ps

module gf180mcu_ocd_ip_sram__sram256x8m8wm1 (
`ifdef USE_POWER_PINS
    inout VDD,
    inout VSS,
`endif
    input  wire        CLK,
    input  wire        CEN,
    input  wire        GWEN,
    input  wire [7:0]  WEN,
    input  wire [7:0]  A,
    input  wire [7:0]  D,
    output wire [7:0]  Q
);
endmodule

module gf180mcu_ocd_ip_sram__sram512x8m8wm1 (
`ifdef USE_POWER_PINS
    inout VDD,
    inout VSS,
`endif
    input  wire        CLK,
    input  wire        CEN,
    input  wire        GWEN,
    input  wire [7:0]  WEN,
    input  wire [8:0]  A,
    input  wire [7:0]  D,
    output wire [7:0]  Q
);
endmodule

module gf180mcu_ocd_ip_sram__sram1024x8m8wm1 (
`ifdef USE_POWER_PINS
    inout VDD,
    inout VSS,
`endif
    input  wire        CLK,
    input  wire        CEN,
    input  wire        GWEN,
    input  wire [7:0]  WEN,
    input  wire [9:0]  A,
    input  wire [7:0]  D,
    output wire [7:0]  Q
);
endmodule

// 5V FD-family macro (used by tile_mem_1r1w_sync.sv:423 in the
// `STYLE_HINT >= STYLE_MACRO_INTENT && GF180_FD_256_ENABLED` path).
// Differs from OCD: VDD/VSS are unconditional ports (no
// `ifdef USE_POWER_PINS` gate).
module gf180mcu_fd_ip_sram__sram256x8m8wm1 (
    input  wire        CLK,
    input  wire        CEN,
    input  wire        GWEN,
    input  wire [7:0]  WEN,
    input  wire [7:0]  A,
    input  wire [7:0]  D,
    output wire [7:0]  Q,
    inout  wire        VDD,
    inout  wire        VSS
);
endmodule

`endif // GF180MCU_OCD_SRAM_LINT_STUBS_V
