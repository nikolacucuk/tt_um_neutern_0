`ifndef TILE_FLIT_TYPES_VH
`define TILE_FLIT_TYPES_VH

// tile_flit_types.vh — compatibility shim
//
// All typedef structs and functions have been moved into tile_pkg.sv so that
// any source file can access them with a plain `import tile_pkg::*;` without
// needing an include-path (-I) flag pointing to this directory.
//
// This shim is kept so that test and formal files that still use
// `include "tile_flit_types.vh" continue to compile correctly: the
// `import tile_pkg::*` below brings all types into scope.

import tile_pkg::*;

`endif // TILE_FLIT_TYPES_VH
