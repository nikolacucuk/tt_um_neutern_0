#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Tuple


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PDK_ROOT = str(REPO_ROOT / ".pdks")
SBY_BIN = os.environ.get("COLDFOOT_SBY_BIN") or shutil.which("sby") or "/foss/tools/yosys/bin/sby"
YOSYS_BIN_DIR = "/foss/tools/yosys/bin"
DEFAULT_FORMAL_JOBS = "10"
FALLBACK_PNR_PDK = "sky130A"
FALLBACK_PNR_SCL = "sky130_fd_sc_hd"
PNR_DEFAULTS = {
    ("soc", "coldfoot"): ("gf180mcuD", "gf180mcu_fd_sc_mcu7t5v0"),
    ("soc", "default"): ("gf180mcuD", "gf180mcu_fd_sc_mcu7t5v0"),
    ("soc", "rv_sw_async"): ("gf180mcuD", "gf180mcu_fd_sc_mcu7t5v0"),
}


def run(cmd: List[str], *, cwd: Path, env: Dict[str, str] | None = None) -> int:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    print(f"[flow] cwd={cwd}")
    print(f"[flow] exec: {' '.join(shlex.quote(part) for part in cmd)}")
    try:
        proc = subprocess.run(cmd, cwd=str(cwd), env=merged_env)
    except FileNotFoundError:
        print(f"error: command not found: {cmd[0]}", file=sys.stderr)
        return 127
    return proc.returncode


def formal(project: str, target: str | None) -> int:
    formal_env = {"PATH": f"{YOSYS_BIN_DIR}:{os.environ.get('PATH', '')}"}
    formal_jobs = os.environ.get("COLDFOOT_FORMAL_JOBS", DEFAULT_FORMAL_JOBS)
    sby_prefix = [SBY_BIN, "-j", formal_jobs, "-f"]
    if project == "common":
        common_formals = (
            ("coldfoot_rr_arbiter", [*sby_prefix, "coldfoot_rr_arbiter_formal.sby"]),
            ("rv_demux_async", [*sby_prefix, "config.sby"]),
            ("rv_depkt", [*sby_prefix, "config.sby"]),
            ("rv_depkt_async", [*sby_prefix, "config.sby"]),
            ("rv_fifo", [*sby_prefix, "config.sby"]),
            ("rv_fifo_async", [*sby_prefix, "config.sby"]),
            ("rv_mux", [*sby_prefix, "config.sby"]),
            ("rv_mux_async", [*sby_prefix, "config.sby"]),
            ("rv_pkt", [*sby_prefix, "config.sby"]),
            ("rv_pkt_async", [*sby_prefix, "config.sby"]),
            ("rv_pkt_mux_async", [*sby_prefix, "config.sby"]),
            ("rv_reg_slice", [*sby_prefix, "config.sby"]),
            ("rv_sw_arb_async", [*sby_prefix, "rv_sw_arb_async_formal.sby"]),
            ("rv_sw_async", [*sby_prefix, "rv_sw_async_formal.sby"]),
            ("rv_sw_rr", [*sby_prefix, "rv_sw_rr_formal.sby"]),
        )
        for dir_name, cmd in common_formals:
            cwd = REPO_ROOT / "hw" / "common" / "formal" / dir_name
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "logical_neuron":
        cwd = REPO_ROOT / "hw" / "ip" / "logical_neuron" / "formal"
        if not cwd.exists():
            cwd = REPO_ROOT / "src" / "logical_neuron" / "formal"
        # Phase 3 (2026-05): all four harnesses + the third
        # state_bank task PASS.  Originally used `smtbmc bitwuzla`
        # which isn't installed in our docker image; switched to
        # yices.  state_bank also needed slang migration because
        # it includes tile_flit_types.vh.
        for cmd in (
            [*sby_prefix, "logical_neuron_context_bank_formal.sby"],
            [*sby_prefix, "logical_neuron_state_bank_formal.sby", "reset_defaults"],
            [*sby_prefix, "logical_neuron_state_bank_formal.sby", "ctrl_write"],
            [*sby_prefix, "logical_neuron_state_bank_formal.sby", "reset_trigger_drain"],
            [*sby_prefix, "logical_neuron_ucode_bank_formal.sby"],
        ):
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "neuron_compute":
        cwd = REPO_ROOT / "hw" / "ip" / "neuron_compute" / "formal"
        if not cwd.exists():
            cwd = REPO_ROOT / "src" / "neuron_compute" / "formal"
        for cmd in (
            [*sby_prefix, "neuron_exec_formal.sby"],
            [*sby_prefix, "neuron_compute_core_formal.sby"],
        ):
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "neural_mesh":
        cwd = REPO_ROOT / "hw" / "ip" / "neural_mesh" / "formal"
        for cmd in (
            # Phase 3 (2026-05): migrated mesh_router_isolation,
            # flit_framing, packet_flit_roundtrip_live from
            # read_verilog to slang.  All three mesh_router_isolation
            # tasks PASS post-migration.  Two tasks (flit_framing's
            # wormhole_contiguous and packet_flit_roundtrip_live's
            # msg_pong_roundtrip) now FAIL with real assertion
            # failures previously hidden by the upstream parse error
            # — deregistered as Phase 3 follow-ups.
            [*sby_prefix, "mesh_router_isolation_formal.sby", "cmd_not_blocked_by_data"],
            [*sby_prefix, "mesh_router_isolation_formal.sby", "data_no_starvation"],
            [*sby_prefix, "mesh_router_isolation_formal.sby", "urgent_elevation"],
            # Phase 3 Group B (2026-05): wormhole_contiguous PASSes
            # after replacing slang's broken `(* anyconst *)` (which
            # the plugin treats as anyseq) with anyseq-latched-at-
            # cycle-0 registers.
            [*sby_prefix, "flit_framing_formal.sby", "wormhole_contiguous"],
            [*sby_prefix, "flit_framing_formal.sby", "length_matches"],
            [*sby_prefix, "flit_framing_formal.sby", "plane_preserved"],
            # Phase 3 Group B (2026-05): msg_pong_roundtrip PASSes
            # after fixing stale PACKET_W=95 → 83 + the slang
            # `(* anyconst *)` workaround (latch anyseq at cycle 0).
            [*sby_prefix, "packet_flit_roundtrip_live_formal.sby", "msg_pong_roundtrip"],
            # ---- Mesh router (active NOC datapath) ----
            # Phase 3 (2026-05): migrated to slang.  $past refactored
            # to delay registers; `initial assert` (which slang rejects)
            # rewritten as `always @* if (!f_past_valid)`.
            [*sby_prefix, "mesh_router_formal.sby", "routing"],
            [*sby_prefix, "mesh_router_formal.sby", "broadcast"],
            [*sby_prefix, "mesh_router_formal.sby", "host_divert"],
            [*sby_prefix, "mesh_router_formal.sby", "backpressure"],
            [*sby_prefix, "mesh_router_formal.sby", "counters"],
            [*sby_prefix, "mesh_router_formal.sby", "local_output_stamp_east"],
            [*sby_prefix, "mesh_router_formal.sby", "local_output_stamp_west"],
            [*sby_prefix, "mesh_router_formal.sby", "local_output_stamp_south"],
            [*sby_prefix, "mesh_router_formal.sby", "local_output_stamp_north"],
            # ---- Mesh router input FIFO (Wave D decomposition) ----
            # Phase 3 Group D (2026-05): replaced the sv2v-flatten
            # build step with a direct slang read.  sv2v couldn't
            # find the `tile_pkg` package (needed transitively via
            # tile_flit_types.vh) and so failed to produce its flat
            # output.  Slang handles interface ports natively and
            # the file-scope-import in tile_flit_types.vh, so it's
            # a clean replacement.
            [*sby_prefix, "mesh_router_input_fifo_formal.sby", "fifo_count_bound"],
            [*sby_prefix, "mesh_router_input_fifo_formal.sby", "credit_no_drift"],
            [*sby_prefix, "mesh_router_input_fifo_formal.sby", "credit_conservation"],
            # ---- Bundle loader ----
            # Phase 3 (2026-05): all four bundle_loader harnesses
            # migrated to slang.  v4 had real $past usage (refactored
            # to delay registers); the other three only needed the
            # file-scope-import migration.
            [*sby_prefix, "mesh_bundle_loader_formal.sby"],
            [*sby_prefix, "mesh_bundle_loader_error_recovery_formal.sby"],
            [*sby_prefix, "mesh_bundle_loader_timeout_formal.sby"],
            [*sby_prefix, "mesh_bundle_loader_v4_formal.sby"],
            # ---- Phase 1 formal-recovery (2026-05): orphans
            # restored to the sweep after slang-reader migration.
            # See docs/formal-recovery-2026-05.md.
            #
            # mesh_router_lifecycle: all 6 tasks compile + PASS on
            # this branch.
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "non_broadcast_single_bit"],
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "local_to_local_safe"],
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "one_fire_per_pending_bit"],
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "fifo_pop_once"],
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "in_out_conservation"],
            [*sby_prefix, "mesh_router_lifecycle_formal.sby", "in_out_conservation_cover"],
            # mesh_router_broadcast: 3 of 7 tasks compile + PASS;
            # 3 (local_unicast_one_egress, broadcast_mask_bounded,
            # stamp_idempotent) hit slang plugin "Feature unimplemented"
            # or rtlil-dump assert and stay parked.
            # `local_to_local_loop` is INTENTIONALLY-failing — the
            # harness asserts the negation as documentation, not a bug
            # claim — so we don't register it in the green sweep.  See
            # the file's preamble for the witness-mechanism note.
            [*sby_prefix, "mesh_router_broadcast_formal.sby", "no_port_replay"],
            [*sby_prefix, "mesh_router_broadcast_formal.sby", "mcast_unknown_drop"],
            [*sby_prefix, "mesh_router_broadcast_formal.sby", "host_port_never_broadcast"],
            # flit_to_packet_liveness: ena was unbound which let the
            # engine drive ena=0 to stall ready trivially.  Phase 2
            # added an `assume(ena)` post-reset; now PASSes.
            [*sby_prefix, "flit_to_packet_liveness_formal.sby"],
            # mesh_bundle_loader_program_meta_backpressure: harness
            # was missing an `initial f_past_valid = 0` so the engine
            # could pick step-0 register state freely.  Phase 2 added
            # the init; now PASSes.
            [*sby_prefix, "mesh_bundle_loader_program_meta_backpressure_formal.sby"],
        ):
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "tile":
        cwd = REPO_ROOT / "hw" / "ip" / "tile" / "formal"
        if not cwd.exists():
            cwd = REPO_ROOT / "src" / "tile" / "formal"
        # Some checkouts only carry the maintained tile_top_formal bundle and
        # not every legacy leaf harness source file referenced by the full
        # sweep below.
        if not (cwd / "tile_noc_egress_formal.sv").exists():
            for cmd in (
                [*sby_prefix, "tile_top_formal.sby", "reset_quiescence"],
                [*sby_prefix, "tile_top_formal.sby", "ena_disabled_no_output"],
                [*sby_prefix, "tile_top_formal.sby", "host_priority_mux"],
                [*sby_prefix, "tile_top_formal.sby", "noc_yields_to_host"],
                [*sby_prefix, "tile_top_formal.sby", "output_valid_stable_until_ready"],
                [*sby_prefix, "tile_top_formal.sby", "host_payload_stable"],
                [*sby_prefix, "tile_top_formal.sby", "noc_payload_stable"],
            ):
                rc = run(cmd, cwd=cwd, env=formal_env)
                if rc != 0:
                    return rc
            return 0
        # Phase 1 of formal-infrastructure recovery (2026-05-05): the
        # optimization branch refactored most of the tile RTL but
        # silently dropped its formal coverage from this registry —
        # ten .sby harnesses sat orphaned on disk.  This list re-adds
        # every harness that COMPILES against the current RTL after
        # porting to the slang reader; the rest have port-drift in
        # the harness `*_formal.sv` source (not in this registry) and
        # are tracked separately in
        # `docs/formal-recovery-2026-05.md`.
        for cmd in (
            [*sby_prefix, "tile_noc_egress_formal.sby"],
            # tile_fanout_executor: 4 of 5 tasks pass cleanly; the 5th
            # (retry_requires_blocked) hits a slang plugin
            # "Feature unimplemented" on a $signed/$unsigned cast and
            # is parked until either slang catches up or the harness
            # rewrites the cast.  Track via the existing TODO in
            # docs/formal-recovery-2026-05.md.
            [*sby_prefix, "tile_fanout_executor_formal.sby", "queue_no_overflow"],
            [*sby_prefix, "tile_fanout_executor_formal.sby", "local_enqueue_conservation"],
            [*sby_prefix, "tile_fanout_executor_formal.sby", "pending_mask_partition"],
            [*sby_prefix, "tile_fanout_executor_formal.sby", "retry_no_double_enqueue"],
            # Phase-2 watch-list item #3: rules out the
            # placeholder-emit-before-route-load race that the
            # multi_edge_return_addressing investigation hypothesised
            # (no noc_child emit may fire before a fanout-bank rsp
            # has been observed since reset).
            [*sby_prefix, "tile_fanout_executor_formal.sby", "no_emit_before_route_loaded"],
            [*sby_prefix, "tile_ingress_formal.sby", "ingress_no_drop"],
            [*sby_prefix, "tile_ingress_route_drain_formal.sby"],
            # Phase-2 watch-list item: previously failed at depth 12 on
            # a `(fires_q - consumes_q) <= INGRESS_QUEUE_DEPTH + 1`
            # ledger assertion.  Reformulated 2026-05 to assert directly
            # on the visible queue + classify register popcount, which
            # is bounded by 5 by construction.  All three properties
            # (onehot_or_broadcast, core_id_matches_target,
            # enqueue_fire_requires_match) now PASS — the actual
            # write_en stuck-at-0 regression they were meant to expose
            # was already fixed in commit 5d2f6ce (Phase 2 RTL fixes).
            [*sby_prefix, "tile_ingress_formal.sby", "ingress_onehot_or_broadcast"],
            [*sby_prefix, "tile_ingress_formal.sby", "core_id_matches_target"],
            [*sby_prefix, "tile_ingress_formal.sby", "enqueue_fire_requires_match"],
            # Phase 1.5 (2026-05): focused new harness for the
            # tile_event_queue_bank rv-handshake liveness in isolation.
            # All three tasks PASS — the bank itself is correct.  The
            # multi_edge_return_addressing deadlock therefore localizes
            # to the tile_top integration (`logical_event_full_c =
            # !eq_enqueue_ready[X]` at tile_top.sv:651 wires the bank's
            # "currently-firing" output through as "is-full", forming
            # a combinational loop with tile_ingress's
            # ingress_spike_enqueue_ready predicate).  Phase 2 RTL
            # fix lands at the tile_top level, not inside the bank.
            [*sby_prefix, "tile_event_queue_bank_handshake_formal.sby", "count_bound"],
            [*sby_prefix, "tile_event_queue_bank_handshake_formal.sby", "enqueue_liveness"],
            [*sby_prefix, "tile_event_queue_bank_handshake_formal.sby", "dequeue_after_push"],
            # Phase 3 Group E (2026-05): tile_eq_sched_formal had a
            # `smtbmc bitwuzla` engine line, but bitwuzla isn't in
            # the iic-osic-tools docker image.  Switched to yices
            # (the standard engine across the rest of the sweep) and
            # all 5 tasks PASS.
            [*sby_prefix, "tile_eq_sched_formal.sby", "coupled_dispatch_enqueue"],
            [*sby_prefix, "tile_eq_sched_formal.sby", "coupled_commits_bounded"],
            [*sby_prefix, "tile_eq_sched_formal.sby", "head_data_freshness"],
            [*sby_prefix, "tile_eq_sched_formal.sby", "refill_race_head_valid"],
            [*sby_prefix, "tile_eq_sched_formal.sby", "count_nonzero_tracks"],
            # Phase 3 follow-up (2026-05): partially-revived
            # tile_host_io* harnesses.  The deeper anti-double-push
            # property (emit_one_shot) and noc_healthy slot-liveness
            # property hit pre-existing failure modes that need
            # property reformulation; the count-bound and
            # arbitrary-nondet tasks PASS post-port-rename.
            [*sby_prefix, "tile_host_io_formal.sby", "fifo_count_bound"],
            [*sby_prefix, "tile_host_io_slot_liveness_formal.sby", "arbitrary_nondet"],
        ):
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "host_gateway":
        cwd = REPO_ROOT / "hw" / "ip" / "host_gateway" / "formal"
        for cmd in (
            # io_local_responder: response-path invariants.  Packet-field
            # tasks (pong_fields, hwinfo_mesh) deferred until yosys handles
            # $past on struct-valued function returns cleanly — see the
            # .sby for details.
            [*sby_prefix, "io_local_responder_formal.sby", "exclusivity"],
            [*sby_prefix, "io_local_responder_formal.sby", "drain"],
            # Phase 3 (2026-05): UART / UDP / transport_router migrated
            # from read_verilog to slang.  The harnesses' `$past` calls
            # were rewritten as explicit one-cycle delay registers
            # (slang doesn't implement $past).  Registering tasks that
            # PASS post-migration; the small handful that newly FAIL
            # surface real RTL/property issues and are tracked as
            # Phase 3 follow-ups (Group B) — not RE-deregistered, just
            # commented inline below.
            [*sby_prefix, "uart_transport_formal.sby", "disabled_idle"],
            [*sby_prefix, "uart_transport_formal.sby", "egress_count_bound"],
            [*sby_prefix, "uart_transport_formal.sby", "ingress_count_bound"],
            [*sby_prefix, "uart_transport_formal.sby", "reserve_eq"],
            [*sby_prefix, "uart_transport_formal.sby", "hold_on_backpressure"],
            [*sby_prefix, "uart_transport_formal.sby", "tx_launch"],
            # tx_frame_shape: PASSes after Group B fixed the stale
            # FRAME_BYTES=12 localparam (now (MESSAGE_W+7)/8 = 11).
            [*sby_prefix, "uart_transport_formal.sby", "tx_frame_shape"],
            [*sby_prefix, "uart_transport_formal.sby", "tx_no_silent_drop"],
            [*sby_prefix, "udp_transport_formal.sby", "disabled_idle"],
            [*sby_prefix, "udp_transport_formal.sby", "egress_count_bound"],
            [*sby_prefix, "udp_transport_formal.sby", "reserve_eq"],
            [*sby_prefix, "udp_transport_formal.sby", "rx_backpressure"],
            [*sby_prefix, "udp_transport_formal.sby", "tx_header_fields"],
            # Phase 3 (2026-05): all 5 transport_router_formal tasks
            # PASS after Group B fixed the stale MESSAGE_W=95 localparam
            # (now 83 to track tile_pkg's TILE_COORD_W: 8→5).
            [*sby_prefix, "transport_router_formal.sby", "disabled_idle"],
            [*sby_prefix, "transport_router_formal.sby", "single_passthrough"],
            [*sby_prefix, "transport_router_formal.sby", "route_modes"],
            [*sby_prefix, "transport_router_formal.sby", "ingress_arb"],
            [*sby_prefix, "transport_router_formal.sby", "route_cover"],
            # Post-refactor host_gateway egress chain coverage.
            # Phase 3 (2026-05): migrated several harnesses from
            # read_verilog to slang to dodge yosys's file-scope-import
            # rejection.  Most tasks PASS; the exceptions are listed
            # inline below with their failure mode (slang doesn't
            # implement $past, and three io_mesh_ingress_merge tasks
            # surface a PREUNSAT after migration that needs harness
            # debug).
            [*sby_prefix, "io_egress_port_fifo_formal.sby", "count_bound"],
            # Phase 3 (2026-05): $past refactored to delay registers.
            [*sby_prefix, "io_egress_port_fifo_formal.sby", "reset_count"],
            [*sby_prefix, "io_egress_port_fifo_formal.sby", "reserve_eq"],
            [*sby_prefix, "io_egress_port_fifo_formal.sby", "no_deadlock"],
            [*sby_prefix, "io_egress_classifier_formal.sby", "mutex_valid"],
            [*sby_prefix, "io_egress_classifier_formal.sby", "ready_path"],
            [*sby_prefix, "io_egress_classifier_formal.sby", "payload_bcast"],
            [*sby_prefix, "io_egress_classifier_formal.sby", "determinism"],
            # Phase 3 (2026-05): $past refactored to delay registers.
            [*sby_prefix, "io_egress_classifier_formal.sby", "cnt_monotonic"],
            [*sby_prefix, "io_egress_arb_formal.sby", "mutex_fire"],
            [*sby_prefix, "io_egress_arb_formal.sby", "local_prio"],
            [*sby_prefix, "io_egress_arb_formal.sby", "mesh_gating"],
            [*sby_prefix, "io_egress_arb_formal.sby", "out_valid_eq"],
            [*sby_prefix, "io_egress_arb_formal.sby", "no_starve"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "mutex_one_consumer"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "host_fastpath_disable"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "host_fastpath_gating"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "reserve_ok_passthrough"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "mesh_fastpath_reserve"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "no_packet_drop"],
            [*sby_prefix, "io_egress_fanout_formal.sby", "registered_drain_eventually_frees"],
            [*sby_prefix, "io_egress_chain_formal.sby", "mesh_to_host_fastpath"],
            [*sby_prefix, "io_response_arb_formal.sby", "mutex"],
            [*sby_prefix, "io_response_arb_formal.sby", "priority"],
            [*sby_prefix, "io_response_arb_formal.sby", "out_valid"],
            [*sby_prefix, "io_response_arb_formal.sby", "post_reset"],
            [*sby_prefix, "io_mesh_egress_dispatch_formal.sby", "src_stamp_nonloader"],
            # Phase 3 Group B (2026-05): src_stamp_loader_passthrough
            # PASSes after fixing the stale PACKET_W=95 → 83 + adding
            # a 83-bit formal_anyseq_u83 helper (the harness was using
            # u95 against an 83-bit interface).
            [*sby_prefix, "io_mesh_egress_dispatch_formal.sby", "src_stamp_loader_passthrough"],
            [*sby_prefix, "io_mesh_egress_dispatch_formal.sby", "closest_edge_mutex"],
            [*sby_prefix, "io_mesh_edge_adapter_formal.sby", "err_reset"],
            [*sby_prefix, "io_mesh_edge_adapter_formal.sby", "err_passthrough"],
            [*sby_prefix, "io_mesh_edge_adapter_formal.sby", "handshake_tx"],
            [*sby_prefix, "io_mesh_edge_adapter_formal.sby", "handshake_rx"],
            # Phase 3 Group C (2026-05): the original `slang`
            # migration of io_mesh_ingress_merge_formal produced a
            # PREUNSAT model — slang's plugin inserts a
            # formalff-cc:981 assume tied to the clock signal that
            # contradicts the BMC engine's clock-edge stepping for
            # this combinational-merge DUT.  Reverted to read_verilog
            # (the DUT doesn't include tile_flit_types.vh, so there's
            # no file-scope-import blocker).  All 6 tasks PASS.
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "mutex_grant"],
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "no_starvation_bounded"],
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "no_packet_drop"],
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "drain_liveness"],
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "no_phantom_fire"],
            [*sby_prefix, "io_mesh_ingress_merge_formal.sby", "progress_under_ready"],
            # Shadow composition proof for the maintained
            # mesh_router(host_divert) -> host_gateway(packet recovery)
            # boundary, split into one-body and zero-body host responses.
            #
            # Phase 3 Group B (2026-05): both PASS after fixing stale
            # PACKET_W=95 → 83.
            [*sby_prefix, "mesh_host_gateway_path_formal.sby", "one_body_roundtrip"],
            [*sby_prefix, "mesh_host_gateway_path_formal.sby", "zero_body_roundtrip"],
            [*sby_prefix, "mesh_edge_stamp_composition_formal.sby", "stamp_composition"],
            [*sby_prefix, "io_graph_clear_broadcaster_formal.sby", "valid_iff_pending"],
            # Phase 3 (2026-05): $past refactored to delay registers.
            [*sby_prefix, "io_graph_clear_broadcaster_formal.sby", "drain_clears"],
            [*sby_prefix, "io_graph_clear_broadcaster_formal.sby", "latch_semantics"],
            [*sby_prefix, "loader_controller_composition_formal.sby", "gate_exclusion"],
            [*sby_prefix, "loader_controller_composition_formal.sby", "clear_settle_ordering"],
            [*sby_prefix, "loader_controller_composition_formal.sby", "clear_bounded_latency"],
            [*sby_prefix, "loader_controller_composition_formal.sby", "tx_mutex"],
            # Narrow mesh_controller pending/drain checks covering the
            # host-out priority mux and telemetry quiescence assumptions.
            [*sby_prefix, "mesh_controller_pending_drain_formal.sby", "debug_drain"],
            [*sby_prefix, "mesh_controller_pending_drain_formal.sby", "trace_drain"],
            [*sby_prefix, "mesh_controller_pending_drain_formal.sby", "telemetry_no_spontaneous"],
        ):
            rc = run(cmd, cwd=cwd, env=formal_env)
            if rc != 0:
                return rc
        return 0

    if project == "soc":
        # The legacy `coldfoot_formal.sby` (which wrapped `noc_aer_core`)
        # was evicted.  SoC-level functional coverage is provided by the
        # cocotb targets in hw/SoC/test/ (soc_dual_neural_mesh etc.) and
        # by the mesh_router + bundle_loader formals.
        print("info: soc formal has no active targets after mesh migration", file=sys.stderr)
        return 0

    print(f"error: unsupported project '{project}'", file=sys.stderr)
    return 2


def _resolve_librelane_config(project: str, target: str, mode: str) -> Tuple[Path, str]:
    if project == "soc":
        mapping = {
            "rv_sw_async": "config_coldfoot.json",
            "coldfoot": "config_librelane_coldfoot.json",
            "default": "config_librelane_coldfoot.json",
        }
        if target not in mapping:
            raise ValueError("valid soc targets: rv_sw_async, coldfoot")
        return REPO_ROOT / "hw" / "SoC", mapping[target]

    raise ValueError(f"unsupported project '{project}'")


def _default_pnr_settings(project: str, target: str) -> tuple[str, str]:
    return PNR_DEFAULTS.get((project, target), (FALLBACK_PNR_PDK, FALLBACK_PNR_SCL))


def pnr(project: str, target: str, pdk: str | None, scl: str | None, pdk_root: str | None) -> int:
    try:
        cwd, config = _resolve_librelane_config(project, target, mode="pnr")
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if pdk is None or scl is None:
        default_pdk, default_scl = _default_pnr_settings(project, target)
        pdk = pdk or default_pdk
        scl = scl or default_scl

    env: Dict[str, str] = {}
    env["PDK_ROOT"] = pdk_root or DEFAULT_PDK_ROOT

    return run(
        ["librelane", "--manual-pdk", "--pdk-root", env["PDK_ROOT"], config, "--pdk", pdk, "--scl", scl],
        cwd=cwd,
        env=env,
    )


def openroad(project: str, target: str) -> int:
    try:
        cwd, _ = _resolve_librelane_config(project, target, mode="openroad")
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    runs_dir = cwd / "runs"
    if not runs_dir.exists():
        print("error: no LibreLane runs directory found; run the matching pnr target first", file=sys.stderr)
        return 2

    odb_runs = [
        run_dir
        for run_dir in runs_dir.iterdir()
        if run_dir.is_dir() and any((run_dir / "final" / "odb").glob("*.odb"))
    ]
    if not odb_runs:
        print("error: no final ODB found; run the matching pnr target first", file=sys.stderr)
        return 2

    latest_run = max(odb_runs, key=lambda p: p.stat().st_mtime)
    odb_files = sorted((latest_run / "final" / "odb").glob("*.odb"))
    odb_path = odb_files[0]
    print(f"[flow] opening ODB: {odb_path}")

    env: Dict[str, str] = {}
    runtime_dir = Path("/tmp/openroad-runtime")
    runtime_dir.mkdir(parents=True, exist_ok=True)
    runtime_dir.chmod(0o700)
    env["XDG_RUNTIME_DIR"] = str(runtime_dir)

    script_path = runtime_dir / "openroad_load_odb.tcl"
    script_path.write_text(f"read_db {odb_path}\n", encoding="utf-8")

    return run(["openroad", "-gui", str(script_path)], cwd=cwd, env=env)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run formal, LibreLane, and direct OpenROAD flows without project Makefiles.")
    sub = parser.add_subparsers(dest="mode", required=True)

    formal_p = sub.add_parser("formal", help="Run SymbiYosys formal flow")
    formal_p.add_argument(
        "--project",
        choices=["common", "logical_neuron", "neuron_compute", "neural_mesh", "tile", "host_gateway"],
        required=True,
    )
    formal_p.add_argument("--target", default=None)

    pnr_p = sub.add_parser("pnr", help="Run LibreLane synthesis/PnR")
    pnr_p.add_argument("--project", choices=["soc"], required=True)
    pnr_p.add_argument("--target", default="default")
    pnr_p.add_argument("--pdk", default=None)
    pnr_p.add_argument("--scl", default=None)
    pnr_p.add_argument("--pdk-root", default=os.getenv("PDK_ROOT", DEFAULT_PDK_ROOT))

    or_p = sub.add_parser("openroad", help="Launch the OpenROAD binary directly")
    or_p.add_argument("--project", choices=["soc"], required=True)
    or_p.add_argument("--target", default="default")

    args = parser.parse_args()

    if args.mode == "formal":
        return formal(args.project, args.target)
    if args.mode == "pnr":
        return pnr(args.project, args.target, args.pdk, args.scl, args.pdk_root)
    if args.mode == "openroad":
        return openroad(args.project, args.target)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
