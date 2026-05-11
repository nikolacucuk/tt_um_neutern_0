#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time
from html import escape
from pathlib import Path
from typing import List

from pushover_notify import notify_from_config

try:
    import yaml
except ModuleNotFoundError:  # pragma: no cover - PyYAML is expected in normal flows.
    yaml = None


REPO_ROOT = Path(__file__).resolve().parents[2]
COCOTB_FLOW = REPO_ROOT / "tools" / "flows" / "cocotb_flow.py"
TOOL_FLOW = REPO_ROOT / "tools" / "flows" / "tool_flow.py"
FPGA_FLOW = REPO_ROOT / "tools" / "flows" / "fpga_flow.py"

LINT_TARGETS = {
    "neural_mesh_rv_sw_async": ("coldfoot:ip:neural_mesh:0.1.0", "lint_rv_sw_async"),
    "neural_mesh_fabric": ("coldfoot:ip:neural_mesh:0.1.0", "lint_neural_mesh_fabric"),
    "mesh_bundle_loader": ("coldfoot:ip:neural_mesh:0.1.0", "lint_mesh_bundle_loader"),
    "packet_to_flit": ("coldfoot:ip:neural_mesh:0.1.0", "lint_neural_mesh_fabric"),
    "flit_to_packet": ("coldfoot:ip:neural_mesh:0.1.0", "lint_neural_mesh_fabric"),
    "mesh_router": ("coldfoot:ip:neural_mesh:0.1.0", "lint_mesh_router"),
    "mesh_router_flit_routing": ("coldfoot:ip:neural_mesh:0.1.0", "lint_mesh_router"),
    "mesh_router_isolation": ("coldfoot:ip:neural_mesh:0.1.0", "lint_mesh_router"),
    "mesh_router_plane_tag": ("coldfoot:ip:neural_mesh:0.1.0", "lint_mesh_router"),
    "multi_gateway_edge": ("coldfoot:ip:neural_mesh:0.1.0", "lint_multi_gateway_edge"),
    "multi_hni": ("coldfoot:ip:neural_mesh:0.1.0", "lint_multi_gateway_edge"),
    "soc": ("coldfoot:soc:coldfoot:0.1.0", "lint_coldfoot"),
    "soc_rv_sw_async": ("coldfoot:soc:coldfoot:0.1.0", "lint_rv_sw_async"),
    "soc_dual_neural_mesh": ("coldfoot:soc:coldfoot:0.1.0", "lint_dual_neural_mesh"),
}

SIM_TARGETS = {
    "calc_if_loopback": ("tile", "tile_smoke"),
    "tile_calc_debug": ("tile", "tile_smoke"),
    "neuron_compute_core": ("neuron_compute", "neuron_compute_core"),
    "logical_neuron_context_bank": ("logical_neuron", "logical_neuron_context_bank"),
    "logical_neuron_state_bank": ("logical_neuron", "logical_neuron_state_bank"),
    "neural_mesh_rv_sw_async": ("neural_mesh", "rv_sw_async"),
    "neural_mesh_fabric": ("neural_mesh", "neural_mesh_fabric"),
    "mesh_bundle_loader": ("neural_mesh", "mesh_bundle_loader"),
    "packet_to_flit": ("neural_mesh", "packet_to_flit"),
    "flit_to_packet": ("neural_mesh", "flit_to_packet"),
    "mesh_router": ("neural_mesh", "mesh_router"),
    "mesh_router_flit_routing": ("neural_mesh", "mesh_router_flit_routing"),
    "mesh_router_isolation": ("neural_mesh", "mesh_router_isolation"),
    "mesh_router_urgent": ("neural_mesh", "mesh_router_urgent"),
    "mesh_router_starvation": ("neural_mesh", "mesh_router_starvation"),
    "mesh_router_plane_tag": ("neural_mesh", "mesh_router_plane_tag"),
    "multi_gateway_edge": ("neural_mesh", "multi_gateway_edge"),
    "multi_hni": ("neural_mesh", "multi_gateway_edge"),
    "host_gateway": ("host_gateway", "host_gateway"),
    "host_gateway_priority": ("host_gateway", "host_gateway_priority"),
    "host_gateway_io_egress_chain": ("host_gateway", "io_egress_chain"),
    # Phase 2 follow-up: these were wired in cocotb_flow.py's registry
    # but never surfaced through flow.py, so run_all_sims.sh missed them.
    # Re-exposing here brings the pure-RTL host-gateway component tests
    # into the green sweep.
    "host_gateway_io_egress_classifier": ("host_gateway", "io_egress_classifier"),
    "host_gateway_io_egress_port_fifo":  ("host_gateway", "io_egress_port_fifo"),
    "soc_coldfoot": ("soc", "coldfoot"),
    "multi_edge_return_addressing": ("soc", "multi_edge_return_addressing"),
    "soc_simrtl_coldfoot": ("soc", "simrtl"),
    "soc_rv_sw_async": ("soc", "rv_sw_async"),
    "soc_dual_neural_mesh": ("soc", "dual_neural_mesh"),
}

FORMAL_TARGETS = {
    "common": ("common", None),
    "logical_neuron": ("logical_neuron", None),
    "neural_mesh": ("neural_mesh", None),
    "neuron_compute": ("neuron_compute", None),
    "host_gateway": ("host_gateway", None),
    "tile": ("tile", None),
}

PNR_TARGETS = {
    # Standalone neural_mesh IP PnR was evicted along with `noc_aer_top.sv`;
    # the `soc_coldfoot` / `soc_rv_sw_async` flows harden the same mesh
    # through the SoC top.
    "soc_rv_sw_async": ("soc", "coldfoot"),
    "soc_coldfoot": ("soc", "coldfoot"),
}

OPENROAD_TARGETS = {
    "soc_coldfoot": ("soc", "coldfoot"),
}

FALLBACK_PNR_PDK = "sky130A"
FALLBACK_PNR_SCL = "sky130_fd_sc_hd"
PNR_DEFAULTS = {
    "soc_coldfoot": ("gf180mcuD", "gf180mcu_fd_sc_mcu7t5v0"),
    "soc_rv_sw_async": ("gf180mcuD", "gf180mcu_fd_sc_mcu7t5v0"),
}


def run(cmd: List[str]) -> int:
    print(f"[flow] exec: {' '.join(shlex.quote(part) for part in cmd)}")
    env = os.environ.copy()
    if os.name != "nt":
        path_parts = ["/foss/tools/klayout", "/foss/tools/bin"]
        current_path = env.get("PATH", "")
        if current_path:
            path_parts.append(current_path)
        env["PATH"] = os.pathsep.join(path_parts)

        ld_parts = ["/foss/tools/iverilog/lib"]
        current_ld = env.get("LD_LIBRARY_PATH", "")
        if current_ld:
            ld_parts.append(current_ld)
        env["LD_LIBRARY_PATH"] = os.pathsep.join(ld_parts)
    try:
        proc = subprocess.run(cmd, cwd=str(REPO_ROOT), env=env)
    except FileNotFoundError:
        print(f"error: command not found: {cmd[0]}", file=sys.stderr)
        return 127
    return proc.returncode


def _format_duration(seconds: float) -> str:
    total_seconds = max(0, int(round(seconds)))
    minutes, secs = divmod(total_seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours}h {minutes}m {secs}s"
    if minutes:
        return f"{minutes}m {secs}s"
    return f"{secs}s"


def _emit_terminal_bell() -> None:
    try:
        sys.stderr.write("\a")
        sys.stderr.flush()
    except Exception:
        pass


def _powershell_single_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def _emit_windows_toast(title: str, message: str) -> None:
    powershell = shutil.which("powershell") or shutil.which("pwsh")
    if powershell is None:
        return

    title_xml = escape(title, quote=False)
    message_xml = escape(message, quote=False)
    script = "\n".join(
        [
            "$title = " + _powershell_single_quote(title),
            "$message = " + _powershell_single_quote(message),
            "if (Get-Command New-BurntToastNotification -ErrorAction SilentlyContinue) {",
            "  New-BurntToastNotification -Text $title, $message",
            "  exit 0",
            "}",
            "[Windows.UI.Notifications.ToastNotificationManager, Windows.UI.Notifications, ContentType = WindowsRuntime] > $null",
            "[Windows.Data.Xml.Dom.XmlDocument, Windows.Data.Xml.Dom.XmlDocument, ContentType = WindowsRuntime] > $null",
            "$xml = @\"",
            "<toast>",
            "  <visual>",
            "    <binding template=\"ToastGeneric\">",
            f"      <text>{title_xml}</text>",
            f"      <text>{message_xml}</text>",
            "    </binding>",
            "  </visual>",
            "</toast>",
            "\"@",
            "$doc = New-Object Windows.Data.Xml.Dom.XmlDocument",
            "$doc.LoadXml($xml)",
            "$toast = [Windows.UI.Notifications.ToastNotification]::new($doc)",
            "$notifier = [Windows.UI.Notifications.ToastNotificationManager]::CreateToastNotifier('coldfoot_soc')",
            "$notifier.Show($toast)",
        ]
    )
    try:
        subprocess.run(
            [powershell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", script],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except Exception:
        pass


def _notify_fpga_completion(mode: str, rc: int, duration_seconds: float) -> None:
    status = "completed" if rc == 0 else "failed"
    duration = _format_duration(duration_seconds)
    title = f"coldfoot_soc {mode} {status}"
    message = f"{mode} {status} in {duration} (exit {rc})"
    print(f"[flow] {message}")
    _emit_terminal_bell()
    if os.name == "nt":
        _emit_windows_toast(title, message)
    try:
        notify_from_config(title, message)
    except Exception as exc:
        print(f"warning: Pushover notification failed: {exc}", file=sys.stderr)


def _parse_int_env(name: str) -> int | None:
    value = os.getenv(name)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        print(f"error: environment variable {name} must be an integer (got '{value}')", file=sys.stderr)
        raise SystemExit(2)


def _find_eda_yaml(start_dir: Path) -> Path | None:
    for candidate in sorted(start_dir.glob("*.eda.yml")):
        return candidate
    return None


def _parse_int_like(value: object, name: str, source: Path) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        try:
            return int(value, 0)
        except ValueError:
            print(
                f"error: parameter {name} in {source} must be an integer (got '{value}')",
                file=sys.stderr,
            )
            raise SystemExit(2)
    return None


def _load_eda_param_defaults(start_dir: Path) -> dict[str, int]:
    if yaml is None:
        return {}
    eda_path = _find_eda_yaml(start_dir)
    if eda_path is None:
        return {}
    try:
        with eda_path.open("r", encoding="utf-8") as handle:
            eda = yaml.safe_load(handle)
    except Exception as exc:
        print(f"warning: failed to parse {eda_path}: {exc}", file=sys.stderr)
        return {}
    params = eda.get("parameters")
    if not isinstance(params, dict):
        return {}
    defaults: dict[str, int] = {}
    for name in ("X_DIM", "Y_DIM", "Z_DIM", "WORKER_DIM", "CLK_HZ", "UART_BAUD", "TELEMETRY_STREAM_PERIOD_CYCLES"):
        param_info = params.get(name)
        if not isinstance(param_info, dict):
            continue
        value = _parse_int_like(param_info.get("default"), name, eda_path)
        if value is not None:
            defaults[name] = value
    return defaults


def lint(name: str) -> int:
    if name not in LINT_TARGETS:
        print(f"error: unsupported lint target '{name}'", file=sys.stderr)
        return 2
    core, target = LINT_TARGETS[name]
    return run(["fusesoc", "run", "--target", target, core])


def compile_target(name: str) -> int:
    # `compile` is a migration alias for lint/elaboration checks.
    return lint(name)


def sim(
    name: str,
    sim_name: str | None,
    waves: str | None,
    mesh_x: int | None,
    mesh_y: int | None,
    mesh_z: int | None,
    worker_dim: int | None,
    telemetry_stream_period_cycles: int | None,
    threads: int | None,
    jobs: int | None,
) -> int:
    if name not in SIM_TARGETS:
        print(f"error: unsupported sim target '{name}'", file=sys.stderr)
        return 2
    project, target = SIM_TARGETS[name]
    cmd = [sys.executable, str(COCOTB_FLOW), "--project", project, "--target", target]
    if sim_name:
        cmd += ["--sim", sim_name]
    if waves is not None:
        cmd += ["--waves", waves]
    if mesh_x is not None:
        cmd += ["--mesh-x", str(mesh_x)]
    if mesh_y is not None:
        cmd += ["--mesh-y", str(mesh_y)]
    if mesh_z is not None:
        cmd += ["--mesh-z", str(mesh_z)]
    if worker_dim is not None:
        cmd += ["--worker-dim", str(worker_dim)]
    if telemetry_stream_period_cycles is not None:
        cmd += ["--telemetry-stream-period-cycles", str(telemetry_stream_period_cycles)]
    if threads is not None:
        cmd += ["--threads", str(threads)]
    if jobs is not None:
        cmd += ["--jobs", str(jobs)]
    return run(cmd)


def formal(name: str) -> int:
    if name not in FORMAL_TARGETS:
        print(f"error: unsupported formal target '{name}'", file=sys.stderr)
        return 2
    project, target = FORMAL_TARGETS[name]
    cmd = [sys.executable, str(TOOL_FLOW), "formal", "--project", project]
    if target:
        cmd += ["--target", target]
    return run(cmd)


def _default_pnr_settings(name: str) -> tuple[str, str]:
    return PNR_DEFAULTS.get(name, (FALLBACK_PNR_PDK, FALLBACK_PNR_SCL))


def pnr(name: str, pdk: str | None, scl: str | None, pdk_root: str | None) -> int:
    if name not in PNR_TARGETS:
        print(f"error: unsupported pnr target '{name}'", file=sys.stderr)
        return 2
    project, target = PNR_TARGETS[name]
    if pdk is None or scl is None:
        default_pdk, default_scl = _default_pnr_settings(name)
        pdk = pdk or default_pdk
        scl = scl or default_scl
    cmd = [
        sys.executable,
        str(TOOL_FLOW),
        "pnr",
        "--project",
        project,
        "--target",
        target,
        "--pdk",
        pdk,
        "--scl",
        scl,
    ]
    if pdk_root:
        cmd += ["--pdk-root", pdk_root]
    return run(cmd)


def openroad(name: str) -> int:
    if name not in OPENROAD_TARGETS:
        print(f"error: unsupported openroad target '{name}'", file=sys.stderr)
        return 2
    project, target = OPENROAD_TARGETS[name]
    cmd = [sys.executable, str(TOOL_FLOW), "openroad", "--project", project, "--target", target]
    return run(cmd)


def fpga_build(
    board: str,
    top: str,
    build_dir: str | None,
    synth_only: bool,
    use_full_core: bool,
    sys_clk_hz: int,
    mesh_x: int,
    mesh_y: int,
    mesh_z: int,
    worker_dim: int,
    uart_baud: int,
    threads: int,
    allow_experimental_uart: bool,
) -> int:
    cmd = [
        sys.executable,
        str(FPGA_FLOW),
        "build",
        "--board",
        board,
    ]
    if build_dir:
        cmd += [
            "--build-dir",
            build_dir,
        ]
    cmd += [
        "--top",
        top,
        "--sys-clk-hz",
        str(sys_clk_hz),
        "--mesh-x",
        str(mesh_x),
        "--mesh-y",
        str(mesh_y),
        "--mesh-z",
        str(mesh_z),
        "--worker-dim",
        str(worker_dim),
        "--uart-baud",
        str(uart_baud),
        "--threads",
        str(threads),
    ]
    if synth_only:
        cmd.append("--synth-only")
    if use_full_core:
        cmd.append("--use-full-core")
    if allow_experimental_uart:
        cmd.append("--allow-experimental-uart")
    return run(cmd)


def fpga_check(
    board: str,
    sys_clk_hz: int,
    uart_baud: int,
    strict: bool,
) -> int:
    cmd = [
        sys.executable,
        str(FPGA_FLOW),
        "check",
        "--board",
        board,
        "--sys-clk-hz",
        str(sys_clk_hz),
        "--uart-baud",
        str(uart_baud),
    ]
    if strict:
        cmd.append("--strict")
    return run(cmd)


def fpga_program(board: str, bitstream: str) -> int:
    return run(
        [
            sys.executable,
            str(FPGA_FLOW),
            "program",
            "--board",
            board,
            "--bitstream",
            bitstream,
        ]
    )


def fpga_flash(
    board: str,
    bitstream: str,
    cfgmem_file: str | None,
    cfgmem_part: str | None,
    interface: str | None,
    size_mbit: int | None,
) -> int:
    cmd = [
        sys.executable,
        str(FPGA_FLOW),
        "flash",
        "--board",
        board,
        "--bitstream",
        bitstream,
    ]
    if cfgmem_file:
        cmd += ["--cfgmem-file", cfgmem_file]
    if cfgmem_part:
        cmd += ["--cfgmem-part", cfgmem_part]
    if interface:
        cmd += ["--interface", interface]
    if size_mbit is not None:
        cmd += ["--size-mbit", str(size_mbit)]
    return run(cmd)


def run_graph(args: argparse.Namespace) -> int:
    """Build or rebuild the Graphify knowledge graph."""
    try:
        import graphify  # noqa: F401  # Following Qodo rule: verify tool availability before use.
    except ModuleNotFoundError:
        print("error: graphify not found. Run: pip install graphifyy", file=sys.stderr)
        return 1
    cmd = [sys.executable, str(REPO_ROOT / "tools" / "agent" / "build_graphify.py"), "--config", str(REPO_ROOT / "graphify.toml")]
    if getattr(args, "force", False):
        cmd.append("--force")
    print(f"+ {' '.join(shlex.quote(part) for part in cmd)}")
    result = subprocess.run(cmd, cwd=str(REPO_ROOT))
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Project flow dispatcher for the FuseSoC migration period.")
    sub = parser.add_subparsers(dest="mode", required=True)

    lint_p = sub.add_parser("lint", help="Run FuseSoC lint target")
    lint_p.add_argument("target", choices=sorted(LINT_TARGETS.keys()))

    compile_p = sub.add_parser("compile", help="Run compile/elaboration gate (alias of lint)")
    compile_p.add_argument("target", choices=sorted(LINT_TARGETS.keys()))

    sim_p = sub.add_parser("sim", help="Run cocotb simulation flow target")
    sim_p.add_argument("target", choices=sorted(SIM_TARGETS.keys()))
    sim_p.add_argument("--sim", choices=["icarus", "verilator"], default=None)
    sim_p.add_argument("--waves", default=None)
    sim_p.add_argument("--mesh-x", type=int, default=None)
    sim_p.add_argument("--mesh-y", type=int, default=None)
    sim_p.add_argument("--mesh-z", type=int, default=None)
    sim_p.add_argument("--worker-dim", type=int, default=None)
    sim_p.add_argument("--telemetry-stream-period-cycles", type=int, default=None)
    sim_p.add_argument("--threads", type=int, default=None)
    sim_p.add_argument("--jobs", type=int, default=None)

    formal_p = sub.add_parser("formal", help="Run formal flow target")
    formal_p.add_argument("target", choices=sorted(FORMAL_TARGETS.keys()))

    pnr_p = sub.add_parser("pnr", help="Run LibreLane flow target")
    pnr_p.add_argument("target", choices=sorted(PNR_TARGETS.keys()))
    pnr_p.add_argument("--pdk", default=None)
    pnr_p.add_argument("--scl", default=None)
    pnr_p.add_argument("--pdk-root", default=None)

    or_p = sub.add_parser("openroad", help="Launch the OpenROAD binary directly")
    or_p.add_argument("target", choices=sorted(OPENROAD_TARGETS.keys()))

    fpga_build_p = sub.add_parser("fpga_build", help="Run Vivado FPGA synthesis/build flow")
    fpga_build_p.add_argument("board", choices=["nexys_video"])
    fpga_build_p.add_argument("--top", choices=["heartbeat", "coldfoot"], default="heartbeat")
    fpga_build_p.add_argument("--build-dir", default=None)
    fpga_build_p.add_argument("--synth-only", action="store_true", default=False)
    fpga_build_p.add_argument("--use-full-core", action="store_true", default=False)
    fpga_build_p.add_argument("--sys-clk-hz", type=int, default=40_000_000)
    fpga_build_p.add_argument("--mesh-x", type=int, default=2)
    fpga_build_p.add_argument("--mesh-y", type=int, default=2)
    fpga_build_p.add_argument("--mesh-z", type=int, default=172)
    fpga_build_p.add_argument("--worker-dim", type=int, default=4)
    fpga_build_p.add_argument("--uart-baud", type=int, default=2_000_000)
    fpga_build_p.add_argument("--threads", type=int, default=24)
    fpga_build_p.add_argument("--allow-experimental-uart", action="store_true", default=False)

    fpga_check_p = sub.add_parser("fpga_check", help="Check FPGA clock/UART settings before a build")
    fpga_check_p.add_argument("board", choices=["nexys_video"])
    fpga_check_p.add_argument("--sys-clk-hz", type=int, default=40_000_000)
    fpga_check_p.add_argument("--uart-baud", type=int, default=2_000_000)
    fpga_check_p.add_argument("--strict", action="store_true", default=False)

    fpga_program_p = sub.add_parser("fpga_program", help="Program FPGA bitstream with Vivado hardware manager")
    fpga_program_p.add_argument("board", choices=["nexys_video"])
    fpga_program_p.add_argument("--bitstream", required=True)

    fpga_flash_p = sub.add_parser("fpga_flash", help="Program non-volatile FPGA configuration flash with Vivado")
    fpga_flash_p.add_argument("board", choices=["nexys_video"])
    fpga_flash_p.add_argument("--bitstream", required=True)
    fpga_flash_p.add_argument("--cfgmem-file", default=None)
    fpga_flash_p.add_argument("--cfgmem-part", default=None)
    fpga_flash_p.add_argument("--interface", default=None)
    fpga_flash_p.add_argument("--size-mbit", type=int, default=None)

    p_graph = sub.add_parser("graph", help="Build the Graphify knowledge graph for AI agents")
    p_graph.add_argument("--force", action="store_true", help="Force full rebuild even if graph is current")
    p_graph.set_defaults(func=run_graph)

    list_p = sub.add_parser("list", help="List target names")
    list_p.add_argument(
        "kind",
        choices=["lint", "compile", "sim", "formal", "pnr", "openroad", "fpga_board", "fpga_top"],
    )

    args = parser.parse_args()
    fpga_start_time = time.monotonic()

    if args.mode == "lint":
        return lint(args.target)
    if args.mode == "compile":
        return compile_target(args.target)
    if args.mode == "sim":
        eda_defaults = _load_eda_param_defaults(Path.cwd())
        mesh_x = args.mesh_x
        mesh_y = args.mesh_y
        mesh_z = args.mesh_z
        worker_dim = args.worker_dim
        if mesh_x is None:
            mesh_x = _parse_int_env("SOC_MESH_X")
        if mesh_x is None:
            mesh_x = eda_defaults.get("X_DIM")
        if mesh_y is None:
            mesh_y = _parse_int_env("SOC_MESH_Y")
        if mesh_y is None:
            mesh_y = eda_defaults.get("Y_DIM")
        if mesh_z is None:
            mesh_z = _parse_int_env("SOC_MESH_Z")
        if mesh_z is None:
            mesh_z = eda_defaults.get("Z_DIM")
        if worker_dim is None:
            worker_dim = eda_defaults.get("WORKER_DIM")
        if worker_dim is None:
            worker_dim = _parse_int_env("SOC_WORKER_DIM")
        if worker_dim is None:
            # Default sim to WORKER_DIM=2 when no eda.yml / env / CLI is
            # present.  The previous fallback was `worker_dim = mesh_z`,
            # which silently forced huge worker counts (e.g. --mesh-z 24
            # → WORKER_DIM=24) and OOM-killed cc1plus on the verilator
            # build of neuron_compute_core.  Two workers is enough for
            # every first-order sim_coldfoot smoke — the big host-facing
            # tests (test_01..04) pass with WORKER_DIM=2 at mesh 2x2 z=24
            # in roughly 10 minutes on the OSIC container build host.
            worker_dim = 2
        # tile_top requires WORKER_CORES_PER_TILE <= NEURONS_PER_TILE; when
        # mesh_z is left unset (SV default = 1) but worker_dim is 2 the
        # elaboration assertion fires.  Bump mesh_z up to at least
        # worker_dim so the maintained sim defaults always elaborate.
        if mesh_z is None:
            mesh_z = worker_dim
        elif mesh_z < worker_dim:
            mesh_z = worker_dim
        telemetry_stream_period_cycles = args.telemetry_stream_period_cycles
        if telemetry_stream_period_cycles is None:
            telemetry_stream_period_cycles = _parse_int_env("SOC_TELEMETRY_STREAM_PERIOD_CYCLES")
        if telemetry_stream_period_cycles is None:
            telemetry_stream_period_cycles = eda_defaults.get("TELEMETRY_STREAM_PERIOD_CYCLES")
        return sim(
            args.target,
            args.sim,
            args.waves,
            mesh_x,
            mesh_y,
            mesh_z,
            worker_dim,
            telemetry_stream_period_cycles,
            args.threads,
            args.jobs,
        )
    if args.mode == "formal":
        return formal(args.target)
    if args.mode == "pnr":
        return pnr(args.target, args.pdk, args.scl, args.pdk_root)
    if args.mode == "openroad":
        return openroad(args.target)
    if args.mode == "fpga_build":
        if args.top == "coldfoot" and not args.use_full_core:
            print(
                "[flow] warning: building coldfoot without --use-full-core selects the interface-free "
                "bring-up core (no SoC UART command path).",
                file=sys.stderr,
            )
        worker_dim = args.worker_dim
        rc = fpga_build(
            args.board,
            args.top,
            args.build_dir,
            args.synth_only,
            args.use_full_core,
            args.sys_clk_hz,
            args.mesh_x,
            args.mesh_y,
            args.mesh_z,
            worker_dim,
            args.uart_baud,
            args.threads,
            args.allow_experimental_uart,
        )
        _notify_fpga_completion(args.mode, rc, time.monotonic() - fpga_start_time)
        return rc
    if args.mode == "fpga_check":
        return fpga_check(args.board, args.sys_clk_hz, args.uart_baud, args.strict)
    if args.mode == "fpga_program":
        rc = fpga_program(args.board, args.bitstream)
        return rc
    if args.mode == "fpga_flash":
        rc = fpga_flash(
            args.board,
            args.bitstream,
            args.cfgmem_file,
            args.cfgmem_part,
            args.interface,
            args.size_mbit,
        )
        return rc
    if args.mode == "graph":
        return args.func(args)
    if args.mode == "list":
        mapping = {
            "lint": LINT_TARGETS,
            "compile": LINT_TARGETS,
            "sim": SIM_TARGETS,
            "formal": FORMAL_TARGETS,
            "pnr": PNR_TARGETS,
            "openroad": OPENROAD_TARGETS,
            "fpga_board": {"nexys_video": None},
            "fpga_top": {"heartbeat": None, "coldfoot": None},
        }[args.kind]
        for key in sorted(mapping.keys()):
            print(key)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
