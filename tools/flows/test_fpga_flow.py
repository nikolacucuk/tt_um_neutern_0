import importlib.util
import pathlib
import sys
import unittest
from unittest import mock


FLOW_PATH = pathlib.Path(__file__).resolve().parent / "fpga_flow.py"
SPEC = importlib.util.spec_from_file_location("coldfoot_fpga_flow", FLOW_PATH)
assert SPEC is not None and SPEC.loader is not None
fpga_flow = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = fpga_flow
SPEC.loader.exec_module(fpga_flow)


class FpgaFlowClockUartTests(unittest.TestCase):
    def test_40mhz_2mbps_is_clean(self) -> None:
        analysis = fpga_flow.analyze_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            sys_clk_hz=40_000_000,
            uart_baud=2_000_000,
        )
        errors, warnings = fpga_flow.validate_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            analysis,
            require_supported_uart=True,
        )
        self.assertEqual(errors, [])
        self.assertEqual(warnings, [])
        self.assertTrue(analysis.mmcm_exact)
        self.assertTrue(analysis.uart_exact)

    def test_24mhz_fails_mmcm_exactness(self) -> None:
        analysis = fpga_flow.analyze_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            sys_clk_hz=24_000_000,
            uart_baud=1_500_000,
        )
        errors, _warnings = fpga_flow.validate_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            analysis,
            require_supported_uart=True,
        )
        self.assertTrue(any("does not divide MMCM VCO" in err for err in errors))

    def test_40mhz_1600000_is_experimental(self) -> None:
        analysis = fpga_flow.analyze_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            sys_clk_hz=40_000_000,
            uart_baud=1_600_000,
        )
        errors, _warnings = fpga_flow.validate_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            analysis,
            require_supported_uart=True,
        )
        self.assertTrue(any("outside the maintained/supported set" in err for err in errors))

    def test_25mhz_2mbps_fails_uart_error_budget(self) -> None:
        analysis = fpga_flow.analyze_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            sys_clk_hz=25_000_000,
            uart_baud=2_000_000,
        )
        errors, _warnings = fpga_flow.validate_clock_uart(
            fpga_flow.NEXYS_VIDEO,
            analysis,
            require_supported_uart=True,
        )
        self.assertTrue(any("differs from requested" in err for err in errors))


class FpgaFlowStatusTests(unittest.TestCase):
    def test_parse_vivado_phase_line_with_detail(self) -> None:
        status = fpga_flow._parse_vivado_phase_line("COLDFOOT_VIVADO_PHASE:place|Explore\n")
        self.assertEqual(status, fpga_flow.VivadoPhaseStatus(phase="place", detail="Explore"))

    def test_parse_vivado_phase_line_without_detail(self) -> None:
        status = fpga_flow._parse_vivado_phase_line("COLDFOOT_VIVADO_PHASE:route\n")
        self.assertEqual(status, fpga_flow.VivadoPhaseStatus(phase="route", detail=None))

    def test_parse_vivado_phase_line_rejects_other_output(self) -> None:
        self.assertIsNone(fpga_flow._parse_vivado_phase_line("INFO: [Common 17-206] Exiting Vivado\n"))

    def test_make_vivado_status_formatter_includes_elapsed_time(self) -> None:
        formatter = fpga_flow.make_vivado_status_formatter(
            10.0,
            fpga_flow.GitBuildInfo(short_hash="abc1234", subject="Add bitstream footer"),
        )
        rendered = formatter(fpga_flow.VivadoPhaseStatus(phase="bitstream", detail=None), 75.0)
        self.assertIn("phase: bitstream", rendered)
        self.assertIn("elapsed 01:05", rendered)
        self.assertIn("abc1234 Add bitstream footer", rendered)

    def test_format_git_build_label_truncates_subject(self) -> None:
        label = fpga_flow._format_git_build_label(
            fpga_flow.GitBuildInfo(
                short_hash="abc1234",
                subject="This is a very long commit subject that should not take over the whole footer line",
            )
        )
        self.assertIn("abc1234 ", label)
        self.assertTrue(label.endswith("..."))
        self.assertLessEqual(len(label), len("abc1234 ") + 48)

    def test_get_git_build_info_parses_git_log_output(self) -> None:
        completed = mock.Mock(stdout="abc1234\x00Improve Vivado footer\n")
        with mock.patch.object(fpga_flow.subprocess, "run", return_value=completed) as run_mock:
            info = fpga_flow.get_git_build_info()
        self.assertEqual(info, fpga_flow.GitBuildInfo(short_hash="abc1234", subject="Improve Vivado footer"))
        run_mock.assert_called_once()


if __name__ == "__main__":
    unittest.main()
