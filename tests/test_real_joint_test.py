"""Check real_joint_test CLI validation without opening DDS channels."""

import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "deploy/robots/a2/build/real_joint_test"


@unittest.skipUnless(BINARY.is_file(), "Build the C++ real_joint_test target first")
class CppJointDiagnosticTest(unittest.TestCase):
    def run_cli(self, *options):
        return subprocess.run(
            [str(BINARY), *options],
            cwd="/tmp",
            stdin=subprocess.DEVNULL,
            text=True,
            capture_output=True,
            timeout=5,
        )

    def test_help_shows_only_supported_modes(self):
        result = self.run_cli("--help")
        self.assertEqual(result.returncode, 0)
        self.assertIn("--network", result.stdout)
        self.assertNotIn("--dry-run", result.stdout)
        self.assertNotIn("--print-mapping", result.stdout)
        self.assertNotIn("Connecting to A2", result.stdout)

    def test_removed_modes_are_rejected_before_connecting(self):
        for option in ("--dry-run", "--print-mapping"):
            with self.subTest(option=option):
                result = self.run_cli(option)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("unrecognised option", result.stderr)
                self.assertNotIn("Connecting to A2", result.stdout)

    def test_invalid_arguments_are_rejected_before_connecting(self):
        for options, error in (
            (["--amplitude", "0.16"], "amplitude"),
            (["--amplitude", "nan"], "amplitude"),
            (["--period", "1"], "period"),
            (["--stand-duration", "2"], "stand"),
            (["--order", "wrong"], "--order"),
            (["--connect-timeout", "0"], "--connect-timeout"),
            ([], "--network"),
            (["--network", "a2_missing_nic"], "--network"),
        ):
            with self.subTest(options=options):
                result = self.run_cli(*options)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(error, result.stderr)
                self.assertNotIn("Connecting to A2", result.stdout)

    def test_network_from_config_and_cli_override(self):
        # JSON is valid YAML. An interactive-terminal error proves interface
        # selection passed, while DEVNULL stdin prevents any DDS connection.
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / "config.yaml"
            for saved, cli, expected_error in (
                ("lo", (), "interactive terminal"),
                ("lo", ("--network", "a2_missing_cli"), "a2_missing_cli"),
                ("a2_missing_cfg", (), "a2_missing_cfg"),
                ("a2_missing_cfg", ("--network", "lo"), "interactive terminal"),
                ("lo", ("--network", ""), "must not be empty"),
            ):
                with self.subTest(saved=saved, cli=cli):
                    config.write_text(json.dumps({
                        "network": {"interface": saved},
                        "FSM": {"Passive": {"kd": [10] * 12}},
                    }))
                    result = self.run_cli("--fsm-config", str(config), *cli)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(expected_error, result.stderr)
                    self.assertNotIn("Connecting to A2", result.stdout)


if __name__ == "__main__":
    unittest.main()
