"""Check copied ONNX folders without connecting to robot DDS."""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from onnx import TensorProto, helper

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "deploy/robots/a2/build/a2_ctrl"
POLICY_DIR = ROOT / "networks/01_EDU_A2_STAIR"


@unittest.skipUnless(CONTROLLER.is_file(), "Build a2_ctrl first")
class PolicyLoadingTest(unittest.TestCase):
    def setUp(self):
        # Test Python edits in a disposable repository layout, never in the real config.
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.controller = self.root / "deploy/robots/a2/build/a2_ctrl"
        self.controller.parent.mkdir(parents=True)
        shutil.copy2(CONTROLLER, self.controller)
        self.config_dir = self.root / "deploy/configs/a2"
        shutil.copytree(ROOT / "deploy/configs/a2", self.config_dir, ignore=shutil.ignore_patterns("__pycache__"))
        # A regular local package must take precedence over the installed
        # editable deployment package in the mjlab interpreter.
        for relative in ("deploy/__init__.py", "deploy/configs/__init__.py"):
            shutil.copy2(ROOT / relative, self.root / relative)
        shutil.copytree(
            ROOT / "mjlab_wrapper", self.root / "mjlab_wrapper",
            ignore=shutil.ignore_patterns("__pycache__"),
        )
        self.config_source = (self.config_dir / "config.py").read_text()
        self.policy_dir = self.root / "networks/01_EDU_A2_STAIR"
        self.policy_dir.mkdir(parents=True)
        shutil.copy2(POLICY_DIR / "policy.onnx", self.policy_dir / "policy.onnx")

    def write_config(self, expression):
        (self.config_dir / "config.py").write_text(
            self.config_source + "\nfrom dataclasses import replace\n"
            "_base = A2DeployCfg()\ndef A2DeployCfg():\n    return " + expression + "\n"
        )

    def run_controller(self, folder=None, check=True, network=None):
        args = [str(self.controller)]
        if folder is not None:
            self.write_config(f"replace(_base, policy=replace(_base.policy, onnx={str(Path(folder) / 'policy.onnx')!r}))")
        if check:
            args += ["--check-policy"]
        if network is not None:
            args += ["--network", network]
        return subprocess.run(
            args, cwd="/tmp", env={**os.environ, "PATH": "/nonexistent", "A2_POLICY_DIR": "/not-used"},
            capture_output=True, text=True, timeout=20,
        )

    def test_selected_folder_needs_only_onnx_with_configured_mjlab_python(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / "policy with spaces"
            folder.mkdir()
            shutil.copy2(POLICY_DIR / "policy.onnx", folder / "policy.onnx")
            result = self.run_controller(folder)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(str(folder / "policy.onnx"), result.stdout)
            self.assertIn("no DDS channels opened", result.stdout)
            self.assertEqual([p.name for p in folder.iterdir()], ["policy.onnx"])

    def test_python_override_and_missing_interpreter_fail_before_dds(self):
        for python, success in ((sys.executable, True), ("/missing/a2-python", False)):
            with self.subTest(python=python):
                result = subprocess.run(
                    [str(self.controller), "--check-policy"], cwd="/tmp",
                    env={**os.environ, "A2_PYTHON": python, "PATH": "/nonexistent"},
                    capture_output=True, text=True, timeout=30,
                )
                self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
                self.assertNotIn("Waiting for connection", result.stdout)
                if not success:
                    self.assertIn("A2_PYTHON", result.stdout)

    def test_default_and_relative_folder_work_outside_repository(self):
        for folder in (None, "networks/01_EDU_A2_STAIR/"):
            with self.subTest(folder=folder):
                result = self.run_controller(folder)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(str(self.policy_dir / "policy.onnx"), result.stdout)

    def test_checkpoint_without_onnx_is_rejected_without_conversion_or_dds(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            (folder / "model_10001.pt").write_bytes(b"not an ONNX model")
            result = self.run_controller(folder, check=False)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("failed before DDS", result.stdout)
            self.assertIn("Run play in edu_rl_lab", result.stdout)
            self.assertFalse((folder / "policy.onnx").exists())
            self.assertNotIn("Waiting for connection", result.stdout)

    def test_invalid_onnx_and_wrong_action_count_fail_before_dds(self):
        graph = helper.make_graph(
            [helper.make_node("Identity", ["obs"], ["actions"])], "wrong_actions",
            [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 225])],
            [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 225])],
        )
        wrong_shape = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=8)
        for contents in (b"invalid model", wrong_shape.SerializeToString()):
            with self.subTest(size=len(contents)), tempfile.TemporaryDirectory() as temp:
                (Path(temp) / "policy.onnx").write_bytes(contents)
                result = self.run_controller(temp, check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("failed before DDS", result.stdout)
                self.assertNotIn("Waiting for connection", result.stdout)

    def test_invalid_network_config_fails_before_dds(self):
        for network in ("", "a2_missing_cfg"):
            with self.subTest(network=network):
                self.write_config(f"replace(_base, network=replace(_base.network, interface={network!r}))")
                result = self.run_controller(check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("Network configuration failed before DDS", result.stdout)
                self.assertNotIn("Waiting for connection", result.stdout)
                if network == "a2_missing_cfg":
                    self.assertIn("a2_missing_cfg", result.stdout)
        # Offline ONNX checks still work on a PC without the robot interface.
        result = self.run_controller()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_invalid_python_and_observation_shape_fail_before_dds(self):
        for expression in (
            "replace(_base, observations=replace(_base.observations, history_length=4))",
            "replace(_base, observations=replace(_base.observations, base_ang_vel=replace(_base.observations.base_ang_vel, func='missing')))",
            "replace(_base, real=replace(_base.real, kp=(25.0,)))",
            "replace(_base, network=replace(_base.network, interface=[]))",
            "1 / 0",
        ):
            with self.subTest(expression=expression):
                self.write_config(expression)
                result = self.run_controller(check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("failed before DDS", result.stdout)
                self.assertNotIn("Waiting for connection", result.stdout)

    def test_network_cli_overrides_config_before_dds(self):
        self.write_config("replace(_base, network=replace(_base.network, interface='a2_missing_cfg'))")
        result = self.run_controller(check=False, network="a2_missing_cli")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Network configuration failed before DDS", result.stdout)
        self.assertIn("a2_missing_cli", result.stdout)
        self.assertNotIn("a2_missing_cfg", result.stdout)
        self.assertNotIn("Waiting for connection", result.stdout)


if __name__ == "__main__":
    unittest.main()
