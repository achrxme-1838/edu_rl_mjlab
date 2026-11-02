"""Check copied ONNX folders without connecting to robot DDS."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from onnx import TensorProto, helper
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "deploy/robots/a2/build/a2_ctrl"
POLICY_DIR = ROOT / "networks/01_EDU_A2_STAIR"


@unittest.skipUnless(CONTROLLER.is_file(), "Build a2_ctrl first")
class PolicyLoadingTest(unittest.TestCase):
    def setUp(self):
        # Test YAML edits in a disposable repository layout, never in the real config.
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.controller = self.root / "deploy/robots/a2/build/a2_ctrl"
        self.controller.parent.mkdir(parents=True)
        shutil.copy2(CONTROLLER, self.controller)
        self.config_dir = self.root / "deploy/robots/a2/config"
        shutil.copytree(ROOT / "deploy/robots/a2/config", self.config_dir)
        self.policy_dir = self.root / "networks/01_EDU_A2_STAIR"
        self.policy_dir.mkdir(parents=True)
        shutil.copy2(POLICY_DIR / "policy.onnx", self.policy_dir / "policy.onnx")

    def run_controller(self, folder=None, check=True, network=None):
        args = [str(self.controller)]
        if folder is not None:
            path = self.config_dir / "deploy.yaml"
            config = yaml.safe_load(path.read_text())
            config["policy_dir"] = str(folder)
            path.write_text(yaml.safe_dump(config))
        if check:
            args += ["--check-policy"]
        if network is not None:
            args += ["--network", network]
        return subprocess.run(
            args, cwd="/tmp", env={**os.environ, "PATH": "/nonexistent", "A2_POLICY_DIR": "/not-used"},
            capture_output=True, text=True, timeout=20,
        )

    def test_selected_folder_needs_only_onnx_and_no_python(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / "policy with spaces"
            folder.mkdir()
            shutil.copy2(POLICY_DIR / "policy.onnx", folder / "policy.onnx")
            result = self.run_controller(folder)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(str(folder / "policy.onnx"), result.stdout)
            self.assertIn("no DDS channels opened", result.stdout)
            self.assertEqual([p.name for p in folder.iterdir()], ["policy.onnx"])

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
        path = self.config_dir / "config.yaml"
        config = yaml.safe_load(path.read_text())
        for network in (None, {}, {"interface": ""}, {"interface": []}, {"interface": "a2_missing_cfg"}):
            with self.subTest(network=network):
                config["network"] = network
                path.write_text(yaml.safe_dump(config))
                result = self.run_controller(check=False)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("Network configuration failed before DDS", result.stdout)
                self.assertNotIn("Waiting for connection", result.stdout)
                if network == {"interface": "a2_missing_cfg"}:
                    self.assertIn("a2_missing_cfg", result.stdout)
        # Offline ONNX checks still work on a PC without the robot interface.
        result = self.run_controller()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_network_cli_overrides_config_before_dds(self):
        path = self.config_dir / "config.yaml"
        config = yaml.safe_load(path.read_text())
        config["network"] = {"interface": "a2_missing_cfg"}
        path.write_text(yaml.safe_dump(config))
        result = self.run_controller(check=False, network="a2_missing_cli")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Network configuration failed before DDS", result.stdout)
        self.assertIn("a2_missing_cli", result.stdout)
        self.assertNotIn("a2_missing_cfg", result.stdout)
        self.assertNotIn("Waiting for connection", result.stdout)


if __name__ == "__main__":
    unittest.main()
