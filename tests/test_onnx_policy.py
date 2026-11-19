"""Validate ONNX-only inference and the deployment input/output contract."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import torch

from deploy.configs.a2 import load_config
from src.tasks.velocity.config.a2.policy import DEFAULT_POLICY, EduA2Policy


class OnnxPolicyTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.cfg = load_config()

    def write_policy(self, *, input_shape=None, input_name="obs", input_type=TensorProto.FLOAT,
                     action_dim=None, output_type=TensorProto.FLOAT, value=0.25,
                     extra_input=False, extra_output=False):
        action_dim = self.cfg.action_dim if action_dim is None else action_dim
        dtype = np.float32 if output_type == TensorProto.FLOAT else np.int64
        output = np.full((1, action_dim), value, dtype=dtype)
        inputs = [helper.make_tensor_value_info(
            input_name, input_type,
            [1, self.cfg.observations.dimension] if input_shape is None else input_shape,
        )]
        outputs = [helper.make_tensor_value_info("actions", output_type, [1, action_dim])]
        nodes = [helper.make_node("Constant", [], ["actions"], value=numpy_helper.from_array(output))]
        if extra_input:
            inputs.append(helper.make_tensor_value_info("state", TensorProto.FLOAT, [1, 8]))
        if extra_output:
            outputs.append(helper.make_tensor_value_info("state_out", output_type, [1, action_dim]))
            nodes.append(helper.make_node("Identity", ["actions"], ["state_out"]))
        graph = helper.make_graph(nodes, "contract_fixture", inputs, outputs)
        path = self.directory / "policy.onnx"
        onnx.save(helper.make_model(
            graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=8
        ), path)
        return path

    def test_config_selection_and_explicit_override_need_only_onnx(self):
        selected = self.write_policy()
        cfg = replace(self.cfg, policy=replace(self.cfg.policy, onnx=str(selected)))
        with patch("torch.load", side_effect=AssertionError("Must not read a .pt checkpoint")):
            policy = EduA2Policy(deploy_cfg=cfg)
            self.assertEqual(policy.policy_path, selected)
            self.assertEqual(policy.session.get_providers(), ["CPUExecutionProvider"])
            obs = torch.zeros(1, cfg.observations.dimension)
            torch.testing.assert_close(policy({"actor": obs}), torch.full((1, cfg.action_dim), 0.25))
            override = EduA2Policy(DEFAULT_POLICY, deploy_cfg=cfg)
            self.assertEqual(override.policy_path, DEFAULT_POLICY)

    def test_invalid_signatures_fail_at_load(self):
        variants = (
            {"input_shape": [1, self.cfg.observations.dimension - 1]},
            {"input_shape": ["batch", self.cfg.observations.dimension]},
            {"input_shape": [2, self.cfg.observations.dimension]},
            {"input_name": "other"},
            {"input_type": TensorProto.DOUBLE},
            {"extra_input": True},
            {"action_dim": self.cfg.action_dim + 1},
            {"output_type": TensorProto.INT64},
            {"extra_output": True},
        )
        for variant in variants:
            with self.subTest(variant=variant):
                path = self.write_policy(**variant)
                with self.assertRaisesRegex(ValueError, "Policy must have one float32"):
                    EduA2Policy(path)

    def test_missing_and_pt_files_are_rejected(self):
        with self.assertRaisesRegex(FileNotFoundError, "ONNX policy not found"):
            EduA2Policy(self.directory / "missing.onnx")
        with self.assertRaisesRegex(ValueError, "exported .onnx"):
            EduA2Policy(self.directory / "model.pt")

    def test_invalid_observations_are_rejected(self):
        policy = EduA2Policy(self.write_policy())
        dim = self.cfg.observations.dimension
        for shape in ((dim,), (2, dim), (1, dim - 1)):
            with self.subTest(shape=shape), self.assertRaisesRegex(ValueError, "Expected"):
                policy(torch.zeros(shape))
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "non-finite"):
                policy(torch.full((1, dim), value))

    def test_nonfinite_model_is_rejected_before_simulation(self):
        with self.assertRaisesRegex(ValueError, "non-finite actions"):
            EduA2Policy(self.write_policy(value=float("nan")))

    def test_nonfinite_actions_are_rejected_during_inference(self):
        dim = self.cfg.observations.dimension
        weights = np.full((dim, self.cfg.action_dim), 1e38, dtype=np.float32)
        graph = helper.make_graph(
            [helper.make_node("MatMul", ["obs", "weights"], ["actions"])],
            "overflow",
            [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, dim])],
            [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, self.cfg.action_dim])],
            initializer=[numpy_helper.from_array(weights, name="weights")],
        )
        path = self.directory / "overflow.onnx"
        onnx.save(helper.make_model(
            graph, opset_imports=[helper.make_opsetid("", 17)], ir_version=8
        ), path)
        policy = EduA2Policy(path)  # Zero-input preflight is finite.
        with self.assertRaisesRegex(ValueError, "non-finite actions"):
            policy(torch.ones(1, dim))


if __name__ == "__main__":
    unittest.main()
