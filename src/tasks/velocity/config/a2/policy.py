"""ONNX inference matching the A2 C++ deployment controller."""

from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
from torch import nn

from deploy.configs.a2 import load_config, resolve_path, validate

DEFAULT_POLICY = resolve_path(load_config().policy.onnx)


class EduA2Policy(nn.Module):
    def __init__(self, onnx=None, device="cpu", deploy_cfg=None):
        super().__init__()
        cfg = validate(deploy_cfg) if deploy_cfg is not None else load_config()
        self.observation_dim = cfg.observations.dimension
        self.action_dim = cfg.action_dim
        self.device = torch.device(device)
        self.policy_path = (
            resolve_path(cfg.policy.onnx)
            if onnx is None
            else Path(onnx).expanduser().resolve()
        )
        if self.policy_path.suffix.lower() != ".onnx":
            raise ValueError("Expected an exported .onnx policy; .pt checkpoints are not supported.")
        if not self.policy_path.is_file():
            raise FileNotFoundError(f"ONNX policy not found: {self.policy_path}")

        # Match the C++ OrtRunner's CPU backend, optimization and thread counts.
        options = ort.SessionOptions()
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_EXTENDED
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(self.policy_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (
            len(inputs) != 1
            or inputs[0].name != "obs"
            or inputs[0].type != "tensor(float)"
            or inputs[0].shape != [1, self.observation_dim]
        ):
            raise ValueError(
                f"Policy must have one float32 input 'obs' with shape [1, {self.observation_dim}]."
            )
        if (
            len(outputs) != 1
            or outputs[0].type != "tensor(float)"
            or outputs[0].shape != [1, self.action_dim]
        ):
            raise ValueError(
                f"Policy must have one float32 output with shape [1, {self.action_dim}]."
            )
        self.output_name = outputs[0].name
        self.eval()
        # Fail before constructing the simulator, as the real controller does.
        self(torch.zeros(1, self.observation_dim))

    @torch.inference_mode()
    def forward(self, observations):
        obs = (
            observations
            if isinstance(observations, torch.Tensor)
            else observations["actor"]
        )
        if tuple(obs.shape) != (1, self.observation_dim):
            raise ValueError(
                f"Expected [1, {self.observation_dim}] actor observations, got {tuple(obs.shape)}"
            )
        values = obs.detach().to(device="cpu", dtype=torch.float32).contiguous().numpy()
        if not np.isfinite(values).all():
            raise ValueError("Policy observations contain non-finite values.")
        actions = self.session.run([self.output_name], {"obs": values})[0]
        if actions.shape != (1, self.action_dim) or not np.isfinite(actions).all():
            raise ValueError("Policy returned invalid or non-finite actions.")
        return torch.from_numpy(actions).to(self.device)
