"""Deterministic inference for the unnormalized EDU A2 ELU actor."""

from pathlib import Path

import torch
from torch import nn

from deploy.configs.a2 import load_config, resolve_path, validate

DEFAULT_CHECKPOINT = resolve_path(load_config().policy.checkpoint)


class EduA2Policy(nn.Module):
    def __init__(self, checkpoint=None, device="cpu", deploy_cfg=None):
        super().__init__()
        cfg = validate(deploy_cfg) if deploy_cfg is not None else load_config()
        self.observation_dim = cfg.observations.dimension
        self.checkpoint = (
            resolve_path(cfg.policy.checkpoint)
            if checkpoint is None
            else Path(checkpoint).expanduser().resolve()
        )
        payload = torch.load(self.checkpoint, map_location="cpu", weights_only=True)
        state = payload["model_state_dict"]
        allowed = {"model_state_dict", "optimizer_state_dict", "iter", "infos"}
        if set(payload) - allowed:
            raise ValueError(
                "Unsupported checkpoint metadata/normalization; expected an EDU A2 checkpoint."
            )
        if any(
            not (key == "std" or key.startswith(("actor.", "critic."))) for key in state
        ):
            raise ValueError(
                "Unsupported policy state: this adapter expects no observation normalization."
            )
        self.iteration = payload.get("iter")
        dims = cfg.actor_dims
        layers = []
        for index, (size_in, size_out) in enumerate(zip(dims, dims[1:])):
            layers.append(nn.Linear(size_in, size_out))
            if index < len(dims) - 2:
                layers.append(nn.ELU())
        self.actor = nn.Sequential(*layers)
        self.actor.load_state_dict(
            {
                k.removeprefix("actor."): v
                for k, v in state.items()
                if k.startswith("actor.")
            },
            strict=True,
        )
        if not all(torch.isfinite(p).all() for p in self.actor.parameters()):
            raise ValueError("Checkpoint contains non-finite actor weights.")
        self.to(device).eval()
        self.requires_grad_(False)

    @torch.inference_mode()
    def forward(self, observations):
        obs = (
            observations
            if isinstance(observations, torch.Tensor)
            else observations["actor"]
        )
        if obs.ndim != 2 or obs.shape[-1] != self.observation_dim:
            raise ValueError(
                f"Expected [batch, {self.observation_dim}] actor observations, got {tuple(obs.shape)}"
            )
        return self.actor(obs)
