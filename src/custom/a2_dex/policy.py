"""Load the saved Isaac RSL-RL actor without changing either RSL-RL package."""

from pathlib import Path

import torch
from torch import nn


class IsaacActor(nn.Module):
    def __init__(self, checkpoint: Path, device: str):
        super().__init__()
        saved = torch.load(checkpoint, map_location="cpu", weights_only=True)
        state = saved["model_state_dict"]
        # This bridge supports the inspected ActorCritic/ELU checkpoint contract.
        unexpected = [key for key in state if not key.startswith(("actor.", "critic.")) and key != "std"]
        if unexpected:
            raise ValueError(f"Unsupported checkpoint state (e.g. normalization): {unexpected}")
        self.actor = nn.Sequential(
            nn.Linear(225, 512), nn.ELU(), nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(), nn.Linear(128, 12),
        )
        self.actor.load_state_dict(
            {key.removeprefix("actor."): value for key, value in state.items() if key.startswith("actor.")},
            strict=True,
        )
        self.iteration = saved.get("iter")
        self.to(device).eval()
        self.requires_grad_(False)

    @torch.inference_mode()
    def forward(self, observations):
        obs = observations["actor"]
        if obs.shape[-1] != 225:
            raise ValueError(f"Expected 225 actor observations, got {tuple(obs.shape)}")
        return self.actor(obs)
