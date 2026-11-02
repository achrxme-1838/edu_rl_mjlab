"""Deterministic inference for the unnormalized EDU A2 ELU actor."""

from pathlib import Path

import torch
from torch import nn

DEFAULT_CHECKPOINT = (
    Path(__file__).resolve().parents[3] / "policies/01_EDU_A2_STAIR/model_10001.pt"
)


class EduA2Policy(nn.Module):
    def __init__(self, checkpoint=DEFAULT_CHECKPOINT, device="cpu"):
        super().__init__()
        self.checkpoint = Path(checkpoint).expanduser().resolve()
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
        self.actor = nn.Sequential(
            nn.Linear(225, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 12),
        )
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
        if obs.ndim != 2 or obs.shape[-1] != 225:
            raise ValueError(
                f"Expected [batch, 225] actor observations, got {tuple(obs.shape)}"
            )
        return self.actor(obs)
