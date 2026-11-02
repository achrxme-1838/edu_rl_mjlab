"""Replay an EDU A2 policy in the registered flat or stairs environment."""

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

if "--headless" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")
if "--device=cpu" in sys.argv or (
    "--device" in sys.argv and sys.argv[sys.argv.index("--device") + 1 :][:1] == ["cpu"]
):
    os.environ["CUDA_VISIBLE_DEVICES"] = ""

import torch
import tyro
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from mjlab.utils.torch import configure_torch_backends
from mjlab.utils.wrappers import VideoRecorder
from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

from src.tasks.registry import list_tasks, load_task
from src.tasks.velocity.config.a2.policy import EduA2Policy


@dataclass(frozen=True)
class PlayConfig:
    agent: Literal["zero", "random", "trained"] = "trained"
    checkpoint_file: str | None = None
    num_envs: int | None = None
    device: str | None = None
    video: bool = False
    video_length: int = 200
    video_height: int | None = None
    video_width: int | None = None
    viewer: Literal["auto", "native", "viser"] = "auto"
    headless: bool = False
    steps: int = 200


def run_play(task_id: str, cfg: PlayConfig):
    configure_torch_backends()
    torch.set_num_threads(1)
    device = cfg.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
    task = load_task(task_id)
    env_cfg = task.env_cfg
    checkpoint = (
        Path(cfg.checkpoint_file).expanduser()
        if cfg.checkpoint_file
        else task.default_checkpoint
    )
    if cfg.agent == "trained" and (checkpoint is None or not checkpoint.is_file()):
        raise FileNotFoundError(
            "Provide a saved policy using --checkpoint-file /path/to/model.pt"
        )
    if cfg.steps < 1:
        raise ValueError("--steps must be positive")
    if cfg.num_envs is not None:
        env_cfg.scene.num_envs = cfg.num_envs
    if cfg.video_height is not None:
        env_cfg.viewer.height = cfg.video_height
    if cfg.video_width is not None:
        env_cfg.viewer.width = cfg.video_width

    raw = ManagerBasedRlEnv(
        env_cfg, device=device, render_mode="rgb_array" if cfg.video else None
    )
    render_env = raw
    if cfg.video:
        render_env = VideoRecorder(
            raw,
            video_folder=ROOT / "outputs/play" / task_id,
            step_trigger=lambda step: step == 0,
            video_length=cfg.video_length,
            disable_logger=True,
        )
    env = RslRlVecEnvWrapper(render_env, clip_actions=None)
    try:
        if cfg.agent == "trained":
            policy = EduA2Policy(checkpoint, device=env.device)
            print(f"[INFO] Checkpoint: {checkpoint}")
        else:

            def policy(obs):
                del obs
                shape = (env.num_envs, env.num_actions)
                return (
                    torch.zeros(shape, device=device)
                    if cfg.agent == "zero"
                    else 0.5 * torch.randn(shape, device=device)
                )

        if cfg.headless:
            with torch.inference_mode():
                obs = env.get_observations()
                for _ in range(cfg.steps):
                    actions = policy(obs)
                    if not torch.isfinite(actions).all():
                        raise FloatingPointError("Non-finite policy actions")
                    obs, _, _, _ = env.step(actions)
                    if not all(torch.isfinite(value).all() for value in obs.values()):
                        raise FloatingPointError("Non-finite observations")
            print(
                f"[PASS] {task_id}: {cfg.steps} playback steps, finite observations and actions"
            )
        else:
            viewer = cfg.viewer
            if viewer == "auto":
                viewer = (
                    "native"
                    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
                    else "viser"
                )
            (NativeMujocoViewer if viewer == "native" else ViserPlayViewer)(
                env, policy
            ).run()
    finally:
        env.close()


def main():
    cli_flags = (tyro.conf.AvoidSubcommands,)
    task_id, remaining = tyro.cli(
        tyro.extras.literal_type_from_choices(list_tasks()),
        add_help=False,
        return_unknown_args=True,
        config=cli_flags,
    )
    cfg = tyro.cli(
        PlayConfig,
        args=remaining,
        prog=f"{sys.argv[0]} {task_id}",
        config=cli_flags,
    )
    run_play(task_id, cfg)


if __name__ == "__main__":
    main()
