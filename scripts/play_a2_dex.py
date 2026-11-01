"""Replay the Isaac-trained A2 actor with its own opt-in mjlab configuration."""

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
# Select EGL before importing MuJoCo's rendering modules.
if "--headless" in sys.argv:
    os.environ.setdefault("MUJOCO_GL", "egl")

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper
from src.custom.a2_dex.env_cfg import A2DexConfig, POLICY_JOINT_NAMES, make_env_cfg
from src.custom.a2_dex.policy import IsaacActor

DEFAULT_CHECKPOINT = (
    REPO.parent / "DEX_RL_LAB/scripts/logs/rsl_rl/a2_base_rl/01_STAIR_SUCCESS/model_10000.pt"
)


def output_path(value: Path) -> Path:
    path = value.expanduser().resolve()
    source = (REPO.parent / "DEX_RL_LAB").resolve()
    if path == source or source in path.parents:
        raise ValueError("Outputs must be outside DEX_RL_LAB")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def run_headless(env, policy, steps, video):
    if steps <= 0:
        raise ValueError("--steps must be positive")
    robot = env.unwrapped.scene["robot"]
    obs = env.get_observations()
    start = robot.data.root_link_pos_w[0].cpu().tolist()
    min_height, min_upright, max_action = float("inf"), 1.0, 0.0
    max_height = start[2]
    first_fall = None
    mean_velocity = torch.zeros(3, device=env.device)
    writer = None
    if video is not None:
        import imageio.v2 as imageio
        writer = imageio.get_writer(str(output_path(video)), fps=round(1 / env.unwrapped.step_dt))
    started = time.perf_counter()
    try:
        for step in range(steps):
            action = policy(obs)
            obs, _, done, _ = env.step(action)
            if not all(torch.isfinite(value).all() for value in (
                action, robot.data.root_link_pose_w, robot.data.root_link_vel_w,
                robot.data.joint_pos, robot.data.joint_vel,
            )):
                raise RuntimeError("Non-finite policy action or robot state")
            if done.any():
                raise RuntimeError("Unexpected reset in inference-only environment")
            min_height = min(min_height, robot.data.root_link_pos_w[0, 2].item())
            max_height = max(max_height, robot.data.root_link_pos_w[0, 2].item())
            min_upright = min(min_upright, -robot.data.projected_gravity_b[0, 2].item())
            if first_fall is None and (min_upright < 0.5 or min_height < 0.18):
                first_fall = {
                    "simulation_seconds": (step + 1) * env.unwrapped.step_dt,
                    "position": robot.data.root_link_pos_w[0].cpu().tolist(),
                }
            max_action = max(max_action, action.abs().max().item())
            mean_velocity += robot.data.root_link_lin_vel_b[0]
            if writer is not None:
                writer.append_data(env.unwrapped.render())
    finally:
        if writer is not None:
            writer.close()
    wall_seconds = time.perf_counter() - started
    return {
        "steps": steps, "simulation_seconds": steps * env.unwrapped.step_dt,
        "wall_seconds": wall_seconds,
        "realtime_factor": steps * env.unwrapped.step_dt / wall_seconds,
        "initial_position": start,
        "final_position": robot.data.root_link_pos_w[0].cpu().tolist(),
        "mean_body_velocity": (mean_velocity / steps).cpu().tolist(),
        "min_base_height_world": min_height, "min_upright_cosine": min_upright,
        "max_base_height_world": max_height, "first_fall": first_fall,
        "max_abs_raw_action": max_action,
        "fall_detected": min_upright < 0.5 or min_height < 0.18,
        "all_finite": True, "automatic_resets": 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--terrain", choices=("flat", "stairs"), default=None)
    parser.add_argument("--command", type=float, nargs=3, metavar=("VX", "VY", "WZ"), default=None)
    parser.add_argument("--stair-height", type=float, default=None)
    parser.add_argument("--stair-width", type=float, default=None)
    parser.add_argument("--stair-count", type=int, default=None)
    device_args = parser.add_mutually_exclusive_group()
    device_args.add_argument("--device", default="cuda:0", help="Physics and policy device: cuda:0 or cpu")
    device_args.add_argument("--cpu", action="store_const", const="cpu", dest="device",
                             help="Run both MuJoCo Warp physics and policy inference on CPU")
    parser.add_argument("--viewer", choices=("auto", "native", "viser"), default="auto")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--video", type=Path, help="Save an MP4 during a finite headless rollout")
    args = parser.parse_args()
    if args.video is not None and not args.headless:
        parser.error("--video requires --headless")
    if args.device == "cpu":
        # Set before Warp initialization; CPU execution must not create CUDA contexts.
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
    checkpoint = args.checkpoint.expanduser().resolve(strict=True)
    torch.set_num_threads(1)
    custom = A2DexConfig()
    overrides = {name: getattr(args, name) for name in
                 ("terrain", "stair_height", "stair_width", "stair_count")
                 if getattr(args, name) is not None}
    if args.command is not None:
        overrides["command"] = tuple(args.command)
    custom = replace(custom, **overrides)
    policy = IsaacActor(checkpoint, args.device)
    raw = ManagerBasedRlEnv(make_env_cfg(custom), device=args.device,
                           render_mode="rgb_array" if args.video else None)
    try:
        env = RslRlVecEnvWrapper(raw, clip_actions=None)
        robot = raw.scene["robot"]
        action_names = raw.action_manager.get_term("joint_pos").target_names
        if tuple(action_names) != POLICY_JOINT_NAMES:
            raise RuntimeError(f"Action order mismatch: {action_names}")
        policy_to_mujoco = [robot.joint_names.index(name) for name in POLICY_JOINT_NAMES]
        metadata = {
            "checkpoint": str(checkpoint), "iteration": policy.iteration,
            "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
            "custom_config": asdict(custom), "policy_joint_names": POLICY_JOINT_NAMES,
            "mujoco_joint_names": robot.joint_names,
            "policy_to_mujoco_indices": policy_to_mujoco,
            "actor_input_shape": list(env.get_observations()["actor"].shape),
            "physics_device": str(raw.sim.wp_device),
            "policy_device": str(next(policy.parameters()).device),
            "body_mass_policy": "Original mjlab model masses and inertias retained",
        }
        print(json.dumps(metadata, indent=2), flush=True)
        with torch.inference_mode():
            if args.headless:
                metadata["rollout"] = run_headless(env, policy, args.steps, args.video)
                device_suffix = "_cpu" if args.device == "cpu" else ""
                report = output_path(args.report or REPO / "outputs/a2_dex" / f"{custom.terrain}{device_suffix}_report.json")
                report.write_text(json.dumps(metadata, indent=2) + "\n")
                print(json.dumps(metadata["rollout"], indent=2), flush=True)
                print(f"Report: {report}", flush=True)
            else:
                from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer
                viewer = args.viewer
                if viewer == "auto":
                    viewer = "native" if os.environ.get("DISPLAY") else "viser"
                if viewer == "native":
                    NativeMujocoViewer(env, policy).run()
                else:
                    ViserPlayViewer(env, policy).run()
    finally:
        raw.close()


if __name__ == "__main__":
    main()
