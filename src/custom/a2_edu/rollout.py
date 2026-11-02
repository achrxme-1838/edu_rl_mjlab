"""Measured, reset-free sim2sim rollouts with optional video."""

import hashlib
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path

import numpy as np
import torch


def run_rollout(env, policy, *, steps, terrain, course, command, video=None):
    if steps < 1:
        raise ValueError("steps must be positive")
    raw = env.unwrapped
    robot = raw.scene["robot"]
    obs = env.get_observations()
    start = robot.data.root_link_pos_w[0].cpu().numpy().copy()
    positions, velocities, yaw_velocities, upright_values = [], [], [], []
    failure = None
    resets = 0
    peak_action = 0.0
    reached_landing = False
    stayed_on_course = True
    writer = None
    if video:
        import imageio.v2 as imageio

        Path(video).parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(
            str(video), fps=round(1 / raw.step_dt), macro_block_size=1
        )
    try:
        with torch.inference_mode():
            for step in range(steps):
                if not torch.isfinite(obs["actor"]).all():
                    failure = f"non-finite observation before step {step}"
                    break
                actions = policy(obs)
                if not torch.isfinite(actions).all():
                    failure = f"non-finite action at step {step}"
                    break
                peak_action = max(peak_action, float(actions.abs().max()))
                obs, _, dones, _ = env.step(actions)
                resets += int(dones.sum())
                state = torch.cat(
                    (
                        robot.data.root_link_pos_w,
                        robot.data.root_link_quat_w,
                        robot.data.root_link_lin_vel_b,
                        robot.data.root_link_ang_vel_b,
                        robot.data.joint_pos,
                        robot.data.joint_vel,
                    ),
                    dim=-1,
                )
                if (
                    not torch.isfinite(state).all()
                    or not torch.isfinite(obs["actor"]).all()
                ):
                    failure = f"non-finite state/observation after step {step}"
                    break
                pos = robot.data.root_link_pos_w[0].cpu().numpy().copy()
                upright = float(-robot.data.projected_gravity_b[0, 2])
                positions.append(pos)
                velocities.append(
                    robot.data.root_link_lin_vel_b[0].cpu().numpy().copy()
                )
                yaw_velocities.append(float(robot.data.root_link_ang_vel_b[0, 2]))
                upright_values.append(upright)
                if terrain == "stairs":
                    in_course = course.start_x <= pos[0] <= course.end_x
                    if in_course and abs(pos[1]) > course.width / 2 - 0.25:
                        stayed_on_course = False
                    if (
                        course.landing_start + 0.3
                        <= pos[0]
                        <= course.landing_start + course.landing_length - 0.3
                        and abs(pos[1]) < course.width / 2 - 0.25
                        and pos[2] > course.step_count * course.step_height + 0.18
                    ):
                        reached_landing = True
                if writer:
                    writer.append_data(raw.render())
                if resets:
                    failure = f"unexpected environment reset at step {step}"
                    break
                if upright < 0.5 or pos[2] < 0.18:
                    failure = f"fall at step {step}"
                    break
                if (step + 1) % 250 == 0:
                    print(
                        f"[rollout] {(step + 1) * raw.step_dt:.1f}s, position={pos.round(3).tolist()}",
                        flush=True,
                    )
    finally:
        if writer:
            writer.close()
    elapsed = len(positions) * raw.step_dt
    end = positions[-1] if positions else start
    warmup = min(round(2.0 / raw.step_dt), max(0, len(velocities) - 1))
    mean_vel = np.mean(velocities[warmup:], axis=0) if velocities else np.zeros(3)
    mean_yaw = float(np.mean(yaw_velocities[warmup:])) if yaw_velocities else 0.0
    tracking_error = float(np.linalg.norm(mean_vel[:2] - np.array(command[:2])))
    tracking_ok = tracking_error <= max(0.2, 0.5 * np.linalg.norm(command[:2])) and abs(
        mean_yaw - command[2]
    ) <= 0.3 + 0.5 * abs(command[2])
    completed = (
        bool(
            reached_landing
            and stayed_on_course
            and end[0] >= course.end_x + 0.7
            and end[2] < 0.65
        )
        if terrain == "stairs"
        else None
    )
    passed = (
        failure is None
        and len(positions) == steps
        and (completed if terrain == "stairs" else tracking_ok)
    )
    return {
        "passed": bool(passed),
        "failure": failure,
        "terrain": terrain,
        "checkpoint": str(policy.checkpoint),
        "checkpoint_sha256": hashlib.sha256(policy.checkpoint.read_bytes()).hexdigest(),
        "checkpoint_iteration": policy.iteration,
        "device": str(env.device),
        "seed": raw.cfg.seed,
        "versions": {
            name: version(name)
            for name in ("torch", "mjlab", "mujoco", "mujoco-warp", "warp-lang")
        },
        "command": list(command),
        "requested_steps": steps,
        "completed_steps": len(positions),
        "simulated_seconds": elapsed,
        "control_dt": raw.step_dt,
        "reset_count": resets,
        "start_position": start.tolist(),
        "end_position": end.tolist(),
        "displacement": (end - start).tolist(),
        "mean_body_velocity_after_warmup": mean_vel.tolist(),
        "mean_yaw_velocity_after_warmup": mean_yaw,
        "planar_velocity_error": tracking_error,
        "min_upright_cosine": min(upright_values) if upright_values else None,
        "min_base_height": min(float(p[2]) for p in positions) if positions else None,
        "max_base_height": max(float(p[2]) for p in positions) if positions else None,
        "peak_absolute_action": peak_action,
        "reached_landing": reached_landing if terrain == "stairs" else None,
        "stayed_on_course": stayed_on_course if terrain == "stairs" else None,
        "completed_stair_course": completed,
        "stair_course": asdict(course) if terrain == "stairs" else None,
        "video": str(Path(video).resolve()) if video else None,
    }
