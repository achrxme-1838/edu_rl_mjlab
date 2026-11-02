#!/usr/bin/env python3
"""Play or validate the bundled EDU A2 policy in mjlab."""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=ROOT / "networks/01_EDU_A2_STAIR/model_10001.pt",
    )
    parser.add_argument("--terrain", choices=("flat", "stairs"), default="stairs")
    parser.add_argument(
        "--command",
        nargs=3,
        type=float,
        default=(0.4, 0.0, 0.0),
        metavar=("VX", "VY", "WZ"),
    )
    parser.add_argument("--stair-height", type=float, default=0.10)
    parser.add_argument(
        "--stair-width", type=float, default=0.30, help="Tread depth in metres"
    )
    parser.add_argument("--stair-count", type=int, default=5)
    parser.add_argument(
        "--device", default=None, help="cpu or cuda:0 (default: CUDA if available)"
    )
    parser.add_argument("--viewer", choices=("auto", "native", "viser"), default="auto")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--steps", type=int, default=2000, help="50 control steps = 1 simulated second"
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--report",
        type=Path,
        help="Headless JSON report; default: outputs/sim2sim/<timestamp>/report.json",
    )
    parser.add_argument("--video", type=Path, help="Headless MP4 output")
    parser.add_argument(
        "--check", action="store_true", help="Exit nonzero if headless validation fails"
    )
    args = parser.parse_args()
    if not args.headless and (args.check or args.report or args.video):
        parser.error("--check, --report and --video require --headless")
    if args.steps < 1:
        parser.error("--steps must be positive")
    if (
        args.headless
        and args.terrain == "stairs"
        and args.check
        and (args.command[0] <= 0 or any(args.command[1:]))
    ):
        parser.error(
            "Stair completion validation needs a straight forward command: --command VX 0 0"
        )
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    if args.device == "cpu":
        os.environ["CUDA_VISIBLE_DEVICES"] = ""

    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper

    from src.tasks.velocity.config.a2.env_cfgs import StairCourse, make_env_cfg
    from src.tasks.velocity.config.a2.policy import EduA2Policy
    from src.tasks.velocity.config.a2.rollout import run_rollout

    torch.set_num_threads(1)
    device = args.device or ("cuda:0" if torch.cuda.is_available() else "cpu")
    course = StairCourse(
        step_height=args.stair_height,
        step_width=args.stair_width,
        step_count=args.stair_count,
    )
    policy = EduA2Policy(args.checkpoint, device=device)
    cfg = make_env_cfg(
        terrain=args.terrain, command=args.command, course=course, seed=args.seed
    )
    raw = ManagerBasedRlEnv(
        cfg, device=device, render_mode="rgb_array" if args.video else None
    )
    env = RslRlVecEnvWrapper(raw, clip_actions=None)
    try:
        print(
            f"[INFO] EDU checkpoint: {policy.checkpoint} (iteration {policy.iteration})",
            flush=True,
        )
        if args.headless:
            report_path = (
                args.report
                or ROOT
                / "outputs/sim2sim"
                / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                / "report.json"
            )
            result = run_rollout(
                env,
                policy,
                steps=args.steps,
                terrain=args.terrain,
                course=course,
                command=args.command,
                video=args.video,
            )
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            print(json.dumps(result, indent=2, allow_nan=False))
            print(f"[INFO] Report: {report_path.resolve()}")
            return 1 if args.check and not result["passed"] else 0
        from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

        viewer = args.viewer
        if viewer == "auto":
            viewer = (
                "native"
                if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
                else "viser"
            )
        (NativeMujocoViewer if viewer == "native" else ViserPlayViewer)(
            env, policy
        ).run()
        return 0
    finally:
        env.close()


if __name__ == "__main__":
    raise SystemExit(main())
