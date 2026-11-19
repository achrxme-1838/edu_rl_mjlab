"""Check the shared source and its non-default contract without robot connections."""

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from mjlab_wrapper import mdp

from deploy.configs.a2 import DEFAULT_CONFIG, REPO_ROOT, load_config, validate
from deploy.configs.a2.runtime import to_runtime


class SharedConfigTest(unittest.TestCase):
    def test_config_uses_local_mjlab_wrappers(self):
        cfg = load_config()
        for term, function in zip(cfg.observations.terms.values(), (
            mdp.base_ang_vel, mdp.projected_gravity, mdp.generated_commands,
            mdp.joint_pos_rel, mdp.joint_vel_rel, mdp.last_action,
        )):
            self.assertIs(term.func, function)
            self.assertEqual(function.__module__, "mjlab_wrapper.mdp.observations")

    def test_cpp_transport_uses_mjlab_python_from_any_directory(self):
        result = subprocess.run(
            [sys.executable, "-I", "-B", str(REPO_ROOT / "deploy/configs/a2/runtime.py")],
            cwd="/tmp", env={**os.environ, "PATH": "/nonexistent", "PYTHONPATH": "/invalid"},
            capture_output=True, text=True, check=True, timeout=30,
        )
        runtime = json.loads(result.stdout)
        self.assertEqual(list(runtime["deploy"]["observations"]), [
            "base_ang_vel", "projected_gravity", "velocity_commands",
            "joint_pos_rel", "joint_vel_rel", "last_action",
        ])
        self.assertEqual(runtime["deploy"]["observation_dim"], 225)
        self.assertEqual(runtime["deploy"]["action_dim"], 12)
        self.assertEqual(runtime["deploy"]["observations"]["velocity_commands"]["func"], "velocity_commands")
        self.assertEqual(runtime["deploy"]["observations"]["velocity_commands"]["params"], {"command_name": "base_velocity"})
        self.assertEqual(runtime["deploy"]["joint_ids_map"], [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])

    def test_invalid_contract_is_rejected(self):
        cfg = load_config()
        invalid = (
            replace(cfg, real=replace(cfg.real, joint_names=("FL_hip_joint",) * 12)),
            replace(cfg, mj=replace(cfg.mj, kp=(-1.0,) * 12)),
            replace(cfg, mj=replace(cfg.mj, timestep=float("nan"))),
            replace(cfg, observations=replace(cfg.observations, history_length=0)),
            replace(cfg, observations=replace(cfg.observations,
                base_ang_vel=replace(cfg.observations.base_ang_vel, size=12))),
            replace(cfg, observations=replace(cfg.observations,
                base_ang_vel=replace(cfg.observations.base_ang_vel, func="base_ang_vel"))),
            replace(cfg, observations=replace(cfg.observations,
                base_ang_vel=replace(cfg.observations.base_ang_vel, func=lambda env: None))),
            replace(cfg, observations=replace(cfg.observations,
                base_ang_vel=replace(cfg.observations.base_ang_vel, func=mdp.joint_pos_rel))),
            replace(cfg, observations=replace(cfg.observations,
                velocity_commands=replace(cfg.observations.velocity_commands, params={"command_name": "missing"}))),
            replace(cfg, network=replace(cfg.network, domain_id=-1)),
        )
        for modified in invalid:
            with self.subTest(modified=modified), self.assertRaises(ValueError):
                validate(modified)

    def test_actor_uses_dimensions_from_selected_config(self):
        import torch
        from torch import nn
        from src.tasks.velocity.config.a2.policy import EduA2Policy

        for history, dims in ((1, (45, 24, 12)), (3, (135, 32, 16, 12))):
            with self.subTest(history=history), tempfile.TemporaryDirectory() as directory:
                checkpoint = Path(directory) / "custom.pt"
                config_path = Path(directory) / "config.py"
                # Independent actors with different input widths, hidden widths
                # and depths exercise the actual checkpoint-loading path.
                layers = []
                for index, (size_in, size_out) in enumerate(zip(dims, dims[1:])):
                    layers.append(nn.Linear(size_in, size_out))
                    if index < len(dims) - 2:
                        layers.append(nn.ELU())
                reference = nn.Sequential(*layers).eval()
                torch.save({"model_state_dict": {
                    f"actor.{key}": value for key, value in reference.state_dict().items()
                }}, checkpoint)
                config_path.write_text(
                    "from dataclasses import replace\n"
                    "from deploy.configs.a2.config import A2DeployCfg as BaseCfg\n"
                    "def A2DeployCfg():\n"
                    "    base = BaseCfg()\n"
                    "    return replace(base,\n"
                    f"        observations=replace(base.observations, history_length={history}),\n"
                    f"        policy=replace(base.policy, checkpoint={str(checkpoint)!r}, "
                    f"hidden_dims={dims[1:-1]!r}))\n"
                )
                cfg = load_config(config_path)
                actor = EduA2Policy(deploy_cfg=cfg)
                self.assertEqual(cfg.actor_dims, dims)
                obs = torch.linspace(-1, 1, 2 * dims[0]).reshape(2, dims[0])
                torch.testing.assert_close(actor(obs), reference(obs), rtol=0, atol=0)
                # Config changes must not silently reshape incompatible weights.
                for incompatible in (
                    replace(cfg, observations=replace(cfg.observations, history_length=history + 1)),
                    replace(cfg, policy=replace(cfg.policy, hidden_dims=(dims[1] + 1,))),
                ):
                    with self.assertRaisesRegex(RuntimeError, "size mismatch"):
                        EduA2Policy(deploy_cfg=incompatible)

    def test_field_aliases_preserve_mjlab_observations_and_history(self):
        import torch
        from mjlab.envs import ManagerBasedRlEnv, mdp as upstream_mdp
        from src.tasks.velocity.config.a2.env_cfgs import make_env_cfg

        cfg = load_config()
        source = DEFAULT_CONFIG.read_text()
        aliases = ("gyro", "gravity", "command", "positions", "velocities", "previous_action")
        for name, alias in zip(cfg.observations.terms, aliases):
            source = source.replace(f"    {name}: ObservationTermCfg", f"    {alias}: ObservationTermCfg")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "alias_config.py"
            path.write_text(source)
            alias_cfg = load_config(path)
        self.assertEqual(tuple(alias_cfg.observations.terms), aliases)
        self.assertIs(alias_cfg.observations.gyro.func, mdp.base_ang_vel)
        self.assertIs(alias_cfg.observations.command.func, mdp.generated_commands)

        environments = []
        try:
            # A non-native policy order also exercises joint-parameter injection
            # by func; selecting joints by the field label would silently break it.
            for current in (cfg, alias_cfg):
                current = replace(current, joints=replace(
                    current.joints, names=current.joints.names[::-1],
                    default_pos=current.joints.default_pos[::-1],
                ))
                env_cfg = make_env_cfg(deploy_cfg=current)
                if not environments:
                    # Compare the wrapped, renamed terms against mjlab itself.
                    for term in env_cfg.observations["actor"].terms.values():
                        term.func = getattr(upstream_mdp, term.func.__name__)
                environments.append(ManagerBasedRlEnv(env_cfg, device="cpu"))
            left, right = environments
            torch.testing.assert_close(left.reset()[0]["actor"], right.reset()[0]["actor"], rtol=0, atol=0)
            for step in range(7):
                action = (torch.arange(12).reshape(1, 12).float() - 5) * 0.03 * (step + 1)
                expected = left.step(action)[0]["actor"]
                actual = right.step(action)[0]["actor"]
                torch.testing.assert_close(actual, expected, rtol=0, atol=0)
        finally:
            for env in environments:
                env.close()

    def test_custom_observations_actions_and_per_joint_gains_reach_runtime(self):
        import torch
        from mjlab.envs import ManagerBasedRlEnv
        from src.tasks.velocity.config.a2.env_cfgs import make_env_cfg

        cfg = load_config()
        cfg = replace(
            cfg,
            joints=replace(cfg.joints, names=cfg.joints.names[::-1], default_pos=cfg.joints.default_pos[::-1]),
            observations=replace(cfg.observations, history_length=3,
                velocity_commands=replace(cfg.observations.velocity_commands, scale=2.0, clip=(-0.1, 0.1))),
            actions=replace(cfg.actions, scale=0.5, clip=(-0.7, 0.7)),
            mj=replace(cfg.mj, kp=tuple(20.0 + i for i in range(12))),
            real=replace(cfg.real, kp=tuple(40.0 + i for i in range(12))),
        )
        runtime = to_runtime(cfg)["deploy"]
        self.assertEqual(runtime["observation_dim"], 135)
        self.assertEqual(runtime["stiffness"], tuple(40.0 + i for i in range(12)))
        self.assertEqual(runtime["observations"]["velocity_commands"]["scale"], [2.0] * 3)
        self.assertEqual(runtime["joint_ids_map"], (8, 11, 2, 5, 7, 10, 1, 4, 6, 9, 0, 3))
        env = ManagerBasedRlEnv(make_env_cfg(deploy_cfg=cfg), device="cpu")
        try:
            obs, _ = env.reset()
            self.assertEqual(tuple(obs["actor"].shape), (1, 135))
            torch.testing.assert_close(obs["actor"][:, 18:27], torch.tensor([[0.2, 0, 0] * 3]))
            robot = env.scene["robot"]
            for motor in robot.actuators:
                for index, name in enumerate(motor.target_names):
                    expected_kp = 20.0 + cfg.mj.joint_names.index(name)
                    self.assertEqual(float(motor.stiffness[0, index]), expected_kp)
            values = torch.arange(12).reshape(1, 12).float() - 5
            env.action_manager.process_action(values)
            env.action_manager.apply_action()
            ids = [robot.joint_names.index(name) for name in cfg.joints.names]
            expected = (torch.tensor(cfg.joints.default_pos) + 0.5 * values).clamp(-0.7, 0.7)
            torch.testing.assert_close(robot.data.joint_pos_target[:, ids], expected)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
