"""Run with the mjlab interpreter: python -m unittest discover -s tests -v."""

import hashlib
import json
import os
import unittest
from collections import deque

os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")

import numpy as np
import torch
import torch.nn.functional as F
import yaml
from mjlab.actuator import ActuatorCmd
from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import RslRlVecEnvWrapper

from src.assets.robots.unitree_a2.a2_constants import get_a2_robot_cfg
from src.custom.a2_edu.env_cfg import (
    DEFAULT_JOINT_POS,
    POLICY_JOINT_NAMES,
    StairCourse,
    make_env_cfg,
)
from src.custom.a2_edu.policy import DEFAULT_CHECKPOINT, EduA2Policy


class SnapshotLoader(yaml.SafeLoader):
    """Handle just the two nonstandard tags emitted by the saved training config."""


SnapshotLoader.add_constructor(
    "tag:yaml.org,2002:python/tuple",
    lambda loader, node: tuple(loader.construct_sequence(node)),
)
SnapshotLoader.add_constructor(
    "tag:yaml.org,2002:python/object/apply:builtins.slice",
    lambda loader, node: slice(*loader.construct_sequence(node)),
)


class PolicyContractTest(unittest.TestCase):
    def test_checkpoint_and_training_contract(self):
        folder = DEFAULT_CHECKPOINT.parent
        provenance = json.loads((folder / "provenance.json").read_text())
        self.assertEqual(
            hashlib.sha256(DEFAULT_CHECKPOINT.read_bytes()).hexdigest(),
            provenance["checkpoint_sha256"],
        )
        source = json.loads((folder / "source_model.json").read_text())
        self.assertEqual(tuple(source["joint_names"]), POLICY_JOINT_NAMES)
        np.testing.assert_allclose(source["default_joint_pos"], DEFAULT_JOINT_POS)
        agent = yaml.safe_load((folder / "params/agent.yaml").read_text())
        self.assertEqual(agent["policy"]["actor_hidden_dims"], [512, 256, 128])
        self.assertEqual(agent["policy"]["activation"], "elu")
        self.assertFalse(agent["policy"]["actor_obs_normalization"])
        self.assertIsNone(agent["clip_actions"])
        saved = yaml.load(
            (folder / "params/env.yaml").read_text(), Loader=SnapshotLoader
        )
        cfg = make_env_cfg()
        self.assertEqual(saved["decimation"], cfg.decimation)
        self.assertEqual(saved["sim"]["dt"], cfg.sim.mujoco.timestep)
        source_obs = saved["observations"]["policy"]
        target_obs = cfg.observations["actor"]
        self.assertEqual(source_obs["history_length"], target_obs.history_length)
        self.assertEqual(
            [k for k, v in source_obs.items() if isinstance(v, dict) and "func" in v],
            list(target_obs.terms),
        )
        for name, term in target_obs.terms.items():
            self.assertEqual(source_obs[name]["clip"], term.clip)
            self.assertEqual(source_obs[name]["scale"], term.scale)
        motor = cfg.scene.entities["robot"].articulation.actuators[0]
        source_motor = saved["scene"]["robot"]["actuators"]["a2_legs"]
        for name in (
            "stiffness",
            "damping",
            "effort_limit",
            "saturation_effort",
            "velocity_limit",
        ):
            self.assertEqual(getattr(motor, name), source_motor[name])

    def test_actor_matches_checkpoint_linear_algebra(self):
        policy = EduA2Policy()
        state = torch.load(DEFAULT_CHECKPOINT, map_location="cpu", weights_only=True)[
            "model_state_dict"
        ]
        generator = torch.Generator().manual_seed(31)
        obs = torch.randn(32, 225, generator=generator)
        reference = obs
        for index in (0, 2, 4, 6):
            reference = F.linear(
                reference, state[f"actor.{index}.weight"], state[f"actor.{index}.bias"]
            )
            if index != 6:
                reference = F.elu(reference)
        torch.testing.assert_close(policy({"actor": obs}), reference, rtol=0, atol=0)
        with self.assertRaises(ValueError):
            policy(torch.zeros(1, 45))

    def test_stock_mass_inertia_and_collision_geometry_unchanged(self):
        stock = Entity(get_a2_robot_cfg()).spec.compile()
        target = Entity(make_env_cfg().scene.entities["robot"]).spec.compile()
        for name in (
            "body_mass",
            "body_inertia",
            "body_ipos",
            "body_iquat",
            "body_pos",
            "body_quat",
            "geom_type",
            "geom_size",
            "geom_pos",
            "geom_quat",
            "geom_contype",
            "geom_conaffinity",
            "geom_condim",
            "geom_friction",
            "geom_solimp",
            "geom_solref",
            "jnt_range",
        ):
            np.testing.assert_array_equal(
                getattr(stock, name), getattr(target, name), err_msg=name
            )
        self.assertAlmostEqual(target.body("base_link").mass[0], 19.651)

    def test_stair_geometry_heights(self):
        course = StairCourse()
        cfg = make_env_cfg(terrain="stairs", course=course)
        from mjlab.scene import Scene

        model = Scene(cfg.scene, device="cpu").compile()
        for i in range(course.step_count):
            geom = model.geom(f"edu_stair_{i}")
            self.assertAlmostEqual(
                float(geom.pos[2] + geom.size[2]), (i + 1) * course.step_height
            )
        self.assertEqual(course.height_at(0, 0), 0)
        self.assertEqual(course.height_at(course.landing_start + 0.5, 0), 0.5)
        self.assertEqual(course.height_at(course.end_x + 0.1, 0), 0)


class RuntimeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.env = RslRlVecEnvWrapper(
            ManagerBasedRlEnv(make_env_cfg(), device="cpu"), clip_actions=None
        )
        cls.raw = cls.env.unwrapped
        cls.robot = cls.raw.scene["robot"]
        cls.ids = [cls.robot.joint_names.index(name) for name in POLICY_JOINT_NAMES]

    @classmethod
    def tearDownClass(cls):
        cls.env.close()

    def current_terms(self):
        data = self.robot.data
        return [
            data.root_link_ang_vel_b.clamp(-100, 100) * 0.2,
            data.projected_gravity_b.clamp(-100, 100),
            self.raw.command_manager.get_command("base_velocity").clamp(-100, 100),
            (data.joint_pos[:, self.ids] - torch.tensor(DEFAULT_JOINT_POS)).clamp(
                -100, 100
            ),
            data.joint_vel[:, self.ids].clamp(-100, 100) * 0.05,
            self.raw.action_manager.action.clamp(-100, 100),
        ]

    def test_action_joint_mapping_and_target_clipping(self):
        self.env.reset()
        action = self.raw.action_manager.get_term("joint_pos")
        self.assertEqual(action.target_names, list(POLICY_JOINT_NAMES))
        self.assertEqual(self.ids, [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11])
        values = torch.tensor(
            [[1000.0, -1000.0, 0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8, 0.9, -1.0]]
        )
        self.raw.action_manager.process_action(values)
        self.raw.action_manager.apply_action()
        expected = (torch.tensor(DEFAULT_JOINT_POS) + 0.25 * values).clamp(-100, 100)
        torch.testing.assert_close(
            self.robot.data.joint_pos_target[:, self.ids], expected
        )
        torch.testing.assert_close(action.raw_action, values)
        self.env.reset()

    def test_five_frame_term_history_and_reset(self):
        obs, _ = self.env.reset()
        history = [
            deque([term.clone() for _ in range(5)], maxlen=5)
            for term in self.current_terms()
        ]
        torch.testing.assert_close(
            obs["actor"],
            torch.cat([torch.cat(list(h), dim=-1) for h in history], dim=-1),
        )
        for step in range(8):
            values = (torch.arange(12).reshape(1, 12).float() - 5) * 0.03 * (step + 1)
            obs, _, dones, _ = self.env.step(values)
            self.assertFalse(dones.any())
            for h, term in zip(history, self.current_terms()):
                h.append(term.clone())
            reference = torch.cat([torch.cat(list(h), dim=-1) for h in history], dim=-1)
            torch.testing.assert_close(obs["actor"], reference)
            # Viewer reads must not advance the observation history.
            for _ in range(2):
                torch.testing.assert_close(
                    self.env.get_observations()["actor"], reference
                )
        obs, _ = self.env.reset()
        expected = torch.cat(
            [term.repeat(1, 5) for term in self.current_terms()], dim=-1
        )
        torch.testing.assert_close(obs["actor"], expected)
        self.assertEqual(torch.count_nonzero(obs["actor"][:, -60:]), 0)

    def test_motor_torque_speed_curve(self):
        motor = self.robot.actuators[0]
        velocities = torch.tensor(
            [[-60.0, -44.0, -22.0, -11.0, -1.0, 0.0, 1.0, 11.0, 22.0, 44.0, 60.0, 5.0]]
        )
        zeros = torch.zeros_like(velocities)
        for target in (-20.0, -0.1, 0.1, 20.0):
            cmd = ActuatorCmd(
                position_target=torch.full_like(zeros, target),
                velocity_target=zeros,
                effort_target=zeros,
                pos=zeros,
                vel=velocities,
            )
            limited_velocity = velocities.clamp(-44.0, 44.0)
            lower = (120 * (-1 - limited_velocity / 22)).clamp(min=-120)
            upper = (120 * (1 - limited_velocity / 22)).clamp(max=120)
            reference = torch.clamp(
                25 * target - 0.5 * velocities, min=lower, max=upper
            )
            torch.testing.assert_close(motor.compute(cmd), reference)


if __name__ == "__main__":
    unittest.main()
