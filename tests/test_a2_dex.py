"""Contract checks; set A2_DEX_TEST_DEVICE=cpu for execution without a GPU."""

from collections import deque
from pathlib import Path
import os
import unittest

import numpy as np
import torch
from torch.nn import functional as F

from mjlab.entity import Entity
from mjlab.envs import ManagerBasedRlEnv
from src.assets.robots.unitree_a2.a2_constants import get_a2_robot_cfg
from src.custom.a2_dex.env_cfg import A2DexConfig, POLICY_JOINT_NAMES, make_env_cfg
from src.custom.a2_dex.policy import IsaacActor

CHECKPOINT = (Path(__file__).resolve().parents[2] /
              "DEX_RL_LAB/scripts/logs/rsl_rl/a2_base_rl/01_STAIR_SUCCESS/model_10000.pt")
DEVICE = os.environ.get("A2_DEX_TEST_DEVICE", "cuda:0")


class TestA2Dex(unittest.TestCase):
    def test_actor_matches_saved_weights(self):
        torch.manual_seed(7)
        obs = torch.randn(9, 225)
        state = torch.load(CHECKPOINT, map_location="cpu", weights_only=True)["model_state_dict"]
        expected = obs
        for layer in (0, 2, 4, 6):
            expected = F.linear(expected, state[f"actor.{layer}.weight"], state[f"actor.{layer}.bias"])
            if layer != 6:
                expected = F.elu(expected)
        actual = IsaacActor(CHECKPOINT, "cpu")({"actor": obs})
        torch.testing.assert_close(actual, expected, rtol=0, atol=0)

    def test_original_model_and_stock_config_preserved(self):
        stock_cfg = get_a2_robot_cfg()
        stock = Entity(stock_cfg).spec.compile()
        custom_cfg = make_env_cfg(A2DexConfig())
        custom = Entity(custom_cfg.scene.entities["robot"]).spec.compile()
        for field in ("body_mass", "body_inertia", "body_ipos", "body_iquat",
                      "jnt_axis", "jnt_range", "geom_friction", "geom_contype", "geom_conaffinity"):
            np.testing.assert_array_equal(getattr(stock, field), getattr(custom, field))
        self.assertEqual(stock_cfg.articulation.actuators[0].stiffness, 100.0)
        self.assertEqual(stock_cfg.init_state.joint_pos[".*calf_joint"], -1.8)

    def test_joint_mapping_targets_history_and_reset(self):
        torch.set_num_threads(1)
        env = ManagerBasedRlEnv(make_env_cfg(A2DexConfig()), device=DEVICE)
        try:
            self.assertEqual(str(env.sim.wp_device), DEVICE)
            obs, _ = env.reset()
            robot = env.scene["robot"]
            torch.testing.assert_close(
                env.command_manager.get_command("base_velocity"),
                torch.tensor([[0.4, 0.0, 0.0]], device=DEVICE),
            )
            # Explicit expected mapping, independently of the implementation.
            ids = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]
            self.assertEqual([robot.joint_names[i] for i in ids], list(POLICY_JOINT_NAMES))
            action_term = env.action_manager.get_term("joint_pos")
            self.assertEqual(action_term.target_ids.tolist(), ids)
            expected_pose = torch.tensor(
                [[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5]],
                device=DEVICE)
            torch.testing.assert_close(robot.data.default_joint_pos[:, ids], expected_pose)

            def current_terms():
                data = robot.data
                return [
                    data.root_link_ang_vel_b.clamp(-100, 100) * 0.2,
                    data.projected_gravity_b.clamp(-100, 100),
                    env.command_manager.get_command("base_velocity").clone(),
                    (data.joint_pos[:, ids] - expected_pose).clamp(-100, 100),
                    data.joint_vel[:, ids].clamp(-100, 100) * 0.05,
                    env.action_manager.action.clamp(-100, 100),
                ]

            history = [deque([term.clone() for _ in range(5)], maxlen=5) for term in current_terms()]
            for step in range(8):
                expected_obs = torch.cat([torch.cat(list(term), dim=-1) for term in history], dim=-1)
                torch.testing.assert_close(obs["actor"], expected_obs)
                # Distinct joint targets catch axis permutations and action-history reorderings.
                action = torch.arange(12, device=DEVICE).float()[None] * 0.02 + step * 0.01
                obs, _, _, _, _ = env.step(action)
                torch.testing.assert_close(action_term._processed_actions, expected_pose + action * 0.25)
                for buffer, term in zip(history, current_terms(), strict=True):
                    buffer.append(term.clone())
            # A reset must refill history with the reset frame, without the previous episode.
            obs, _ = env.reset()
            expected_reset = torch.cat([term.repeat(1, 5) for term in current_terms()], dim=-1)
            torch.testing.assert_close(obs["actor"], expected_reset)
            action_term.process_actions(torch.full((1, 12), 1000.0, device=DEVICE))
            torch.testing.assert_close(action_term._processed_actions, torch.full((1, 12), 100.0, device=DEVICE))
            torch.testing.assert_close(action_term.raw_action, torch.full((1, 12), 1000.0, device=DEVICE))
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
