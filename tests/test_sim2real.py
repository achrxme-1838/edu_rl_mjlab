"""Check EDU -> existing C++ deployment, without creating DDS channels."""

import hashlib
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")

import numpy as np
import mujoco
import onnx
import torch
import yaml

from src.tasks.velocity.config.a2.env_cfgs import (
    DEFAULT_JOINT_POS,
    POLICY_JOINT_NAMES,
    make_env_cfg,
)
from src.tasks.velocity.config.a2.policy import DEFAULT_CHECKPOINT, EduA2Policy

ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = ROOT / "networks/01_EDU_A2_STAIR"
PROBE = ROOT / "deploy/robots/a2/build/a2_policy_probe"
DEPLOY_CFG = ROOT / "deploy/robots/a2/config/deploy.yaml"
CFG = yaml.safe_load(DEPLOY_CFG.read_text())
SDK_JOINTS = [
    element.attrib["joint"]
    for element in ET.parse(ROOT / "src/assets/robots/unitree_a2/xmls/scene_a2.xml")
    .getroot()
    .find("actuator")
]


class DeploymentConfigTest(unittest.TestCase):
    def test_joint_mapping_preserves_existing_sdk_order(self):
        mj_joints = [
            f"{leg}_{joint}_joint"
            for leg in ("FL", "FR", "RL", "RR")
            for joint in ("hip", "thigh", "calf")
        ]
        self.assertEqual(
            [SDK_JOINTS.index(name) for name in mj_joints],
            [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8],
        )
        self.assertEqual(
            CFG["joint_ids_map"],
            [SDK_JOINTS.index(name) for name in POLICY_JOINT_NAMES],
        )

    def test_config_matches_python_policy_contract(self):
        cfg = make_env_cfg()
        self.assertEqual(
            list(CFG["observations"]), list(cfg.observations["actor"].terms)
        )
        for name, term in cfg.observations["actor"].terms.items():
            saved = CFG["observations"][name]
            self.assertEqual(
                saved["history_length"], cfg.observations["actor"].history_length
            )
            self.assertEqual(saved["clip"], list(term.clip))
            scale = term.scale if term.scale is not None else 1.0
            self.assertTrue(all(value == scale for value in saved["scale"]))
        np.testing.assert_array_equal(CFG["default_joint_pos"], DEFAULT_JOINT_POS)
        action = CFG["actions"]["JointPositionAction"]
        np.testing.assert_array_equal(action["offset"], DEFAULT_JOINT_POS)
        self.assertEqual(action["scale"], [0.25] * 12)
        self.assertEqual(action["clip"], [[-100, 100]] * 12)
        self.assertEqual(CFG["stiffness"], [25] * 12)
        self.assertEqual(CFG["damping"], [0.5] * 12)
        self.assertEqual(CFG["step_dt"], cfg.decimation * cfg.sim.mujoco.timestep)
        # These are the command limits recorded with the EDU checkpoint.
        self.assertEqual(
            CFG["commands"]["base_velocity"]["ranges"],
            {
                "lin_vel_x": [-0.6, 0.6],
                "lin_vel_y": [-0.4, 0.4],
                "ang_vel_z": [-1.0, 1.0],
                "heading": None,
            },
        )

    def test_fixstand_final_pose_and_export_provenance(self):
        fsm = yaml.safe_load(
            (ROOT / "deploy/robots/a2/config/config.yaml").read_text()
        )["FSM"]
        expected_sdk = np.empty(12)
        expected_sdk[CFG["joint_ids_map"]] = DEFAULT_JOINT_POS
        np.testing.assert_array_equal(fsm["FixStand"]["qs"][-1], expected_sdk)
        self.assertEqual(fsm["FixStand"]["kp"], [200] * 12)
        self.assertEqual(fsm["FixStand"]["kd"], [5] * 12)
        self.assertNotIn("policy_dir", fsm["Velocity"])
        self.assertEqual(CFG["policy_dir"], "networks/01_EDU_A2_STAIR/")
        model = onnx.load(str(POLICY_DIR / "policy.onnx"))
        metadata = {item.key: item.value for item in model.metadata_props}
        self.assertEqual(metadata["checkpoint_file"], DEFAULT_CHECKPOINT.name)
        self.assertEqual(metadata["checkpoint_sha256"], hashlib.sha256(DEFAULT_CHECKPOINT.read_bytes()).hexdigest())
        self.assertEqual(metadata["task"], "Edu-A2-Velocity-v0")
        self.assertEqual(metadata["actor_obs_normalization"], "false")

    def test_lowstand_lowers_body_five_cm_within_joint_limits(self):
        fsm = yaml.safe_load(
            (ROOT / "deploy/robots/a2/config/config.yaml").read_text()
        )["FSM"]
        model = mujoco.MjModel.from_xml_path(
            str(ROOT / "src/assets/robots/unitree_a2/xmls/a2.xml")
        )
        data = mujoco.MjData(model)
        feet = []
        for state in ("FixStand", "LowStand"):
            for name, value in zip(SDK_JOINTS, fsm[state]["qs"][-1], strict=True):
                joint = model.joint(name)
                self.assertGreaterEqual(value, joint.range[0])
                self.assertLessEqual(value, joint.range[1])
                data.qpos[joint.qposadr[0]] = value
            mujoco.mj_forward(model, data)
            feet.append(np.array([data.site(leg).xpos.copy() for leg in ("FR", "FL", "RR", "RL")]))
        # Fixed base: feet rise by 5 cm without x/y motion. With feet planted,
        # this corresponds to lowering the body by 5 cm in model kinematics.
        np.testing.assert_allclose(feet[1] - feet[0], [[0, 0, 0.05]] * 4, atol=1e-6)


@unittest.skipUnless(
    PROBE.is_file(), "Build a2_policy_probe with -DBUILD_OFFLINE_TESTS=ON first"
)
class CppDeploymentTest(unittest.TestCase):
    def test_sensor_history_onnx_and_motor_targets_match_python(self):
        rng = np.random.default_rng(812)
        ids = [SDK_JOINTS.index(name) for name in POLICY_JOINT_NAMES]
        pose = np.array(DEFAULT_JOINT_POS, dtype=np.float32)
        frames = []
        for index in range(16):
            q_sdk = np.empty(12, dtype=np.float32)
            q_sdk[ids] = pose + rng.normal(0, 0.15, 12)
            quaternion = rng.normal(size=4)
            quaternion /= np.linalg.norm(quaternion)
            frame = {
                "q_sdk": q_sdk.tolist(),
                "dq_sdk": rng.normal(0, 4, 12).tolist(),
                "quat_wxyz": quaternion.tolist(),
                "gyro": rng.normal(0, 0.7, 3).tolist(),
                "command": rng.uniform(-1, 1, 3).tolist(),
                "reset": index in (0, 9),
            }
            if index == 4:
                frame["gyro"] = [150, -150, 0.2]
                frame["dq_sdk"][2] = 200
                frame["q_sdk"][8] = -150
                frame["override_action"] = np.linspace(-500, 500, 12).tolist()
            frames.append(frame)

        with tempfile.TemporaryDirectory() as directory:
            inputs, outputs = (
                Path(directory) / "frames.yaml",
                Path(directory) / "output.yaml",
            )
            inputs.write_text(yaml.safe_dump(frames))
            subprocess.run(
                [
                    str(PROBE),
                    str(DEPLOY_CFG),
                    str(POLICY_DIR / "policy.onnx"),
                    str(inputs),
                    str(outputs),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=30,
            )
            result = yaml.safe_load(outputs.read_text())
        self.assertTrue(result["rejected_short_observation"])
        self.assertTrue(result["rejected_nonfinite_observation"])
        self.assertEqual(result["bad_orientation"], [False, False, True, True])
        self.assertTrue(result["nonfinite_orientation_rejected"])

        actor = EduA2Policy()
        previous_action = np.zeros(12, dtype=np.float32)
        histories = []
        filtered_command = np.zeros(3, dtype=np.float32)
        max_obs_error = max_action_error = max_target_error = 0.0
        for frame, actual in zip(frames, result["samples"], strict=True):
            # SDK Axis defaults: deadzone 0.01, smoothing 0.03, applied once per packet.
            raw_command = np.array(frame["command"], dtype=np.float32)
            deadzone_command = np.where(np.abs(raw_command) < np.float32(0.01), 0.0, raw_command)
            smooth = float(np.float32(0.03))
            filtered_command = (
                filtered_command.astype(np.float64) * (1.0 - smooth)
                + deadzone_command.astype(np.float64) * smooth
            ).astype(np.float32)
            if frame["reset"]:
                previous_action.fill(0)
            if "override_action" in frame:
                previous_action = np.array(frame["override_action"], dtype=np.float32)
                np.testing.assert_allclose(
                    actual["targets_before_step"],
                    np.clip(pose + 0.25 * previous_action, -100, 100),
                    rtol=1e-6,
                    atol=1e-6,
                )
            w, x, y, z = np.array(frame["quat_wxyz"], dtype=np.float32)
            gravity = np.array(
                [2 * (w * y - x * z), -2 * (y * z + w * x), 2 * (x * x + y * y) - 1],
                dtype=np.float32,
            )
            terms = [
                np.clip(np.array(frame["gyro"], dtype=np.float32), -100, 100)
                * np.float32(0.2),
                gravity,
                (np.clip(filtered_command, -1, 1) * np.array([0.6, 0.4, 1], dtype=np.float32)
                 * np.array([0.5, 0.5, 0.5], dtype=np.float32)),
                np.clip(
                    np.array(frame["q_sdk"], dtype=np.float32)[ids] - pose, -100, 100
                ),
                np.clip(np.array(frame["dq_sdk"], dtype=np.float32)[ids], -100, 100)
                * np.float32(0.05),
                np.clip(previous_action, -100, 100),
            ]
            if frame["reset"]:
                histories = [
                    deque([term.copy() for _ in range(5)], maxlen=5) for term in terms
                ]
                np.testing.assert_allclose(
                    actual["targets_before_step"], pose, atol=1e-7
                )
            for history, term in zip(histories, terms, strict=True):
                history.append(term.copy())
            obs = np.concatenate([np.concatenate(list(h)) for h in histories])
            np.testing.assert_allclose(actual["obs"], obs, rtol=2e-5, atol=2e-5)
            action = actor(torch.tensor(obs).reshape(1, 225)).numpy()[0]
            np.testing.assert_allclose(
                actual["raw_action"], action, rtol=2e-5, atol=2e-4
            )
            targets = np.clip(pose + np.float32(0.25) * action, -100, 100)
            np.testing.assert_allclose(
                actual["policy_targets"], targets, rtol=2e-5, atol=5e-5
            )
            sdk_targets = np.empty(12, dtype=np.float32)
            sdk_targets[ids] = targets
            np.testing.assert_allclose(
                actual["sdk_targets"], sdk_targets, rtol=2e-5, atol=5e-5
            )
            max_obs_error = max(
                max_obs_error, float(np.max(np.abs(np.array(actual["obs"]) - obs)))
            )
            max_action_error = max(
                max_action_error,
                float(np.max(np.abs(np.array(actual["raw_action"]) - action))),
            )
            max_target_error = max(
                max_target_error,
                float(np.max(np.abs(np.array(actual["sdk_targets"]) - sdk_targets))),
            )
            # Compare both backends on the same input; history must retain the
            # raw action actually produced by C++, including ONNX rounding.
            previous_action = np.array(actual["raw_action"], dtype=np.float32)
        print(
            f"C++/Python maximum errors: obs={max_obs_error:.3g}, action={max_action_error:.3g}, motor_target={max_target_error:.3g}"
        )


if __name__ == "__main__":
    unittest.main()
