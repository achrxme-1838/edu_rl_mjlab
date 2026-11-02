"""Inference environment matching the EDU A2 actor's input and control contract."""

import math
from copy import deepcopy
from dataclasses import dataclass
from functools import partial

import mujoco
import torch
from mjlab.actuator import DcMotorActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg, mdp
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.velocity.mdp.velocity_command import (
    UniformVelocityCommand,
    UniformVelocityCommandCfg,
)
from mjlab.terrains import TerrainEntityCfg
from mjlab.viewer import ViewerConfig

from src.assets.robots.unitree_a2.a2_constants import get_a2_robot_cfg

POLICY_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for joint in ("hip", "thigh", "calf")
    for leg in ("FL", "FR", "RL", "RR")
)
DEFAULT_JOINT_POS = (0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5)
HISTORY_LENGTH = 5
OBSERVATION_DIM = 225
CONTROL_DT = 0.02


class PolicyJointPositionAction(JointPositionAction):
    def _find_targets(self, cfg):
        # mjlab 1.2's joint action resolver otherwise uses actuator/model order.
        return self._entity.find_joints(cfg.actuator_names, preserve_order=True)

    def process_actions(self, actions):
        super().process_actions(actions)
        # IsaacLab clips position targets AFTER applying scale and default offset.
        self._processed_actions.clamp_(-100.0, 100.0)


@dataclass(kw_only=True)
class PolicyJointPositionActionCfg(JointPositionActionCfg):
    def build(self, env):
        return PolicyJointPositionAction(self, env)


class FixedVelocityCommand(UniformVelocityCommand):
    def _resample_command(self, env_ids):
        self.vel_command_b[env_ids] = torch.tensor(self.cfg.command, device=self.device)
        self.is_standing_env[env_ids] = False
        self.is_heading_env[env_ids] = False


@dataclass(kw_only=True)
class FixedVelocityCommandCfg(UniformVelocityCommandCfg):
    command: tuple[float, float, float] = (0.4, 0.0, 0.0)

    def build(self, env):
        return FixedVelocityCommand(self, env)


@dataclass(frozen=True)
class StairCourse:
    step_height: float = 0.10
    step_width: float = 0.30
    step_count: int = 5
    start_x: float = 1.5
    landing_length: float = 2.0
    width: float = 3.0

    def __post_init__(self):
        values = (self.step_height, self.step_width, self.landing_length, self.width)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError("Stair dimensions must be finite and positive.")
        if self.step_count < 1 or not math.isfinite(self.start_x) or self.start_x < 0.6:
            raise ValueError(
                "Stairs need at least one step and 0.6 m of starting clearance."
            )

    @property
    def landing_start(self):
        return self.start_x + (self.step_count - 1) * self.step_width

    @property
    def end_x(self):
        return (
            self.landing_start
            + self.landing_length
            + (self.step_count - 1) * self.step_width
        )

    def height_at(self, x, y):
        if abs(y) > self.width / 2 or x < self.start_x or x >= self.end_x:
            return 0.0
        level = min(
            self.step_count,
            int((x - self.start_x) / self.step_width) + 1,
            int((self.end_x - x) / self.step_width) + 1,
        )
        return level * self.step_height


def add_stairs(spec, course):
    # Nested boxes form an ascending flight, a landing, and a descending flight.
    for i in range(course.step_count):
        left = course.start_x + i * course.step_width
        right = course.end_x - i * course.step_width
        height = (i + 1) * course.step_height
        spec.worldbody.add_geom(
            name=f"edu_stair_{i}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=((right - left) / 2, course.width / 2, height / 2),
            pos=((left + right) / 2, 0, height / 2),
            rgba=(0.40 + 0.04 * (i % 2), 0.46, 0.52, 1),
            contype=1,
            conaffinity=1,
            friction=(0.8, 0.005, 0.0001),
        )


def make_env_cfg(*, terrain="flat", command=(0.4, 0.0, 0.0), course=None, seed=42):
    if terrain not in ("flat", "stairs"):
        raise ValueError(f"Unknown terrain: {terrain}")
    if len(command) != 3 or not all(math.isfinite(x) for x in command):
        raise ValueError("Command must contain three finite values: vx, vy, wz.")
    robot = deepcopy(get_a2_robot_cfg())
    robot.init_state = EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),
        joint_pos=dict(zip(POLICY_JOINT_NAMES, DEFAULT_JOINT_POS)),
        joint_vel={".*": 0.0},
    )
    robot.articulation = EntityArticulationInfoCfg(
        actuators=(
            DcMotorActuatorCfg(
                target_names_expr=(".*_joint",),
                stiffness=25.0,
                damping=0.5,
                effort_limit=120.0,
                saturation_effort=120.0,
                velocity_limit=22.0,
                armature=0.0,
                frictionloss=0.0,
            ),
        ),
        soft_joint_pos_limit_factor=0.9,
    )
    joints = SceneEntityCfg(
        "robot", joint_names=POLICY_JOINT_NAMES, preserve_order=True
    )
    clip = (-100.0, 100.0)
    terms = {
        "base_ang_vel": ObservationTermCfg(func=mdp.base_ang_vel, scale=0.2, clip=clip),
        "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity, clip=clip),
        "velocity_commands": ObservationTermCfg(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"},
            clip=clip,
        ),
        "joint_pos_rel": ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": deepcopy(joints)},
            clip=clip,
        ),
        "joint_vel_rel": ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": deepcopy(joints)},
            scale=0.05,
            clip=clip,
        ),
        "last_action": ObservationTermCfg(func=mdp.last_action, clip=clip),
    }
    return ManagerBasedRlEnvCfg(
        seed=seed,
        scene=SceneCfg(
            num_envs=1,
            entities={"robot": robot},
            terrain=TerrainEntityCfg(terrain_type="plane"),
            spec_fn=partial(add_stairs, course=course or StairCourse())
            if terrain == "stairs"
            else None,
        ),
        observations={
            "actor": ObservationGroupCfg(
                terms=terms,
                history_length=HISTORY_LENGTH,
                flatten_history_dim=True,
                concatenate_terms=True,
                enable_corruption=False,
                nan_policy="error",
            )
        },
        actions={
            "joint_pos": PolicyJointPositionActionCfg(
                entity_name="robot",
                actuator_names=POLICY_JOINT_NAMES,
                preserve_order=True,
                scale=0.25,
                use_default_offset=True,
            )
        },
        commands={
            "base_velocity": FixedVelocityCommandCfg(
                entity_name="robot",
                command=tuple(command),
                resampling_time_range=(1e9, 1e9),
                heading_command=False,
                rel_standing_envs=0.0,
                debug_vis=True,
                ranges=UniformVelocityCommandCfg.Ranges(
                    lin_vel_x=(-1.0, 1.0),
                    lin_vel_y=(-0.5, 0.5),
                    ang_vel_z=(-1.0, 1.0),
                ),
            )
        },
        sim=SimulationCfg(mujoco=MujocoCfg(timestep=0.005), njmax=300),
        decimation=4,
        episode_length_s=1e9,
        viewer=ViewerConfig(
            origin_type=ViewerConfig.OriginType.ASSET_BODY,
            entity_name="robot",
            body_name="base_link",
            distance=3.5,
            elevation=-20,
            azimuth=135,
            width=1280,
            height=720,
        ),
    )
