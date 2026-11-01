"""DEX A2 policy interface on the original mjlab A2 model.

Source: 01_STAIR_SUCCESS/params/{env.yaml,agent.yaml,a2_base_rl.py}.
The model's masses, inertias, geometry and contact settings remain mjlab's.
Only initial pose, actuator control and the policy interface follow training.
"""

from copy import deepcopy
from dataclasses import dataclass
from functools import partial
from typing import Literal

import mujoco
import torch

from mjlab.actuator import DcMotorActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg, mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.envs.mdp.actions.actions import JointPositionAction
from mjlab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand
from mjlab.terrains import TerrainEntityCfg
from mjlab.viewer import ViewerConfig
from src.assets.robots.unitree_a2.a2_constants import get_a2_robot_cfg


# Confirmed by loading the original USD in Isaac/PhysX. Do NOT use MJCF order.
POLICY_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint"
    for joint in ("hip", "thigh", "calf")
    for leg in ("FL", "FR", "RL", "RR")
)


@dataclass(frozen=True)
class A2DexConfig:
    """Edit this configuration, leaving the stock A2 task/configuration intact."""

    terrain: Literal["flat", "stairs"] = "flat"
    command: tuple[float, float, float] = (0.4, 0.0, 0.0)
    physics_dt: float = 0.005
    decimation: int = 4
    history_length: int = 5
    action_scale: float = 0.25
    target_clip: float = 100.0
    kp: float = 25.0
    kd: float = 0.5
    effort_limit: float = 120.0
    saturation_effort: float = 120.0
    velocity_limit: float = 22.0
    motor_armature: float = 0.0
    motor_frictionloss: float = 0.0
    initial_height: float = 0.45
    hip_left: float = 0.1
    hip_right: float = -0.1
    thigh_front: float = 0.8
    thigh_rear: float = 1.0
    calf: float = -1.5
    ang_vel_scale: float = 0.2
    joint_vel_scale: float = 0.05
    observation_clip: float = 100.0
    # A straight ascending flight, landing, and descending flight.
    # Deterministic test course, not a recreation of Isaac's random terrain grid.
    stair_height: float = 0.10
    stair_width: float = 0.30
    stair_count: int = 5
    stair_start: float = 1.5
    stair_half_width: float = 1.5
    landing_length: float = 2.0


class PolicyJointPositionAction(JointPositionAction):
    """Keep raw actions in policy order, and resolve physical joints by name.

    mjlab 1.2's default JOINT action resolves in natural joint order even when
    preserve_order=True. Override that resolution only for this custom action.
    """

    def _find_targets(self, cfg):
        return self._entity.find_joints(cfg.actuator_names, preserve_order=True)

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        # Isaac clips the resulting POSITION TARGET, not raw network outputs.
        self._processed_actions.clamp_(-self.cfg.target_clip, self.cfg.target_clip)


@dataclass(kw_only=True)
class PolicyJointPositionActionCfg(JointPositionActionCfg):
    target_clip: float = 100.0

    def build(self, env):
        return PolicyJointPositionAction(self, env)


class FixedVelocityCommand(UniformVelocityCommand):
    """Fixed reset command with non-degenerate ranges for the Viser joystick."""

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        self.vel_command_b[env_ids] = torch.tensor(self.cfg.fixed_command, device=self.device)
        self.is_standing_env[env_ids] = False
        self.is_heading_env[env_ids] = False


@dataclass(kw_only=True)
class FixedVelocityCommandCfg(UniformVelocityCommandCfg):
    fixed_command: tuple[float, float, float]

    def build(self, env):
        return FixedVelocityCommand(self, env)


def add_stairs(spec: mujoco.MjSpec, cfg: A2DexConfig) -> None:
    """Add solid stair boxes to the scene, keeping the robot XML unchanged."""
    heights = [cfg.stair_height * i for i in range(1, cfg.stair_count)]
    widths = [cfg.stair_width] * len(heights)
    heights += [cfg.stair_height * cfg.stair_count]
    widths += [cfg.landing_length + 2 * cfg.stair_width]
    heights += [cfg.stair_height * i for i in range(cfg.stair_count - 1, 0, -1)]
    widths += [cfg.stair_width] * (cfg.stair_count - 1)
    x = cfg.stair_start
    for index, (height, width) in enumerate(zip(heights, widths, strict=True)):
        spec.worldbody.add_geom(
            name=f"dex_stair_{index}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[width / 2, cfg.stair_half_width, height / 2],
            pos=[x + width / 2, 0.0, height / 2],
            rgba=[0.45, 0.49, 0.55, 1.0], contype=0, conaffinity=1,
        )
        x += width


def make_env_cfg(custom: A2DexConfig) -> ManagerBasedRlEnvCfg:
    if custom.terrain not in ("flat", "stairs"):
        raise ValueError(f"Unknown terrain: {custom.terrain}")
    if min(custom.stair_height, custom.stair_width, custom.stair_count,
           custom.stair_half_width, custom.landing_length) <= 0:
        raise ValueError("Stair dimensions and count must be positive")

    robot = deepcopy(get_a2_robot_cfg())
    robot.init_state = EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, custom.initial_height),
        joint_pos={
            ".*L_hip_joint": custom.hip_left,
            ".*R_hip_joint": custom.hip_right,
            "F[LR]_thigh_joint": custom.thigh_front,
            "R[LR]_thigh_joint": custom.thigh_rear,
            ".*_calf_joint": custom.calf,
        },
        joint_vel={".*": 0.0},
    )
    robot.articulation = EntityArticulationInfoCfg(
        actuators=(DcMotorActuatorCfg(
            target_names_expr=(".*_joint",), stiffness=custom.kp, damping=custom.kd,
            effort_limit=custom.effort_limit, saturation_effort=custom.saturation_effort,
            velocity_limit=custom.velocity_limit, armature=custom.motor_armature,
            frictionloss=custom.motor_frictionloss,
        ),),
        soft_joint_pos_limit_factor=0.9,
    )
    joints = SceneEntityCfg("robot", joint_names=POLICY_JOINT_NAMES, preserve_order=True)
    clip = (-custom.observation_clip, custom.observation_clip)
    terms = {
        "base_ang_vel": ObservationTermCfg(func=mdp.base_ang_vel, scale=custom.ang_vel_scale, clip=clip),
        "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity, clip=clip),
        "velocity_commands": ObservationTermCfg(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}, clip=clip),
        "joint_pos_rel": ObservationTermCfg(func=mdp.joint_pos_rel, params={"asset_cfg": deepcopy(joints)}, clip=clip),
        "joint_vel_rel": ObservationTermCfg(
            func=mdp.joint_vel_rel, params={"asset_cfg": deepcopy(joints)}, scale=custom.joint_vel_scale, clip=clip),
        "last_action": ObservationTermCfg(func=mdp.last_action, clip=clip),
    }
    return ManagerBasedRlEnvCfg(
        seed=42, decimation=custom.decimation, episode_length_s=1e9,
        scene=SceneCfg(
            num_envs=1, terrain=TerrainEntityCfg(terrain_type="plane"),
            entities={"robot": robot},
            spec_fn=partial(add_stairs, cfg=custom) if custom.terrain == "stairs" else None,
        ),
        sim=SimulationCfg(njmax=300, mujoco=MujocoCfg(timestep=custom.physics_dt)),
        observations={"actor": ObservationGroupCfg(
            terms=terms, history_length=custom.history_length, flatten_history_dim=True,
            concatenate_terms=True, enable_corruption=False, nan_policy="error",
        )},
        actions={"joint_pos": PolicyJointPositionActionCfg(
            entity_name="robot", actuator_names=POLICY_JOINT_NAMES, preserve_order=True,
            scale=custom.action_scale, use_default_offset=True, target_clip=custom.target_clip,
        )},
        commands={"base_velocity": FixedVelocityCommandCfg(
            entity_name="robot", resampling_time_range=(1e9, 1e9),
            fixed_command=custom.command,
            heading_command=False, rel_standing_envs=0.0, debug_vis=True,
            ranges=UniformVelocityCommandCfg.Ranges(
                lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.5, 0.5), ang_vel_z=(-1.0, 1.0)),
        )},
        # No noise, pushes, random resets or automatic fall resets during testing.
        viewer=ViewerConfig(origin_type=ViewerConfig.OriginType.ASSET_BODY,
                            entity_name="robot", body_name="base_link",
                            distance=3.0, elevation=-20.0, azimuth=135.0,
                            height=720, width=1280),
    )
