"""A2 inference settings shared by mjlab and the C++ Unitree controller.

Only deployment settings belong here. Paths are relative to the repository root.
Tuple orders are explicit: policy, MuJoCo model, or real SDK motor order.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import Callable

from mjlab_wrapper import mdp


@dataclass(frozen=True)
class PolicyCfg:
    checkpoint: str = "networks/01_EDU_A2_STAIR/model_10001.pt"
    onnx: str = "networks/01_EDU_A2_STAIR/policy.onnx"
    hidden_dims: tuple[int, ...] = (512, 256, 128)  # ELU, no observation normalization.


@dataclass(frozen=True)
class JointsCfg:
    # Actor input/output order: hip FL/FR/RL/RR, then thigh, then calf.
    names: tuple[str, ...] = tuple(
        f"{leg}_{joint}_joint"
        for joint in ("hip", "thigh", "calf")
        for leg in ("FL", "FR", "RL", "RR")
    )
    default_pos: tuple[float, ...] = (
        0.1, -0.1, 0.1, -0.1,  # hip
        0.8, 0.8, 1.0, 1.0,  # thigh
        -1.5, -1.5, -1.5, -1.5,  # calf
    )


@dataclass(frozen=True)
class ObservationTermCfg:
    func: Callable
    size: int
    scale: float | None = None  # None means identity.
    clip: tuple[float, float] = (-100.0, 100.0)
    params: dict = field(default_factory=dict)


@dataclass(frozen=True)
class ObservationsCfg:
    # Declaration order is the actor concatenation order. Clip BEFORE scaling.
    # Field names are labels; func selects the observation computation.
    base_ang_vel: ObservationTermCfg = ObservationTermCfg(func=mdp.base_ang_vel, size=3, scale=0.2)
    projected_gravity: ObservationTermCfg = ObservationTermCfg(func=mdp.projected_gravity, size=3)
    velocity_commands: ObservationTermCfg = ObservationTermCfg(
        func=mdp.generated_commands, size=3, params={"command_name": "base_velocity"}
    )
    joint_pos_rel: ObservationTermCfg = ObservationTermCfg(func=mdp.joint_pos_rel, size=12)
    joint_vel_rel: ObservationTermCfg = ObservationTermCfg(func=mdp.joint_vel_rel, size=12, scale=0.05)
    last_action: ObservationTermCfg = ObservationTermCfg(func=mdp.last_action, size=12)
    # Per term: oldest -> newest, then concatenate terms (45 * 5 = 225).
    # Reset repeats the first observation; previous RAW action resets to zero.
    history_length: int = 5

    @property
    def terms(self):
        return {
            field.name: getattr(self, field.name)
            for field in fields(self)
            if isinstance(getattr(self, field.name), ObservationTermCfg)
        }

    @property
    def dimension(self):
        return sum(term.size for term in self.terms.values()) * self.history_length


@dataclass(frozen=True)
class ActionsCfg:
    # Policy-order target = clip(default_pos + scale * raw_action).
    scale: float = 0.25
    clip: tuple[float, float] = (-100.0, 100.0)


@dataclass(frozen=True)
class SpeedScaleCfg:
    initial: float = 0.5
    step: float = 0.1
    min: float = 0.0
    max: float = 1.0


@dataclass(frozen=True)
class CommandsCfg:
    lin_vel_x: tuple[float, float] = (-0.6, 0.6)
    lin_vel_y: tuple[float, float] = (-0.4, 0.4)
    ang_vel_z: tuple[float, float] = (-1.0, 1.0)
    linear: SpeedScaleCfg = SpeedScaleCfg()
    angular: SpeedScaleCfg = SpeedScaleCfg()


@dataclass(frozen=True)
class NetworkCfg:
    interface: str = ""  # Robot Ethernet interface, e.g. "eth0"; --network overrides it.
    domain_id: int = 0


@dataclass(frozen=True)
class MujocoCfg:
    # Native MJCF joint order; gains follow this order, resolved by joint name.
    joint_names: tuple[str, ...] = tuple(
        f"{leg}_{joint}_joint"
        for leg in ("FL", "FR", "RL", "RR")
        for joint in ("hip", "thigh", "calf")
    )
    kp: tuple[float, ...] = (25.0,) * 12
    kd: tuple[float, ...] = (0.5,) * 12
    timestep: float = 0.005
    decimation: int = 4
    effort_limit: float = 120.0
    saturation_effort: float = 120.0
    velocity_limit: float = 22.0
    armature: float = 0.0
    frictionloss: float = 0.0
    initial_height: float = 0.45
    command: tuple[float, float, float] = (0.4, 0.0, 0.0)


@dataclass(frozen=True)
class RealCfg:
    # SDK motor IDs 0..11. All gains and low_stand_pos below use this order.
    joint_names: tuple[str, ...] = tuple(
        f"{leg}_{joint}_joint"
        for leg in ("FR", "FL", "RR", "RL")
        for joint in ("hip", "thigh", "calf")
    )
    kp: tuple[float, ...] = (25.0,) * 12
    kd: tuple[float, ...] = (0.5,) * 12
    stand_kp: tuple[float, ...] = (200.0,) * 12
    stand_kd: tuple[float, ...] = (5.0,) * 12
    passive_kd: tuple[float, ...] = (10.0,) * 12
    stand_duration: float = 2.0
    initial_state: str = "FixStand"
    # Approximately 5 cm lower, computed from A2 model kinematics.
    low_stand_pos: tuple[float, ...] = (
        -0.1141366, 0.9269101, -1.7403147, 0.1141366, 0.9269101, -1.7403147,
        -0.1146329, 1.1493832, -1.7323059, 0.1146329, 1.1493832, -1.7323059,
    )


@dataclass(frozen=True)
class A2DeployCfg:
    policy: PolicyCfg = PolicyCfg()
    joints: JointsCfg = JointsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    network: NetworkCfg = NetworkCfg()
    mj: MujocoCfg = MujocoCfg()
    real: RealCfg = RealCfg()

    @property
    def action_dim(self):
        return len(self.joints.names)

    @property
    def actor_dims(self):
        """Layer widths derived from the configured observations, policy and joints."""
        return (self.observations.dimension, *self.policy.hidden_dims, self.action_dim)

    @property
    def control_dt(self):
        return self.mj.timestep * self.mj.decimation

    def joint_ids(self, backend):
        """Policy index -> MuJoCo joint index / real SDK motor ID."""
        names = getattr(self, backend).joint_names
        return tuple(names.index(name) for name in self.joints.names)
