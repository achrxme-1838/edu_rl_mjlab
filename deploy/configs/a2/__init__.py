"""A2 configuration loading, validation and mjlab-to-C++ observation metadata."""

from dataclasses import asdict, dataclass
from importlib.util import module_from_spec, spec_from_file_location
import math
from pathlib import Path
import sys

from mjlab_wrapper import mdp

from .config import A2DeployCfg

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = Path(__file__).with_name("config.py")


@dataclass(frozen=True)
class ObservationFunction:
    size: int
    cpp_name: str
    policy_joints: bool = False


# Metadata only: the local wrappers execute mjlab and map to these C++ functions.
OBSERVATION_FUNCTIONS = {
    mdp.base_ang_vel: ObservationFunction(3, "base_ang_vel"),
    mdp.projected_gravity: ObservationFunction(3, "projected_gravity"),
    mdp.generated_commands: ObservationFunction(3, "velocity_commands"),
    mdp.joint_pos_rel: ObservationFunction(12, "joint_pos_rel", policy_joints=True),
    mdp.joint_vel_rel: ObservationFunction(12, "joint_vel_rel", policy_joints=True),
    mdp.last_action: ObservationFunction(12, "last_action"),
}


def resolve_path(path):
    path = Path(path).expanduser()
    return path if path.is_absolute() else REPO_ROOT / path


def validate(cfg):
    """Reject malformed contracts before constructing a simulator or opening DDS."""
    def vector(values, size, name, nonnegative=False):
        if len(values) != size or not all(math.isfinite(x) for x in values):
            raise ValueError(f"{name} must contain {size} finite values")
        if nonnegative and any(x < 0 for x in values):
            raise ValueError(f"{name} must be nonnegative")

    def interval(values, name):
        vector(values, 2, name)
        if values[0] >= values[1]:
            raise ValueError(f"{name} must have min < max")

    expected = {
        f"{leg}_{joint}_joint"
        for leg in ("FL", "FR", "RL", "RR")
        for joint in ("hip", "thigh", "calf")
    }
    for name, names in (
        ("policy", cfg.joints.names),
        ("mj", cfg.mj.joint_names),
        ("real", cfg.real.joint_names),
    ):
        if len(names) != 12 or set(names) != expected:
            raise ValueError(f"{name} joint names must be a permutation of the 12 A2 joints")
    vector(cfg.joints.default_pos, 12, "joints.default_pos")
    for backend in (cfg.mj, cfg.real):
        for key in ("kp", "kd"):
            vector(getattr(backend, key), 12, key, nonnegative=True)
    for key in ("stand_kp", "stand_kd", "passive_kd"):
        vector(getattr(cfg.real, key), 12, key, nonnegative=True)
    vector(cfg.real.low_stand_pos, 12, "real.low_stand_pos")
    for key in (
        "timestep", "effort_limit", "saturation_effort", "velocity_limit", "initial_height"
    ):
        value = getattr(cfg.mj, key)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"mj.{key} must be finite and positive")
    vector((cfg.mj.armature, cfg.mj.frictionloss), 2, "motor losses", nonnegative=True)
    vector(cfg.mj.command, 3, "mj.command")
    for name, value in (
        ("decimation", cfg.mj.decimation),
        ("history_length", cfg.observations.history_length),
    ):
        if type(value) is not int or value < 1:
            raise ValueError(f"{name} must be a positive integer")
    if not cfg.observations.terms:
        raise ValueError("At least one observation term is required")
    for name, term in cfg.observations.terms.items():
        if not callable(term.func) or term.func not in OBSERVATION_FUNCTIONS:
            raise ValueError(f"{name}.func must be a registered A2 observation callable")
        spec = OBSERVATION_FUNCTIONS[term.func]
        if type(term.size) is not int or term.size != spec.size:
            raise ValueError(f"Wrong observation dimension for {name}")
        # The deployed A2 has one command source. Joint selection is injected
        # from cfg.joints in the mjlab adapter and from joint_ids_map in C++.
        expected_params = (
            {"command_name": "base_velocity"}
            if term.func is mdp.generated_commands else {}
        )
        if not isinstance(term.params, dict) or term.params != expected_params:
            raise ValueError(f"Unsupported params for {name} ({term.func.__name__})")
        interval(term.clip, f"{name}.clip")
        if term.scale is not None and not math.isfinite(term.scale):
            raise ValueError(f"{name}.scale must be finite")
    interval(cfg.actions.clip, "actions.clip")
    if not math.isfinite(cfg.actions.scale) or cfg.actions.scale <= 0:
        raise ValueError("actions.scale must be finite and positive")
    for name in ("lin_vel_x", "lin_vel_y", "ang_vel_z"):
        interval(getattr(cfg.commands, name), name)
    for speed in (cfg.commands.linear, cfg.commands.angular):
        vector(tuple(asdict(speed).values()), 4, "speed scale", nonnegative=True)
        if not speed.min <= speed.initial <= speed.max or speed.step <= 0:
            raise ValueError("Invalid initial speed scale or step")
    if cfg.real.initial_state not in ("Passive", "FixStand", "LowStand"):
        raise ValueError("real.initial_state must be Passive, FixStand or LowStand")
    if not math.isfinite(cfg.real.stand_duration) or cfg.real.stand_duration <= 0:
        raise ValueError("real.stand_duration must be finite and positive")
    if not isinstance(cfg.network.interface, str) or "\0" in cfg.network.interface:
        raise ValueError("network.interface must be a string without NUL")
    if type(cfg.network.domain_id) is not int or not 0 <= cfg.network.domain_id <= 232:
        raise ValueError("network.domain_id must be an integer in 0..232")
    if not isinstance(cfg.policy.onnx, str) or not cfg.policy.onnx.strip():
        raise ValueError("policy.onnx must be a nonempty string")
    return cfg


def load_config(path=None):
    if path is None or Path(path).expanduser().resolve() == DEFAULT_CONFIG:
        return validate(A2DeployCfg())
    spec = spec_from_file_location("_a2_deploy_config", Path(path).expanduser().resolve())
    if spec is None or spec.loader is None:
        raise ValueError("--config must select a Python file defining A2DeployCfg")
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return validate(module.A2DeployCfg())
