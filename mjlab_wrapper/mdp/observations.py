"""Observation functions from mjlab 1.2.0, with explicit local signatures.

These wrappers forward to mjlab; clipping, scaling and history belong to the
observation manager. Joint selection is controlled by ``asset_cfg``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from mjlab.envs.mdp import observations as _observations
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def base_lin_vel(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Base linear velocity in the body frame, shape (num_envs, 3)."""
    return _observations.base_lin_vel(env, asset_cfg=asset_cfg)


def base_ang_vel(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Base angular velocity in the body frame, shape (num_envs, 3)."""
    return _observations.base_ang_vel(env, asset_cfg=asset_cfg)


def projected_gravity(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Gravity projected into the body frame, shape (num_envs, 3)."""
    return _observations.projected_gravity(env, asset_cfg=asset_cfg)


def joint_pos_rel(
    env: ManagerBasedRlEnv,
    biased: bool = False,
    asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
    """Selected joint positions minus defaults; optionally use encoder bias."""
    return _observations.joint_pos_rel(env, biased=biased, asset_cfg=asset_cfg)


def joint_vel_rel(
    env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
    """Selected joint velocities minus their default velocities."""
    return _observations.joint_vel_rel(env, asset_cfg=asset_cfg)


def last_action(env: ManagerBasedRlEnv, action_name: str | None = None) -> torch.Tensor:
    """Previous raw action, for all action terms or one named term."""
    return _observations.last_action(env, action_name=action_name)


def generated_commands(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
    """Command manager output selected by name (e.g. ``base_velocity``)."""
    return _observations.generated_commands(env, command_name=command_name)


def builtin_sensor(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
    """Read a BuiltinSensor's data tensor by its scene name."""
    return _observations.builtin_sensor(env, sensor_name=sensor_name)


def height_scan(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    offset: float = 0.0,
    miss_value: float | None = None,
) -> torch.Tensor:
    """RayCastSensor frame height above each hit, shape (num_envs, num_rays).

    Subtract ``offset`` from hits. Misses use ``miss_value`` or the sensor's
    maximum distance when it is None.
    """
    return _observations.height_scan(
        env, sensor_name=sensor_name, offset=offset, miss_value=miss_value
    )
