"""Default A2 playback tasks using the bundled EDU policy."""

from src.tasks.registry import register_play_task

from .env_cfgs import make_env_cfg
from .policy import DEFAULT_POLICY

register_play_task(
    "Unitree-A2-Flat",
    make_env_cfg(terrain="flat"),
    default_policy=DEFAULT_POLICY,
)
register_play_task(
    "Unitree-A2-Stairs",
    make_env_cfg(terrain="stairs"),
    default_policy=DEFAULT_POLICY,
)
