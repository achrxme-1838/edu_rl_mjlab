"""Registry of A2 playback environments and their checkpoints."""

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

from mjlab.envs import ManagerBasedRlEnvCfg


@dataclass(frozen=True)
class PlayTask:
    env_cfg: ManagerBasedRlEnvCfg
    default_checkpoint: Path


_TASKS: dict[str, PlayTask] = {}


def register_play_task(
    task_id: str, env_cfg: ManagerBasedRlEnvCfg, *, default_checkpoint: Path
):
    if task_id in _TASKS:
        raise ValueError(f"Task {task_id!r} is already registered")
    _TASKS[task_id] = PlayTask(env_cfg, default_checkpoint)


def list_tasks():
    return sorted(_TASKS)


def load_task(task_id):
    return deepcopy(_TASKS[task_id])


def load_env_cfg(task_id):
    return load_task(task_id).env_cfg
