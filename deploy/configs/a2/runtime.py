"""Translate the Python config to C++ manager nodes (JSON is valid YAML).

This is an in-memory transport adapter, not a second editable configuration.
Run with the mjlab Python environment; stdout preserves observation term order.
"""

from dataclasses import asdict
from contextlib import redirect_stdout
import json
from pathlib import Path
import sys

# Also works with python -I runtime.py, independent of the working directory.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

# Keep import diagnostics out of the machine-readable stdout document.
with redirect_stdout(sys.stderr):
    from deploy.configs.a2 import OBSERVATION_FUNCTIONS, load_config, validate


def to_runtime(cfg):
    validate(cfg)
    real = cfg.real
    ids = cfg.joint_ids("real")
    stand = [0.0] * 12
    for index, motor in enumerate(ids):
        stand[motor] = cfg.joints.default_pos[index]
    ranges = {key: getattr(cfg.commands, key) for key in ("lin_vel_x", "lin_vel_y", "ang_vel_z")}
    observations = {
        name: {
            "func": OBSERVATION_FUNCTIONS[term.func].cpp_name,
            "params": dict(term.params),
            "clip": term.clip,
            "scale": [1.0 if term.scale is None else term.scale] * term.size,
            "history_length": cfg.observations.history_length,
        }
        for name, term in cfg.observations.terms.items()
    }

    def fixed_pose(pose, transitions):
        return {"transitions": transitions, "kp": real.stand_kp, "kd": real.stand_kd,
                "ts": [0, real.stand_duration], "qs": [[], pose]}

    return {
        "network": asdict(cfg.network),
        "deploy": {
            "policy_path": cfg.policy.onnx,
            "observation_dim": cfg.observations.dimension,
            "joint_names": cfg.joints.names,
            "joint_ids_map": ids,
            "step_dt": cfg.control_dt,
            "stiffness": real.kp,
            "damping": real.kd,
            "default_joint_pos": cfg.joints.default_pos,
            "commands": {"base_velocity": {
                "ranges": {**ranges, "heading": None},
                "speed_control": {"linear": asdict(cfg.commands.linear), "angular": asdict(cfg.commands.angular)},
            }},
            "actions": {"JointPositionAction": {
                "clip": [cfg.actions.clip] * 12, "scale": [cfg.actions.scale] * 12,
                "offset": cfg.joints.default_pos, "joint_ids": None,
            }},
            "observations": observations,
        },
        "FSM": {
            "initial": real.initial_state,
            "_": {"Passive": {"id": 1}, "FixStand": {"id": 2},
                  "Velocity": {"id": 3, "type": "RLBase"}, "LowStand": {"id": 4, "type": "FixStand"}},
            "Passive": {"transitions": {"FixStand": "start.on_pressed + !A", "LowStand": "A.on_pressed"},
                        "mode": [1] * 12, "kd": real.passive_kd},
            "FixStand": fixed_pose(stand, {"LowStand": "A.on_pressed", "Velocity": "start.on_pressed + !A"}),
            "Velocity": {"transitions": {"FixStand": "start.on_pressed + !A", "LowStand": "A.on_pressed"}},
            "LowStand": fixed_pose(real.low_stand_pos, {"FixStand": "start.on_pressed + !A"}),
        },
    }


if __name__ == "__main__":
    with redirect_stdout(sys.stderr):
        runtime = to_runtime(load_config(sys.argv[1] if len(sys.argv) > 1 else None))
    print(json.dumps(runtime, allow_nan=False))
