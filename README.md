# Unitree A2 Mjlab

This workspace runs the EDU A2 policy in MuJoCo. A2 is the only supported robot.
Training scripts, PPO configurations, critics, rewards, and curricula have been
removed. The A2 C++ deployment controller and its shared runtime remain available.

## Installation

See [the installation guide](doc/setup_en.md). The existing local environment is `mjlab`.

## Sim2sim

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab
bash scripts/sim2sim.sh
bash scripts/sim2sim.sh --terrain flat
```

The default course has five 10 cm steps, a 2 m landing, and descending steps.
The default checkpoint is `networks/01_EDU_A2_STAIR/model_10001.pt`.
The environment, actor loader, and rollout reporting live in
`src/tasks/velocity/config/a2/`.
See [configuration and measured results](doc/sim2sim.md) for observation order,
motor settings, recording options, and known rollout limitations.

The same policy is available through the registered playback tasks:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/list_envs.py
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/play.py Unitree-A2-Flat
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/play.py Unitree-A2-Stairs
```

Only `Unitree-A2-Flat` and `Unitree-A2-Stairs` are registered. `play.py` loads the
actor directly; `--checkpoint-file` selects another compatible EDU checkpoint.
`--headless --steps 200` runs a bounded check. `visualize_terrain.py` previews
terrain with an optional A2 model.

## Verification

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
bash scripts/sim2sim.sh --headless --steps 2000 --check
```

`tests/` is used to check policy loading, observation history, joint mapping,
model properties, and task registration. Reports are saved under `outputs/sim2sim/`.
The snapshots in `networks/01_EDU_A2_STAIR/params/` record the source checkpoint's
settings for these checks; no training entry point uses them.

## A2 C++ runtime

`deploy/robots/a2/` retains the existing ONNX deployment controller.
`simulate/config.yaml` selects the A2 model for the DDS simulator.
These tools require the Unitree SDK2/DDS wrapper, C++ dependencies listed in the
installation guide, and a compatible ONNX policy with its `deploy.yaml`.
The bundled EDU `.pt` checkpoint is loaded by the Python sim2sim tools;
the existing C++ controller still uses its original A2 observation/action settings.

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build -j2
```

For DDS simulation with a compatible ONNX deployment policy and a connected gamepad,
run the simulator and controller in separate terminals:

```bash
./simulate/build/unitree_mujoco
./deploy/robots/a2/build/a2_ctrl --network=lo
```

The SDK's `g1`/`go2` namespaces still appear in shared DDS message wrappers used by
A2. No models, tasks, or deployment targets for those robots remain.

## Acknowledgements

- [mjlab](https://github.com/mujocolab/mjlab): simulation framework
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp): GPU simulation
- [MuJoCo](https://github.com/google-deepmind/mujoco): physics engine
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2): robot communication
