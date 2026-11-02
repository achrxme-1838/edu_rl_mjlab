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

List the available A2 environment configurations:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/list_envs.py
```

Only `Unitree-A2-Flat` and `Unitree-A2-Stairs` are registered. Run them through
`sim2sim.sh --terrain flat` or `sim2sim.sh --terrain stairs`.
`--checkpoint /path/to/model.pt` selects another compatible EDU checkpoint;
`--headless --steps 200` runs a bounded rollout. `visualize_terrain.py` previews
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

## A2 sim2real

`deploy/robots/a2/` runs the EDU actor through the existing C++/DDS controller.
The exported model is `config/policy/velocity/v0/exported/policy.onnx` under that
folder. `params/deploy.yaml` matches EDU observations, history, joint order, PD,
and action processing. See [the sim2real guide](doc/sim2real.md) for the exact
policy-to-motor mapping, FSM settings, export commands, and runtime instructions.

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2
ctest --test-dir deploy/robots/a2/build --output-on-failure
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
```

The offline probe compares the production C++ observation/ONNX/action path with
Python using synthetic sensor frames. It opens no DDS channels. Physical robot
walking and C++ DDS simulation rollouts have not been tested.

The SDK's `g1`/`go2` namespaces are shared DDS wrappers used by A2.

## Real joint mapping test

`deploy/robots/a2/build/real_joint_test` uses the same C++ Unitree SDK2 as `a2_ctrl`.
It moves to the EDU default pose, then oscillates one
joint at a time in EDU order. It prints the expected joint name and SDK motor ID.
Space returns the joint and advances; Q / Esc / Ctrl-C sends Passive damping and exits.
Support or suspend the robot: this tool has no balancing policy.
See [setup, controls, and offline preview](doc/real_joint_test.md) before running.

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build --target real_joint_test -j2
./deploy/robots/a2/build/real_joint_test --dry-run
# Replace YOUR_ROBOT_NIC with the robot Ethernet interface; stop other controllers first.
./deploy/robots/a2/build/real_joint_test --network YOUR_ROBOT_NIC
```

This diagnostic does not require Python, Conda, or ONNX Runtime.

## Acknowledgements

- [mjlab](https://github.com/mujocolab/mjlab): simulation framework
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp): GPU simulation
- [MuJoCo](https://github.com/google-deepmind/mujoco): physics engine
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2): robot communication
