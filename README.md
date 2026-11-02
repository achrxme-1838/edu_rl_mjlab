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
Run `play` in `edu_rl_lab` to export ONNX, then manually copy `policy.onnx` to
`networks/01_EDU_A2_STAIR/` (or another policy folder). Set `policy_dir` in
`deploy/robots/a2/config/deploy.yaml` to select the folder; the controller only loads ONNX and does
not convert checkpoints. `deploy/robots/a2/config/deploy.yaml` matches EDU
observations, history, joint order, PD, and action processing. See
[the sim2real guide](doc/sim2real.md) for copying and running the policy on NX (a2).

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2
ctest --test-dir deploy/robots/a2/build --output-on-failure
bash scripts/sim2real.sh --check-policy
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
```

The offline probe compares the production C++ observation/ONNX/action path with
Python using synthetic sensor frames. It opens no DDS channels. Physical robot
walking and C++ DDS simulation rollouts have not been tested.

After configuring `network.interface` in `deploy/robots/a2/config/config.yaml`,
run `bash scripts/sim2real.sh` on NX (a2). Add `--log` to save the controller log.

The SDK's `g1`/`go2` namespaces are shared DDS wrappers used by A2.
Joystick inputs retain the SDK's `UnitreeJoystick` mapping and filter defaults.
The controller automatically moves to the fixed standing pose in 2 seconds.
**Start** toggles fixed standing / policy; **A** selects the lower fixed pose.
From the lower pose, Start returns to normal standing before policy can be selected.
**R1 / L1** increase / decrease the linear speed scale; **R2 / L2** adjust the angular scale.
Each press changes the scale by 0.1 (initial 0.5, range 0–1), configured in `deploy.yaml`.
Press **R1 + R2** together to exit `a2_ctrl`: stop the policy, send Passive damping
for 100 ms, then close the controller. Exit takes priority over increasing speed.

## Real joint mapping test

`deploy/robots/a2/build/real_joint_test` uses the same C++ Unitree SDK2 as `a2_ctrl`.
It moves to the EDU default pose, then oscillates one
joint at a time in EDU order. It prints the expected joint name and SDK motor ID.
Space returns the joint and advances; Q / Esc / Ctrl-C sends Passive damping and exits.
Support or suspend the robot: this tool has no balancing policy.
See [setup and controls](doc/real_joint_test.md) before running.

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build --target real_joint_test -j2
# Set network.interface in deploy/robots/a2/config/config.yaml; stop other controllers first.
./deploy/robots/a2/build/real_joint_test
```

This diagnostic does not require Python, Conda, or ONNX Runtime.

## Acknowledgements

- [mjlab](https://github.com/mujocolab/mjlab): simulation framework
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp): GPU simulation
- [MuJoCo](https://github.com/google-deepmind/mujoco): physics engine
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2): robot communication
