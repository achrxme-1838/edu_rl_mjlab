# Unitree A2 Mjlab

本工作区用于在 MuJoCo 中运行 EDU A2 策略，仅保留 A2。
训练入口、PPO 配置、critic、奖励函数和训练课程已移除。
A2 C++ 部署控制器及其公共运行库仍然保留。
安装方法见[安装配置文档](doc/setup_zh.md)，本机使用 `mjlab` Conda 环境。

## Sim2sim

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab
bash scripts/sim2sim.sh
bash scripts/sim2sim.sh --terrain flat
```

默认场景为 5 级 10 cm 上行台阶、2 m 平台及下行台阶。
默认网络为 `networks/01_EDU_A2_STAIR/model_10001.pt`。
环境、actor 加载和回放报告位于 `src/tasks/velocity/config/a2/`。
观测顺序、电机参数、录像方法及已知限制见 [sim2sim 文档](doc/sim2sim.md)。

也可以通过已注册的任务运行同一策略：

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/list_envs.py
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/play.py Unitree-A2-Flat
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/play.py Unitree-A2-Stairs
```

仅注册 `Unitree-A2-Flat` 和 `Unitree-A2-Stairs`。
`play.py` 直接加载 actor，可通过 `--checkpoint-file` 指定其他兼容的 EDU 网络。
`--headless --steps 200` 用于有限步数检查。
`visualize_terrain.py` 用于预览地形，可选择显示 A2。

## 验证

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
bash scripts/sim2sim.sh --headless --steps 2000 --check
```

`tests/` 用于验证策略加载、观测历史、关节映射、模型物性和任务注册。
报告保存在 `outputs/sim2sim/`。
`networks/01_EDU_A2_STAIR/params/` 保存源网络的配置快照，供上述验证使用。

## A2 C++ 运行工具

`deploy/robots/a2/` 保留原有 ONNX 部署控制器。
DDS 仿真器通过 `simulate/config.yaml` 默认加载 A2。
这些工具需要 Unitree SDK2/DDS wrapper、安装文档中的 C++ 依赖，
以及与 `deploy.yaml` 一致的 ONNX 策略。
内置 EDU `.pt` 网络由 Python sim2sim 加载；C++ 控制器仍使用原有 A2 的观测和动作配置。

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build -j2
```

准备兼容的 ONNX 策略并连接手柄后，在两个终端分别启动 DDS 仿真器和控制器：

```bash
./simulate/build/unitree_mujoco
./deploy/robots/a2/build/a2_ctrl --network=lo
```

公共 DDS 消息封装中的 SDK `g1`/`go2` 命名空间仍被 A2 使用。
这些机器人对应的模型、任务和部署目标均已移除。

## 致谢

- [mjlab](https://github.com/mujocolab/mjlab)：仿真框架
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp)：GPU 仿真
- [MuJoCo](https://github.com/google-deepmind/mujoco)：物理引擎
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)：机器人通信
