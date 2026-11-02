# Unitree A2 Mjlab

本工作区用于在 MuJoCo 中运行 EDU A2 策略，仅保留 A2。
训练入口、PPO 配置、critic、奖励函数和训练课程已移除。
A2 C++ 部署控制器及其公共运行库仍然保留。
安装方法见[安装配置文档](doc/setup_zh.md)，本机使用 `mjlab` Conda 环境。

## Sim2sim

```bash
conda activate mjlab
cd /home/lim/rl_ws/edu_rl_mjlab
bash scripts/sim2sim.sh
bash scripts/sim2sim.sh --terrain flat
```

默认场景为 5 级 10 cm 上行台阶、2 m 平台及下行台阶。
默认网络为 `networks/01_EDU_A2_STAIR/model_10001.pt`。
环境、actor 加载和回放报告位于 `src/tasks/velocity/config/a2/`。
观测顺序、电机参数、录像方法及已知限制见 [sim2sim 文档](doc/sim2sim.md)。

查看可用的 A2 环境配置：

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/list_envs.py
```

仅注册 `Unitree-A2-Flat` 和 `Unitree-A2-Stairs`。
通过 `sim2sim.sh --terrain flat` 或 `sim2sim.sh --terrain stairs` 运行。
可通过 `--checkpoint /path/to/model.pt` 指定其他兼容的 EDU 网络。
`--headless --steps 200` 用于有限步数回放。
`visualize_terrain.py` 用于预览地形，可选择显示 A2。

## 验证

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
bash scripts/sim2sim.sh --headless --steps 2000 --check
```

`tests/` 用于验证策略加载、观测历史、关节映射、模型物性和任务注册。
报告保存在 `outputs/sim2sim/`。
`networks/01_EDU_A2_STAIR/params/` 保存源网络的配置快照，供上述验证使用。

## A2 sim2real

`deploy/robots/a2/` 使用现有 C++/DDS 控制器运行 EDU actor。
在 `edu_rl_lab` 中运行 `play` 自动导出 ONNX，然后手动将 `policy.onnx` 复制到
`networks/01_EDU_A2_STAIR/` 等策略目录。在 `deploy/robots/a2/config/deploy.yaml`
中修改 `policy_dir` 选择目录；控制器只读取 ONNX，不转换检查点。`deploy/robots/a2/config/deploy.yaml`
保存 EDU 观测、历史、关节顺序、PD 和动作处理配置。
NX (a2) 上的复制及运行方法见 [sim2real 文档](doc/sim2real.md)。

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2
ctest --test-dir deploy/robots/a2/build --output-on-failure
bash scripts/sim2real.sh --check-policy
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
```

离线工具用合成传感器数据对比实际 C++ 观测、ONNX 推理和动作处理与 Python 输出，
不创建 DDS 通道。本次未验证实机行走或 C++ DDS 仿真行走。
在 `deploy/robots/a2/config/config.yaml` 中设置 `network.interface` 后，
在 NX (a2) 上运行 `bash scripts/sim2real.sh`，加上 `--log` 可保存控制器日志。

SDK 中的 `g1`/`go2` 命名空间是 A2 使用的公共 DDS 封装。
手柄输入保持 SDK `UnitreeJoystick` 的默认按键映射和滤波设置。
控制器启动后用 2 秒自动移动至固定站姿。**Start** 切换固定站姿 / 策略模式，
**A** 切换至较低的固定姿态；低姿态下先按 Start 返回正常站姿，再按 Start 进入策略模式。
**R1 / L1** 增加 / 降低线速度倍率，**R2 / L2** 增加 / 降低角速度倍率。
每次按下变化 0.1，初始 0.5，范围 0–1，可在 `deploy.yaml` 中修改。
同时按 **R1 + R2** 退出 `a2_ctrl`：停止策略，发送约 100 ms Passive 阻尼，再结束控制器。
退出操作优先于速度增加。

## 实机关节映射检查

`deploy/robots/a2/build/real_joint_test` 与 `a2_ctrl` 使用同一个 C++ Unitree SDK2。
它先移动到 EDU 默认姿态，再按 EDU 顺序逐个小幅摆动关节，
同时显示预期关节名称和 SDK 电机 ID。Space 先复位当前关节再进入下一个；
Q / Esc / Ctrl-C 发送 Passive 阻尼命令并退出。
此工具没有平衡控制，执行时须支撑或悬吊机器人。
安装与操作见[关节检查文档](doc/real_joint_test.md)。

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build --target real_joint_test -j2
# 在 deploy/robots/a2/config/config.yaml 中设置 network.interface，并先停止其他控制器。
./deploy/robots/a2/build/real_joint_test
```

该工具无需 Python、Conda 或 ONNX Runtime。

## 致谢

- [mjlab](https://github.com/mujocolab/mjlab)：仿真框架
- [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp)：GPU 仿真
- [MuJoCo](https://github.com/google-deepmind/mujoco)：物理引擎
- [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)：机器人通信
