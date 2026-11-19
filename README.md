# Unitree A2 Mjlab

EDU A2 정책의 **MuJoCo sim2sim**과 **C++/DDS sim2real** 실행 저장소다.
A2만 지원하며, 정책 학습과 ONNX export는 `edu_rl_lab`에서 진행한다.
두 실행 경로 모두 `PolicyCfg.onnx`에 지정한 **동일한 ONNX 정책**을 사용한다.

[설치](#설치) · [Sim2sim](#sim2sim) · [Sim2real](#sim2real) · [설정 변경](#설정-변경) · [검증 및 문제 해결](#검증-및-문제-해결)

## 설치

기준 환경은 **Ubuntu 22.04 / Python 3.11**이다. GPU 시뮬레이션에는 NVIDIA GPU·드라이버가 필요하며,
sim2sim은 `--device cpu`도 지원한다. C++ ONNX 추론은 CPU에서 실행하지만 **설정 로딩용 mjlab Python은 실기 PC에도 필요**하다.

### Python 환경과 저장소

```bash
sudo apt update
sudo apt install -y git wget build-essential cmake pkg-config \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

<details>
<summary>Conda가 없는 경우: Miniconda 설치</summary>

`uname -m`이 `x86_64` 또는 `aarch64`인 Linux에서 실행한다. 기존 Conda가 있으면 생략한다.

```bash
a2_conda_installer="/tmp/Miniconda3-Linux-$(uname -m).sh"
wget "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-$(uname -m).sh" \
  -O "$a2_conda_installer"
bash "$a2_conda_installer" -b -p "$HOME/miniconda3"
"$HOME/miniconda3/bin/conda" init bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
rm "$a2_conda_installer"
```

</details>

이미 `mjlab` 환경이나 저장소가 있으면 생성·clone을 생략하고 기존 경로를 사용한다.
아래 `~/control_ws`는 예시이며, 현재 개발 PC의 저장소는 `/home/lim/rl_ws/edu_rl_mjlab`이다.

```bash
conda create -n mjlab python=3.11 -y
conda activate mjlab
mkdir -p "$HOME/control_ws"
git clone https://github.com/achrxme-1838/edu_rl_mjlab.git "$HOME/control_ws/edu_rl_mjlab"
cd "$HOME/control_ws/edu_rl_mjlab"
python -m pip install -e '.[export]'
python -I -c "from mjlab.envs import mdp; import onnxruntime; import onnx"
```

`setup.py`가 mjlab 1.2.0, mujoco-warp 3.5.0, ONNX Runtime 1.22.0을 설치하며, `[export]`는 ONNX 검사에 필요한 패키지를 포함한다.
기존 환경도 `python -m pip install -e '.[export]'`로 업데이트한다.
영상 저장이 필요하면 `python -m pip install imageio imageio-ffmpeg`를 추가 실행한다.
이후 명령은 **활성화한 mjlab 환경과 저장소 루트**에서 실행한다.

### 실기용 SDK·C++ 빌드

sim2sim만 사용하면 이 절은 생략한다. C++ 빌드는 `x86_64`와 `aarch64`를 지원하며 ONNX Runtime은 저장소에 포함되어 있다.
실기 PC에도 위 Python 환경과 저장소를 준비하고, 변경한 소스·정책 파일을 함께 반영한다.

```bash
# Unitree SDK2 설치. 이미 같은 SDK가 설치되어 있으면 생략한다.
git clone https://github.com/unitreerobotics/unitree_sdk2.git "$HOME/control_ws/unitree_sdk2"
git -C "$HOME/control_ws/unitree_sdk2" checkout --detach \
  c8a71e281093593f4dcc7bceb3b3b529ff0e36b4
cmake -S "$HOME/control_ws/unitree_sdk2" -B "$HOME/control_ws/unitree_sdk2/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_EXAMPLES=OFF
cmake --build "$HOME/control_ws/unitree_sdk2/build" -j2
sudo cmake --install "$HOME/control_ws/unitree_sdk2/build"
sudo ldconfig

# 저장소 루트에서 제어기·관절 테스트·오프라인 테스트 빌드
cmake -S deploy/robots/a2 -B deploy/robots/a2/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_OFFLINE_TESTS=ON \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build deploy/robots/a2/build -j2
ldd deploy/robots/a2/build/a2_ctrl
ldd deploy/robots/a2/build/real_joint_test
ctest --test-dir deploy/robots/a2/build --output-on-failure
bash scripts/sim2real.sh --check-policy
```

`ldd`에 `not found`가 없어야 한다. `--check-policy`는 ONNX 입출력을 검사하고 **DDS 연결 없이 종료**한다.
`Policy check complete (no DDS channels opened).`가 정상 결과다.

<details>
<summary>기존 unitree-a2-pc2의 사용자 경로 SDK를 쓰는 경우</summary>

해당 x86_64 PC의 SDK는 `~/.local/unitree_sdk2-21d0a3b`에 설치되어 있다.
사용 버전은 `21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b`이며 출처는 SDK 소스의 `SOURCE_VERIFICATION.json`에 기록돼 있다.
이 설치를 사용할 때는 위 제어기 configure 명령을 다음으로 대체한다.

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_OFFLINE_TESTS=ON \
  -DPython3_EXECUTABLE="$(command -v python)" \
  -DCMAKE_PREFIX_PATH="$HOME/.local/unitree_sdk2-21d0a3b" \
  -DCMAKE_EXE_LINKER_FLAGS=-Wl,--disable-new-dtags
cmake --build deploy/robots/a2/build -j2
```

마지막 옵션은 ROS/SLAM의 `LD_LIBRARY_PATH`보다 빌드에 사용한 라이브러리 경로를 우선한다.
`ldd`에서 `libddsc`와 `libddscxx`가 위 설치 경로를 가리키는지 확인한다.

</details>

## Sim2sim

```bash
bash scripts/sim2sim.sh                 # 10 cm × 5단 상승 → 2 m 플랫폼 → 하강
bash scripts/sim2sim.sh --terrain flat  # 평지
bash scripts/sim2sim.sh --viewer viser  # 웹 뷰어
bash scripts/sim2sim.sh --terrain flat --command 0.3 0 0 \
  --onnx networks/01_EDU_A2_STAIR/policy.onnx
```

기본 명령은 전진 0.4 m/s다. Native 뷰어는 **Space**로 일시정지, **Enter**로 초기화한다.
웹 뷰어는 Base velocity에서 Enable을 켜고 속도를 조절한다. GUI는 창을 닫을 때까지 실행되며 `--steps`는 headless 전용이다.
`--device cpu` 또는 `--device cuda:0`는 MuJoCo 시뮬레이션 장치를 선택한다. 정책 추론은 실기와 같은 ONNX Runtime CPU를 사용하며 action을 시뮬레이션 장치로 전달한다.

```bash
# 평지 20초 검증
bash scripts/sim2sim.sh --terrain flat --headless --steps 1000 --check
# 계단 40초 + 영상. 계단 높이 변경은 --stair-height 0.15 등으로 지정
bash scripts/sim2sim.sh --headless --steps 2000 --check \
  --video outputs/sim2sim/stairs.mp4
```

`--report /path/report.json`으로 결과 경로를 지정할 수 있다. 기본 보고서는 `outputs/sim2sim/<timestamp>/report.json`에 저장된다.
`--check`는 비정상 수치·리셋·전도와 평지 속도 오차 또는 계단 완주 여부를 검사하며, 실패하면 종료 코드 1을 반환한다.
등록된 task는 `Unitree-A2-Flat`, `Unitree-A2-Stairs`다.

## Sim2real

### 정책과 통신 준비

1. `edu_rl_lab`에서 `play`로 생성한 `exported/<체크포인트 이름>/policy.onnx`를 이 저장소에 복사한다.
2. [config.py](deploy/configs/a2/config.py)의 `PolicyCfg.onnx`에 ONNX **파일 경로**를 지정한다. 제어기는 `.pt`를 ONNX로 변환하지 않는다.
3. `ip -br link`, `ip -br addr`로 로봇망 NIC를 확인하고 `NetworkCfg.interface`에 지정한다. DDS `domain_id`는 로봇 측과 맞춘다. IP·넷마스크는 OS에서 설정한다.
4. `bash scripts/sim2real.sh --check-policy`를 확인한 뒤, 로봇을 저수준/debug 모드로 전환하고 다른 모터 제어 프로그램을 종료한다.

### 관절 매핑 확인

**로봇을 지지하거나 매단 상태에서 실행한다. 연결 즉시 움직이며 균형 제어는 없다.**

```bash
./deploy/robots/a2/build/real_joint_test
# 진폭·주기 변경 예
./deploy/robots/a2/build/real_joint_test --amplitude 0.03 --period 4
```

기본 동작은 5초 동안 기본 자세로 이동한 후 관절별 ±0.05 rad, 3초 주기로 왕복한다.
터미널의 예상 관절·SDK 모터 ID와 실제 움직임을 비교한다. **Space**는 다음 관절,
**Q / Esc / Ctrl-C**는 Passive 감쇠 후 종료다. 종료하면 위치 유지가 풀린다.
`--order sdk`는 모터 ID 순서이며, 나머지 옵션은 `--help`로 확인한다.

### 정책 실행과 조작

관절 테스트를 종료하고 조이스틱을 중립에 둔다. **제어기 연결 후 2초 동안 자동 기립한다.**

```bash
bash scripts/sim2real.sh
# 로그가 필요하면 --log 추가: deploy/robots/a2/log/log.txt
```

| 조작 | 동작 |
| --- | --- |
| Start | 기립 완료 후 고정 자세 ↔ 정책 모드 |
| A | 약 5 cm 낮은 고정 자세. 이 상태에서 Start는 기본 자세로 복귀 |
| 왼쪽 스틱 상하 / 좌우 | 전후 속도 / 회전 속도 |
| 오른쪽 스틱 좌우 | 좌우 이동 속도 |
| R1 / L1 | 선속도 배율 증가 / 감소 |
| R2 / L2 | 각속도 배율 증가 / 감소 |
| **R1 + R2** | Passive 감쇠 약 0.1초 후 종료 |

속도 배율은 초기 0.5, 증감 0.1, 범위 0~1이다. 배율 1에서 전후 ±0.6 m/s, 좌우 ±0.4 m/s, 회전 ±1 rad/s다.
종료는 **R1 + R2**로 하고 `Controller stopped.`를 확인한다. `a2_ctrl`의 Ctrl-C에는 별도 감쇠 처리가 없다.

## 설정 변경

설정 원본은 **[deploy/configs/a2/config.py](deploy/configs/a2/config.py)**다.
`env_cfgs.py`는 sim2sim 환경 구성, `runtime.py`는 C++ 설정 전달을 담당하므로 일반적인 설정 변경 때는 수정하지 않는다.
**설정값 변경은 재시작, C++ 코드 변경은 재빌드**로 반영한다.

| 목표 | 수정할 설정 | 필요한 작업 |
| --- | --- | --- |
| Policy network 변경 | `PolicyCfg.onnx` | 학습한 정책을 ONNX로 export하고 양쪽에서 사용할 파일 경로 지정 |
| 통신 network 변경 | `NetworkCfg.interface`, `domain_id` | 실제 NIC와 DDS domain 지정. 임시 NIC 변경은 `--network 인터페이스명` |
| Obs 구성 변경 | `ObservationsCfg` 항목·순서·`func`·`params`·`size`·`scale`·`clip`·`history_length` | 학습 당시 입력 구성과 일치시키거나, 새 구성으로 학습된 정책 준비 |
| 관절·action 변경 | `JointsCfg`, `ActionsCfg` | 정책 관절 순서·기본 자세·action scale·목표각 clip 지정 |
| 물리·PD 변경 | `MujocoCfg`, `RealCfg` | 시뮬레이션 dt·모터 한계, 각 backend의 관절별 PD·기립·Passive 설정 |
| 명령 범위 변경 | `CommandsCfg` | 속도 범위와 조이스틱 배율 지정 |

### Policy network

- `edu_rl_lab`에서 학습한 정책을 ONNX로 export한 뒤 `PolicyCfg.onnx`를 바꾼다. 상대 경로는 저장소 루트 기준이다. 이번 sim2sim 실행만 변경하려면 `--onnx /path/policy.onnx`를 사용한다.
- 은닉층 크기·개수·활성화 함수는 ONNX 파일에 포함된다. 배포 cfg에 `checkpoint`나 `hidden_dims`를 입력하지 않는다. `.pt`는 원본 가중치와의 검증용 기록으로 남긴다.
- 입력은 이름 `obs`, float32 `[1, sum(obs.size) × history_length]`이며 출력은 float32 `[1, 관절 개수]` 하나다. 실행 전에 파일의 입출력을 설정과 대조한다. 동적 batch나 RNN의 추가 입력·상태는 현재 지원하지 않는다.
- 기본 ONNX는 정규화 없는 ELU actor다. 다른 모델의 정규화도 export된 그래프와 학습 당시 관측 처리에 맞춰야 한다. cfg를 바꿔도 기존 ONNX가 새 구조로 변환되지는 않는다.

### Obs 구성

필드 이름은 라벨이고 **선언 순서가 입력 순서**다. `func`는 [로컬 관측 함수](mjlab_wrapper/mdp/observations.py)를 지정하며,
`size`는 실제 반환 차원과 맞춰야 한다. 아래는 공통 cfg에서 지원하는 6종과 기본값이다.

| 순서 | Python 함수 | 프레임당 차원 | scale |
| --- | --- | ---: | ---: |
| 1 | `mdp.base_ang_vel` | 3 | 0.2 |
| 2 | `mdp.projected_gravity` | 3 | 1 |
| 3 | `mdp.generated_commands` | 3 | 1 |
| 4 | `mdp.joint_pos_rel` | 12 | 1 |
| 5 | `mdp.joint_vel_rel` | 12 | 0.05 |
| 6 | `mdp.last_action` | 12 | 1 |

처리는 **clip → scale → 항목별 과거→현재 history → 항목 연결** 순서다. 기본 clip은 ±100, history는 5이며,
reset 시 첫 관측을 반복하고 이전 action은 0으로 채운다. history를 3으로 바꾸면 입력은 225 → 135가 된다.
**순서·scale·clip·history를 학습 설정과 맞춘다.** 같은 차원에서 의미만 달라지는 변경은 자동 검사로 잡히지 않을 수 있다.

```python
# ObservationsCfg 안에서 선언하는 예
from mjlab_wrapper import mdp
from deploy.configs.a2.config import ObservationTermCfg

gyro: ObservationTermCfg = ObservationTermCfg(func=mdp.base_ang_vel, size=3, scale=0.2)
command: ObservationTermCfg = ObservationTermCfg(
    func=mdp.generated_commands, size=3, params={"command_name": "base_velocity"}
)
```

현재 `params`는 명령 함수의 `command_name="base_velocity"`만 사용하며 나머지는 빈 dict다. 관절 선택은 정책 순서로 자동 연결된다.
래퍼에는 `base_lin_vel`, `builtin_sensor`, `height_scan`도 있지만 공통 cfg에 아직 등록되지 않았다.
새 관측·인자는 Python 계산, [등록·검증 코드](deploy/configs/a2/__init__.py), 실기용 C++ 계산과 센서·추정값을 함께 연결한다.

### 관절·제어 기본값

| 배열 | 순서 |
| --- | --- |
| Policy | hip FL/FR/RL/RR → thigh FL/FR/RL/RR → calf FL/FR/RL/RR |
| MuJoCo | FL → FR → RL → RR, 각 다리 hip/thigh/calf |
| Real SDK | FR → FL → RR → RL, 각 다리 hip/thigh/calf; 인덱스가 모터 ID |

관절 매핑은 이름에서 자동 계산한다. 기본 Policy → SDK 매핑은 `[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]`이다.
목표각은 `clip(default_pos + actions.scale × raw_action, actions.clip)`이며 기본 scale은 0.25다.
정책 PD는 Kp/Kd 25/0.5, 고정 자세는 200/5, Passive는 0/10이다.
제어 주기는 `mj.timestep × mj.decimation`으로 공통 계산하며 기본은 `0.005 × 4 = 0.02초`다.
`mj.kp/kd`와 `real.kp/kd`·`low_stand_pos`는 각각 위 backend 순서로 지정한다.

## 검증 및 문제 해결

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
ctest --test-dir deploy/robots/a2/build --output-on-failure
bash scripts/sim2real.sh --check-policy
# C++로 전달되는 설정만 확인
python -I -B deploy/configs/a2/runtime.py | python -m json.tool
```

C++ 관련 테스트는 위의 `BUILD_OFFLINE_TESTS=ON` 빌드 후 실행한다. 오프라인 테스트는 DDS 채널을 열지 않는다.
검증 범위는 관측 history·관절 매핑·모터·정책 로딩과 Python/C++ 출력 비교이며, **실기 보행과 C++ DDS 시뮬레이션 rollout은 미검증**이다.
기존 시뮬레이션 기록에서 평지 20초는 통과했지만, 10 cm 계단은 완주 후 재검증에서 측면 이탈이 있었고 15 cm 계단은 전도했다.

| 증상 | 확인할 항목 |
| --- | --- |
| `a2_ctrl` 실행 파일 없음 | [설치](#설치)의 SDK·C++ 빌드 수행 |
| Python 설정 import 실패 | mjlab 환경에서 `python -m pip install -e . --no-deps`; CMake의 `Python3_EXECUTABLE` 확인 |
| 실행 시 다른 Python 필요 | `A2_PYTHON=/path/to/mjlab/bin/python bash scripts/sim2real.sh --check-policy` |
| `ldd`에 `not found` 또는 DDS 라이브러리 충돌 | SDK 설치·`sudo ldconfig`·실제 로딩 경로 확인. 사용자 설치는 위 접기 영역 참고 |
| 정책 크기 불일치 | ONNX 입출력·관측 크기/history·관절 개수가 같은 학습 설정인지 확인 |
| NIC 없음 | `ip -br link`와 `NetworkCfg.interface` 확인. `--check-policy`는 통신 연결을 검사하지 않음 |
| Isaac Python 경로 혼입 | `scripts/*.sh` 사용. Python 직접 실행 시 `env -u PYTHONPATH PYTHONNOUSERSITE=1` 추가 |

실기 배포 시 `deploy/configs/a2/`, `mjlab_wrapper/`, 정책 파일도 함께 옮기고 실행 파일은 `deploy/robots/a2/build/` 또는 `bin/`에 둔다.
`networks/.../params/`는 학습 당시 기록이며 실행 설정이 아니다. 체크포인트 출처·모델 정보는 같은 폴더의 `provenance.json`, `source_model.json`에서 확인한다.

기반 프로젝트: [mjlab](https://github.com/mujocolab/mjlab), [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp), [MuJoCo](https://github.com/google-deepmind/mujoco), [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2).
