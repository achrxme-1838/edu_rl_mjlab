# A2 공통 배포 설정

sim2sim과 sim2real의 설정 원본은
[`deploy/configs/a2/config.py`](../deploy/configs/a2/config.py)의 **`A2DeployCfg`**다.
표준 라이브러리 `dataclass`로 구성하며 `from mjlab_wrapper import mdp`를 사용한다.
학습용 reward, critic, curriculum, randomization은 포함하지 않는다.

## 구성

| `A2DeployCfg` 필드 | 클래스 | 내용과 배열 순서 |
| --- | --- | --- |
| `policy` | `PolicyCfg` | `.pt` / `.onnx` 파일 경로, ELU actor hidden dimensions |
| `joints` | `JointsCfg` | policy 관절 순서, 같은 순서의 기본 관절각 |
| `observations` | `ObservationsCfg` | 관측별 func·params·크기·scale·clip, 선언 순서와 history 길이 |
| `actions` | `ActionsCfg` | raw action scale, 최종 위치 목표 clip |
| `commands` | `CommandsCfg` | vx/vy/wz 범위, 실기 조이스틱 선속도·각속도 배율 |
| `network` | `NetworkCfg` | DDS interface, domain ID |
| `mj` | `MujocoCfg` | MuJoCo 관절 순서의 kp/kd, dt·decimation, DC 모터 한계, 초기 높이·명령 |
| `real` | `RealCfg` | SDK 순서의 kp/kd, 고정 자세 PD·시간·낮은 자세, Passive kd·시작 상태 |

정책 상대 경로는 **저장소 루트** 기준이다. `.pt`는 sim2sim에서, `.onnx`는
sim2real에서 사용한다. ONNX 변환은 기존대로 `edu_rl_lab`의 `play`에서 수행한다.
제어 주기는 `mj.timestep × mj.decimation`에서 공통으로 계산한다(기본 0.02초).
지형·뷰어·기록 옵션은 sim2sim 실행 코드와 CLI에서 관리한다.

## 관절과 게인

- Policy: hip FL/FR/RL/RR → thigh FL/FR/RL/RR → calf FL/FR/RL/RR.
- MuJoCo: FL → FR → RL → RR, 각 다리 hip/thigh/calf.
- Real SDK: FR → FL → RR → RL, 각 다리 hip/thigh/calf. 배열 인덱스가 모터 ID다.

`cfg.joint_ids("mj")`는 `[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`,
`cfg.joint_ids("real")`은 `[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]`에 해당한다.
두 매핑은 이름에서 계산되므로 별도로 수정하지 않는다.
MuJoCo 게인은 이름으로 실제 관절에 연결한다. 실기 관절 순서는 실제 SDK 모터 ID와 맞춰야 한다.

양쪽 정책 PD는 기본 Kp 25 / Kd 0.5다. `mj.kp/kd`와 `real.kp/kd`는 각각
12개 튜플이므로 관절별로 다른 게인도 설정할 수 있다.
고정 기립에는 `real.stand_kp/kd`(200/5), Passive에는 `real.passive_kd`(10)를 사용한다.
기본 기립 자세와 action offset은 `joints.default_pos`를 공통으로 사용한다.
`real.low_stand_pos`는 SDK 순서로 지정한다.

## Observation 계약

`ObservationsCfg`의 항목 선언 순서가 입력 순서다. **필드 이름은 라벨이며,
실제 계산 함수는 `func`로 지정한다.** 같은 위치에서 필드 이름만 바꿔도
Python·C++의 관측값과 연결 순서는 유지된다.

```python
from mjlab_wrapper import mdp
from deploy.configs.a2.config import ObservationTermCfg

# ObservationsCfg 안에서 선언하는 예. base_ang_vel 대신 gyro라는 이름을 써도 된다.
gyro: ObservationTermCfg = ObservationTermCfg(func=mdp.base_ang_vel, size=3, scale=0.2)
command: ObservationTermCfg = ObservationTermCfg(
    func=mdp.generated_commands, size=3, params={"command_name": "base_velocity"}
)
```

`func`에는 **로컬 mjlab 래퍼 함수**를 지정한다.
[`mjlab_wrapper/mdp/observations.py`](../mjlab_wrapper/mdp/observations.py)에서
함수별 인자와 설명을 확인할 수 있다. 래퍼는 입력을 그대로 mjlab 원본에 전달한다.
관측 함수 9개의 목록은 [래퍼 안내](../mjlab_wrapper/mdp/README.md)를 참고한다.
sim2sim에서는 래퍼를 통해 mjlab 함수를 실행하고, sim2real에서는
[`OBSERVATION_FUNCTIONS`](../deploy/configs/a2/__init__.py)의 메타데이터로
C++ 함수 키를 전달한다. 설정을 읽는 Python 환경에는 mjlab이 설치돼 있어야 한다.

| Python `func` | C++ 등록 함수 키 |
| --- | --- |
| `mdp.base_ang_vel` | `base_ang_vel` |
| `mdp.projected_gravity` | `projected_gravity` |
| `mdp.generated_commands` | `velocity_commands` |
| `mdp.joint_pos_rel` | `joint_pos_rel` |
| `mdp.joint_vel_rel` | `joint_vel_rel` |
| `mdp.last_action` | `last_action` |

`size`는 선택한 함수의 반환 차원과 일치해야 한다. 현재 명령 함수의 `params`는
`{"command_name": "base_velocity"}`를 사용하며 나머지 함수는 빈 `params`를 사용한다.
관절 관측의 `asset_cfg`는 policy 관절 순서에 맞게 어댑터가 자동으로 연결한다.
미등록 함수와 지원하지 않는 params는 DDS 연결 전에 거부된다. 새 관측 계산을 추가하려면
`OBSERVATION_FUNCTIONS`에 로컬 래퍼의 메타데이터와 대응하는 C++ 관측 함수도 함께 추가한다.
래퍼에 있는 모든 함수가 A2 실기에서 지원되는 것은 아니며, 현재 공통 cfg에서는 위 6개만 지원한다.

기본 설정의 구성은 다음과 같다.

| 순서 | 항목 | 프레임당 차원 | scale | 기본 history 포함 구간 |
| --- | --- | ---: | ---: | --- |
| 1 | `base_ang_vel` | 3 | 0.2 | `[0:15]` |
| 2 | `projected_gravity` | 3 | 1 | `[15:30]` |
| 3 | `velocity_commands` | 3 | 1 | `[30:45]` |
| 4 | `joint_pos_rel` | 12 | 1 | `[45:105]` |
| 5 | `joint_vel_rel` | 12 | 0.05 | `[105:165]` |
| 6 | `last_action` | 12 | 1 | `[165:225]` |

각 항목은 **clip → scale → 항목별 history → 항목 연결** 순서로 처리한다.
clip 기본값은 `(-100, 100)`이고 `scale=None`은 1과 같다.
history는 항목마다 과거→현재 5프레임이다. reset에서는 첫 관측을 반복해 채우고
이전 raw action은 0으로 만든다. observation noise와 normalization은 사용하지 않는다.
관절 관측과 action은 모두 policy 순서를 따른다.

입력 차원은 `cfg.observations.dimension`으로 계산한다(기본 45 × 5 = 225).
기본 actor는 `225 → 512 → 256 → 128 → 12` ELU다.
관측 순서·scale·history를 바꾸면 해당 계약으로 학습한 정책도 함께 선택해야 한다.
차원이 달라지면 Python 가중치 로딩/C++ ONNX 사전 검사에서 실패하지만,
순서나 scale처럼 차원이 같은 의미 변경은 정책 메타데이터와 자동 대조하지 않는다.
위치 목표는 `clip(default_pos + actions.scale × raw_action, actions.clip)`이다.

## 수정과 실행

기본값은 `config.py`의 해당 클래스에서 수정한다. 실행을 종료하고 다시 시작하면 반영된다.
일시적인 별도 설정은 Python 파일에서 필요한 부분만 교체할 수 있다.

```python
# /tmp/a2_custom.py
from dataclasses import dataclass, replace
from deploy.configs.a2.config import A2DeployCfg as BaseCfg, NetworkCfg, MujocoCfg

base = BaseCfg()

@dataclass(frozen=True)
class A2DeployCfg(BaseCfg):
    network: NetworkCfg = replace(base.network, interface="eth0")
    mj: MujocoCfg = replace(base.mj, command=(0.3, 0.0, 0.0))
```

```bash
bash scripts/sim2sim.sh --config /tmp/a2_custom.py --terrain flat
bash scripts/sim2real.sh --config /tmp/a2_custom.py --check-policy
./deploy/robots/a2/build/real_joint_test --config /tmp/a2_custom.py --help
```

`--config`의 상대 경로는 실행 디렉터리 기준이다.
`--checkpoint` / `--command`(sim2sim), `--network`(실기)는 실행 시 cfg 값을 덮어쓴다.

## C++ 연결 및 이전 설정

C++은 시작 시 **CMake의 `Python3_EXECUTABLE`로 선택한 Python**에 `-I -B`를 붙여
`runtime.py`를 실행한다. 이 Python에는 mjlab과 그 의존성이 설치돼 있어야 한다.
현재 개발 PC에서는 `/home/lim/anaconda3/envs/mjlab/bin/python`을 사용한다.
실기 PC에서는 해당 PC에 설치한 환경의 경로를 지정한다.

```bash
conda activate mjlab
cmake -S deploy/robots/a2 -B deploy/robots/a2/build \
  -DBUILD_OFFLINE_TESTS=ON -DPython3_EXECUTABLE="$(command -v python)"
cmake --build deploy/robots/a2/build -j2
```

실행 시 다른 Python을 선택하려면 `A2_PYTHON`에 실행 파일의 절대 경로를 지정한다.
설정 자체를 읽기 전에 필요한 경로이므로 cfg 외부에서 선택한다.

```bash
A2_PYTHON=/path/to/mjlab/bin/python bash scripts/sim2real.sh --check-policy
```

어댑터는 공통 cfg를 검증한 뒤 JSON을 pipe로 전달하고 종료한다.
C++의 기존 yaml-cpp 파서가 이 JSON을 메모리에서 읽는다. observation 선언 순서와 라벨은
유지되며 각 항목의 `func`로 C++ 계산 함수를 선택한다.
추론과 DDS 통신에는 Python이 관여하지 않는다. 설정 오류는 DDS 연결 전에 종료시킨다.
cfg 로딩을 위해 mjlab을 import하지만 MuJoCo 환경을 생성하거나 정책을 Python으로 실행하지는 않는다.

변환 결과만 확인하려면 다음 명령을 사용한다. 로봇에 연결하지 않는다.

```bash
conda activate mjlab
python -I -B deploy/configs/a2/runtime.py | python -m json.tool
```

기존 `deploy/robots/a2/config/deploy.yaml` / `config.yaml`은 제거했다.
`policy_dir` 폴더 지정은 `PolicyCfg.onnx` 파일 지정으로 변경됐으며,
`real_joint_test --fsm-config`는 공통 `--config`로 통합했다.
`networks/.../params/*.yaml`은 학습 당시 기록이므로 그대로 보존하고 실행 설정으로 읽지 않는다.

최초 전환 시 C++ 실행 파일을 다시 빌드한다. sim2sim 환경에서는 `pip install -e . --no-deps`를
한 번 실행해 `deploy.configs`와 `mjlab_wrapper` 패키지를 등록한다. 이후 cfg 값만 수정할 때는 재빌드가 필요 없다.
실행 파일은 `deploy/robots/a2/build/` 또는 `bin/` 위치를 유지해야 하며,
저장소의 `deploy/configs/a2/`와 `mjlab_wrapper/`도 함께 배포해야 한다.
