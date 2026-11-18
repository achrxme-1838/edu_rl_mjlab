# mjlab Observation 함수 목록

```python
from mjlab_wrapper import mdp
from deploy.configs.a2.config import ObservationTermCfg

# 공통 배포 cfg에서 사용
base_ang_vel = ObservationTermCfg(func=mdp.base_ang_vel, size=3, scale=0.2)
```

이 폴더는 **mjlab 1.2.0의 `mjlab.envs.mdp.observations`에 공개된 관측 함수 9개**를
로컬에서 찾아볼 수 있도록 감싼다. `observations.py`에 실제 `def`, 인자·기본값·타입,
짧은 설명이 있다. 계산은 같은 인자를 받아 mjlab 원본 함수가 수행한다.
`__init__.py`의 명시적 import와 `__all__`에서 전체 목록을 볼 수 있다.

## Observation

[`observations.py`](observations.py)에는 관측 함수 9개가 있다.
관측 결과의 clip·scale·history는 관측 매니저에서 처리한다.

| 함수 | 내용 |
| --- | --- |
| `base_lin_vel` | body 좌표계 base 선속도, 3차원 |
| `base_ang_vel` | body 좌표계 base 각속도, 3차원 |
| `projected_gravity` | body 좌표계 중력 방향, 3차원 |
| `joint_pos_rel` | 선택한 관절 위치 − 기본 위치; `biased`로 encoder bias 선택 |
| `joint_vel_rel` | 선택한 관절 속도 − 기본 속도 |
| `last_action` | 이전 raw action; `action_name`으로 특정 항목 선택 가능 |
| `generated_commands` | `command_name`으로 명령 선택 |
| `builtin_sensor` | `sensor_name`으로 BuiltinSensor 데이터 조회 |
| `height_scan` | RayCastSensor의 hit까지 높이; `offset`, `miss_value` 지원 |

## A2 배포 cfg와 연결

[`deploy/configs/a2/config.py`](../../deploy/configs/a2/config.py)는
`from mjlab_wrapper import mdp`로 이 폴더의 관측 래퍼를 참조한다.
필드 이름은 라벨이고 `func`가 계산을 선택하며, 선언 순서가 정책 입력 순서다.

A2 sim2real은 `base_ang_vel`, `projected_gravity`, `generated_commands`,
`joint_pos_rel`, `joint_vel_rel`, `last_action`의 **6개 관측만 지원**한다.
새 관측을 공통 cfg에 쓰려면
[`OBSERVATION_FUNCTIONS`](../../deploy/configs/a2/__init__.py) 등록과 C++ 구현도 필요하다.
래퍼의 전체 목록과 실기 지원 목록은 다르다.

mjlab 설치 환경은 계속 필요하다. 기존 환경에서는 저장소 루트에서
`python -m pip install -e . --no-deps`를 한 번 실행해 `mjlab_wrapper`를 등록한다.
실기 PC에도 이 폴더를 포함한 `mjlab_wrapper/` 패키지를 함께 배포한다.
reward·event·termination·metric·action·DR은 포함하지 않는다.
