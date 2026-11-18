# A2 실기 관절 순서 확인

`real_joint_test`는 관절을 하나씩 움직여 터미널에 표시된 예상 관절과 실제 움직임을 비교하는 도구다.
관절 매핑·기본 자세·PD 게인은 공통 [`deploy/configs/a2/config.py`](../deploy/configs/a2/config.py)의
`JointsCfg`와 `RealCfg`에서 읽는다. 공통 cfg가 mjlab을 import하므로 시작 시
mjlab이 설치된 Python이 필요하다. 관절 테스트 자체는 ONNX Runtime을 사용하지 않는다.

## 빌드와 실행

SDK 설치와 네트워크 설정은 [sim2real 절차](sim2real.md)를 따른다.
NX (a2)에서 관절 테스트만 빌드하려면:

```bash
cd "$HOME/control_ws/edu_rl_mjlab"
conda activate mjlab
cmake -S deploy/robots/a2 -B deploy/robots/a2/build \
  -DPython3_EXECUTABLE="$(command -v python)"
cmake --build deploy/robots/a2/build --target real_joint_test -j2
```

로봇을 저수준/debug 모드로 전환하고 `a2_ctrl` 등 다른 모터 제어 프로그램을 종료한다.
**로봇을 지지하거나 매단 상태에서 실행한다. 연결되면 자동으로 움직이며 균형 제어는 없다.**
`deploy/configs/a2/config.py`의 `NetworkCfg.interface`에 로봇 Ethernet 인터페이스를 저장한 뒤 실행한다.

```bash
./deploy/robots/a2/build/real_joint_test
```

## 동작과 조작

1. 현재 측정 자세에서 5초 동안 EDU 기본 자세로 이동한다. 첫 1초 동안 Kp가 증가한다.
2. 관절 하나씩 ±0.05 rad, 3초 주기로 왕복하며 나머지 관절은 기본 자세를 유지한다.
3. 터미널의 관절 이름, EDU 번호, SDK 모터 ID, 목표각·측정각을 실제 움직임과 비교한다.
4. **Space**: 현재 관절을 1초 동안 기본 위치로 돌린 뒤 다음 관절로 이동한다.
5. 마지막 관절 이후 Space를 누르면 기본 자세를 유지한다.
6. **Q / Esc / Ctrl-C**: Passive 감쇠(Kp 0 / Kd 10)를 약 0.1초 보내고 종료한다. 위치 유지가 풀린다.

좌우는 로봇이 앞을 보는 기준이다. 기본 순서는 FL → FR → RL → RR의 hip 4개, thigh 4개, calf 4개다.
EDU → SDK 모터 ID 매핑은 `[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]`이다.

```text
[TEST 1/12] expected: left front hip (FL_hip_joint) | EDU joint 0 -> motor ID 3
[TEST 2/12] expected: right front hip (FR_hip_joint) | EDU joint 1 -> motor ID 0
```

## 실행 옵션

더 작은 진폭과 느린 주기로 실행하는 예:

```bash
./deploy/robots/a2/build/real_joint_test --amplitude 0.03 --period 4
```

| 옵션 | 기본값 / 용도 |
| --- | --- |
| `--network` | 선택 사항. 지정하면 `NetworkCfg.interface` 대신 사용 |
| `--amplitude` | 0.05 rad, 0 초과~0.15 이하 |
| `--period` | 3초, 최소 2초 |
| `--stand-duration` | 초기 자세 전환 5초, 최소 3초 |
| `--order` | `edu`; `sdk` 지정 시 모터 ID 순서 |
| `--connect-timeout` | 연결 대기 5초, 최소 1초 |
| `--config` | `A2DeployCfg`를 정의한 Python 파일. 기본값: `deploy/configs/a2/config.py` |

기본 설정은 실행 파일 위치에서 저장소 루트를 찾아 읽는다.
`deploy/robots/a2/build/`(또는 `bin/`), `deploy/configs/a2/`, `mjlab_wrapper/` 배치를 유지한다.
별도 cfg를 지정해도 `runtime.py` 어댑터는 저장소의 `deploy/configs/a2/`에서 읽는다.
기존 `--fsm-config` 옵션은 제거됐으며 `--config` 하나로 통신·관절·Passive 설정을 선택한다.
설정 로딩에는 CMake에서 선택한 Python을 사용한다. `A2_PYTHON` 환경변수에 실행 파일의
절대 경로를 지정하면 실행 시 Python을 바꿀 수 있다.
