# EDU A2 → Unitree C++ deployment

`deploy/robots/a2/`는 EDU A2 actor를 기존 Unitree C++ 배포 흐름에 연결한다.
입력과 출력 처리는 기존 `BaseArticulation`, `ObservationManager`, `ActionManager`,
`OrtRunner`를 사용하며, Passive → FixStand → Velocity 상태 전환과 DDS 통신을 유지한다.
실제 로봇 연결이나 모터 명령 전송은 수행하지 않았다. 아래 검증은 모두 오프라인이다.

## 정책 파일

- 원본: `networks/01_EDU_A2_STAIR/model_10001.pt`
- 실행 모델: `deploy/robots/a2/config/policy/velocity/v0/exported/policy.onnx`
- 실행 설정: 같은 `v0/params/deploy.yaml`
- 출처 및 파일 해시: 같은 `v0/exported/manifest.json`

ONNX에는 actor만 들어 있다. 고정 입력 `obs: [1, 225]`, 출력 `actions: [1, 12]`,
float32, opset 17이며 출력은 EDU 순서의 raw action이다.
관측 정규화나 관절 재배열을 ONNX 내부에 추가하지 않았다.

내보낸 파일은 포함되어 있다. 체크포인트를 교체할 때는 다음을 실행한다.
같은 관측·관절·네트워크 구조의 EDU 체크포인트를 사용해야 한다.

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab
# onnx 패키지가 없는 환경은 먼저 python -m pip install -e '.[export]'
env -u PYTHONPATH PYTHONNOUSERSITE=1 python scripts/export_sim2real.py
# 또는 --checkpoint networks/다른_실행/model.pt
```

## 관절 순서

기존 mjlab 다리별 순서(FL → FR → RL → RR)를 SDK 모터 ID로 바꾸는 매핑은
`[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]`이었다.
EDU 순서를 mjlab 순서에 대응시키는 인덱스
`[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`와 합성했다.

| EDU 인덱스 | 관절 | mjlab 인덱스 | SDK 모터 ID |
| ---: | --- | ---: | ---: |
| 0 | FL_hip | 0 | 3 |
| 1 | FR_hip | 3 | 0 |
| 2 | RL_hip | 6 | 9 |
| 3 | RR_hip | 9 | 6 |
| 4 | FL_thigh | 1 | 4 |
| 5 | FR_thigh | 4 | 1 |
| 6 | RL_thigh | 7 | 10 |
| 7 | RR_thigh | 10 | 7 |
| 8 | FL_calf | 2 | 5 |
| 9 | FR_calf | 5 | 2 |
| 10 | RL_calf | 8 | 11 |
| 11 | RR_calf | 11 | 8 |

최종 `joint_ids_map`은 `[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]`이다.
센서는 `q_policy[i] = motor_state[map[i]].q`로 읽고,
출력은 `motor_cmd[map[i]].q = target_policy[i]`로 쓴다.
기본 자세와 action offset은 EDU 순서다. PD 배열과 FixStand의 자세 배열은 기존처럼
SDK 순서를 사용한다. 이름과 SDK ID의 대응은 기존 설정 및 `scene_a2.xml`의
actuator 순서로 교차 확인했으며, 실제 하드웨어에서 모터를 움직여 확인한 것은 아니다.

실제 관절을 하나씩 움직여 비교하려면 [real_joint_test 사용법](real_joint_test.md)을 따른다.
같은 `deploy.yaml` 매핑과 C++ Unitree SDK2를 사용하며,
EDU 순서로 예상 이름과 모터 ID를 표시한다. 실행 파일은 `build/real_joint_test`다.

## 관측과 제어

관측 항목은 각속도 3, 중력 방향 3, 속도 명령 3, 상대 관절 위치 12,
관절 속도 12, 이전 raw action 12다. 각 항목의 5프레임을 과거→현재 순으로 펼친 뒤
항목 순서대로 이어 붙여 225차원을 만든다. gait phase는 제거했다.
각 항목을 ±100으로 clip한 뒤 각속도는 0.2, 관절 속도는 0.05를 곱한다.
초기화할 때 현재 관측을 5번 채우고 이전 raw action은 0으로 만든다.

- 정책 주기: 0.02초(50 Hz). 기존 DDS/FSM 송신 주기 0.001초는 유지한다.
- 정책 제어 PD: 모든 관절 Kp 25, Kd 0.5.
- 목표: `clip(default_joint_pos + 0.25 * raw_action, -100, 100)`.
- 속도 명령 범위: EDU 학습 기록대로 vx ±0.6 m/s, vy ±0.4 m/s, wz ±1.0 rad/s.
- 조이스틱 축과 부호는 기존 그대로다. ly → vx, -lx → vy, -rx → wz이며 범위 밖 값은 clip한다.
- 기본 자세: `[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1, 1, -1.5, -1.5, -1.5, -1.5]`.

사용자가 선택한 대로 FixStand의 마지막 자세만 EDU 기본 자세의 SDK 순서로 변경했다.
중간 자세, 전환 시간, FixStand PD 200/5, Passive 감쇠, 버튼 조합은 유지했다.
기존에 꺼져 있던 자세 이상 검사는 사용자 선택에 따라 복구했다.
몸통 기울기가 1.0 rad를 넘거나 중력 방향에 비유한 값이 있으면 Passive로 전환한다.
기존 DDS timeout 전환을 유지하고, 정책 추론 오류도 Passive 전환 조건으로 처리한다.

정책 시작 직후 첫 목표가 0으로 발행되지 않도록 reset 시 offset을 적용한 목표를
동기적으로 준비한다. 추론 스레드는 목표 배열을 짧은 lock으로 교체하고,
1 kHz 제어 스레드는 이 배열을 읽는다. ONNX 추론 중에는 이 lock을 잡지 않는다.
조이스틱도 센서 갱신 때 복사해 서로 다른 스레드에서 읽고 쓰는 문제를 피했다.

SDK에 `KeyBase`가 없는 경우에도 기존 버튼 조합을 사용할 수 있도록 공통 버튼 상태를
읽는 호환 처리를 추가했다. `pressed_time`이 없는 SDK에서는 hold-time 표현을
명시적으로 거부한다. 현재 A2 버튼 설정은 hold-time을 사용하지 않는다.

## 빌드와 오프라인 검증

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2
ctest --test-dir deploy/robots/a2/build --output-on-failure
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v
```

`a2_policy_probe`는 실제 DDS 메시지 타입과 배포용 C++ 센서/관측/행동/ONNX 코드를
사용하되 DDS 채널을 만들지 않는다. 가짜 센서 프레임 16개를 입력해 Python actor와
비교한다. 5프레임 이력, 재진입 reset, raw action 이력, 관측 clipping,
목표 clipping, 모든 모터 ID의 출력값을 확인한다. 잘못된 관측 크기와 NaN 입력,
1.0 rad 자세 판정도 검사한다. 별도 C++ 테스트로 기존 버튼 조합을 확인한다.

현재 PC에서 컨트롤러 빌드, Python 테스트 12개(skip 없음), C++ 버튼 테스트 1개를 통과했다.
C++와 Python의 최대 관측 오차는 `4.91e-8`, raw action 오차는 `4.58e-5`,
모터 목표각 오차는 `1.15e-5 rad`였다. 결과는 `outputs/sim2real/validation.json`에 기록했다.

C++ probe를 빌드하지 않은 환경에서는 해당 Python 테스트 하나가 skip된다.
실기 배포 전 검증 결과를 확인할 때는 위 빌드 명령부터 실행해 skip 없이 확인한다.

## 실행 경로

실행 프로그램은 `deploy/robots/a2/build/a2_ctrl`이다. 설정은 실행 파일 위치를 기준으로
찾으므로 위 `build/` 위치를 유지한다. 다음은 사용자가 로봇에서 실행할 명령이며,
이번 작업 중에는 실행하지 않았다.

```bash
./deploy/robots/a2/build/a2_ctrl --network=YOUR_ROBOT_NIC
```

기존 버튼 전환은 L2 + 위 → FixStand, R2 + A → Velocity,
L2 + B → Passive다. 모델 경로는 `config/config.yaml`에서 `config/policy/velocity/v0`로
명시해 다른 디렉터리의 모델을 자동 선택하지 않도록 했다.

`simulate/`의 DDS 시뮬레이터에는 `--network=lo`로 연결할 수 있다.
다만 이번에는 C++ DDS 통합 주행이나 실기 보행을 검증하지 않았다.
Python `scripts/sim2sim.py`의 검증 결과와 C++ DDS 주행 검증은 별개다.
실제 모터 응답·토크 제한·제어 지연·보행 안정성은 이 오프라인 검증으로 확인되지 않는다.
