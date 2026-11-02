# EDU A2 → mjlab sim2sim

`edu_rl_lab`의 `Edu-A2-Velocity-v0`에서 학습한 정책을 이 저장소의 A2 모델로 실행한다.
기본 체크포인트는 `policies/01_EDU_A2_STAIR/model_10001.pt`다.
학습 실행 `2026-11-01_22-55-17_105020`의 원본을 그대로 복사했으며 SHA-256은
`4c2109d1ba39437927f3f22dfa8f9ee8ff79093bd062a32acf9e1aec315ec1f3`이다.

같은 폴더에 학습 당시 `params/agent.yaml`, `params/env.yaml`, 출처 기록
`provenance.json`, EDU USD를 Isaac Sim에서 열어 확인한 관절/기본 자세/질량
`source_model.json`이 있다. YAML의 원래 USD 경로는 기록용이며 실행에는 쓰지 않는다.
실행 시 Isaac Sim이나 다른 학습 저장소를 참조하지 않는다.

## 실행

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab

# 10 cm × 5단 상승 → 2 m 플랫폼 → 하강, 전진 명령 0.4 m/s
bash scripts/play_a2_edu.sh

# 평지
bash scripts/play_a2_edu.sh --terrain flat

# 웹 뷰어: Base velocity 폴더에서 Enable을 켜고 속도 슬라이더 조절
bash scripts/play_a2_edu.sh --viewer viser

# 별도 체크포인트 또는 다른 속도
bash scripts/play_a2_edu.sh --terrain flat --command 0.3 0 0 \
  --checkpoint policies/01_EDU_A2_STAIR/model_10001.pt
```

데스크톱에서는 기본적으로 native 뷰어가 열린다. Space는 일시정지, Enter는 초기화다.
GUI 실행은 창을 닫을 때까지 계속되며 `--steps`는 headless 실행에만 적용된다.
쉘 스크립트는 Isaac 쪽 `PYTHONPATH`와 사용자 site-packages 혼입을 방지한다.
Python 스크립트를 직접 실행하려면 앞에 `env -u PYTHONPATH PYTHONNOUSERSITE=1`을 붙인다.

검증한 환경: Python 3.11, torch 2.7.1+cu126, mjlab 1.2.0, MuJoCo 3.5.0,
mujoco-warp 3.5.0, warp-lang 1.12.0, rsl-rl-lib 5.0.1.
영상 저장에는 imageio와 imageio-ffmpeg를 사용한다.

## 이식 계약

Actor는 정규화 없이 `225 → 512 → 256 → 128 → 12` ELU MLP를 실행한다.
학습 시 critic, 보상, PPO optimizer는 inference에 필요하지 않다.

| 관측 항목 | 프레임당 차원 | 스케일 |
| --- | ---: | ---: |
| 몸체 좌표계 각속도 | 3 | 0.2 |
| 몸체 좌표계 중력 방향 | 3 | 1 |
| 속도 명령 vx, vy, wz | 3 | 1 |
| 기본 자세 대비 관절 위치 | 12 | 1 |
| 관절 속도 | 12 | 0.05 |
| 이전 raw action | 12 | 1 |

각 항목을 ±100으로 clip한 후 scale한다. 항목별 과거→현재 5프레임을 먼저
펼치고 위 표 순서대로 이어 붙인다. 전체 45차원 프레임을 5번 이어 붙이는 형식과 다르다.
초기화 시 첫 관측을 5번 채우고 이전 action은 0이 된다. 뷰어에서 관측을 읽어도
이력이 추가되지 않으며 제어 step마다 한 번씩 갱신된다. 평가 중 관측 노이즈,
외력, 질량/마찰 무작위화는 비활성화되어 있다.

정책의 관절 순서는 `FL, FR, RL, RR`의 hip 4개 → thigh 4개 → calf 4개다.
mjlab의 leg별 관절 순서에 대응하는 인덱스는
`[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`이다.
관측과 action 양쪽에 동일하게 적용한다.

- 기본 자세: `[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1, 1, -1.5, -1.5, -1.5, -1.5]`
- 위치 목표: `clip(default + 0.25 × raw_action, -100, 100)`
- PD: Kp 25, Kd 0.5; DC 모터 effort/saturation 120 Nm, 속도 22 rad/s
- 모터 armature와 frictionloss: 0
- 물리 dt: 0.005초, decimation: 4, 정책 주기: 50 Hz
- 초기 몸통 높이: 0.45 m

사용자가 선택한 대로 mjlab A2의 몸체 질량·관성·충돌 형상·접촉 설정은 유지한다.
예를 들어 몸통 질량은 mjlab 19.651 kg, EDU USD 20.651 kg으로 다르다.
기본 A2 자산과 기존 학습 task는 수정하지 않으며, 이식 설정은 `src/custom/a2_edu`에 있다.

## 검증 및 결과

2026-11-02, RTX 4090, seed 42, 명령 `(0.4, 0, 0)`으로 측정했다.
자동 리셋 없이 전 구간을 실행했다. CUDA 접촉 계산에 따른 수치 차이로 반복 시
궤적과 실패 시점이 조금 달라질 수 있다.

| 테스트 | 결과 |
| --- | --- |
| 계약 검증 7개 | 통과: 체크포인트/설정, actor 수치 출력, 관절 매핑·clip, 5프레임 이력·reset, 모터 토크 곡선, 모델 물성, 계단 형상 |
| 평지 20초 | 통과, x 방향 6.82 m 이동, 평균 몸체 전진 속도 0.355 m/s |
| 10 cm × 5단 계단, 40초 | 상승·하강 완주, 전도·리셋 없음, 평균 몸체 전진 속도 0.338 m/s |
| 15 cm × 5단 계단 | 실패: 상승 중 측면 이탈 후 약 12–13초에 전도 |
| 0.2초의 미완주 코스 | `--check`가 종료 코드 1을 반환하는 것 확인 |

평지에서도 회전 편향이 있어 20초 뒤 y가 약 -1.06 m였다. 고정 속도 명령은
세계 좌표계 경로를 추종하는 제어기가 아니다. 10 cm 통과가 학습 지형 전체나
15 cm 이상 계단의 성공을 보장하지 않는다.

```bash
# 관절/관측/가중치/모터/물성 검증
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m unittest discover -s tests -v

# 평지 20초
bash scripts/play_a2_edu.sh --terrain flat --headless --steps 1000 --check \
  --report outputs/a2_edu/flat/report.json

# 계단 40초 + 영상
bash scripts/play_a2_edu.sh --headless --steps 2000 --check \
  --report outputs/a2_edu/stairs_10cm/report.json \
  --video outputs/a2_edu/stairs_10cm/rollout.mp4

# 확인된 실패 조건 재현
bash scripts/play_a2_edu.sh --headless --stair-height 0.15 --steps 2000 --check \
  --report outputs/a2_edu/stairs_15cm/report.json \
  --video outputs/a2_edu/stairs_15cm/rollout.mp4
```

`--check`는 실패하면 종료 코드 1을 반환한다. 공통 조건은 유한한 상태/관측/action,
리셋 없음, 전도 없음이다. 전도는 upright cosine < 0.5 또는 세계 좌표계 몸통 높이
< 0.18 m로 판단한다. 평지는 처음 2초를 제외한 평균 속도 오차도 검사한다
(평면 속도 오차 ≤ max(0.2 m/s, 명령 크기의 50%), yaw 오차 ≤ 0.3 + 명령 크기의 50%).
계단은 폭 안에서 플랫폼에 도달하고 하강 끝에서 0.7 m 이상 전진한 뒤 몸통 높이가
0.65 m 미만이어야 완주로 판정한다. 짧게 실행해 계단에 도달하지 못하면 실패다.

`--report`를 생략하면 `outputs/a2_edu/<timestamp>/report.json`에 저장한다.
`--device cpu`도 지원한다. `outputs/`의 보고서와 영상은 Git에서 제외된다.
