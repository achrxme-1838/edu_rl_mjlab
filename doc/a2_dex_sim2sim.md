# Isaac A2 checkpoint → mjlab / MuJoCo

`DEX_RL_LAB/scripts/logs/rsl_rl/a2_base_rl/01_STAIR_SUCCESS/model_10000.pt`를
재학습·변환 없이 읽어 실행하는 **추론 전용** 설정이다.
DEX_RL_LAB과 기존 mjlab 소스/config/XML은 수정하지 않았다.

## 실행

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab

# 로컬 MuJoCo 창. 기본 속도: vx=0.4 m/s, vy=0, yaw rate=0.
bash scripts/play_a2_dex.sh --terrain flat --viewer native
bash scripts/play_a2_dex.sh --terrain stairs --viewer native

# SSH/화면 없는 세션: 터미널에 표시되는 Viser 주소 사용
bash scripts/play_a2_dex.sh --terrain stairs --viewer viser

# 속도 명령과 계단 높이/폭 변경
bash scripts/play_a2_dex.sh --terrain stairs --command 0.4 0 0 \
  --stair-height 0.10 --stair-width 0.30 --stair-count 5

# 유한 길이 headless 테스트와 영상 저장 (50 control steps = 1초)
bash scripts/play_a2_dex.sh --terrain flat --headless --steps 1000 \
  --video outputs/a2_dex/flat.mp4
bash scripts/play_a2_dex.sh --terrain stairs --headless --steps 2000 \
  --video outputs/a2_dex/stairs.mp4
```

`--checkpoint /absolute/path/model_N.pt`로 다른 동일 구조 체크포인트를 선택한다.
모델 구조·관측 정규화가 다른 체크포인트는 별도 대응이 필요하다.
쉘 스크립트는 현재 활성화된 conda의 Python을 사용하고,
이 머신의 ROS/Isaac `PYTHONPATH` 및 사용자 site-packages가 끼어들지 않게 한다.
`--viewer auto`는 DISPLAY가 있으면 native, 없으면 Viser를 사용한다.
Native: Enter 리셋, Space 일시정지, `-`/`=` 배속 조정.
Viser에서는 명령 항목의 조이스틱 UI를 사용할 수 있다.

첫 실행은 Warp 커널 컴파일 때문에 느릴 수 있다. CPU와 CUDA는 별도 캐시를 사용한다.
출력은 `unitree_rl_mjlab/outputs/a2_dex/`에 저장하며 체크포인트 폴더에 쓰지 않는다.
자동 리셋을 끄므로 넘어져도 실패가 가려지지 않는다. Headless 보고서의
`fall_detected`는 기울기/높이 기반 진단이며 지형 성공률 평가 지표는 아니다.

## CPU 모드

같은 `mjlab` conda 환경에서 `--cpu`를 추가한다 (`--device cpu`와 동일).
정책 추론과 MuJoCo Warp 물리 계산을 모두 CPU에서 수행하며,
해당 실행 프로세스에서는 CUDA 장치를 숨긴다. GPU 기본 실행은 기존과 같다.
조인트 순서, 관측 이력, PD, 물성, 시간 간격은 CPU에서도 같은 설정을 사용한다.

```bash
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab

bash scripts/play_a2_dex.sh --cpu --terrain flat
bash scripts/play_a2_dex.sh --cpu --terrain stairs

# 원격 환경에서는 Viser 사용
bash scripts/play_a2_dex.sh --cpu --terrain stairs --viewer viser

# GPU/화면 없이 수치 테스트
bash scripts/play_a2_dex.sh --cpu --terrain flat --headless --steps 1000
bash scripts/play_a2_dex.sh --cpu --terrain stairs --headless --steps 2000

# 동일한 관측/액션 검증을 GPU 없이 실행
env -u PYTHONPATH PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' \
  A2_DEX_TEST_DEVICE=cpu MUJOCO_GL=egl \
  python -m unittest discover -s tests -p test_a2_dex.py -v
```

CPU 보고서는 `outputs/a2_dex/flat_cpu_report.json`, `stairs_cpu_report.json`으로
분리된다. `physics_device`, `policy_device`가 모두 `cpu`인지 확인할 수 있으며,
`wall_seconds`, `realtime_factor`에는 실제 실행 시간과 시뮬레이션 배속이 기록된다.
`--cpu`와 `--device`는 둘 중 하나만 지정한다.
Native 창/MP4 렌더링은 별도의 OpenGL 환경을 사용한다. 그래픽 장치가 없는
환경에서는 Viser 또는 영상 없는 headless 실행을 사용한다.
이 버전의 Warp는 CUDA를 숨기면 초기화 중 `no CUDA-capable device is detected`
로그를 출력하지만 CPU 실행은 계속된다. 보고서의 두 device 항목으로 실행 장치를 확인한다.

CPU 검증 결과(2026-11-01, 커널 컴파일 이후, 영상 없음): 평지 20초는 실제 10.96초,
10 cm 계단 40초는 실제 30.08초에 계산됐으며 두 실행 모두 넘어짐이 없었다.
CPU 관측/액션 검증 3개와 CPU Viser 25 step도 통과했다.
GPU 기본 실행 및 `--device cpu` 별칭도 별도로 확인했다.

## 내 설정을 바꾸는 위치

| 목적 | 수정 위치 |
|---|---|
| 속도, PD, 초기 자세, 액션/관측 스케일, 시간 간격, 계단 크기 | `src/custom/a2_dex/env_cfg.py`의 `A2DexConfig` |
| 관측 항목·순서, 로봇 설정, 이벤트 | 같은 파일의 `make_env_cfg()` |
| 계단 형태 | 같은 파일의 `add_stairs()` |
| 체크포인트 로더/네트워크 구조 | `src/custom/a2_dex/policy.py`의 `IsaacActor` |
| 기본 체크포인트 경로, 실행·보고서 옵션 | `scripts/play_a2_dex.py` |

실행 스크립트가 `make_env_cfg(A2DexConfig(...))`를 직접 호출한다.
**위 전용 스크립트로 실행해야 이 설정이 적용된다.** CLI 인자는
`A2DexConfig`의 기본값 중 명시한 항목만 덮어쓴다.
`scripts/play.py Unitree-A2-Flat`은 계속 기존 기본 설정을 사용한다.
기존 task registry, `src/tasks/velocity/config/a2/`,
`src/assets/robots/unitree_a2/a2_constants.py`, `a2.xml`은 변경하지 않았다.
이 설정을 기존 학습 task로 등록하지 않았으므로 `scripts/train.py`에서는 사용하지 않는다.

## 조인트 순서

실제 학습 USD를 Isaac/PhysX에 로드해 확인한 정책 순서:

```text
FL_hip, FR_hip, RL_hip, RR_hip,
FL_thigh, FR_thigh, RL_thigh, RR_thigh,
FL_calf, FR_calf, RL_calf, RR_calf
```

각 이름 뒤에 `_joint`가 붙는다. MuJoCo 모델의 내부 순서는:

```text
FL_hip, FL_thigh, FL_calf,
FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf,
RR_hip, RR_thigh, RR_calf
```

정책 순서로 MuJoCo joint 배열을 읽는 인덱스는
`[0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]`이다.
MuJoCo 내부 배열 순서로 policy action을 나열하면
`[0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]`이다.

XML이나 체크포인트의 순서를 바꾼 것이 아니다. 관측은 이름으로 정책 순서대로
읽고, `PolicyJointPositionAction`이 같은 정책 순서의 각 액션을 해당 물리 조인트에 쓴다.
`last_action`은 정책 출력 순서를 유지한다. 축 부호 반전은 없다.
mjlab 1.2 기본 JOINT 액션은 `preserve_order`를 무시하고 natural order를 사용하므로
이 전용 클래스에서 `_find_targets()`만 이름 기반으로 재정의했다.

## 학습과 맞춘 내용 / 남아 있는 차이

| 항목 | 커스텀 설정 |
|---|---|
| 입력 | 각속도 3, 중력 3, 명령 3, 관절 상대위치 12, 상대속도 12, 이전 액션 12 |
| 이력 | 항목마다 5시점, 과거→현재, 총 225차원 (항목별로 이력을 먼저 펼침) |
| 관측 스케일 | 각속도 0.2, 관절속도 0.05, 그 외 1 |
| 관측 clipping | 각 항목 ±100을 **스케일 전에** 적용 |
| 정책 | ELU MLP 225→512→256→128→12, 정규화 없음, 결정론적 actor 출력 |
| 목표 관절각 | 초기 관절각 + 0.25 × raw action, 결과 각도를 ±100 rad로 clip |
| PD / DC motor | Kp=25, Kd=0.5, continuous/stall torque=120 Nm, no-load speed=22 rad/s |
| 모터 armature / frictionloss | 0 / 0 (학습 actuator 기준; 기본 mjlab armature 0.03과 다름) |
| 시간 | 물리 0.005초, decimation 4, 정책 50 Hz |
| 초기 자세 | z=0.45, 왼 hip +0.1, 오른 hip −0.1, 앞 thigh 0.8, 뒤 thigh 1.0, calf −1.5 |

5시점 이력의 리셋 직후 빈 자리는 첫 프레임으로 채운다. Actor는 height scan이나
phase를 입력받지 않는다. Critic과 optimizer는 추론에 사용하지 않는다.
최근 RSL-RL 형식으로 가중치를 억지 변환하지 않고 원래 actor 가중치를 그대로 읽는다.

사용자 선택에 따라 **mjlab A2의 질량·관성·형상·접촉 설정을 유지**한다.
몸통 기본 질량은 Isaac USD 20.651 kg, mjlab 19.651 kg이다.
학습 시 질량·마찰 randomization도 있었으므로 여기서 같은 동역학이 보장되지는 않는다.
모터 제어는 위의 학습 설정을 사용한다. 테스트에서는 관측 노이즈,
외부 push, 랜덤 초기화 및 terrain curriculum을 사용하지 않는다.

계단은 앞쪽 x=1.5 m부터 시작하는 직선 5단 상승/중앙 평탄부/하강 코스다.
기본 높이 0.10 m, 디딤판 0.30 m, 너비 3 m, 중앙 추가 길이 2 m이다.
학습의 랜덤 pyramid terrain grid를 재현하는 것은 아니다.

## 환경과 검증

새 conda 환경 `mjlab`을 만들었다. Python 3.11, torch 2.7.1+cu126,
mjlab 1.2.0, mujoco-warp 3.5.0, MuJoCo 3.5.0, Warp 1.12.0을 사용한다.
이 머신의 드라이버 555.42.02에서 CUDA 실행을 확인했다.
환경을 다시 만들 경우:

```bash
conda create -y -n mjlab python=3.11 pip
conda activate mjlab
cd /home/lim/rl_ws/unitree_rl_mjlab
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m pip install torch==2.7.1 \
  --index-url https://download.pytorch.org/whl/cu126
env -u PYTHONPATH PYTHONNOUSERSITE=1 python -m pip install -e . scipy \
  -c src/custom/a2_dex/constraints.txt
```

`scipy`는 mjlab terrain 모듈이 사용하지만 설치 의존성에서 빠져 있어 별도 설치했다.
CUDA 12 Warp 패키지의 드라이버 조건은
[NVIDIA 설치 문서](https://nvidia.github.io/warp/v1.12/user_guide/installation.html)를 참고했다.
이번 결과는 실제 로컬 CUDA 실행으로도 검증했다.

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 MUJOCO_GL=egl \
  python -m unittest discover -s tests -p test_a2_dex.py -v
```

세 검사는 actor 수치 일치, 원본 모델 질량/관성/접촉값 유지 및 기본 설정 비변경,
조인트 매핑/초기각/목표각/관측 이력/리셋/clipping을 검증한다.
실행 보고서는 `outputs/a2_dex/flat_report.json`, `stairs_report.json`에,
영상은 같은 폴더의 `flat.mp4`, `stairs.mp4`에 저장된다.

2026-11-01 최종 영상 실행(`vx=0.4`, 단일 환경):

- 평지 20초: x 방향 6.28 m 이동, 넘어짐 없음.
- 10 cm 계단 40초: 상승·하강 후 x=11.52 m까지 이동, 넘어짐 없음.
- 같은 설정으로 추가 headless 40초: x=11.95 m, 넘어짐 없음
  (`outputs/a2_dex/stairs_repeat_report.json`).
- 검증 3개 통과, Viser 서버/시뮬레이션 25 step 실행 통과.
- 앞선 검증 실행에서는 계단 하강 끝에서 넘어짐도 관찰했다. 위 영상 한 번의
  통과를 반복 성공률로 해석하면 안 된다. 모델 물성이나 정책을 추가 튜닝하지 않았다.
- Native 데스크톱 창은 이 화면 없는 세션에서 직접 확인하지 않았다.

재현용 패키지 목록은 `outputs/a2_dex/pip-freeze.txt`, Isaac에서 읽은 조인트/물성
참고값은 `outputs/a2_dex/isaac_reference.json`에 있다.
