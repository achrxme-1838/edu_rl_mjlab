# A2 sim2real: NX (a2) 실기 실행

**NX (a2)**는 A2 로봇 내장 PC다. Ubuntu와 ROS 2 Humble이 설치된 NX (a2)에서
아래 명령을 순서대로 실행한다. 작업 경로는 `~/control_ws`다.

## 1. 실행 환경

C++ Unitree SDK2와 ONNX Runtime으로 제어기를 실행한다.
필요한 빌드 도구는 CMake, C++ 컴파일러, Boost, yaml-cpp이며 ONNX Runtime은 저장소에 포함되어 있다.

```bash
uname -m  # aarch64 확인
mkdir -p "$HOME/control_ws"
```

## 2. unitree_rl_mjlab 설치

EDU A2 배포 코드가 포함된 저장소를 받는다. 폴더가 이미 있으면 clone을 생략한다.

```bash
git clone https://github.com/achrxme-1838/unitree_rl_mjlab.git \
  "$HOME/control_ws/unitree_rl_mjlab"
```

로컬에서 수정한 코드를 사용할 경우 해당 소스와 정책 파일도 NX (a2)에 반영한다.

## 3. Unitree SDK2 설치

SDK 소스를 `~/control_ws/unitree_sdk2`에 받고 `/usr/local`에 설치한다.
SDK 폴더가 이미 있으면 clone을 생략한다.

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git \
  "$HOME/control_ws/unitree_sdk2"

git -C "$HOME/control_ws/unitree_sdk2" checkout --detach \
  c8a71e281093593f4dcc7bceb3b3b529ff0e36b4

cmake -S "$HOME/control_ws/unitree_sdk2" \
  -B "$HOME/control_ws/unitree_sdk2/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_INSTALL_LIBDIR=lib \
  -DBUILD_EXAMPLES=OFF

cmake --build "$HOME/control_ws/unitree_sdk2/build" -j2
sudo cmake --install "$HOME/control_ws/unitree_sdk2/build"
sudo ldconfig
```

설치 확인:

```bash
ls /usr/local/lib/libunitree_sdk2.a
ls /usr/local/lib/libddsc.so* /usr/local/lib/libddscxx.so*
ls /usr/local/include/unitree/dds_wrapper/robots/g1/g1.h
```

## 4. 정책 경로 설정

개발 PC의 `edu_rl_lab`에서 `play`하면 체크포인트 폴더 아래
`exported/<체크포인트 이름>/policy.onnx`가 생성된다.
이 파일을 NX (a2)의 `unitree_rl_mjlab/networks/01_EDU_A2_STAIR/`에 복사해 사용한다.

[deploy/robots/a2/config/deploy.yaml](../deploy/robots/a2/config/deploy.yaml)의
`policy_dir`을 사용할 **폴더 경로**로 지정한다.

```yaml
policy_dir: networks/01_EDU_A2_STAIR/
```

상대 경로는 저장소 루트 기준이며 해당 폴더의 `policy.onnx`를 읽는다.
정책 파일이나 설정을 바꿀 때는 제어기를 종료하고, 수정 후 다시 실행한다.

## 5. C++ 빌드와 확인

NX (a2)에서 빌드한다.

```bash
cd "$HOME/control_ws/unitree_rl_mjlab"

cmake -S deploy/robots/a2 -B deploy/robots/a2/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2

ldd deploy/robots/a2/build/a2_ctrl
ldd deploy/robots/a2/build/real_joint_test
ctest --test-dir deploy/robots/a2/build --output-on-failure
./deploy/robots/a2/build/a2_ctrl --check-policy
```

`ldd`에 `not found`가 없어야 하며 CTest가 통과해야 한다.
정책 검사에서 `Policy check complete (no DDS channels opened).`가 출력되면 준비 완료다.
위 확인 명령은 모터를 구동하지 않는다.

## 6. 로봇과 네트워크 준비

A2를 저수준/debug 모드로 전환하고 다른 모터 제어 프로그램을 종료한다.
관절 테스트는 균형 제어가 없으므로 로봇을 지지하거나 매단 상태에서 진행한다.

NX (a2)의 로봇 제어망 Ethernet 인터페이스 이름을 확인한다.

```bash
ip -br link
ip -br addr
```

[deploy/robots/a2/config/config.yaml](../deploy/robots/a2/config/config.yaml)의
`network.interface`에 해당 이름을 저장한다. 아래 `eth0`는 예시이며 실제 이름으로 바꾼다.

```yaml
network:
  interface: "eth0"
```

`a2_ctrl`과 `real_joint_test` 모두 이 설정을 사용한다.
다른 인터페이스를 임시로 사용할 때만 `--network 인터페이스명`을 지정한다.
IP 주소·넷마스크는 Ubuntu 네트워크 설정에서 로봇 제어망에 맞게 설정한다.
이미 연결되어 있으면 기존 IP 설정을 사용한다.

## 7. real_joint_test 실기 실행

**연결되면 별도 시작 버튼 없이 관절이 움직인다.** `a2_ctrl`을 종료한 상태에서 실행한다.

```bash
cd "$HOME/control_ws/unitree_rl_mjlab"
./deploy/robots/a2/build/real_joint_test
```

1. 현재 측정 자세에서 5초 동안 EDU 기본 자세로 이동한다.
2. 관절 하나씩 ±0.05 rad, 3초 주기로 왕복한다.
3. 터미널의 예상 관절 이름·SDK 모터 ID와 실제 움직이는 관절을 비교한다.
4. **Space**: 현재 관절을 기본 위치로 돌린 뒤 다음 관절로 이동한다.
5. **Q / Esc / Ctrl-C**: Passive 감쇠를 약 0.1초 보내고 종료한다. 종료하면 위치 유지가 풀린다.

관절의 좌우는 로봇이 앞을 보는 기준이다. 예상과 다른 관절이 움직이면 매핑을 확인한다.
진폭과 주기를 바꾸는 예:

```bash
./deploy/robots/a2/build/real_joint_test --amplitude 0.03 --period 4
```

전체 옵션은 [관절 테스트 사용법](real_joint_test.md)을 참고한다.

## 8. a2_ctrl 정책 실행

### 실행

관절 테스트를 종료하고 조이스틱을 중립에 둔다.
**제어기 연결 후 2초 동안 기본 고정 자세로 자동 기립한다.**

```bash
cd "$HOME/control_ws/unitree_rl_mjlab"
./deploy/robots/a2/build/a2_ctrl --log
```

`Connected to robot.` 및 `FSM: Start FixStand`를 확인한다.
로그는 `deploy/robots/a2/log/log.txt`에 저장된다.

### 조이스틱

| 조작 | 동작 |
| --- | --- |
| **제어기 실행** | 2초 동안 기본 고정 자세로 자동 기립 |
| **Start** | 기본 고정 자세(FixStand) ↔ 정책 모드(Velocity) |
| **A** | 기본 자세보다 몸체가 약 5cm 낮은 고정 자세(LowStand) |
| LowStand에서 **Start** | 기본 자세로 복귀. 기립 완료 후 다시 누르면 정책 모드 |
| **R1 / L1** | 선속도 배율 증가 / 감소 |
| **R2 / L2** | 각속도 배율 증가 / 감소 |
| **R1 + R2** | 모든 상태에서 Passive 감쇠 후 제어기 종료. 속도 증가보다 우선 |

기본 자세로 이동하는 2초가 지난 뒤 Start를 눌러 정책 모드에 진입한다.
고정 자세는 정책 없이 PD 제어로 유지한다.
버튼은 누를 때마다 한 번 동작하며, 입력에는 SDK `UnitreeJoystick` 기본 처리를 사용한다.

| 스틱 | 명령 | 배율 0.5에서 범위 | 배율 1.0에서 범위 |
| --- | --- | --- | --- |
| 왼쪽 상하 | 전후 선속도 | ±0.3 m/s | ±0.6 m/s |
| 왼쪽 좌우 | 좌우 선속도 | ±0.2 m/s | ±0.4 m/s |
| 오른쪽 좌우 | 회전 각속도 | ±0.5 rad/s | ±1.0 rad/s |

속도 배율은 선속도·각속도 각각 **초기 0.5, 누를 때마다 ±0.1, 범위 0~1**이다.
상태를 전환해도 유지되며 프로그램을 다시 시작하면 초기화된다. 스틱 중립에서는 이동 명령이 0이다.

### 설정 변경

| 파일 | 수정 항목 |
| --- | --- |
| [deploy.yaml](../deploy/robots/a2/config/deploy.yaml) | `policy_dir`: 정책 폴더, `commands.base_velocity.speed_control`: 속도 배율 |
| [config.yaml](../deploy/robots/a2/config/config.yaml) | `network.interface`: 통신 인터페이스, `FSM.initial`: 시작 상태, `FixStand`·`LowStand`: 고정 자세와 이동 시간 |

속도 배율 설정:

```yaml
commands:
  base_velocity:
    speed_control:
      linear: {initial: 0.5, step: 0.1, min: 0.0, max: 1.0}
      angular: {initial: 0.5, step: 0.1, min: 0.0, max: 1.0}
```

고정 자세의 `ts: [0, 2]`는 이동 시간, `qs` 마지막 배열은 목표 관절각(rad), `kp`·`kd`는 게인이다.
`qs` 첫 항목 `[]`는 진입 시 측정 자세다. 배열 순서는 SDK 모터 순서인
FR → FL → RR → RL, 각 다리의 hip/thigh/calf다.

### 정지와 종료

정책을 멈추고 자세를 유지하려면 **Start** 또는 **A**를 누른다.
제어기를 종료하려면 **R1 + R2**를 함께 누르고 `Controller stopped.`를 확인한다.
종료 시 Passive 감쇠(Kp 0 / Kd 10)를 약 0.1초 전송하며 위치 유지가 풀린다.
`a2_ctrl`의 Ctrl-C에는 별도 감쇠 처리가 없으므로 R1+R2로 종료한다.
