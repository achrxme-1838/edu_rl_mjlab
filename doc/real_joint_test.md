# A2 실기 관절 순서 확인

`deploy/robots/a2/build/real_joint_test`는 EDU 순서로 관절 하나씩 움직여, 터미널에 표시된
예상 관절과 실제 움직이는 관절을 비교하는 도구다. 정책 추론 없이
sim2real의 `deploy.yaml`에서 `joint_ids_map`, 기본 자세, PD 게인을 읽는다.
소스는 `deploy/robots/a2/real_joint_test.cpp`이며, `a2_ctrl`과 같은 C++ Unitree SDK2,
HG 메시지 및 SDK CRC 함수를 사용한다. Python SDK, Conda, ONNX Runtime은 필요 없다.
로봇의 좌우는 **로봇이 앞을 보는 기준**이다.

## 빌드와 실행

`a2_ctrl`을 빌드하는 C++ SDK2 / CycloneDDS / Boost / yaml-cpp 환경을 사용한다.
로봇 내부 PC에서 실행하려면 그 PC의 아키텍처에 맞춰 빌드한다.
관절 테스트 실행 파일만 빌드할 수 있다.

```bash
cd /home/lim/rl_ws/unitree_rl_mjlab
cmake -S deploy/robots/a2 -B deploy/robots/a2/build
cmake --build deploy/robots/a2/build --target real_joint_test -j2
```

실행 파일 위치를 기준으로 `../config/`에서 설정을 찾는다. 실행 디렉터리는 자유지만,
다른 PC로 옮길 때도 `a2/build/real_joint_test`와 `a2/config/` 구조를 유지한다.
별도 설정을 쓰려면 `--config`(정책 설정)와 `--fsm-config`(Passive 게인 설정)를 지정한다.

**로봇을 지지하거나 매단 상태에서 실행한다.** 이 도구에는 균형 제어가 없으며,
종료하면 위치 유지 게인을 0으로 바꾸므로 로봇이 주저앉을 수 있다.
기존 저수준/debug 모드에 진입한 뒤 `a2_ctrl` 등 다른 모터 명령 프로그램을 종료한다.
이 프로그램은 로봇 모드를 자동 전환하지 않는다.

```bash
./deploy/robots/a2/build/real_joint_test --network YOUR_ROBOT_NIC
```

`YOUR_ROBOT_NIC`를 로봇과 연결된 실제 Ethernet 인터페이스 이름으로 바꾼다.
예를 들어 `enp3s0`이며, 이름은 `ip -br link`로 확인할 수 있다.
연결된 A2의 상태를 받으면 추가 입력 없이 자세 전환이 시작된다.

1. 측정된 현재 관절각에서 **5초 동안 EDU 기본 자세로 이동**한다. Kp는 첫 1초에 걸쳐 증가한다.
2. EDU 0번 `left front hip`부터 **±0.05 rad(약 ±2.9°), 3초 주기**로 왕복한다.
   나머지 관절은 기본 자세를 유지한다. 예상 관절 이름, EDU 번호, SDK 모터 ID,
   목표각 및 측정각을 출력한다.
3. **Space**를 누르면 현재 관절을 1초 동안 기본 위치로 돌린 뒤 다음 관절로 넘어간다.
   Enter는 필요 없다. 자세 전환 중에는 Space를 무시한다.
4. 마지막 관절에서 Space를 누르면 기본 자세를 유지한다.
5. **Q / Esc / Ctrl-C**로 종료한다. 기존 Passive 설정의 Kp 0 / Kd 10 명령을
   약 0.1초 전송한 뒤 연결을 닫는다. 네트워크가 끊기면 이 명령의 수신은 보장되지 않는다.

처음 두 관절의 출력 예:

```text
[TEST 1/12] expected: left front hip (FL_hip_joint) | EDU joint 0 -> motor ID 3
[TEST 2/12] expected: right front hip (FR_hip_joint) | EDU joint 1 -> motor ID 0
```

기본 순서는 FL → FR → RL → RR의 hip 4개, thigh 4개, calf 4개다.
EDU → SDK 매핑은 `[3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8]`이며,
전체 표는 [sim2real 문서](sim2real.md#관절-순서)에 있다.
기본 자세는 EDU 순서 `[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1, 1, -1.5, -1.5, -1.5, -1.5]`다.
현재 PD는 Kp 25 / Kd 0.5이며, 명령 송신 목표 주기는 2 ms다.
일반 Linux 스케줄러를 사용하므로 실시간 주기를 보장하지 않는다.

LowState 미수신, `mode_machine != 1`, 비유한 관절각이면 움직임을 시작하지 않는다.
실행 중 상태가 0.2초 이상 갱신되지 않으면 감쇠 명령을 보내고 종료한다.
시작 전 최소 1초 동안 다른 `rt/lowcmd` 송신도 검사한다. 실행 이후 다른 제어기를
시작하는 경우까지 차단하는 장치는 아니므로 동시에 실행하지 않는다.

## 로봇 없이 미리 확인

```bash
# 같은 터미널 조작과 명령 순서를 확인한다. DDS 채널을 만들지 않는다.
./deploy/robots/a2/build/real_joint_test --dry-run

# 매핑 표만 출력하고 종료한다.
./deploy/robots/a2/build/real_joint_test --print-mapping
```

`--dry-run`의 측정각은 이전 명령을 그대로 따라가는 가상 값이다. 물리 시뮬레이션은 아니다.
`--amplitude 0.03 --period 4`로 움직임을 더 작고 느리게 바꿀 수 있다.
진폭은 최대 0.15 rad, 주기는 최소 2초, 초기 자세 전환은 최소 3초로 제한한다.
필요하면 `--order sdk`로 모터 ID 0부터 확인할 수 있지만 기본은 EDU 순서다.

## 검증 범위

```bash
cmake -S deploy/robots/a2 -B deploy/robots/a2/build -DBUILD_OFFLINE_TESTS=ON
cmake --build deploy/robots/a2/build -j2
ctest --test-dir deploy/robots/a2/build --output-on-failure
# 선택: Python SDK 없이 C++ 실행 파일의 터미널 조작을 자동 검증
python3 -m unittest discover -s tests -p test_real_joint_test.py -v
```

`deploy/robots/a2/tests/joint_test.cpp`는 12개 관절의 단독 명령, EDU 기본 자세와 매핑,
자세 보간, Space 전환, 센서 상태 검사, Passive 게인, HG 명령 및 CRC를 검사한다.
이전 SDK로 생성한 기준 패킷과 C++ 패킷의 CRC 일치도 확인한다.
`tests/test_real_joint_test.py`는 C++ 실행 파일을 실행해 매핑 표시, 잘못된 옵션 거부,
Space/Q/SIGINT 및 터미널 설정 복구를 확인한다. 이 테스트에도 Python SDK는 필요 없다.
실제 DDS 연결, 모터 움직임, 물리적인 관절 이름의 대응은 검증하지 않았다.
