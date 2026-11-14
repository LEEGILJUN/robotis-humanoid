# ROBOTIS Premium Humanoid 개발

ROBOTIS Premium 휴머노이드 Type A(AX-12A 18개)와 AX-18A 그리퍼 2개를 macOS에서 진단하고 개발하기 위한 프로젝트입니다.
개발 단계에서는 CM-530을 거치지 않고 USB2Dynamixel(또는 U2D2)로 DYNAMIXEL 버스에 직접 연결해서 Python으로 제어합니다.

## 준비물

- USB 변환기: USB2Dynamixel(보유) 또는 U2D2
- 전원 보드: SMPS2Dynamixel(보유) 또는 U2D2 Power Hub Board
- 12V DC 어댑터 (5A 이상, 12V를 넘기면 안 됩니다)

## 연결 방법 (USB2Dynamixel + SMPS2Dynamixel)

1. USB2Dynamixel 옆면의 스위치를 **TTL**로 맞춥니다.
2. CM-530의 DXL 포트에 꽂혀 있는 모터 케이블을 모두 뽑습니다.
3. 뽑은 체인들을 하나로 이어 줍니다. 한 체인의 케이블을 다른 체인 끝 모터의 빈 포트에 꽂거나, 3핀 허브를 씁니다.
4. 이어진 체인을 SMPS2Dynamixel의 TTL 포트 하나에 꽂고, 나머지 TTL 포트를 USB2Dynamixel의 TTL 포트와 연결합니다.
5. SMPS2Dynamixel에 12V 어댑터를 연결하고, USB2Dynamixel을 Mac에 꽂습니다. macOS 내장 FTDI 드라이버로 `/dev/cu.usbserial-*` 포트가 생깁니다.
6. CM-530의 배터리와 SMPS2Dynamixel을 같은 버스에 동시에 연결하지 않습니다.

## 설치

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 점검 순서

로봇을 바닥에 눕히거나 거치대에 매단 상태에서 진행합니다.

```bash
# 1. 버스 스캔: 응답하는 모터, 모델, 전압, 온도, 에러 비트를 확인합니다
.venv/bin/python -m tools.scan

# 2. 설정에 없는 ID(그리퍼 등)가 보이면 LED를 깜빡여 위치를 확인하고 config/robot.yaml에 추가합니다
.venv/bin/python -m tools.scan --blink 19 20

# 3. 모터별 동작 시험: 현재 위치 기준 ±10°를 토크 40%로 움직이며 추종 오차, 부하, 위치 센서 노이즈를 봅니다
.venv/bin/python -m tools.motor_test

# 4. 로봇 전체 시험: 손으로 자세를 만들어 기록하고, 재생하면서 관절별 부하를 기록합니다
.venv/bin/python -m tools.pose record stand
.venv/bin/python -m tools.pose play stand --duration 2 --hold 10
.venv/bin/python -m tools.pose monitor
```

아무 모터도 응답하지 않으면 보드레이트가 다를 수 있습니다. `--all-bauds --full` 로 전체를 스캔합니다.
결과는 `logs/`에 JSON으로 저장됩니다.

## 하드웨어 없이 실행

모든 도구는 `--port sim` 으로 시뮬레이터에서 실행할 수 있습니다. `--sim-faults` 를 붙이면
응답 없음, 기어 걸림(과부하), 과열, 위치 센서 노이즈, 토크 한계 0 상태를 일부러 넣어서 판정 결과를 미리 볼 수 있습니다.

```bash
.venv/bin/python -m tools.motor_test --port sim --sim-faults -y
.venv/bin/python -m pytest -q
```

## 구조

- `config/robot.yaml`: ID와 관절 매핑, 시험 기준값 (ID 매핑은 표준 BIOLOID 배치이며 실물 확인 전입니다)
- `humanoid/control_table.py`: AX/MX Protocol 1.0 컨트롤 테이블과 모델 정보
- `humanoid/bus.py`: 실제 버스(`DxlBus`)와 시뮬레이터(`SimBus`)
- `humanoid/status.py`: 모터 상태 읽기와 건강 판정
- `tools/`: `scan`, `motor_test`, `pose` 명령과 다음 보조 도구
  - `set_id`: 모터 ID 변경 (새 ID가 비어 있는지, 모델이 맞는지 확인한 뒤 변경)
  - `gripper_test`: Wheel 모드 그리퍼의 열림·닫힘 끝 찾기 (출력 비율로 구동, 멈춤 감지)
  - `horn_move`: 분리한 혼을 절대 위치로 이동 (약한 탐색 이동 후 본 이동, 걸림 시 즉시 정지)

## 현재 하드웨어 상태 (2026-11-15)

- 모터 20개 모두 정상. 1~18번은 표준 BIOLOID 배치와 일치합니다.
- 그리퍼는 공장 출하 ID 1이 겹쳐 있던 것을 19(오른쪽), 20(왼쪽)으로 바꿨습니다.
- 그리퍼는 위치 센서 사각 구간을 피하도록 혼을 180° 돌려 다시 조립했고, 관절 모드로 바꿨습니다.
- PC 연결은 CM-530을 MANAGE 모드로 켜 두고 남는 DXL 포트에 USB2Dynamixel을 꽂는 방식을 씁니다.
- macOS에서는 USB 지연 때문에 20개 모터를 한 바퀴 읽는 데 약 0.3초가 걸립니다.

## 주의 사항

- AX 시리즈는 Protocol 1.0에서 Bulk Read를 지원하지 않아서, 모터를 하나씩 읽습니다.
- MX-64는 공장 출하 시 ID 1, 57600bps이므로, AX 버스에 연결하기 전에 단독으로 연결해서 ID와 보드레이트를 바꿔야 합니다.
