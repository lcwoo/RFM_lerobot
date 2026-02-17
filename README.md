# RFM (Robot Foundation Models)

Gello로 UR5를 움직여 **데이터셋을 수집**하고, **Parquet(LeRobot 포맷)으로 변환**할 수 있으며, 학습한 **체크포인트로 UR5를 움직이게** 할 수 있는 레포입니다.

- **데이터 수집**: Gello로 UR5를 텔레오퍼레이션하며 데모를 기록
- **변환**: Gello pkl → LeRobot Parquet (`scripts/data/convert_to_lerobot.py`)
- **정책 실행**: 학습된 SmolVLA/Octo 체크포인트로 UR5 제어 — **`rfm.policies.runner`** 가 **`rfm.robots.ur5_bridge`** 와 ROS2 토픽으로 통신해 로봇을 움직입니다.

---

## 프로젝트 구조

```
RFM/
├── rfm/                    # 메인 Python 패키지
│   ├── robots/            # 로봇 인터페이스 (UR5 브리지 등)
│   ├── policies/          # 정책 실행 (SmolVLA/Octo)
│   └── utils/            # 유틸리티 (수학 함수 등)
├── scripts/               # 실행 스크립트
│   ├── data/             # 데이터 수집/변환
│   └── tools/            # 도구/디버그
├── nodes/                 # ROS2 노드
├── launch/                # ROS2 launch 파일
├── configs/               # 설정 파일
├── docs/                  # 문서 (quickstart.md 등)
├── tests/                 # 테스트
├── gello_software/        # Gello 서브모듈
├── lerobot/               # LeRobot 서브모듈
├── octo/                  # Octo 서브모듈
└── datasets/              # 데이터셋
```

자세한 구조는 `PROJECT_STRUCTURE.md` 참고.

---

## 주요 변경 이력

- **프로젝트 구조 재구성** (최신)
  - 계층적 패키지 구조로 재구성: `rfm/robots/`, `rfm/policies/`, `rfm/utils/`
  - 실행 스크립트 정리: `scripts/data/`, `scripts/tools/`
  - ROS2 노드 분리: `nodes/`
  - 문서 정리: `docs/quickstart.md`
  - `setup.py` 추가: 패키지 설치 및 CLI 명령 지원

- **Gello 캘리브레이션 개선**
  - 로봇 고정 초기 자세로 이동 후 Gello 자동 캘리브레이션
  - `dynamixel.py`: 정확한 오프셋 보정 (2π 래핑 → 실제 값 매핑)
  - FTA2U0V8 포트 설정 추가

- **UR5 정책 실행 및 브리지**
  - **`rfm.policies.runner`**: SmolVLA/Octo 체크포인트로 UR5 정책 실행. **bridge(`rfm.robots.ur5_bridge`)와 통신**해 `/ur5/goal_joint`, `/ur5/gripper_cmd` 로 명령을 보내고 `/ur5/status` 로 상태를 확인합니다.  
  - **`rfm.robots.ur5_bridge`**: UR5 RTDE와 ROS2 사이 브릿지. `goal_joint`(절대 관절값), `gripper_cmd`, `cmd`(go/list/save 등) 구독 → moveJ/그리퍼 제어, `status`(IDLE/MOVING) 퍼블리시.  
  - **`scripts/data/convert_to_lerobot.py`**: Gello pkl → LeRobot Parquet 변환.

---

## rfm.policies.runner ↔ rfm.robots.ur5_bridge 통신 (자세히)

두 노드는 **ROS2 토픽/서비스**로만 통신합니다. RTDE(로봇 제어)는 bridge만 사용하고, `rfm.policies.runner`는 **명령 전송**과 **상태 확인**만 합니다.

### 역할 분담

| 쪽 | 역할 |
|----|------|
| **rfm.robots.ur5_bridge** | UR5와 RTDE 연결(rtde_control, rtde_receive, rtde_io/ RobotiqGripper). 토픽으로 받은 명령을 moveJ/그리퍼로 실행하고, 상태를 `/ur5/status`로 퍼블리시. |
| **rfm.policies.runner** | 카메라·모델 추론 → action(관절 목표 + 그리퍼) → bridge가 구독하는 토픽으로 전송. `/ur5/status`를 구독해 **IDLE일 때만** 다음 관절 명령 전송. 관절 **현재값**은 rtde_receive로 **직접** 읽음(정책 입력용). |

### 토픽·서비스 정리

| 토픽/서비스 | 메시지 타입 | 방향 | 설명 |
|-------------|-------------|------|------|
| `/ur5/goal_joint` | `sensor_msgs/JointState` | run_policy → bridge | **절대 관절 목표(rad)**. `msg.position`에 **6개 값** (joint 0~5). bridge는 수신 시 `rtde_c.moveJ(q)` 실행. |
| `/ur5/gripper_cmd` | `std_msgs/Float64` | run_policy → bridge | 그리퍼 명령. bridge는 `value > gripper_mid`면 열림(True), 아니면 닫힘(False). RTDE Tool DO 0 또는 RobotiqGripper로 출력. |
| `/ur5/cmd` | `std_msgs/String` | run_policy → bridge | 문자열 명령. `run_policy_ur5.py`는 **시작 포즈** 이동 시 `"go <이름>"` 퍼블리시. bridge는 `go <name>`이면 `ur5_saved_poses.json`의 해당 관절값으로 moveJ. (기타: `where` / `list` / `save <name>`) |
| `/ur5/status` | `std_msgs/String` | bridge → run_policy | **"IDLE"** 또는 **"MOVING"**. bridge가 **명령 수신 직후** "MOVING"으로 바꾸고, moveJ 스레드 **완료 시** "IDLE"로 바꿔서 퍼블리시. 짧은 동작도 최소 한 번 MOVING → IDLE 전환이 있음. |
| `/ur5/stop` | `std_srvs/Trigger` (서비스) | run_policy → bridge | 긴급 정지. run_policy는 Ctrl+C 등으로 종료 시 이 서비스를 호출. bridge는 `rtde_c.stopL(0.5)` 등으로 동작 정지. |

### 제어 흐름 (한 스텝)

1. **rfm.policies.runner** 제어 루프(예: 30 Hz):
   - `robot.spin_once()` → `/ur5/status` 콜백으로 최신 상태 갱신.
   - **`robot.is_idle`가 False**(= status가 "MOVING")이면 이번 스텝은 **스킵** (이전 moveJ 완료 대기).
   - **IDLE이면**: 카메라 캡처 + **rtde_receive**로 현재 관절값 읽기 → 모델 추론 → `joint_target`(6개), `gripper_cmd` 계산.
   - **`/ur5/goal_joint`** 에 `JointState(position=joint_target)` 퍼블리시.
   - **`/ur5/gripper_cmd`** 에 `Float64(gripper_cmd)` 퍼블리시.

2. **rfm.robots.ur5_bridge**:
   - `/ur5/goal_joint` 수신 시 `_on_joint_abs()` 호출 → `_start_motion()`으로 **별도 스레드**에서 `rtde_c.moveJ(q)` 실행. 즉시 **status = "MOVING"** 퍼블리시.
   - moveJ가 끝나면 **status = "IDLE"** 퍼블리시.
   - `/ur5/gripper_cmd` 수신 시 `_on_gripper_cmd()`에서 RTDE IO 또는 RobotiqGripper로 그리퍼 제어(비동기, status와 무관).

3. **관절 현재값**: runner는 **bridge를 거치지 않고** `rtde_receive.RTDEReceiveInterface(robot_ip)`로 직접 `getActualQ()`를 호출합니다. (읽기 전용은 다중 연결 가능하므로 bridge와 충돌 없음.)

### 메시지 형식 요약

- **`/ur5/goal_joint`**: `JointState.position` = 길이 6 이상 리스트, 단위 **rad**. 앞 6개만 사용.
- **`/ur5/gripper_cmd`**: `Float64.data` = 한 개 실수. bridge는 `gripper_mid`와 비교해 열림/닫힘으로 변환; RobotiqGripper 사용 시 `gripper_min_hw`~`gripper_max_hw`로 스케일 후 0~255로 보냄.
- **`/ur5/cmd`**: `String.data` = `"go observe"` 등. 공백 한 칸 뒤에 포즈 이름.
- **`/ur5/status`**: `String.data` = `"IDLE"` 또는 `"MOVING"` (따옴표 제외 문자열).

---

## 설치 및 실행

### 설치

```bash
cd /home/lcw/RFM
pip install -e .
```

또는 개발 모드로 사용 (코드 수정 시 자동 반영).

### UR5 정책 실행

**방법 1: 모듈로 실행 (권장)**
```bash
# 터미널 1: bridge 실행
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
python -m rfm.robots.ur5_bridge

# 터미널 2: 정책 실행 (SmolVLA 예)
python -m rfm.policies.runner --model-type smolvla --checkpoint outputs/train/eggplant/checkpoints/020000/pretrained_model

# Octo 예
python -m rfm.policies.runner --model-type octo --checkpoint <체크포인트경로> --task "Pick up the eggplant and place it on the plate."
```

**방법 2: CLI 명령 (pip install -e . 후)**
```bash
rfm-ur5-bridge
rfm-run-policy --model-type smolvla --checkpoint ...
```

정책 실행 전에 **반드시 bridge가 실행 중**이어야 합니다.

---

## 프로젝트 개요 (전체 워크플로우)

RFM 프로젝트는 **3단계 워크플로우**로 구성되어 있습니다:

### 1단계: 데이터 수집 및 변환 (Data Collection & Conversion)

```
Gello Hand Controller (텔레오퍼레이션)
    ↓
Gello Software (gello_software/run_env.py)
    ↓
Gello PKL Files (Raw Data)
    ↓
scripts/data/convert_to_lerobot.py
    ↓
LeRobot Parquet Dataset (Standardized)
```

- **입력**: Gello Hand Controller를 통한 텔레오퍼레이션 및 UR5 물리 로봇으로부터의 데이터 수집
- **처리**: `gello_software/run_env.py`를 통해 데모 녹화 및 데이터 수집
- **변환**: `scripts/data/convert_to_lerobot.py`가 Gello PKL 파일을 LeRobot Parquet 형식으로 변환
- **출력**: 표준화된 LeRobot Parquet 데이터셋

### 2단계: 모델 학습 (외부) (Model Training - External)

```
LeRobot Parquet Dataset
    ↓
SmolVLA / Octo Model Training (External System)
    ↓
Trained Checkpoint (SmolVLA/Octo)
```

- **입력**: 1단계에서 생성된 LeRobot Parquet 데이터셋
- **처리**: 외부 시스템에서 SmolVLA 또는 Octo 모델 학습 수행
- **출력**: 학습된 체크포인트 파일

### 3단계: 정책 실행 (주요 초점) (Policy Execution - Main Focus)

```
Trained Checkpoint
    ↓
rfm.policies.runner (정책 실행)
    ↕ (ROS2 토픽 통신)
rfm.robots.ur5_bridge (UR5 브리지)
    ↕ (RTDE)
UR5 Robot (Physical) - Policy Execution
```

**통신 흐름:**

- **`rfm.policies.runner` → `rfm.robots.ur5_bridge`**:
  - `/ur5/goal_joint` (JointState): 절대 관절 목표값
  - `/ur5/gripper_cmd` (Float64): 그리퍼 명령
  - `/ur5/cmd` (String): 문자열 명령 (예: 'go observe')
  - `/ur5/stop` (Trigger Service): 긴급 정지

- **`rfm.robots.ur5_bridge` → `rfm.policies.runner`**:
  - `/ur5/status` (String): "IDLE" 또는 "MOVING" 상태

- **`rfm.robots.ur5_bridge` ↔ UR5 Robot**:
  - RTDE Control & IO: moveJ, 그리퍼 제어 명령 전송
  - RTDE Receive: getActualQ, 현재 자세 정보 수신

- **`rfm.policies.runner` → UR5 Robot (직접)**:
  - RTDE Receive: 관절 현재값 직접 읽기 (정책 입력용)

**주요 수정 파일 (lcwoo 수정사항):**
- `gello_software/`: Gello 데이터 수집 소프트웨어
- `scripts/data/convert_to_lerobot.py`: 데이터 변환 스크립트
- `rfm/policies/runner.py`: 정책 실행 모듈
- `rfm/robots/ur5_bridge.py`: UR5 RTDE-ROS2 브리지

---

## 전체 워크플로우 (상세 실행 가이드)

### 1단계: 데이터셋 수집 (Gello 텔레오퍼레이션)

**전제 조건:**
- UR5 로봇이 Remote Control 모드
- Gello 하드웨어 연결 및 캘리브레이션 완료 (필요시 `docs/quickstart.md` 참고)

**실행 순서:**

```bash
# 터미널 1: 로봇 서버 실행 (ZMQ 서버, 포트 6001)
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python experiments/launch_nodes.py --robot ur --robot_ip 192.168.0.43 --hostname 0.0.0.0 --robot-port 6001

# 터미널 2: Gello 데이터 수집
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python experiments/run_env.py --agent=gello --use-save-interface --hostname 127.0.0.1
```

- **저장**: `s` 키 누르면 현재 궤적 저장
- **종료**: `q` 키
- 데이터 저장 위치: `gello_software/data/` 또는 `~/bc_data/`

### 2단계: 데이터셋 변환 (Gello → LeRobot Parquet)

```bash
cd /home/lcw/RFM && source venv/bin/activate
python scripts/data/convert_to_lerobot.py \
  --input /home/lcw/RFM/gello_data \
  --output /home/lcw/RFM/datasets/eggplant \
  --fps 30 \
  --task "Pick up the eggplant and place it on the plate."
```

- `--input`: Gello pkl 데이터 경로
- `--output`: LeRobot Parquet 출력 경로
- `--task`: 태스크 설명 (task-conditioned 학습용)
- `--fps`: 프레임레이트 (기본 30)

### 3단계: 정책 학습 (SmolVLA 또는 Octo)

**SmolVLA 학습:**

```bash
cd /home/lcw/RFM && source venv/bin/activate
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=eggplant \
  --dataset.root=/home/lcw/RFM/datasets/eggplant \
  --rename_map='{"observation.images.wrist":"observation.images.camera1"}' \
  --policy.empty_cameras=2 \
  --batch_size=64 \
  --steps=20000 \
  --save_freq=5000 \
  --output_dir=outputs/train/eggplant \
  --job_name=eggplant_smolvla \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.push_to_hub=false
```

**Octo 학습:**

```bash
cd /home/lcw/RFM && source venv/bin/activate
# Octo 학습 명령어 (예시)
python -m octo.scripts.train \
  --config octo/configs/base_config.py \
  --dataset.path=/home/lcw/RFM/datasets/eggplant \
  --output_dir=outputs/octo_finetune/eggplant
```

### 4단계: 학습된 정책 실행

**전제 조건:**
- UR5 로봇이 Remote Control 모드
- 학습된 체크포인트 경로 확인

**SmolVLA 정책 실행:**

```bash
# 터미널 1: UR5 브리지 실행 (필수 - 먼저 실행)
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
python -m rfm.robots.ur5_bridge
# 또는: rfm-ur5-bridge (pip install -e . 후)

# 터미널 2: SmolVLA 정책 실행
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
python -m rfm.policies.runner \
  --model-type smolvla \
  --checkpoint outputs/train/eggplant/checkpoints/020000/pretrained_model
# 또는: rfm-run-policy --model-type smolvla --checkpoint ...
```

**Octo 정책 실행:**

```bash
# 터미널 1: UR5 브리지 실행 (필수 - 먼저 실행)
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
python -m rfm.robots.ur5_bridge

# 터미널 2: Octo 정책 실행
python -m rfm.policies.runner \
  --model-type octo \
  --checkpoint outputs/octo_finetune/eggplant/5000 \
  --task "Pick up the eggplant and place it on the plate." \
  --window-size 2 \
  --exec-horizon 1
```

**주요 옵션:**
- `--model-type`: `smolvla` 또는 `octo`
- `--checkpoint`: 체크포인트 경로
- `--task`: Octo 사용 시 태스크 설명 (필수)
- `--window-size`: Octo 히스토리 윈도우 크기 (기본 2)
- `--exec-horizon`: Octo 실행 호라이즌 (기본 1)
- `--gripper-min`, `--gripper-max`: 그리퍼 하드웨어 범위
- `--dry-run`: 로봇 없이 모델만 테스트

---

## Gello 데이터 수집 (상세)

### 실행 순서 (2단계)

**1단계 — 터미널 1: 로봇 서버 실행**
```bash
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python experiments/launch_nodes.py --robot ur --robot_ip 192.168.0.43 --hostname 0.0.0.0 --robot-port 6001
```

**2단계 — 터미널 2: Gello 데이터 수집**
```bash
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python experiments/run_env.py --agent=gello --use-save-interface --hostname 127.0.0.1
```

- **저장**: `s` 키
- **종료**: `q` 키
- 데이터는 `gello_software/data/` 또는 `~/bc_data/` 에 저장됨

### Gello 초기화 및 캘리브레이션

- **로봇 초기 자세**: `run_env.py` 실행 시 로봇이 **고정 자세** `[0, -90, 90, -90, -90, 0]` deg + 그리퍼 닫힘으로 이동
- **Gello 캘리브레이션**: 로봇이 초기 자세에 도달한 시점에, **Gello의 현재 읽기를 "로봇 자세"로 매핑** (사용자가 Gello를 맞출 필요 없음)
- **Gello 포트 설정**: 시리얼 포트 자동 감지. 여러 포트가 있으면 `--gello-port /dev/serial/by-id/...` 로 지정

### Gello 오프셋 캘리브레이션 (초기 설정)

새로운 Gello 장치를 사용할 때는 오프셋을 캘리브레이션해야 합니다:

```bash
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python scripts/gello_get_offset.py \
  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U0V8-if00-port0 \
  --start-joints 0 0 0 0 0 0 \
  --joint-signs 1 1 -1 1 1 1 \
  --gripper
```

출력된 `best offsets`와 `gripper open/close` 값을 `gello_software/gello/agents/gello_agent.py`의 `PORT_CONFIG_MAP`에 추가.

### 주요 수정 사항

- **`run_env.py`**: 로봇을 고정 초기 자세로 이동 후, Gello를 그 자세에 자동 캘리브레이션
- **`dynamixel.py`**: `start_joints` 전달 시 오프셋을 "현재 읽기 = start_joints"가 되도록 정확히 조정 (기존 2π 래핑 방식 개선)
- **`gello_agent.py`**: FTA2U0V8 포트 설정 추가 (캘리브레이션 결과 반영)

---

## 참고 문서

- **`docs/quickstart.md`**: RFM 프로젝트 실행 명령어 정리 (venv, UR5 브리지, Gello 데이터 수집, 데이터셋 변환, 학습 등)
- **`PROJECT_STRUCTURE.md`**: 프로젝트 구조 및 파일 이동 계획 상세 설명
