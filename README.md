# RFM (lcw)

Gello로 UR5를 움직여 **데이터셋을 수집**하고, **Parquet(LeRobot 포맷)으로 변환**할 수 있으며, 학습한 **체크포인트로 UR5를 움직이게** 할 수 있는 레포입니다.

- **데이터 수집**: Gello로 UR5를 텔레오퍼레이션하며 데모를 기록
- **변환**: Gello pkl → LeRobot Parquet (`scripts/convert_gello_to_lerobot.py`)
- **정책 실행**: 학습된 SmolVLA/Octo 체크포인트로 UR5 제어 — **`run_policy_ur5.py`** 가 **`ur5_rtde_bridge`** 와 ROS2 토픽으로 통신해 로봇을 움직입니다.

---

## lcwoo가 수정/추가한 내용 (git 기준)

- **first commit**  
  - 초기 코드: README, 텔레오프 예제, 마스크/포즈 트래커 노드, `run_all.sh` 등

- **Add gello_software (exclude third_party)**  
  - `gello_software/` 추가 (Gello 기반 로봇 제어·데이터 수집용, third_party 제외)

- **Add UR5 policy runner, newbie, and gello->lerobot converter**  
  - **`run_policy_ur5.py`**: SmolVLA/Octo 체크포인트로 UR5 정책 실행. **bridge(`ur5_rtde_bridge.py`)와 통신**해 `/ur5/goal_joint`, `/ur5/gripper_cmd` 로 명령을 보내고 `/ur5/status` 로 상태를 확인합니다.  
  - **`ur5_rtde_bridge.py`**: UR5 RTDE와 ROS2 사이 브릿지. `goal_joint`(절대 관절값), `gripper_cmd`, `cmd`(go/list/save 등) 구독 → moveJ/그리퍼 제어, `status`(IDLE/MOVING) 퍼블리시.  
  - **`scripts/convert_gello_to_lerobot.py`**: Gello pkl → LeRobot Parquet 변환.  
  - 기타: `newbie`, `scripts/` (그리퍼 체크/디버그, venv 설정), `requirements-venv.txt`, `.gitignore` 등.

---

## run_policy_ur5.py ↔ ur5_rtde_bridge.py 통신 (자세히)

두 노드는 **ROS2 토픽/서비스**로만 통신합니다. RTDE(로봇 제어)는 bridge만 사용하고, `run_policy_ur5.py`는 **명령 전송**과 **상태 확인**만 합니다.

### 역할 분담

| 쪽 | 역할 |
|----|------|
| **ur5_rtde_bridge.py** | UR5와 RTDE 연결(rtde_control, rtde_receive, rtde_io/ RobotiqGripper). 토픽으로 받은 명령을 moveJ/그리퍼로 실행하고, 상태를 `/ur5/status`로 퍼블리시. |
| **run_policy_ur5.py** | 카메라·모델 추론 → action(관절 목표 + 그리퍼) → bridge가 구독하는 토픽으로 전송. `/ur5/status`를 구독해 **IDLE일 때만** 다음 관절 명령 전송. 관절 **현재값**은 rtde_receive로 **직접** 읽음(정책 입력용). |

### 토픽·서비스 정리

| 토픽/서비스 | 메시지 타입 | 방향 | 설명 |
|-------------|-------------|------|------|
| `/ur5/goal_joint` | `sensor_msgs/JointState` | run_policy → bridge | **절대 관절 목표(rad)**. `msg.position`에 **6개 값** (joint 0~5). bridge는 수신 시 `rtde_c.moveJ(q)` 실행. |
| `/ur5/gripper_cmd` | `std_msgs/Float64` | run_policy → bridge | 그리퍼 명령. bridge는 `value > gripper_mid`면 열림(True), 아니면 닫힘(False). RTDE Tool DO 0 또는 RobotiqGripper로 출력. |
| `/ur5/cmd` | `std_msgs/String` | run_policy → bridge | 문자열 명령. `run_policy_ur5.py`는 **시작 포즈** 이동 시 `"go <이름>"` 퍼블리시. bridge는 `go <name>`이면 `ur5_saved_poses.json`의 해당 관절값으로 moveJ. (기타: `where` / `list` / `save <name>`) |
| `/ur5/status` | `std_msgs/String` | bridge → run_policy | **"IDLE"** 또는 **"MOVING"**. bridge가 **명령 수신 직후** "MOVING"으로 바꾸고, moveJ 스레드 **완료 시** "IDLE"로 바꿔서 퍼블리시. 짧은 동작도 최소 한 번 MOVING → IDLE 전환이 있음. |
| `/ur5/stop` | `std_srvs/Trigger` (서비스) | run_policy → bridge | 긴급 정지. run_policy는 Ctrl+C 등으로 종료 시 이 서비스를 호출. bridge는 `rtde_c.stopL(0.5)` 등으로 동작 정지. |

### 제어 흐름 (한 스텝)

1. **run_policy_ur5.py** 제어 루프(예: 30 Hz):
   - `robot.spin_once()` → `/ur5/status` 콜백으로 최신 상태 갱신.
   - **`robot.is_idle`가 False**(= status가 "MOVING")이면 이번 스텝은 **스킵** (이전 moveJ 완료 대기).
   - **IDLE이면**: 카메라 캡처 + **rtde_receive**로 현재 관절값 읽기 → 모델 추론 → `joint_target`(6개), `gripper_cmd` 계산.
   - **`/ur5/goal_joint`** 에 `JointState(position=joint_target)` 퍼블리시.
   - **`/ur5/gripper_cmd`** 에 `Float64(gripper_cmd)` 퍼블리시.

2. **ur5_rtde_bridge.py**:
   - `/ur5/goal_joint` 수신 시 `_on_joint_abs()` 호출 → `_start_motion()`으로 **별도 스레드**에서 `rtde_c.moveJ(q)` 실행. 즉시 **status = "MOVING"** 퍼블리시.
   - moveJ가 끝나면 **status = "IDLE"** 퍼블리시.
   - `/ur5/gripper_cmd` 수신 시 `_on_gripper_cmd()`에서 RTDE IO 또는 RobotiqGripper로 그리퍼 제어(비동기, status와 무관).

3. **관절 현재값**: run_policy는 **bridge를 거치지 않고** `rtde_receive.RTDEReceiveInterface(robot_ip)`로 직접 `getActualQ()`를 호출합니다. (읽기 전용은 다중 연결 가능하므로 bridge와 충돌 없음.)

### 메시지 형식 요약

- **`/ur5/goal_joint`**: `JointState.position` = 길이 6 이상 리스트, 단위 **rad**. 앞 6개만 사용.
- **`/ur5/gripper_cmd`**: `Float64.data` = 한 개 실수. bridge는 `gripper_mid`와 비교해 열림/닫힘으로 변환; RobotiqGripper 사용 시 `gripper_min_hw`~`gripper_max_hw`로 스케일 후 0~255로 보냄.
- **`/ur5/cmd`**: `String.data` = `"go observe"` 등. 공백 한 칸 뒤에 포즈 이름.
- **`/ur5/status`**: `String.data` = `"IDLE"` 또는 `"MOVING"` (따옴표 제외 문자열).

---

## UR5 정책 실행 (실행 순서)

실행 순서 예:

```bash
# 터미널 1: bridge 실행
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
python ur5_rtde_bridge.py

# 터미널 2: 정책 실행 (SmolVLA 예)
python run_policy_ur5.py --model-type smolvla --checkpoint outputs/train/eggplant/checkpoints/020000/pretrained_model

# Octo 예
python run_policy_ur5.py --model-type octo --checkpoint <체크포인트경로> --task "Pick up the eggplant and place it on the plate."
```

정책 실행 전에 **반드시 `ur5_rtde_bridge.py` 가 실행 중**이어야 합니다.

---

## 데이터셋 수집·변환 요약

| 단계 | 설명 |
|------|------|
| 수집 | Gello(gello_software)로 UR5 텔레오퍼레이션 → pkl 등으로 저장 |
| 변환 | `scripts/convert_gello_to_lerobot.py` 로 Gello → LeRobot Parquet |
| 학습 | LeRobot/Octo 등으로 체크포인트 학습 |
| 실행 | `run_policy_ur5.py` + `ur5_rtde_bridge.py` 로 학습된 정책으로 UR5 구동 |
