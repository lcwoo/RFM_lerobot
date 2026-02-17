# RFM (Robot Foundation Models)

![Project Overview](overview.png)

UR5 + Gello 기반 데이터 수집 → LeRobot 변환 → SmolVLA/Octo 정책 실행을
지원하는 ROS2 기반 로봇 실행 레포입니다.

------------------------------------------------------------------------

# ⚡ TL;DR --- UR5 정책 실행 (가장 많이 씀)

## 1. Bridge 실행 (먼저)

``` bash
source /opt/ros/humble/setup.bash
cd /home/lcw/RFM
source .venv/bin/activate
python -m rfm.robots.ur5_bridge
```

## 2. 정책 실행

``` bash
python -m rfm.policies.runner \
  --model-type smolvla \
  --checkpoint <checkpoint_path>
```

또는 Octo:

``` bash
python -m rfm.policies.runner \
  --model-type octo \
  --checkpoint <checkpoint_path> \
  --task "Pick up the eggplant and place it on the plate."
```

⚠️ UR5는 반드시 **Remote Control 모드**\
⚠️ bridge를 **반드시 먼저 실행**해야 함

------------------------------------------------------------------------

# 설치

``` bash
cd /home/lcw/RFM
pip install -e .
```

CLI 사용:

``` bash
rfm-ur5-bridge
rfm-run-policy --model-type smolvla --checkpoint ...
```

------------------------------------------------------------------------

# 시스템 개요

RFM은 다음 워크플로우로 구성됩니다:

1.  Data Collection (Gello)
2.  Convert to LeRobot
3.  Model Training (External)
4.  Policy Execution

------------------------------------------------------------------------

# 정책 실행 아키텍처

    Trained Checkpoint
        ↓
    rfm.policies.runner
        ↕ (ROS2)
    rfm.robots.ur5_bridge
        ↕ (RTDE)
    UR5 Robot

------------------------------------------------------------------------

# rfm.policies.runner ↔ rfm.robots.ur5_bridge 통신 (상세)

> 디버깅 및 구현 참고용 상세 설명입니다.

## 역할 분담

  -----------------------------------------------------------------------
  컴포넌트                                     역할
  -------------------------------------------- --------------------------
  rfm.robots.ur5_bridge                        RTDE 연결 및 moveJ/그리퍼
                                               실행, `/ur5/status`
                                               퍼블리시

  rfm.policies.runner                          모델 추론 → 관절/그리퍼
                                               명령 생성 → 토픽 전송
  -----------------------------------------------------------------------

------------------------------------------------------------------------

## 토픽·서비스 정리

  -----------------------------------------------------------------------
  토픽/서비스                   타입          방향          설명
  ----------------------------- ------------- ------------- -------------
  `/ur5/goal_joint`             JointState    runner →      절대 관절
                                              bridge        목표(rad)

  `/ur5/gripper_cmd`            Float64       runner →      그리퍼 명령
                                              bridge        

  `/ur5/cmd`                    String        runner →      문자열 명령
                                              bridge        

  `/ur5/status`                 String        bridge →      "IDLE" /
                                              runner        "MOVING"

  `/ur5/stop`                   Trigger       runner →      긴급 정지
                                              bridge        
  -----------------------------------------------------------------------

------------------------------------------------------------------------

# 데이터 수집 (Gello)

## 실행 순서

### 터미널 1

``` bash
cd gello_software
source .venv/bin/activate
python experiments/launch_nodes.py --robot ur --robot_ip 192.168.0.43 --hostname 0.0.0.0 --robot-port 6001
```

### 터미널 2

``` bash
cd gello_software
source .venv/bin/activate
python experiments/run_env.py --agent=gello --use-save-interface --hostname 127.0.0.1
```

-   `s` → 저장
-   `q` → 종료

------------------------------------------------------------------------

# 🔧 Gello 초기화 및 캘리브레이션 (상세)

## 자동 초기화 동작

-   `run_env.py` 실행 시 로봇은 고정 초기 자세로 이동:

        [0, -90, 90, -90, -90, 0] (deg)

-   그리퍼는 닫힘 상태로 설정

-   로봇이 초기 자세에 도달하면 **Gello 현재 읽기를 로봇 자세에 자동
    매핑**

-   사용자가 Gello를 수동으로 맞출 필요 없음

------------------------------------------------------------------------

## 오프셋 캘리브레이션 (신규 장치 사용 시)

``` bash
cd /home/lcw/RFM/gello_software
source .venv/bin/activate
python scripts/gello_get_offset.py \
  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U0V8-if00-port0 \
  --start-joints 0 0 0 0 0 0 \
  --joint-signs 1 1 -1 1 1 1 \
  --gripper
```

출력된:

-   `best offsets`
-   `gripper open/close` 값

을 `gello_agent.py`의 `PORT_CONFIG_MAP`에 추가해야 함.

------------------------------------------------------------------------

## 내부 수정 사항

-   `run_env.py`\
    → 로봇 초기 자세 이동 후 Gello 자동 캘리브레이션

-   `dynamixel.py`\
    → 기존 2π 래핑 방식 제거\
    → "현재 읽기 = start_joints" 되도록 정확히 오프셋 보정

-   `gello_agent.py`\
    → FTA2U0V8 포트 설정 추가

------------------------------------------------------------------------

# 데이터 변환

``` bash
python scripts/data/convert_to_lerobot.py \
  --input /home/lcw/RFM/gello_data \
  --output /home/lcw/RFM/datasets/eggplant \
  --fps 30 \
  --task "Pick up the eggplant and place it on the plate."

# 이미지가 없는 데이터셋의 경우:
# --create-dummy-image 옵션 추가
```

------------------------------------------------------------------------

# 모델 학습

## SmolVLA

``` bash
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
  --wandb.enable=true
```

## Octo

⚠️ **Octo는 RLDS 형식 데이터셋이 필요합니다.** LeRobot Parquet를 RLDS로 변환 필요.

``` bash
cd /home/lcw/RFM/octo
source .venv/bin/activate  # Octo 전용 venv

python -m octo.scripts.train \
  --config octo/scripts/configs/finetune_config.py:full,multimodal \
  --config.pretrained_path=hf://rail-berkeley/octo-base \
  --config.pretrained_step=250000 \
  --config.save_dir=/home/lcw/RFM/outputs/octo_finetune/eggplant \
  --config.dataset_kwargs.name=eggplant_rlds \
  --config.dataset_kwargs.data_dir=/home/lcw/RFM/datasets/rlds/eggplant_rlds \
  --config.wandb.project=octo_finetune \
  --name=eggplant_octo
```

------------------------------------------------------------------------

# 프로젝트 구조

    RFM/
    ├── rfm/
    │   ├── robots/
    │   ├── policies/
    │   └── utils/
    ├── scripts/
    ├── nodes/
    ├── launch/
    ├── configs/
    ├── docs/
    ├── tests/
    ├── gello_software/
    ├── lerobot/
    ├── octo/
    └── datasets/

자세한 구조는 `PROJECT_STRUCTURE.md` 참고.
