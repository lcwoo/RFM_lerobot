# RFM venv + UR5 명령어 정리

## 0. gello_software 설치 (텔레오퍼레이션·데이터 수집)
# 저장소: https://github.com/wuphilipp/gello_software
# 가상환경: gello_software 전용 .venv (Python 3.10)

# 0-1. gello_software가 없으면 클론 (RFM 루트에서)
# git clone https://github.com/wuphilipp/gello_software /home/lcw/RFM/gello_software

# 0-2. venv 없으면 생성 (gello_software 안에서)
# cd /home/lcw/RFM/gello_software && python3 -m venv .venv && source .venv/bin/activate && pip install -e .

# 0-2b. 의존성 설치 (tyro 등 — ModuleNotFoundError 나면 실행)
# cd /home/lcw/RFM/gello_software && source .venv/bin/activate && pip install -r requirements.txt

# 0-2c. gello 패키지 에디터블 설치 (No module named 'gello' 나면 실행)
# cd /home/lcw/RFM/gello_software && source .venv/bin/activate && pip install -e .

# 0-2d. RFM .venv 쓸 때 Gello용 추가 패키지 (dynamixel_sdk 등 — ModuleNotFoundError 나면 실행)
# cd /home/lcw/RFM && source .venv/bin/activate && pip install dynamixel_sdk

# 0-2e. Gello 초기 팔이 어긋날 때 (오프셋 캘리브레이션)
# 1) Gello를 원하는 "0 자세"에 맞춰 둔 뒤, gello_software에서:
#    python scripts/gello_get_offset.py --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA2U0V8-if00-port0 --start-joints 0 0 0 0 0 0 --joint-signs 1 1 -1 1 1 1 --gripper
# 2) 출력된 best offsets / gripper open,close 를 gello_software/gello/agents/gello_agent.py 의 FTA2U0V8 항목에 반영

# --- 0-3. Gello 데이터 수집: 꼭 2단계 (서버 먼저, 그 다음 클라이언트) ---
# run_env.py는 ZMQ 클라이언트라서, 6001 포트에서 대기하는 "로봇 서버"가 먼저 떠 있어야 함.

cd /home/lcw/RFM/gello_software && source .venv/bin/activate && python experiments/run_env.py --agent=gello --use-save-interface

# [2단계] 터미널 2 — Gello 데이터 수집 (저장: s, 종료: q)
# 서버가 같은 PC면 --hostname 127.0.0.1, 서버가 192.168.0.43 이면 --hostname 192.168.0.43
cd /home/lcw/RFM/gello_software && source .venv/bin/activate
python experiments/run_env.py --agent=gello --use-save-interface --hostname 127.0.0.1
# 서버가 다른 PC(예: 192.168.0.43)에 있으면:
# python experiments/run_env.py --agent=gello --use-save-interface --hostname 192.168.0.43

# YAM + YAML 설정인 경우 (데이터 저장 포함)
# python experiments/launch_yaml.py --left-config-path configs/yam_passive.yaml --use-save-interface

# 저장 데이터는 gello_software/data/ 에 생성됨. RFM 변환 스크립트용으로 쓰려면:
#   ln -sf /home/lcw/RFM/gello_software/data /home/lcw/RFM/gello_data
#   또는 변환 시 --input /home/lcw/RFM/gello_software/data 지정

## 1. venv 활성화
cd /home/lcw/RFM && source .venv/bin/activate

## 2. UR5 작동시키기 (ROS2 필요)
# 먼저 ROS2 source (한 터미널 세션당 한 번)
source /opt/ros/humble/setup.bash

# 그 다음 venv 활성화
cd /home/lcw/RFM && source .venv/bin/activate

# UR5 RTDE 브리지 실행 (로봇 IP는 기본 192.168.0.43)
python ur5_rtde_bridge.py

# 또는 ros2 run으로 실행하는 경우
# ros2 run <패키지명> <노드명>

## 3. 한 번에 복사해서 쓸 때 (venv만)
cd /home/lcw/RFM && source venv/bin/activate

## 4. 한 번에 복사해서 쓸 때 (UR5 브리지 실행까지)
source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate && python ur5_rtde_bridge.py

## 5. Gello to LeRobot 데이터셋 변환 (dataset/eggplant)
# v3.0 포맷으로 meta/info.json, meta/stats.json, meta/tasks.parquet, meta/episodes/ 가 생성됨 (episodes.jsonl 등 레거시 .jsonl 불필요)
# --task 에 넣은 문구가 데이터셋의 "명령"으로 저장됨 (task-conditioned 학습용)
# 기존 dataset 삭제 후 변환
rm -rf /home/lcw/RFM/datasets/eggplant
cd /home/lcw/RFM && source venv/bin/activate
python scripts/convert_gello_to_lerobot.py \
  --input /home/lcw/RFM/gello_data \
  --output /home/lcw/RFM/datasets/eggplant \
  --fps 30 \
  --task "Pick up the eggplant and place it on the plate."

## 6. LeRobot 학습 (로컬 데이터셋: datasets/eggplant, 카메라 1개 → SmolVLA 3카메라 기대 대응)
# 데이터셋은 observation.images.wrist 1개뿐 → rename_map으로 camera1로 이름 맞춤, empty_cameras=2로 나머지 2개는 빈 이미지
# 체크포인트: 기본 save_freq=20000 (2만 step마다 + 마지막 step). 더 자주 저장하려면 --save_freq=5000 등 추가
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
  --job_name=eggplant_0209 \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.push_to_hub=false





  