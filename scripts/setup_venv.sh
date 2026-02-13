#!/usr/bin/env bash
# RFM venv 설정: LeRobot + UR5(RTDE) 환경
# 사용법: ./scripts/setup_venv.sh   (프로젝트 루트에서 실행 권장)

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${REPO_ROOT}/venv"
PYTHON_VERSION=3.10

echo "=== RFM venv 설정 (LeRobot + UR5) ==="
echo "프로젝트 루트: ${REPO_ROOT}"
echo ""

# 1) venv 없으면 생성
if [[ ! -d "${VENV_DIR}" ]]; then
  echo ">>> venv 생성 중: ${VENV_DIR}"
  python3 -m venv "${VENV_DIR}"
else
  echo ">>> 기존 venv 사용: ${VENV_DIR}"
fi

# 2) 활성화 (이 스크립트 내에서만 유효)
echo ">>> venv 활성화"
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

# 3) pip 업그레이드
echo ">>> pip 업그레이드"
pip install --upgrade pip

# 4) LeRobot 설치 (editable, 코어만 – [all]은 flash-attn 등 부담 큼)
echo ">>> LeRobot 설치 (editable)"
pip install -e "${REPO_ROOT}/lerobot"

# 5) UR5 RTDE 제어용 패키지
echo ">>> UR5 RTDE 패키지 설치 (ur-rtde)"
pip install ur-rtde

# 6) RFM 프로젝트 공통 의존성 (버전 충돌 없이 호환되는 것만)
# requirements.txt는 numpy==1.26.4 등으로 opencv-python과 충돌할 수 있어
# 개별 패키지만 추가 (이미 lerobot으로 설치된 것은 제외)
echo ">>> RFM 추가 패키지 (opencv-python GUI용 등)"
pip install opencv-python scipy matplotlib

echo ""
echo "=== 설치 완료 ==="
echo ""
echo "다음처럼 venv를 활성화한 뒤 사용하세요:"
echo "  source ${VENV_DIR}/bin/activate"
echo ""
echo "UR5 브리지(ur5_rtde_bridge.py)를 쓰려면 ROS2를 먼저 source 하세요:"
echo "  source /opt/ros/humble/setup.bash   # 또는 사용 중인 ROS2 디스트로"
echo "  source ${VENV_DIR}/bin/activate"
echo "  python ur5_rtde_bridge.py   # 또는 ros2 run ..."
echo ""
