#!/usr/bin/env python3
"""
UR5에서 학습된 정책(SmolVLA 또는 Octo)을 실행하는 추론 스크립트.

구조:
  ur5_rtde_bridge.py (별도 터미널에서 실행 중)
     ↑ /ur5/goal_joint  (JointState, 절대 관절값)
     ↓ /ur5/status       (String, "IDLE" / "MOVING")
  이 스크립트 (ROS2 노드)
     - 모델 추론 → action (절대 관절값) → /ur5/goal_joint 퍼블리시
     - 카메라 이미지 (pyrealsense2)
     - 로봇 관절 상태 (rtde_receive 별도 연결)

사용법:
  # 터미널 1: bridge 실행
  source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
  python ur5_rtde_bridge.py

  # 터미널 2: SmolVLA 정책 실행
  source /opt/ros/humble/setup.bash && cd /home/lcw/RFM && source venv/bin/activate
  python run_policy_ur5.py --model-type smolvla \
    --checkpoint outputs/train/eggplant/checkpoints/020000/pretrained_model

  # 터미널 2: Octo 정책 실행
  python run_policy_ur5.py --model-type octo \
    --checkpoint /home/lcw/RFM/outputs/octo_finetune/.../5000 \
    --task "Pick up the eggplant and place it on the plate." \
    --window-size 2 --exec-horizon 1

  # 드라이런 (로봇/카메라/ROS2 없이 모델만 테스트)
  python run_policy_ur5.py --model-type octo --dry-run \
    --checkpoint /home/lcw/RFM/outputs/octo_finetune/.../5000

그리퍼:
  - 학습 데이터(Gello): action[7] = 그리퍼 위치, 높을수록 열림. convert_gello_to_lerobot.py 변환 시 min/max가 출력됨.
  - Gello 데이터에서 그리퍼 최소값은 0이 아니라 약 0.05(닫힘). 0을 보내면 많은 하드웨어에서 그리퍼가 안 움직임 → 변환 스크립트 출력의 min을 --gripper-min으로 쓰세요.
  - 실제 그리퍼가 반대면 --invert-gripper. 범위는 --gripper-min / --gripper-max. --calibrate-gripper 로 자동 수집 가능.

주의사항:
  - 실행 전에 반드시 ur5_rtde_bridge.py가 돌아가고 있어야 합니다.
  - UR5가 리모트 컨트롤 모드인지 확인하세요.
  - 비상 시 Ctrl+C로 즉시 정지합니다.
"""

import argparse
import json
import logging
import signal
import sys
import threading
import time
from collections import deque
from functools import partial
from pathlib import Path

import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

# ──────────────────────────────────────────────
# 기본 설정값
# ──────────────────────────────────────────────
DEFAULT_CHECKPOINT = "outputs/train/eggplant/checkpoints/020000/pretrained_model"
DEFAULT_TASK = "Pick up the eggplant and place it on the plate."
DEFAULT_ROBOT_IP = "192.168.0.43"
DEFAULT_DEVICE = "cuda"
DEFAULT_FPS = 30
DEFAULT_DURATION = 200


def parse_args():
    p = argparse.ArgumentParser(description="UR5에서 SmolVLA / Octo 정책 실행")
    p.add_argument("--model-type", type=str, default="smolvla",
                    choices=["smolvla", "octo"],
                    help="모델 유형 (smolvla / octo)")
    p.add_argument("--checkpoint", type=str, default=DEFAULT_CHECKPOINT,
                    help="체크포인트 디렉토리 경로")
    p.add_argument("--task", type=str, default=DEFAULT_TASK,
                    help="정책에 전달할 task 문자열")
    p.add_argument("--robot-ip", type=str, default=DEFAULT_ROBOT_IP,
                    help="UR5 로봇 IP (rtde_receive용)")
    p.add_argument("--device", type=str, default=DEFAULT_DEVICE,
                    help="모델 디바이스 (cuda / cpu, SmolVLA 전용)")
    p.add_argument("--fps", type=float, default=DEFAULT_FPS,
                    help="제어 루프 주기 (Hz)")
    p.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                    help="실행 시간 (초)")
    p.add_argument("--dry-run", action="store_true",
                    help="로봇/카메라/ROS2 없이 모델만 테스트")
    p.add_argument("--start-pose", type=str, default="observe",
                    help="시작 전 이동할 저장된 포즈 이름 (ur5_saved_poses.json)")
    p.add_argument("--no-start-pose", action="store_true",
                    help="시작 포즈로 이동하지 않음")
    p.add_argument("--invert-gripper", action="store_true",
                    help="그리퍼 명령 반전 (정책: 높을수록 열림 → 하드웨어가 반대일 때 사용)")
    p.add_argument("--gripper-min", type=float, default=0.0,
                    help="그리퍼 하드웨어 최소값(닫힘). 0이면 안 움직일 수 있음 → scripts/convert_gello_to_lerobot.py 변환 시 출력되는 min 사용 권장")
    p.add_argument("--gripper-max", type=float, default=1.0,
                    help="그리퍼 하드웨어 최대값(열림). convert_gello_to_lerobot.py 변환 시 출력되는 max 사용 권장")
    p.add_argument("--calibrate-gripper", action="store_true",
                    help="시작 후 일정 시간 동안 그리퍼 값을 수집해 gripper_min/max 자동 설정")
    p.add_argument("--calibrate-gripper-sec", type=float, default=15.0,
                    help="그리퍼 캘리브레이션 수집 시간(초). 이 동안 정책이 움직이면 min/max가 갱신됨")
    p.add_argument("--use-ros2-camera", action="store_true",
                    help="ROS2 토픽에서 카메라 이미지 구독 (realsense2_camera_node 사용 시)")
    p.add_argument("--camera-topic", type=str, default="/wrist_cam/camera/color/image_raw",
                    help="ROS2 카메라 이미지 토픽 (--use-ros2-camera 사용 시)")
    # ── Octo 전용 옵션 ──
    p.add_argument("--window-size", type=int, default=2,
                    help="Octo 관측 이력 윈도우 크기 (학습 시 설정과 동일하게)")
    p.add_argument("--exec-horizon", type=int, default=1,
                    help="Octo action chunk 중 실행할 스텝 수 (1=가장 반응적)")
    return p.parse_args()


# ──────────────────────────────────────────────
# 모델 로딩 — SmolVLA
# ──────────────────────────────────────────────
def load_smolvla_policy(checkpoint_path: str, device: str):
    """SmolVLA 정책 + 전/후처리기 로드."""
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    logger.info(f"[SmolVLA] 체크포인트 로딩: {checkpoint_path}")
    config = PreTrainedConfig.from_pretrained(checkpoint_path)
    config.device = device

    policy_class = get_policy_class(config.type)
    policy = policy_class.from_pretrained(checkpoint_path, config=config)
    policy = policy.to(device)
    policy.eval()
    logger.info(f"[SmolVLA] 정책 로드 완료: {config.type} on {device}")

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=checkpoint_path,
        dataset_stats=None,
        preprocessor_overrides={
            "device_processor": {"device": device},
        },
    )
    logger.info("[SmolVLA] 전/후처리기 로드 완료")

    return policy, preprocessor, postprocessor, config


# ──────────────────────────────────────────────
# 모델 로딩 — Octo
# ──────────────────────────────────────────────
def load_octo_policy(checkpoint_path: str):
    """Octo 모델 + 정책 함수 로드."""
    # Octo 경로를 sys.path에 추가 (octo 패키지가 pip install 안 되어 있을 수 있음)
    octo_root = Path(__file__).resolve().parent / "octo"
    if octo_root.exists() and str(octo_root) not in sys.path:
        sys.path.insert(0, str(octo_root))

    from octo.model.octo_model import OctoModel
    from octo.utils.train_callbacks import supply_rng

    logger.info(f"[Octo] 체크포인트 로딩: {checkpoint_path}")
    model = OctoModel.load_pretrained(checkpoint_path)

    # 모델 정보 로그
    example_obs = model.example_batch["observation"]
    obs_keys = [k for k in example_obs.keys() if k != "timestep_pad_mask"]
    action_dim = model.example_batch["action"].shape[-1]
    logger.info(f"[Octo] observation keys: {obs_keys}")
    logger.info(f"[Octo] action dim: {action_dim}")
    logger.info(f"[Octo] dataset_statistics keys: {list(model.dataset_statistics.keys())}")

    # policy_fn: supply_rng가 매 호출마다 새로운 JAX RNG 키를 자동 공급
    policy_fn = supply_rng(
        partial(
            model.sample_actions,
            unnormalization_statistics=model.dataset_statistics["action"],
        ),
    )
    logger.info("[Octo] 정책 함수 준비 완료")

    return model, policy_fn


# ──────────────────────────────────────────────
# Octo 관측 이력 관리
# ──────────────────────────────────────────────
class OctoObservationHistory:
    """HistoryWrapper와 동일한 로직으로 관측 이력을 관리.

    Gym 환경 없이 수동으로 관측을 쌓을 때 사용.
    """

    def __init__(self, window_size: int):
        self.window_size = window_size
        self.history = deque(maxlen=window_size)
        self.num_obs = 0

    def reset(self, obs: dict) -> dict:
        """초기 관측으로 히스토리 초기화 (패딩 포함)."""
        self.num_obs = 1
        self.history.clear()
        for _ in range(self.window_size):
            self.history.append(obs)
        return self._get_stacked()

    def add(self, obs: dict) -> dict:
        """새 관측 추가 후 윈도우 반환."""
        self.num_obs += 1
        self.history.append(obs)
        return self._get_stacked()

    def _get_stacked(self) -> dict:
        """히스토리를 스택하고 패딩 마스크 추가."""
        stacked = {
            k: np.stack([dic[k] for dic in self.history])
            for k in self.history[0]
        }
        pad_length = self.window_size - min(self.num_obs, self.window_size)
        timestep_pad_mask = np.ones(self.window_size)
        timestep_pad_mask[:pad_length] = 0
        stacked["timestep_pad_mask"] = timestep_pad_mask
        return stacked


# ──────────────────────────────────────────────
# 카메라 (RealSense 직접 접근 또는 ROS2 토픽)
# ──────────────────────────────────────────────
class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        import pyrealsense2 as rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.pipeline.start(config)
        for _ in range(30):
            self.pipeline.wait_for_frames()
        logger.info(f"RealSense 카메라 초기화 완료 ({width}x{height} @ {fps}fps)")

    def capture(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def stop(self):
        self.pipeline.stop()


class ROS2Camera:
    """ROS2 토픽에서 카메라 이미지를 구독하는 클래스."""
    def __init__(self, node, topic="/wrist_cam/camera/color/image_raw", width=640, height=480, fps=30):
        """
        Args:
            node: ROS2 Node 인스턴스 (rclpy.init() 후에 생성된 노드)
            topic: 카메라 이미지 토픽
            width, height, fps: 예상 이미지 크기 (로깅용)
        """
        from sensor_msgs.msg import Image
        from rclpy.qos import qos_profile_sensor_data
        
        self.node = node
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # ROS2 구독 생성
        self.sub = self.node.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )
        
        # 첫 이미지 수신 대기
        logger.info(f"ROS2 카메라 토픽 구독 대기: {topic}")
        for _ in range(100):  # 최대 10초 대기
            with self.image_lock:
                if self.latest_image is not None:
                    logger.info(f"ROS2 카메라 초기화 완료 ({width}x{height})")
                    return
            # ROS2 spin을 한 번 실행하여 메시지 수신
            import rclpy
            rclpy.spin_once(self.node, timeout_sec=0.1)
        raise RuntimeError(f"ROS2 카메라 토픽에서 이미지를 수신하지 못했습니다: {topic}")
    
    def _on_image(self, msg):
        """이미지 메시지 수신 콜백."""
        try:
            import cv2
            # cv_bridge 대신 직접 numpy 배열로 변환 (NumPy 호환성 문제 회피)
            # sensor_msgs.msg.Image의 데이터를 직접 처리
            if msg.encoding == "rgb8":
                # RGB8 형식
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_array.reshape((msg.height, msg.width, 3))
            elif msg.encoding == "bgr8":
                # BGR8 형식 -> RGB로 변환
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img_bgr = img_array.reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "mono8":
                # 그레이스케일 -> RGB로 변환
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img_gray = img_array.reshape((msg.height, msg.width))
                img = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)
            else:
                # 다른 형식은 cv_bridge 시도 (실패할 수 있음)
                try:
                    from cv_bridge import CvBridge
                    bridge = CvBridge()
                    cv_image = bridge.imgmsg_to_cv2(msg, "rgb8")
                    img = cv_image
                except Exception as e:
                    logger.warning(f"이미지 형식 {msg.encoding} 변환 실패: {e}")
                    return
            
            with self.image_lock:
                self.latest_image = img
        except Exception as e:
            logger.warning(f"이미지 변환 실패: {e}")
            import traceback
            logger.debug(traceback.format_exc())
    
    def capture(self) -> np.ndarray:
        """최신 이미지 캡처."""
        # ROS2 spin을 한 번 실행하여 최신 메시지 수신
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.01)
        
        with self.image_lock:
            if self.latest_image is None:
                raise RuntimeError("카메라 이미지가 아직 수신되지 않았습니다")
            return self.latest_image.copy()
    
    def stop(self):
        """정리."""
        # 구독은 노드가 파괴될 때 자동으로 정리됨
        pass


class DummyCamera:
    def __init__(self, width=640, height=480):
        self.width, self.height = width, height
        logger.info(f"DummyCamera 초기화 ({width}x{height}) - 실제 카메라 없이 더미 이미지 사용")
        logger.info(f"더미 카메라 초기화 ({width}x{height})")

    def capture(self) -> np.ndarray:
        return np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)

    def stop(self):
        pass


# ──────────────────────────────────────────────
# UR5 인터페이스 (ROS2 토픽 방식)
# ──────────────────────────────────────────────
class UR5ROS2Interface:
    """
    ur5_rtde_bridge.py와 통신하는 ROS2 인터페이스.

    - 관절 상태 읽기: rtde_receive 직접 연결 (읽기 전용, 다중 연결 가능)
    - 관절 명령: /ur5/goal_joint 토픽에 JointState 퍼블리시
    - 상태 확인: /ur5/status 구독 ("IDLE" / "MOVING")
    - 시작 포즈: /ur5/cmd 토픽에 "go <name>" 퍼블리시
    - 정지: /ur5/stop 서비스 호출
    """

    def __init__(self, node, robot_ip: str):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        self.node = node
        self._JointState = JointState

        # RTDE receive (읽기 전용 - bridge와 별개로 연결 가능)
        import rtde_receive
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        logger.info(f"rtde_receive 연결: {robot_ip}")

        # 퍼블리셔
        self.pub_goal_joint = node.create_publisher(JointState, "/ur5/goal_joint", 10)
        self.pub_cmd = node.create_publisher(String, "/ur5/cmd", 10)
        from std_msgs.msg import Float64
        self.pub_gripper = node.create_publisher(Float64, "/ur5/gripper_cmd", 10)

        # 상태 구독
        self._status = "UNKNOWN"
        node.create_subscription(String, "/ur5/status", self._on_status, 10)

        # 정지 서비스 클라이언트
        self.stop_client = node.create_client(Trigger, "/ur5/stop")

        logger.info("ROS2 인터페이스 초기화 완료")

    def _on_status(self, msg):
        self._status = msg.data

    @property
    def is_idle(self) -> bool:
        return self._status == "IDLE"

    def get_joint_positions(self) -> np.ndarray:
        """현재 6개 관절 위치 (rad)."""
        return np.array(self.rtde_r.getActualQ(), dtype=np.float32)

    def get_gripper_position(self) -> float:
        """그리퍼 위치. TODO: 실제 그리퍼에 맞게 수정."""
        return 0.0

    def send_joint_command(self, joint_targets: np.ndarray):
        """
        절대 관절값을 /ur5/goal_joint 토픽으로 퍼블리시.
        ur5_rtde_bridge.py가 받아서 moveJ 실행.
        """
        msg = self._JointState()
        msg.position = [float(v) for v in joint_targets[:6]]
        self.pub_goal_joint.publish(msg)
        logger.debug(f"📤 /ur5/goal_joint 발행: {[f'{v:.3f}' for v in joint_targets[:6]]}")
    
    def send_gripper_command(self, gripper_value: float):
        """그리퍼 명령을 /ur5/gripper_cmd 토픽으로 퍼블리시."""
        from std_msgs.msg import Float64
        msg = Float64()
        msg.data = float(gripper_value)
        self.pub_gripper.publish(msg)
        logger.debug(f"📤 /ur5/gripper_cmd 발행: {gripper_value:.3f}")

    def go_to_pose(self, pose_name: str):
        """/ur5/cmd 토픽에 'go <name>' 퍼블리시."""
        from std_msgs.msg import String
        msg = String()
        msg.data = f"go {pose_name}"
        self.pub_cmd.publish(msg)

    def stop(self):
        """긴급 정지."""
        from std_srvs.srv import Trigger
        if self.stop_client.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            future = self.stop_client.call_async(req)
            import rclpy
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            logger.info("로봇 정지 완료")
        else:
            logger.warning("정지 서비스 응답 없음")

    def spin_once(self, timeout_sec=0.0):
        """ROS2 콜백 한 번 처리 (status 업데이트 등)."""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)


class DummyUR5Interface:
    """드라이런용 더미 인터페이스."""

    def __init__(self):
        self._q = np.array([0.78, -1.49, 1.82, -1.78, -1.48, 0.10], dtype=np.float32)
        self._status = "IDLE"
        logger.info("더미 로봇 초기화")

    @property
    def is_idle(self) -> bool:
        return True

    def get_joint_positions(self) -> np.ndarray:
        return self._q.copy()

    def get_gripper_position(self) -> float:
        return 0.05

    def send_joint_command(self, joint_targets: np.ndarray):
        self._q = joint_targets[:6].astype(np.float32)

    def go_to_pose(self, pose_name: str):
        logger.info(f"[DRY-RUN] go {pose_name}")

    def stop(self):
        pass

    def spin_once(self, timeout_sec=0.0):
        pass


# ──────────────────────────────────────────────
# 관측(observation) 구성 — SmolVLA
# ──────────────────────────────────────────────
def build_observation_smolvla(
    image: np.ndarray,
    joint_positions: np.ndarray,
    gripper_position: float,
    task: str,
) -> dict:
    """
    SmolVLA 학습 데이터와 동일한 형태의 관측 딕셔너리 구성.

    학습 데이터 (convert_gello_to_lerobot.py 기준):
      observation.state = concat([
          joint_positions(7),     # UR5 6관절 + Gello 7번째축
          joint_velocities(7),    # 실제 데이터에서는 joint_positions와 동일
          ee_pos_quat(7),         # 실제 데이터에서는 전부 0
          gripper_position(1),    # 그리퍼
      ])  # 총 22차원

      action = concat([
          control(7),             # 목표 관절값 (6 UR5 + 1 Gello)
          gripper(1),             # 그리퍼
      ])  # 총 8차원
    """
    import torch

    # --- 이미지: (H,W,3) uint8 → (1, 3, H, W) float32 [0,1] ---
    img = image.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img_tensor = torch.from_numpy(img).unsqueeze(0)

    # --- 상태: 22차원 (학습 데이터와 동일 구조) ---
    jp = np.zeros(7, dtype=np.float32)
    jp[:6] = joint_positions[:6]
    jp[6] = gripper_position  # Gello 7번째축 → 그리퍼로 대체

    state = np.concatenate([
        jp,                                # joint_positions (7)
        jp,                                # joint_velocities → 학습 데이터에서 동일 (7)
        np.zeros(7, dtype=np.float32),     # ee_pos_quat → 0 (7)
        np.array([gripper_position], dtype=np.float32),  # gripper (1)
    ])  # 22차원

    state_tensor = torch.from_numpy(state).unsqueeze(0)

    return {
        "observation.images.wrist": img_tensor,
        "observation.state": state_tensor,
        "task": [task],
    }


# ──────────────────────────────────────────────
# 관측(observation) 구성 — Octo (단일 프레임)
# ──────────────────────────────────────────────
def build_single_obs_octo(
    image: np.ndarray,
    joint_positions: np.ndarray,
    gripper_position: float,
    model,
) -> dict:
    """
    Octo 모델의 example_batch에 맞는 단일 프레임 관측 구성.

    RLDS 학습 데이터 (convert_gello_to_rlds.py 기준):
      observation/image_0: (480, 640, 3) uint8  → image_primary 로 매핑
      observation/state: (7,) float32  (6 joint + 1 gripper)
      action: (7,) float32

    이미지는 학습 시 256×256으로 리사이즈되었으므로 동일하게 처리.
    """
    from PIL import Image as PILImage

    # 이미지 리사이즈: 학습 시 primary는 256x256으로 리사이즈됨
    example_obs = model.example_batch["observation"]
    # image_primary 키의 shape에서 목표 크기 추출
    if "image_primary" in example_obs:
        target_h, target_w = example_obs["image_primary"].shape[-3:-1]
    else:
        target_h, target_w = 256, 256

    img_pil = PILImage.fromarray(image)
    img_resized = np.array(img_pil.resize((target_w, target_h), PILImage.LANCZOS))

    obs = {"image_primary": img_resized}

    # proprio/state가 있으면 추가 (학습 데이터에 포함되어 있었을 경우)
    if "proprio" in example_obs:
        state_dim = example_obs["proprio"].shape[-1]
        state = np.zeros(state_dim, dtype=np.float32)
        state[:min(6, state_dim)] = joint_positions[:min(6, state_dim)]
        if state_dim > 6:
            state[6] = gripper_position
        obs["proprio"] = state

    return obs


# ──────────────────────────────────────────────
# 메인 제어 루프
# ──────────────────────────────────────────────
def main():
    args = parse_args()
    model_type = args.model_type

    # ── 모델 로드 ──
    if model_type == "smolvla":
        policy, preprocessor, postprocessor, config = load_smolvla_policy(
            args.checkpoint, args.device
        )
    else:  # octo
        import jax
        octo_model, octo_policy_fn = load_octo_policy(args.checkpoint)
        # Octo 태스크 생성 (한 번만)
        octo_task = octo_model.create_tasks(texts=[args.task])
        # 관측 이력 관리
        obs_history = OctoObservationHistory(window_size=args.window_size)
        # action chunk 버퍼
        action_queue = deque()
        history_initialized = False

    # ── ROS2 초기화 (카메라 또는 로봇 인터페이스가 필요할 때) ──
    ros2_node = None
    if not args.dry_run:
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        ros2_node = rclpy.create_node(f"{model_type}_policy_runner")

    # ── 카메라 초기화 ──
    if args.dry_run:
        camera = DummyCamera()
    elif args.use_ros2_camera:
        # ROS2 노드가 필요하므로 초기화 후에 생성
        camera = ROS2Camera(node=ros2_node, topic=args.camera_topic, width=640, height=480, fps=30)
    else:
        camera = RealSenseCamera(width=640, height=480, fps=30)

    # ── 로봇 인터페이스 초기화 ──
    if args.dry_run:
        robot = DummyUR5Interface()
    else:
        robot = UR5ROS2Interface(ros2_node, args.robot_ip)

        # bridge 연결 대기 (status 토픽 수신 확인)
        logger.info("ur5_rtde_bridge 상태 확인 중...")
        bridge_found = False
        for i in range(30):
            robot.spin_once(timeout_sec=0.1)
            if robot.is_idle:
                bridge_found = True
                break
            if i == 0:
                logger.info(f"  Bridge 응답 대기 중... (status={robot._status})")
        if not bridge_found:
            logger.error("❌ bridge 상태를 확인할 수 없습니다!")
            logger.error("   확인: ur5_rtde_bridge.py가 실행 중인지 확인하세요.")
            logger.error("   확인: ros2 topic echo /ur5/status")
            logger.warning("   계속 진행하지만 로봇이 움직이지 않을 수 있습니다.")
        else:
            logger.info("✅ bridge 연결 확인 (IDLE)")
        
        # ROS2 토픽 확인
        import subprocess
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if "/ur5/goal_joint" in result.stdout:
                logger.info("✅ ROS2 토픽 확인: /ur5/goal_joint 존재")
            else:
                logger.warning("⚠️ ROS2 토픽 /ur5/goal_joint를 찾을 수 없습니다.")
        except Exception as e:
            logger.warning(f"ROS2 토픽 확인 실패: {e}")

    # ── 시작 포즈로 이동 ──
    if not args.dry_run and not args.no_start_pose:
        poses_path = Path(__file__).resolve().parent / "ur5_saved_poses.json"
        if poses_path.exists():
            with open(poses_path) as f:
                saved_poses = json.load(f)
            if args.start_pose in saved_poses:
                logger.info(f"시작 포즈 '{args.start_pose}'로 이동 중...")
                robot.go_to_pose(args.start_pose)
                time.sleep(0.5)
                for _ in range(100):
                    robot.spin_once(timeout_sec=0.1)
                    if robot.is_idle:
                        break
                time.sleep(1.0)
                logger.info("시작 포즈 도착")
            else:
                logger.warning(f"저장된 포즈 '{args.start_pose}'를 찾을 수 없습니다.")
        else:
            logger.warning("ur5_saved_poses.json 파일이 없습니다.")

    # 그리퍼 자동 캘리: 수집 구간은 메인 루프 안에서 처리 (calibrate_gripper_samples 리스트 사용)

    # ── 종료 핸들러 ──
    shutdown = False

    def signal_handler(sig, frame):
        nonlocal shutdown
        logger.info("\n종료 신호 수신. 로봇 정지 중...")
        shutdown = True

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ── 제어 루프 ──
    interval = 1.0 / args.fps
    logger.info("=" * 60)
    logger.info("정책 실행 시작!")
    logger.info(f"  모델: {model_type}")
    logger.info(f"  체크포인트: {args.checkpoint}")
    logger.info(f"  태스크: {args.task}")
    logger.info(f"  FPS: {args.fps}, 지속시간: {args.duration}초")
    if model_type == "octo":
        logger.info(f"  window_size: {args.window_size}, exec_horizon: {args.exec_horizon}")
    logger.info(f"  드라이런: {args.dry_run}")
    g_min = getattr(args, "gripper_min", 0.0)
    g_max = getattr(args, "gripper_max", 1.0)
    logger.info(f"  그리퍼: 반전={getattr(args, 'invert_gripper', False)}, 범위=[{g_min:.2f}, {g_max:.2f}]")
    if g_min == 0.0 and not args.dry_run:
        logger.warning("그리퍼 최소값이 0입니다. Gello 데이터는 닫힘 시에도 ~0.05 이상이라 0이면 하드웨어가 안 움직일 수 있습니다. scripts/convert_gello_to_lerobot.py 변환 시 출력되는 min을 --gripper-min으로 지정하세요.")
    logger.info("  Ctrl+C로 정지")
    logger.info("=" * 60)

    start_time = time.time()
    step = 0
    skipped = 0
    calibrate_gripper_samples = []
    calibrate_end_time = start_time + getattr(args, "calibrate_gripper_sec", 15.0)
    if not getattr(args, "calibrate_gripper", False) or args.dry_run:
        calibrate_end_time = 0  # 비활성
    elif calibrate_end_time > start_time:
        logger.info(f"그리퍼 자동 캘리: {getattr(args, 'calibrate_gripper_sec', 15):.0f}초 동안 값을 수집합니다.")

    try:
        while not shutdown and (time.time() - start_time) < args.duration:
            loop_start = time.perf_counter()

            # ROS2 콜백 처리 (status 업데이트)
            robot.spin_once(timeout_sec=0.0)

            # bridge가 MOVING이면 이번 스텝 스킵 (이전 moveJ 완료 대기)
            if not robot.is_idle:
                skipped += 1
                dt = time.perf_counter() - loop_start
                if dt < interval:
                    time.sleep(max(0, interval - dt))
                continue

            # ======================================
            # SmolVLA 추론 경로
            # ======================================
            if model_type == "smolvla":
                import torch

                # 1) 관측 수집
                image = camera.capture()
                joint_positions = robot.get_joint_positions()
                gripper_position = robot.get_gripper_position()
                if calibrate_end_time > 0 and time.time() < calibrate_end_time:
                    calibrate_gripper_samples.append(float(gripper_position))

                # 2) 관측 딕셔너리 구성
                obs = build_observation_smolvla(
                    image, joint_positions, gripper_position, args.task
                )

                # 3) 전처리 → 추론 → 후처리
                processed_obs = preprocessor(obs)
                with torch.no_grad():
                    actions = policy.select_action(processed_obs)
                actions = postprocessor(actions)
                actions = actions.squeeze(0).cpu().numpy()  # (8,)

                # 4) 액션 해석: [control_0..6, gripper_0]
                joint_target = actions[:6]
                gripper_cmd = actions[7] if len(actions) > 7 else actions[6]

            # ======================================
            # Octo 추론 경로
            # ======================================
            else:
                # 버퍼에 남은 action이 있으면 모델 호출 없이 실행
                if action_queue:
                    action = action_queue.popleft()
                    joint_target = action[:6]
                    gripper_cmd = action[6] if len(action) > 6 else 0.0
                else:
                    # 1) 관측 수집
                    image = camera.capture()
                    joint_positions = robot.get_joint_positions()
                    gripper_position = robot.get_gripper_position()
                    if calibrate_end_time > 0 and time.time() < calibrate_end_time:
                        calibrate_gripper_samples.append(float(gripper_position))

                    # 2) 단일 프레임 관측 구성
                    single_obs = build_single_obs_octo(
                        image, joint_positions, gripper_position, octo_model
                    )

                    # 3) 히스토리에 추가 (또는 초기화)
                    if not history_initialized:
                        stacked_obs = obs_history.reset(single_obs)
                        history_initialized = True
                    else:
                        stacked_obs = obs_history.add(single_obs)

                    # 4) 배치 차원 추가: (window, ...) → (1, window, ...)
                    batched_obs = jax.tree.map(lambda x: x[None], stacked_obs)

                    # 5) 모델 추론 → (1, action_horizon, action_dim)
                    raw_actions = octo_policy_fn(batched_obs, octo_task)
                    raw_actions = np.array(raw_actions[0])  # (action_horizon, action_dim)

                    # 6) exec_horizon 만큼 action queue에 넣기
                    for i in range(min(args.exec_horizon, len(raw_actions))):
                        action_queue.append(raw_actions[i])

                    # 7) 첫 번째 action 꺼내기
                    action = action_queue.popleft()
                    joint_target = action[:6]
                    gripper_cmd = action[6] if len(action) > 6 else 0.0

            # 그리퍼 자동 캘리: 수집 구간 끝나면 min/max 적용 (한 번만)
            if calibrate_end_time > 0 and time.time() >= calibrate_end_time and calibrate_gripper_samples:
                g_min_hw = min(calibrate_gripper_samples)
                g_max_hw = max(calibrate_gripper_samples)
                args.gripper_min = float(g_min_hw)
                args.gripper_max = float(g_max_hw)
                logger.info(f"그리퍼 자동 캘리 적용: min={args.gripper_min:.3f}, max={args.gripper_max:.3f} (샘플 {len(calibrate_gripper_samples)}개)")
                if args.gripper_min == args.gripper_max:
                    logger.warning("그리퍼 min==max. get_gripper_position() 구현 여부를 확인하세요.")
                calibrate_end_time = 0
                calibrate_gripper_samples.clear()

            # 그리퍼: 정책 [0,1] → 반전(옵션) → [gripper_min, gripper_max] 로 스케일.
            # Gello 데이터는 닫힘도 ~0.05 이상이므로 gripper_min=0 이면 "닫힘" 명령이 0이 되어 일부 하드웨어에서 안 움직임 → convert_gello_to_lerobot.py 출력 min/max 사용 권장.
            g_policy = float(np.clip(gripper_cmd, 0.0, 1.0))  # 정책 출력 [0,1]
            g = g_policy
            if getattr(args, "invert_gripper", False):
                g = 1.0 - g
            g_min = getattr(args, "gripper_min", 0.0)
            g_max = getattr(args, "gripper_max", 1.0)
            gripper_cmd_scaled = g_min + g * (g_max - g_min)  # 스케일링된 값

            # ── 공통: 관절 명령 퍼블리시 ──
            robot.send_joint_command(joint_target)
            
            # ── 공통: 그리퍼 명령 퍼블리시 ──
            robot.send_gripper_command(gripper_cmd_scaled)

            # ── 공통: 로깅 ──
            step += 1
            if step % 10 == 0:
                elapsed = time.time() - start_time
                extra = ""
                if model_type == "octo":
                    extra = f" | queue={len(action_queue)}"
                # 그리퍼 값 표시: 정책 출력 [0,1] → 스케일링된 값 [g_min, g_max]
                gripper_display = f"gripper={gripper_cmd_scaled:.3f}"
                if g_min != 0.0 or g_max != 1.0:
                    gripper_display += f" (정책:{g_policy:.3f}→[{g_min:.3f},{g_max:.3f}])"
                logger.info(
                    f"[Step {step:4d}] "
                    f"t={elapsed:.1f}s | "
                    f"target={np.array2string(joint_target, precision=3, suppress_small=True)} | "
                    f"{gripper_display} | "
                    f"skipped={skipped}{extra}"
                )
                skipped = 0

            # ── 공통: 타이밍 ──
            dt = time.perf_counter() - loop_start
            sleep_time = max(0, interval - dt)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        logger.error(f"오류 발생: {e}")
        import traceback
        traceback.print_exc()

    finally:
        logger.info("정리 중...")
        robot.stop()
        camera.stop()
        if ros2_node is not None:
            ros2_node.destroy_node()
            import rclpy
            rclpy.shutdown()
        logger.info(f"완료. 총 {step} 스텝 실행.")


if __name__ == "__main__":
    main()
