#!/usr/bin/env python3
"""
토이: observe 포즈로 간 다음 그리퍼만 반복해서 닫았다 열었다 하는 스크립트.

run_policy_ur5.py 와 동일하게 ur5_rtde_bridge.py 가 켜져 있어야 합니다.
먼저 ur5_saved_poses.json 의 observe(또는 --start-pose)로 이동한 뒤,
/ur5/gripper_cmd (Float64) 토픽으로 그리퍼 값을 보냅니다.
그리퍼가 실제로 움직이려면 /ur5/gripper_cmd 를 구독해서 하드웨어를 제어하는 노드가 필요합니다.

사용 예:
  # 터미널 1: bridge
  source /opt/ros/humble/setup.bash && python ur5_rtde_bridge.py

  # 터미널 2: 그리퍼 토이
  cd /home/lcw/RFM && source .venv/bin/activate
  source /opt/ros/humble/setup.bash
  python scripts/gripper_toy.py
  python scripts/gripper_toy.py --closed -1 --open 1   # -1~1 범위 (기본값)
  python scripts/gripper_toy.py --closed 0.05 --open 0.77   # Gello 0~1 범위일 때 (bridge는 gripper_mid:=0.5 로 실행)
  python scripts/gripper_toy.py --no-go-pose   # observe 이동 생략
"""

import argparse
import json
import time
from pathlib import Path

# RFM 루트 (scripts/ 한 단계 위)
RFM_ROOT = Path(__file__).resolve().parent.parent


def parse_args():
    p = argparse.ArgumentParser(description="그리퍼 열기/닫기 반복 (토이)")
    p.add_argument("--start-pose", type=str, default="observe",
                    help="먼저 이동할 포즈 이름 (ur5_saved_poses.json). run_policy_ur5.py 와 동일")
    p.add_argument("--no-go-pose", action="store_true",
                    help="시작 포즈로 이동하지 않음")
    p.add_argument("--closed", type=float, default=-1.0,
                    help="닫힘 명령 값 (-1~1 범위면 -1, Gello 0~1 범위면 0.05)")
    p.add_argument("--open", type=float, default=1.0,
                    help="열림 명령 값 (-1~1 범위면 1, Gello 0~1 범위면 0.77)")
    p.add_argument("--cycles", type=int, default=3,
                    help="닫기→열기 사이클 횟수")
    p.add_argument("--hold", type=float, default=2.0,
                    help="한 동작 유지 시간(초)")
    p.add_argument("--dry-run", action="store_true",
                    help="로봇/ROS2 없이 값만 출력")
    return p.parse_args()


def main():
    args = parse_args()
    closed = args.closed
    open_val = args.open
    cycles = args.cycles
    hold = args.hold
    dry = args.dry_run

    print("Gripper toy: observe 포즈로 간 뒤 닫기 ↔ 열기 반복")
    print(f"  start_pose={args.start_pose}, no_go_pose={args.no_go_pose}")
    print(f"  닫힘={closed:.3f}, 열림={open_val:.3f}, {cycles}회, 유지={hold}s, dry_run={dry}")
    print()

    if dry:
        for i in range(cycles):
            print(f"--- 사이클 {i + 1}/{cycles} ---")
            print(f"  [DRY-RUN] gripper = {closed:.3f}")
            time.sleep(0.1)
            print(f"  [DRY-RUN] gripper = {open_val:.3f}")
            time.sleep(0.1)
        print("끝.")
        return

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float64

    rclpy.init()
    node = rclpy.create_node("gripper_toy")
    pub_cmd = node.create_publisher(String, "/ur5/cmd", 10)
    pub_gripper = node.create_publisher(Float64, "/ur5/gripper_cmd", 10)

    status = {"value": "UNKNOWN"}

    def on_status(msg):
        status["value"] = msg.data or "UNKNOWN"

    node.create_subscription(String, "/ur5/status", on_status, 10)

    # bridge 쪽에서 status 보내기까지 잠시 대기
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if status["value"] != "UNKNOWN":
            break
    if status["value"] == "UNKNOWN":
        print("경고: /ur5/status 수신 없음. ur5_rtde_bridge.py 가 실행 중인지 확인하세요.")
        print("  확인: ros2 topic echo /ur5/status")
    else:
        print(f"[DEBUG] Bridge 연결 확인: status = {status['value']}")
    
    # 그리퍼 토픽 구독자 확인
    import subprocess
    gripper_subscriber_found = False
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "/ur5/gripper_cmd"],
            capture_output=True,
            text=True,
            timeout=2
        )
        if "Subscribers:" in result.stdout:
            lines = result.stdout.split("\n")
            subs = [line.strip() for line in lines if line.strip() and ("/" in line or "ur5" in line.lower())]
            if len(subs) > 1:
                print(f"[✓] /ur5/gripper_cmd 구독자 발견: {subs[1:]}")
                gripper_subscriber_found = True
            else:
                print("[✗] /ur5/gripper_cmd 구독자가 없습니다!")
                print("    → ur5_rtde_bridge.py가 실행 중인지 확인하세요.")
        else:
            print("[✗] /ur5/gripper_cmd 토픽 정보를 가져올 수 없습니다.")
    except Exception as e:
        print(f"[?] 토픽 정보 확인 실패 (무시 가능): {e}")
    
    print("\n" + "="*60)
    print("[그리퍼 동작 확인 방법]")
    print("="*60)
    print("1. Bridge 로그 확인 (Bridge 실행 터미널에서):")
    print("   ✓ 정상: 'gripper_cmd: X.XXX → Tool DO 0 = True/False'")
    print("   ✓ 정상: 'gripper_cmd: X.XXX → RobotiqGripper pos=XXX'")
    print("   ✗ 문제: 'gripper_cmd 수신했지만 그리퍼 제어 불가'")
    print("   ✗ 문제: 'RTDE IO unavailable' 또는 'RobotiqGripper 연결 실패'")
    print()
    print("2. 시각적 확인:")
    print("   - 그리퍼가 실제로 열리고 닫히는지 직접 확인하세요")
    print("   - 각 사이클마다 그리퍼가 움직여야 합니다")
    print()
    if not gripper_subscriber_found:
        print("[경고] Bridge가 /ur5/gripper_cmd를 구독하지 않습니다!")
        print("       그리퍼 명령이 전달되지 않을 수 있습니다.")
        print()
    print("="*60)
    print()

    # 1) observe(또는 start_pose)로 이동
    if not args.no_go_pose:
        poses_path = RFM_ROOT / "ur5_saved_poses.json"
        if poses_path.exists():
            with open(poses_path) as f:
                saved_poses = json.load(f)
            if args.start_pose in saved_poses:
                print(f"'{args.start_pose}' 포즈로 이동 중...")
                msg = String()
                msg.data = f"go {args.start_pose}"
                pub_cmd.publish(msg)
                # 이동 끝날 때까지 대기 (IDLE 될 때까지)
                for _ in range(150):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if status["value"] == "IDLE":
                        break
                time.sleep(0.5)
                print("포즈 도착.")
            else:
                print(f"경고: 저장된 포즈 '{args.start_pose}' 없음. 계속 진행합니다.")
        else:
            print("경고: ur5_saved_poses.json 없음. --no-go-pose 로 건너뛰거나 포즈를 저장하세요.")

    # 2) 그리퍼 닫기/열기 반복 (값은 /ur5/gripper_cmd 로 전송)
    def send_gripper(value: float, action_name: str = ""):
        msg = Float64()
        msg.data = float(value)
        pub_gripper.publish(msg)
        # 메시지가 전송되도록 여러 번 spin
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.01)
        print(f"    [→] /ur5/gripper_cmd = {value:.3f} 전송됨 {action_name}")
        print(f"    [→] Bridge 로그에서 'gripper_cmd: {value:.3f}' 확인하세요")
        time.sleep(0.2)  # Bridge가 명령을 처리할 시간

    print("\n그리퍼 테스트 시작...")
    print(f"닫힘 값: {closed:.3f}, 열림 값: {open_val:.3f}")
    print(f"각 동작을 {hold}초 동안 유지합니다.\n")
    
    for i in range(cycles):
        print(f"\n{'='*60}")
        print(f"사이클 {i + 1}/{cycles}")
        print(f"{'='*60}")
        
        print(f"\n[1/2] 닫기 명령 (값: {closed:.3f})")
        send_gripper(closed, "(닫기)")
        print(f"      → {hold}초 대기 중... (그리퍼가 닫히는지 확인하세요)")
        time.sleep(hold)
        
        print(f"\n[2/2] 열기 명령 (값: {open_val:.3f})")
        send_gripper(open_val, "(열기)")
        print(f"      → {hold}초 대기 중... (그리퍼가 열리는지 확인하세요)")
        time.sleep(hold)
        
        print(f"\n[완료] 사이클 {i + 1}/{cycles} 완료")
        
        if i < cycles - 1:
            print("      다음 사이클까지 1초 대기...")
            time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()
    
    print("\n" + "="*60)
    print("[테스트 완료]")
    print("="*60)
    print(f"총 {cycles}개 사이클 완료")
    print("\n[확인 사항]")
    print("1. Bridge 로그에서 각 명령이 성공적으로 처리되었는지 확인")
    print("2. 그리퍼가 실제로 열리고 닫혔는지 시각적으로 확인")
    print("3. 문제가 있다면:")
    print("   - Bridge 로그의 에러 메시지 확인")
    print("   - 'ros2 topic echo /ur5/gripper_cmd' 로 명령 전송 확인")
    print("   - Bridge의 RTDE IO 또는 RobotiqGripper 연결 상태 확인")
    print("="*60)


if __name__ == "__main__":
    main()
