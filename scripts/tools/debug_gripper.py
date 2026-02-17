#!/usr/bin/env python3
"""
그리퍼 동작 문제 디버깅 스크립트

확인 사항:
1. ROS2 토픽에서 그리퍼 명령이 발행되는지
2. Bridge가 명령을 받고 있는지
3. Bridge의 그리퍼 인터페이스 상태
4. 실제 그리퍼 하드웨어 연결 상태
"""

import sys
import time
import argparse
from pathlib import Path

RFM_ROOT = Path(__file__).resolve().parent.parent


def check_ros2_topic():
    """ROS2 토픽 확인"""
    print("=" * 60)
    print("[1] ROS2 토픽 확인")
    print("=" * 60)
    
    import subprocess
    
    # 토픽 존재 확인
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=2
        )
        if "/ur5/gripper_cmd" in result.stdout:
            print("✓ /ur5/gripper_cmd 토픽 존재")
        else:
            print("✗ /ur5/gripper_cmd 토픽 없음")
            return False
    except Exception as e:
        print(f"✗ ROS2 토픽 확인 실패: {e}")
        return False
    
    # 구독자 확인
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "/ur5/gripper_cmd"],
            capture_output=True,
            text=True,
            timeout=2
        )
        print("\n토픽 정보:")
        print(result.stdout)
        
        if "Subscribers:" in result.stdout:
            lines = result.stdout.split("\n")
            for line in lines:
                if "ur5" in line.lower() or "/" in line:
                    print(f"  → {line.strip()}")
    except Exception as e:
        print(f"✗ 토픽 정보 확인 실패: {e}")
    
    return True


def check_bridge_logs():
    """Bridge 로그 확인 안내"""
    print("\n" + "=" * 60)
    print("[2] Bridge 로그 확인")
    print("=" * 60)
    print("Bridge 실행 터미널에서 다음을 확인하세요:")
    print()
    print("✓ 정상 동작:")
    print("  - 'gripper_cmd: X.XXX → Tool DO 0 = True/False'")
    print("  - 'gripper_cmd: X.XXX → RobotiqGripper pos=XXX'")
    print()
    print("✗ 문제 상황:")
    print("  - 'gripper_cmd 수신했지만 그리퍼 제어 불가'")
    print("  - 'RTDE IO unavailable' 또는 'RobotiqGripper 연결 실패'")
    print("  - 'rtde_io=None, robotiq_gripper=None'")
    print()


def test_gripper_command():
    """그리퍼 명령 테스트"""
    print("=" * 60)
    print("[3] 그리퍼 명령 테스트")
    print("=" * 60)
    
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64
        
        rclpy.init()
        node = rclpy.create_node("gripper_debug")
        pub = node.create_publisher(Float64, "/ur5/gripper_cmd", 10)
        
        print("그리퍼 명령 전송 테스트...")
        print("Bridge 로그를 확인하세요.\n")
        
        # 테스트 값들
        test_values = [
            (0.0471, "닫힘 (최소값)"),
            (0.7725, "열림 (최대값)"),
            (0.5, "중간값"),
        ]
        
        for value, desc in test_values:
            print(f"전송: {value:.3f} ({desc})")
            msg = Float64()
            msg.data = float(value)
            pub.publish(msg)
            
            # 메시지 전송 대기
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
            
            time.sleep(1.0)
            print("  → Bridge 로그 확인: 'gripper_cmd: ...' 메시지가 있는지 확인")
            print()
        
        node.destroy_node()
        rclpy.shutdown()
        
    except Exception as e:
        print(f"✗ 테스트 실패: {e}")
        import traceback
        traceback.print_exc()


def check_bridge_interface():
    """Bridge 인터페이스 상태 확인"""
    print("=" * 60)
    print("[4] Bridge 인터페이스 상태 확인")
    print("=" * 60)
    print("Bridge 코드에서 확인할 사항:")
    print()
    print("1. RTDE IO 상태:")
    print("   - ur5_rtde_bridge.py 초기화 시 'RTDE IO 사용 중' 또는 'RTDE IO unavailable'")
    print("   - self.rtde_io가 None인지 확인")
    print()
    print("2. RobotiqGripper 상태:")
    print("   - 'RobotiqGripper 연결 성공' 또는 'RobotiqGripper 연결 실패'")
    print("   - self.robotiq_gripper가 None인지 확인")
    print()
    print("3. 그리퍼 mid 값:")
    print("   - gripper_mid 파라미터 값 확인")
    print("   - 0.047~0.772 범위에서 적절한 mid 값 설정 필요")
    print()


def check_gripper_range():
    """그리퍼 값 범위 확인"""
    print("=" * 60)
    print("[5] 그리퍼 값 범위 확인")
    print("=" * 60)
    
    print("현재 사용 중인 범위:")
    print("  --gripper-min 0.0471")
    print("  --gripper-max 0.7725")
    print()
    print("문제 가능성:")
    print("  1. 범위가 실제 하드웨어와 맞지 않을 수 있음")
    print("  2. Bridge의 gripper_mid 값이 범위 밖일 수 있음")
    print("  3. RobotiqGripper: 0~255 범위로 변환 시 문제")
    print()
    print("해결 방법:")
    print("  1. scripts/gripper_toy.py로 실제 동작 범위 확인")
    print("  2. Bridge 실행 시 gripper_mid 파라미터 조정:")
    print("     --ros-args -p gripper_mid:=0.4")
    print("  3. RobotiqGripper 직접 테스트:")
    print("     python -c 'from gello.robots.robotiq_gripper import RobotiqGripper; g=RobotiqGripper(); g.connect(\"192.168.0.43\", 63352); g.move(128, 255, 10)'")
    print()


def main():
    parser = argparse.ArgumentParser(description="그리퍼 디버깅 도구")
    parser.add_argument("--test-command", action="store_true",
                        help="그리퍼 명령 테스트 실행")
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("그리퍼 동작 문제 디버깅")
    print("=" * 60 + "\n")
    
    # 1. ROS2 토픽 확인
    check_ros2_topic()
    
    # 2. Bridge 로그 확인 안내
    check_bridge_logs()
    
    # 3. 그리퍼 명령 테스트 (옵션)
    if args.test_command:
        test_gripper_command()
    else:
        print("=" * 60)
        print("[3] 그리퍼 명령 테스트 (건너뜀)")
        print("=" * 60)
        print("테스트하려면: python scripts/debug_gripper.py --test-command")
        print()
    
    # 4. Bridge 인터페이스 상태 확인
    check_bridge_interface()
    
    # 5. 그리퍼 값 범위 확인
    check_gripper_range()
    
    print("=" * 60)
    print("다음 단계:")
    print("=" * 60)
    print("1. Bridge 로그에서 그리퍼 명령 수신 여부 확인")
    print("2. Bridge의 rtde_io 또는 robotiq_gripper 상태 확인")
    print("3. scripts/gripper_toy.py로 직접 테스트")
    print("4. Bridge 실행 시 gripper_mid 파라미터 조정")
    print()


if __name__ == "__main__":
    main()
