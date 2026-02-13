#!/bin/bash
# 그리퍼 직접 테스트 스크립트

echo "=========================================="
echo "그리퍼 직접 테스트 (ROS2 명령)"
echo "=========================================="
echo ""
echo "Bridge가 실행 중이어야 합니다!"
echo ""

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

echo "[1] 닫기 명령 (0.0471)"
ros2 topic pub /ur5/gripper_cmd std_msgs/msg/Float64 "{data: 0.0471}" --once
echo "→ Bridge 로그 확인: 'gripper_cmd: 0.047'"
sleep 2

echo ""
echo "[2] 열기 명령 (0.7725)"
ros2 topic pub /ur5/gripper_cmd std_msgs/msg/Float64 "{data: 0.7725}" --once
echo "→ Bridge 로그 확인: 'gripper_cmd: 0.772'"
sleep 2

echo ""
echo "[3] 중간값 (0.4)"
ros2 topic pub /ur5/gripper_cmd std_msgs/msg/Float64 "{data: 0.4}" --once
echo "→ Bridge 로그 확인: 'gripper_cmd: 0.400'"
sleep 2

echo ""
echo "[4] 최소값 (0.0)"
ros2 topic pub /ur5/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}" --once
echo "→ Bridge 로그 확인: 'gripper_cmd: 0.000'"
sleep 2

echo ""
echo "[5] 최대값 (1.0)"
ros2 topic pub /ur5/gripper_cmd std_msgs/msg/Float64 "{data: 1.0}" --once
echo "→ Bridge 로그 확인: 'gripper_cmd: 1.000'"
sleep 2

echo ""
echo "=========================================="
echo "테스트 완료"
echo "=========================================="
echo "Bridge 로그에서 다음을 확인하세요:"
echo "  ✓ 'gripper_cmd: X.XXX → Tool DO 0 = True/False'"
echo "  ✓ 'gripper_cmd: X.XXX → RobotiqGripper pos=XXX'"
echo "  ✗ 'gripper_cmd 수신했지만 그리퍼 제어 불가'"
echo ""
