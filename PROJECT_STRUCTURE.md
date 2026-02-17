# RFM 프로젝트 구조 재구성 제안

## 현재 문제점

- 루트에 실행 스크립트들이 흩어져 있음 (`run_policy_ur5.py`, `ur5_rtde_bridge.py` 등)
- 유틸리티 파일이 루트에 있음 (`math_utils.py`, `utils.py`)
- ROS2 노드들이 루트에 있음 (`gsam_cutie_tracker_node.py`, `pose_tracker_action_node.py` 등)
- `scripts/` 디렉터리가 있지만 일부만 포함

## 제안하는 계층적 구조

```
RFM/
├── rfm/                          # 메인 Python 패키지
│   ├── __init__.py
│   ├── robots/                   # 로봇 인터페이스
│   │   ├── __init__.py
│   │   ├── ur5_bridge.py         # ur5_rtde_bridge.py → 이동
│   │   └── ur5_robot.py           # UR5 래퍼 클래스 (필요시)
│   ├── policies/                 # 정책 실행
│   │   ├── __init__.py
│   │   ├── runner.py             # run_policy_ur5.py → 이동
│   │   └── base.py                # 기본 정책 인터페이스
│   ├── utils/                     # 유틸리티
│   │   ├── __init__.py
│   │   ├── math.py                # math_utils.py → 이동
│   │   └── common.py              # utils.py → 이동
│   └── configs/                   # 설정 관리 (선택)
│       └── default.yaml
│
├── scripts/                        # 실행 스크립트 (CLI 진입점)
│   ├── data/                      # 데이터 관련
│   │   ├── collect_gello.py       # Gello 데이터 수집 래퍼
│   │   └── convert_to_lerobot.py # convert_gello_to_lerobot.py → 이동
│   ├── training/                  # 학습 관련 (필요시)
│   └── tools/                     # 도구/디버그
│       ├── check_gripper.py      # check_gripper_range.py → 이동
│       ├── debug_gripper.py      # debug_gripper.py → 이동
│       └── gripper_toy.py         # gripper_toy.py → 이동
│
├── launch/                         # ROS2 launch 파일들
│   ├── ur5_bridge.launch.py      # UR5 브리지 실행
│   ├── gsam_tracker.launch.py    # gsam_cutie_tracker_node → 이동
│   └── pose_tracker.launch.py    # pose_tracker_action_node → 이동
│
├── nodes/                          # ROS2 노드 (또는 rfm/nodes/)
│   ├── gsam_cutie_tracker.py     # gsam_cutie_tracker_node.py → 이동
│   ├── pose_tracker_action.py    # pose_tracker_action_node.py → 이동
│   └── publish_static_tf.py       # publish_static_tf.py → 이동
│
├── configs/                        # 설정 파일
│   ├── ur5_bridge.yaml            # UR5 브리지 설정
│   └── policy.yaml                # 정책 실행 설정
│
├── docs/                           # 문서
│   ├── README.md                  # 메인 README (그대로)
│   ├── SETUP.md                   # 설치 가이드
│   └── newbie                     # newbie → 이동 (또는 docs/quickstart.md)
│
├── tests/                          # 테스트
│   ├── test_ur5_bridge.py         # ur5_rtde_bridge_test.py → 이동
│   └── ...
│
├── thirdparty/                     # 서드파티 (그대로)
├── datasets/                       # 데이터셋 (그대로)
├── outputs/                        # 학습 출력 (그대로)
├── gello_software/                 # Gello 서브모듈 (그대로)
├── lerobot/                        # LeRobot 서브모듈 (그대로)
├── octo/                           # Octo 서브모듈 (그대로)
│
├── setup.py                        # 패키지 설치 (필요시)
├── requirements.txt                # 의존성 (그대로)
├── .gitignore                      # 그대로
└── README.md                       # 그대로
```

## 파일 이동 계획

### 1. Python 패키지로 이동
- `ur5_rtde_bridge.py` → `rfm/robots/ur5_bridge.py`
- `run_policy_ur5.py` → `rfm/policies/runner.py`
- `math_utils.py` → `rfm/utils/math.py`
- `utils.py` → `rfm/utils/common.py`

### 2. 스크립트로 이동
- `scripts/convert_gello_to_lerobot.py` → `scripts/data/convert_to_lerobot.py`
- `scripts/check_gripper_range.py` → `scripts/tools/check_gripper.py`
- `scripts/debug_gripper.py` → `scripts/tools/debug_gripper.py`
- `scripts/gripper_toy.py` → `scripts/tools/gripper_toy.py`

### 3. ROS2 노드로 이동
- `gsam_cutie_tracker_node.py` → `nodes/gsam_cutie_tracker.py`
- `pose_tracker_action_node.py` → `nodes/pose_tracker_action.py`
- `publish_static_tf.py` → `nodes/publish_static_tf.py`

### 4. 문서로 이동
- `newbie` → `docs/quickstart.md` 또는 `docs/newbie`

### 5. 테스트로 이동
- `ur5_rtde_bridge_test.py` → `tests/test_ur5_bridge.py`

### 6. 유지할 파일 (루트)
- `README.md`
- `requirements.txt`
- `.gitignore`
- `setup.py` (새로 생성)
- `run_all.sh` (또는 `scripts/run_all.sh`로 이동)

## 실행 방법 변경

### Before
```bash
python run_policy_ur5.py --model-type smolvla ...
python ur5_rtde_bridge.py
```

### After (옵션 1: 패키지로 설치)
```bash
pip install -e .
rfm-run-policy --model-type smolvla ...
rfm-ur5-bridge
```

### After (옵션 2: 모듈로 실행, 권장)
```bash
python -m rfm.policies.runner --model-type smolvla ...
python -m rfm.robots.ur5_bridge
```

### After (옵션 3: 스크립트 래퍼)
```bash
scripts/run_policy.sh --model-type smolvla ...
scripts/run_ur5_bridge.sh
```

## 장점

1. **명확한 책임 분리**: robots/, policies/, utils/ 등으로 역할 구분
2. **재사용성**: 패키지로 설치하면 다른 프로젝트에서도 import 가능
3. **확장성**: 새로운 로봇/정책 추가 시 해당 디렉터리에만 추가
4. **표준 구조**: Python 프로젝트 표준 구조 준수
5. **테스트 용이**: tests/ 디렉터리로 테스트 분리

## 마이그레이션 체크리스트

- [ ] `rfm/` 패키지 구조 생성
- [ ] 파일 이동 및 import 경로 수정
- [ ] `setup.py` 작성 (entry_points로 CLI 명령 추가)
- [ ] `__init__.py` 파일들 생성
- [ ] README.md 업데이트 (새 경로 반영)
- [ ] `newbie` 문서 경로 업데이트
- [ ] 테스트 실행하여 import 경로 확인
- [ ] Git 커밋 (파일 이동은 `git mv` 사용)
