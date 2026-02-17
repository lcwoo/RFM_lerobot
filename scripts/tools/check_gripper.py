#!/usr/bin/env python3
"""
GELLO 데이터셋에서 그리퍼(gripper_position)의 최소/최대 범위를 계산합니다.

사용법:
  python scripts/check_gripper_range.py --source /home/bi_admin/bc_data/gello
"""

import argparse
import pickle
from pathlib import Path
from typing import Optional

import numpy as np


def load_pkl(path: Path) -> Optional[dict]:
    """pkl 파일 로드. 실패 시 None 반환."""
    try:
        with open(path, "rb") as f:
            return pickle.load(f)
    except Exception as e:
        print(f"  [WARN] Failed to load {path}: {e}")
        return None


def get_gripper_value(data: dict) -> Optional[float]:
    """데이터에서 그리퍼 값을 추출."""
    # gripper_position이 있으면 사용
    if "gripper_position" in data:
        gp = data["gripper_position"]
        if isinstance(gp, np.ndarray):
            if gp.size > 0:
                return float(gp[0] if gp.ndim > 0 else gp.item())
        elif isinstance(gp, (int, float)):
            return float(gp)
    
    # joint_positions의 마지막 요소가 그리퍼일 수 있음 (7-DOF: 6 arm + 1 gripper)
    if "joint_positions" in data:
        jp = data["joint_positions"]
        if isinstance(jp, np.ndarray) and len(jp) >= 7:
            return float(jp[6])  # 마지막 요소가 그리퍼
    
    return None


def main():
    parser = argparse.ArgumentParser(description="GELLO 데이터셋 그리퍼 범위 계산")
    parser.add_argument("--source", type=str, default="/home/bi_admin/bc_data/gello",
                        help="GELLO pkl 소스 디렉터리")
    parser.add_argument("--min-frames", type=int, default=1,
                        help="최소 프레임 수 (필터링용)")
    args = parser.parse_args()

    source_dir = Path(args.source).expanduser()
    if not source_dir.exists():
        print(f"❌ 소스 디렉터리 없음: {source_dir}")
        return

    print(f"📁 소스 디렉터리: {source_dir}")
    print(f"🔍 그리퍼 값 수집 중...\n")

    gripper_values = []
    total_pkls = 0
    valid_pkls = 0
    episodes_processed = 0

    # 모든 에피소드 폴더 순회
    for ep_dir in sorted(source_dir.iterdir()):
        if not ep_dir.is_dir():
            continue
        
        pkls = sorted(ep_dir.glob("*.pkl"))
        if len(pkls) < args.min_frames:
            print(f"  [SKIP] {ep_dir.name}: {len(pkls)} frames (< {args.min_frames})")
            continue
        
        episodes_processed += 1
        ep_gripper_values = []
        
        for pkl_path in pkls:
            total_pkls += 1
            data = load_pkl(pkl_path)
            if data is None:
                continue
            
            gval = get_gripper_value(data)
            if gval is not None:
                valid_pkls += 1
                ep_gripper_values.append(gval)
                gripper_values.append(gval)
        
        if ep_gripper_values:
            ep_min = min(ep_gripper_values)
            ep_max = max(ep_gripper_values)
            print(f"  [{ep_dir.name}] {len(ep_gripper_values)}/{len(pkls)} frames, "
                  f"gripper range: [{ep_min:.4f}, {ep_max:.4f}]")

    print()
    print("=" * 60)
    print("📊 전체 통계")
    print("=" * 60)
    print(f"  에피소드: {episodes_processed}개")
    print(f"  전체 pkl 파일: {total_pkls}개")
    print(f"  그리퍼 값 있는 파일: {valid_pkls}개 ({valid_pkls/total_pkls*100:.1f}%)")
    
    if not gripper_values:
        print("\n❌ 그리퍼 값을 찾을 수 없습니다.")
        print("   확인: pkl 파일에 'gripper_position' 또는 'joint_positions' (7-DOF)가 있는지 확인하세요.")
        return
    
    print(f"  그리퍼 값 샘플 수: {len(gripper_values)}개")
    print()
    
    gripper_array = np.array(gripper_values)
    print("📈 그리퍼 범위:")
    print(f"  최소값 (min): {np.min(gripper_array):.6f}")
    print(f"  최대값 (max): {np.max(gripper_array):.6f}")
    print(f"  평균값 (mean): {np.mean(gripper_array):.6f}")
    print(f"  중앙값 (median): {np.median(gripper_array):.6f}")
    print(f"  표준편차 (std): {np.std(gripper_array):.6f}")
    print()
    
    # 백분위수
    percentiles = [1, 5, 25, 50, 75, 95, 99]
    print("📊 백분위수:")
    for p in percentiles:
        val = np.percentile(gripper_array, p)
        print(f"  {p:2d}%: {val:.6f}")
    print()
    
    # 히스토그램 (간단한 버전)
    print("📊 분포 히스토그램 (10 bins):")
    hist, bins = np.histogram(gripper_array, bins=10)
    bin_width = bins[1] - bins[0]
    for i in range(len(hist)):
        bar = "█" * int(hist[i] / max(hist) * 40) if max(hist) > 0 else ""
        print(f"  [{bins[i]:.4f}, {bins[i+1]:.4f}): {hist[i]:5d} {bar}")
    print()
    
    print("=" * 60)
    print("💡 사용 예:")
    print("=" * 60)
    print(f"  # run_policy_ur5.py 또는 gripper_toy.py에서 사용:")
    print(f"  --gripper-min {np.min(gripper_array):.4f}")
    print(f"  --gripper-max {np.max(gripper_array):.4f}")
    print()
    print(f"  # gripper_toy.py에서 사용:")
    print(f"  --closed {np.min(gripper_array):.4f} --open {np.max(gripper_array):.4f}")
    print()


if __name__ == "__main__":
    main()
