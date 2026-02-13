#!/usr/bin/env python3
"""
Gello pkl 데이터셋을 LeRobot 포맷으로 변환합니다.
원본 파일은 수정하지 않고, RFM 안의 새 폴더에 데이터셋을 생성합니다.

에피소드별로 파일을 분리합니다:
  - data/chunk-000/episode_000000.parquet, episode_000001.parquet, ...
  - videos/observation.images.wrist/chunk-000/episode_000000.mp4, episode_000001.mp4, ...

사용 예:
  python scripts/convert_gello_to_lerobot.py \\
    --input /home/lcw/RFM/gello_data \\
    --output /home/lcw/RFM/datasets/gello_lerobot \\
    --fps 30

FPS: 타임스탬프 기준으로 샘플 에피소드에서 약 30 Hz로 측정됨. 100 Hz로 알고 계시면 --fps 100 로 지정.
"""

import argparse
import json
import pickle
import re
from pathlib import Path

import numpy as np

# LeRobot은 RFM/lerobot 또는 venv에 설치된 상태에서 실행
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import write_json


# Gello pkl 한 프레임당 키: wrist_rgb, wrist_depth, joint_positions, joint_velocities, ee_pos_quat, gripper_position, control
def build_lerobot_features():
    """LeRobot info.json에 들어갈 features 정의 (gello 스키마 기준)."""
    return {
        "observation.images.wrist": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (22,),
            "names": [
                "joint_positions_0",
                "joint_positions_1",
                "joint_positions_2",
                "joint_positions_3",
                "joint_positions_4",
                "joint_positions_5",
                "joint_positions_6",
                "joint_velocities_0",
                "joint_velocities_1",
                "joint_velocities_2",
                "joint_velocities_3",
                "joint_velocities_4",
                "joint_velocities_5",
                "joint_velocities_6",
                "ee_pos_quat_0",
                "ee_pos_quat_1",
                "ee_pos_quat_2",
                "ee_pos_quat_3",
                "ee_pos_quat_4",
                "ee_pos_quat_5",
                "ee_pos_quat_6",
                "gripper_position_0",
            ],
        },
        "action": {
            "dtype": "float32",
            "shape": (8,),
            "names": [
                "control_0",
                "control_1",
                "control_2",
                "control_3",
                "control_4",
                "control_5",
                "control_6",
                "gripper_0",
            ],
        },
    }


def pkl_frame_to_lerobot_frame(pkl_data: dict, task: str = "gello") -> dict:
    """Gello pkl 한 프레임을 LeRobot add_frame()에 넣을 frame dict로 변환."""
    # observation.images.wrist: (H, W, C) 유지 (LeRobot은 이미지 저장 시 그대로 사용)
    rgb = pkl_data["wrist_rgb"]
    if rgb.dtype != np.uint8:
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

    state = np.concatenate([
        np.asarray(pkl_data["joint_positions"], dtype=np.float32),
        np.asarray(pkl_data["joint_velocities"], dtype=np.float32),
        np.asarray(pkl_data["ee_pos_quat"], dtype=np.float32),
        np.asarray(pkl_data["gripper_position"], dtype=np.float32),
    ])
    action = np.concatenate([
        np.asarray(pkl_data["control"], dtype=np.float32),
        np.asarray(pkl_data["gripper_position"], dtype=np.float32),
    ])

    return {
        "observation.images.wrist": rgb,
        "observation.state": state,
        "action": action,
        "task": task,
    }


def timestamp_from_filename(name: str) -> float | None:
    """파일명에서 초 단위 타임스탬프 추출. 예: 2026-02-06T21:37:05.115988.pkl"""
    m = re.search(r"(\d{2}):(\d{2}):(\d{2})\.(\d+)", name)
    if not m:
        return None
    h, mi, s, sub = m.groups()
    return int(h) * 3600 + int(mi) * 60 + int(s) + int(sub) / 1e6


def list_episode_dirs(input_root: Path) -> list[Path]:
    """에피소드 디렉토리만 나열 (이름이 숫자_숫자 형태인 폴더)."""
    dirs = [d for d in input_root.iterdir() if d.is_dir()]
    return sorted(dirs, key=lambda p: p.name)


def main():
    parser = argparse.ArgumentParser(description="Convert Gello pkl dataset to LeRobot format.")
    parser.add_argument("--input", type=Path, default=Path("/home/lcw/RFM/gello_data"), help="Gello 데이터셋 루트 (RFM/gello_data 또는 외부 경로)")
    parser.add_argument("--output", type=Path, default=Path("/home/lcw/RFM/datasets/gello_lerobot"), help="LeRobot 데이터셋이 생성될 디렉토리 (repo_id로 사용)")
    parser.add_argument("--fps", type=int, default=30, help="데이터 수집 시 fps. 타임스탬프 기준으로 약 30 Hz 측정됨. 100 Hz면 100 지정.")
    parser.add_argument(
        "--task",
        type=str,
        default="gello",
        help='에피소드 task(명령) 문자열. 예: "Pick up the eggplant and place it on the plate."',
    )
    parser.add_argument("--max-episodes", type=int, default=None, help="변환할 최대 에피소드 수 (테스트용)")
    args = parser.parse_args()

    input_root = args.input.resolve()
    output_root = args.output.resolve()
    if not input_root.exists():
        raise FileNotFoundError(f"Input directory not found: {input_root}")

    # LeRobot 데이터셋이 output_root 에 직접 생성됨 (원본 gello 폴더는 건드리지 않음)
    repo_id = output_root.name
    root = output_root
    if root.exists():
        raise FileExistsError(
            f"Output path already exists. Use an empty folder or remove first: {output_root}"
        )

    features = build_lerobot_features()
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=args.fps,
        features=features,
        root=root,
        robot_type="gello",
        use_videos=True,
        batch_encoding_size=1,
        vcodec="h264",
    )
    # 에피소드당 파일 하나씩 생성되도록 용량 제한을 극히 작게 설정
    dataset.meta.update_chunk_settings(
        data_files_size_in_mb=0.001,
        video_files_size_in_mb=0.001,
    )

    episode_dirs = list_episode_dirs(input_root)
    if args.max_episodes is not None:
        episode_dirs = episode_dirs[: args.max_episodes]

    print(f"Converting {len(episode_dirs)} episodes from {input_root} -> {output_root} (fps={args.fps})")

    for ep_idx, ep_dir in enumerate(episode_dirs):
        pkl_files = sorted(ep_dir.glob("*.pkl"), key=lambda p: timestamp_from_filename(p.name) or 0)
        if not pkl_files:
            print(f"  Skip empty episode: {ep_dir.name}")
            continue

        for pkl_path in pkl_files:
            with open(pkl_path, "rb") as f:
                pkl_data = pickle.load(f)
            frame = pkl_frame_to_lerobot_frame(pkl_data, task=args.task)
            dataset.add_frame(frame)
        dataset.save_episode()
        if (ep_idx + 1) % 5 == 0 or ep_idx == 0:
            print(f"  Saved episode {ep_idx + 1}/{len(episode_dirs)}: {ep_dir.name} ({len(pkl_files)} frames)")

    dataset.finalize()

    # file-XXX -> episode_XXXXXX 로 이름 변경 후 info.json 경로 패턴 수정
    num_episodes = dataset.meta.total_episodes
    _rename_to_episode_files(dataset.root, num_episodes=num_episodes)
    print(f"Done. LeRobot dataset at: {dataset.root} ({num_episodes} episodes, one parquet + one mp4 per episode)")


def _rename_to_episode_files(root: Path, num_episodes: int) -> None:
    """data/ 및 videos/ 아래 file-XXX 파일을 episode_XXXXXX 로 바꾸고 info.json 경로 패턴 수정."""
    root = Path(root)
    # data/chunk-000/file-000.parquet -> episode_000000.parquet (에피소드당 1파일)
    data_chunk = root / "data" / "chunk-000"
    if data_chunk.exists():
        for i in range(num_episodes):
            old_name = f"file-{i:03d}.parquet"
            new_name = f"episode_{i:06d}.parquet"
            old_path = data_chunk / old_name
            new_path = data_chunk / new_name
            if old_path.exists():
                old_path.rename(new_path)

    # videos/<video_key>/chunk-000/file-XXX.mp4 -> episode_XXXXXX.mp4
    videos_dir = root / "videos"
    if videos_dir.exists():
        for video_key_dir in videos_dir.iterdir():
            if video_key_dir.is_dir():
                chunk_dir = video_key_dir / "chunk-000"
                if chunk_dir.exists():
                    for i in range(num_episodes):
                        old_name = f"file-{i:03d}.mp4"
                        new_name = f"episode_{i:06d}.mp4"
                        old_path = chunk_dir / old_name
                        new_path = chunk_dir / new_name
                        if old_path.exists():
                            old_path.rename(new_path)

    # meta/info.json: data_path, video_path 를 episode_{file_index:06d} 패턴으로 변경 (로더가 file_index로 경로 생성)
    info_path = root / "meta" / "info.json"
    if info_path.exists():
        with open(info_path) as f:
            info = json.load(f)
        info["data_path"] = "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet"
        info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4"
        write_json(info, info_path)


if __name__ == "__main__":
    main()
