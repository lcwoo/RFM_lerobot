#!/usr/bin/env python3
"""
Convert Gello pickle datasets to LeRobot format.

This script reads raw Gello demonstration data (per-frame .pkl files organized
by episode) and produces a fully compliant LeRobot dataset with:
  - Per-episode Parquet files for state/action data
  - Per-episode MP4 videos for wrist camera observations

The original Gello data is never modified; all outputs are written to a new
directory specified by --output.

Output structure:
  <output>/
    meta/info.json
    data/chunk-000/episode_000000.parquet, episode_000001.parquet, ...
    videos/observation.images.wrist/chunk-000/episode_000000.mp4, ...

Usage:
  python scripts/convert_gello_to_lerobot.py \\
      --input /home/lcw/RFM/gello_data \\
      --output /home/lcw/RFM/datasets/gello_lerobot \\
      --fps 30

Note on FPS:
  Measured ~30 Hz from sample episode timestamps. If your setup records at
  a different rate (e.g. 100 Hz), pass --fps 100.
"""

import argparse
import json
import pickle
import re
from pathlib import Path

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import write_json


# ---------------------------------------------------------------------------
# Schema definition
# ---------------------------------------------------------------------------

# Each Gello .pkl frame contains the following keys:
#   wrist_rgb          (H, W, 3)  - RGB image from wrist camera
#   wrist_depth        (H, W)     - depth map (unused in this conversion)
#   joint_positions    (7,)       - 7-DOF joint positions
#   joint_velocities   (7,)       - 7-DOF joint velocities
#   ee_pos_quat        (7,)       - end-effector pose (position + quaternion)
#   gripper_position   (1,)       - gripper opening
#   control            (7,)       - commanded joint-level action


def build_lerobot_features() -> dict:
    """Define the LeRobot feature schema matching the Gello observation/action space.

    Returns a dict suitable for LeRobotDataset.create(features=...).
    """
    return {
        # Wrist camera RGB stream, encoded as video
        "observation.images.wrist": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        },
        # Proprioceptive state: [joint_pos(7), joint_vel(7), ee_pose(7), gripper(1)] = 22
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
        # Action: [control(7), gripper(1)] = 8
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


# ---------------------------------------------------------------------------
# Frame conversion
# ---------------------------------------------------------------------------


def pkl_frame_to_lerobot_frame(pkl_data: dict, task: str = "gello") -> dict:
    """Convert a single Gello .pkl frame into a LeRobot-compatible frame dict.

    Args:
        pkl_data: Raw data loaded from a Gello .pkl file.
        task: Language task description attached to every frame.

    Returns:
        A dict ready to be passed to ``LeRobotDataset.add_frame()``.
    """
    # Ensure RGB image is uint8 (Gello sometimes stores float [0, 1])
    rgb = pkl_data["wrist_rgb"]
    if rgb.dtype != np.uint8:
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)

    # Concatenate all proprioceptive signals into a single state vector
    state = np.concatenate([
        np.asarray(pkl_data["joint_positions"], dtype=np.float32),
        np.asarray(pkl_data["joint_velocities"], dtype=np.float32),
        np.asarray(pkl_data["ee_pos_quat"], dtype=np.float32),
        np.asarray(pkl_data["gripper_position"], dtype=np.float32),
    ])

    # Action = joint control command + gripper
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


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------


def timestamp_from_filename(name: str) -> float | None:
    """Extract a seconds-precision timestamp from a Gello filename.

    Expected format: ``2026-02-06T21:37:05.115988.pkl``

    Returns:
        Timestamp in seconds (float), or None if the pattern doesn't match.
    """
    m = re.search(r"(\d{2}):(\d{2}):(\d{2})\.(\d+)", name)
    if not m:
        return None
    h, mi, s, sub = m.groups()
    return int(h) * 3600 + int(mi) * 60 + int(s) + int(sub) / 1e6


def list_episode_dirs(input_root: Path) -> list[Path]:
    """Return sorted list of episode directories under *input_root*.

    Gello stores episodes as subdirectories (e.g. ``1_1/``, ``2_1/``).
    Only immediate child directories are considered.
    """
    dirs = [d for d in input_root.iterdir() if d.is_dir()]
    return sorted(dirs, key=lambda p: p.name)


# ---------------------------------------------------------------------------
# Post-processing: rename auto-generated files to per-episode convention
# ---------------------------------------------------------------------------


def _rename_to_episode_files(root: Path, num_episodes: int) -> None:
    """Rename LeRobot's default ``file-XXX`` outputs to ``episode_XXXXXX``.

    LeRobot internally names chunk files as ``file-000.parquet``, etc.
    This function renames them to the more descriptive ``episode_000000``
    pattern and updates ``meta/info.json`` path templates accordingly.

    Args:
        root: Root directory of the generated LeRobot dataset.
        num_episodes: Total number of episodes to rename.
    """
    root = Path(root)

    # --- Rename Parquet files ---
    data_chunk = root / "data" / "chunk-000"
    if data_chunk.exists():
        for i in range(num_episodes):
            old = data_chunk / f"file-{i:03d}.parquet"
            new = data_chunk / f"episode_{i:06d}.parquet"
            if old.exists():
                old.rename(new)

    # --- Rename video files ---
    videos_dir = root / "videos"
    if videos_dir.exists():
        for video_key_dir in videos_dir.iterdir():
            if not video_key_dir.is_dir():
                continue
            chunk_dir = video_key_dir / "chunk-000"
            if not chunk_dir.exists():
                continue
            for i in range(num_episodes):
                old = chunk_dir / f"file-{i:03d}.mp4"
                new = chunk_dir / f"episode_{i:06d}.mp4"
                if old.exists():
                    old.rename(new)

    # --- Update info.json path templates ---
    # LeRobot's loader constructs file paths using {chunk_index} and
    # {file_index} format variables, so we update the templates to match
    # our new naming convention.
    info_path = root / "meta" / "info.json"
    if info_path.exists():
        with open(info_path) as f:
            info = json.load(f)
        info["data_path"] = "data/chunk-{chunk_index:03d}/episode_{file_index:06d}.parquet"
        info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/episode_{file_index:06d}.mp4"
        write_json(info, info_path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Convert Gello pkl dataset to LeRobot format.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=Path("/home/lcw/RFM/gello_data"),
        help="Root directory of the raw Gello dataset (default: %(default)s)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/lcw/RFM/datasets/gello_lerobot"),
        help="Destination directory for the LeRobot dataset (default: %(default)s)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Recording frequency in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--task",
        type=str,
        default="gello",
        help='Task description string for each episode (e.g. "Pick up the eggplant")',
    )
    parser.add_argument(
        "--max-episodes",
        type=int,
        default=None,
        help="Limit the number of episodes to convert (useful for testing)",
    )
    args = parser.parse_args()

    input_root = args.input.resolve()
    output_root = args.output.resolve()

    if not input_root.exists():
        raise FileNotFoundError(f"Input directory not found: {input_root}")
    if output_root.exists():
        raise FileExistsError(
            f"Output directory already exists — remove it first or choose a new path: {output_root}"
        )

    # ---- Initialize LeRobot dataset ----
    features = build_lerobot_features()
    dataset = LeRobotDataset.create(
        repo_id=output_root.name,
        fps=args.fps,
        features=features,
        root=output_root,
        robot_type="gello",
        use_videos=True,
        batch_encoding_size=1,
        vcodec="h264",
    )

    # Force one file per episode by setting tiny chunk size limits
    dataset.meta.update_chunk_settings(
        data_files_size_in_mb=0.001,
        video_files_size_in_mb=0.001,
    )

    # ---- Discover and sort episodes ----
    episode_dirs = list_episode_dirs(input_root)
    if args.max_episodes is not None:
        episode_dirs = episode_dirs[: args.max_episodes]

    print(
        f"Converting {len(episode_dirs)} episodes: {input_root} -> {output_root} "
        f"(fps={args.fps})"
    )

    # ---- Convert each episode ----
    for ep_idx, ep_dir in enumerate(episode_dirs):
        # Sort frames by timestamp embedded in the filename
        pkl_files = sorted(
            ep_dir.glob("*.pkl"),
            key=lambda p: timestamp_from_filename(p.name) or 0,
        )
        if not pkl_files:
            print(f"  [skip] Empty episode directory: {ep_dir.name}")
            continue

        for pkl_path in pkl_files:
            with open(pkl_path, "rb") as f:
                pkl_data = pickle.load(f)
            frame = pkl_frame_to_lerobot_frame(pkl_data, task=args.task)
            dataset.add_frame(frame)

        dataset.save_episode()

        # Progress logging (first episode + every 5th)
        if ep_idx == 0 or (ep_idx + 1) % 5 == 0:
            print(
                f"  [{ep_idx + 1}/{len(episode_dirs)}] {ep_dir.name} "
                f"({len(pkl_files)} frames)"
            )

    dataset.finalize()

    # ---- Post-process: rename files to episode-based naming ----
    num_episodes = dataset.meta.total_episodes
    _rename_to_episode_files(dataset.root, num_episodes=num_episodes)

    print(
        f"\nDone! LeRobot dataset saved to: {dataset.root}\n"
        f"  Episodes : {num_episodes}\n"
        f"  Format   : 1 parquet + 1 mp4 per episode"
    )


if __name__ == "__main__":
    main()