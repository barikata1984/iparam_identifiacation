#!/usr/bin/env python3
"""
Preview an excitation trajectory in RViz.

Loads a trajectory JSON (metadata + trajectory [{t, q, dq, ddq}])
and publishes joint states to /preview/joint_states for RViz visualization.

Usage:
    1. Launch RViz with the robot model:
         roslaunch iparam_identification preview_trajectory.launch
    2. Run this script:
         rosrun iparam_identification preview_excitation_trajectory.py [--speed 0.2]
    3. Press Enter to start playback, 'r' to replay, 'q' to quit.
"""

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rospy

IPARAM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(IPARAM_ROOT / "src"))

from utilities.preview_base import TrajectoryPreviewBase  # noqa: E402

DEFAULT_TRAJECTORY = str(IPARAM_ROOT / "data" / "trajectories" / "excitation_trajectory.json")


class ExcitationTrajectoryPreviewNode(TrajectoryPreviewBase):
    """Preview excitation trajectory in RViz."""

    def __init__(self, trajectory_path: str, speed: float = 1.0):
        super().__init__("excitation_trajectory_preview")
        self.speed = speed
        self._load_trajectory(trajectory_path)

    def _load_trajectory(self, trajectory_path: str) -> None:
        with open(trajectory_path, "r") as f:
            data = json.load(f)

        metadata = data["metadata"]
        trajectory = data["trajectory"]

        self.times = np.array([step["t"] for step in trajectory])
        self.positions = np.array([step["q"] for step in trajectory])
        self.velocities = np.array([step["dq"] for step in trajectory])
        self.duration = metadata["duration"]

        # Print trajectory info
        q_min_deg = np.rad2deg(self.positions.min(axis=0))
        q_max_deg = np.rad2deg(self.positions.max(axis=0))
        q0_deg = np.rad2deg(self.positions[0])
        dq_max = np.abs(self.velocities).max(axis=0)
        print(f"Loaded: {trajectory_path}")
        print(f"  Duration: {self.duration}s, Steps: {len(trajectory)}, dt: {metadata['dt']}s")
        print(f"  Playback speed: {self.speed}x (real time: {self.duration / self.speed:.1f}s)")
        print(f"  Condition number: {metadata.get('condition_number', 'N/A')}")
        print(f"  q0 (deg): {q0_deg.round(1).tolist()}")
        print(f"  q_min (deg): {q_min_deg.round(1).tolist()}")
        print(f"  q_max (deg): {q_max_deg.round(1).tolist()}")
        print(f"  |dq|_max (rad/s): {dq_max.round(2).tolist()}")

    def _print_commands(self) -> None:
        print("\n" + "=" * 60)
        print("Excitation Trajectory Preview")
        print("=" * 60)
        print("Commands:")
        print("  [Enter] - Play trajectory")
        print("  [s]     - Show start position")
        print("  [e]     - Show end position")
        print("  [r]     - Replay")
        print("  [1-9]   - Set speed (1=0.1x, 5=0.5x, 9=0.9x, 0=1.0x)")
        print("  [q]     - Quit")
        print("=" * 60)


@dataclass
class PreviewConfig:
    """Preview excitation trajectory in RViz."""

    trajectory: str = DEFAULT_TRAJECTORY
    """Path to trajectory JSON file."""
    speed: float = 1.0
    """Playback speed multiplier."""


def main(config: PreviewConfig) -> None:
    try:
        node = ExcitationTrajectoryPreviewNode(config.trajectory, speed=config.speed)
        node.interactive_mode()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    import tyro

    main(tyro.cli(PreviewConfig))
