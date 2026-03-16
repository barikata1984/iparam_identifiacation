#!/usr/bin/env python3
"""
Replay excitation trajectory on real UR5e robot and identify inertial parameters.

Workflow:
    1. Free teleoperation to grasp object
    2. Close Robotiq gripper fully
    3. Move to excitation trajectory's initial joint pose
    4. Execute trajectory replay with synchronized data recording
    5. Identify inertial parameters from trimmed time window (default 1s-4s)
    6. Publish parameters and re-sync leader arm

Prerequisites:
    - Robot driver running (ur_robot_driver)
    - roslaunch iparam_identification replay_excitation_trajectory.launch

Usage:
    rosrun iparam_identification replay_excitation_trajectory.py \
        _trajectory_path:=/path/to/excitation_trajectory.json
"""

import datetime
import json
import os
import sys
from pathlib import Path

import numpy as np

# --- Path setup ---
# iparam_identification/src for identifiers
IPARAM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(IPARAM_ROOT / "src"))

# osx_bilateral for teleop components
OSX_BILATERAL_ROOT = Path(__file__).resolve().parent.parent.parent / "osx_bilateral"
sys.path.insert(0, str(OSX_BILATERAL_ROOT))

import matplotlib  # noqa: E402
import message_filters  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import Vector3, WrenchStamped  # noqa: E402
from identifiers.tls import solve_tls_compare  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from src.core.terminal import check_enter_pressed  # noqa: E402
from src.teleop.config import TeleopConfig  # noqa: E402
from src.teleop.factory import create_teleop_components  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray  # noqa: E402
from ur_control.fzi_cartesian_compliance_controller import CompliantController  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Default trajectory path
DEFAULT_TRAJECTORY = str(IPARAM_ROOT / "data" / "trajectories" / "excitation_trajectory.json")

PARAM_NAMES = ["m", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz", "Ixy", "Iyz", "Izx"]


class ExcitationTrajectoryReplayNode:
    """Replay excitation trajectory on real robot and identify inertial parameters."""

    def __init__(self):
        rospy.init_node("excitation_trajectory_replay", anonymous=True)

        # --- ROS parameters ---
        trajectory_path = rospy.get_param("~trajectory_path", DEFAULT_TRAJECTORY)
        self.wrench_topic = rospy.get_param("~wrench_topic", "/wrench")
        self.kinematics_ns = rospy.get_param("~kinematics_ns", "/tool0_kinematics")
        self.trim_start = rospy.get_param("~trim_start", 1.0)
        self.trim_end = rospy.get_param("~trim_end", 4.0)

        # --- Load trajectory ---
        with open(trajectory_path) as f:
            data = json.load(f)

        metadata = data["metadata"]
        trajectory = data["trajectory"]

        self.times = np.array([step["t"] for step in trajectory])
        self.positions = np.array([step["q"] for step in trajectory])
        self.velocities = np.array([step["dq"] for step in trajectory])
        self.accelerations = np.array([step["ddq"] for step in trajectory])
        self.duration = metadata["duration"]
        self.dt = metadata["dt"]

        rospy.loginfo(f"Loaded trajectory: {trajectory_path}")
        rospy.loginfo(f"  Duration: {self.duration}s, Steps: {len(trajectory)}, dt: {self.dt}s")
        rospy.loginfo(f"  Trim window: [{self.trim_start}s, {self.trim_end}s]")
        rospy.loginfo(f"  Condition number: {metadata.get('condition_number', 'N/A')}")

        # --- Teleop components (for grasping phase) ---
        self.teleop_components = create_teleop_components(
            TeleopConfig(), init_ros_node=False, use_cameras=False
        )
        self.controller = self.teleop_components.controller
        self.robot = self.teleop_components.robot

        # --- Synchronized data recording (same pattern as batch_identifier.py) ---
        self.recorded_frames: list[dict] = []
        self.is_recording = False

        self.sub_joint = message_filters.Subscriber("/joint_states", JointState)
        self.sub_wrench = message_filters.Subscriber(self.wrench_topic, WrenchStamped)
        self.sub_lv = message_filters.Subscriber(f"{self.kinematics_ns}/lv_tool0", Vector3)
        self.sub_av = message_filters.Subscriber(f"{self.kinematics_ns}/av_tool0", Vector3)
        self.sub_la = message_filters.Subscriber(f"{self.kinematics_ns}/la_tool0", Vector3)
        self.sub_aa = message_filters.Subscriber(f"{self.kinematics_ns}/aa_tool0", Vector3)
        self.sub_regressor = message_filters.Subscriber(
            f"{self.kinematics_ns}/regressor", Float64MultiArray
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.sub_joint,
                self.sub_wrench,
                self.sub_lv,
                self.sub_av,
                self.sub_la,
                self.sub_aa,
                self.sub_regressor,
            ],
            queue_size=100,
            slop=0.01,
            allow_headerless=True,
        )
        self.ts.registerCallback(self._recording_callback)

        # --- Publishers ---
        self.inertia_pub = rospy.Publisher(
            "~inertia_params", Float64MultiArray, queue_size=1, latch=True
        )
        self.identified_pub = rospy.Publisher("~iparams_identified", Bool, queue_size=1, latch=True)

        rospy.loginfo("ExcitationTrajectoryReplayNode initialized.")

    # =========================================================================
    # Data recording callback (same as batch_identifier.py)
    # =========================================================================

    def _recording_callback(self, joint_msg, wrench_msg, lv_msg, av_msg, la_msg, aa_msg, reg_msg):
        if not self.is_recording:
            return

        frame = {
            "time": joint_msg.header.stamp.to_sec(),
            "joint_position": list(joint_msg.position),
            "joint_velocity": list(joint_msg.velocity),
            "tool0_kinematics": {
                "lv": [lv_msg.x, lv_msg.y, lv_msg.z],
                "av": [av_msg.x, av_msg.y, av_msg.z],
                "la": [la_msg.x, la_msg.y, la_msg.z],
                "aa": [aa_msg.x, aa_msg.y, aa_msg.z],
            },
            "regressor": np.array(reg_msg.data).reshape(6, 10).tolist(),
            "wrench": [
                wrench_msg.wrench.force.x,
                wrench_msg.wrench.force.y,
                wrench_msg.wrench.force.z,
                wrench_msg.wrench.torque.x,
                wrench_msg.wrench.torque.y,
                wrench_msg.wrench.torque.z,
            ],
        }
        self.recorded_frames.append(frame)

    # =========================================================================
    # Phase 1: Teleop grasp
    # =========================================================================

    def phase_teleop_grasp(self):
        print("\n" + "=" * 60)
        print("  PHASE 1: TELEOP GRASP")
        print("=" * 60)

        # Step 1: Move to home position (same as teleop.py)
        self.controller.move_to_home(wait_for_input=True)

        # Step 2: Sync leader arm with robot
        self.controller.sync_with_leader()

        # Step 3: Free teleoperation for grasping
        print()
        print("  >>> TELEOP ACTIVE <<<")
        print("  Grasp the object, then press ENTER to proceed.")
        print()

        self.controller.activate()

        rate = rospy.Rate(self.controller.config.control_frequency)
        while not rospy.is_shutdown():
            if check_enter_pressed():
                break
            self.controller.step()
            rate.sleep()

        # Deactivate teleop WITHOUT zeroing F/T sensor.
        # controller.deactivate() is not used because it calls zero_ft_sensor()
        # twice (directly + via deactivate_compliance), which would destroy the
        # zero point set at driver startup (object-free).
        if self.controller._teleop_active_pub:
            self.controller._teleop_active_pub.publish(Bool(False))
        self.robot._arm.activate_joint_trajectory_controller()
        print("  Teleop deactivated (F/T zero point preserved).")

    # =========================================================================
    # Phase 2: Close gripper
    # =========================================================================

    def phase_close_gripper(self):
        print("\n" + "=" * 60)
        print("  PHASE 2: CLOSE GRIPPER")
        print("=" * 60)

        self.robot.set_gripper(1.0)
        rospy.sleep(1.0)
        print("  Gripper fully closed.")

    # =========================================================================
    # Phase 3: Move to trajectory start pose
    # =========================================================================

    def phase_move_to_start(self):
        print("\n" + "=" * 60)
        print("  PHASE 3: MOVE TO START POSE")
        print("=" * 60)

        q0 = self.positions[0]
        q0_deg = np.rad2deg(q0)
        current_deg = np.rad2deg(self.robot.joint_angles())

        print(f"  Current (deg): {np.round(current_deg, 1).tolist()}")
        print(f"  Target  (deg): {np.round(q0_deg, 1).tolist()}")

        input("  Press ENTER to move to start pose (Ctrl+C to abort)...")

        # Switch to joint trajectory controller WITHOUT zeroing F/T sensor.
        # deactivate_compliance() is NOT used here because it internally calls
        # zero_ft_sensor(), which would destroy the zero point set at driver
        # startup (object-free). The regressor includes gravity via proper
        # acceleration, so the sensor must measure the full payload wrench.
        self.robot._arm.activate_joint_trajectory_controller()
        self.robot.move_to_joints(q0, duration=5.0)
        rospy.sleep(1.0)

        # Verify
        actual_deg = np.rad2deg(self.robot.joint_angles())
        error_deg = np.abs(actual_deg - q0_deg)
        print(f"  Arrived (deg): {np.round(actual_deg, 1).tolist()}")
        print(f"  Error   (deg): {np.round(error_deg, 2).tolist()}")

        if np.max(error_deg) > 2.0:
            rospy.logwarn(f"Position error exceeds 2 deg: {np.max(error_deg):.2f}")

        print("  Ready for trajectory replay.")

    # =========================================================================
    # Phase 4: Replay trajectory and record data
    # =========================================================================

    def phase_replay_and_record(self) -> list[dict]:
        print("\n" + "=" * 60)
        print("  PHASE 4: TRAJECTORY REPLAY + RECORDING")
        print("=" * 60)

        input("  Press ENTER to START trajectory replay (Ctrl+C to abort)...")

        # Start recording
        self.recorded_frames = []
        self.is_recording = True

        # Execute trajectory via CompliantController directly
        arm: CompliantController = self.robot._arm
        arm.activate_joint_trajectory_controller()

        print(f"  Executing trajectory ({self.duration:.1f}s)...")
        arm.set_joint_trajectory(
            target_time=self.duration,
            trajectory=self.positions,
            velocities=self.velocities,
            accelerations=self.accelerations,
            wait=False,
        )

        # Monitor progress
        rate = rospy.Rate(10)
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start
            if elapsed >= self.duration + 0.5:
                break
            progress = min(elapsed / self.duration * 100, 100)
            sys.stdout.write(
                f"\r  Progress: {progress:5.1f}% | Frames: {len(self.recorded_frames)}"
            )
            sys.stdout.flush()
            rate.sleep()

        self.is_recording = False
        print(f"\n  Recording complete: {len(self.recorded_frames)} frames")

        return self.recorded_frames

    # =========================================================================
    # Phase 5: Identify inertial parameters
    # =========================================================================

    def phase_identify(self, frames: list[dict]) -> bool:
        print("\n" + "=" * 60)
        print("  PHASE 5: INERTIAL PARAMETER IDENTIFICATION")
        print("=" * 60)

        if len(frames) < 10:
            print("  ERROR: Not enough frames recorded!")
            return False

        # Convert to relative time and trim
        start_time = frames[0]["time"]
        trimmed_frames = []
        all_frames = []
        for frame in frames:
            rel_time = frame["time"] - start_time
            frame_copy = frame.copy()
            frame_copy["time"] = rel_time
            all_frames.append(frame_copy)
            if self.trim_start <= rel_time <= self.trim_end:
                trimmed_frames.append(frame_copy)

        print(f"  Total frames: {len(all_frames)}")
        print(f"  Trimmed [{self.trim_start}s, {self.trim_end}s]: {len(trimmed_frames)} frames")

        if len(trimmed_frames) < 10:
            print("  ERROR: Not enough frames in trim window!")
            return False

        # Build regressor and wrench matrices
        S_list = [np.array(f["regressor"]) for f in trimmed_frames]
        W_list = [np.array(f["wrench"]) for f in trimmed_frames]

        S_total = np.vstack(S_list)  # (6*N, 10)
        W_total = np.hstack(W_list)  # (6*N,)

        print(f"  S matrix: {S_total.shape}, W vector: {W_total.shape}")

        # OLS
        print("  Solving OLS...")
        try:
            pi_ols, _, rank, _ = np.linalg.lstsq(S_total, W_total, rcond=None)
        except Exception as e:
            print(f"  OLS failed: {e}")
            pi_ols = np.zeros(10)
            rank = 0

        # TLS
        print("  Solving TLS...")
        tls_results = solve_tls_compare(S_total, W_total)

        if tls_results.get("column") is not None:
            pi_tls = tls_results["column"].x
        elif tls_results.get("none") is not None:
            pi_tls = tls_results["none"].x
        else:
            pi_tls = np.zeros(10)

        # Display results
        print()
        print("=" * 60)
        print("  INERTIA PARAMETER ESTIMATION RESULTS")
        print("=" * 60)
        print(f"  Data range: [{self.trim_start}s, {self.trim_end}s]")
        print(f"  Frames used: {len(trimmed_frames)}, Rank: {rank}")
        print()
        print(f"  {'Param':10s} {'OLS':>15s} {'TLS':>15s}")
        print("  " + "-" * 42)
        for i, name in enumerate(PARAM_NAMES):
            print(f"  {name:10s} {pi_ols[i]:>15.6f} {pi_tls[i]:>15.6f}")
        print("=" * 60)

        # Save results
        results_dir = self._save_results(all_frames, trimmed_frames, pi_ols, pi_tls, rank)
        print(f"  Results saved to: {results_dir}")

        # Prompt to accept
        while True:
            response = input("\n  Accept and publish? (y/n) > ").strip().lower()
            if response in ("y", "yes"):
                self._publish_inertia_params(pi_ols)
                print("  Inertia parameters published.")
                return True
            elif response in ("n", "no"):
                print("  Results not published.")
                return False
            else:
                print("  Please enter 'y' or 'n'.")

    def _save_results(
        self,
        all_frames: list[dict],
        trimmed_frames: list[dict],
        pi_ols: np.ndarray,
        pi_tls: np.ndarray,
        rank: int,
    ) -> str:
        import rospkg

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("iparam_identification")

        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dirname = f"excitation_replay_{timestamp_str}"
        results_dir = os.path.join(package_path, "results", dirname)
        os.makedirs(results_dir, exist_ok=True)

        # Save result JSON
        output_data = {
            "meta": {
                "timestamp": timestamp_str,
                "total_frames": len(all_frames),
                "trimmed_frames": len(trimmed_frames),
                "trim_start": self.trim_start,
                "trim_end": self.trim_end,
                "rank": int(rank),
            },
            "results": {
                "ols": {"params": pi_ols.tolist()},
                "tls": {"params": pi_tls.tolist()},
            },
            "frames": trimmed_frames,
        }

        result_path = os.path.join(results_dir, "result.json")
        with open(result_path, "w") as f:
            json.dump(output_data, f, indent=2)

        # Save plots
        self._save_plots(trimmed_frames, results_dir)

        return results_dir

    def _save_plots(self, frames: list[dict], save_dir: str):
        times = [f["time"] for f in frames]
        wrench = np.array([f["wrench"] for f in frames])
        lv = np.array([f["tool0_kinematics"]["lv"] for f in frames])
        av = np.array([f["tool0_kinematics"]["av"] for f in frames])
        la = np.array([f["tool0_kinematics"]["la"] for f in frames])
        aa = np.array([f["tool0_kinematics"]["aa"] for f in frames])

        # Velocity plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        for i, label in enumerate(["x", "y", "z"]):
            ax1.plot(times, lv[:, i], label=label)
            ax2.plot(times, av[:, i], label=label)
        ax1.set_title("Linear Velocity (lv)")
        ax1.set_ylabel("[m/s]")
        ax1.legend()
        ax1.grid(True)
        ax2.set_title("Angular Velocity (av)")
        ax2.set_ylabel("[rad/s]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "velocity.png"))
        plt.close()

        # Acceleration plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        for i, label in enumerate(["x", "y", "z"]):
            ax1.plot(times, la[:, i], label=label)
            ax2.plot(times, aa[:, i], label=label)
        ax1.set_title("Linear Acceleration (la)")
        ax1.set_ylabel("[m/s^2]")
        ax1.legend()
        ax1.grid(True)
        ax2.set_title("Angular Acceleration (aa)")
        ax2.set_ylabel("[rad/s^2]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "acceleration.png"))
        plt.close()

        # Wrench plot
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        for i, label in enumerate(["Fx", "Fy", "Fz"]):
            ax1.plot(times, wrench[:, i], label=label)
        for i, label in enumerate(["Tx", "Ty", "Tz"]):
            ax2.plot(times, wrench[:, 3 + i], label=label)
        ax1.set_title("Force")
        ax1.set_ylabel("[N]")
        ax1.legend()
        ax1.grid(True)
        ax2.set_title("Torque")
        ax2.set_ylabel("[Nm]")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(save_dir, "wrench.png"))
        plt.close()

        rospy.loginfo("Plots saved.")

    def _publish_inertia_params(self, params: np.ndarray):
        msg = Float64MultiArray()
        msg.data = params.tolist()
        self.inertia_pub.publish(msg)
        self.identified_pub.publish(Bool(True))
        rospy.loginfo("Published inertia parameters and iparams_identified=True")

    # =========================================================================
    # Phase 6: Re-sync leader
    # =========================================================================

    def phase_resync_leader(self):
        print("\n" + "=" * 60)
        print("  PHASE 6: RE-SYNC LEADER")
        print("=" * 60)

        print("  Moving to home position...")
        self.robot.move_to_initial_pose()
        print("  At home position.")

        print("  Align leader arm with robot, then press ENTER.")
        self.controller.sync_with_leader()
        print("  Leader re-synced.")

    # =========================================================================
    # Main run
    # =========================================================================

    def run(self):
        print()
        print("=" * 60)
        print("  EXCITATION TRAJECTORY REPLAY")
        print("=" * 60)

        # Phase 1: Teleop grasp
        self.phase_teleop_grasp()

        # Phase 2: Close gripper
        self.phase_close_gripper()

        # Phase 3: Move to start pose
        self.phase_move_to_start()

        # Phase 4: Replay and record
        frames = self.phase_replay_and_record()

        # Phase 5: Identify
        accepted = self.phase_identify(frames)

        # Phase 6: Re-sync
        self.phase_resync_leader()

        print()
        print("=" * 60)
        if accepted:
            print("  COMPLETE - Parameters published")
        else:
            print("  COMPLETE - Parameters NOT published")
        print("=" * 60)

        # Keep node alive for latched publishers
        print("  Press Ctrl+C to exit.")
        rospy.spin()


def main():
    try:
        node = ExcitationTrajectoryReplayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
