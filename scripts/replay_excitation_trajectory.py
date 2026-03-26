#!/usr/bin/env python3
"""
Replay excitation trajectory on real UR5e robot and identify inertial parameters.

Workflow (teleop mode):
    1. Move to home → zero_ftsensor (bare flange+gripper) → teleop grasp object
    2. Close Robotiq gripper
    3. Move to start pose
    4-6. Replay → identify → re-sync leader

Workflow (skip_teleop mode):
    3. Move to start pose → zero_ftsensor (bare flange)
    4-6. Replay → identify → re-sync leader

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
from geometry_msgs.msg import WrenchStamped  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from src.core.terminal import check_enter_pressed  # noqa: E402
from src.teleop.config import TeleopConfig  # noqa: E402
from src.teleop.factory import create_teleop_components  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from ur_control.fzi_cartesian_compliance_controller import CompliantController  # noqa: E402
from utilities.tool0_kinematics import (  # noqa: E402
    JOINT_ORDER,
    Tool0KinematicsCalculator,
    reorder_joint_state,
)
from identifiers.tls import ScalingMode, solve_tls_weighted  # noqa: E402
from utilities.identification_utils import PARAM_NAMES, plot_kinematics_wrench  # noqa: E402
from utilities.wrist_end_kinematics_utils import get_regressor_matrix  # noqa: E402

matplotlib.use("Agg")

# Default trajectory path
DEFAULT_TRAJECTORY = str(IPARAM_ROOT / "data" / "trajectories" / "excitation_trajectory.json")


class ExcitationTrajectoryReplayNode:
    """Replay excitation trajectory on real robot and identify inertial parameters."""

    def __init__(self):
        rospy.init_node("excitation_trajectory_replay", anonymous=False)

        # --- ROS parameters ---
        trajectory_path = rospy.get_param("~trajectory_path", DEFAULT_TRAJECTORY)
        self.wrench_topic = rospy.get_param("~wrench_topic", "/wrench")
        self.trim_start = float(rospy.get_param("~trim_start", 0.0))
        self.trim_end = float(rospy.get_param("~trim_end", "inf"))

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

        # --- Robot control ---
        self.skip_teleop = rospy.get_param("~skip_teleop", False)

        # --- TLS scaling mode ---
        scaling_str = rospy.get_param("~tls_scaling", "noise_variance")
        try:
            self.tls_scaling = ScalingMode(scaling_str)
        except ValueError:
            valid = [m.value for m in ScalingMode]
            rospy.logwarn(f"Unknown tls_scaling '{scaling_str}', using noise_variance. Valid: {valid}")
            self.tls_scaling = ScalingMode.NOISE_VARIANCE
        rospy.loginfo(f"  TLS scaling mode: {self.tls_scaling.value}")

        if self.skip_teleop:
            # Direct robot control without teleop/gripper (e.g. bare flange runs)
            self._arm = CompliantController(gripper_type=None)
            self._arm.dashboard_services.activate_ros_control_on_ur()
            self.controller = None
            self.robot = None
            rospy.loginfo("Skip-teleop mode: no gripper, no leader arm.")
        else:
            # Full teleop components (for grasping phase)
            self.teleop_components = create_teleop_components(
                TeleopConfig(), init_ros_node=False, use_cameras=False
            )
            self.controller = self.teleop_components.controller
            self.robot = self.teleop_components.robot
            self._arm = self.robot._arm

        # --- Synchronized data recording ---
        # Kinematics are computed directly in the callback (not via external node)
        # to guarantee la/regressor/wrench consistency within each frame.
        self.recorded_frames: list[dict] = []
        self.is_recording = False
        self._ur_joint_names = set(JOINT_ORDER)

        cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)
        gravity_list = rospy.get_param("~gravity", [0.0, 0.0, -9.81])
        self._kinematics = Tool0KinematicsCalculator(acc_cutoff_freq=cutoff_freq)
        self._gravity = np.array(gravity_list)

        self.sub_joint = message_filters.Subscriber("/joint_states", JointState)
        self.sub_wrench = message_filters.Subscriber(self.wrench_topic, WrenchStamped)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_joint, self.sub_wrench],
            queue_size=100,
            slop=0.01,
        )
        self.ts.registerCallback(self._recording_callback)

        # --- Publishers ---
        self.inertia_pub = rospy.Publisher(
            "~inertia_params", Float64MultiArray, queue_size=1, latch=True
        )
        self.identified_pub = rospy.Publisher("~iparams_identified", Bool, queue_size=1, latch=True)

        rospy.loginfo("ExcitationTrajectoryReplayNode initialized.")

    # =========================================================================
    # Data recording callback
    # =========================================================================

    def _recording_callback(self, joint_msg, wrench_msg):
        if not self.is_recording:
            return

        # Filter non-UR messages (e.g. Robotiq gripper)
        if not self._ur_joint_names.issubset(joint_msg.name):
            return

        # Reorder joints to Pinocchio model order
        q, v = reorder_joint_state(
            list(joint_msg.name), list(joint_msg.position), list(joint_msg.velocity)
        )
        t = joint_msg.header.stamp.to_sec()

        # Compute kinematics directly (no external node dependency)
        result = self._kinematics.compute_with_gravity(q, v, t, self._gravity)
        la = result["proper_linear_acceleration"]
        av = result["angular_velocity"]
        aa = result["angular_acceleration"]

        # Build regressor from the SAME kinematics result (guaranteed consistent)
        regressor = get_regressor_matrix(linear_acc=la, angular_vel=av, angular_acc=aa)

        frame = {
            "time": t,
            "joint_position": q.tolist(),
            "joint_velocity": v.tolist(),
            "tool0_kinematics": {
                "lv": result["linear_velocity"].tolist(),
                "av": av.tolist(),
                "la": la.tolist(),
                "aa": aa.tolist(),
            },
            "regressor": regressor.tolist(),
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

        # Step 2: Zero F/T sensor (bare flange + gripper, before grasping object)
        self._zero_ftsensor()
        self._print_wrench("Wrench after zero_ftsensor (should be ~0):")

        # Step 3: Sync leader arm with robot
        self.controller.sync_with_leader()

        # Step 4: Free teleoperation for grasping
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
    # Phase 3: Move to trajectory start pose (flange down)
    # =========================================================================

    def phase_move_to_start(self):
        print("\n" + "=" * 60)
        print("  PHASE 3: MOVE TO START POSE (flange DOWN)")
        print("=" * 60)

        q0 = self.positions[0]
        q0_deg = np.rad2deg(q0)
        current_deg = np.rad2deg(self._arm.joint_angles())

        print(f"  Current (deg): {np.round(current_deg, 1).tolist()}")
        print(f"  Target  (deg): {np.round(q0_deg, 1).tolist()}")

        input("  Press ENTER to move to start pose (Ctrl+C to abort)...")

        self._arm.activate_joint_trajectory_controller()
        self._arm.set_joint_positions(target_time=5.0, positions=q0, wait=True)
        rospy.sleep(1.0)

        # Verify
        actual_deg = np.rad2deg(self._arm.joint_angles())
        error_deg = np.abs(actual_deg - q0_deg)
        print(f"  Arrived (deg): {np.round(actual_deg, 1).tolist()}")
        print(f"  Error   (deg): {np.round(error_deg, 2).tolist()}")

        if np.max(error_deg) > 2.0:
            rospy.logwarn(f"Position error exceeds 2 deg: {np.max(error_deg):.2f}")

        # Zero F/T sensor in skip_teleop mode (bare flange at start pose)
        if self.skip_teleop:
            self._zero_ftsensor()
            self._print_wrench("Wrench after zero_ftsensor (should be ~0):")

        # Show wrench at start pose.
        # Flange DOWN: tool0 Z points down, gravity pulls payload down → Fz ≈ +mg
        self._print_wrench("Wrench at start pose (flange DOWN, expect Fz ~ +mg):")

        print("  Ready for trajectory replay.")

    # =========================================================================
    # F/T sensor helpers
    # =========================================================================

    def _zero_ftsensor(self):
        """Call zero_ftsensor service."""
        service_name = "/ur_hardware_interface/zero_ftsensor"
        print(f"  Calling {service_name}...")
        try:
            rospy.wait_for_service(service_name, timeout=5.0)
            zero_ft = rospy.ServiceProxy(service_name, Trigger)
            resp = zero_ft()
            if resp.success:
                print("  F/T sensor zeroed successfully.")
            else:
                rospy.logwarn(f"  zero_ftsensor returned: {resp.message}")
        except rospy.ROSException as e:
            rospy.logerr(f"  zero_ftsensor service not available: {e}")

        rospy.sleep(0.5)

    def _print_wrench(self, label: str):
        """Read and display current wrench."""
        try:
            wrench_msg = rospy.wait_for_message(self.wrench_topic, WrenchStamped, timeout=2.0)
            f = wrench_msg.wrench.force
            t = wrench_msg.wrench.torque
            print()
            print(f"  {label}")
            print(f"    Fx: {f.x:>10.4f} N    Fy: {f.y:>10.4f} N    Fz: {f.z:>10.4f} N")
            print(f"    Tx: {t.x:>10.4f} Nm   Ty: {t.y:>10.4f} Nm   Tz: {t.z:>10.4f} Nm")
            print()
        except rospy.ROSException as e:
            rospy.logerr(f"  Failed to read wrench: {e}")

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
        self._arm.activate_joint_trajectory_controller()

        print(f"  Executing trajectory ({self.duration:.1f}s)...")
        self._arm.set_joint_trajectory(
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

        N = len(trimmed_frames)
        print(f"  S matrix: {S_total.shape}, W vector: {W_total.shape}")

        # Augment regressor with constant bias columns to absorb F/T sensor offset.
        # Following Kubus et al. (2007) Approach 2: [A | I_6] @ [phi; b] = W
        bias_block = np.tile(np.eye(6), (N, 1))  # (6*N, 6)
        S_aug = np.hstack([S_total, bias_block])  # (6*N, 16)

        # --- Solve with 4 methods ---
        results = {}

        # OLS
        print("  Solving OLS...")
        try:
            pi, _, rank, _ = np.linalg.lstsq(S_total, W_total, rcond=None)
            results["OLS"] = pi
        except Exception as e:
            print(f"  OLS failed: {e}")
            results["OLS"] = np.zeros(10)

        # TLS
        print(f"  Solving TLS ({self.tls_scaling.value})...")
        try:
            tls_result = solve_tls_weighted(S_total, W_total, scaling_mode=self.tls_scaling)
            results["TLS"] = tls_result.x
        except Exception as e:
            print(f"  TLS failed: {e}")
            results["TLS"] = np.zeros(10)

        # OLS+bias
        print("  Solving OLS+bias...")
        try:
            pi_aug, _, rank_aug, _ = np.linalg.lstsq(S_aug, W_total, rcond=None)
            results["OLS+bias"] = pi_aug[:10]
            bias_ols = pi_aug[10:]
        except Exception as e:
            print(f"  OLS+bias failed: {e}")
            results["OLS+bias"] = np.zeros(10)
            bias_ols = np.zeros(6)

        # TLS+bias (Partial EIV: bias columns are error-free)
        print(f"  Solving TLS+bias ({self.tls_scaling.value}, partial EIV)...")
        try:
            bias_col_indices = list(range(10, 16))
            tls_bias_result = solve_tls_weighted(
                S_aug, W_total,
                scaling_mode=self.tls_scaling,
                error_free_cols=bias_col_indices,
            )
            results["TLS+bias"] = tls_bias_result.x[:10]
            bias_tls = tls_bias_result.x[10:]
        except Exception as e:
            print(f"  TLS+bias failed: {e}")
            results["TLS+bias"] = np.zeros(10)
            bias_tls = np.zeros(6)

        # --- Display results ---
        methods = ["OLS", "TLS", "OLS+bias", "TLS+bias"]
        print()
        print("=" * 76)
        print("  INERTIA PARAMETER ESTIMATION RESULTS")
        print("=" * 76)
        print(f"  Data range: [{self.trim_start}s, {self.trim_end}s]")
        print(f"  Frames used: {N}")
        print()
        header = f"  {'param':10s}" + "".join(f" {m:>14s}" for m in methods)
        print(header)
        print("  " + "-" * (10 + 15 * len(methods)))
        for i, name in enumerate(PARAM_NAMES):
            row = f"  {name:10s}"
            for m in methods:
                row += f" {results[m][i]:>14.6f}"
            print(row)
        print()

        bias_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        print(f"  {'bias':10s} {'OLS+bias':>14s} {'TLS+bias':>14s}")
        print("  " + "-" * 40)
        for i, label in enumerate(bias_labels):
            unit = "N" if i < 3 else "Nm"
            print(f"  {label:10s} {bias_ols[i]:>13.4f}{unit} {bias_tls[i]:>13.4f}{unit}")
        print("=" * 76)

        # Save results
        results_dir = self._save_results(
            all_frames,
            trimmed_frames,
            results,
            bias_ols,
            bias_tls,
        )
        print(f"  Results saved to: {results_dir}")

        # Prompt to accept (publish OLS+bias as default)
        while True:
            response = input("\n  Accept and publish OLS+bias? (y/n) > ").strip().lower()
            if response in ("y", "yes"):
                self._publish_inertia_params(results["OLS+bias"])
                print("  Inertia parameters published (OLS+bias).")
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
        results: dict[str, np.ndarray],
        bias_ols: np.ndarray,
        bias_tls: np.ndarray,
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
                "tls_scaling": self.tls_scaling.value,
            },
            "results": {method: {"params": params.tolist()} for method, params in results.items()},
            "bias": {
                "ols": bias_ols.tolist(),
                "tls": bias_tls.tolist(),
            },
            "frames": trimmed_frames,
        }

        result_path = os.path.join(results_dir, "result.json")
        with open(result_path, "w") as f:
            json.dump(output_data, f, indent=2)

        # Save plots
        plot_kinematics_wrench(trimmed_frames, results_dir)

        return results_dir

    def _publish_inertia_params(self, params: np.ndarray):
        msg = Float64MultiArray()
        msg.data = params.tolist()
        self.inertia_pub.publish(msg)
        self.identified_pub.publish(Bool(True))
        rospy.loginfo("Published inertia parameters and iparams_identified=True")

    # =========================================================================
    # Main run
    # =========================================================================

    def run(self):
        print()
        print("=" * 60)
        print("  EXCITATION TRAJECTORY REPLAY")
        print("=" * 60)

        if not self.skip_teleop:
            # Phase 1: Teleop grasp
            self.phase_teleop_grasp()

            # Phase 2: Close gripper
            self.phase_close_gripper()

        accepted = False
        while not accepted:
            # Phase 3: Move to start pose (flange down)
            self.phase_move_to_start()

            # Phase 4: Replay and record
            frames = self.phase_replay_and_record()

            # Phase 5: Identify
            accepted = self.phase_identify(frames)

            if not accepted:
                print("\n  Retrying from start pose...")

        print()
        print("=" * 60)
        print("  COMPLETE - Parameters published")
        print("=" * 60)

        # Keep node alive for latched publishers
        print("  Node alive (latched topics active). Press Ctrl+C to exit.")
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
