#!/usr/bin/env python3
"""
Replay excitation trajectory on real UR5e robot and identify inertial parameters.

Supports two modes:
  - standalone (default): Interactive CLI workflow (teleop → replay → identify)
  - action: ActionServer mode for integration with data_collection

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
    # Standalone (default):
    rosrun iparam_identification replay_excitation_trajectory.py \
        _trajectory_path:=/path/to/excitation_trajectory.json

    # ActionServer mode (for data_collection integration):
    rosrun iparam_identification replay_excitation_trajectory.py _standalone:=false
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

import actionlib  # noqa: E402
import matplotlib  # noqa: E402
import message_filters  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import WrenchStamped  # noqa: E402
from iparam_identification.msg import (  # noqa: E402
    ExcitationAction,
    ExcitationFeedback,
    ExcitationResult,
)
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
from identifiers.pipeline import IdentificationPipeline, IdentificationResult  # noqa: E402
from identifiers.tls import ScalingMode  # noqa: E402
from utilities.identification_utils import (  # noqa: E402
    PARAM_NAMES,
    plot_kinematics_wrench,
    plot_replay_recording,
    plot_velocity_acceleration_comparison,
    plot_wrist3_torque,
)

matplotlib.use("Agg")

# Default paths
DEFAULT_TRAJECTORY = str(IPARAM_ROOT / "data" / "trajectories" / "excitation_trajectory.json")
DEFAULT_GRIPPER_CAL = str(IPARAM_ROOT / "data" / "calibration" / "gripper.json")


class ExcitationTrajectoryReplayNode:
    """Replay excitation trajectory on real robot and identify inertial parameters.

    Supports standalone (interactive CLI) and action server modes.
    """

    def __init__(self):
        rospy.init_node("excitation_trajectory_replay", anonymous=False)

        # --- Mode ---
        self.standalone = rospy.get_param("~standalone", True)

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

        # --- Replay options ---
        # Number of times the (periodic) excitation trajectory is replayed back-to-back.
        self.n_laps = max(1, int(rospy.get_param("~n_laps", 3)))
        # When False, skip phase 5+ (identification/publish) and only replay+record.
        self.run_identification = rospy.get_param("~run_identification", True)
        # When False, never call zero_ftsensor (record raw, un-tared F/T).
        self.zero_ftsensor_enabled = rospy.get_param("~zero_ftsensor", True)

        # --- TLS scaling mode ---
        scaling_str = rospy.get_param("~tls_scaling", "noise_variance")
        try:
            self.tls_scaling = ScalingMode(scaling_str)
        except ValueError:
            valid = [m.value for m in ScalingMode]
            rospy.logwarn(f"Unknown tls_scaling '{scaling_str}', using noise_variance. Valid: {valid}")
            self.tls_scaling = ScalingMode.NOISE_VARIANCE
        rospy.loginfo(f"  TLS scaling mode: {self.tls_scaling.value}")

        # --- Gripper calibration (for difference method) ---
        gripper_cal_path = rospy.get_param("~gripper_calibration", "")
        self.gripper_cal = None
        self._gripper_cal_params = None  # (10,) ndarray for Pipeline
        if gripper_cal_path:
            try:
                with open(gripper_cal_path) as f:
                    self.gripper_cal = json.load(f)
                # Extract OLS+bias params for pipeline difference method
                if "methods" in self.gripper_cal and "OLS+bias" in self.gripper_cal["methods"]:
                    self._gripper_cal_params = np.array(
                        self.gripper_cal["methods"]["OLS+bias"]["params"]
                    )
                rospy.loginfo(f"  Gripper calibration loaded: {gripper_cal_path}")
                rospy.loginfo("  → Object identification mode: gripper inertia will be subtracted")
            except (FileNotFoundError, json.JSONDecodeError) as e:
                rospy.logwarn(f"  Failed to load gripper calibration: {e}")

        if self.skip_teleop:
            self._arm = CompliantController(gripper_type=None)
            self._arm.dashboard_services.activate_ros_control_on_ur()
            self.controller = None
            self.robot = None
            rospy.loginfo("Skip-teleop mode: no gripper, no leader arm.")
        else:
            self.teleop_components = create_teleop_components(
                TeleopConfig(), init_ros_node=False, use_cameras=False
            )
            self.controller = self.teleop_components.controller
            self.robot = self.teleop_components.robot
            self._arm = self.robot._arm

        # --- IdentificationPipeline ---
        cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)
        gravity_list = rospy.get_param("~gravity", [0.0, 0.0, -9.81])
        self._pipeline = IdentificationPipeline(
            tls_scaling=self.tls_scaling,
            acc_cutoff_freq=cutoff_freq,
            gravity=np.array(gravity_list),
        )

        # --- Forward kinematics for gripper-tip (tool0) base-frame position ---
        self._fk = Tool0KinematicsCalculator(acc_cutoff_freq=cutoff_freq)

        # --- Synchronized data recording ---
        self.recorded_frames: list[dict] = []
        self._lap_start_times: list[float] = []
        self.is_recording = False
        self._ur_joint_names = set(JOINT_ORDER)

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

        # --- ActionServer (always created, active only in non-standalone mode) ---
        self._action_server = actionlib.SimpleActionServer(
            "~excitation",
            ExcitationAction,
            execute_cb=self._action_execute_cb,
            auto_start=False,
        )

        rospy.loginfo(
            f"ExcitationTrajectoryReplayNode initialized (standalone={self.standalone})."
        )

    # =========================================================================
    # Data recording callback
    # =========================================================================

    def _recording_callback(self, joint_msg, wrench_msg):
        if not self.is_recording:
            return

        if not self._ur_joint_names.issubset(joint_msg.name):
            return

        q, v = reorder_joint_state(
            list(joint_msg.name), list(joint_msg.position), list(joint_msg.velocity)
        )
        t = joint_msg.header.stamp.to_sec()
        wrench = np.array([
            wrench_msg.wrench.force.x,
            wrench_msg.wrench.force.y,
            wrench_msg.wrench.force.z,
            wrench_msg.wrench.torque.x,
            wrench_msg.wrench.torque.y,
            wrench_msg.wrench.torque.z,
        ])

        # Feed into pipeline (kinematics + regressor computed internally)
        self._pipeline.process_frame(q, v, t, wrench)

        # Gripper-tip (tool0) position and Jacobian-based velocity w.r.t. base frame
        tip_pos = self._fk.tool0_position(q)
        tip_vel_jac = self._fk.tool0_velocity(q, v)

        # Also keep raw frame data for saving/plotting
        self.recorded_frames.append({
            "time": t,
            "joint_position": q.tolist(),
            "joint_velocity": v.tolist(),
            "wrench": wrench.tolist(),
            "tool0_position": tip_pos.tolist(),
            "tool0_velocity": tip_vel_jac.tolist(),
        })

    # =========================================================================
    # Phase 1: Teleop grasp
    # =========================================================================

    def phase_teleop_grasp(self):
        print("\n" + "=" * 60)
        print("  PHASE 1: TELEOP GRASP")
        print("=" * 60)

        self.controller.move_to_home(wait_for_input=True)

        self._zero_ftsensor()
        self._print_wrench("Wrench after zero_ftsensor (should be ~0):")

        self.controller.sync_with_leader()

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

        actual_deg = np.rad2deg(self._arm.joint_angles())
        error_deg = np.abs(actual_deg - q0_deg)
        print(f"  Arrived (deg): {np.round(actual_deg, 1).tolist()}")
        print(f"  Error   (deg): {np.round(error_deg, 2).tolist()}")

        if np.max(error_deg) > 2.0:
            rospy.logwarn(f"Position error exceeds 2 deg: {np.max(error_deg):.2f}")

        if self.skip_teleop and self.standalone:
            self._zero_ftsensor()
            self._print_wrench("Wrench after zero_ftsensor (should be ~0):")

        self._print_wrench("Wrench at start pose (flange DOWN, expect Fz ~ +mg):")

        print("  Ready for trajectory replay.")

    # =========================================================================
    # F/T sensor helpers
    # =========================================================================

    def _zero_ftsensor(self):
        """Call zero_ftsensor service (no-op if disabled via ~zero_ftsensor:=false)."""
        if not self.zero_ftsensor_enabled:
            rospy.logwarn("zero_ftsensor disabled (~zero_ftsensor=false): skipping F/T zeroing.")
            print("  [zero_ftsensor DISABLED] skipping F/T sensor zeroing (recording raw wrench).")
            return

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

    def phase_replay_and_record(self, feedback_cb=None) -> int:
        """Replay trajectory and record data via pipeline.

        Args:
            feedback_cb: Optional callback(progress, frame_count) for ActionServer feedback.

        Returns:
            Number of recorded frames.
        """
        print("\n" + "=" * 60)
        print("  PHASE 4: TRAJECTORY REPLAY + RECORDING")
        print("=" * 60)

        if feedback_cb is None:
            input("  Press ENTER to START trajectory replay (Ctrl+C to abort)...")

        # Reset pipeline and frame buffer for new recording
        self._pipeline.reset()
        self.recorded_frames = []
        self._lap_start_times = []
        self.is_recording = True

        self._arm.activate_joint_trajectory_controller()

        # The excitation trajectory is periodic (q[0]==q[-1], dq=0 at both ends),
        # so it can be replayed back-to-back without repositioning between laps.
        print(f"  Executing trajectory ({self.duration:.1f}s) x {self.n_laps} lap(s)...")
        rate = rospy.Rate(10)
        for lap in range(self.n_laps):
            print(f"\n  --- Lap {lap + 1}/{self.n_laps} ---")
            self._arm.set_joint_trajectory(
                target_time=self.duration,
                trajectory=self.positions,
                velocities=self.velocities,
                accelerations=self.accelerations,
                wait=False,
            )

            start = rospy.Time.now().to_sec()
            self._lap_start_times.append(start)
            while not rospy.is_shutdown():
                elapsed = rospy.Time.now().to_sec() - start
                if elapsed >= self.duration + 0.5:
                    break
                lap_progress = min(elapsed / self.duration, 1.0)
                overall_progress = (lap + lap_progress) / self.n_laps
                sys.stdout.write(
                    f"\r  Lap {lap + 1}/{self.n_laps} | Progress: {overall_progress * 100:5.1f}%"
                    f" | Frames: {self._pipeline.frame_count}"
                )
                sys.stdout.flush()
                if feedback_cb:
                    feedback_cb(overall_progress, self._pipeline.frame_count)
                rate.sleep()

        self.is_recording = False
        n_frames = self._pipeline.frame_count
        print(f"\n  Recording complete: {n_frames} frames")

        # Save + plot the trajectory-following measurements (F/T, tip pos & derivatives)
        self._save_replay_recording()

        return n_frames

    def _save_replay_recording(self) -> str | None:
        """Save recorded F/T and gripper-tip (tool0) kinematics, then plot them.

        Records per frame: F/T (6 components), tool0 position w.r.t. base, and the
        1st/2nd numerical time derivatives of that position. Saves a .npz of the raw
        arrays and a combined PNG plot to a timestamped results directory.
        """
        if len(self.recorded_frames) < 3:
            rospy.logwarn("Not enough frames recorded to plot (need >= 3).")
            return None

        times = np.array([f["time"] for f in self.recorded_frames])
        wrench = np.array([f["wrench"] for f in self.recorded_frames])
        tip_pos = np.array([f["tool0_position"] for f in self.recorded_frames])
        tip_vel_jac = np.array([f["tool0_velocity"] for f in self.recorded_frames])
        # Joint position/velocity (order = JOINT_ORDER, so index 5 = wrist_3) for
        # correlating measured torque against joint-velocity sign reversals.
        joint_pos = np.array([f["joint_position"] for f in self.recorded_frames])
        joint_vel = np.array([f["joint_velocity"] for f in self.recorded_frames])

        # Enforce strictly increasing time so np.gradient stays well-defined
        order = np.argsort(times)
        times, wrench, tip_pos, tip_vel_jac, joint_pos, joint_vel = (
            times[order], wrench[order], tip_pos[order], tip_vel_jac[order],
            joint_pos[order], joint_vel[order],
        )
        keep = np.concatenate([[True], np.diff(times) > 0])
        times, wrench, tip_pos, tip_vel_jac, joint_pos, joint_vel = (
            times[keep], wrench[keep], tip_pos[keep], tip_vel_jac[keep],
            joint_pos[keep], joint_vel[keep],
        )

        if len(times) < 3:
            rospy.logwarn("Not enough distinct timestamps to differentiate.")
            return None

        # Velocity: numerical (d/dt of position) and Jacobian (J·q̇, already recorded).
        tip_vel_num = np.gradient(tip_pos, times, axis=0)
        # Acceleration: 2nd numerical derivative of position (num) and 1st numerical
        # derivative of the Jacobian velocity (jac). No measured joint accel exists,
        # so both require at least one differentiation; the Jacobian path needs one
        # fewer than the position double-difference.
        tip_acc_num = np.gradient(tip_vel_num, times, axis=0)
        tip_acc_jac = np.gradient(tip_vel_jac, times, axis=0)

        # Commanded/reference tip trajectory, tiled over the executed laps
        ref_times, ref_pos, ref_vel, ref_acc = self._commanded_tip_reference()

        results_dir = self._make_results_dir("replay_recording")
        np.savez(
            os.path.join(results_dir, "recording.npz"),
            time=times,
            wrench=wrench,
            joint_position=joint_pos,
            joint_velocity=joint_vel,
            joint_names=np.array(JOINT_ORDER),
            tip_position=tip_pos,
            tip_velocity_numerical=tip_vel_num,
            tip_velocity_jacobian=tip_vel_jac,
            tip_acceleration_numerical=tip_acc_num,
            tip_acceleration_jacobian=tip_acc_jac,
            ref_time=ref_times if ref_times is not None else np.array([]),
            ref_tip_position=ref_pos if ref_pos is not None else np.empty((0, 3)),
            ref_tip_velocity=ref_vel if ref_vel is not None else np.empty((0, 3)),
            ref_tip_acceleration=ref_acc if ref_acc is not None else np.empty((0, 3)),
        )
        # Overview uses the Jacobian-based velocity (J·q̇) and its derivative for
        # acceleration (lower noise than the position double-difference).
        plot_replay_recording(
            times, wrench, tip_pos, tip_vel_jac, tip_acc_jac, results_dir,
            ref_times=ref_times, ref_pos=ref_pos, ref_vel=ref_vel, ref_acc=ref_acc,
        )
        plot_velocity_acceleration_comparison(
            times, tip_vel_num, tip_vel_jac, tip_acc_num, tip_acc_jac,
            ref_times, ref_vel, ref_acc, results_dir,
        )
        # wrist_3 position vs Tz (Coulomb-friction check; Tz should switch at the
        # wrist_3 position turning points where joint velocity reverses).
        w3_idx = JOINT_ORDER.index("wrist_3_joint")
        plot_wrist3_torque(times, joint_pos[:, w3_idx], wrench[:, 5], results_dir)
        print(f"  Recording data + plots saved to: {results_dir}")
        return results_dir

    def _commanded_tip_reference(self):
        """Build the commanded tip (tool0) reference tiled over executed laps.

        The single-period reference is computed analytically from the commanded
        joint trajectory (no numerical differentiation): position FK(q), velocity
        J(q)·q̇, and classical acceleration from (q, q̇, q̈) using the commanded ddq.
        NaN separators are inserted between laps so gaps are not drawn connected.

        Returns:
            (ref_times, ref_pos, ref_vel, ref_acc) wall-clock-aligned arrays, or
            (None, None, None, None) if lap timing is unavailable.
        """
        if not self._lap_start_times:
            return None, None, None, None

        t_rel = self.times - self.times[0]
        pos_period = np.array([self._fk.tool0_position(q) for q in self.positions])
        vel_period = np.array([
            self._fk.tool0_velocity(q, dq)
            for q, dq in zip(self.positions, self.velocities, strict=True)
        ])
        acc_period = np.array([
            self._fk.tool0_acceleration(q, dq, ddq)
            for q, dq, ddq in zip(
                self.positions, self.velocities, self.accelerations, strict=True
            )
        ])

        nan_t = np.array([np.nan])
        nan_v = np.full((1, 3), np.nan)
        t_parts, p_parts, v_parts, a_parts = [], [], [], []
        for s in self._lap_start_times:
            t_parts += [s + t_rel, nan_t]
            p_parts += [pos_period, nan_v]
            v_parts += [vel_period, nan_v]
            a_parts += [acc_period, nan_v]

        return (
            np.concatenate(t_parts),
            np.concatenate(p_parts),
            np.concatenate(v_parts),
            np.concatenate(a_parts),
        )

    def _make_results_dir(self, prefix: str) -> str:
        """Create and return a timestamped directory under the package's results/."""
        import rospkg

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("iparam_identification")
        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        results_dir = os.path.join(package_path, "results", f"{prefix}_{timestamp_str}")
        os.makedirs(results_dir, exist_ok=True)
        return results_dir

    # =========================================================================
    # Phase 5: Identify inertial parameters
    # =========================================================================

    def phase_identify(self, interactive: bool = True) -> IdentificationResult | None:
        """Run identification using pipeline.

        Args:
            interactive: If True, display results and prompt for acceptance.
                If False (action mode), auto-accept.

        Returns:
            IdentificationResult if accepted/auto, None if rejected.
        """
        print("\n" + "=" * 60)
        print("  PHASE 5: INERTIAL PARAMETER IDENTIFICATION")
        print("=" * 60)

        if self._pipeline.frame_count < 10:
            print("  ERROR: Not enough frames recorded!")
            return None

        # Run identification via pipeline
        result = self._pipeline.identify(
            trim_start=self.trim_start,
            trim_end=self.trim_end,
            gripper_cal=self._gripper_cal_params,
        )

        # Display results
        self._print_identification_result(result)

        # Save results
        results_dir = self._save_results(result)
        print(f"  Results saved to: {results_dir}")
        result.meta["results_dir"] = results_dir

        # Determine which parameters to publish:
        # If gripper calibration is available, publish object params (difference method)
        # Otherwise, publish total params (gripper + payload)
        if result.object_params is not None and "OLS+bias" in result.object_params:
            publish_params = result.object_params["OLS+bias"]
            publish_label = "OLS+bias object (difference method)"
        else:
            publish_params = result.params
            publish_label = "OLS+bias total"

        if not interactive:
            # Auto-accept in action mode
            self._publish_inertia_params(publish_params)
            print(f"  Inertia parameters published ({publish_label}, auto-accepted).")
            return result

        # Interactive prompt
        while True:
            response = input(f"\n  Accept and publish {publish_label}? (y/n) > ").strip().lower()
            if response in ("y", "yes"):
                self._publish_inertia_params(publish_params)
                print(f"  Inertia parameters published ({publish_label}).")
                return result
            elif response in ("n", "no"):
                print("  Results not published.")
                return None
            else:
                print("  Please enter 'y' or 'n'.")

    def _print_identification_result(self, result: IdentificationResult):
        """Display identification results in tabular format."""
        methods = ["OLS", "TLS", "OLS+bias", "TLS+bias"]
        all_params = {
            "OLS": result.ols,
            "TLS": result.tls.x,
            "OLS+bias": result.ols_bias,
            "TLS+bias": result.tls_bias.x[:10],
        }

        N = result.meta["trimmed_frames"]
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
                row += f" {all_params[m][i]:>14.6f}"
            print(row)
        print()

        bias_labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        print(f"  {'bias':10s} {'OLS+bias':>14s} {'TLS+bias':>14s}")
        print("  " + "-" * 40)
        for i, label in enumerate(bias_labels):
            unit = "N" if i < 3 else "Nm"
            print(f"  {label:10s} {result.bias_ols[i]:>13.4f}{unit} {result.bias_tls[i]:>13.4f}{unit}")
        print("=" * 76)

        # Difference method results
        if result.object_params is not None:
            print()
            print("=" * 76)
            print("  OBJECT INERTIA (difference method: φ_total - φ_gripper)")
            print("=" * 76)
            available = [m for m in methods if m in result.object_params]
            if available:
                header = f"  {'param':10s}" + "".join(f" {m:>14s}" for m in available)
                print(header)
                print("  " + "-" * (10 + 15 * len(available)))
                for i, name in enumerate(PARAM_NAMES):
                    row = f"  {name:10s}"
                    for m in available:
                        row += f" {result.object_params[m][i]:>14.6f}"
                    print(row)
            print("=" * 76)

    def _save_results(self, result: IdentificationResult) -> str:
        """Save identification results to timestamped directory."""
        import rospkg

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("iparam_identification")

        timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dirname = f"excitation_replay_{timestamp_str}"
        results_dir = os.path.join(package_path, "results", dirname)
        os.makedirs(results_dir, exist_ok=True)

        output_data = {
            "meta": {
                "timestamp": timestamp_str,
                "total_frames": result.meta["total_frames"],
                "trimmed_frames": result.meta["trimmed_frames"],
                "trim_start": result.meta["trim_start"],
                "trim_end": result.meta["trim_end"],
                "tls_scaling": result.meta["tls_scaling"],
                "gripper_calibration": result.object_params is not None,
            },
            "results": {
                "OLS": {"params": result.ols.tolist()},
                "TLS": {"params": result.tls.x.tolist()},
                "OLS+bias": {"params": result.ols_bias.tolist()},
                "TLS+bias": {"params": result.tls_bias.x[:10].tolist()},
            },
            "bias": {
                "ols": result.bias_ols.tolist(),
                "tls": result.bias_tls.tolist(),
            },
        }

        if result.object_params is not None:
            output_data["object_results"] = {
                method: {"params": params.tolist()}
                for method, params in result.object_params.items()
            }

        result_path = os.path.join(results_dir, "result.json")
        with open(result_path, "w") as f:
            json.dump(output_data, f, indent=2)

        return results_dir

    def _publish_inertia_params(self, params: np.ndarray):
        msg = Float64MultiArray()
        msg.data = params.tolist()
        self.inertia_pub.publish(msg)
        self.identified_pub.publish(Bool(True))
        rospy.loginfo("Published inertia parameters and iparams_identified=True")

    # =========================================================================
    # ActionServer callback
    # =========================================================================

    def _action_execute_cb(self, goal):
        """Execute excitation identification via actionlib."""
        rospy.loginfo("Excitation action goal received.")

        # Override parameters from goal if provided
        if goal.trim_start != 0.0 or goal.trim_end != 0.0:
            self.trim_start = goal.trim_start
            self.trim_end = goal.trim_end if goal.trim_end > 0 else float("inf")

        if goal.gripper_calibration_path:
            try:
                with open(goal.gripper_calibration_path) as f:
                    cal = json.load(f)
                if "methods" in cal and "OLS+bias" in cal["methods"]:
                    self._gripper_cal_params = np.array(cal["methods"]["OLS+bias"]["params"])
                    rospy.loginfo(f"Gripper calibration loaded from goal: {goal.gripper_calibration_path}")
            except (FileNotFoundError, json.JSONDecodeError) as e:
                rospy.logwarn(f"Failed to load gripper calibration from goal: {e}")

        action_result = ExcitationResult()

        def send_feedback(progress, frame_count):
            fb = ExcitationFeedback()
            fb.phase = 4
            fb.phase_name = "replay"
            fb.progress = progress
            fb.recorded_frames = frame_count
            self._action_server.publish_feedback(fb)

        try:
            # Phase 3: Move to start
            fb = ExcitationFeedback()
            fb.phase = 3
            fb.phase_name = "move_to_start"
            fb.progress = 0.0
            self._action_server.publish_feedback(fb)
            self.phase_move_to_start()

            # Phase 4: Replay and record
            self.phase_replay_and_record(feedback_cb=send_feedback)

            # Phase 5: Identify (non-interactive)
            fb = ExcitationFeedback()
            fb.phase = 5
            fb.phase_name = "identify"
            fb.progress = 0.0
            fb.recorded_frames = self._pipeline.frame_count
            self._action_server.publish_feedback(fb)

            result = self.phase_identify(interactive=False)

            if result is not None:
                action_result.success = True
                # Return object params if difference method was used
                if result.object_params and "OLS+bias" in result.object_params:
                    action_result.inertia_params = result.object_params["OLS+bias"].tolist()
                    action_result.object_inertia_params = result.object_params["OLS+bias"].tolist()
                else:
                    action_result.inertia_params = result.params.tolist()
                action_result.bias = result.bias.tolist()
                action_result.results_dir = result.meta.get("results_dir", "")
                action_result.message = f"Identified ({result.meta['trimmed_frames']} frames)"
                self._action_server.set_succeeded(action_result)
            else:
                action_result.success = False
                action_result.message = "Identification failed (insufficient frames)"
                self._action_server.set_aborted(action_result)

        except Exception as e:
            rospy.logerr(f"Excitation action failed: {e}")
            action_result.success = False
            action_result.message = str(e)
            self._action_server.set_aborted(action_result)

    # =========================================================================
    # Main run
    # =========================================================================

    def run(self):
        if self.standalone:
            self._run_standalone()
        else:
            self._run_action_server()

    def _run_standalone(self):
        """Interactive standalone mode (original behavior)."""
        print()
        print("=" * 60)
        print("  EXCITATION TRAJECTORY REPLAY")
        print("=" * 60)

        if not self.skip_teleop:
            self.phase_teleop_grasp()
            self.phase_close_gripper()

        if not self.run_identification:
            # Phase 5+ (identification/publish) skipped: replay + record only.
            self.phase_move_to_start()
            n_frames = self.phase_replay_and_record()
            print()
            print("=" * 60)
            print("  PHASE 5+ SKIPPED (run_identification:=false)")
            print(f"  Replayed {self.n_laps} lap(s), recorded {n_frames} frames (not identified).")
            print("=" * 60)
        else:
            result = None
            while result is None:
                self.phase_move_to_start()
                self.phase_replay_and_record()
                result = self.phase_identify(interactive=True)
                if result is None:
                    print("\n  Retrying from start pose...")

            print()
            print("=" * 60)
            print("  COMPLETE - Parameters published")
            print("=" * 60)

        print("  Node alive (latched topics active). Press Ctrl+C to exit.")
        rospy.spin()

    def _run_action_server(self):
        """ActionServer mode: wait for goals from data_collection."""
        self._action_server.start()
        rospy.loginfo("ExcitationActionServer started. Waiting for goals...")
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
