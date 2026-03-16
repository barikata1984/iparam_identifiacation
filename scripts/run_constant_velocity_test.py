#!/usr/bin/env python3
"""
Run constant velocity test on real robot and collect kinematics data.

This script:
1. Moves the robot to the start position (with user confirmation)
2. Executes the trajectory while collecting joint states
3. Computes tool0 classical velocities and accelerations
4. Saves results and generates verification plots

Usage:
    python3 run_constant_velocity_test.py [trajectory_json_path]
"""

import json
import sys
import time
import numpy as np
from datetime import datetime

# Add path for imports
sys.path.insert(0, "/root/osx-ur/catkin_ws/src/iparam_identification/src")

import rospy
from sensor_msgs.msg import JointState

from utilities.tool0_kinematics import (
    Tool0KinematicsCalculator,
    reorder_joint_state,
    JOINT_ORDER,
)

# Import ur_control for robot control
from ur_control.fzi_cartesian_compliance_controller import CompliantController


class ConstantVelocityTest:
    """Execute constant velocity test and collect data."""

    def __init__(self, trajectory_path: str):
        """Initialize the test."""
        rospy.init_node("constant_velocity_test", anonymous=True)

        # Load trajectory
        with open(trajectory_path, "r") as f:
            self.trajectory_data = json.load(f)

        self.joint_names = self.trajectory_data["joint_names"]
        self.start_pos_deg = np.array(self.trajectory_data["start_position_deg"])
        self.end_pos_deg = np.array(self.trajectory_data["end_position_deg"])
        self.control_freq = self.trajectory_data["control_freq_hz"]

        # Trajectory data
        traj = self.trajectory_data["trajectory"]
        self.times = np.array(traj["times"])
        self.positions_deg = np.array(traj["positions_deg"])
        self.velocities_deg_s = np.array(traj["velocities_deg_s"])

        # Convert to radians
        self.start_pos_rad = np.deg2rad(self.start_pos_deg)
        self.end_pos_rad = np.deg2rad(self.end_pos_deg)
        self.positions_rad = np.deg2rad(self.positions_deg)
        self.velocities_rad_s = np.deg2rad(self.velocities_deg_s)

        # Moving joint information
        self.moving_joint_index = self.trajectory_data.get("moving_joint_index", 0)
        self.moving_joint_name = self.trajectory_data.get("moving_joint_name", "shoulder_pan_joint")

        # Phase information (support both old "j1_trajectory" and new "moving_joint_trajectory" keys)
        moving_traj = self.trajectory_data.get("moving_joint_trajectory", self.trajectory_data.get("j1_trajectory"))
        phases = moving_traj["phases"]
        self.const_vel_start = phases["constant_velocity"]["start"]
        self.const_vel_end = phases["constant_velocity"]["end"]
        self.expected_velocity_deg_s = moving_traj["v_max_deg_s"]

        # Tool0 kinematics calculator
        self.calculator = Tool0KinematicsCalculator(acc_cutoff_freq=10.0)

        # Data collection
        self.collected_data = []
        self.is_collecting = False
        self.collection_start_time = None

        # Initialize robot controller (using CompliantController from ur_control)
        print("Initializing robot controller...")
        self.arm = CompliantController(
            namespace=None,
            joint_names_prefix=None,
            ee_link="tool0",
            ft_topic="wrench",
            gripper_type=None,
        )

        # Activate ROS control on UR robot (required before switching controllers)
        print("Activating ROS control on UR...")
        self.arm.dashboard_services.activate_ros_control_on_ur()
        rospy.sleep(0.5)

        # Ensure trajectory controller is active
        print("Activating trajectory controller...")
        self.arm.activate_joint_trajectory_controller()
        print("Robot controller initialized.")

        # ROS subscriber for data collection
        self.joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback
        )

        print(f"Loaded trajectory: {trajectory_path}")
        print(f"  Start: {self.start_pos_deg} [deg]")
        print(f"  End: {self.end_pos_deg} [deg]")
        print(f"  Duration: {self.times[-1]:.2f}s")
        print(f"  Constant velocity phase: {self.const_vel_start}s - {self.const_vel_end}s")

    def joint_state_callback(self, msg: JointState):
        """Collect joint state data during trajectory execution."""
        if not self.is_collecting:
            return

        # Get current time relative to collection start
        t = rospy.Time.now().to_sec() - self.collection_start_time

        # Reorder joint states
        q, v = reorder_joint_state(
            list(msg.name), list(msg.position), list(msg.velocity)
        )

        # Compute tool0 kinematics
        result = self.calculator.compute(q, v, msg.header.stamp.to_sec())

        # Store data
        self.collected_data.append({
            "time": t,
            "timestamp": msg.header.stamp.to_sec(),
            "joint_position": q.tolist(),
            "joint_velocity": v.tolist(),
            "joint_acceleration": result["joint_acceleration"].tolist(),
            "angular_velocity": result["angular_velocity"].tolist(),
            "linear_velocity": result["linear_velocity"].tolist(),
            "angular_acceleration": result["angular_acceleration"].tolist(),
            "linear_acceleration": result["linear_acceleration"].tolist(),
        })

    def get_current_joint_positions(self) -> np.ndarray:
        """Get current joint positions."""
        return self.arm.joint_angles()

    def move_to_position(self, target_rad: np.ndarray, duration: float = 5.0):
        """Move robot to target position using CompliantController."""
        print(f"  Moving to target (duration: {duration}s)...")
        self.arm.set_joint_positions(
            target_time=duration,
            positions=target_rad,
            wait=True,
        )

    def execute_trajectory(self):
        """Execute the full trajectory using set_joint_trajectory."""
        # Prepare trajectory
        trajectory = self.positions_rad
        velocities = self.velocities_rad_s
        duration = self.times[-1]

        # Start data collection
        self.collected_data = []
        self.calculator.reset()
        self.is_collecting = True
        self.collection_start_time = rospy.Time.now().to_sec()

        # Execute trajectory
        print(f"Executing trajectory ({duration:.1f}s)...")
        result = self.arm.set_joint_trajectory(
            target_time=duration,
            trajectory=trajectory,
            velocities=velocities,
            wait=False,  # Don't block so we can monitor progress
        )

        # Monitor progress
        rate = rospy.Rate(10)
        start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start < duration + 1.0:
            elapsed = rospy.Time.now().to_sec() - start
            progress = min(elapsed / duration * 100, 100)
            sys.stdout.write(f"\r  Progress: {progress:5.1f}% | Samples: {len(self.collected_data)}")
            sys.stdout.flush()
            rate.sleep()

        self.is_collecting = False
        print(f"\n  Collection complete: {len(self.collected_data)} samples")

    def run(self):
        """Run the full test."""
        print("\n" + "=" * 60)
        print("Constant Velocity Test")
        print("=" * 60)

        # Check current position
        current_pos = self.get_current_joint_positions()
        current_pos_deg = np.rad2deg(current_pos)
        print(f"\nCurrent position: {np.round(current_pos_deg, 1)} [deg]")
        print(f"Start position:   {self.start_pos_deg} [deg]")

        # Check if already at start position
        pos_error = np.abs(current_pos_deg - self.start_pos_deg)
        if np.max(pos_error) > 5.0:
            print(f"\nPosition error: {np.round(pos_error, 1)} [deg]")
            print("\n*** Robot needs to move to start position ***")
            input("Press Enter to move to start position (or Ctrl+C to abort)...")

            self.move_to_position(self.start_pos_rad, duration=5.0)
            rospy.sleep(1.0)

            # Verify position
            current_pos = self.get_current_joint_positions()
            current_pos_deg = np.rad2deg(current_pos)
            print(f"New position: {np.round(current_pos_deg, 1)} [deg]")
        else:
            print("Robot is already at start position.")

        # Confirm trajectory execution
        print("\n*** Ready to execute trajectory ***")
        print(f"  Duration: {self.times[-1]:.1f}s")
        ji = self.moving_joint_index
        print(f"  J{ji+1} ({self.moving_joint_name}) will move from {self.start_pos_deg[ji]}° to {self.end_pos_deg[ji]}°")
        input("Press Enter to START trajectory (or Ctrl+C to abort)...")

        # Execute trajectory and collect data
        self.execute_trajectory()

        # Save and analyze results
        return self.analyze_and_save()

    def analyze_and_save(self) -> str:
        """Analyze collected data and save results."""
        if len(self.collected_data) < 10:
            print("ERROR: Not enough data collected!")
            return None

        # Convert to numpy arrays
        times = np.array([d["time"] for d in self.collected_data])
        angular_vel = np.array([d["angular_velocity"] for d in self.collected_data])
        linear_vel = np.array([d["linear_velocity"] for d in self.collected_data])
        angular_acc = np.array([d["angular_acceleration"] for d in self.collected_data])
        linear_acc = np.array([d["linear_acceleration"] for d in self.collected_data])
        joint_vel = np.array([d["joint_velocity"] for d in self.collected_data])
        joint_acc = np.array([d["joint_acceleration"] for d in self.collected_data])

        # Find constant velocity phase indices
        const_mask = (times >= self.const_vel_start) & (times <= self.const_vel_end)

        print("\n" + "=" * 60)
        print("RESULTS")
        print("=" * 60)

        print(f"\nTotal samples: {len(times)}")
        print(f"Constant velocity samples: {np.sum(const_mask)}")

        # Analyze constant velocity phase
        print(f"\n--- Constant Velocity Phase ({self.const_vel_start}s - {self.const_vel_end}s) ---")

        # Moving joint velocity
        ji = self.moving_joint_index
        expected_vel_rad_s = np.deg2rad(self.expected_velocity_deg_s)
        jn_vel_const = joint_vel[const_mask, ji]
        print(f"\nJ{ji+1} Velocity [rad/s]:")
        print(f"  Expected: {expected_vel_rad_s:.4f} rad/s ({self.expected_velocity_deg_s}°/s)")
        print(f"  Mean:     {np.mean(jn_vel_const):.4f}")
        print(f"  Std:      {np.std(jn_vel_const):.4f}")

        # Moving joint acceleration (should be ~0)
        jn_acc_const = joint_acc[const_mask, ji]
        print(f"\nJ{ji+1} Acceleration [rad/s²]:")
        print(f"  Expected: 0.0")
        print(f"  Mean:     {np.mean(jn_acc_const):.6f}")
        print(f"  Std:      {np.std(jn_acc_const):.6f}")
        print(f"  Max |a|:  {np.max(np.abs(jn_acc_const)):.6f}")

        # Tool0 accelerations in constant velocity phase
        lin_acc_const = linear_acc[const_mask]
        ang_acc_const = angular_acc[const_mask]

        print(f"\nTool0 Linear Acceleration [m/s²] (constant vel phase):")
        print(f"  Mean: [{np.mean(lin_acc_const[:,0]):.6f}, {np.mean(lin_acc_const[:,1]):.6f}, {np.mean(lin_acc_const[:,2]):.6f}]")
        print(f"  Std:  [{np.std(lin_acc_const[:,0]):.6f}, {np.std(lin_acc_const[:,1]):.6f}, {np.std(lin_acc_const[:,2]):.6f}]")
        print(f"  Max |a|: {np.max(np.abs(lin_acc_const)):.6f}")

        print(f"\nTool0 Angular Acceleration [rad/s²] (constant vel phase):")
        print(f"  Mean: [{np.mean(ang_acc_const[:,0]):.6f}, {np.mean(ang_acc_const[:,1]):.6f}, {np.mean(ang_acc_const[:,2]):.6f}]")
        print(f"  Std:  [{np.std(ang_acc_const[:,0]):.6f}, {np.std(ang_acc_const[:,1]):.6f}, {np.std(ang_acc_const[:,2]):.6f}]")
        print(f"  Max |a|: {np.max(np.abs(ang_acc_const)):.6f}")

        # Verification
        acc_threshold = 0.1  # m/s² or rad/s²
        lin_acc_pass = np.max(np.abs(lin_acc_const)) < acc_threshold
        ang_acc_pass = np.max(np.abs(ang_acc_const)) < acc_threshold

        print(f"\n--- Verification (threshold: {acc_threshold}) ---")
        print(f"  Linear acceleration < {acc_threshold}: {'PASS' if lin_acc_pass else 'FAIL'}")
        print(f"  Angular acceleration < {acc_threshold}: {'PASS' if ang_acc_pass else 'FAIL'}")

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = f"/root/osx-ur/catkin_ws/src/iparam_identification/results/constant_velocity_test_{timestamp}.json"

        results = {
            "timestamp": timestamp,
            "trajectory_file": self.trajectory_data.get("description", "unknown"),
            "total_samples": len(times),
            "constant_velocity_phase": {
                "start": self.const_vel_start,
                "end": self.const_vel_end,
                "samples": int(np.sum(const_mask)),
            },
            "raw_data": self.collected_data,
            "moving_joint_index": ji,
            "moving_joint_name": self.moving_joint_name,
            "analysis": {
                f"j{ji+1}_velocity_const": {
                    "expected_rad_s": float(expected_vel_rad_s),
                    "mean": float(np.mean(jn_vel_const)),
                    "std": float(np.std(jn_vel_const)),
                },
                f"j{ji+1}_acceleration_const": {
                    "mean": float(np.mean(jn_acc_const)),
                    "std": float(np.std(jn_acc_const)),
                    "max_abs": float(np.max(np.abs(jn_acc_const))),
                },
                "tool0_linear_acc_const": {
                    "mean": np.mean(lin_acc_const, axis=0).tolist(),
                    "std": np.std(lin_acc_const, axis=0).tolist(),
                    "max_abs": float(np.max(np.abs(lin_acc_const))),
                },
                "tool0_angular_acc_const": {
                    "mean": np.mean(ang_acc_const, axis=0).tolist(),
                    "std": np.std(ang_acc_const, axis=0).tolist(),
                    "max_abs": float(np.max(np.abs(ang_acc_const))),
                },
            },
            "verification": {
                "linear_acc_pass": bool(lin_acc_pass),
                "angular_acc_pass": bool(ang_acc_pass),
                "threshold": acc_threshold,
            },
        }

        with open(output_path, "w") as f:
            json.dump(results, f, indent=2)

        print(f"\nResults saved to: {output_path}")

        # Generate plots
        plot_path = self.generate_plots(times, joint_vel, joint_acc,
                                         linear_vel, linear_acc,
                                         angular_vel, angular_acc,
                                         const_mask, timestamp, ji)
        print(f"Plots saved to: {plot_path}")

        return output_path

    def generate_plots(self, times, joint_vel, joint_acc,
                       linear_vel, linear_acc, angular_vel, angular_acc,
                       const_mask, timestamp, joint_index=0):
        """Generate verification plots."""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 2, figsize=(14, 12))
        jn = joint_index + 1  # Human-readable joint number

        # Constant velocity region shading
        def shade_const_vel(ax):
            ax.axvspan(self.const_vel_start, self.const_vel_end,
                      alpha=0.2, color='green', label='Const vel phase')

        # Moving joint velocity
        ax = axes[0, 0]
        ax.plot(times, np.rad2deg(joint_vel[:, joint_index]), 'b-', linewidth=0.8)
        ax.axhline(y=self.expected_velocity_deg_s, color='r', linestyle='--', alpha=0.5,
                   label=f'Expected ({self.expected_velocity_deg_s}°/s)')
        shade_const_vel(ax)
        ax.set_ylabel(f'J{jn} Velocity [°/s]')
        ax.set_title(f'Joint {jn} Velocity')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Moving joint acceleration
        ax = axes[0, 1]
        ax.plot(times, np.rad2deg(joint_acc[:, joint_index]), 'r-', linewidth=0.8)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        shade_const_vel(ax)
        ax.set_ylabel(f'J{jn} Acceleration [°/s²]')
        ax.set_title(f'Joint {jn} Acceleration')
        ax.grid(True, alpha=0.3)

        # Tool0 Linear Velocity
        ax = axes[1, 0]
        ax.plot(times, linear_vel[:, 0], label='x', linewidth=0.8)
        ax.plot(times, linear_vel[:, 1], label='y', linewidth=0.8)
        ax.plot(times, linear_vel[:, 2], label='z', linewidth=0.8)
        shade_const_vel(ax)
        ax.set_ylabel('Linear Velocity [m/s]')
        ax.set_title('Tool0 Linear Velocity (LOCAL frame)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Tool0 Linear Acceleration
        ax = axes[1, 1]
        ax.plot(times, linear_acc[:, 0], label='x', linewidth=0.8)
        ax.plot(times, linear_acc[:, 1], label='y', linewidth=0.8)
        ax.plot(times, linear_acc[:, 2], label='z', linewidth=0.8)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        shade_const_vel(ax)
        ax.set_ylabel('Linear Acceleration [m/s²]')
        ax.set_title('Tool0 Linear Acceleration (Classical, LOCAL frame)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Tool0 Angular Velocity
        ax = axes[2, 0]
        ax.plot(times, angular_vel[:, 0], label='x', linewidth=0.8)
        ax.plot(times, angular_vel[:, 1], label='y', linewidth=0.8)
        ax.plot(times, angular_vel[:, 2], label='z', linewidth=0.8)
        shade_const_vel(ax)
        ax.set_ylabel('Angular Velocity [rad/s]')
        ax.set_title('Tool0 Angular Velocity (LOCAL frame)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Tool0 Angular Acceleration
        ax = axes[2, 1]
        ax.plot(times, angular_acc[:, 0], label='x', linewidth=0.8)
        ax.plot(times, angular_acc[:, 1], label='y', linewidth=0.8)
        ax.plot(times, angular_acc[:, 2], label='z', linewidth=0.8)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        shade_const_vel(ax)
        ax.set_ylabel('Angular Acceleration [rad/s²]')
        ax.set_title('Tool0 Angular Acceleration (Classical, LOCAL frame)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Acceleration magnitude (constant vel phase only)
        ax = axes[3, 0]
        lin_acc_mag = np.linalg.norm(linear_acc, axis=1)
        ax.plot(times, lin_acc_mag, 'b-', linewidth=0.8)
        shade_const_vel(ax)
        ax.set_ylabel('|Linear Acc| [m/s²]')
        ax.set_xlabel('Time [s]')
        ax.set_title('Tool0 Linear Acceleration Magnitude')
        ax.grid(True, alpha=0.3)

        ax = axes[3, 1]
        ang_acc_mag = np.linalg.norm(angular_acc, axis=1)
        ax.plot(times, ang_acc_mag, 'r-', linewidth=0.8)
        shade_const_vel(ax)
        ax.set_ylabel('|Angular Acc| [rad/s²]')
        ax.set_xlabel('Time [s]')
        ax.set_title('Tool0 Angular Acceleration Magnitude')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = f"/root/osx-ur/catkin_ws/src/iparam_identification/results/constant_velocity_test_{timestamp}.png"
        plt.savefig(plot_path, dpi=150)
        plt.close()

        return plot_path


def main():
    trajectory_path = "/root/osx-ur/catkin_ws/src/iparam_identification/results/j1_constant_velocity_trajectory.json"

    if len(sys.argv) > 1:
        trajectory_path = sys.argv[1]

    try:
        test = ConstantVelocityTest(trajectory_path)
        test.run()
    except rospy.ROSInterruptException:
        print("\nTest interrupted.")
    except KeyboardInterrupt:
        print("\nTest aborted by user.")


if __name__ == "__main__":
    main()
