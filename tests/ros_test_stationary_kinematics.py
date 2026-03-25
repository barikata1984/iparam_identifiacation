#!/usr/bin/env python3
"""
Stationary Test for Tool0 Kinematics Calculator.

This script verifies that the Tool0KinematicsCalculator produces near-zero
velocities and accelerations when the robot is stationary.

Usage:
    rosrun iparam_identification test_tool0_kinematics_stationary.py

Or directly:
    python3 test_tool0_kinematics_stationary.py
"""

import sys
import numpy as np

# Add the src directory to path for imports
sys.path.insert(0, "/root/osx-ur/catkin_ws/src/iparam_identification/src")

import rospy
from sensor_msgs.msg import JointState

from utilities.tool0_kinematics import (
    Tool0KinematicsCalculator,
    reorder_joint_state,
    DEFAULT_URDF_PATH,
)


class StationaryTest:
    """Test that verifies near-zero velocities and accelerations at rest."""

    def __init__(self, num_samples: int = 100, warmup_samples: int = 10):
        """
        Initialize the test.

        Parameters
        ----------
        num_samples : int
            Number of samples to collect for verification
        warmup_samples : int
            Number of initial samples to discard (differentiator warmup)
        """
        self.num_samples = num_samples
        self.warmup_samples = warmup_samples

        self.calculator = Tool0KinematicsCalculator(
            urdf_path=DEFAULT_URDF_PATH, acc_cutoff_freq=10.0
        )

        self.samples = []
        self.sample_count = 0
        self.test_complete = False

    def joint_state_callback(self, msg: JointState):
        """Process joint state messages."""
        if self.test_complete:
            return

        # Reorder joint states to match pinocchio model
        q, v = reorder_joint_state(
            list(msg.name), list(msg.position), list(msg.velocity)
        )

        t = msg.header.stamp.to_sec()

        # Compute tool0 kinematics
        result = self.calculator.compute(q, v, t)

        self.sample_count += 1

        # Skip warmup samples
        if self.sample_count <= self.warmup_samples:
            return

        # Collect sample
        self.samples.append(result)

        # Check if we have enough samples
        if len(self.samples) >= self.num_samples:
            self.test_complete = True

    def run(self):
        """Run the stationary test."""
        rospy.init_node("test_tool0_kinematics_stationary", anonymous=True)

        print("=" * 60)
        print("Tool0 Kinematics Stationary Test")
        print("=" * 60)
        print(f"Collecting {self.num_samples} samples (after {self.warmup_samples} warmup)...")
        print("Robot should be STATIONARY during this test.")
        print("-" * 60)

        sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # Wait for samples
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and not self.test_complete:
            rate.sleep()

        sub.unregister()

        if not self.samples:
            print("ERROR: No samples collected!")
            return False

        return self.analyze_results()

    def analyze_results(self):
        """Analyze collected samples and print results."""
        # Stack results
        angular_velocities = np.array([s["angular_velocity"] for s in self.samples])
        linear_velocities = np.array([s["linear_velocity"] for s in self.samples])
        angular_accelerations = np.array([s["angular_acceleration"] for s in self.samples])
        linear_accelerations = np.array([s["linear_acceleration"] for s in self.samples])
        joint_velocities = np.array([s["joint_velocity"] for s in self.samples])
        joint_accelerations = np.array([s["joint_acceleration"] for s in self.samples])

        print("\n" + "=" * 60)
        print("RESULTS")
        print("=" * 60)

        # Thresholds for "near zero"
        vel_threshold = 1e-3  # m/s or rad/s
        acc_threshold = 1e-2  # m/s² or rad/s²

        results = {}

        # Angular velocity
        av_mean = np.mean(angular_velocities, axis=0)
        av_std = np.std(angular_velocities, axis=0)
        av_max = np.max(np.abs(angular_velocities))
        results["angular_velocity"] = av_max < vel_threshold
        print(f"\nAngular Velocity ω [rad/s]:")
        print(f"  Mean: [{av_mean[0]:+.6f}, {av_mean[1]:+.6f}, {av_mean[2]:+.6f}]")
        print(f"  Std:  [{av_std[0]:.6f}, {av_std[1]:.6f}, {av_std[2]:.6f}]")
        print(f"  Max |ω|: {av_max:.6f}")
        print(f"  PASS: {results['angular_velocity']} (threshold: {vel_threshold})")

        # Linear velocity
        lv_mean = np.mean(linear_velocities, axis=0)
        lv_std = np.std(linear_velocities, axis=0)
        lv_max = np.max(np.abs(linear_velocities))
        results["linear_velocity"] = lv_max < vel_threshold
        print(f"\nLinear Velocity ṗ [m/s]:")
        print(f"  Mean: [{lv_mean[0]:+.6f}, {lv_mean[1]:+.6f}, {lv_mean[2]:+.6f}]")
        print(f"  Std:  [{lv_std[0]:.6f}, {lv_std[1]:.6f}, {lv_std[2]:.6f}]")
        print(f"  Max |ṗ|: {lv_max:.6f}")
        print(f"  PASS: {results['linear_velocity']} (threshold: {vel_threshold})")

        # Angular acceleration
        aa_mean = np.mean(angular_accelerations, axis=0)
        aa_std = np.std(angular_accelerations, axis=0)
        aa_max = np.max(np.abs(angular_accelerations))
        results["angular_acceleration"] = aa_max < acc_threshold
        print(f"\nAngular Acceleration ω̇ [rad/s²]:")
        print(f"  Mean: [{aa_mean[0]:+.6f}, {aa_mean[1]:+.6f}, {aa_mean[2]:+.6f}]")
        print(f"  Std:  [{aa_std[0]:.6f}, {aa_std[1]:.6f}, {aa_std[2]:.6f}]")
        print(f"  Max |ω̇|: {aa_max:.6f}")
        print(f"  PASS: {results['angular_acceleration']} (threshold: {acc_threshold})")

        # Linear acceleration
        la_mean = np.mean(linear_accelerations, axis=0)
        la_std = np.std(linear_accelerations, axis=0)
        la_max = np.max(np.abs(linear_accelerations))
        results["linear_acceleration"] = la_max < acc_threshold
        print(f"\nLinear Acceleration p̈ [m/s²]:")
        print(f"  Mean: [{la_mean[0]:+.6f}, {la_mean[1]:+.6f}, {la_mean[2]:+.6f}]")
        print(f"  Std:  [{la_std[0]:.6f}, {la_std[1]:.6f}, {la_std[2]:.6f}]")
        print(f"  Max |p̈|: {la_max:.6f}")
        print(f"  PASS: {results['linear_acceleration']} (threshold: {acc_threshold})")

        # Joint velocities (raw from /joint_states)
        jv_max = np.max(np.abs(joint_velocities))
        print(f"\nJoint Velocities (raw from /joint_states) [rad/s]:")
        print(f"  Max |v|: {jv_max:.6f}")

        # Joint accelerations (from numerical differentiation)
        ja_max = np.max(np.abs(joint_accelerations))
        print(f"\nJoint Accelerations (numerical diff) [rad/s²]:")
        print(f"  Max |a|: {ja_max:.6f}")

        # Overall result
        print("\n" + "=" * 60)
        all_pass = all(results.values())
        if all_pass:
            print("OVERALL: ✓ ALL TESTS PASSED")
        else:
            print("OVERALL: ✗ SOME TESTS FAILED")
            for name, passed in results.items():
                if not passed:
                    print(f"  - {name}: FAILED")
        print("=" * 60)

        return all_pass


def main():
    try:
        test = StationaryTest(num_samples=100, warmup_samples=20)
        success = test.run()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        print("Test interrupted.")
        sys.exit(1)


if __name__ == "__main__":
    main()
