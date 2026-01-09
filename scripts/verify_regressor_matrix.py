#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import sys


def callback(msg):
    # Data is a flat list of 60 elements
    data = np.array(msg.data)

    if len(data) != 60:
        rospy.logerr(f"Received data of length {len(data)}, expected 60")
        return

    # Reshape to 6x10
    matrix = data.reshape(6, 10)

    # Clear screen and print
    print("\033[H\033[J")  # ANSI escape code to clear screen
    print("Received Regressor Matrix (6x10):")
    print("-" * 80)

    # Define headers for columns (Parameters)
    headers = ["m", "hx", "hy", "hz", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]

    # Print headers
    header_str = "      "
    for h in headers:
        header_str += f"{h:>8} "
    print(header_str)
    print("-" * 80)

    # Row labels (Wrench components)
    row_labels = ["Fx", "Fy", "Fz", "Nx", "Ny", "Nz"]

    for i in range(6):
        row_str = f"{row_labels[i]:<4} |"
        for j in range(10):
            val = matrix[i, j]
            # Highlight non-zero values significantly
            if abs(val) > 0.01:
                row_str += f"\033[1m{val:>8.3f}\033[0m "
            else:
                row_str += f"{val:>8.3f} "
        print(row_str)

    print("-" * 80)
    print("Quick Check (Stationary, approx):")
    print("Col 'm' (0) should match proper acceleration [ax, ay, az, 0, 0, 0]^T")
    print(
        "Cols 'h' (1-3) rows 4-6 should contain acceleration terms for cross products."
    )


def listener():
    rospy.init_node("verify_regressor", anonymous=True)
    rospy.Subscriber("/wrist_end_kinematics/regressor", Float64MultiArray, callback)
    print("Listening to /wrist_end_kinematics/regressor...")
    rospy.spin()


if __name__ == "__main__":
    listener()
