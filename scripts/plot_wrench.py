#!/usr/bin/env python3
import json
import sys
import numpy as np
import matplotlib.pyplot as plt
import os


def plot_wrench(json_path):
    print(f"Loading {json_path}...")
    with open(json_path, "r") as f:
        data = json.load(f)

    frames = data["frames"]
    times = [f["time"] for f in frames]
    wrench = np.array([f["wrench"] for f in frames])  # Nx6

    # Forces
    fx = wrench[:, 0]
    fy = wrench[:, 1]
    fz = wrench[:, 2]

    # Torques
    tx = wrench[:, 3]
    ty = wrench[:, 4]
    tz = wrench[:, 5]

    # Plot Forces
    plt.figure(figsize=(10, 6))
    plt.plot(times, fx, label="Fx")
    plt.plot(times, fy, label="Fy")
    plt.plot(times, fz, label="Fz")
    plt.title("Wrench: Forces")
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.legend()
    plt.grid(True)

    force_plot_path = os.path.dirname(json_path) + "/force_plot.png"
    plt.savefig(force_plot_path)
    print(f"Saved force plot to {force_plot_path}")

    # Plot Torques
    plt.figure(figsize=(10, 6))
    plt.plot(times, tx, label="Tx")
    plt.plot(times, ty, label="Ty")
    plt.plot(times, tz, label="Tz")
    plt.title("Wrench: Torques")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.legend()
    plt.grid(True)

    torque_plot_path = os.path.dirname(json_path) + "/torque_plot.png"
    plt.savefig(torque_plot_path)
    print(f"Saved torque plot to {torque_plot_path}")

    # Print statistics
    print("-" * 30)
    print("Wrench Statistics:")
    print(
        f"Force  Max (Abs): x={np.max(np.abs(fx)):.2f}, y={np.max(np.abs(fy)):.2f}, z={np.max(np.abs(fz)):.2f}"
    )
    print(
        f"Torque Max (Abs): x={np.max(np.abs(tx)):.2f}, y={np.max(np.abs(ty)):.2f}, z={np.max(np.abs(tz)):.2f}"
    )
    print("-" * 30)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./plot_wrench.py <json_path>")
        sys.exit(1)

    plot_wrench(sys.argv[1])
