"""Shared utilities for inertial parameter identification scripts."""

import os

import matplotlib.pyplot as plt
import numpy as np

PARAM_NAMES = ["m", "mcx", "mcy", "mcz", "Ixx", "Iyy", "Izz", "Ixy", "Iyz", "Izx"]


def plot_kinematics_wrench(frames: list[dict], save_dir: str) -> None:
    """Save velocity, acceleration, and wrench plots from recorded identification frames.

    Args:
        frames: List of frame dicts with keys "time", "wrench",
                and "tool0_kinematics" containing "lv", "av", "la", "aa".
        save_dir: Directory to save PNG files.
    """
    times = [f["time"] for f in frames]
    lv = np.array([f["tool0_kinematics"]["lv"] for f in frames])
    av = np.array([f["tool0_kinematics"]["av"] for f in frames])
    la = np.array([f["tool0_kinematics"]["la"] for f in frames])
    aa = np.array([f["tool0_kinematics"]["aa"] for f in frames])
    wrench = np.array([f["wrench"] for f in frames])

    xyz = ["x", "y", "z"]

    # Velocity
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for i, label in enumerate(xyz):
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

    # Acceleration
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for i, label in enumerate(xyz):
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

    # Wrench
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
