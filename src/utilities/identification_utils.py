"""Shared utilities for inertial parameter identification scripts."""

import os

import matplotlib.pyplot as plt
import numpy as np

PARAM_NAMES = ["m", "hx", "hy", "hz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]


def plot_replay_recording(
    times: np.ndarray,
    wrench: np.ndarray,
    tip_pos: np.ndarray,
    tip_vel: np.ndarray,
    tip_acc: np.ndarray,
    save_dir: str,
    ref_times: np.ndarray = None,
    ref_pos: np.ndarray = None,
    ref_vel: np.ndarray = None,
    ref_acc: np.ndarray = None,
) -> None:
    """Plot F/T and gripper-tip (tool0) position with its 1st/2nd numerical derivatives.

    Measured signals are drawn solid; if commanded/reference arrays are provided they
    are overlaid dashed (same color per axis). F/T has no commanded counterpart.

    Args:
        times: (N,) timestamps [s].
        wrench: (N, 6) F/T measurements [Fx, Fy, Fz, Tx, Ty, Tz].
        tip_pos: (N, 3) tool0 position w.r.t. base frame [m].
        tip_vel: (N, 3) 1st numerical time derivative of tip_pos [m/s].
        tip_acc: (N, 3) 2nd numerical time derivative of tip_pos [m/s^2].
        save_dir: Directory to save the PNG file.
        ref_times: (M,) commanded timestamps [s], wall-clock aligned with `times`.
        ref_pos: (M, 3) commanded tip position [m].
        ref_vel: (M, 3) commanded tip velocity [m/s].
        ref_acc: (M, 3) commanded tip acceleration [m/s^2].
    """
    t = np.asarray(times) - times[0]
    xyz = ["x", "y", "z"]
    colors = ["C0", "C1", "C2"]
    has_ref = ref_times is not None and ref_pos is not None
    rt = np.asarray(ref_times) - times[0] if has_ref else None

    fig, axes = plt.subplots(5, 1, figsize=(11, 16), sharex=True)

    for i, label in enumerate(["Fx", "Fy", "Fz"]):
        axes[0].plot(t, wrench[:, i], color=colors[i], label=label)
    axes[0].set_title("Force (a)")
    axes[0].set_ylabel("[N]")

    for i, label in enumerate(["Tx", "Ty", "Tz"]):
        axes[1].plot(t, wrench[:, 3 + i], color=colors[i], label=label)
    axes[1].set_title("Torque (a)")
    axes[1].set_ylabel("[Nm]")

    def _meas_ref(ax, meas, ref):
        for i, label in enumerate(xyz):
            ax.plot(t, meas[:, i], color=colors[i], label=f"{label} meas")
        if has_ref and ref is not None:
            for i, label in enumerate(xyz):
                ax.plot(rt, ref[:, i], color=colors[i], ls="--", label=f"{label} cmd")

    _meas_ref(axes[2], tip_pos, ref_pos)
    axes[2].set_title("Gripper tip position w.r.t. base (b, tool0)")
    axes[2].set_ylabel("[m]")

    _meas_ref(axes[3], tip_vel, ref_vel)
    axes[3].set_title("Tip position 1st numerical derivative (b.1)")
    axes[3].set_ylabel("[m/s]")

    _meas_ref(axes[4], tip_acc, ref_acc)
    axes[4].set_title("Tip position 2nd numerical derivative (b.2)")
    axes[4].set_ylabel("[m/s^2]")
    axes[4].set_xlabel("Time [s]")

    for ax in axes:
        ax.legend(loc="upper right", ncol=2)
        ax.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "replay_recording.png"), dpi=150, bbox_inches="tight")
    plt.close()


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
