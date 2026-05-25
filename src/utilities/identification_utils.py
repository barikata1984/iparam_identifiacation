"""Shared utilities for inertial parameter identification scripts."""

import os

import matplotlib.pyplot as plt
import numpy as np

PARAM_NAMES = ["m", "hx", "hy", "hz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]


def plot_velocity_acceleration_comparison(
    times: np.ndarray,
    vel_num: np.ndarray,
    vel_jac: np.ndarray,
    acc_num: np.ndarray,
    acc_jac: np.ndarray,
    ref_times: np.ndarray,
    ref_vel: np.ndarray,
    ref_acc: np.ndarray,
    save_dir: str,
) -> None:
    """Compare tip velocity and acceleration estimates against the commanded target.

    Layout: 6 rows x 2 columns. Rows 1-3 are velocity (vx, vy, vz); rows 4-6 are
    acceleration (ax, ay, az). The left column is the numerical estimate (velocity =
    d/dt of position, acceleration = d²/dt² of position); the right column is the
    Jacobian-based estimate (velocity = J·q̇, acceleration = d/dt of that velocity).
    Each row is compared against the commanded target and shares a y-axis between the
    two columns so the noise difference is directly comparable.

    Args:
        times: (N,) measured timestamps [s].
        vel_num: (N, 3) numerically differentiated tip velocity [m/s].
        vel_jac: (N, 3) Jacobian-based tip velocity [m/s].
        acc_num: (N, 3) tip acceleration from the 2nd derivative of position [m/s²].
        acc_jac: (N, 3) tip acceleration from the 1st derivative of vel_jac [m/s²].
        ref_times: (M,) commanded timestamps [s], wall-clock aligned with `times`.
        ref_vel: (M, 3) commanded (target) tip velocity [m/s].
        ref_acc: (M, 3) commanded (target) tip acceleration [m/s²].
        save_dir: Directory to save the PNG file.
    """
    t = np.asarray(times) - times[0]
    xyz = ["x", "y", "z"]
    colors = ["C0", "C1", "C2"]
    has_ref = ref_times is not None and ref_vel is not None
    rt = np.asarray(ref_times) - times[0] if has_ref else None

    # 6 rows: (vx, vy, vz, ax, ay, az). Per row: (measured, target, ylabel, ref).
    rows = [
        (vel_num, vel_jac, ref_vel, f"v{a} [m/s]", i) for i, a in enumerate(xyz)
    ] + [
        (acc_num, acc_jac, ref_acc, f"a{a} [m/s²]", i) for i, a in enumerate(xyz)
    ]

    fig, axes = plt.subplots(6, 2, figsize=(14, 20), sharex=True)
    col_title = [
        "Numerical (d/dt, d²/dt²) vs target",
        "Jacobian: J·q̇ & its d/dt vs target",
    ]
    axes[0, 0].set_title(col_title[0])
    axes[0, 1].set_title(col_title[1])

    for r, (meas_l, meas_r, ref, ylabel, comp) in enumerate(rows):
        col_meas = [meas_l, meas_r]
        # Independent y-axes per subplot so the clean Jacobian estimate stays readable
        # even when the numerical estimate is far noisier; magnitudes are on the ticks.
        for col in range(2):
            ax = axes[r, col]
            ax.plot(t, col_meas[col][:, comp], color=colors[comp], label="measured")
            if has_ref and ref is not None:
                ax.plot(rt, ref[:, comp], color="k", ls="--", lw=1.0, label="target")
            ax.set_ylabel(ylabel)
            ax.grid(True)
            ax.legend(loc="upper right", fontsize=8)
    axes[5, 0].set_xlabel("Time [s]")
    axes[5, 1].set_xlabel("Time [s]")

    fig.suptitle(
        "Gripper tip velocity & acceleration (base frame): numerical vs Jacobian-based",
        y=0.995,
    )
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    plt.savefig(
        os.path.join(save_dir, "velocity_acceleration_comparison.png"),
        dpi=150,
        bbox_inches="tight",
    )
    plt.close()


def plot_wrist3_torque(
    times: np.ndarray,
    wrist3_pos: np.ndarray,
    tz: np.ndarray,
    save_dir: str,
) -> None:
    """Overlay wrist_3 joint position and measured torque Tz on a twin-axis plot.

    Used to check the Coulomb-friction hypothesis: Tz is expected to switch level at
    the wrist_3 position turning points (where the joint velocity reverses sign).

    Args:
        times: (N,) timestamps [s].
        wrist3_pos: (N,) wrist_3 joint position [rad].
        tz: (N,) measured torque about the tool z-axis [Nm].
        save_dir: Directory to save the PNG file.
    """
    t = np.asarray(times) - times[0]
    c_pos, c_tz = "C0", "C3"

    fig, ax1 = plt.subplots(figsize=(13, 6))
    ln1 = ax1.plot(t, wrist3_pos, color=c_pos, lw=1.5, label="wrist_3 position")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("wrist_3 position [rad]", color=c_pos)
    ax1.tick_params(axis="y", labelcolor=c_pos)
    ax1.grid(True, alpha=0.4)

    ax2 = ax1.twinx()
    ln2 = ax2.plot(t, tz, color=c_tz, lw=0.8, label="Tz")
    ax2.set_ylabel("Torque z [Nm]", color=c_tz)
    ax2.tick_params(axis="y", labelcolor=c_tz)

    lns = ln1 + ln2
    ax1.legend(lns, [l.get_label() for l in lns], loc="upper right", fontsize=8)
    ax1.set_title("wrist_3 position vs measured torque Tz")

    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, "wrist3_position_torque_z.png"), dpi=150, bbox_inches="tight")
    plt.close()


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
    """Plot F/T and gripper-tip (tool0) kinematics in a 5x3 (quantity x axis) grid.

    Rows (top to bottom): Force, Torque, Position, Velocity, Acceleration.
    Columns (left to right): x, y, z. Measured signals are solid; commanded/reference
    arrays (when provided) are overlaid as a black dashed line. Force and Torque have
    no commanded counterpart.

    Args:
        times: (N,) timestamps [s].
        wrench: (N, 6) F/T measurements [Fx, Fy, Fz, Tx, Ty, Tz].
        tip_pos: (N, 3) tool0 position w.r.t. base frame [m].
        tip_vel: (N, 3) tip velocity [m/s].
        tip_acc: (N, 3) tip acceleration [m/s^2].
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

    # Each row: (measured (N,3), commanded (M,3) or None, row label).
    rows = [
        (wrench[:, 0:3], None, "Force [N]"),
        (wrench[:, 3:6], None, "Torque [Nm]"),
        (tip_pos, ref_pos, "Position [m]"),
        (tip_vel, ref_vel, "Velocity [m/s]"),
        (tip_acc, ref_acc, "Acceleration [m/s²]"),
    ]

    fig, axes = plt.subplots(5, 3, figsize=(16, 16), sharex=True)

    for r, (meas, ref, row_label) in enumerate(rows):
        for c in range(3):
            ax = axes[r, c]
            ax.plot(t, meas[:, c], color=colors[c], label="measured")
            if has_ref and ref is not None:
                ax.plot(rt, ref[:, c], color="k", ls="--", lw=1.0, label="target")
            ax.grid(True)
            if r == 0:
                ax.set_title(xyz[c])
            if c == 0:
                ax.set_ylabel(row_label)
            if ref is not None:
                ax.legend(loc="upper right", fontsize=8)
    for c in range(3):
        axes[4, c].set_xlabel("Time [s]")

    fig.suptitle("Replay recording (base frame): F/T and gripper-tip kinematics", y=0.995)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
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
