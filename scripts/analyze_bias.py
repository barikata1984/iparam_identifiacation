#!/usr/bin/env python3
import json
import numpy as np
import sys


def analyze(json_path):
    print(f"Loading {json_path}...")
    with open(json_path, "r") as f:
        data = json.load(f)

    frames = data["frames"]
    print(f"Loaded {len(frames)} frames.")

    # Collect accelerations (from regressor) and forces (from wrench)
    # Regressor col 0 is [ax, ay, az, 0, 0, 0] for each block

    forces_x = []
    forces_y = []
    forces_z = []

    acc_x = []
    acc_y = []
    acc_z = []

    for frame in frames:
        W = frame["wrench"]  # [fx, fy, fz, nx, ny, nz]
        S = frame["regressor"]  # 6x10 matrix (list of lists)

        # Force X corresponds to Row 0
        fx = W[0]
        ax = S[0][0]  # First element of first row is mass coeff (acceleration x)

        # Force Y corresponds to Row 1
        fy = W[1]
        ay = S[1][0]

        # Force Z corresponds to Row 2
        fz = W[2]
        az = S[2][0]

        forces_x.append(fx)
        forces_y.append(fy)
        forces_z.append(fz)

        acc_x.append(ax)
        acc_y.append(ay)
        acc_z.append(az)

    forces_x = np.array(forces_x)
    acc_x = np.array(acc_x)

    # Simple lin fitting for each axis
    # F = m * a + c
    m_x, c_x = np.polyfit(acc_x, forces_x, 1)
    m_y, c_y = np.polyfit(acc_y, forces_y, 1)
    m_z, c_z = np.polyfit(acc_z, forces_z, 1)

    print(f"Estimated Mass per axis (Slope of F vs a):")
    print(f"X-axis: {m_x:.4f} (Offset: {c_x:.4f})")
    print(f"Y-axis: {m_y:.4f} (Offset: {c_y:.4f})")
    print(f"Z-axis: {m_z:.4f} (Offset: {c_z:.4f})")

    # Check overall
    all_f = np.concatenate([forces_x, forces_y, forces_z])
    all_a = np.concatenate([acc_x, acc_y, acc_z])
    m_all, c_all = np.polyfit(all_a, all_f, 1)
    print(f"Overall Mass Estimate (Slope): {m_all:.4f}")

    if m_all < 0:
        print("\n[CONCLUSION] The slope is NEGATIVE.")
        print("Possible causes:")
        print(
            "1. Force sensor sign convention is inverted (F_sensor = -F_robot_on_env)."
        )
        print(
            "2. Acceleration sign is inverted (Gravity compensation wrong direction)."
        )
    else:
        print("\n[CONCLUSION] The slope is POSITIVE.")

    print("-" * 30)
    print("Excitation Analysis:")

    # Check Angular Acceleration
    ang_acc_magnitudes = []
    for frame in frames:
        aa = frame["tool0_kinematics"]["aa"]
        mag = np.linalg.norm(aa)
        ang_acc_magnitudes.append(mag)

    ang_acc_magnitudes = np.array(ang_acc_magnitudes)
    print(
        f"Angular Accel: Max={np.max(ang_acc_magnitudes):.4f}, Mean={np.mean(ang_acc_magnitudes):.4f}, Std={np.std(ang_acc_magnitudes):.4f}"
    )

    if np.max(ang_acc_magnitudes) < 1e-2:
        print(
            "-> WARNING: Angular acceleration is effectively ZERO. Inertia parameters cannot be identified."
        )

    # Check Regressor Rank/Condition
    S_list = [np.array(f["regressor"]) for f in frames]
    S_total = np.vstack(S_list)
    print(f"Regressor Shape: {S_total.shape}")

    # Singular Values and Condition Number
    U, s, Vh = np.linalg.svd(S_total, full_matrices=False)
    cond_num = s[0] / s[-1] if s[-1] > 1e-10 else float("inf")

    print("-" * 30)
    print("Matrix Condition Analysis:")
    print(f"Singular Values: Max={s[0]:.4f}, Min={s[-1]:.4f}")
    print(f"Condition Number: {cond_num:.4e}")
    if cond_num > 1000:
        print(
            "-> WARNING: Matrix is Ill-Conditioned! Small noise will cause large errors in estimates."
        )
        print("   TLS is particularly sensitive to this if data is unscaled.")

    rank = np.linalg.matrix_rank(S_total)
    print(f"Numerical Rank: {rank}")

    # Check column norms
    col_norms = np.linalg.norm(S_total, axis=0)
    print("Regressor Column Norms (Excitation per parameter):")
    param_names = ["m", "hx", "hy", "hz", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz"]
    for name, norm in zip(param_names, col_norms):
        print(f"  {name}: {norm:.4f}")

    if np.any(col_norms < 1e-4):
        print(
            "-> WARNING: Some columns have near-zero norm. Parameters are unobservable.",
            flush=True,
        )


if __name__ == "__main__":
    print("Starting analysis script...", flush=True)
    if len(sys.argv) < 2:
        print("Usage: ./analyze_bias.py <json_path>", flush=True)
        sys.exit(1)

    path = sys.argv[1]
    print(f"Target path: {path}", flush=True)
    analyze(path)
