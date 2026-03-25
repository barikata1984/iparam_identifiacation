# UR5e Analytical Kinematics Implementation Notes

> **注意 (2026-03-24)**: このドキュメントが参照する `src/utilities/ur5e_analytical_kinematics.py` は
> Pinocchio ベースの `Tool0KinematicsCalculator` (`src/utilities/tool0_kinematics.py`) への移行に伴い
> 削除済み。以下は歴史的記録として残す。

This document details the implementation of analytical forward kinematics (position, velocity, and acceleration) for the UR5e robot, used for inertial parameter identification.

## Overview

To calculate the regressor matrix for inertial parameter identification, precise knowledge of the end-effector's kinematic state (specifically linear/angular velocity and acceleration) is required. Existing libraries like `ur_pykdl` often lack direct support for calculating acceleration. Therefore, we implemented two analytical approaches to compute these values without relying on numerical differentiation or external physics engines.

The implementation was located in: `src/utilities/ur5e_analytical_kinematics.py` (deleted, superseded by Pinocchio)

## 1. Modified DH Parameters (Standard Method)

This implementation follows the **Modified Denavit-Hartenberg (DH)** convention and the **Recursive Newton-Euler (RNE)** formulation as described by John J. Craig.

### Reference

* **Source**: *Introduction to Robotics: Mechanics and Control*, John J. Craig (Chapter 6: Manipulator Dynamics).
* **Class**: `UR5eAnalyticalKinematics`

### Algorithm: Forward Newton-Euler (First Pass)

The algorithm propagates velocities and accelerations from the base to the end-effector link by link.

**Notation**:

* $\{i\}$: Frame attached to link $i$.
* ${}^{i}P_{i+1}$: Position vector from origin of $\{i\}$ to $\{i+1\}$.
* $\omega_i, \dot{\omega}_i$: Angular velocity and acceleration of link $i$.
* $v_i, \dot{v}_i$: Linear velocity and acceleration of the origin of frame $\{i\}$.

**Propagation Equations**:

1. **Angular Velocity**:
    $$ {}^{i}\omega_{i} = {}^{i}\omega_{i-1} + z_i \dot{\theta}_i $$
2. **Angular Acceleration**:
    $$ {}^{i}\dot{\omega}_{i} = {}^{i}\dot{\omega}_{i-1} + z_i \ddot{\theta}_i + {}^{i}\omega_{i-1} \times (z_i \dot{\theta}_i) $$
3. **Linear Acceleration**:
    $$ {}^{i}\dot{v}_{i} = {}^{i}\dot{v}_{i-1} + {}^{i}\dot{\omega}_{i-1} \times {}^{i}P_{i-1,i} + {}^{i}\omega_{i-1} \times ({}^{i}\omega_{i-1} \times {}^{i}P_{i-1,i}) $$

*(Note: Our implementation performs these vector calculations in the **Base Frame {0}** for simplicity, using rotation matrices to transform vectors as needed.)*

---

## 2. Twist-Wrench Formulation (Modern Robotics Method)

This implementation uses the **Product of Exponentials (PoE)** formula and **Spatial Vector Algebra** (Lie Theory), consistent with the *Modern Robotics* text.

### Reference

* **Source**: *Modern Robotics: Mechanics, Planning, and Control*, Kevin M. Lynch and Frank C. Park (Chapter 8: Dynamics of Open Chains).
* **Class**: `UR5eTwistKinematics` (Inherits from `UR5eAnalyticalKinematics`)

### Algorithm: Forward Newton-Euler (Twist-based)

This method propagates Spatial Velocity (Twist) and Spatial Acceleration.

**Notation**:

* $\mathcal{V}_i$: Spatial velocity (Twist) of link $i$, expressed in frame $\{i\}$. $\mathcal{V}_i = [\omega_i; v_i]$.
* $\mathcal{A}_i$: Screw axis of joint $i$, expressed in frame $\{i\}$.
* $Ad_T$: Adjoint map of SE(3).
* $ad_{\mathcal{V}}$: Lie bracket operator.

**Propagation Equations (Algorithm 8.1)**:

1. **Pose Propagation**:
    $$ T_{0,i} = T_{0,i-1} M_{i, i-1} e^{[\mathcal{A}_i]\theta_i} $$
    *(In implementation, we use DH transforms to derive $T_{i,i-1}$)*
2. **Twist Propagation**:
3. **Twist Propagation**:
    $$ \mathcal{V}_i = Ad_{T_{i,i-1}^{-1}}(\mathcal{V}_{i-1}) + \mathcal{A}_i \dot{\theta}_i $$
4. **Acceleration Propagation**:
    $$ \dot{\mathcal{V}}_i = Ad_{T_{i,i-1}^{-1}}(\dot{\mathcal{V}}_{i-1}) + [\mathcal{V}_i, \mathcal{A}_i \dot{\theta}_i] + \mathcal{A}_i \ddot{\theta}_i $$
    where $[\mathcal{V}_a, \mathcal{V}_b] = \text{ad}_{\mathcal{V}_a}(\mathcal{V}_b)$.

### UR5e Specifics

* We derive the relative transforms $M_{i,i-1}$ and joint axes $\mathcal{A}_i$ directly from the UR5e Modified DH parameters to ensure consistency.
* For a revolute joint $i$ in Modified DH, the rotation is always about the $Z_i$ axis. Therefore, the screw axis in the body frame is always:
    $$ \mathcal{A}_i = [0, 0, 1, 0, 0, 0]^T $$

---

## 3. Closed-Form Matrix Formulation (Modern Robotics Method)

This implementation corresponds to the **Closed-Form Forward Dynamics** described in conventional robotics literature (e.g., Lynch & Park Chapter 8.1.2). Instead of recursive loops, it constructs a lower-triangular block matrix $L$ that propagates twists across all links simultaneously.

### Reference

* **Source**: *Modern Robotics: Mechanics, Planning, and Control*, Lynch & Park (Chapter 8.1.2).
* **Class**: `UR5eMatrixKinematics` (Inherits from `UR5eTwistKinematics`)

### Algorithm

The velocities and accelerations of all links are computed in a single matrix operation.
Define stacked vectors $\mathcal{V} = [\mathcal{V}_1^T, \dots, \mathcal{V}_n^T]^T \in \mathbb{R}^{6n}$.

1. **Velocity**:
    $$ \mathcal{V} = L (\mathcal{A} \dot{\theta}) $$
    where $L$ is a lower-triangular block matrix with $L_{i,j} = Ad_{T_{i,j}}$ for $i \ge j$.
2. **Acceleration**:
    $$ \dot{\mathcal{V}} = L (\mathcal{A} \ddot{\theta} + \text{ad}_{\mathcal{V}}(\mathcal{A}\dot{\theta})) + L \cdot \dot{\mathcal{V}}_{\text{base}} $$
    where $\text{ad}_{\mathcal{V}}$ term represents the Coriolis/Centrifugal acceleration.

This approach is mathematically elegant and avoids Python-level loops for propagation, relying instead on optimized `numpy` matrix multiplication.

## Usage

You can select the implementation method using the `build_ur5e_kinematics_func` helper:

```python
from utilities.ur5e_analytical_kinematics import build_ur5e_kinematics_func

# 1. Standard Method (Craig / Modified DH)
kinematics_func_dh = build_ur5e_kinematics_func(method="standard")

# 2. Modern Robotics Method (Lynch & Park / Twist)
kinematics_func_twist = build_ur5e_kinematics_func(method="twist")

# 3. Closed-Form Matrix Method (Lynch & Park / Matrix)
kinematics_func_matrix = build_ur5e_kinematics_func(method="matrix")

# The returned function has the signature:
# regressor_matrix = kinematics_func(q, dq, ddq)
# Returns: (6, 10) np.ndarray
```
